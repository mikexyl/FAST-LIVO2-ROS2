#pragma once
// Native primitive registration. No surface expansion or point covariances.
#include "registration_common.hpp"
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <openssl/sha.h>
#include <iomanip>
#include <sstream>

namespace s3e_registration {
inline std::string ellipsoid_payload_sha256(const std::string& path) {
  std::ifstream file(path,std::ios::binary);
  if(!file)throw std::invalid_argument("missing primitive payload");
  SHA256_CTX context;SHA256_Init(&context);char buffer[65536];
  while(file.read(buffer,sizeof(buffer))||file.gcount())SHA256_Update(&context,buffer,file.gcount());
  unsigned char hash[SHA256_DIGEST_LENGTH];SHA256_Final(hash,&context);
  std::ostringstream result;result<<std::hex<<std::setfill('0');
  for(unsigned char byte:hash)result<<std::setw(2)<<static_cast<unsigned>(byte);
  return result.str();
}
struct Ellipsoids {
  std::vector<Eigen::Vector4d> centers;
  std::vector<Eigen::Vector3d> axes;
  std::vector<Eigen::Matrix3d> bases, projectors;
  std::shared_ptr<gtsam_points::KdTree> tree;
};
using EllipsoidsPtr = std::shared_ptr<const Ellipsoids>;
inline Eigen::Matrix3d cross_matrix(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s; s << 0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0; return s;
}
inline EllipsoidsPtr load_ellipsoids(const Json& spec) {
  const auto n=spec.at("points").get<size_t>();
  if(n<3 || n>50000)throw std::invalid_argument("invalid ellipsoid count");
  std::ifstream f(spec.at("path").get<std::string>(),std::ios::binary|std::ios::ate);
  if(!f || f.tellg()!=static_cast<std::streamoff>(n*15*sizeof(double)))
    throw std::invalid_argument("ellipsoid binary size mismatch");
  f.seekg(0); auto e=std::make_shared<Ellipsoids>();
  e->centers.reserve(n);e->axes.reserve(n);e->bases.reserve(n);e->projectors.reserve(n);
  for(size_t k=0;k<n;++k) {
    Eigen::Matrix<double,15,1> row;f.read(reinterpret_cast<char*>(row.data()),15*sizeof(double));
    Eigen::Matrix3d b=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(row.data()+6);
    const Eigen::Vector3d axes=row.segment<3>(3),inv=axes.cwiseInverse();
    if(!f || !row.allFinite() || (axes.array()<=0).any() ||
       (b.transpose()*b-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff()>1e-3 ||
       inv[1]-inv[0]<-1e-5*inv.maxCoeff() || inv[2]-inv[1]<-1e-5*inv.maxCoeff())
      throw std::invalid_argument("invalid native ellipsoid axes/basis");
    Eigen::Vector3d w(std::max(0.,inv[2]-inv[1]),std::max(0.,inv[1]-inv[0]),inv[0]);w/=w.sum();
    e->centers.emplace_back(row[0],row[1],row[2],1);
    e->axes.push_back(axes);e->bases.push_back(b);
    e->projectors.push_back(w[0]*b.col(2)*b.col(2).transpose()+
      w[1]*(Eigen::Matrix3d::Identity()-b.col(0)*b.col(0).transpose())+w[2]*Eigen::Matrix3d::Identity());
  }
  e->tree=std::make_shared<gtsam_points::KdTree>(e->centers.data(),n);
  return e;
}
inline double huber_cost(double norm,double delta) {
  return norm<=delta?.5*norm*norm:delta*(norm-.5*delta);
}
struct EllipsoidTerms {
  Eigen::Matrix<double,12,12> H=Eigen::Matrix<double,12,12>::Zero();
  Eigen::Matrix<double,12,1> gradient=Eigen::Matrix<double,12,1>::Zero();
  double cost=0;size_t matched=0;
};
class EllipsoidFactor : public gtsam::NonlinearFactor {
 public:
  EllipsoidFactor(gtsam::Key i,gtsam::Key j,EllipsoidsPtr target,EllipsoidsPtr source,
                  double radius,double huber,size_t min_support=6)
    :gtsam::NonlinearFactor(gtsam::KeyVector{i,j}),target_(std::move(target)),source_(std::move(source)),
     radius_(radius),huber_(huber),min_support_(min_support) {
    if(i>=j || !target_ || !source_ || source_->centers.empty() ||
       !std::isfinite(radius) || !std::isfinite(huber) || radius<=0 || huber<=0)
      throw std::invalid_argument("invalid canonical ellipsoid factor");
  }
  size_t dim() const override {return 6;}
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<EllipsoidFactor>(*this);
  }
  double scale=1;
  mutable size_t linearizations=0,unsupported_linearizations=0,last_correspondences=0;
  EllipsoidTerms terms(const gtsam::Values& values,bool jacobians) const {
    const auto T=values.at<gtsam::Pose3>(keys()[0]).between(values.at<gtsam::Pose3>(keys()[1]));
    const Eigen::Matrix3d R=T.rotation().matrix(); EllipsoidTerms result;
    for(const auto& center:source_->centers) {
      const Eigen::Vector3d p=center.head<3>(),q=T.transformFrom(p);
      const Eigen::Vector4d q4(q[0],q[1],q[2],1);size_t idx=0;double distance=0;
      if(!target_->tree->knn_search(q4.data(),1,&idx,&distance) || distance>=radius_*radius_) {
        result.cost+=huber_cost(radius_,huber_);continue;
      }
      ++result.matched;
      const auto& P=target_->projectors[idx];
      const Eigen::Vector3d residual=P*(q-target_->centers[idx].head<3>());
      const double norm=residual.norm();result.cost+=huber_cost(norm,huber_);
      if(jacobians) {
        Eigen::Matrix<double,3,12> J;
        J.block<3,3>(0,0)=P*cross_matrix(q);J.block<3,3>(0,3)=-P;
        J.block<3,3>(0,6)=-P*R*cross_matrix(p);J.block<3,3>(0,9)=P*R;
        const double weight=std::min(1.,huber_/std::max(norm,1e-12));
        result.H.noalias()+=weight*J.transpose()*J;
        result.gradient.noalias()+=weight*J.transpose()*residual;
      }
    }
    const double count=source_->centers.size();
    result.H/=count;result.gradient/=count;result.cost/=count;
    return result;
  }
  double error(const gtsam::Values& v) const override {return scale*terms(v,false).cost;}
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& v) const override {
    const auto a=terms(v,true);++linearizations;last_correspondences=a.matched;
    if(a.matched<min_support_)++unsupported_linearizations;
    const auto H=(scale*a.H).eval();const auto g=(-scale*a.gradient).eval();
    return std::make_shared<gtsam::HessianFactor>(keys()[0],keys()[1],H.block<6,6>(0,0),
      H.block<6,6>(0,6),g.head<6>(),H.block<6,6>(6,6),g.tail<6>(),2*scale*a.cost);
  }
 private:
  EllipsoidsPtr target_,source_;
  double radius_,huber_;size_t min_support_;
};
inline Json ellipsoid_quality(const Ellipsoids& target,const Ellipsoids& source,
                              const gtsam::Pose3& T,const Json& cfg) {
  const double radius=cfg.at("correspondence_m"),inlier=cfg.at("huber_m");
  Matrix6 H=Matrix6::Zero();size_t count=0,reverse=0;double squared=0;
  auto direction=[&](const Ellipsoids& a,const Ellipsoids& b,const gtsam::Pose3& delta,bool forward) {
    size_t good=0;
    for(const auto& p:b.centers) {
      const Eigen::Vector3d q=delta.transformFrom(Eigen::Vector3d(p.head<3>()));
      const Eigen::Vector4d q4(q[0],q[1],q[2],1);size_t idx=0;double d=0;
      if(!a.tree->knn_search(q4.data(),1,&idx,&d)||d>=radius*radius)continue;
      const auto& P=a.projectors[idx];const Eigen::Vector3d r=P*(q-a.centers[idx].head<3>());
      if(r.norm()>=inlier)continue;
      ++good;squared+=r.squaredNorm();
      if(forward) {
        Eigen::Matrix<double,3,6> J;const auto R=delta.rotation().matrix();
        J.leftCols<3>()=-P*R*cross_matrix(p.head<3>());J.rightCols<3>()=P*R;
        H.noalias()+=J.transpose()*J;
      }
    }return good;
  };
  count=direction(target,source,T,true);reverse=direction(source,target,T.inverse(),false);
  const Eigen::Matrix<double,6,1> eig=Eigen::SelfAdjointEigenSolver<Matrix6>(H/std::max<size_t>(1,count)).eigenvalues();
  return {{"inliers",count},{"reverse_inliers",reverse},
    {"overlap",std::min(double(count)/source.centers.size(),double(reverse)/target.centers.size())},
    {"rmse_m",count+reverse?Json(std::sqrt(squared/(count+reverse))):Json(nullptr)},
    {"observability",eig[0]},{"condition",eig[5]/std::max(eig[0],1e-15)}};
}
inline std::string ellipsoid_rejection(const Json& q,const Json& cfg) {
  if(q.at("inliers").get<size_t>()<cfg.at("min_inliers").get<size_t>())return "insufficient_inliers";
  if(q.at("overlap").get<double>()<cfg.at("min_overlap").get<double>())return "low_overlap";
  if(q.at("rmse_m").is_null()||q.at("rmse_m").get<double>()>cfg.at("max_rmse_m").get<double>())return "high_rmse";
  if(q.at("observability").get<double>()<cfg.at("min_observability").get<double>()||
     q.at("condition").get<double>()>cfg.at("max_condition").get<double>())return "unobservable";
  return "accepted";
}
}  // namespace s3e_registration
