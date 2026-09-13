// MapClosures serialization adapter. Feature extraction and pose construction
// follow PRBonn/MapClosures 1710f15; see LICENSE for upstream's MIT notice.
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <map_closures/MapClosures.hpp>
#include <map_closures/GroundAlign.hpp>
#include <map_closures/AlignRansac2D.hpp>
#include <set>
#include <map>
#include <cstring>

namespace py = pybind11;
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
using Bytes = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>;
struct Features {
    Eigen::Matrix4d ground = Eigen::Matrix4d::Identity();
    std::vector<cv::KeyPoint> points;
    cv::Mat descriptors;
};

Features decode(const py::dict &packet) {
    Features f;
    f.ground = packet["ground"].cast<Eigen::Matrix4d>();
    Array xy = packet["xy"].cast<Array>();
    Bytes bits = packet["bits"].cast<Bytes>();
    if (xy.ndim()!=2 || xy.shape(1)!=2 || bits.ndim()!=2 || bits.shape(1)!=32 || bits.shape(0)!=xy.shape(0) || xy.shape(0)>500)
        throw std::invalid_argument("Invalid MapClosures feature packet");
    if (!f.ground.allFinite()) throw std::invalid_argument("Invalid ground alignment");
    auto p=xy.unchecked<2>();
    for (ssize_t i=0;i<xy.shape(0);++i) {
        if (!std::isfinite(p(i,0)) || !std::isfinite(p(i,1))) throw std::invalid_argument("Invalid density keypoint");
        f.points.emplace_back(float(p(i,1)),float(p(i,0)),31.f);
    }
    f.descriptors=cv::Mat(int(bits.shape(0)),32,CV_8U,const_cast<uint8_t*>(bits.data())).clone();
    return f;
}

py::dict encode(const Features &f) {
    Array xy({ssize_t(f.points.size()),ssize_t(2)});
    Bytes bits({ssize_t(f.points.size()),ssize_t(32)});
    auto p=xy.mutable_unchecked<2>();
    for (size_t i=0;i<f.points.size();++i) {p(i,0)=f.points[i].pt.y;p(i,1)=f.points[i].pt.x;}
    if (!f.points.empty()) std::memcpy(bits.mutable_data(),f.descriptors.data,f.points.size()*32);
    py::dict d;d["ground"]=f.ground;d["xy"]=xy;d["bits"]=bits;
    return d;
}

std::vector<Matchable*> matchables(const Features &f, int id=0) {
    std::vector<Matchable*> out;out.reserve(f.points.size());
    for (size_t i=0;i<f.points.size();++i) out.push_back(new Matchable(f.points[i],f.descriptors.row(int(i)),id));
    return out;
}

class Adapter {
protected:
    map_closures::Config config;
    cv::Ptr<cv::ORB> orb;
    Tree tree;
    std::map<int,Eigen::Matrix4d> grounds;
public:
    Adapter(float resolution,float threshold,int hamming) {
        if (resolution<=0 || threshold<0 || threshold>=1 || hamming<=0 || hamming>256)
            throw std::invalid_argument("Invalid MapClosures configuration");
        config.density_map_resolution=resolution;config.density_threshold=threshold;config.hamming_distance_threshold=hamming;
        cv::setNumThreads(1);
        orb=cv::ORB::create(500,1.f,1,31,0,2,cv::ORB::HARRIS_SCORE,31,35);
    }
    py::dict describe(Array cloud) {
        if (cloud.ndim()!=2 || cloud.shape(1)!=3 || cloud.shape(0)<30)
            throw std::invalid_argument("MapClosures needs a nonempty Nx3 local map");
        auto x=cloud.unchecked<2>();std::vector<Eigen::Vector3d> points;points.reserve(cloud.shape(0));
        for (ssize_t i=0;i<cloud.shape(0);++i) {
            Eigen::Vector3d p(x(i,0),x(i,1),x(i,2));
            if (!p.allFinite() || p.norm()>1000) throw std::invalid_argument("Invalid local map point");
            points.push_back(p);
        }
        Features f;
        f.ground=map_closures::AlignToLocalGround(points,config.density_map_resolution);
        if (!f.ground.allFinite()) throw std::runtime_error("Ground alignment failed");
        const auto density=map_closures::GenerateDensityMap(points,f.ground,config.density_map_resolution,config.density_threshold);
        std::vector<cv::KeyPoint> keypoints;cv::Mat descriptors;
        orb->detectAndCompute(density.grid,cv::noArray(),keypoints,descriptors);
        if (descriptors.rows>=2) {
            std::vector<std::vector<cv::DMatch>> self;
            cv::BFMatcher(cv::NORM_HAMMING).knnMatch(descriptors,descriptors,self,2);
            for (const auto &m:self) if (m.size()==2 && m[1].distance>35) {
                const int i=m[0].queryIdx;auto key=keypoints[i];
                key.pt.x+=float(density.lower_bound.y());key.pt.y+=float(density.lower_bound.x());
                f.points.push_back(key);f.descriptors.push_back(descriptors.row(i));
            }
        }
        auto result=encode(f);result["density_rows"]=density.grid.rows;result["density_cols"]=density.grid.cols;
        return result;
    }
    void add(int id,const py::dict &packet) {
        if (id<0 || grounds.count(id)) throw std::invalid_argument("Duplicate/invalid local map ID");
        auto f=decode(packet);grounds.emplace(id,f.ground);
        auto m=matchables(f,id);tree.add(m,srrg_hbst::SplittingStrategy::SplitEven);
    }
    py::dict align(int id,const Tree::MatchVector &matches,const Features &query,const Eigen::Matrix4d &reference_ground) const {
        py::dict out;out["keyframe_id"]=id;out["matches"]=matches.size();out["inliers"]=0;out["valid_pose"]=false;
        if (matches.size()<=2) return out;
        std::vector<map_closures::PointPair> pairs;pairs.reserve(matches.size());
        for (const auto &m:matches) pairs.emplace_back(
            Eigen::Vector2d(m.object_references[0].pt.y,m.object_references[0].pt.x),
            Eigen::Vector2d(m.object_query.pt.y,m.object_query.pt.x));
        const auto [planar,n]=map_closures::RansacAlignment2D(pairs);
        Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
        T.block<2,2>(0,0)=planar.linear();T.block<2,1>(0,3)=planar.translation()*config.density_map_resolution;
        T=query.ground.inverse()*T*reference_ground;
        const Eigen::Matrix3d R=T.block<3,3>(0,0);
        bool valid=T.allFinite() && std::abs(R.determinant()-1)<1e-5 && (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()<1e-5;
        out["inliers"]=n;out["valid_pose"]=valid;
        if (valid) out["T_i_j"]=T;
        return out;
    }
    py::list query(const py::dict &packet,const std::vector<int> &eligible,int top_k) {
        auto f=decode(packet);auto q=matchables(f);Tree::MatchVectorMap matches;
        tree.match(q,matches,config.hamming_distance_threshold);
        for (auto p:q) delete p;
        std::vector<std::pair<int,size_t>> ranked;
        for (int id:eligible) {
            if (!grounds.count(id)) throw std::invalid_argument("Unindexed local map");
            auto it=matches.find(id);
            if (it!=matches.end() && it->second.size()>2) ranked.emplace_back(id,it->second.size());
        }
        // Cheap binary-match shortlist bounds native 2D RANSAC work per query.
        std::sort(ranked.begin(),ranked.end(),[](auto a,auto b){return a.second!=b.second?a.second>b.second:a.first<b.first;});
        if (int(ranked.size())>top_k) ranked.resize(top_k);
        py::list out;
        for (const auto &[id,n]:ranked) out.append(align(id,matches.at(id),f,grounds.at(id)));
        return out;
    }
    py::dict pair(const py::dict &query,const py::dict &candidate) {
        auto qf=decode(query),cf=decode(candidate);Tree local;
        local.add(matchables(cf,0),srrg_hbst::SplittingStrategy::SplitEven);
        auto q=matchables(qf);Tree::MatchVectorMap matches;local.match(q,matches,config.hamming_distance_threshold);
        for (auto p:q) delete p;
        return align(0,matches[0],qf,cf.ground);
    }
};

#ifndef S3E_INSPECTION_BUILD
PYBIND11_MODULE(s3e_mapclosures_native,m) {
    m.attr("upstream_commit")="1710f15db000a579324e3ba045cd64a0b4d706da";
    py::class_<Adapter>(m,"MapClosures")
        .def(py::init<float,float,int>())
        .def("describe",&Adapter::describe).def("add",&Adapter::add)
        .def("query",&Adapter::query).def("pair",&Adapter::pair);
}
#endif
