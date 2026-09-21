// Visualization-only access to the same native density/ORB/HBST/RANSAC stages.
#define S3E_INSPECTION_BUILD
#include "adapter.cpp"

namespace map_closures {
std::pair<Eigen::Isometry2d,std::size_t> InspectRansacAlignment2D(
    const std::vector<PointPair>&,std::vector<int>&);
}

class Inspector : public Adapter {
public:
    using Adapter::Adapter;
    py::dict density(Array cloud,const Eigen::Matrix4d &ground) {
        if (cloud.ndim()!=2 || cloud.shape(1)!=3 || cloud.shape(0)<30 || !ground.allFinite())
            throw std::invalid_argument("Invalid density inspection inputs");
        auto x=cloud.unchecked<2>();std::vector<Eigen::Vector3d> points;
        for (ssize_t i=0;i<cloud.shape(0);++i) {
            Eigen::Vector3d p(x(i,0),x(i,1),x(i,2));
            if (!p.allFinite() || p.norm()>1000) throw std::invalid_argument("Invalid cloud point");
            points.push_back(p);
        }
        const auto map=map_closures::GenerateDensityMap(points,ground,config.density_map_resolution,config.density_threshold);
        Bytes image({ssize_t(map.grid.rows),ssize_t(map.grid.cols)});
        std::memcpy(image.mutable_data(),map.grid.data,map.grid.total());
        std::vector<cv::KeyPoint> keys;cv::Mat bits;
        orb->detectAndCompute(map.grid,cv::noArray(),keys,bits);
        Array xy({ssize_t(keys.size()),ssize_t(2)}),angles(keys.size()),responses(keys.size());
        auto p=xy.mutable_unchecked<2>();auto a=angles.mutable_unchecked<1>();auto r=responses.mutable_unchecked<1>();
        for (size_t i=0;i<keys.size();++i) {p(i,0)=keys[i].pt.x;p(i,1)=keys[i].pt.y;a(i)=keys[i].angle;r(i)=keys[i].response;}
        std::vector<int> kept;
        if (bits.rows>=2) {
            std::vector<std::vector<cv::DMatch>> self;cv::BFMatcher(cv::NORM_HAMMING).knnMatch(bits,bits,self,2);
            for (const auto &m:self) if (m.size()==2 && m[1].distance>35) kept.push_back(m[0].queryIdx);
        }
        Bytes descriptors({ssize_t(keys.size()),ssize_t(32)});
        if (!keys.empty()) std::memcpy(descriptors.mutable_data(),bits.data,keys.size()*32);
        py::dict out;out["image"]=image;out["lower_bound"]=map.lower_bound;
        out["orb_xy"]=xy;out["orb_angles"]=angles;out["orb_responses"]=responses;
        out["orb_bits"]=descriptors;out["kept_indices"]=kept;
        return out;
    }
    py::dict correspondences(const py::dict &packet,int id) {
        auto f=decode(packet);auto q=matchables(f);Tree::MatchVectorMap matches;
        tree.match(q,matches,config.hamming_distance_threshold);for(auto p:q) delete p;
        const auto &ms=matches[id];const ssize_t n=ms.size();
        Array qxy({n,ssize_t(2)}),rxy({n,ssize_t(2)}),distances(n);
        auto a=qxy.mutable_unchecked<2>(),b=rxy.mutable_unchecked<2>();auto d=distances.mutable_unchecked<1>();
        std::vector<map_closures::PointPair> pairs;
        for (ssize_t i=0;i<n;++i) {
            const auto &m=ms[i];a(i,0)=m.object_query.pt.y;a(i,1)=m.object_query.pt.x;
            b(i,0)=m.object_references[0].pt.y;b(i,1)=m.object_references[0].pt.x;d(i)=m.distance;
            pairs.emplace_back(Eigen::Vector2d(b(i,0),b(i,1)),Eigen::Vector2d(a(i,0),a(i,1)));
        }
        std::vector<int> membership;Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
        if(n>2) {
            const auto [planar,count]=map_closures::InspectRansacAlignment2D(pairs,membership);
            T.block<2,2>(0,0)=planar.linear();T.block<2,1>(0,3)=planar.translation()*config.density_map_resolution;
            T=f.ground.inverse()*T*grounds.at(id);
        }
        py::dict out;out["query_xy"]=qxy;out["candidate_xy"]=rxy;out["hamming"]=distances;
        out["ransac_inlier_indices"]=membership;out["T_i_j"]=T;return out;
    }
};

PYBIND11_MODULE(s3e_mapclosures_inspection,m) {
    m.attr("upstream_commit")="1710f15db000a579324e3ba045cd64a0b4d706da";
    py::class_<Inspector>(m,"Inspector").def(py::init<float,float,int>())
        .def("density",&Inspector::density).def("add",&Inspector::add).def("correspondences",&Inspector::correspondences);
}
