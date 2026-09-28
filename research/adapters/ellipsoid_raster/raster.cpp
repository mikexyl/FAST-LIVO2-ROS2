#include "coverage.hpp"
#include <algorithm>
#include <exception>
using namespace ellipsoid_raster;
extern "C" int raster_cpu(const double* primitives,int count,const int* offsets,const int* ids,
 const double* bands,int layers,double radius,double step,int origin,int size,int tile,double* output) {
 try {
  int tiles=(size+tile-1)/tile;
  for(int x=0;x<size;++x)for(int y=0;y<size;++y) {
   int t=(x/tile)*tiles+y/tile;
   for(int k=0;k<layers;++k) {
    Slice s{bands[k*6],bands[k*6+1],bands[k*6+2],bands[k*6+3],bands[k*6+4],bool(bands[k*6+5])};
    double value=0;
    for(int j=offsets[t];j<offsets[t+1];++j) {
     const auto& e=*reinterpret_cast<const Primitive*>(primitives+12*ids[j]);
     value+=coverage(e,s,(origin+x)*step,(origin+y)*step,step,radius);
    }
    output[(k*size+x)*size+y]=value;
   }
  }
  return 0;
 } catch(const std::exception&) { return 1; }
}
