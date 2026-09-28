#include "coverage.hpp"
#include <cuda_runtime.h>
using namespace ellipsoid_raster;
extern "C" const char* raster_cuda_variant() { return "pixel_serial_v1"; }
__global__ void render(const double* p,const int* offsets,const int* ids,const double* bands,int layers,
 double radius,double step,int origin,int size,int tile,double* out) {
 int x=blockIdx.x*tile+threadIdx.x,y=blockIdx.y*tile+threadIdx.y;
 if(x>=size||y>=size)return;
 int t=blockIdx.x*gridDim.y+blockIdx.y;
 for(int k=0;k<layers;++k) {
  Slice s{bands[k*6],bands[k*6+1],bands[k*6+2],bands[k*6+3],bands[k*6+4],bool(bands[k*6+5])};
  double value=0;
  for(int j=offsets[t];j<offsets[t+1];++j)
   value+=coverage(*reinterpret_cast<const Primitive*>(p+12*ids[j]),s,(origin+x)*step,(origin+y)*step,step,radius);
  out[(k*size+x)*size+y]=value;
 }
}
extern "C" int raster_cuda(const double* p,int count,const int* offsets,const int* ids,const double* bands,
 int layers,double radius,double step,int origin,int size,int tile,double* out,size_t* peak_bytes) {
 int tiles=(size+tile-1)/tile,nt=tiles*tiles,ni=offsets[nt];
 double *dp=nullptr,*db=nullptr,*dst=nullptr;int *dof=nullptr,*di=nullptr;cudaError_t status=cudaSuccess;
 size_t ps=count*12*sizeof(double),bs=layers*6*sizeof(double),os=(nt+1)*sizeof(int),is=ni*sizeof(int),ds=layers*size*size*sizeof(double);
 *peak_bytes=ps+bs+os+is+ds;
 #define CK(expr) if((status=(expr))!=cudaSuccess)goto done
 CK(cudaMalloc(&dp,ps?ps:1));CK(cudaMalloc(&db,bs));CK(cudaMalloc(&dof,os));CK(cudaMalloc(&di,is?is:1));CK(cudaMalloc(&dst,ds));
 CK(cudaMemcpy(dp,p,ps,cudaMemcpyHostToDevice));CK(cudaMemcpy(db,bands,bs,cudaMemcpyHostToDevice));
 CK(cudaMemcpy(dof,offsets,os,cudaMemcpyHostToDevice));CK(cudaMemcpy(di,ids,is,cudaMemcpyHostToDevice));
 render<<<dim3(tiles,tiles),dim3(tile,tile)>>>(dp,dof,di,db,layers,radius,step,origin,size,tile,dst);
 CK(cudaGetLastError());CK(cudaDeviceSynchronize());CK(cudaMemcpy(out,dst,ds,cudaMemcpyDeviceToHost));
 done:cudaFree(dp);cudaFree(db);cudaFree(dof);cudaFree(di);cudaFree(dst);return int(status);
 #undef CK
}
