// Deterministic two-stage voxel centroids for the existing surface sampler.
// Integer accumulation at 0.1 micrometre precision avoids atomic sum ordering.
#include <cuda_runtime.h>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <cstring>

namespace {
struct Cell { unsigned key,count; unsigned long long x,y,z; };
#ifndef ELLIPSOID_INITIAL_CAPACITY
#define ELLIPSOID_INITIAL_CAPACITY (1U<<23)
#endif
constexpr unsigned local_capacity=1<<19;
constexpr double quantization=1e7;
std::string error;
void check(cudaError_t code) { if(code!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(code)); }
template<class T> void allocate(T*& p,size_t n) {check(cudaMalloc(&p,n*sizeof(T)));}
__device__ void insert(Cell* cells,unsigned capacity,unsigned* active,unsigned* size,int* failed,
                       unsigned key,long long x,long long y,long long z) {
  unsigned h=(key*2654435761U)&(capacity-1);
  for(unsigned probe=0;probe<capacity;++probe,h=(h+1)&(capacity-1)) {
    unsigned previous=atomicCAS(&cells[h].key,0U,key);
    if(previous!=0U && previous!=key)continue;
    if(previous==0U)active[atomicAdd(size,1U)]=h;
    atomicAdd(&cells[h].count,1U);
    atomicAdd(&cells[h].x,static_cast<unsigned long long>(x));
    atomicAdd(&cells[h].y,static_cast<unsigned long long>(y));
    atomicAdd(&cells[h].z,static_cast<unsigned long long>(z));return;
  }
  atomicExch(failed,1);
}
__global__ void sample(const double* ellipses,int n,const double* sphere,int samples,
                       double voxel,double range,int half,Cell* cells,unsigned* active,unsigned* size,int* failed) {
  int index=blockIdx.x*blockDim.x+threadIdx.x;if(index>=n*samples)return;
  const double* e=ellipses+(index/samples)*15;const double* u=sphere+(index%samples)*3;
  double a=e[3]*u[0],b=e[4]*u[1],c=e[5]*u[2];
  double x=e[0]+e[6]*a+e[7]*b+e[8]*c;
  double y=e[1]+e[9]*a+e[10]*b+e[11]*c;
  double z=e[2]+e[12]*a+e[13]*b+e[14]*c;
  if(x*x+y*y+z*z>range*range)return;
  int ix=int(floor(x/voxel))+half,iy=int(floor(y/voxel))+half,iz=int(floor(z/voxel))+half;
  int width=2*half;if(ix<0 || iy<0 || iz<0 || ix>=width || iy>=width || iz>=width)return;
  unsigned key=(ix*width+iy)*width+iz+1;
  insert(cells,local_capacity,active,size,failed,key,llrint(x*quantization),llrint(y*quantization),llrint(z*quantization));
}
__global__ void merge(const Cell* local,const unsigned* active,const unsigned* size,
                      Cell* global,unsigned capacity,unsigned* global_active,unsigned* global_size,int* failed) {
  unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=*size)return;
  Cell cell=local[active[i]];
  insert(global,capacity,global_active,global_size,failed,cell.key,
      llrint(double(static_cast<long long>(cell.x))/cell.count),
      llrint(double(static_cast<long long>(cell.y))/cell.count),
      llrint(double(static_cast<long long>(cell.z))/cell.count));
}
// Preserve each voxel's integer sums/count and active order exactly on growth.
__global__ void rehash(const Cell* old,const unsigned* active,unsigned n,
                       Cell* grown,unsigned capacity,unsigned* grown_active) {
  unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=n)return;
  Cell c=old[active[i]];unsigned h=(c.key*2654435761U)&(capacity-1);
  while(atomicCAS(&grown[h].key,0U,c.key)!=0U)h=(h+1)&(capacity-1);
  grown[h].count=c.count;grown[h].x=c.x;grown[h].y=c.y;grown[h].z=c.z;
  grown_active[i]=h;
}
__global__ void compact(const Cell* cells,const unsigned* active,unsigned n,double* points) {
  unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=n)return;Cell c=cells[active[i]];
  points[3*i]=double(static_cast<long long>(c.x))/c.count/quantization;
  points[3*i+1]=double(static_cast<long long>(c.y))/c.count/quantization;
  points[3*i+2]=double(static_cast<long long>(c.z))/c.count/quantization;
}
struct Sampler {
  Cell *local=nullptr,*global=nullptr;unsigned *local_active=nullptr,*global_active=nullptr,*local_size=nullptr,*global_size=nullptr;
  int* failed=nullptr;double *ellipses=nullptr,*sphere=nullptr,*output=nullptr;
  double voxel,range;int half,sphere_size=0;
  unsigned global_capacity=ELLIPSOID_INITIAL_CAPACITY;
  Sampler(double v,double r):voxel(v),range(r),half(int(ceil(r/v))) {
    if(v<=0 || r<=0 || 8LL*half*half*half>=0xffffffffLL)throw std::runtime_error("Invalid CUDA voxel domain");
    allocate(local,local_capacity);allocate(global,global_capacity);allocate(local_active,local_capacity);allocate(global_active,global_capacity);
    allocate(local_size,1);allocate(global_size,1);allocate(failed,1);allocate(ellipses,10000*15);allocate(sphere,8200*3);allocate(output,global_capacity*3);
  }
  ~Sampler() {cudaFree(local);cudaFree(global);cudaFree(local_active);cudaFree(global_active);cudaFree(local_size);cudaFree(global_size);
    cudaFree(failed);cudaFree(ellipses);cudaFree(sphere);cudaFree(output);}
  void ensure_capacity(unsigned occupied,unsigned incoming) {
    if(size_t(occupied)+incoming<=global_capacity/2)return;
    unsigned capacity=global_capacity;
    while(size_t(occupied)+incoming>capacity/2) {
      if(capacity>=(1U<<30))throw std::runtime_error("CUDA voxel table index range exceeded");
      capacity*=2;
    }
    Cell* cells=nullptr;unsigned* active=nullptr;double* points=nullptr;
    try {
      allocate(cells,capacity);allocate(active,capacity);allocate(points,size_t(capacity)*3);
      check(cudaMemset(cells,0,size_t(capacity)*sizeof(Cell)));
      if(occupied)rehash<<<(occupied+255)/256,256>>>(global,global_active,occupied,cells,capacity,active);
      check(cudaGetLastError());check(cudaDeviceSynchronize());
    } catch(...) {cudaFree(cells);cudaFree(active);cudaFree(points);throw;}
    cudaFree(global);cudaFree(global_active);cudaFree(output);
    global=cells;global_active=active;output=points;global_capacity=capacity;
  }
};
}
extern "C" {
const char* surface_error() {return error.c_str();}
void* surface_create(double voxel,double range) {
  try {return new Sampler(voxel,range);}catch(const std::exception& e){error=e.what();return nullptr;}
}
void surface_destroy(void* p) {delete static_cast<Sampler*>(p);}
int surface_reset(void* p) {
  try {auto& s=*static_cast<Sampler*>(p);check(cudaMemset(s.global,0,size_t(s.global_capacity)*sizeof(Cell)));
    check(cudaMemset(s.global_size,0,4));check(cudaMemset(s.failed,0,4));return 0;
  }catch(const std::exception& e){error=e.what();return -1;}
}
int surface_add(void* p,const double* ellipses,int n,const double* sphere,int count) {
  try {auto& s=*static_cast<Sampler*>(p);if(n>10000 || count>8200 || n*count>250000)throw std::runtime_error("CUDA batch exceeds fixed budget");
    check(cudaMemset(s.local,0,local_capacity*sizeof(Cell)));check(cudaMemset(s.local_size,0,4));
    check(cudaMemcpy(s.ellipses,ellipses,n*15*sizeof(double),cudaMemcpyHostToDevice));
    if(s.sphere_size!=count){check(cudaMemcpy(s.sphere,sphere,count*3*sizeof(double),cudaMemcpyHostToDevice));s.sphere_size=count;}
    sample<<<(n*count+255)/256,256>>>(s.ellipses,n,s.sphere,count,s.voxel,s.range,s.half,s.local,s.local_active,s.local_size,s.failed);
    unsigned occupied,incoming;
    check(cudaMemcpy(&occupied,s.global_size,sizeof(unsigned),cudaMemcpyDeviceToHost));
    check(cudaMemcpy(&incoming,s.local_size,sizeof(unsigned),cudaMemcpyDeviceToHost));
    s.ensure_capacity(occupied,incoming);
    merge<<<(250000+255)/256,256>>>(s.local,s.local_active,s.local_size,s.global,s.global_capacity,s.global_active,s.global_size,s.failed);
    check(cudaGetLastError());return 0;
  }catch(const std::exception& e){error=e.what();return -1;}
}
int surface_size(void* p) {
  try {auto& s=*static_cast<Sampler*>(p);unsigned size;int failed;
    check(cudaMemcpy(&size,s.global_size,4,cudaMemcpyDeviceToHost));check(cudaMemcpy(&failed,s.failed,4,cudaMemcpyDeviceToHost));
    if(failed || size>s.global_capacity/2)throw std::runtime_error("CUDA voxel hash capacity exceeded");return int(size);
  }catch(const std::exception& e){error=e.what();return -1;}
}
int surface_copy(void* p,double* output,int n) {
  try {auto& s=*static_cast<Sampler*>(p);if(n){compact<<<(n+255)/256,256>>>(s.global,s.global_active,n,s.output);
      check(cudaMemcpy(output,s.output,n*3*sizeof(double),cudaMemcpyDeviceToHost));}return 0;
  }catch(const std::exception& e){error=e.what();return -1;}
}
unsigned surface_capacity(void* p) {return static_cast<Sampler*>(p)->global_capacity;}
}
