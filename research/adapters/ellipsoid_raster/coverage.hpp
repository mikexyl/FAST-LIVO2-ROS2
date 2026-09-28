#pragma once
#include <cmath>
#ifdef __CUDACC__
#define ER_HD __host__ __device__
#else
#define ER_HD
#endif
namespace ellipsoid_raster {
ER_HD inline double clamp(double x,double a,double b) { return fmin(b,fmax(a,x)); }
// c[3], S[9]: geometric shape matrix B diag(axes^2) B^T, not uncertainty.
struct Primitive { double c[3], s[9]; };
struct Slice { double a,b,c,lo,hi; bool bounded; };
struct Integrand {
 const Primitive* e; Slice band; double y0,y1,radius;
 ER_HD double operator()(double x) const {
  const auto& s=e->s; double dx=x-e->c[0];
  double q=1-dx*dx/s[0]; if(q<=0||fabs(x)>=radius) return 0;
  double my=e->c[1]+s[3]/s[0]*dx, mz=e->c[2]+s[6]/s[0]*dx;
  double vy=fmax(0.,q*(s[4]-s[3]*s[1]/s[0]));
  double yz=q*(s[5]-s[3]*s[2]/s[0]);
  double vz=fmax(0.,q*(s[8]-s[6]*s[2]/s[0]));
  double sy=sqrt(vy), low=my-sy, high=my+sy;
  if(band.bounded) {
   double mh=mz-band.a*x-band.b*my-band.c;
   double vh=fmax(0.,vz-2*band.b*yz+band.b*band.b*vy);
   double yh=yz-band.b*vy, sh=sqrt(vh);
   double h0=fmax(band.lo,mh-sh), h1=fmin(band.hi,mh+sh);
   if(h0>h1) return 0;
   if(vh>1e-25&&sy>1e-15) {
    double hl=clamp(mh-yh/sy,h0,h1)-mh;
    double hu=clamp(mh+yh/sy,h0,h1)-mh;
    double cond=fmax(0.,vy-yh*yh/vh);
    low=my+yh/vh*hl-sqrt(fmax(0.,cond*(1-hl*hl/vh)));
    high=my+yh/vh*hu+sqrt(fmax(0.,cond*(1-hu*hu/vh)));
   } else if(mh<band.lo||mh>band.hi) return 0;
  }
  double disk=sqrt(fmax(0.,radius*radius-x*x));
  return fmax(0.,fmin(fmin(high,y1),disk)-fmax(fmax(low,y0),-disk));
 }
};
// Composite Gauss quadrature plus adaptive subdivision. X is clipped to the
// exact primitive support first, retaining even subpixel-width ellipsoids.
ER_HD inline double gauss8(const Integrand& f,double a,double b) {
 const double t[4]={.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363};
 const double w[4]={.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763};
 double m=(a+b)*.5,h=(b-a)*.5,v=0;
 for(int k=0;k<4;++k)v+=w[k]*(f(m-h*t[k])+f(m+h*t[k]));
 return h*v;
}
// Extremize x subject to one linear slab n.p in [lo,hi]. This clips
// integration to the actual support of even arbitrarily thin height bands.
ER_HD inline bool restrict_x(const Primitive& e, double nx,double ny,double nz,
                            double lo,double hi,double& a,double& b) {
 double m=nx*e.c[0]+ny*e.c[1]+nz*e.c[2];
 double cov=e.s[0]*nx+e.s[1]*ny+e.s[2]*nz;
 double v=nx*cov+ny*(e.s[3]*nx+e.s[4]*ny+e.s[5]*nz)+nz*(e.s[6]*nx+e.s[7]*ny+e.s[8]*nz);
 double sh=sqrt(fmax(0.,v)),sx=sqrt(e.s[0]);
 double l=fmax(lo,m-sh),h=fmin(hi,m+sh);if(l>=h)return false;
 if(v>1e-25) {
  double u=clamp(m-cov/sx,l,h)-m,w=clamp(m+cov/sx,l,h)-m;
  double cond=fmax(0.,e.s[0]-cov*cov/v);
  a=fmax(a,e.c[0]+cov/v*u-sqrt(fmax(0.,cond*(1-u*u/v))));
  b=fmin(b,e.c[0]+cov/v*w+sqrt(fmax(0.,cond*(1-w*w/v))));
 }
 return a<b;
}
ER_HD inline double ellipse_arc(double d,double sx) {
 double u=clamp(d/sx,-1.,1.);
 return .5*sx*(u*sqrt(fmax(0.,1-u*u))+asin(u));
}
// Exact rectangle integral of a projected conic. Break only where either
// ellipse envelope crosses a horizontal pixel edge; each piece has an analytic
// antiderivative. Most primitives are wholly inside one height band and disk.
ER_HD inline double rectangle_coverage(const Primitive& e,double a,double b,double y0,double y1) {
 double sx=sqrt(e.s[0]),sy=sqrt(e.s[4]);
 double beta=e.s[1]/e.s[0],cond=sqrt(fmax(0.,e.s[4]-e.s[1]*e.s[1]/e.s[0]));
 double cuts[6];int n=2;cuts[0]=a;cuts[1]=b;
 for(int k=0;k<2;++k) {
  double dy=(k?y1:y0)-e.c[1],q=1-dy*dy/e.s[4];
  if(q<=0)continue;
  double m=e.c[0]+e.s[1]/e.s[4]*dy;
  double h=sqrt(fmax(0.,q*(e.s[0]-e.s[1]*e.s[1]/e.s[4])));
  if(m-h>a&&m-h<b)cuts[n++]=m-h;
  if(m+h>a&&m+h<b)cuts[n++]=m+h;
 }
 for(int i=1;i<n;++i)for(int j=i;j>0&&cuts[j]<cuts[j-1];--j) {
  double t=cuts[j];cuts[j]=cuts[j-1];cuts[j-1]=t;
 }
 double area=0;
 for(int i=0;i<n-1;++i) {
  double l=cuts[i]-e.c[0],r=cuts[i+1]-e.c[0],m=(l+r)*.5;
  double center=e.c[1]+beta*m,h=cond*sqrt(fmax(0.,1-m*m/e.s[0]));
  if(center+h<=y0||center-h>=y1)continue;
  double mean=e.c[1]*(r-l)+.5*beta*(r*r-l*l);
  double arc=cond*(ellipse_arc(r,sx)-ellipse_arc(l,sx));
  double upper=center+h<y1?mean+arc:y1*(r-l);
  double lower=center-h>y0?mean-arc:y0*(r-l);
  area+=fmax(0.,upper-lower);
 }
 return area;
}
ER_HD inline double coverage(const Primitive& e,const Slice& slice,double x0,double y0,double step,double radius) {
 Slice band=slice;
 if(band.bounded) {
  double nx=-band.a,ny=-band.b,m=nx*e.c[0]+ny*e.c[1]+e.c[2]-band.c;
  double h=sqrt(fmax(0.,nx*(e.s[0]*nx+e.s[1]*ny+e.s[2])+ny*(e.s[3]*nx+e.s[4]*ny+e.s[5])+e.s[6]*nx+e.s[7]*ny+e.s[8]));
  if(m+h<=band.lo||m-h>=band.hi)return 0;
  if(m-h>=band.lo&&m+h<=band.hi)band.bounded=false;
 }
 double sy=sqrt(e.s[4]); if(y0>=e.c[1]+sy||y0+step<=e.c[1]-sy)return 0;
 double sx=sqrt(e.s[0]),a=fmax(fmax(x0,e.c[0]-sx),-radius);
 double b=fmin(fmin(x0+step,e.c[0]+sx),radius); if(a>=b)return 0;
 if(!restrict_x(e,0,1,0,y0,y0+step,a,b))return 0;
 if(band.bounded&&!restrict_x(e,-band.a,-band.b,1,band.lo+band.c,band.hi+band.c,a,b))return 0;
 double xmax=fmax(fabs(a),fabs(b)),ymax=fmax(fabs(y0),fabs(y0+step));
 if(!band.bounded&&xmax*xmax+ymax*ymax<=radius*radius)
  return clamp(rectangle_coverage(e,a,b,y0,y0+step)/(step*step),0.,1.);
 Integrand f{&e,band,y0,y0+step,radius};
 struct Node { double a,b,whole,eps; int depth; }; Node stack[16]; int n=1;
 stack[0]={a,b,gauss8(f,a,b),1e-7*fmin(step*step,sx*sy),0}; double sum=0;
 while(n) {
  Node v=stack[--n];double m=(v.a+v.b)*.5,l=gauss8(f,v.a,m),r=gauss8(f,m,v.b);
  // Subdivide at least twice: thin slab intersections can lie between nodes.
  if(v.depth>=12||(v.depth>=2&&fabs(l+r-v.whole)<=v.eps))sum+=l+r;
  else { stack[n++]={m,v.b,r,v.eps*.5,v.depth+1};stack[n++]={v.a,m,l,v.eps*.5,v.depth+1}; }
 }
 return clamp(sum/(step*step),0.,1.);
}
}
