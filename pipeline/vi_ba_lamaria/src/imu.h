#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace lamaria_ba {
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using M9 = Eigen::Matrix<double,9,9>;
using Bias = Eigen::Matrix<double,6,1>; // gyro, accel
inline M3 Skew(const V3& a) { M3 m; m << 0,-a.z(),a.y(),a.z(),0,-a.x(),-a.y(),a.x(),0; return m; }
inline Eigen::Quaterniond Exp(const V3& w) {
  const double n=w.norm();
  if(n<1e-10) return Eigen::Quaterniond(1,.5*w.x(),.5*w.y(),.5*w.z()).normalized();
  return Eigen::Quaterniond(Eigen::AngleAxisd(n,w/n));
}
inline V3 Log(Eigen::Quaterniond q) {
  q.normalize(); if(q.w()<0) q.coeffs()*=-1;
  const double n=q.vec().norm();
  return n<1e-10 ? 2*q.vec() : (2*std::atan2(n,q.w())/n)*q.vec();
}
inline M3 RightJacobian(const V3& w) {
  const double n=w.norm(); const M3 k=Skew(w);
  if(n<1e-6) return M3::Identity()-.5*k+k*k/6;
  return M3::Identity()-(1-std::cos(n))/(n*n)*k+(n-std::sin(n))/(n*n*n)*k*k;
}
struct Sample { int64_t ns; V3 w,a; };
struct Segment { double dt; V3 w,a; };
struct Preint {
  double dt=0;
  Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
  V3 v=V3::Zero(), p=V3::Zero();
  Bias bias=Bias::Zero();
  Eigen::Matrix<double,9,6> J=Eigen::Matrix<double,9,6>::Zero(); // dp,dv,dr
  M9 covariance=M9::Zero(), W=M9::Zero();
};
inline std::vector<Sample> LoadImu(const std::string& path) {
  std::ifstream f(path); if(!f) throw std::runtime_error("Cannot read IMU CSV: "+path);
  std::vector<Sample> data; std::string line;
  while(std::getline(f,line)) {
    if(line.empty() || line[0]=='#') continue;
    std::replace(line.begin(),line.end(),',',' '); std::istringstream s(line); Sample x;
    if(!(s>>x.ns>>x.w.x()>>x.w.y()>>x.w.z()>>x.a.x()>>x.a.y()>>x.a.z()))
      throw std::runtime_error("Malformed native IMU CSV");
    if(!x.w.allFinite() || !x.a.allFinite() || (!data.empty() && x.ns<=data.back().ns))
      throw std::runtime_error("Nonfinite or nonmonotonic native IMU CSV");
    data.push_back(x);
  }
  if(data.size()<2) throw std::runtime_error("Insufficient IMU data"); return data;
}
inline Sample Interpolate(const Sample& a,const Sample& b,int64_t ns) {
  const double r=double(ns-a.ns)/double(b.ns-a.ns);
  return {ns,(1-r)*a.w+r*b.w,(1-r)*a.a+r*b.a};
}
// Exact image boundaries are interpolated from bracketing native IMU samples.
// Every intervening native sample is retained; a long image interval is NOT dropped.
inline std::vector<Segment> Slice(const std::vector<Sample>& imu,int64_t t0,int64_t t1) {
  if(t0>=t1 || t0<imu.front().ns || t1>imu.back().ns) throw std::runtime_error("IMU does not bracket image interval");
  auto it=std::upper_bound(imu.begin(),imu.end(),t0,[](int64_t t,const Sample& x){return t<x.ns;});
  if(it==imu.end()) throw std::runtime_error("Bad IMU boundary");
  Sample prev=Interpolate(*(it-1),*it,t0); std::vector<Segment> out;
  while(prev.ns<t1) {
    if(it==imu.end()) throw std::runtime_error("Missing IMU tail");
    if(it->ns-(it-1)->ns>10000000) throw std::runtime_error("IMU sample gap exceeds 10 ms");
    Sample next=it->ns<=t1 ? *it : Interpolate(*(it-1),*it,t1);
    out.push_back({double(next.ns-prev.ns)*1e-9,.5*(prev.w+next.w),.5*(prev.a+next.a)});
    prev=next; ++it;
  }
  return out;
}
inline void MidpointJacobians(const Eigen::Quaterniond& q,const V3& w,const V3& a,double d,
                              M9& F,Eigen::Matrix<double,9,6>& G) {
  const V3 half=w*(.5*d);
  const M3 Rh=(q*Exp(half)).toRotationMatrix();
  // Right attitude error must be transported from the interval start into
  // the midpoint body frame where the specific force is evaluated.
  const M3 transport=Exp(-half).toRotationMatrix(),Jhalf=RightJacobian(half);
  F=M9::Identity();F.block<3,3>(0,3)=M3::Identity()*d;
  F.block<3,3>(0,6)=-.5*Rh*Skew(a)*transport*d*d;
  F.block<3,3>(3,6)=-Rh*Skew(a)*transport*d;
  F.block<3,3>(6,6)=Exp(-w*d).toRotationMatrix();
  G.setZero();
  // Sampled noise is additive in angular rate/specific force. A rate error
  // perturbs the midpoint rotation through Jr(w*d/2), not the identity.
  G.block<3,3>(0,0)=-.25*Rh*Skew(a)*Jhalf*d*d*d;
  G.block<3,3>(3,0)=-.5*Rh*Skew(a)*Jhalf*d*d;
  G.block<3,3>(6,0)=RightJacobian(w*d)*d;
  G.block<3,3>(0,3)=.5*Rh*d*d;
  G.block<3,3>(3,3)=Rh*d;
}
inline Preint Integrate(const std::vector<Segment>& data,const Bias& bias,double ng,double na,bool uncertainty=true) {
  Preint x; x.bias=bias;
  for(const auto& s:data) {
    const double d=s.dt; const V3 w=s.w-bias.head<3>(), a=s.a-bias.tail<3>();
    const M3 Rh=(x.q*Exp(w*(.5*d))).toRotationMatrix();
    if(uncertainty) {
      M9 F;Eigen::Matrix<double,9,6> G;MidpointJacobians(x.q,w,a,d,F,G);
      // Noise densities: sampled angular/acceleration errors have variance sigma^2 / dt.
      Eigen::Matrix<double,6,1> variance; variance.head<3>().setConstant(ng*ng/d); variance.tail<3>().setConstant(na*na/d);
      x.covariance=F*x.covariance*F.transpose()+G*variance.asDiagonal()*G.transpose();
    }
    x.p+=x.v*d+.5*(Rh*a)*d*d; x.v+=(Rh*a)*d;
    x.q=(x.q*Exp(w*d)).normalized(); x.dt+=d;
  }
  if(uncertainty) {
    // Bias derivatives of the exact same discrete integration, checked independently in contracts.
    for(int j=0;j<6;++j) {
      const double h=j<3 ? 1e-5:1e-4; Bias bp=bias,bm=bias; bp[j]+=h;bm[j]-=h;
      const auto p=Integrate(data,bp,ng,na,false), m=Integrate(data,bm,ng,na,false);
      x.J.block<3,1>(0,j)=(p.p-m.p)/(2*h); x.J.block<3,1>(3,j)=(p.v-m.v)/(2*h);
      x.J.block<3,1>(6,j)=(Log(x.q.conjugate()*p.q)-Log(x.q.conjugate()*m.q))/(2*h);
    }
    x.covariance=.5*(x.covariance+x.covariance.transpose()).eval();
    x.covariance.diagonal().array()+=1e-15;
    Eigen::LLT<M9> llt(x.covariance);
    if(llt.info()!=Eigen::Success) throw std::runtime_error("IMU covariance is not SPD");
    x.W=llt.matrixL().solve(M9::Identity());
  }
  return x;
}
} // namespace
