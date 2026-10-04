#ifndef ORB_SLAM3_G2O_FULL_CALIBRATION_H
#define ORB_SLAM3_G2O_FULL_CALIBRATION_H
#include "G2oRadialCalibration.h"
namespace ORB_SLAM3 {
typedef Eigen::Matrix<double,15,1> IntrinsicVector;
// Changing f/cx/cy may move a ray across the angular admission boundary; that
// is distinct from a failed inverse. Preserve the physical sensor mask, test
// actual numeric inversion of every previously valid sampled pixel, and reject
// radial or tangential/prism folds throughout the admitted angular domain.
inline bool ValidateFullCalibrationDomain(const Fisheye624& source,Fisheye624& proposal,int width,int height,
 size_t& sourceCount,size_t& angularExcluded,size_t& numericFailed,double& worst){
 sourceCount=angularExcluded=numericFailed=0;worst=0;bool valid=width>0&&height>0;
 for(int v=0;v<height;v+=16)for(int u=0;u<width;u+=16){
  cv::Point2f pixel(u,v);cv::Point3f oldRay,newRay,admitted;
  if(!source.tryUnproject(pixel,oldRay))continue;++sourceCount;
  if(!proposal.tryUnprojectGeometry(pixel,newRay)){++numericFailed;valid=false;continue;}
  double e=cv::norm(proposal.project(newRay)-pixel);worst=std::max(worst,e);
  if(!std::isfinite(e)||e>.01){++numericFailed;valid=false;}
  if(!proposal.tryUnproject(pixel,admitted))++angularExcluded;
 }
 for(int n=0;n<=200;++n){
  const double t=proposal.maximumSolidAngle()*n/200.,t2=t*t;double poly=1,derivative=1,power=t2;
  for(int k=0;k<6;++k){poly+=proposal.getParameter(4+k)*power;derivative+=(2*k+3)*proposal.getParameter(4+k)*power;power*=t2;}
  if(!(derivative>1e-4)||!std::isfinite(poly))valid=false;
  for(int a=0;a<48;++a){
   const double phi=2*M_PI*a/48.,x=t*poly*std::cos(phi),y=t*poly*std::sin(phi),rho=x*x+y*y;
   const double p0=proposal.getParameter(10),p1=proposal.getParameter(11),s0=proposal.getParameter(12),s1=proposal.getParameter(13),s2=proposal.getParameter(14),s3=proposal.getParameter(15);
   Eigen::Matrix2d J;J<<1+6*p0*x+2*p1*y+2*s0*x+4*s1*rho*x,2*p0*y+2*p1*x+2*s0*y+4*s1*rho*y,
     2*p1*x+2*p0*y+2*s2*x+4*s3*rho*x,1+6*p1*y+2*p0*x+2*s2*y+4*s3*rho*y;
   if(!J.allFinite()||J.determinant()<=1e-4)valid=false;
   const Eigen::Vector3d bearing(std::sin(t)*std::cos(phi),std::sin(t)*std::sin(phi),std::cos(t));
   const Eigen::Vector2d uv=proposal.project(bearing);const cv::Point2f pixel(uv.x(),uv.y());
   if(proposal.isInPhysicalMask(pixel)) {
    cv::Point3f inverse;
    if(!proposal.tryUnprojectGeometry(pixel,inverse)) {++numericFailed;valid=false;}
    else if((Eigen::Vector3d(inverse.x,inverse.y,inverse.z)-bearing).norm()>1e-4) {++numericFailed;valid=false;}
   }
  }
 }
 return valid&&sourceCount>0;
}

// Native Aria parameterization: one focal value per camera, cx/cy, six
// radial, two tangential and four prism coefficients. Source is a private
// copy; the fixed factory reference and limits persist across online updates.
class VertexFullCalibration : public g2o::BaseVertex<15,IntrinsicVector> {
public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 VertexFullCalibration(const Fisheye624& source,const std::vector<float>& factory)
 :model_(source) {
  if(factory.size()!=16 || std::abs(factory[0]-factory[1])>1e-4)
   throw std::invalid_argument("Expected native single-focal Fisheye624");
  base_[0]=factory[0];for(int i=1;i<15;++i)base_[i]=factory[i+1];
  bounds_ << .05*factory[0],4,4,.02,.01,.003,.001,.0003,.0001,.001,.001,.001,.001,.001,.001;
  step_=bounds_*.05;prior_=bounds_*.25;
  IntrinsicVector current;current[0]=model_.getParameter(0);
  for(int i=1;i<15;++i)current[i]=model_.getParameter(i+1);
  setDelta(current-base_);
 }
 bool read(std::istream&)override{return false;}bool write(std::ostream&)const override{return false;}
 void setToOriginImpl()override{_estimate.setZero();}
 void oplusImpl(const double* update)override{
  ORB_FINITE_GUARD(update,15);
  const IntrinsicVector before=delta();
  IntrinsicVector candidate=(bounds_.array()*(_estimate+Eigen::Map<const IntrinsicVector>(update)).array().tanh()).matrix();
  for(int i=0;i<15;++i)candidate[i]=std::max(-.999*bounds_[i],std::min(.999*bounds_[i],before[i]+std::max(-step_[i],std::min(step_[i],candidate[i]-before[i]))));
  setDelta(candidate);
 }
 IntrinsicVector delta()const{return (bounds_.array()*_estimate.array().tanh()).matrix();}
 IntrinsicVector derivative()const{return (bounds_.array()*(1-_estimate.array().tanh().square())).matrix();}
 const IntrinsicVector& bounds()const{return bounds_;}const IntrinsicVector& step()const{return step_;}const IntrinsicVector& prior()const{return prior_;}
 void setDelta(const IntrinsicVector& d){
  if(!d.allFinite() || (d.array().abs()>=bounds_.array()).any())throw std::invalid_argument("Intrinsic delta outside factory bounds");
  _estimate=(d.array()/bounds_.array()).unaryExpr([](double x){return std::atanh(x);}).matrix();
 }
 Fisheye624* camera()const{
  IntrinsicVector p=base_+delta();model_.setParameter(p[0],0);model_.setParameter(p[0],1);
  for(int i=1;i<15;++i)model_.setParameter(p[i],i+1);return &model_;
 }
 Eigen::Matrix<double,2,15> projectCalibrationJac(const Eigen::Vector3d& P)const{
  Fisheye624* c=camera();Eigen::Matrix<double,2,15> J=Eigen::Matrix<double,2,15>::Zero();
  J(0,1)=1;J(1,2)=1;const double r=std::hypot(P.x(),P.y());
  if(r<1e-12)return J*derivative().asDiagonal();
  const double t=std::atan2(r,P.z()),t2=t*t;double poly=1,power=t2;
  for(int i=0;i<6;++i){poly+=c->getParameter(4+i)*power;power*=t2;}
  const double x=t*poly*P.x()/r,y=t*poly*P.y()/r,rho=x*x+y*y;
  const double f=c->getParameter(0),p0=c->getParameter(10),p1=c->getParameter(11),s0=c->getParameter(12),s1=c->getParameter(13),s2=c->getParameter(14),s3=c->getParameter(15);
  J(0,0)=x+p0*(rho+2*x*x)+2*p1*x*y+s0*rho+s1*rho*rho;
  J(1,0)=y+p1*(rho+2*y*y)+2*p0*x*y+s2*rho+s3*rho*rho;
  Eigen::Matrix2d chain;
  chain << 1+6*p0*x+2*p1*y+2*s0*x+4*s1*rho*x,2*p0*y+2*p1*x+2*s0*y+4*s1*rho*y,
   2*p1*x+2*p0*y+2*s2*x+4*s3*rho*x,1+6*p1*y+2*p0*x+2*s2*y+4*s3*rho*y;
  power=t*t2;
  for(int k=0;k<6;++k){J.col(3+k)=f*chain*(power/r*P.head<2>());power*=t2;}
  J.col(9)=f*Eigen::Vector2d(rho+2*x*x,2*x*y);J.col(10)=f*Eigen::Vector2d(2*x*y,rho+2*y*y);
  J(0,11)=f*rho;J(0,12)=f*rho*rho;J(1,13)=f*rho;J(1,14)=f*rho*rho;
  return J*derivative().asDiagonal();
 }
private:mutable Fisheye624 model_;IntrinsicVector base_,bounds_,step_,prior_;
};
class EdgePriorFullCalibration:public g2o::BaseUnaryEdge<15,IntrinsicVector,VertexFullCalibration>{
public:EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 EdgePriorFullCalibration(){setMeasurement(IntrinsicVector::Zero());}
 bool read(std::istream&)override{return false;}bool write(std::ostream&)const override{return false;}
 void computeError()override{_error=static_cast<VertexFullCalibration*>(_vertices[0])->delta()-_measurement;}
 void linearizeOplus()override{_jacobianOplusXi=static_cast<VertexFullCalibration*>(_vertices[0])->derivative().asDiagonal();}
};
class EdgeMonoFullCalibration:public g2o::BaseMultiEdge<2,Eigen::Vector2d>{
public:EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 explicit EdgeMonoFullCalibration(int c):cameraIndex(c){resize(3);}
 bool read(std::istream&)override{return false;}bool write(std::ostream&)const override{return false;}
 void computeError()override{
  auto* point=static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);auto* pose=static_cast<VertexPose*>(_vertices[1]);auto* camera=static_cast<VertexFullCalibration*>(_vertices[2]);
  _error=_measurement-camera->camera()->project(Eigen::Vector3d(pose->estimate().Rcw[cameraIndex]*point->estimate()+pose->estimate().tcw[cameraIndex]));
 }
 bool isDepthPositive()const{return static_cast<const VertexPose*>(_vertices[1])->estimate().isDepthPositive(static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0])->estimate(),cameraIndex);}
 Eigen::Matrix<double,2,24> GetJacobian()const{
  auto* point=static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);auto* pose=static_cast<const VertexPose*>(_vertices[1]);auto* c=static_cast<const VertexFullCalibration*>(_vertices[2]);auto& p=pose->estimate();
  Eigen::Vector3d X=p.Rcw[cameraIndex]*point->estimate()+p.tcw[cameraIndex],b=p.Rbc[cameraIndex]*X+p.tbc[cameraIndex];
  auto J=c->camera()->projectJac(X);Eigen::Matrix<double,3,6> M;double x=b.x(),y=b.y(),z=b.z();M<<0,z,-y,1,0,0,-z,0,x,0,1,0,y,-x,0,0,0,1;
  Eigen::Matrix<double,2,24> out;out.block<2,3>(0,0)=-J*p.Rcw[cameraIndex];out.block<2,6>(0,3)=J*p.Rcb[cameraIndex]*M;out.block<2,15>(0,9)=-c->projectCalibrationJac(X);return out;
 }
 void linearizeOplus()override{const auto J=GetJacobian();_jacobianOplus[0]=J.block<2,3>(0,0);_jacobianOplus[1]=J.block<2,6>(0,3);_jacobianOplus[2]=J.block<2,15>(0,9);}
 const int cameraIndex;
};
}
#endif
