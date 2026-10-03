// Native dual-Fisheye624 global rig/visual-inertial BA. No GT inputs.
// COLMAP poses are camera/rig <- world, stored quaternion xyzw then t.
#include "imu.h"
#include "observation_metadata.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/sphere_manifold.h>
#include <opencv2/core.hpp>
#include "colmap/scene/reconstruction.h"
#include "colmap/sensor/models.h"
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <numeric>

namespace lb=lamaria_ba;
using lb::V3; using lb::M3; using lb::Bias; using lb::Preint;
using PoseManifold=ceres::ProductManifold<ceres::EigenQuaternionManifold,ceres::EuclideanManifold<3>>;
template<class T> using Vec=Eigen::Matrix<T,3,1>;
template<class T> using Quat=Eigen::Quaternion<T>;

struct CameraState {
  std::array<double,16> base;
  Eigen::Quaterniond qcr=Eigen::Quaterniond::Identity();
  V3 tcr=V3::Zero();
  std::array<double,15> z{}; // normalized deviations from immutable supplied calibration
  std::array<double,15> sigma{};
  explicit CameraState(const std::vector<double>& p) {
    if(p.size()!=16 || std::abs(p[0]-p[1])>1e-6) throw std::runtime_error("Expected native Fisheye624 with shared-axis focal");
    std::copy(p.begin(),p.end(),base.begin());
    sigma={p[0]*.01,1.,1., .005,.0025,.0012,.0006,.0003,.00015, .0005,.0005,.0005,.0002,.0005,.0002};
  }
  template<class T> void Params(const T* d,T* p) const {
    p[0]=T(base[0])+T(sigma[0])*d[0]; p[1]=p[0];
    for(int k=2;k<16;++k) p[k]=T(base[k])+T(sigma[k-1])*d[k-1];
  }
  std::array<double,16> Params() const { std::array<double,16> p; Params(z.data(),p.data());return p; }
};
struct VisualCost {
  VisualCost(const CameraState& camera,const Eigen::Vector2d& xy):camera(&camera),xy(xy){}
  template<class T> bool Project(const T* const pose,const T* const point,const T* const z,T* r) const {
    const Eigen::Map<const Quat<T>> qrw(pose); const Eigen::Map<const Vec<T>> trw(pose+4), X(point);
    const Vec<T> xc=camera->qcr.cast<T>()*(qrw*X+trw)+camera->tcr.cast<T>();
    T p[16],u,v;camera->Params(z,p);
    if(!colmap::RadTanThinPrismFisheyeModel::ImgFromCam(p,xc.x(),xc.y(),xc.z(),&u,&v)) return false;
    r[0]=u-T(xy.x());r[1]=v-T(xy.y());return true;
  }
  template<class T> bool operator()(const T* const pose,const T* const point,const T* const z,T* r) const {
    // Match COLMAP's production Reproj/RigReproj cost contract: a trial point
    // outside the camera's projectable half-space contributes zero, rather
    // than vetoing a global step with millions of other valid observations.
    // Initial and final projectability counts are reported separately.
    if(!Project(pose,point,z,r))r[0]=r[1]=T(0);
    return true;
  }
  const CameraState* camera;Eigen::Vector2d xy;
};
struct FixedVisualCost : VisualCost {
  using VisualCost::VisualCost;
  template<class T> bool operator()(const T* const pose,const T* const point,T* r) const {
    T z[15];for(int i=0;i<15;++i)z[i]=T(camera->z[i]);
    return VisualCost::operator()(pose,point,z,r);
  }
};
struct CalibrationPrior {
  template<class T> bool operator()(const T* const z,T* r) const {for(int i=0;i<15;++i) r[i]=z[i];return true;}
};
struct BiasPrior {
  template<class T> bool operator()(const T* const b,T* r) const {for(int i=0;i<6;++i)r[i]=b[i]/T(i<3?.03:.3);return true;}
};
struct BiasWalk {
  BiasWalk(double dt,double gw,double aw):gw(gw*std::sqrt(dt)),aw(aw*std::sqrt(dt)){}
  template<class T> bool operator()(const T* const a,const T* const b,T* r) const {
    for(int i=0;i<6;++i)r[i]=(b[i]-a[i])/T(i<3?gw:aw);return true;
  }
  double gw,aw;
};
template<class T> void BodyPose(const T* pose,const Eigen::Quaterniond& qbc,const V3& tbc,Quat<T>& qwb,Vec<T>& pwb) {
  const Eigen::Map<const Quat<T>> qcw(pose);const Eigen::Map<const Vec<T>> tcw(pose+4);
  qwb=(qbc.cast<T>()*qcw).conjugate();pwb=-(qwb*(qbc.cast<T>()*tcw+tbc.cast<T>()));
}
template<class T> Quat<T> ExpT(const Vec<T>& v) {
  // Ceres handles exactly zero angle and Jet derivatives.
  T out[4];ceres::AngleAxisToQuaternion(v.data(),out);return Quat<T>(out[0],out[1],out[2],out[3]);
}
struct ImuCost {
  ImuCost(const Preint* pre,Eigen::Quaterniond qbc,V3 tbc):pre(pre),qbc(qbc),tbc(tbc){}
  template<class T> bool operator()(const T* const pi,const T* const pj,const T* const vi,const T* const vj,const T* const bi,const T* const gravity,T* out) const {
    Quat<T> Ri,Rj;Vec<T> Xi,Xj;BodyPose(pi,qbc,tbc,Ri,Xi);BodyPose(pj,qbc,tbc,Rj,Xj);
    const Eigen::Map<const Vec<T>> Vi(vi),Vj(vj),g(gravity);
    const Eigen::Map<const Eigen::Matrix<T,6,1>> b(bi);
    const Eigen::Matrix<T,6,1> db=b-pre->bias.cast<T>();
    const Eigen::Matrix<T,9,1> correction=pre->J.cast<T>()*db;
    const Quat<T> dq=pre->q.cast<T>()*ExpT(Vec<T>(correction.template tail<3>()));
    Quat<T> er=dq.conjugate()*(Ri.conjugate()*Rj);
    if(er.w()<T(0)) er.coeffs()*=T(-1);
    T ewxyz[4]={er.w(),er.x(),er.y(),er.z()},aa[3];ceres::QuaternionToAngleAxis(ewxyz,aa);
    Eigen::Matrix<T,9,1> e;
    const T d=T(pre->dt);
    e.template segment<3>(0)=Ri.conjugate()*(Xj-Xi-Vi*d-T(.5)*g*d*d)-pre->p.cast<T>()-correction.template segment<3>(0);
    e.template segment<3>(3)=Ri.conjugate()*(Vj-Vi-g*d)-pre->v.cast<T>()-correction.template segment<3>(3);
    for(int k=0;k<3;++k)e[6+k]=aa[k];
    Eigen::Map<Eigen::Matrix<T,9,1>> r(out);r=pre->W.cast<T>()*e;return true;
  }
  const Preint* pre;Eigen::Quaterniond qbc;V3 tbc;
};
struct State {
  int64_t ns;colmap::frame_t fid;double* pose;
  std::array<double,3> velocity{};
  std::array<double,6> bias{};
};
struct Interval { std::vector<lb::Segment> data;Preint pre; };
inline int64_t Timestamp(const std::string& s){return std::stoll(std::filesystem::path(s).stem().string());}
inline void Require(bool ok,const std::string& msg){if(!ok)throw std::runtime_error(msg);}
bool CameraDomain(const CameraState& c) {
  const auto p=c.Params();
  for(int j=0;j<=14;++j) for(int i=0;i<24;++i) {
    const double th=.1*j,a=2*M_PI*i/24,x=std::sin(th)*std::cos(a),y=std::sin(th)*std::sin(a),z=std::cos(th);
    double u,v,xn,yn;
    if(!colmap::RadTanThinPrismFisheyeModel::ImgFromCam(p.data(),x,y,z,&u,&v) ||
       !colmap::RadTanThinPrismFisheyeModel::CamFromImg(p.data(),u,v,&xn,&yn) ||
       !std::isfinite(xn+yn) || std::hypot(xn-x/z,yn-y/z)>1e-5) return false;
  }
  return true;
}
cv::Mat ReadMatrix(cv::FileStorage& fs,const std::string& key) {
  cv::Mat m;fs[key]>>m;Require(m.rows==4&&m.cols==4,"Missing matrix "+key);m.convertTo(m,CV_64F);return m;
}
colmap::Rigid3d Transform(const cv::Mat& m) {
  M3 R;V3 t;for(int i=0;i<3;++i){t[i]=m.at<double>(i,3);for(int j=0;j<3;++j)R(i,j)=m.at<double>(i,j);}
  return colmap::Rigid3d(Eigen::Quaterniond(R).normalized(),t);
}

int main(int argc,char** argv) try {
  std::map<std::string,std::string> args;
  for(int i=1;i<argc;++i){std::string k=argv[i];Require(k.rfind("--",0)==0 && i+1<argc,"Usage: --input_path MODEL --output_path OUT --imu_csv CSV --settings_yaml YAML --mode visual_rig|vi_fixed|vi_calib [--max_iterations 30] [--max_threads 4]");args[k]=argv[++i];}
  for(const auto& k:{"--input_path","--output_path","--settings_yaml","--mode"})Require(args.count(k),std::string("Missing ")+k);
  const std::string mode=args["--mode"],output=args["--output_path"];
  const bool vi=mode!="visual_rig",calibrate=mode=="vi_calib";
  Require(mode=="visual_rig"||mode=="vi_fixed"||mode=="vi_calib","Invalid mode");
  const int iterations=args.count("--max_iterations")?std::stoi(args["--max_iterations"]):30;
  const int threads=args.count("--max_threads")?std::stoi(args["--max_threads"]):4;
  const std::string linear=args.count("--linear_solver")?args["--linear_solver"]:"iterative_schur";
  const bool explicit_schur=args.count("--explicit_schur") && std::stoi(args["--explicit_schur"])!=0;
  const bool release_metadata=args.count("--release_observation_metadata") && std::stoi(args["--release_observation_metadata"])!=0;
  const int max_linear=args.count("--max_linear_iterations")?std::stoi(args["--max_linear_iterations"]):1000;
  const double eta=args.count("--linear_eta")?std::stod(args["--linear_eta"]):.001;
  Require(linear=="iterative_schur" || linear=="sparse_schur","Unknown linear solver");
  Require(!explicit_schur || linear=="iterative_schur","Explicit Schur applies to the iterative Schur solver");
  Require(max_linear>0 && eta>0 && eta<1,"Invalid iterative linear solve limits");
  Require(iterations>0 && threads>0,"Bad solve limits");
  Require(!std::filesystem::exists(std::filesystem::path(output)/"result.json"),"Output result already exists");std::filesystem::create_directories(output);
  cv::FileStorage fs(args["--settings_yaml"],cv::FileStorage::READ);Require(fs.isOpened(),"Cannot read settings");
  const auto Tbc=Transform(ReadMatrix(fs,"IMU.T_b_c1"));
  const auto Tc0c1=Transform(ReadMatrix(fs,"Rig.T_c0_c1"));
  const double ng=double(fs["IMU.NoiseGyro"]),na=double(fs["IMU.NoiseAcc"]),gw=double(fs["IMU.GyroWalk"]),aw=double(fs["IMU.AccWalk"]);
  Require(std::min({ng,na,gw,aw})>0,"Invalid IMU noise density");
  colmap::Reconstruction rec;rec.Read(args["--input_path"]);
  Require(rec.NumRegImages()>2 && rec.NumPoints3D()>20,"Insufficient reconstruction");
  std::map<int64_t,State> sorted;std::map<colmap::camera_t,CameraState> cameras;std::map<colmap::camera_t,int> camindex;
  std::map<int64_t,std::set<int>> frame_cameras;
  for(auto iid:rec.RegImageIds()) {
    auto& im=rec.Image(iid);const auto ns=Timestamp(im.Name());const bool cam0=im.Name().rfind("cam0/",0)==0;
    Require(cam0 || im.Name().rfind("cam1/",0)==0,"Image name must be cam0/<native_ns>.* or cam1/<native_ns>.*");
    Require(im.IsRefInFrame()==cam0,"COLMAP rig reference must be native cam0");
    auto& fr=*im.FramePtr();
    const auto Tcr=im.CamFromWorld()*colmap::Inverse(fr.RigFromWorld());
    const auto expected=cam0?colmap::Rigid3d():colmap::Inverse(Tc0c1);
    Require(lb::Log(Tcr.rotation()*expected.rotation().conjugate()).norm()<1e-5 && (Tcr.translation()-expected.translation()).norm()<1e-5,"COLMAP rig differs from supplied metric calibration");
    if(!sorted.count(ns))sorted.emplace(ns,State{ns,im.FrameId(),fr.RigFromWorld().params.data()});
    else Require(sorted.at(ns).fid==im.FrameId(),"Simultaneous cameras must share one rig frame");
    frame_cameras[ns].insert(cam0?0:1);camindex[im.CameraId()]=cam0?0:1;
    if(!cameras.count(im.CameraId())) {
      const auto& c=rec.Camera(im.CameraId());Require(c.model_id==colmap::CameraModelId::kRadTanThinPrismFisheye,"Exact native RAD_TAN_THIN_PRISM_FISHEYE required");
      cameras.emplace(im.CameraId(),CameraState(c.params));
      cameras.at(im.CameraId()).qcr=Tcr.rotation();cameras.at(im.CameraId()).tcr=Tcr.translation();
    }
  }
  Require(cameras.size()==2,"Exactly two shared native camera calibrations required");
  std::vector<State*> states;for(auto& [t,s]:sorted){Require(frame_cameras.at(t).size()==2,"Missing synchronized camera image");states.push_back(&s);}
  const size_t n=states.size();std::vector<V3> positions(n);std::vector<Eigen::Quaterniond> rotations(n);
  for(size_t i=0;i<n;++i)BodyPose(states[i]->pose,Tbc.rotation(),Tbc.translation(),rotations[i],positions[i]);
  for(size_t i=0;i<n;++i){size_t a=i?i-1:i,b=i+1<n?i+1:i;const V3 v=(positions[b]-positions[a])/(double(states[b]->ns-states[a]->ns)*1e-9);Eigen::Map<V3>(states[i]->velocity.data())=v;}
  std::array<double,3> gravity{0,0,-9.81};
  std::vector<Interval> intervals;std::vector<lb::Sample> imu;
  if(vi) {
    Require(args.count("--imu_csv"),"VI requires calibrated native IMU CSV");imu=lb::LoadImu(args["--imu_csv"]);intervals.reserve(n-1);
    M3 H=M3::Identity()*1e-6;V3 rhs=V3::Zero();
    for(size_t i=0;i+1<n;++i) {
      Interval f;f.data=lb::Slice(imu,states[i]->ns,states[i+1]->ns);f.pre=lb::Integrate(f.data,Bias::Zero(),ng,na);
      const V3 err=lb::Log(f.pre.q.conjugate()*(rotations[i].conjugate()*rotations[i+1]));const M3 J=f.pre.J.block<3,3>(6,0);
      if(err.norm()<.2){H+=J.transpose()*J;rhs+=J.transpose()*err;}
      intervals.push_back(std::move(f));
    }
    const V3 bg=H.ldlt().solve(rhs);Require(bg.allFinite(),"Invalid gyro initialization");
    for(auto* s:states)for(int k=0;k<3;++k)s->bias[k]=std::clamp(bg[k],-.05,.05);
    for(size_t i=0;i+1<n;++i)intervals[i].pre=lb::Integrate(intervals[i].data,Eigen::Map<Bias>(states[i]->bias.data()),ng,na);
    V3 force=V3::Zero();double time=0;
    for(size_t i=0;i+1<n;++i){force+=rotations[i]*intervals[i].pre.v;time+=intervals[i].pre.dt;}
    Require(force.norm()>time*5,"Gravity initialization lacks a valid specific-force signal");
    Eigen::Map<V3>(gravity.data())=-9.81*force.normalized();
    std::cout<<"IMU intervals="<<intervals.size()<<" native_samples="<<imu.size()<<" initialized_gyro_bias="<<bg.transpose()<<std::endl;
  }
  ceres::Problem problem;
  auto ordering=std::make_shared<ceres::ParameterBlockOrdering>();
  for(auto* s:states){problem.AddParameterBlock(s->pose,7,new PoseManifold);ordering->AddElementToGroup(s->pose,1);}
  // Fixed metric baseline provides scale. Only the first rig pose fixes world gauge.
  problem.SetParameterBlockConstant(states.front()->pose);
  for(auto& [cid,c]:cameras){problem.AddParameterBlock(c.z.data(),15);ordering->AddElementToGroup(c.z.data(),2);
    if(!calibrate)problem.SetParameterBlockConstant(c.z.data());
    else problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CalibrationPrior,15,15>(new CalibrationPrior),nullptr,c.z.data());}
  size_t observations=0,invalid=0,crosscamera=0;std::vector<ceres::ResidualBlockId> visual_ids,imu_ids;
  // One immutable robust loss and one extrinsic per physical camera. Ceres
  // owns each distinct pointer once, even when millions of residuals share it.
  auto* visual_loss=new ceres::HuberLoss(2.);
  std::map<colmap::frame_t,std::array<size_t,2>> support;
  for(auto pid:rec.Point3DIds()) {
    auto& p=rec.Point3D(pid);std::set<int> seen;
    for(const auto& el:p.track.Elements()) {
      auto& im=rec.Image(el.image_id);if(!im.HasPose())continue;
      auto& fr=*im.FramePtr();const auto xy=im.Point2D(el.point2D_idx).xy;
      auto& c=cameras.at(im.CameraId());VisualCost probe(c,xy);double r[2];
      if(!probe.Project(fr.RigFromWorld().params.data(),p.xyz.data(),c.z.data(),r))++invalid;
      else Require(std::isfinite(r[0]+r[1]),"Nonfinite initial visual observation");
      if(calibrate)visual_ids.push_back(problem.AddResidualBlock(new ceres::AutoDiffCostFunction<VisualCost,2,7,3,15>(new VisualCost(c,xy)),visual_loss,fr.RigFromWorld().params.data(),p.xyz.data(),c.z.data()));
      else visual_ids.push_back(problem.AddResidualBlock(new ceres::AutoDiffCostFunction<FixedVisualCost,2,7,3>(new FixedVisualCost(c,xy)),visual_loss,fr.RigFromWorld().params.data(),p.xyz.data()));
      ++observations;seen.insert(camindex.at(im.CameraId()));
      ++support[im.FrameId()][camindex.at(im.CameraId())];
    }
    if(problem.HasParameterBlock(p.xyz.data()))ordering->AddElementToGroup(p.xyz.data(),0);
    if(seen.size()==2)++crosscamera;
  }
  Require(crosscamera>=20,"Fewer than 20 cross-camera landmarks: visual rig scale is insufficiently constrained");
  size_t zero_visual_frames=0;
  for(auto* s:states)if(support[s->fid][0]+support[s->fid][1]==0){++zero_visual_frames;if(!vi)problem.SetParameterBlockConstant(s->pose);}
  if(vi) {
    problem.AddParameterBlock(gravity.data(),3,new ceres::SphereManifold<3>);ordering->AddElementToGroup(gravity.data(),2);
    for(auto* s:states){problem.AddParameterBlock(s->velocity.data(),3);problem.AddParameterBlock(s->bias.data(),6);ordering->AddElementToGroup(s->velocity.data(),1);ordering->AddElementToGroup(s->bias.data(),1);}
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<BiasPrior,6,6>(new BiasPrior),nullptr,states.front()->bias.data());
    for(size_t i=0;i+1<n;++i){auto& f=intervals[i];
      imu_ids.push_back(problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ImuCost,9,7,7,3,3,6,3>(new ImuCost(&f.pre,Tbc.rotation(),Tbc.translation())),nullptr,states[i]->pose,states[i+1]->pose,states[i]->velocity.data(),states[i+1]->velocity.data(),states[i]->bias.data(),gravity.data()));
      problem.AddResidualBlock(new ceres::AutoDiffCostFunction<BiasWalk,6,6,6>(new BiasWalk(f.pre.dt,gw,aw)),nullptr,states[i]->bias.data(),states[i+1]->bias.data());
    }
  }
  std::cout<<"GLOBAL_BA mode="<<mode<<" frames="<<n<<" points="<<rec.NumPoints3D()<<" observations="<<observations<<" invalid="<<invalid<<" cross_camera_points="<<crosscamera<<std::endl;
  std::unique_ptr<lb::ObservationMetadata> metadata;
  if(release_metadata) {
    metadata=std::make_unique<lb::ObservationMetadata>(rec,std::filesystem::path(output)/"observation_metadata.cache");
    std::cout<<"METADATA_RELEASE images="<<metadata->image_count<<" points="<<metadata->point_count
      <<" features="<<metadata->feature_count<<" track_entries="<<metadata->track_count
      <<" released_capacity_bytes="<<metadata->released_capacity_bytes<<" sha256="<<metadata->digest<<std::endl;
  }
  ceres::Solver::Options options;options.num_threads=threads;options.linear_solver_type=linear=="iterative_schur"?ceres::ITERATIVE_SCHUR:ceres::SPARSE_SCHUR;options.preconditioner_type=ceres::SCHUR_JACOBI;options.max_linear_solver_iterations=max_linear;options.eta=eta;options.sparse_linear_algebra_library_type=ceres::SUITE_SPARSE;options.linear_solver_ordering=ordering;
  options.use_explicit_schur_complement=explicit_schur;
  options.max_num_iterations=1;options.minimizer_progress_to_stdout=true;options.function_tolerance=1e-7;options.gradient_tolerance=1e-10;options.parameter_tolerance=1e-9;
  auto Cost=[&](const std::vector<ceres::ResidualBlockId>& blocks){ceres::Problem::EvaluateOptions e;e.num_threads=threads;e.residual_blocks=blocks;double cost=0;Require(problem.Evaluate(e,&cost,nullptr,nullptr,nullptr),"Cost evaluation failed");return cost;};
  const double initial_visual=Cost(visual_ids),initial_imu=vi?Cost(imu_ids):0;
  // Initial state adaptation leaves image geometry untouched, avoiding a poor
  // finite-difference velocity initializer pulling the map during the first step.
  if(vi){
    for(auto* s:states)problem.SetParameterBlockConstant(s->pose);
    for(auto pid:rec.Point3DIds())if(problem.HasParameterBlock(rec.Point3D(pid).xyz.data()))problem.SetParameterBlockConstant(rec.Point3D(pid).xyz.data());
    for(auto& [cid,c]:cameras)problem.SetParameterBlockConstant(c.z.data());
    ceres::Solver::Options warm=options;warm.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;warm.linear_solver_ordering.reset();warm.max_num_iterations=8;
    warm.use_explicit_schur_complement=false;
    ceres::Solver::Summary summary;ceres::Solve(warm,&problem,&summary);Require(summary.IsSolutionUsable(),"Inertial state initialization failed");std::cout<<"STATE_INITIALIZATION "<<summary.BriefReport()<<std::endl;
    for(auto* s:states)problem.SetParameterBlockVariable(s->pose);problem.SetParameterBlockConstant(states.front()->pose);
    for(auto pid:rec.Point3DIds())if(problem.HasParameterBlock(rec.Point3D(pid).xyz.data()))problem.SetParameterBlockVariable(rec.Point3D(pid).xyz.data());
    if(calibrate)for(auto& [cid,c]:cameras)problem.SetParameterBlockVariable(c.z.data());
  }
  std::ofstream history(std::filesystem::path(output)/"optimization.csv");history<<"iteration,cost,visual_cost,imu_cost,reintegration_count,max_intrinsic_step_sigma,accepted,linear_iterations,successful_steps,unsuccessful_steps\n";
  double finalcost=0,firstcost=0;int accepted=0,reintegration_total=0,last_linear_iterations=0,linear_capped_steps=0;bool usable=true,converged=false;
  std::vector<double*> blocks;problem.GetParameterBlocks(&blocks);
  for(int iteration=0;iteration<iterations;++iteration) {
    int reintegrated=0;
    if(vi)for(size_t i=0;i+1<n;++i){const Bias b=Eigen::Map<Bias>(states[i]->bias.data());const Bias db=b-intervals[i].pre.bias;
      if(db.head<3>().norm()>.002 || db.tail<3>().norm()>.02 || iteration==0){intervals[i].pre=lb::Integrate(intervals[i].data,b,ng,na);++reintegrated;}}
    reintegration_total+=reintegrated;
    std::map<colmap::camera_t,std::array<double,15>> old_cameras;
    for(auto& [cid,c]:cameras){old_cameras[cid]=c.z;if(calibrate)for(int k=0;k<15;++k){problem.SetParameterLowerBound(c.z.data(),k,std::max(-3.,c.z[k]-.15));problem.SetParameterUpperBound(c.z.data(),k,std::min(3.,c.z[k]+.15));}}
    std::vector<std::vector<double>> backup;if(calibrate)for(auto* b:blocks)backup.emplace_back(b,b+problem.ParameterBlockSize(b));
    ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);
    if(iteration==0)firstcost=summary.initial_cost;finalcost=summary.final_cost;usable=summary.IsSolutionUsable();
    double maxstep=0;bool domain=true;for(auto& [cid,c]:cameras){for(int k=0;k<15;++k)maxstep=std::max(maxstep,std::abs(c.z[k]-old_cameras[cid][k]));if(calibrate)domain&=CameraDomain(c);}
    Require(maxstep<=.150000001,"Per-step calibration bound violated");
    const bool good=usable&&domain&&std::isfinite(finalcost)&&finalcost<=summary.initial_cost+1e-7;
    const bool step_accepted=good&&std::any_of(summary.iterations.begin(),summary.iterations.end(),[](const ceres::IterationSummary& it){return it.iteration>0&&it.step_is_successful;});
    if(!good&&calibrate){for(size_t i=0;i<blocks.size();++i)std::copy(backup[i].begin(),backup[i].end(),blocks[i]);finalcost=summary.initial_cost;}
    int linear_iterations=0;for(const auto& it:summary.iterations)linear_iterations+=it.linear_solver_iterations;
    last_linear_iterations=linear_iterations;if(linear=="iterative_schur" && linear_iterations>=max_linear)++linear_capped_steps;
    history<<std::setprecision(17)<<iteration<<','<<finalcost<<','<<Cost(visual_ids)<<','<<(vi?Cost(imu_ids):0)<<','<<reintegrated<<','<<maxstep<<','<<step_accepted<<','<<linear_iterations<<','<<summary.num_successful_steps<<','<<summary.num_unsuccessful_steps<<'\n';history.flush();
    std::cout<<"GLOBAL_ITERATION "<<iteration<<" valid_state="<<good<<" accepted_step="<<step_accepted<<" calibrated_domain="<<domain<<" max_intrinsic_step_sigma="<<maxstep<<" "<<summary.BriefReport()<<std::endl;
    if(!good){usable=false;break;}if(step_accepted)++accepted;
    if(!summary.iterations.empty())options.initial_trust_region_radius=summary.iterations.back().trust_region_radius;
    if(summary.termination_type==ceres::CONVERGENCE){converged=true;break;}
  }
  // Evaluate objective before pruning: Ceres parameter pointers refer to the
  // reconstruction's landmark storage, which pruning may destroy.
  const double final_visual_cost=Cost(visual_ids),final_imu_cost=vi?Cost(imu_ids):0;
  // This is an audit, not another optimizer step. The final accepted update
  // can move bias beyond the last preintegration linearization point.
  double final_max_gyro_bias_displacement=0,final_max_accel_bias_displacement=0;
  if(vi)for(size_t i=0;i+1<n;++i){
    const Bias b=Eigen::Map<Bias>(states[i]->bias.data()),db=b-intervals[i].pre.bias;
    final_max_gyro_bias_displacement=std::max(final_max_gyro_bias_displacement,db.head<3>().norm());
    final_max_accel_bias_displacement=std::max(final_max_accel_bias_displacement,db.tail<3>().norm());
    intervals[i].pre=lb::Integrate(intervals[i].data,b,ng,na);
  }
  const double final_imu_exact_cost=vi?Cost(imu_ids):0,final_exact_cost=Cost({});
  const double final_imu_cost_relative_shift=std::abs(final_imu_exact_cost-final_imu_cost)/std::max(1.,final_imu_cost);
  if(vi)std::cout<<"FINAL_IMU_AUDIT max_gyro_bias_linearization_displacement="<<final_max_gyro_bias_displacement
    <<" max_accel_bias_linearization_displacement="<<final_max_accel_bias_displacement
    <<" linearized_cost="<<final_imu_cost<<" exact_reintegrated_cost="<<final_imu_exact_cost
    <<" relative_shift="<<final_imu_cost_relative_shift<<std::endl;
  if(metadata) {
    metadata->Restore(rec);
    std::ofstream audit(std::filesystem::path(output)/"observation_metadata.json");
    audit<<"{\"restored\":true,\"images\":"<<metadata->image_count<<",\"points\":"<<metadata->point_count
      <<",\"features\":"<<metadata->feature_count<<",\"track_entries\":"<<metadata->track_count
      <<",\"released_capacity_bytes\":"<<metadata->released_capacity_bytes<<",\"sha256\":\""<<metadata->digest
      <<"\",\"restored_memory_sha256\":\""<<metadata->restored_digest<<"\"}\n";
    std::cout<<"METADATA_RESTORE exact_in_memory_digest=1 sha256="<<metadata->restored_digest<<std::endl;
  }
  const size_t points_before_filter=rec.NumPoints3D();
  const auto original_support=support;
  size_t unprojectable_final=0;double final_pixel_squared=0;size_t projectable_final=0;
  std::vector<std::pair<colmap::image_t,colmap::point2D_t>> invalid_final_observations;
  for(auto pid:rec.Point3DIds()) {
    auto& p=rec.Point3D(pid);double sum=0;size_t count=0;
    for(const auto& el:p.track.Elements()) {
      auto& im=rec.Image(el.image_id);if(!im.HasPose())continue;
      VisualCost cost(cameras.at(im.CameraId()),im.Point2D(el.point2D_idx).xy);double residual[2];
      if(!cost.Project(im.FramePtr()->RigFromWorld().params.data(),p.xyz.data(),cameras.at(im.CameraId()).z.data(),residual) || !std::isfinite(residual[0]+residual[1])){++unprojectable_final;invalid_final_observations.emplace_back(el.image_id,el.point2D_idx);continue;}
      const double squared=residual[0]*residual[0]+residual[1]*residual[1];sum+=std::sqrt(squared);++count;final_pixel_squared+=squared;++projectable_final;
    }
    p.error=count?sum/count:-1;
  }
  // Standard COLMAP observation deletion also removes a landmark if fewer
  // than two observations remain. Invalid observations never become green
  // associations in exported images. No measurements are removed pre-solve.
  for(const auto& [iid,index]:invalid_final_observations)
    if(rec.Image(iid).Point2D(index).HasPoint3D())rec.DeleteObservation(iid,index);
  support.clear();size_t final_observations=0;zero_visual_frames=0;
  for(auto pid:rec.Point3DIds())for(const auto& el:rec.Point3D(pid).track.Elements()){
    const auto& im=rec.Image(el.image_id);++support[im.FrameId()][camindex.at(im.CameraId())];++final_observations;
  }
  for(auto* s:states)if(support[s->fid][0]+support[s->fid][1]==0)++zero_visual_frames;
  if(unprojectable_final>std::max(size_t(100),observations/1000)){
    usable=false;std::cerr<<"FINAL_GEOMETRY_REJECT excessive_nonprojectable_observations="<<unprojectable_final<<" threshold="<<std::max(size_t(100),observations/1000)<<std::endl;
  }
  std::cout<<"FINAL_GEOMETRY input_observations="<<observations<<" retained="<<final_observations<<" unprojectable="<<unprojectable_final<<" removed_landmarks="<<points_before_filter-rec.NumPoints3D()<<std::endl;
  for(auto& [cid,c]:cameras){auto p=c.Params();rec.Camera(cid).params.assign(p.begin(),p.end());}
  rec.WriteText(output);
  std::ofstream statesfile(std::filesystem::path(output)/"imu_states.csv");statesfile<<"timestamp_ns,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n"<<std::setprecision(17);
  for(auto* s:states){statesfile<<s->ns;for(double x:s->velocity)statesfile<<','<<x;for(double x:s->bias)statesfile<<','<<x;statesfile<<'\n';}
  std::ofstream bodyfile(std::filesystem::path(output)/"body_trajectory_ns.txt");bodyfile<<"# timestamp_ns px py pz qx qy qz qw; world_from_body; native timestamps, no GT alignment\n"<<std::setprecision(17);
  for(auto* s:states){Eigen::Quaterniond R;V3 p;BodyPose(s->pose,Tbc.rotation(),Tbc.translation(),R,p);bodyfile<<s->ns<<' '<<p.transpose()<<' '<<R.coeffs().transpose()<<'\n';}
  std::ofstream supportfile(std::filesystem::path(output)/"frame_support.csv");supportfile<<"timestamp_ns,frame_id,cam0_observations,cam1_observations,pose_support\n";
  for(auto* s:states){auto count=support[s->fid];const auto original=original_support.count(s->fid)?original_support.at(s->fid):std::array<size_t,2>{};supportfile<<s->ns<<','<<s->fid<<','<<count[0]<<','<<count[1]<<','<<(count[0]+count[1]?(vi?"visual_imu":"visual"):(vi?"imu_only":(original[0]+original[1]?"visual_constraints_filtered":"unchanged_input")))<<'\n';}
  std::ofstream calibration(std::filesystem::path(output)/"calibration.json");calibration<<std::setprecision(17)<<"{\"prior_sigma_units\": \"physical per native parameter\", \"max_step_sigma\":0.15,\"max_total_sigma\":3,\"cameras\":[";bool first=true;
  for(auto& [cid,c]:cameras){if(!first)calibration<<',';first=false;calibration<<"{\"camera_id\":"<<cid<<",\"cam_index\":"<<camindex.at(cid)<<",\"initial\":[";for(int i=0;i<16;++i){if(i)calibration<<',';calibration<<c.base[i];}calibration<<"],\"final\":[";auto p=c.Params();for(int i=0;i<16;++i){if(i)calibration<<',';calibration<<p[i];}calibration<<"],\"sigma_15\":[";for(int i=0;i<15;++i){if(i)calibration<<',';calibration<<c.sigma[i];}calibration<<"]}";}calibration<<"]}\n";
  std::ofstream result(std::filesystem::path(output)/"result.json");result<<std::setprecision(17)<<"{\"mode\":\""<<mode<<"\",\"usable\":"<<(usable?"true":"false")<<",\"frames\":"<<n<<",\"zero_visual_frames\":"<<zero_visual_frames<<",\"linear_solver\":\""<<linear<<"\",\"explicit_schur\":"<<(explicit_schur?"true":"false")<<",\"initial_cost_stage\":\"after_inertial_state_warmup\",\"images\":"<<rec.NumRegImages()<<",\"points\":"<<rec.NumPoints3D()<<",\"observations\":"<<final_observations<<",\"input_observations\":"<<observations<<",\"dropped_observations\":"<<observations-final_observations<<",\"points_before_filter\":"<<points_before_filter<<",\"dropped_landmarks\":"<<points_before_filter-rec.NumPoints3D()<<",\"invalid_observations\":"<<invalid<<",\"unprojectable_final_observations\":"<<unprojectable_final<<",\"projectable_final_observations\":"<<projectable_final<<",\"final_projectable_pixel_rms\":"<<std::sqrt(final_pixel_squared/std::max(size_t(1),projectable_final))<<",\"cross_camera_points\":"<<crosscamera<<",\"imu_intervals\":"<<intervals.size()<<",\"imu_interval_coverage\":"<<(vi?1:0)<<",\"accepted_iterations\":"<<accepted<<",\"converged\":"<<(converged?"true":"false")<<",\"linear_eta\":"<<eta<<",\"max_linear_iterations\":"<<max_linear<<",\"last_linear_iterations\":"<<last_linear_iterations<<",\"linear_capped_steps\":"<<linear_capped_steps<<",\"reintegration_count\":"<<reintegration_total<<",\"initial_cost\":"<<firstcost<<",\"final_cost\":"<<finalcost<<",\"initial_visual_cost\":"<<initial_visual<<",\"final_visual_cost\":"<<final_visual_cost<<",\"initial_imu_cost\":"<<initial_imu<<",\"final_imu_cost\":"<<final_imu_cost<<",\"final_imu_linearized_cost\":"<<final_imu_cost<<",\"final_imu_exact_cost\":"<<final_imu_exact_cost<<",\"final_exact_cost\":"<<final_exact_cost<<",\"final_imu_cost_relative_shift\":"<<final_imu_cost_relative_shift<<",\"final_max_gyro_bias_linearization_displacement\":"<<final_max_gyro_bias_displacement<<",\"final_max_accel_bias_linearization_displacement\":"<<final_max_accel_bias_displacement<<",\"final_exact_reintegrations\":"<<(vi?intervals.size():0)<<",\"gravity\":["<<gravity[0]<<','<<gravity[1]<<','<<gravity[2]<<"],\"metric_rig_baseline\":"<<Tc0c1.translation().norm()<<",\"timestamp_start_ns\":"<<states.front()->ns<<",\"timestamp_end_ns\":"<<states.back()->ns<<"}\n";
  return usable?0:2;
} catch(const std::exception& e){std::cerr<<"LAMARIA_BA_ERROR "<<e.what()<<std::endl;return 1;}
