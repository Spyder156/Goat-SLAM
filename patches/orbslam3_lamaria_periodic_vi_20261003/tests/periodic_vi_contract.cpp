// Exercise the actual compiled FullCalibrationBA on a synthetic metric rig.
// System is constructed normally; test-only public visibility exposes its
// tracker after both workers are shut down. No recorded images/GT are used.
#include "System.h"
#include "Optimizer.h"
#include "G2oTypes.h"
#include "CameraModels/Fisheye624.h"
#include "ORBextractor.h"
#include <iostream>
#include <memory>
#include <cmath>
#include <fstream>
using namespace ORB_SLAM3;
int failures=0;
void check(bool ok,const char* text){std::cout<<"CHECK "<<(ok?"PASS ":"FAIL ")<<text<<std::endl;failures+=!ok;}
ConstraintPoseImu* prior(){return new ConstraintPoseImu(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Matrix15d::Identity());}
int main(int argc,char** argv){
 if(argc!=3)return 2;cv::setNumThreads(1);
 {std::ofstream voc(argv[2]);voc<<"2 1 0 0\n";for(int leaf=0;leaf<2;++leaf){voc<<"0 1 ";for(int j=0;j<32;++j)voc<<(leaf?255:0)<<' ';voc<<"1\n";}}
 System system(argv[2],argv[1],System::IMU_STEREO,false);system.Shutdown();
 Tracking* tracker=system.mpTracker;Map* map=system.mpAtlas->GetCurrentMap();
 map->SetInertialSensor();map->SetImuInitialized();map->SetIniertialBA2();
 Fisheye624* cam[2]={dynamic_cast<Fisheye624*>(system.settings_->camera1()),dynamic_cast<Fisheye624*>(system.settings_->camera2())};
 std::vector<float> intr[2];for(int c=0;c<2;++c){for(int j=0;j<16;++j)intr[c].push_back(cam[c]->getParameter(j));tracker->mFactoryIntrinsics[c]=intr[c];}
 Sophus::SE3f Tlr=system.settings_->Tlr();const Sophus::SE3f rigBefore=Tlr;
 Rig::PublishGlobals(false);cv::Mat image(480,640,CV_8UC1);cv::RNG random(31);random.fill(image,cv::RNG::UNIFORM,0,256);
 ORBextractor ex0(100,1.2,8,20,7),ex1(100,1.2,8,20,7);cv::Mat K=cam[0]->toK(),dist=cv::Mat::zeros(4,1,CV_32F);
 IMU::Calib calibration(Sophus::SE3f(),.001f,.01f,.00001f,.0001f);
 Frame prototype(image,image,0.,&ex0,&ex1,system.mpVocabulary,K,dist,30,10,cam[0],cam[1],Tlr,nullptr,calibration);
 std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> truth[2];
 for(int c=0;c<2;++c)for(int r:{80,140,220,265,290})for(int a=0;a<32;++a){
  float phi=2*M_PI*a/32;cv::Point2f pixel(cam[c]->getParameter(2)+r*std::cos(phi),cam[c]->getParameter(3)+r*std::sin(phi));
  if(pixel.x<4||pixel.x>636||pixel.y<4||pixel.y>476)continue;
  cv::Point3f ray;if(!cam[c]->tryUnproject(pixel,ray))continue;Eigen::Vector3f pc=Eigen::Vector3f(ray.x,ray.y,ray.z)*(3.f+.02f*a);truth[c].push_back(c?Tlr*pc:pc);
 }
 const int originalRight=truth[1].size();std::vector<int> sharedLeft;
 for(size_t j=0;j<truth[0].size();++j){
  bool valid=true;for(float x:{0.f,.462f}){Eigen::Vector3f pc=Tlr.inverse()*(truth[0][j]-Eigen::Vector3f(x,0,0));auto uv=cam[1]->project(pc);cv::Point3f ray;valid &= cam[1]->tryUnproject(cv::Point2f(uv.x(),uv.y()),ray)&&uv.x()>4&&uv.x()<636&&uv.y()>4&&uv.y()<476;}
  if(valid){sharedLeft.push_back(j);truth[1].push_back(truth[0][j]);}
 }
 check(sharedLeft.size()>10,"shared stereo landmarks make metric scale observable");
 const int left=truth[0].size(),right=truth[1].size();check(left>80&&right>80,"both cameras have centre and peripheral synthetic observations");
 std::vector<std::unique_ptr<Frame>> frames;std::vector<KeyFrame*> keys;
 for(int n=0;n<12;++n){
  frames.emplace_back(new Frame(prototype));Frame& f=*frames.back();f.mnId=n;f.mTimeStamp=.2*n;f.Nleft=left;f.Nright=right;f.N=left+right;
  f.mvKeys.clear();f.mvKeysRight.clear();f.mvpMapPoints.assign(f.N,nullptr);f.mvbOutlier.assign(f.N,false);f.mvuRight.assign(left,-1);f.mvDepth.assign(left,-1);f.mvStereo3Dpoints.assign(left,Eigen::Vector3f::Zero());f.mvLeftToRightMatch.assign(left,-1);f.mvRightToLeftMatch.assign(right,-1);
  f.mDescriptors=cv::Mat::zeros(f.N,32,CV_8U);f.mDescriptorsRight=cv::Mat::zeros(right,32,CV_8U);
  Sophus::SE3f actual(Eigen::Matrix3f::Identity(),Eigen::Vector3f(-.02f*n-.002f*n*n,0,0));
  for(int c=0;c<2;++c)for(const auto& p:truth[c]){Eigen::Vector3f pc=actual*p;if(c)pc=Tlr.inverse()*pc;auto uv=cam[c]->project(pc);(c?f.mvKeysRight:f.mvKeys).push_back(cv::KeyPoint(uv.x(),uv.y(),1,0,0,0));}
  f.mvKeysUn=f.mvKeys;
  for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y){f.mGrid[x][y].clear();f.mGridRight[x][y].clear();}
  Eigen::Vector3f error=n?Eigen::Vector3f(.005f*std::sin(n),.004f*std::cos(n),.002f*std::sin(2*n)):Eigen::Vector3f::Zero();
  f.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),actual.translation()+error));f.SetVelocity(Eigen::Vector3f(.1f+.02f*n,.003f,0));f.mImuBias=IMU::Bias();
  f.mpImuPreintegrated=nullptr;f.mpImuPreintegratedFrame=nullptr;f.mpcpi=nullptr;
  if(n){f.mpImuPreintegrated=new IMU::Preintegrated(IMU::Bias(),calibration);for(int i=0;i<20;++i)f.mpImuPreintegrated->IntegrateNewMeasurement(Eigen::Vector3f(.1f,0,9.81f),Eigen::Vector3f::Zero(),.01f);}
  auto* key=new KeyFrame(f,map,system.mpKeyFrameDatabase);if(n){key->mPrevKF=keys.back();keys.back()->mNextKF=key;}keys.push_back(key);map->AddKeyFrame(key);
 }
 std::vector<MapPoint*> points;
 for(int c=0;c<2;++c)for(size_t j=0;j<truth[c].size();++j){
  Eigen::Vector3f offset(.009f*std::sin(j),.007f*std::cos(j),.005f);MapPoint* p;
  if(c==1 && j>=static_cast<size_t>(originalRight))p=points[sharedLeft[j-originalRight]];
  else {p=new MapPoint(truth[c][j]+offset,keys.front(),map);points.push_back(p);map->AddMapPoint(p);}
  for(auto* k:keys){int idx=c?left+j:j;k->AddMapPoint(p,idx);p->AddObservation(k,idx);}p->UpdateNormalAndDepth();
 }
 // An independent populated map shares the same physical camera objects.
 // Direct fixed-VI calls must leave its geometry and calibration untouched.
 Map other(10000);other.SetImuInitialized();KeyFrame unrelated(*frames.front(),&other,system.mpKeyFrameDatabase);other.AddKeyFrame(&unrelated);auto unrelatedPose=unrelated.GetPose();
 tracker->mCurrentFrame=*frames.back();tracker->mLastFrame=*frames[10];tracker->mInitialFrame=*frames.front();
 tracker->mCurrentFrame.mpReferenceKF=tracker->mCurrentFrame.mpLastKeyFrame=keys.back();tracker->mLastFrame.mpReferenceKF=tracker->mLastFrame.mpLastKeyFrame=keys[10];tracker->mInitialFrame.mpReferenceKF=tracker->mInitialFrame.mpLastKeyFrame=keys.front();
 tracker->mCurrentFrame.mpcpi=prior();tracker->mLastFrame.mpcpi=tracker->mCurrentFrame.mpcpi;
 const auto anchorPose=keys.front()->GetPose();const auto cachedDepth=keys.back()->mvDepth;const auto cachedFrameDepth=tracker->mCurrentFrame.mvDepth;const cv::Mat cachedK=tracker->mCurrentFrame.mK.clone();
 const int version=map->GetMapChangeIndex();double initialError=0;for(size_t j=0;j<points.size();++j){const auto& p=j<truth[0].size()?truth[0][j]:truth[1][j-truth[0].size()];initialError+=(points[j]->GetWorldPos()-p).norm();}
 bool accepted=Optimizer::FullCalibrationBA(map,tracker,2.2,false);
 check(accepted,"exact compiled fixed-intrinsic FullCalibrationBA accepts consistent synthetic VI graph");
 bool identical=true;for(int c=0;c<2;++c)for(int j=0;j<16;++j)identical&=cam[c]->getParameter(j)==intr[c][j];
 check(identical,"all 32 live intrinsic scalars remain bit-identical");
 check((keys.front()->GetPose().matrix()-anchorPose.matrix()).norm()==0,"fixed first-pose gauge is preserved");
 check((keys.back()->GetRelativePoseTlr().matrix()-rigBefore.matrix()).norm()==0,"physical stereo transform stays unchanged");
 check((unrelated.GetPose().matrix()-unrelatedPose.matrix()).norm()==0,"other-map pose sharing camera objects stays unchanged");
 check(keys.back()->mvDepth==cachedDepth&&tracker->mCurrentFrame.mvDepth==cachedFrameDepth&&cv::norm(tracker->mCurrentFrame.mK,cachedK,cv::NORM_INF)==0,"fixed-camera commit preserves stereo and intrinsic caches");
 check(tracker->mCurrentFrame.mpcpi==nullptr&&tracker->mLastFrame.mpcpi==nullptr,"shared obsolete current/last marginal prior invalidated exactly once");
 check(map->GetMapChangeIndex()==version+1,"accepted full VI publishes exactly one map update");
 double finalError=0;for(size_t j=0;j<points.size();++j){const auto& p=j<truth[0].size()?truth[0][j]:truth[1][j-truth[0].size()];finalError+=(points[j]->GetWorldPos()-p).norm();}
 std::cout<<"GEOMETRY mean_initial="<<initialError/points.size()<<" mean_final="<<finalError/points.size()<<std::endl;
 check(finalError<initialError*.2,"metric landmark geometry improves without intrinsic changes");
 // Reject an invalid private solve before any live-state commit.
 keys[1]->mpImuPreintegrated->C.setZero();tracker->mCurrentFrame.mpcpi=prior();auto* retainedPrior=tracker->mCurrentFrame.mpcpi;
 std::vector<Sophus::SE3f> saved;std::vector<Eigen::Vector3f> velocity,position;for(auto* k:keys){saved.push_back(k->GetPose());velocity.push_back(k->GetVelocity());}for(auto* p:points)position.push_back(p->GetWorldPos());
 const int rejectVersion=map->GetMapChangeIndex();bool rejected=!Optimizer::FullCalibrationBA(map,tracker,62.2,false);bool unchanged=rejected;
 for(size_t i=0;i<keys.size();++i)unchanged&=(saved[i].matrix()-keys[i]->GetPose().matrix()).norm()==0&&(velocity[i]-keys[i]->GetVelocity()).norm()==0;
 for(size_t i=0;i<points.size();++i)unchanged&=(position[i]-points[i]->GetWorldPos()).norm()==0;
 check(unchanged&&map->GetMapChangeIndex()==rejectVersion&&tracker->mCurrentFrame.mpcpi==retainedPrior,"invalid IMU chain rejects atomically with original states/prior intact");
 delete retainedPrior;tracker->mCurrentFrame.mpcpi=nullptr;
 std::cout<<"RESULT failures="<<failures<<std::endl;return failures?1:0;
}
