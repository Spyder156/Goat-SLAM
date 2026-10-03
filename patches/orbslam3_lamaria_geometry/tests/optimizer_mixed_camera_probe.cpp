// Focused runtime regression: actual fork classes, actual calibrated Fisheye624,
// synthetic observations/IMU. Unequal left/right pools deliberately include
// camera1-only observations. No dataset ground truth enters this fixture.
#include "Optimizer.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "CameraModels/Fisheye624.h"
#include "ORBextractor.h"
#include "Rig.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <cstdlib>
#include <cmath>
using namespace ORB_SLAM3;

void require(bool v,const char* reason){if(!v)throw std::runtime_error(reason);}
Sophus::SE3f readPose(cv::FileStorage& fs,const char* name){
    cv::Mat m;fs[name]>>m;Eigen::Matrix3f R;Eigen::Vector3f t;
    for(int r=0;r<3;++r){for(int c=0;c<3;++c)R(r,c)=m.at<float>(r,c);t[r]=m.at<float>(r,3);}
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(R,Eigen::ComputeFullU|Eigen::ComputeFullV);
    R=svd.matrixU()*svd.matrixV().transpose();return Sophus::SE3f(R,t);
}
struct Fixture {
    static const int nl=48,nr=64,np=60;
    Map* map;KeyFrameDatabase db;Fisheye624 *c0,*c1;Sophus::SE3f T01,Tbc;
    std::vector<KeyFrame*> kfs;std::vector<MapPoint*> points;std::vector<Eigen::Vector3f> original;
    std::vector<Sophus::SE3f> poses;
    Fixture(const char* yaml,bool imu):map(new Map(KeyFrame::nNextId)){
        cv::FileStorage fs(yaml,cv::FileStorage::READ);
        const std::vector<std::string> names={"fx","fy","cx","cy","k1","k2","k3","k4","k5","k6","p1","p2","s1","s2","s3","s4"};
        std::vector<float>a,b;for(auto n:names){a.push_back(float(fs["Camera1."+n]));b.push_back(float(fs["Camera2."+n]));}
        c0=new Fisheye624(a);c1=new Fisheye624(b);c0->mvLappingArea={0,640};c1->mvLappingArea={0,640};
        T01=readPose(fs,"Rig.T_c0_c1");Tbc=readPose(fs,"IMU.T_b_c1");
        IMU::Calib calib(Tbc,.01,.02,.001,.002);if(imu)map->SetImuInitialized();
        cv::Mat K=c0->toK(),dist=cv::Mat::zeros(4,1,CV_32F),im(480,640,CV_8UC1);
        cv::RNG rng(94731);rng.fill(im,cv::RNG::UNIFORM,0,255);
        ORBextractor e0(180,1.2,8,20,7),e1(220,1.2,8,20,7);
        for(int j=0;j<np;++j){const float z=2.3f+.04f*j;original.emplace_back(.1f*(j%10-5),-.6f*z+.07f*(j/10-2),z);}
        const Eigen::Vector3f vel(.4f,0,0);const Eigen::Matrix3f Rwb=Tbc.rotationMatrix().transpose();
        const Eigen::Vector3f acc=Rwb.transpose()*Eigen::Vector3f(0,0,IMU::GRAVITY_VALUE);
        for(int i=0;i<11;++i){
            Frame f(im,im,450+.05*i,&e0,&e1,nullptr,K,dist,T01.translation().norm()*a[0],60,c0,c1,T01,nullptr,calib);
            require(f.Nleft>0&&f.Nright>0,"real Frame constructor must initialize both pools");
            f.Nleft=nl;f.Nright=nr;f.N=nl+nr;
            f.mvKeys.assign(nl,cv::KeyPoint(100,100,8));f.mvKeysUn=f.mvKeys;f.mvKeysRight.assign(nr,cv::KeyPoint(200,200,8));
            f.mDescriptors=cv::Mat::zeros(nl+nr,32,CV_8U);f.mDescriptorsRight=cv::Mat::zeros(nr,32,CV_8U);
            f.mvpMapPoints.assign(nl+nr,nullptr);f.mvbOutlier.assign(nl+nr,false);
            f.mvuRight.assign(nl,-1);f.mvDepth.assign(nl,-1);f.mvLeftToRightMatch.assign(nl,-1);f.mvRightToLeftMatch.assign(nr,-1);
            f.mnDataset=0;f.mpImuPreintegrated=nullptr;f.mImuBias=IMU::Bias();f.mNameFile="synthetic_mixed_pool";
            const Eigen::Vector3f center=.05f*i*vel;const Sophus::SE3f Tcw(Eigen::Matrix3f::Identity(),-center);
            poses.push_back(Tcw);f.SetPose(Tcw);f.SetVelocity(vel);
            if(imu){f.mpImuPreintegrated=new IMU::Preintegrated(IMU::Bias(),calib);for(int s=0;s<50;++s)f.mpImuPreintegrated->IntegrateNewMeasurement(acc,Eigen::Vector3f::Zero(),.001f);}
            for(int j=0;j<np;++j){const auto inds=indices(i,j);const Eigen::Vector3f pc0=Tcw*original[j],pc1=T01.inverse()*pc0;
                require(pc0.z()>0&&pc1.z()>0,"fixture point must face both cameras");
                const Eigen::Vector2f uv0=c0->project(pc0),uv1=c1->project(pc1);require(uv0.allFinite()&&uv1.allFinite(),"finite camera projections");
                if(inds.first>=0){f.mvKeys[inds.first]=cv::KeyPoint(uv0.x(),uv0.y(),8,-1,1,j%3);f.mvKeysUn[inds.first]=f.mvKeys[inds.first];}
                if(inds.second>=0)f.mvKeysRight[inds.second]=cv::KeyPoint(uv1.x(),uv1.y(),8,-1,1,(j+1)%3);
            }
            auto* kf=new KeyFrame(f,map,&db);kfs.push_back(kf);map->AddKeyFrame(kf);
            if(i>0&&i!=4){kf->mPrevKF=kfs[i-1];kfs[i-1]->mNextKF=kf;}
        }
        for(int j=0;j<np;++j){auto* p=new MapPoint(original[j],kfs[0],map);p->mnTrackScaleLevel=0;p->mnBALocalForMerge=0;points.push_back(p);map->AddMapPoint(p);
            for(int i=0;i<11;++i){const auto inds=indices(i,j);if(inds.first>=0){kfs[i]->AddMapPoint(p,inds.first);p->AddObservation(kfs[i],inds.first);}if(inds.second>=0){kfs[i]->AddMapPoint(p,nl+inds.second);p->AddObservation(kfs[i],nl+inds.second);}}
            p->UpdateNormalAndDepth();
        }
    }
    static std::pair<int,int> indices(int i,int j){
        if(i==1&&j==0)return {-1,1}; // KF2 camera1-only for a KF1 camera0 match.
        if(i==1&&j==1)return {-1,-1}; // genuinely absent KF2 observation: bAllPoints.
        if(j<24)return {j,-1};if(j<36)return {-1,j-24+16};return {j-36+24,j-36+28};
    }
    double maxError(int* obs=nullptr){double worst=0;int n=0;for(auto* p:points){require(!p->isBad(),"valid fixture landmark removed");require(p->GetWorldPos().allFinite(),"non-finite optimized landmark");
        for(auto o:p->GetObservations()){auto*k=o.first;require(k->GetPose().matrix().allFinite(),"non-finite optimized pose");for(int cam=0;cam<2;++cam){int ix=cam?std::get<1>(o.second):std::get<0>(o.second);if(ix<0)continue;
            require(ix<(cam?nl+nr:nl)&&(!cam||ix>=nl),"invalid pooled index in observations");const auto& kp=cam?k->mvKeysRight[ix-nl]:k->mvKeysUn[ix];
            Eigen::Vector3f pc=k->GetPose()*p->GetWorldPos();if(cam)pc=T01.inverse()*pc;const Eigen::Vector2f uv=(cam?c1:c0)->project(pc);
            require(uv.allFinite(),"non-finite optimized projection");worst=std::max(worst,double((uv-Eigen::Vector2f(kp.pt.x,kp.pt.y)).norm()));++n;
        }}}
        if(obs)*obs=n;return worst;
    }
    double pointMovement(){double max=0;for(int j=0;j<np;++j)max=std::max(max,double((points[j]->GetWorldPos()-original[j]).norm()));return max;}
};
int main(int argc,char**argv){try{
    if(argc!=3)return 2;unsetenv("KP_DIR");unsetenv("KP_DIR1");cv::setNumThreads(1);Rig::PublishGlobals(false);
    const std::string mode(argv[2]);Fixture f(argv[1],mode=="inertial");int obsBefore=0;const double before=f.maxError(&obsBefore);
    std::cout<<std::setprecision(12)<<"fixture left="<<Fixture::nl<<" right="<<Fixture::nr<<" landmarks="<<Fixture::np<<" observations="<<obsBefore<<" max_initial_pixel_error="<<before<<std::endl;
    require(before<.0001,"initial exact fixture reprojection");bool stop=false;
    // The merge solvers have a finite iteration budget; 0.01 px requires over
    // 98.6% correction of the deliberately perturbed right-only landmarks.
    if(mode!="sim3"){for(int j=24;j<36;++j)f.points[j]->SetWorldPos((f.original[j]+Eigen::Vector3f(.01f,-.006f,.008f)).eval());std::cout<<"perturbed_camera1_only_pixel_error="<<f.maxError()<<std::endl;}
    if(mode=="sim3"){
        std::vector<MapPoint*> matches(Fixture::nl+Fixture::nr,nullptr);for(int j=0;j<Fixture::np;++j){auto ix=Fixture::indices(0,j);if(ix.first>=0)matches[ix.first]=f.points[j];if(ix.second>=0)matches[Fixture::nl+ix.second]=f.points[j];}
        Sophus::SE3f T12=f.kfs[0]->GetPose()*f.kfs[1]->GetPoseInverse();g2o::Sim3 S(T12.unit_quaternion().cast<double>(),T12.translation().cast<double>(),1);
        Eigen::Matrix<double,7,7> H=Eigen::Matrix<double,7,7>::Zero();int n=Optimizer::OptimizeSim3(f.kfs[0],f.kfs[1],matches,S,5.991,true,H,true);
        require(n>=46,"lost exact cam0 Sim3 inliers");require(!matches[0],"camera1-only KF2 match not filtered");require(matches[1],"absent-KF2 camera-pixel fallback rejected exact match");
        for(int i=Fixture::nl;i<int(matches.size());++i)require(!matches[i],"camera1 pooled Sim3 match not filtered");
        require(std::isfinite(S.scale())&&std::abs(S.scale()-1)<1e-10,"Sim3 fixed metric scale changed");require((S.translation()-T12.translation().cast<double>()).norm()<1e-4,"Sim3 transform changed exact geometry");
        std::cout<<"sim3 inliers="<<n<<" scale="<<S.scale()<<" translation_error="<<(S.translation()-T12.translation().cast<double>()).norm()<<std::endl;
    }else if(mode=="visual"){
        Optimizer::LocalBundleAdjustment(f.kfs[10],{f.kfs[2],f.kfs[3],f.kfs[4],f.kfs[5],f.kfs[6],f.kfs[7],f.kfs[8],f.kfs[9],f.kfs[10]},{f.kfs[0],f.kfs[1]},&stop);
    }else if(mode=="inertial"){
        LoopClosing::KeyFrameAndPose corrected;Optimizer::MergeInertialBA(f.kfs[10],f.kfs[2],&stop,f.map,corrected);require(corrected.size()==10,"inertial merge did not optimize expected keyframe window");
        for(auto*k:f.kfs){require(k->GetVelocity().allFinite(),"non-finite optimized velocity");require((k->GetVelocity()-Eigen::Vector3f(.4f,0,0)).norm()<.005,"exact inertial velocity changed");}
        std::cout<<"inertial corrected_poses="<<corrected.size()<<" preintegration_dt="<<f.kfs[10]->mpImuPreintegrated->dT<<std::endl;
    }else return 3;
    int afterObs=0;const double after=f.maxError(&afterObs);std::cout<<"mode="<<mode<<" max_final_pixel_error="<<after<<" landmark_movement="<<f.pointMovement()<<" observations_after="<<afterObs<<std::endl;
    require(afterObs==obsBefore,"exact mixed-camera observations unexpectedly removed");require(after<(mode=="sim3"?.002:.01),"optimizer violated calibrated mixed-camera reprojection");require(f.pointMovement()<.005,"optimizer distorted exact metric landmarks");std::cout<<"PASS "<<mode<<std::endl;return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<std::endl;return 1;}}
