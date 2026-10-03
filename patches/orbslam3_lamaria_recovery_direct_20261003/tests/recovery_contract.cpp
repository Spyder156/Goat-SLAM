// Fixture data only. Matcher, fisheye projection, MLPnP and pose optimization
// execute the selected production library, with the unchanged numeric gates.
#include "Atlas.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "MLPnPsolver.h"
#include "CameraModels/Fisheye624.h"
#include "TrackingStateRollback.h"
#include "Rig.h"
#include <chrono>
#include <iostream>
#include <set>
using namespace ORB_SLAM3;

struct Fixture {
    ORBVocabulary vocabulary;
    KeyFrameDatabase database;
    Atlas atlas;
    Fisheye624 cam0,cam1;
    ORBextractor extractor0,extractor1;
    cv::Mat texture,descriptors;
    std::vector<Eigen::Vector3f> points;
    int failures=0;
    Fixture(int count=80):database(vocabulary),atlas(0),cam0(parameters()),cam1(parameters()),
        extractor0(100,1.2,8,20,7),extractor1(100,1.2,8,20,7) {
        cv::RNG random(1423);texture=cv::Mat(480,640,CV_8UC1);
        random.fill(texture,cv::RNG::UNIFORM,0,256);
        descriptors=cv::Mat(count,32,CV_8UC1);random.fill(descriptors,cv::RNG::UNIFORM,0,256);
        cam0.mvLappingArea={0,640};cam1.mvLappingArea={0,640};
        for(int i=0;i<count;++i)points.push_back(Eigen::Vector3f((i%10-4.5f)*.38f,(i/10-3.5f)*.27f,4.f+.13f*(i%11)));
    }
    static std::vector<float> parameters(){std::vector<float> p(16,0);p[0]=p[1]=240;p[2]=320;p[3]=240;return p;}
    Frame frame(const Sophus::SE3f& pose) {
        Sophus::SE3f T01(Eigen::AngleAxisf(.12f,Eigen::Vector3f::UnitY()).toRotationMatrix(),Eigen::Vector3f(.14f,0,0));
        cv::Mat K=cam0.toK(),distortion=cv::Mat::zeros(4,1,CV_32F);
        Frame f(texture,texture,10.+Frame::nNextId,&extractor0,&extractor1,&vocabulary,
                K,distortion,33.6f,60.f,&cam0,&cam1,T01);
        f.SetPose(pose);f.SetVelocity(Eigen::Vector3f::Zero());f.mnDataset=0;f.mNameFile="recovery_contract";
        f.Nleft=points.size();f.Nright=points.size();f.N=f.Nleft+f.Nright;
        f.mvKeys.clear();f.mvKeysRight.clear();
        for(const auto& p:points) {
            const auto left=cam0.project(pose*p),right=cam1.project(f.GetRelativePoseTrl()*pose*p);
            f.mvKeys.emplace_back(left.x(),left.y(),1,0,1,0);
            f.mvKeysRight.emplace_back(right.x(),right.y(),1,0,1,0);
        }
        f.mvKeysUn=f.mvKeys;
        cv::vconcat(descriptors,descriptors,f.mDescriptors);
        f.mvpMapPoints.assign(f.N,nullptr);f.mvbOutlier.assign(f.N,false);
        f.mvuRight.assign(f.Nleft,-1);f.mvDepth.assign(f.Nleft,-1);
        f.mvLeftToRightMatch.assign(f.Nleft,-1);f.mvRightToLeftMatch.assign(f.Nright,-1);
        f.mBowVec.clear();f.mFeatVec.clear();f.mBowVec[1]=1;
        for(int i=0;i<f.N;++i)f.mFeatVec[1].push_back(i);
        for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y){f.mGrid[x][y].clear();f.mGridRight[x][y].clear();}
        return f;
    }
    KeyFrame* keyframe() {
        Frame f=frame(Sophus::SE3f());f.mFeatVec.clear();f.mBowVec.clear();f.mBowVec[0]=1;
        for(int i=0;i<f.N;++i)f.mFeatVec[0].push_back(i);
        auto* kf=new KeyFrame(f,atlas.GetCurrentMap(),&database);atlas.GetCurrentMap()->AddKeyFrame(kf);
        for(size_t i=0;i<points.size();++i) {
            auto* p=new MapPoint(points[i],kf,atlas.GetCurrentMap());atlas.GetCurrentMap()->AddMapPoint(p);
            kf->AddMapPoint(p,i);p->AddObservation(kf,i);
            kf->AddMapPoint(p,i+points.size());p->AddObservation(kf,i+points.size());
        }
        return kf;
    }
    void check(bool ok,const char* name){std::cout<<"CHECK "<<(ok?"PASS ":"FAIL ")<<name<<std::endl;failures+=!ok;}
};

int main() {
    cv::setNumThreads(1);Rig::PublishGlobals(false);Fixture f;
    static_assert(sizeof(ORBmatcher)==8,"Matcher class layout changed; incremental build no longer valid");
    ORBmatcher matcher(.75,true);KeyFrame* kf=f.keyframe();
    const Sophus::SE3f truth(Eigen::AngleAxisf(.08f,Eigen::Vector3f::UnitY()).toRotationMatrix(),Eigen::Vector3f(.3f,.1f,-.05f));
    Frame frame=f.frame(truth);std::vector<MapPoint*> matches;ORBmatcher::DirectRecoveryStats stats;
    f.check(matcher.SearchByBoW(kf,frame,matches)==0,"disjoint ORB vocabulary nodes suppress all fixture correspondences");
    const int n=matcher.SearchByDescriptor(kf,frame,matches,&stats);
    f.check(n==160 && stats.accepted[0]==80 && stats.accepted[1]==80,"direct production matcher recovers all80 landmarks in both lenses");
    bool exact=true;for(int i=0;i<frame.N;++i)exact&=matches[i]==kf->GetMapPoint(i);
    f.check(exact,"direct correspondences preserve pooled-camera landmark identity");
    bool hamming=true;for(int i=0;i<80;++i)
        hamming&=ORBmatcher::DescriptorDistance(f.descriptors.row(i),f.descriptors.row((i+1)%80))==cv::norm(f.descriptors.row(i),f.descriptors.row((i+1)%80),cv::NORM_HAMMING);
    f.check(hamming,"OpenCV exact Hamming kernel agrees with production DescriptorDistance");
    for(int camera=0;camera<2;++camera) {
        MLPnPsolver solver(frame,matches,camera);solver.SetRansacParameters(.99,10,300,6,.5,5.991);
        bool exhausted=false,solved=false;std::vector<bool> inliers;int count=0;Eigen::Matrix4f result;
        for(int iteration=0;iteration<60 && !exhausted && !solved;++iteration)solved=solver.iterate(5,exhausted,inliers,count,result);
        f.check(solved,camera ? "native cam1 MLPnP recovers a valid pose" : "native cam0 MLPnP recovers a valid pose");
        if(solved) {
            Sophus::SE3f pose(result);if(camera==1)pose=frame.GetRelativePoseTlr()*pose;
            frame.SetPose(pose);frame.mvpMapPoints=matches;std::fill(frame.mvbOutlier.begin(),frame.mvbOutlier.end(),false);
            const int verified=Optimizer::PoseOptimization(&frame);
            f.check(verified>=50 && (frame.GetPose().matrix()-truth.matrix()).norm()<1e-3,
                    camera ? "cam1 pose converts to cam0 rig coordinates and passes unchanged native optimizer" : "cam0 pose passes unchanged native rig optimizer");
        }
    }
    // Strong descriptors with deliberately incorrect image geometry must not
    // pass the native 50-inlier geometric acceptance used by recovery.
    Frame corrupted=f.frame(truth);corrupted.mvpMapPoints=matches;
    for(size_t i=0;i<corrupted.mvKeys.size();++i) {
        corrupted.mvKeys[i].pt=cv::Point2f(70+(i*137)%500,60+(i*83)%360);
        corrupted.mvKeysRight[i].pt=cv::Point2f(80+(i*179)%480,50+(i*127)%370);
    }
    f.check(Optimizer::PoseOptimization(&corrupted)<50,"correct descriptors cannot bypass native geometric rejection");
    Frame ambiguous=f.frame(truth);ambiguous.mDescriptors.row(0).copyTo(ambiguous.mDescriptors.row(1));
    matcher.SearchByDescriptor(kf,ambiguous,matches,&stats);
    auto* first=kf->GetMapPoint(0);bool ambiguousAccepted=false;
    for(int i=0;i<ambiguous.Nleft;++i)ambiguousAccepted|=matches[i]==first;
    f.check(!ambiguousAccepted && matches[ambiguous.Nleft]==first,"zero-distance descriptor ambiguity fails ratio independently in cam0");

    // Deliberately conflicting KF slots exercise global current-feature and
    // per-camera landmark ownership without altering the native matcher.
    kf->mDescriptors.row(0).copyTo(kf->mDescriptors.row(1));
    matcher.SearchByDescriptor(kf,frame,matches,&stats);
    std::set<MapPoint*> seen[2];bool unique=true;
    for(int i=0;i<frame.N;++i)if(matches[i])unique&=seen[i>=frame.Nleft].insert(matches[i]).second;
    f.check(unique && matches[0]==first,"competing source landmarks cannot claim one image feature; deterministic owner");
    kf->AddMapPoint(first,1);f.descriptors.row(1).copyTo(kf->mDescriptors.row(1));
    matcher.SearchByDescriptor(kf,frame,matches,&stats);
    int claims=0;for(int i=0;i<frame.Nleft;++i)claims+=matches[i]==first;
    f.check(claims==1,"duplicate source observation cannot give one landmark multiple features per lens");

    Fixture gate(2);auto* gateKF=gate.keyframe();Frame gateFrame=gate.frame(Sophus::SE3f());
    gateKF->EraseMapPointMatch(1);gateKF->EraseMapPointMatch(3);cv::Mat gateDescriptors=gateKF->mDescriptors;gateDescriptors.setTo(0);
    gateFrame.mDescriptors.setTo(255);
    for(int camera=0;camera<2;++camera) {
        const int row=camera*gateFrame.Nleft,bits=70+camera;gateFrame.mDescriptors.row(row).setTo(0);
        for(int bit=0;bit<bits;++bit)gateFrame.mDescriptors.at<unsigned char>(row,bit/8)|=1u<<(bit%8);
    }
    matcher.SearchByDescriptor(gateKF,gateFrame,matches,&stats);
    f.check(stats.accepted[0]==1 && stats.accepted[1]==0,"distance70 accepted and71 rejected, same fixed Hamming gate");
    gateFrame.N=4097;matcher.SearchByDescriptor(gateKF,gateFrame,matches,&stats);
    f.check(stats.rejectedInput,"explicit4096-feature work bound rejects oversized input safely");
    gateFrame.N=4;gateFrame.mDescriptors=cv::Mat::zeros(4,128,CV_32F);
    matcher.SearchByDescriptor(gateKF,gateFrame,matches,&stats);
    f.check(stats.rejectedInput,"float descriptors cannot silently enter binary-distance matcher");

    Frame state;state.mTimeStamp=20.;state.mImuCalib=IMU::Calib(Sophus::SE3f(Eigen::Matrix3f::Identity(),Eigen::Vector3f(.05f,-.03f,.08f)),.001,.01,.0001,.001);
    const Eigen::Vector3f position(1,2,3),velocity(.4,.5,.6);const IMU::Bias bias(.01,.02,.03,.001,.002,.003);
    state.SetImuPoseVelocity(Eigen::Matrix3f::Identity(),position,velocity);state.mImuBias=bias;
    const TrackingStateRollback prediction(state);double lostSince=17.;
    state.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),Eigen::Vector3f(14,-5,8)));
    const TrackingStateRollback tooLate(state);
    state.SetVelocity(Eigen::Vector3f(8,9,10));state.mImuBias=IMU::Bias(.5,.4,.3,.2,.1,.6);
    state.mpcpi=new ConstraintPoseImu(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Matrix15d::Identity());
    MapPoint point;state.mvpMapPoints={&point};state.mvbOutlier={false};
    prediction.finishRelatch(state,false,lostSince);
    f.check((state.GetImuPosition()-position).norm()<1e-6 && (state.GetVelocity()-velocity).norm()<1e-6 && state.mImuBias.bax==bias.bax,
            "failed local confirmation restores pre-relatch native body pose velocity and bias");
    f.check(state.mpcpi==nullptr && state.mvbOutlier[0] && lostSince==17.,"failed relatch invalidates rejected prior and cannot refresh grace");
    f.check((state.GetPose().matrix()-tooLate.pose.matrix()).norm()>1.,"contract distinguishes correct snapshot from v2 late-snapshot defect");
    state.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),Eigen::Vector3f(3,4,5)));const auto accepted=state.GetPose();
    prediction.finishRelatch(state,true,lostSince);
    f.check((state.GetPose().matrix()-accepted.matrix()).norm()==0 && lostSince==20.,"confirmed relatch keeps accepted pose and refreshes grace once");

    Fixture large(1500);auto* largeKF=large.keyframe();Frame largeFrame=large.frame(Sophus::SE3f());
    const auto start=std::chrono::steady_clock::now();
    matcher.SearchByDescriptor(largeKF,largeFrame,matches,&stats);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::cout<<"BENCHMARK one_candidate_3000_features_ms="<<ms<<" comparisons="<<stats.comparisons<<std::endl;
    f.check(stats.comparisons==9000000,"full1500-per-lens cache has deterministic nine-million-comparison candidate bound");
    std::cout<<"RESULT failures="<<f.failures<<std::endl;return f.failures?1:0;
}
