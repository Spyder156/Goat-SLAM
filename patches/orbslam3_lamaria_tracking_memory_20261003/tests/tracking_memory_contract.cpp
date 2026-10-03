#include "Atlas.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "CameraModels/Fisheye624.h"
#include "Rig.h"
#include "LamariaTrackingMemory.h"
#include <iostream>
#include <memory>
#include <string>
using namespace ORB_SLAM3;
struct Fixture {
    ORBVocabulary vocabulary;
    KeyFrameDatabase database;
    Atlas atlas;
    Fisheye624 cam0,cam1;
    ORBextractor extractor0,extractor1;
    cv::Mat texture;
    Eigen::Vector3f truth;
    int failures=0;
    Fixture(): database(vocabulary),atlas(0),cam0(parameters()),cam1(parameters()),
        extractor0(100,1.2,8,20,7),extractor1(100,1.2,8,20,7),truth(0,0,5) {
        cv::RNG random(12345);texture=cv::Mat(480,640,CV_8UC1);
        random.fill(texture,cv::RNG::UNIFORM,0,256);
        cam0.mvLappingArea={0,640};cam1.mvLappingArea={0,640};
    }
    static std::vector<float> parameters() {
        std::vector<float> p(16,0.f);p[0]=p[1]=240;p[2]=320;p[3]=240;return p;
    }
    Map* map() { return atlas.GetCurrentMap(); }
    void grid(Frame& f) {
        for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y){f.mGrid[x][y].clear();f.mGridRight[x][y].clear();}
        for(int side=0;side<2;++side){
            const auto& keys=side ? f.mvKeysRight:f.mvKeys;
            for(size_t i=0;i<keys.size();++i){
                int x,y;if(!f.PosInGrid(keys[i],x,y))throw std::runtime_error("fixture key outside grid");
                (side?f.mGridRight[x][y]:f.mGrid[x][y]).push_back(i);
            }
        }
    }
    std::unique_ptr<Frame> frame(double t) {
        cv::Mat K=cam0.toK(),dist=cv::Mat::zeros(4,1,CV_32F);
        Sophus::SE3f T01(Eigen::Matrix3f::Identity(),Eigen::Vector3f(.14f,0,0));
        std::unique_ptr<Frame> ptr(new Frame(texture,texture,t,&extractor0,&extractor1,
                &vocabulary,K,dist,33.6f,60.f,&cam0,&cam1,T01));
        Frame& f=*ptr;
        if(f.N==0) throw std::runtime_error("deterministic fixture texture has no features");
        f.SetPose(Sophus::SE3f());f.mnDataset=0;f.mNameFile="tracking_memory_contract";
        f.Nleft=3;f.Nright=2;f.N=5;
        const auto l=cam0.project(truth),r=cam1.project(f.GetRelativePoseTrl()*truth);
        f.mvKeys={cv::KeyPoint(l.x(),l.y(),1,0,1,0),cv::KeyPoint(100,100,1,0,1,0),cv::KeyPoint(500,350,1,0,1,0)};
        f.mvKeysUn=f.mvKeys;
        f.mvKeysRight={cv::KeyPoint(r.x(),r.y(),1,0,1,0),cv::KeyPoint(200,350,1,0,1,0)};
        f.mDescriptors=cv::Mat::zeros(f.N,32,CV_8U);
        f.mDescriptors.row(1).setTo(255);f.mDescriptors.row(2).setTo(255);f.mDescriptors.row(4).setTo(255);
        f.mvpMapPoints.assign(f.N,nullptr);f.mvbOutlier.assign(f.N,false);
        f.mvuRight.assign(f.Nleft,-1);f.mvDepth.assign(f.Nleft,-1);
        f.mvLeftToRightMatch={0,-1,-1};f.mvRightToLeftMatch={0,-1};
        f.mBowVec[0]=1.;f.mFeatVec[0]={0};grid(f);return ptr;
    }
    MapPoint* point(Frame& frame) {
        KeyFrame* kf=new KeyFrame(frame,map(),&database);map()->AddKeyFrame(kf);
        MapPoint* p=new MapPoint(truth,kf,map());map()->AddMapPoint(p);
        kf->AddMapPoint(p,0);p->AddObservation(kf,0);
        kf->AddMapPoint(p,3);p->AddObservation(kf,3);
        p->ComputeDistinctiveDescriptors();p->UpdateNormalAndDepth();
        frame.mvpMapPoints[0]=p;frame.mvpMapPoints[3]=p;
        return p;
    }
    void check(bool ok,const std::string& name) {
        std::cout<<"CHECK "<<(ok?"PASS ":"FAIL ")<<name<<std::endl;failures+=!ok;
    }
};
int main(int argc,char** argv) {
    if(argc!=2)return 2;
    cv::setNumThreads(1);Rig::PublishGlobals(false);
    Fixture f;const std::string mode=argv[1];
    auto old=f.frame(20.);MapPoint* point=f.point(*old);
    LamariaTrackingMemory memory;memory.begin(&f,f.map()->GetId(),20.,f.map()->GetInitKFid());
    if(mode=="rejected") {old->mvbOutlier[0]=old->mvbOutlier[3]=true;}
    if(mode=="camera_owner") old->mvpMapPoints[0]=nullptr;
    memory.remember(*old,f.map());
    if(mode=="rejected") {
        f.check(memory.size()==0,"rejected image observations never enter cache");return f.failures?1:0;
    }
    if(mode=="expired") memory.begin(&f,f.map()->GetId(),20.51,f.map()->GetInitKFid());
    else if(mode=="map_boundary") memory.begin(&f,f.map()->GetId()+1,20.1,f.map()->GetInitKFid());
    else if(mode=="map_epoch") memory.begin(&f,f.map()->GetId(),20.1,f.map()->GetInitKFid()+1);
    else if(mode=="owner_boundary") memory.begin(&memory,f.map()->GetId(),20.1,f.map()->GetInitKFid());
    else if(mode=="time_boundary") memory.begin(&f,f.map()->GetId(),19.9,f.map()->GetInitKFid());
    else if(mode!="short_gap") memory.begin(&f,f.map()->GetId(),20.1,f.map()->GetInitKFid());
    if(mode=="expired" || mode=="map_boundary" || mode=="map_epoch" || mode=="owner_boundary" || mode=="time_boundary") {
        f.check(memory.size()==0,"cache clears at expiration or ownership/time discontinuity");return f.failures?1:0;
    }
    auto now=f.frame(20.1);
    if(mode=="descriptor_copy") old->mDescriptors.setTo(255);
    if(mode=="short_gap") {
        auto missing=f.frame(20.05);
        memory.begin(&f,f.map()->GetId(),20.05,f.map()->GetInitKFid());memory.remember(*missing,f.map());
        memory.begin(&f,f.map()->GetId(),20.1,f.map()->GetInitKFid());
        f.check(memory.size()==2,"accepted descriptors survive one association-free frame");
    }
    if(mode=="outside_projection") {
        now->mvKeys[0].pt.x+=30.f;now->mvKeysRight[0].pt.x+=30.f;f.grid(*now);
    }
    if(mode=="scale_gate") {
        now->mvKeys[0].octave=4;now->mvKeysRight[0].octave=4;f.grid(*now);
    }
    if(mode=="ratio_gate") {
        now->mDescriptors.setTo(0);
        now->mDescriptors.at<unsigned char>(0,0)=1;
        now->mDescriptors.at<unsigned char>(1,0)=2;
        now->mDescriptors.at<unsigned char>(2,0)=4;
        now->mDescriptors.at<unsigned char>(3,0)=1;
        now->mDescriptors.at<unsigned char>(4,0)=2;
    }
    if(mode=="camera_owner") now->mDescriptors.row(3).setTo(255);
    if(mode=="occupied_feature") {
        auto other=f.frame(21.);MapPoint* occupant=f.point(*other);
        now->mvpMapPoints[0]=occupant;now->mvpMapPoints[3]=occupant;
    }
    if(mode=="occupied_landmark") {
        now->mvpMapPoints[1]=point;now->mvpMapPoints[4]=point;
    }
    if(mode=="erased_point") {f.map()->EraseMapPoint(point);delete point;point=nullptr;}
    if(mode=="bad_point") point->SetBadFlag();
    memory.seed(*now,f.map(),2.f,false,0.f);
    const bool positive=mode=="short_gap" || mode=="descriptor_copy" || mode=="normal";
    if(positive) {
        f.check(memory.result.seeded.size()==2,"strict native projection restores one landmark in each camera");
        f.check(now->mvpMapPoints[0]==point && now->mvpMapPoints[3]==point,"left and pooled-right identity is correct");
    } else {
        f.check(memory.result.seeded.empty(),"invalid/stale/conflicting/ambiguous candidate is rejected");
        if(mode=="outside_projection")
            f.check(memory.result.geometryRejected[0]==1 && memory.result.geometryRejected[1]==1,"native geometry independently rejects perfect appearance match");
        if(mode=="occupied_landmark")
            f.check(!now->mvpMapPoints[0] && !now->mvpMapPoints[3],"same landmark cannot claim two features of one lens");
    }
    std::cout<<"RESULT "<<f.failures<<std::endl;
    return f.failures?1:0;
}
