// Regression fixture linked to the unchanged production ORB-SLAM3 library.
// This file supplies observations; all ownership, mapping, fusion and BA logic
// under test lives in the selected libORB_SLAM3.so.
#include "Atlas.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "CameraModels/Fisheye624.h"
#include "Rig.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

using namespace ORB_SLAM3;

struct Mapper : LocalMapping {
    explicit Mapper(Atlas* atlas): LocalMapping(nullptr,atlas,false,false) {}
    void process(KeyFrame* kf) { InsertKeyFrame(kf); ProcessNewKeyFrame(); }
    size_t recent(MapPoint* p) const {
        return std::count(mlpRecentAddedMapPoints.begin(),mlpRecentAddedMapPoints.end(),p);
    }
};

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
    KeyFrame* keyframe() {
        cv::Mat K=cam0.toK(),dist=cv::Mat::zeros(4,1,CV_32F);
        Sophus::SE3f T01(Eigen::Matrix3f::Identity(),Eigen::Vector3f(.14f,0,0));
        // Run the production constructor to establish private stereo transforms.
        Frame f(texture,texture,10.+Frame::nNextId,&extractor0,&extractor1,
                &vocabulary,K,dist,33.6f,60.f,&cam0,&cam1,T01);
        if(f.N==0) throw std::runtime_error("deterministic fixture texture has no features");
        f.SetPose(Sophus::SE3f());f.mnDataset=0;f.mNameFile="observation_contract";
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
        f.mBowVec[0]=1.;f.mFeatVec[0]={0};
        for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y){f.mGrid[x][y].clear();f.mGridRight[x][y].clear();}
        for(int side=0;side<2;++side){
            const auto& keys=side ? f.mvKeysRight:f.mvKeys;
            for(size_t i=0;i<keys.size();++i){
                int x,y;if(!f.PosInGrid(keys[i],x,y))throw std::runtime_error("fixture key outside grid");
                (side?f.mGridRight[x][y]:f.mGrid[x][y]).push_back(i);
            }
        }
        KeyFrame* kf=new KeyFrame(f,map(),&database);
        return kf;
    }
    MapPoint* point(KeyFrame* reference) {
        MapPoint* p=new MapPoint(truth,reference,map());map()->AddMapPoint(p);return p;
    }
    void observe(MapPoint* p,KeyFrame* kf,int index) {
        kf->AddMapPoint(p,index);p->AddObservation(kf,index);
    }
    void check(bool ok,const std::string& name) {
        std::cout << "CHECK " << (ok?"PASS ":"FAIL ") << name << std::endl;
        failures+=!ok;
    }
    void tuple(MapPoint* p,KeyFrame* kf,int left,int right,const char* label) {
        auto indices=p->GetIndexInKeyFrame(kf);
        std::cout<<"OBS "<<label<<" left="<<std::get<0>(indices)<<" right="<<std::get<1>(indices)
                 <<" count="<<p->Observations()<<std::endl;
        check(indices==std::make_tuple(left,right),std::string(label)+" camera tuple");
    }
};

int main(int argc,char** argv) {
    if(argc!=2)return 2;
    cv::setNumThreads(1);Rig::PublishGlobals(false);
    Fixture f;std::string name=argv[1];
    if(name=="mapper_existing"){
        KeyFrame* source=f.keyframe();f.map()->AddKeyFrame(source);
        MapPoint* p=f.point(source);f.observe(p,source,0);
        KeyFrame* next=f.keyframe();next->AddMapPoint(p,0);next->AddMapPoint(p,3);
        const int before=p->Observations();Mapper mapper(&f.atlas);mapper.process(next);
        f.tuple(p,next,0,3,"existing landmark next KF");
        f.check(p->Observations()-before==2,"exactly two new camera observations");
        std::cout<<"RECENT "<<mapper.recent(p)<<std::endl;
        f.check(mapper.recent(p)==0,"old landmark excluded from recent-point culling queue");
        f.check(next->GetMapPoint(0)==p && next->GetMapPoint(3)==p,"both KF slots retain existing landmark");
    }else if(name=="mapper_new_stereo"){
        KeyFrame* kf=f.keyframe();MapPoint* p=f.point(kf);
        f.observe(p,kf,0);f.observe(p,kf,3);
        Mapper mapper(&f.atlas);mapper.process(kf);
        f.tuple(p,kf,0,3,"new stereo landmark");
        f.check(p->Observations()==2,"pre-registered stereo observations not counted again");
        std::cout<<"RECENT "<<mapper.recent(p)<<std::endl;
        f.check(mapper.recent(p)==1,"new stereo point queued exactly once");
    }else if(name=="add_idempotent"){
        KeyFrame* kf=f.keyframe();MapPoint* p=f.point(kf);
        f.observe(p,kf,0);f.observe(p,kf,3);
        for(int i=0;i<5;++i){p->AddObservation(kf,0);p->AddObservation(kf,3);}
        f.tuple(p,kf,0,3,"repeated registration");
        f.check(p->Observations()==2,"repeated camera registration does not inflate count");
    }else if(name=="add_conflict" || name=="add_conflict_other"){
        KeyFrame* kf=f.keyframe();MapPoint* p=f.point(kf);f.observe(p,kf,0);
        MapPoint* occupant=name=="add_conflict_other"?f.point(kf):p;
        kf->AddMapPoint(occupant,1);
        if(occupant!=p)occupant->AddObservation(kf,1);
        p->AddObservation(kf,1);
        f.tuple(p,kf,0,-1,"same-camera conflicting index");
        f.check(p->Observations()==1,"same-camera conflict does not inflate count");
        f.check(kf->GetMapPoint(0)==p,"original accepted KF slot preserved");
        f.check(kf->GetMapPoint(1)==(occupant==p?nullptr:occupant),
                occupant==p?"rejected duplicate KF slot cleared":"unrelated incoming-slot occupant preserved");
    }else if(name=="erase_last"){
        KeyFrame* kf=f.keyframe();MapPoint* p=f.point(kf);f.observe(p,kf,0);
        kf->EraseMapPointMatch(0); // caller clears its reciprocal slot, as BA does
        p->EraseObservation(kf);
        f.check(p->GetObservations().empty(),"last observation removed");
        f.check(p->Observations()==0,"empty observation count is zero");
        f.check(p->GetReferenceKeyFrame()==nullptr,"empty observation set has null reference KF");
        f.check(p->isBad(),"unobserved landmark marked bad");
    }else if(name=="replace_complementary" || name=="replace_conflict"){
        KeyFrame* kf=f.keyframe();MapPoint* survivor=f.point(kf);MapPoint* donor=f.point(kf);
        f.observe(survivor,kf,0);f.observe(donor,kf,3);
        if(name=="replace_conflict")f.observe(donor,kf,1);
        donor->Replace(survivor);
        f.tuple(survivor,kf,0,3,"replacement survivor");
        f.check(survivor->Observations()==2,"replacement counts distinct camera observations");
        f.check(kf->GetMapPoint(0)==survivor && kf->GetMapPoint(3)==survivor,"replacement preserves complementary camera slot");
        if(name=="replace_conflict")f.check(kf->GetMapPoint(1)==nullptr,"same-camera donor slot cleared");
        bool dangling=false;for(auto* p:kf->GetMapPointMatches())dangling|=(p==donor);
        f.check(!dangling,"no KF slot points to replaced donor");
        f.check(donor->isBad() && donor->GetReplaced()==survivor,"donor lifecycle linked to survivor");
    }else if(name=="fuse_right" || name=="fuse_left" || name=="fuse_sim3_left"){
        const bool right=name=="fuse_right";KeyFrame* kf=f.keyframe();MapPoint* p=f.point(kf);
        f.observe(p,kf,right?0:3);p->ComputeDistinctiveDescriptors();p->UpdateNormalAndDepth();
        ORBmatcher matcher(.8,false);std::vector<MapPoint*> points={p};
        Sophus::Sim3f sim;std::vector<MapPoint*> replacements(1,nullptr);
        auto fuse=[&](){return name=="fuse_sim3_left"?matcher.Fuse(kf,sim,points,3.,replacements):matcher.Fuse(kf,points,3.,right);};
        int fused=fuse();
        std::cout<<"FUSED "<<fused<<std::endl;
        f.check(fused==1,"fusion adds missing camera to existing KF observation");
        f.tuple(p,kf,0,3,"fused landmark");
        f.check(p->Observations()==2,"fusion counts both cameras exactly once");
        f.check(kf->GetMapPoint(0)==p && kf->GetMapPoint(3)==p,"fusion fills pooled right slot");
        int repeated=fuse();
        f.check(repeated==0,"repeated same-camera fusion is a no-op");
    }else if(name=="ba_stereo_depth" || name=="ba_mapper_depth" || name=="ba_monocular_control"){
        // One fixed production KF: left bearing alone cannot determine depth.
        // The calibrated right observation supplies the missing metric constraint.
        KeyFrame* kf=nullptr;MapPoint* p=nullptr;
        if(name=="ba_mapper_depth"){
            KeyFrame* source=f.keyframe();f.map()->AddKeyFrame(source);p=f.point(source);f.observe(p,source,0);
            kf=f.keyframe();kf->AddMapPoint(p,0);kf->AddMapPoint(p,3);
            Mapper mapper(&f.atlas);mapper.process(kf);
            f.tuple(p,kf,0,3,"production mapper before BA");
            f.map()->SetInitKFid(kf->mnId);
        }else{
            kf=f.keyframe();f.map()->AddKeyFrame(kf);p=f.point(kf);f.observe(p,kf,0);
            if(name=="ba_stereo_depth")f.observe(p,kf,3);
        }
        p->SetWorldPos(1.6f*f.truth);
        // Exclude source KF from optimizer vertices: no temporal-depth information.
        // The single included KF is fixed, isolating whether cam1's edge exists.
        Optimizer::BundleAdjustment({kf},{p},20,nullptr,0,false);
        auto pworld=p->GetWorldPos();
        const float e0=(f.cam0.project(kf->GetPose()*pworld)-f.cam0.project(f.truth)).norm();
        const float e1=(f.cam1.project(kf->GetRightPose()*pworld)-f.cam1.project(kf->GetRelativePoseTrl()*f.truth)).norm();
        std::cout<<"BA depth="<<pworld.z()<<" left_error="<<e0<<" right_error="<<e1<<std::endl;
        if(name=="ba_monocular_control"){
            f.check(std::abs(pworld.z()-8.f)<.005,"single-bearing BA cannot recover metric depth");
            f.check(e0<.01 && e1>1.,"missing cam1 leaves observable right residual");
        }else{
            f.check((pworld-f.truth).norm()<.005,"production stereo BA recovers metric depth");
            f.check(e0<.01 && e1<.01,"both production camera residuals satisfied");
        }
    }else {std::cerr<<"unknown case"<<std::endl;return 2;}
    std::cout<<"RESULT "<<name<<" failures="<<f.failures<<std::endl;
    return f.failures?1:0;
}
