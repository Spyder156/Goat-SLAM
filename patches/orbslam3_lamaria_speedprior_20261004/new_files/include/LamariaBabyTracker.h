#ifndef ORB_SLAM3_LAMARIA_BABY_TRACKER_H
#define ORB_SLAM3_LAMARIA_BABY_TRACKER_H

#include "Frame.h"
#include "CameraModels/GeometricCamera.h"
#include "LamariaBabySolver.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <limits>
#include <unordered_map>

namespace ORB_SLAM3 {

inline bool LamariaBabyEnabled() {
    static const bool enabled=[] { const char* value=std::getenv("LAMARIA_BABY_FEATURES");
        return value && std::string(value)!="0"; }();
    return enabled;
}

// The snapshots own interval preintegrations, never live marginal priors or
// MapPoints. Background matching only changes this private observation cache.
struct LamariaBabyFrame {
    std::unique_ptr<Frame> frame;
    std::unique_ptr<IMU::Preintegrated> integration;
    std::vector<uint64_t> ids;
    std::vector<bool> mapped;
    bool normal=false;
    bool sos=false;
    LamariaBabyFrame(Frame& source, const std::vector<uint64_t>& trackIds,
                     bool normalAccepted, bool babyAccepted)
        :frame(new Frame(source)),ids(trackIds),normal(normalAccepted),sos(babyAccepted)
    {
        mapped.resize(source.N,false);
        for(int i=0;i<source.N;++i)
            mapped[i]=source.mvpMapPoints[i] && !source.mvbOutlier[i];
        if(source.mpImuPreintegratedFrame)
            integration.reset(new IMU::Preintegrated(source.mpImuPreintegratedFrame));
        frame->mpImuPreintegratedFrame=integration.get();
        frame->mpImuPreintegrated=nullptr;
        frame->mpcpi=nullptr;
        frame->mpPrevFrame=nullptr;
        frame->mpReferenceKF=nullptr;
        frame->mpLastKeyFrame=nullptr;
        std::fill(frame->mvpMapPoints.begin(),frame->mvpMapPoints.end(),nullptr);
        std::fill(frame->mvbOutlier.begin(),frame->mvbOutlier.end(),false);
    }
};

struct LamariaBabyTracker {
    std::deque<std::unique_ptr<LamariaBabyFrame>> window;
    std::vector<uint64_t> currentIds;
    std::vector<float> calibration;
    uint64_t nextId=1;
    long mapId=-1;
    unsigned long matchedFrame=std::numeric_limits<unsigned long>::max();
    bool active=false,exhausted=false,promotionPending=false;
    int acceptedStreak=0,lastPairs=0;
    double enteredAt=0;
    double lastPromotionAt=-1;
    BabySolveResult lastSolve;

    void report(Frame& frame,const char* event,int inliers=0,const char* reason="") const {
        auto finite=[](double value){return std::isfinite(value)?value:-1.;};
        std::fprintf(stderr,"[BABY_DIAG] {\"frame\":%lu,\"time\":%.9f,\"map\":%ld,\"event\":\"%s\",\"window\":%zu,\"tracks\":%d,\"inliers\":%d,\"age_s\":%.6f,\"reason\":\"%s\",\"candidates\":%d,\"triangulated\":%d,\"camera_inliers\":[%d,%d],\"median_chi2\":%.8g,\"inertial_chi2\":%.8g,\"max_active_inertial_chi2\":%.8g,\"pose_delta_m\":%.8g,\"rotation_delta_rad\":%.8g,\"velocity_delta_mps\":%.8g}\n",
            frame.mnId,frame.mTimeStamp,mapId,event,window.size(),lastPairs,inliers,
            active?frame.mTimeStamp-enteredAt:0.0,reason,lastSolve.candidates,lastSolve.tracks,
            lastSolve.currentInliers[0],lastSolve.currentInliers[1],finite(lastSolve.medianChi2),
            finite(lastSolve.inertialChi2),finite(lastSolve.maxActiveInertialChi2),finite(lastSolve.poseDelta),
            finite(lastSolve.rotationDelta),finite(lastSolve.velocityDelta));
    }

    void clearWindow() {
        window.clear();currentIds.clear();matchedFrame=std::numeric_limits<unsigned long>::max();
        promotionPending=false;acceptedStreak=0;
    }

    void begin(Frame& frame,long currentMap,bool mapUpdated) {
        std::vector<float> values;
        GeometricCamera* cameras[2]={frame.mpCamera,frame.mpCamera2};
        for(auto* camera:cameras) if(camera)
            for(size_t j=0;j<camera->size();++j)values.push_back(camera->getParameter(j));
        const bool changedMap=mapId!=currentMap;
        const bool timeJump=!window.empty() &&
            (frame.mTimeStamp<=window.back()->frame->mTimeStamp ||
             frame.mTimeStamp-window.back()->frame->mTimeStamp>0.15);
        if(changedMap || timeJump || mapUpdated || values!=calibration) {
            if(!window.empty())report(frame,"reset",0,changedMap?"map_change":
                timeJump?"time_jump":mapUpdated?"map_update":"calibration_update");
            clearWindow();
            if(changedMap || timeJump) {active=false;exhausted=false;enteredAt=0;lastPromotionAt=-1;}
        }
        mapId=currentMap;calibration=std::move(values);
        match(frame);
    }

    void match(Frame& current) {
        if(matchedFrame==current.mnId)return;
        matchedFrame=current.mnId;
        currentIds.assign(current.N,0);lastPairs=0;
        if(!window.empty() && current.Nleft>=0) {
            auto& previous=*window.back();
            Frame& old=*previous.frame;
            for(int camera=0;camera<2;++camera) {
                const int offOld=camera?old.Nleft:0,offNew=camera?current.Nleft:0;
                const int nOld=camera?old.Nright:old.Nleft,nNew=camera?current.Nright:current.Nleft;
                if(offOld<0 || offNew<0 || nOld<2 || nNew<2 ||
                   old.mDescriptors.type()!=CV_8U || current.mDescriptors.type()!=CV_8U ||
                   old.mDescriptors.cols!=32 || current.mDescriptors.cols!=32 ||
                   old.mDescriptors.rows<offOld+nOld || current.mDescriptors.rows<offNew+nNew)continue;
                cv::BFMatcher matcher(cv::NORM_HAMMING);
                std::vector<std::vector<cv::DMatch>> forward;
                std::vector<cv::DMatch> backward;
                matcher.knnMatch(old.mDescriptors.rowRange(offOld,offOld+nOld),
                                 current.mDescriptors.rowRange(offNew,offNew+nNew),forward,2);
                matcher.match(current.mDescriptors.rowRange(offNew,offNew+nNew),
                              old.mDescriptors.rowRange(offOld,offOld+nOld),backward);
                for(int k=0;k<nOld;++k) {
                    if(static_cast<size_t>(k)>=forward.size() || forward[k].size()<2)continue;
                    const cv::DMatch& best=forward[k][0];
                    if(best.distance>70 || best.distance>=0.8f*forward[k][1].distance ||
                       best.trainIdx<0 || best.trainIdx>=nNew ||
                       static_cast<size_t>(best.trainIdx)>=backward.size() ||
                       backward[best.trainIdx].trainIdx!=k)continue;
                    currentIds[offNew+best.trainIdx]=previous.ids[offOld+k];++lastPairs;
                }
            }
        }
        for(auto& id:currentIds)if(!id)id=nextId++;
    }

    std::vector<BabyTrack> tracks(Frame& current) const {
        std::unordered_map<uint64_t,BabyTrack> all;
        // Only currently unassociated features enter the SOS graph. Established
        // map associations continue to belong to the ordinary tracker alone.
        for(int i=0;i<current.N;++i) {
            if(current.mvpMapPoints[i] && !current.mvbOutlier[i])continue;
            BabyTrack t;t.id=currentIds[i];all.emplace(t.id,std::move(t));
        }
        auto observe=[&](Frame& frame,const std::vector<uint64_t>& ids,size_t frameIndex,
                         const std::vector<bool>* mapped) {
            for(int i=0;i<frame.N;++i) {
                if(mapped && (*mapped)[i])continue;
                auto found=all.find(ids[i]);if(found==all.end())continue;
                const int camera=frame.Nleft>=0 && i>=frame.Nleft?1:0;
                const int index=camera?i-frame.Nleft:i;
                const auto& keys=camera?frame.mvKeysRight:frame.mvKeys;
                if(index<0 || static_cast<size_t>(index)>=keys.size())continue;
                const cv::KeyPoint& key=keys[index];
                BabyObservation observation;
                observation.frame=frameIndex;observation.camera=camera;
                observation.featureIndex=i;
                observation.pixel=Eigen::Vector2d(key.pt.x,key.pt.y);
                observation.inverseVariance=key.octave>=0 && static_cast<size_t>(key.octave)<frame.mvInvLevelSigma2.size()
                    ?frame.mvInvLevelSigma2[key.octave]:1.;
                found->second.observations.push_back(observation);
            }
        };
        for(size_t j=0;j<window.size();++j)observe(*window[j]->frame,window[j]->ids,j,&window[j]->mapped);
        observe(current,currentIds,window.size(),nullptr);
        std::vector<BabyTrack> result;
        for(auto& entry:all)
            if(entry.second.observations.size()>=2)result.push_back(std::move(entry.second));
        std::sort(result.begin(),result.end(),[](const BabyTrack& a,const BabyTrack& b) {
            if(a.observations.size()!=b.observations.size())return a.observations.size()>b.observations.size();
            return a.id<b.id;
        });
        if(result.size()>350)result.resize(350);
        return result;
    }

    BabySolveResult solve(Frame& current,bool returningToMap=false) {
        std::vector<Frame*> frames;
        std::vector<bool> fixed;
        for(auto& stored:window) {frames.push_back(stored->frame.get());fixed.push_back(stored->normal);}
        frames.push_back(&current);fixed.push_back(returningToMap);
        if(!fixed.empty())fixed[0]=true;
        return SolveBabyWindow(frames,tracks(current),fixed);
    }

    void remember(Frame& frame,bool normalAccepted,bool babyAccepted) {
        if(!frame.isSet() || currentIds.size()!=static_cast<size_t>(frame.N))return;
        if(!window.empty() && window.back()->frame->mnId==frame.mnId)return;
        window.emplace_back(new LamariaBabyFrame(frame,currentIds,normalAccepted,babyAccepted));
        // The next graph contains seven retained states and the current one.
        while(window.size()>7)window.pop_front();
        if(!active && frame.mnId%100==0)report(frame,"warm");
    }
};

inline std::unordered_map<const void*,std::unique_ptr<LamariaBabyTracker>>& LamariaBabyStates() {
    static std::unordered_map<const void*,std::unique_ptr<LamariaBabyTracker>> states;
    return states;
}
inline LamariaBabyTracker& LamariaBabyState(const void* owner) {
    auto& pointer=LamariaBabyStates()[owner];
    if(!pointer)pointer.reset(new LamariaBabyTracker);
    return *pointer;
}
}
#endif
