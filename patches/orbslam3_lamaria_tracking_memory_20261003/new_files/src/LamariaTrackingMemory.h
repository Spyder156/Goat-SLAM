// Short accepted-observation memory for the native LaMAria tracking experiment.
// Does not own Frames, MapPoints, camera models, or inertial priors.
#ifndef LAMARIA_TRACKING_MEMORY_H
#define LAMARIA_TRACKING_MEMORY_H

#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "ORBmatcher.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <unordered_map>

namespace ORB_SLAM3 {

class LamariaTrackingMemory {
public:
    struct Result {
        int considered[2]={0,0}, geometryRejected[2]={0,0};
        std::vector<int> seeded;
    };
    static bool Enabled() {
        const char* value=std::getenv("LAMARIA_TRACKING_MEMORY");
        return !value || std::string(value)!="0";
    }

    // IDs, timestamps and copied descriptors are the ONLY persistent payload.
    // Even a culled/deleted map point is never dereferenced through this cache.
    void begin(const void* owner, unsigned long mapId, double timestamp, unsigned long initialKeyFrameId) {
        result=Result();
        if(!initialized_ || owner_!=owner || mapId_!=mapId || initialKeyFrameId_!=initialKeyFrameId ||
           !std::isfinite(timestamp) || timestamp<=timestamp_) entries_.clear();
        owner_=owner; mapId_=mapId; initialKeyFrameId_=initialKeyFrameId; timestamp_=timestamp; initialized_=true;
        expire();
    }

    size_t size() const { return entries_.size(); }

    void remember(const Frame& frame, Map* map) {
        if(!compatible(frame,map)) return;
        for(int camera=0; camera<2; ++camera) {
            const int offset=camera ? frame.Nleft : 0;
            const int count=camera ? frame.Nright : frame.Nleft;
            std::set<unsigned long> unique;
            for(int local=0; local<count; ++local) {
                const int i=offset+local;
                MapPoint* point=frame.mvpMapPoints[i];
                if(!point || frame.mvbOutlier[i] || point->isBad() ||
                   point->GetMap()!=map || point->Observations()==0 ||
                   !unique.insert(point->mnId).second) continue;
                Entry entry;
                entry.timestamp=timestamp_;
                entry.descriptor=frame.mDescriptors.row(i).clone();
                entries_[Key(camera,point->mnId)]=entry;
            }
        }
        // Age is the main bound; this also caps memory/work for pathological
        // rapid feature turnover. Deterministic oldest-first eviction.
        if(entries_.size()>4096) {
            std::vector<std::pair<double,Key>> order;
            for(const auto& item:entries_) order.emplace_back(item.second.timestamp,item.first);
            std::sort(order.begin(),order.end());
            const size_t excess=entries_.size()-4096;
            for(size_t i=0;i<excess;++i) entries_.erase(order[i].second);
        }
    }

    // Caller supplies the EXACT threshold selected by SearchLocalPoints.
    // Called only after its normal search, only in tracking state OK.
    void seed(Frame& frame, Map* map, float searchFactor,
              bool farPoints, float farThreshold) {
        if(entries_.empty() || !compatible(frame,map)) return;
        int incoming=0;
        for(size_t i=0;i<frame.mvpMapPoints.size();++i)
            incoming+=frame.mvpMapPoints[i] && !frame.mvbOutlier[i];
        if(incoming>=80) return;

        // Tracking already holds map->mMutexMapUpdate. Resolve through its live
        // membership, not a cached raw pointer or a copied old Frame.
        std::unordered_map<unsigned long,MapPoint*> live;
        for(MapPoint* point:map->GetAllMapPoints())
            if(point && !point->isBad() && point->GetMap()==map && point->Observations()>0)
                live.emplace(point->mnId,point);
        for(int camera=0;camera<2;++camera) {
            const int offset=camera ? frame.Nleft : 0;
            const int count=camera ? frame.Nright : frame.Nleft;
            if(count<2) continue;
            std::set<unsigned long> occupied;
            for(int i=0;i<count;++i)
                if(frame.mvpMapPoints[offset+i]) occupied.insert(frame.mvpMapPoints[offset+i]->mnId);
            cv::Mat oldDescriptors;
            std::vector<MapPoint*> points;
            for(const auto& item:entries_) {
                if(item.first.first!=camera || occupied.count(item.first.second)) continue;
                const auto found=live.find(item.first.second);
                if(found==live.end()) continue;
                oldDescriptors.push_back(item.second.descriptor);
                points.push_back(found->second);
            }
            result.considered[camera]=points.size();
            if(points.empty()) continue;
            const cv::Mat current=frame.mDescriptors.rowRange(offset,offset+count);
            cv::BFMatcher matcher(cv::NORM_HAMMING);
            std::vector<std::vector<cv::DMatch>> forward;
            std::vector<cv::DMatch> backward;
            matcher.knnMatch(oldDescriptors,current,forward,2);
            matcher.match(current,oldDescriptors,backward);
            for(size_t k=0;k<points.size();++k) {
                if(k>=forward.size() || forward[k].size()<2) continue;
                const cv::DMatch& best=forward[k][0];
                if(best.distance>70.f || best.distance>=.8f*forward[k][1].distance ||
                   best.trainIdx<0 || best.trainIdx>=count ||
                   static_cast<size_t>(best.trainIdx)>=backward.size() ||
                   backward[best.trainIdx].trainIdx!=static_cast<int>(k)) continue;
                const int index=offset+best.trainIdx;
                MapPoint* point=points[k];
                if(frame.mvpMapPoints[index] || occupied.count(point->mnId)) continue;
                if(!projectedCandidate(frame,point,camera,best.trainIdx,
                                       searchFactor,farPoints,farThreshold)) {
                    ++result.geometryRejected[camera]; continue;
                }
                frame.mvpMapPoints[index]=point;
                frame.mvbOutlier[index]=false;
                occupied.insert(point->mnId);
                result.seeded.push_back(index);
            }
        }
    }

    Result result;

private:
    typedef std::pair<int,unsigned long> Key;
    struct Entry { double timestamp=0.; cv::Mat descriptor; };
    std::map<Key,Entry> entries_;
    const void* owner_=nullptr;
    unsigned long mapId_=0, initialKeyFrameId_=0;
    double timestamp_=0.;
    bool initialized_=false;

    void expire() {
        for(auto it=entries_.begin();it!=entries_.end();)
            if(!std::isfinite(timestamp_) || timestamp_-it->second.timestamp>.5 ||
               timestamp_<it->second.timestamp) it=entries_.erase(it);
            else ++it;
    }
    bool compatible(const Frame& frame, Map* map) const {
        return map && map->GetId()==mapId_ && map->GetInitKFid()==initialKeyFrameId_ && std::isfinite(timestamp_) &&
            frame.mTimeStamp==timestamp_ && frame.Nleft>=0 && frame.Nright>=0 &&
            frame.N==frame.Nleft+frame.Nright && frame.mDescriptors.type()==CV_8U &&
            frame.mDescriptors.cols==32 && frame.mDescriptors.rows==frame.N &&
            frame.mvpMapPoints.size()==static_cast<size_t>(frame.N) &&
            frame.mvbOutlier.size()==static_cast<size_t>(frame.N);
    }
    struct Window : ORBmatcher { using ORBmatcher::RadiusByViewingCos; };
    static bool projectedCandidate(Frame& frame, MapPoint* point, int camera,
                                   int feature, float factor, bool far, float distance) {
        if(!frame.isInFrustum(point,.5f) ||
           !(camera ? point->mbTrackInViewR : point->mbTrackInView)) return false;
        // Keep the native local matcher's far-point policy as well.
        if(far && point->mTrackDepth>distance) return false;
        const int level=camera ? point->mnTrackScaleLevelR : point->mnTrackScaleLevel;
        if(level<0 || static_cast<size_t>(level)>=frame.mvScaleFactors.size()) return false;
        Window window;
        const float radius=window.RadiusByViewingCos(camera ? point->mTrackViewCosR : point->mTrackViewCos)
                           *factor*frame.mvScaleFactors[level];
        const auto indices=frame.GetFeaturesInArea(
            camera ? point->mTrackProjXR : point->mTrackProjX,
            camera ? point->mTrackProjYR : point->mTrackProjY,
            radius,level-1,level,camera!=0);
        return std::find(indices.begin(),indices.end(),static_cast<size_t>(feature))!=indices.end();
    }
};

} // namespace ORB_SLAM3
#endif
