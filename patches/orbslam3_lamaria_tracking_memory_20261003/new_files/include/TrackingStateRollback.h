#ifndef ORB_SLAM3_TRACKING_STATE_ROLLBACK_H
#define ORB_SLAM3_TRACKING_STATE_ROLLBACK_H
#include "Frame.h"
#include "G2oTypes.h"
namespace ORB_SLAM3 {
// A rejected visual optimizer may already have consumed the previous marginal
// prior and created a new one. Its linearization belongs to the rejected pose.
// Restore the propagated state and invalidate that prior; the next optimizer
// must rebuild from its valid keyframe/preintegration, never recycle rejected
// image information as inertial certainty.
inline Eigen::Vector3f TransportFrameVelocity(const Eigen::Vector3f& frameVelocity,const Eigen::Vector3f& oldAnchorVelocity,const Eigen::Vector3f& newAnchorVelocity,const Eigen::Matrix3f& worldCorrection) {
 return newAnchorVelocity+worldCorrection*(frameVelocity-oldAnchorVelocity);
}
struct TrackingStateRollback {
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 Sophus::SE3f pose;Eigen::Vector3f velocity;IMU::Bias bias;
 explicit TrackingStateRollback(Frame& frame):pose(frame.GetPose()),velocity(frame.GetVelocity()),bias(frame.mImuBias){}
 // A provisional relatch refreshes grace only after local-map acceptance.
 void finishRelatch(Frame& frame,bool accepted,double& lostSince)const{
  if(accepted)lostSince=frame.mTimeStamp;else restore(frame);
 }
 void restore(Frame& frame)const{
  frame.SetPose(pose);frame.SetVelocity(velocity);frame.mImuBias=bias;
  if(frame.mpcpi){delete frame.mpcpi;frame.mpcpi=nullptr;}
  for(size_t i=0;i<frame.mvpMapPoints.size();++i)if(frame.mvpMapPoints[i] && i<frame.mvbOutlier.size())frame.mvbOutlier[i]=true;
 }
};
}
#endif
