#include "Frame.h"
#include "MapPoint.h"
#include "TrackingStateRollback.h"
#include <iostream>

using namespace ORB_SLAM3;
int main(int, char**) {
    int failures=0;
    auto check=[&](bool ok,const char* message){std::cout<<"CHECK "<<(ok?"PASS ":"FAIL ")<<message<<std::endl;if(!ok)++failures;};
    Frame frame;
    const Sophus::SE3f Tbc(Eigen::AngleAxisf(.4f,Eigen::Vector3f::UnitY()).toRotationMatrix(),Eigen::Vector3f(.05f,-.03f,.08f));
    frame.mImuCalib=IMU::Calib(Tbc,.001f,.01f,.0001f,.001f);
    const Eigen::Matrix3f R=Eigen::AngleAxisf(.2f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Vector3f p(1,2,3),v(.4f,.5f,.6f);
    const IMU::Bias bias(.01,.02,.03,.001,.002,.003);
    frame.SetImuPoseVelocity(R,p,v);frame.mImuBias=bias;
    // Production PredictStateIMU captures here, before TryCoastRelatch.
    const TrackingStateRollback prediction(frame);
    MapPoint point;frame.mvpMapPoints={&point};frame.mvbOutlier={false};
    frame.SetPose(Sophus::SE3f(R,Eigen::Vector3f(14,-5,8))); // visual relatch hypothesis
    const TrackingStateRollback tooLate(frame); // old boundary, demonstrably different
    frame.SetImuPoseVelocity(Eigen::Matrix3f::Identity(),Eigen::Vector3f(2,3,4),Eigen::Vector3f(8,9,10));
    frame.mImuBias=IMU::Bias(.5,.4,.3,.2,.1,.6);
    frame.mpcpi=new ConstraintPoseImu(Eigen::Matrix3d::Identity(),Eigen::Vector3d(2,3,4),Eigen::Vector3d(8,9,10),Eigen::Vector3d(.2,.1,.6),Eigen::Vector3d(.5,.4,.3),Matrix15d::Identity());
    prediction.finish(frame,false); // same completion policy invoked by TrackLocalMap
    check((frame.GetImuPosition()-p).norm()<1e-6 && (frame.GetImuRotation()-R).norm()<1e-6,
          "rejected relatch plus optimization exports the original propagated body pose");
    check((frame.GetVelocity()-v).norm()<1e-6 && frame.mImuBias.bax==bias.bax && frame.mImuBias.bwz==bias.bwz,
          "rejected relatch preserves coherent propagated velocity and bias");
    check(frame.mpcpi==nullptr && frame.mvbOutlier[0],
          "rejected relatch cannot retain optimizer marginal prior or trusted feature associations");
    check((frame.GetPose().matrix()-tooLate.pose.matrix()).norm()>1.,
          "regression distinguishes pre-relatch snapshot from the faulty optimizer-entry snapshot");
    frame.SetImuPoseVelocity(R,Eigen::Vector3f(1.1f,2.1f,3.1f),Eigen::Vector3f(.7f,.8f,.9f));
    frame.mImuBias=IMU::Bias(.04,.05,.06,.004,.005,.006);
    frame.mpcpi=new ConstraintPoseImu(R.cast<double>(),Eigen::Vector3d(1.1,2.1,3.1),Eigen::Vector3d(.7,.8,.9),Eigen::Vector3d(.004,.005,.006),Eigen::Vector3d(.04,.05,.06),Matrix15d::Identity());
    const auto acceptedPose=frame.GetPose();const auto acceptedVelocity=frame.GetVelocity();auto* acceptedPrior=frame.mpcpi;
    prediction.finish(frame,true);
    check((frame.GetPose().matrix()-acceptedPose.matrix()).norm()==0 && (frame.GetVelocity()-acceptedVelocity).norm()==0 && frame.mImuBias.bax==.04f,
          "successful visual-inertial optimization retains its accepted pose velocity and bias");
    check(frame.mpcpi==acceptedPrior,"successful visual-inertial update keeps its matching marginal prior");
    delete frame.mpcpi;frame.mpcpi=nullptr;
    std::cout<<"RESULT failures="<<failures<<std::endl;
    return failures?1:0;
}
