// Real production camera models, keyframes, IMU preintegration and optimizer.
// Synthetic measurements isolate scale/lever-arm contracts, not feature quality.
#include "Atlas.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "Optimizer.h"
#include "G2oTypes.h"
#include "CameraModels/Fisheye624.h"
#include "Rig.h"
#include "Thirdparty/g2o/g2o/core/jacobian_workspace.h"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using namespace ORB_SLAM3;

struct Fixture {
    ORBVocabulary vocabulary;
    KeyFrameDatabase database;
    Atlas atlas;
    Fisheye624 cam0,cam1;
    ORBextractor extractor0,extractor1;
    cv::Mat texture;
    IMU::Calib calibration;
    std::vector<KeyFrame*> keys;
    std::vector<MapPoint*> points;
    std::vector<Sophus::SE3f,Eigen::aligned_allocator<Sophus::SE3f>> trueCameraPoses;
    int failures=0;
    Fixture(double trueScale, bool stationary=false, int count=36)
        : database(vocabulary),atlas(0),cam0(parameters()),cam1(parameters()),
          extractor0(100,1.2,8,20,7),extractor1(100,1.2,8,20,7),
          calibration(Sophus::SE3f(Eigen::Matrix3f::Identity(),Eigen::Vector3f(.08,-.025,.04)),.01,.08,.0001,.001) {
        cv::RNG random(12345);texture=cv::Mat(480,640,CV_8UC1);
        random.fill(texture,cv::RNG::UNIFORM,0,256);cam0.mvLappingArea={0,640};cam1.mvLappingArea={0,640};
        std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> truth;
        for(int j=0;j<count;++j) truth.emplace_back(-1.5+.5*(j%6),-.8+.3*(j/6),4.5+.25*(j%4));
        Eigen::Matrix3f rotation=Eigen::Matrix3f::Identity();
        Eigen::Vector3f position=Eigen::Vector3f::Zero(),velocity=Eigen::Vector3f::Zero(),gravity(0,0,-IMU::GRAVITY_VALUE);
        for(int i=0;i<13;++i) {
            IMU::Preintegrated* pi=nullptr;
            if(i) {
                pi=new IMU::Preintegrated(IMU::Bias(),calibration);
                for(int step=0;step<60;++step) {
                    const double time=(i-1)*.3+step*.005;
                    const Eigen::Vector3f acceleration=stationary?Eigen::Vector3f(0,0,IMU::GRAVITY_VALUE):
                        Eigen::Vector3f(.6*std::sin(1.7*time),.4*std::cos(1.3*time),IMU::GRAVITY_VALUE+.2*std::sin(2.1*time));
                    const Eigen::Vector3f omega=stationary?Eigen::Vector3f::Zero():Eigen::Vector3f(0,0,.18);
                    pi->IntegrateNewMeasurement(acceleration,omega,.005);
                }
                const float dt=pi->dT;
                position+=velocity*dt+.5f*gravity*dt*dt+rotation*pi->GetOriginalDeltaPosition();
                velocity+=gravity*dt+rotation*pi->GetOriginalDeltaVelocity();
                rotation=rotation*pi->GetOriginalDeltaRotation();
            }
            const Sophus::SE3f truthCamera(rotation,position+rotation*calibration.mTbc.translation());
            trueCameraPoses.push_back(truthCamera);
            Sophus::SE3f sourceCamera=truthCamera;sourceCamera.translation()/=trueScale;
            cv::Mat K=cam0.toK(),dist=cv::Mat::zeros(4,1,CV_32F);
            Sophus::SE3f T01(Eigen::Matrix3f::Identity(),Eigen::Vector3f(.14f,0,0));
            Frame f(texture,texture,10.+i*.3,&extractor0,&extractor1,&vocabulary,K,dist,33.6f,60.f,&cam0,&cam1,T01,nullptr,calibration);
            f.SetPose(sourceCamera.inverse());f.mnDataset=0;f.mNameFile="metric_inertial_contract";
            f.Nleft=count;f.Nright=count;f.N=2*count;f.mvKeys.clear();f.mvKeysRight.clear();
            for(int j=0;j<count;++j) {
                Eigen::Vector3f pc=truthCamera.inverse()*truth[j];
                const Eigen::Vector2f a=cam0.project(pc),b=cam1.project(f.GetRelativePoseTrl()*pc);
                f.mvKeys.emplace_back(a.x(),a.y(),1,0,1,0);f.mvKeysRight.emplace_back(b.x(),b.y(),1,0,1,0);
            }
            f.mvKeysUn=f.mvKeys;f.mDescriptors=cv::Mat::zeros(f.N,32,CV_8U);
            f.mvpMapPoints.assign(f.N,nullptr);f.mvbOutlier.assign(f.N,false);
            f.mvuRight.assign(count,-1);f.mvDepth.assign(count,-1);f.mvLeftToRightMatch.assign(count,-1);f.mvRightToLeftMatch.assign(count,-1);
            f.mBowVec[0]=1.;f.mFeatVec[0]={0};
            KeyFrame* k=new KeyFrame(f,map(),&database);map()->AddKeyFrame(k);
            k->mpImuPreintegrated=pi;k->SetVelocity(Eigen::Vector3f::Zero());k->SetNewBias(IMU::Bias());
            if(i){k->mPrevKF=keys.back();keys.back()->mNextKF=k;}
            keys.push_back(k);
            for(int j=0;j<count;++j) {
                if(!i){points.push_back(new MapPoint(truth[j]/trueScale,k,map()));map()->AddMapPoint(points.back());}
                k->AddMapPoint(points[j],j);points[j]->AddObservation(k,j);
                k->AddMapPoint(points[j],count+j);points[j]->AddObservation(k,count+j);
            }
        }
    }
    Map* map(){return atlas.GetCurrentMap();}
    static std::vector<float> parameters(){std::vector<float> p(16,0);p[0]=p[1]=240;p[2]=320;p[3]=240;p[4]=.015;p[5]=-.002;return p;}
    void check(bool ok,const std::string& name){std::cout<<"CHECK "<<(ok?"PASS ":"FAIL ")<<name<<std::endl;failures+=!ok;}
    std::string snapshot(){
        std::ostringstream out;out<<std::setprecision(17)<<map()->GetWorldFrameVersion()<<" "<<map()->GetMapChangeIndex();
        for(KeyFrame* k:keys){
            out<<k->GetPose().matrix()<<k->GetVelocity()<<k->GetGyroBias()<<k->GetAccBias();
            if(k->mpImuPreintegrated){auto* p=k->mpImuPreintegrated;auto b=p->GetUpdatedBias();out<<p->C<<p->dT<<b.bwx<<b.bwy<<b.bwz<<b.bax<<b.bay<<b.baz<<p->GetUpdatedDeltaRotation()<<p->GetUpdatedDeltaVelocity()<<p->GetUpdatedDeltaPosition();}
        }
        for(auto* p:points)out<<p->GetWorldPos()<<p->Observations();return out.str();
    }
};

struct InspectEdge : EdgeInertialGS {
    explicit InspectEdge(IMU::Preintegrated* p):EdgeInertialGS(p){}
    Eigen::MatrixXd jacobian(int i)const{return _jacobianOplus[i];}
};

int main(int argc,char** argv){
    if(argc!=2)return 2;cv::setNumThreads(1);Rig::PublishGlobals(false);const std::string name=argv[1];
    if(name=="jacobians" || name=="lever_arm"){
        Fixture f(.86);KeyFrame* a=f.keys[4],*b=f.keys[5];
        VertexPose p0(a),p1(b);VertexVelocity v0,v1;VertexGyroBias bg;VertexAccBias ba;
        v0.setEstimate(Eigen::Vector3d(.3,-.2,.1));v1.setEstimate(Eigen::Vector3d(.5,.1,-.1));
        bg.setEstimate(Eigen::Vector3d::Zero());ba.setEstimate(Eigen::Vector3d::Zero());
        VertexGDir gravity(Eigen::Matrix3d::Identity());VertexScale scale(.65);
        InspectEdge e(b->mpImuPreintegrated);
        g2o::OptimizableGraph::Vertex* vertices[]={&p0,&v0,&bg,&ba,&p1,&v1,&gravity,&scale};
        for(int i=0;i<8;++i)e.setVertex(i,vertices[i]);
        if(name=="jacobians"){
            g2o::JacobianWorkspace workspace;workspace.updateSize(&e);workspace.allocate();
            for(double s:{.65,1.,1.4}){
                scale.setEstimate(s);e.computeError();
                e.g2o::BaseMultiEdge<9,Vector9d>::linearizeOplus(workspace);
                double maximum=0;
                for(int i=0;i<8;++i){
                    const Eigen::MatrixXd analytic=e.jacobian(i);
                    for(int d=0;d<vertices[i]->dimension();++d){
                        // Preintegrated::GetDelta* returns float values and
                        // normalizes rotations in float. Bias differences
                        // below that resolution falsely appear to be zero.
                        const double epsilon=(i==2 || i==3)?1e-3:1e-6;
                        Eigen::VectorXd step=Eigen::VectorXd::Zero(vertices[i]->dimension());step[d]=epsilon;
                        vertices[i]->push();vertices[i]->oplus(step.data());e.computeError();const Vector9d plus=e.error();vertices[i]->pop();
                        step[d]=-epsilon;vertices[i]->push();vertices[i]->oplus(step.data());e.computeError();const Vector9d minus=e.error();vertices[i]->pop();
                        const Eigen::VectorXd numerical=(plus-minus)/(2*epsilon);
                        if((numerical-analytic.col(d)).cwiseAbs().maxCoeff()>.005)
                            std::cout<<"JAC_DETAIL vertex="<<i<<" coordinate="<<d<<" numerical="<<numerical.transpose()<<" analytic="<<analytic.col(d).transpose()<<std::endl;
                        maximum=std::max(maximum,(numerical-analytic.col(d)).cwiseAbs().maxCoeff());
                    }
                }
                std::cout<<"JACOBIAN scale="<<s<<" max_absolute_error="<<maximum<<std::endl;
                f.check(maximum<.005,"all eight production edge Jacobians at scale "+std::to_string(s));
            }
        }else{
            for(double s:{.65,1.,1.4}){
                scale.setEstimate(s);e.computeError();const Vector9d predicted=e.error();
                const auto original0=p0.estimate(),original1=p1.estimate();const auto vv0=v0.estimate(),vv1=v1.estimate();
                auto state0=original0,state1=original1;
                state0.twb=s*original0.twb+(s-1)*original0.Rwb*original0.tbc[0];
                state1.twb=s*original1.twb+(s-1)*original1.Rwb*original1.tbc[0];
                p0.setEstimate(state0);p1.setEstimate(state1);v0.setEstimate(s*vv0);v1.setEstimate(s*vv1);scale.setEstimate(1);
                e.computeError();double discrepancy=(e.error()-predicted).norm();
                std::cout<<"LEVER_ARM scale="<<s<<" residual_discrepancy="<<discrepancy<<std::endl;
                f.check(discrepancy<1e-9,"residual equals metric camera-centre scale contract");
                p0.setEstimate(original0);p1.setEstimate(original1);v0.setEstimate(vv0);v1.setEstimate(vv1);
            }
            a->SetVelocity(Eigen::Vector3f(.3,-.2,.1));const auto before=a->GetPoseInverse();const float s=.86;
            const Eigen::Vector3f expected=s*before.translation()+before.rotationMatrix()*a->mImuCalib.mTcb.translation();
            f.map()->ApplyScaledRotation(Sophus::SE3f(),s,true);
            f.check((a->GetImuPosition()-expected).norm()<1e-6,"production Map::ApplyScaledRotation preserves metric lever arm");
        }
        return f.failures?1:0;
    }
#ifndef METRIC_EDGE_BASELINE_ONLY
    const bool joint=name.find("joint_")==0;
    const bool stationary=name=="stationary" || name=="joint_stationary",weak=name=="weak_stereo";
    const double target=name=="scale_large"?1.15:.86;
    Fixture f(target,stationary,weak?5:36);
    if(name=="joint_distorted" || name=="joint_boundary_conflict") {
        for(size_t i=0;i<f.points.size();++i) {
            Eigen::Vector3f p=f.points[i]->GetWorldPos();p.z()*=1.0+.04*std::sin(i*.7);f.points[i]->SetWorldPos(p);
        }
        for(size_t i=1;i<f.keys.size();++i) {
            Sophus::SE3f pose=f.keys[i]->GetPose();
            pose.translation()+=Eigen::Vector3f(.006*std::sin(i),.004*std::cos(i),.003*std::sin(i*.4));
            f.keys[i]->SetPose(pose);
        }
    }
    const auto before=f.snapshot();
    MetricInertialInitOptions options;
    options.jointRefinement=joint;
    if(name=="joint_boundary" || name=="joint_boundary_conflict") {options.maximumKeyFrames=10;options.minimumDurationSeconds=2.5;}
    const auto r=Optimizer::ProposeMetricInertialInitialization(f.map(),Eigen::Matrix3d::Identity(),options);
    std::cout<<std::setprecision(12)<<"PROPOSAL accepted="<<r.accepted<<" reason="<<r.reason<<" scale="<<r.scale<<" expected="<<target
             <<" sigma="<<r.conditionalLogScaleStdDev<<" imu_sigma="<<r.imuOnlyLogScaleStdDev<<" stereo="<<r.stereoLandmarks
             <<" error_px="<<r.finalStereoMedianPixels<<" initial_cost="<<r.initialCost<<" final_cost="<<r.finalCost;
    if(joint) std::cout<<" joint="<<r.jointRefined<<" native_median0="<<r.finalVisualMedianPixels[0]<<" native_median1="<<r.finalVisualMedianPixels[1]
        <<" joint_sigma="<<r.jointLogBaselineStdDev<<" rank="<<r.jointHessianRank<<" dimension="<<r.jointHessianDimension<<" boundary="<<r.boundaryStates.size();
    std::cout<<std::endl;
    f.check(before==f.snapshot(),"proposal leaves all source poses/points/velocities/biases/preintegrations unchanged");
    if(stationary||weak){f.check(!r.accepted,"unsupported metric initialization is rejected");}
    else if(joint) {
        f.check(r.accepted && r.jointRefined,"joint geometry window accepted");
        f.check(!r.points.empty(),"joint proposal contains landmark updates");
        f.check(r.finalVisualMedianPixels[0]<.15 && r.finalVisualMedianPixels[1]<.15,"both native camera residuals repaired jointly");
        f.check(std::isfinite(r.jointLogBaselineStdDev) && r.jointLogBaselineStdDev<.05,"joint metric baseline is observable after landmark marginalization");
        if(r.accepted) {
            const auto& a=r.states.front();const auto& b=r.states.back();
            size_t ai=std::find(f.keys.begin(),f.keys.end(),a.keyFrame)-f.keys.begin(),bi=std::find(f.keys.begin(),f.keys.end(),b.keyFrame)-f.keys.begin();
            const double actual=(b.optimizedPose.inverse().translation()-a.optimizedPose.inverse().translation()).norm();
            const double expected=(f.trueCameraPoses[bi].translation()-f.trueCameraPoses[ai].translation()).norm();
            f.check(std::abs(actual/expected-1)<.02,"joint output camera baseline recovers metric length within 2 percent");
        }
        if(name=="joint_boundary" || name=="joint_boundary_conflict") f.check(r.boundaryStates.size()==3,"all three outside-window observer keyframes are retained as fixed boundary factors");
    } else {
        f.check(r.accepted,"well-conditioned stereo-inertial window accepted");
        f.check(std::abs(r.scale-target)<.003,"known metric scale recovered within 0.3 percent");
        f.check(r.finalStereoMedianPixels<.03,"native fisheye stereo reprojections agree after scale correction");
        f.check(r.bg.norm()<.005 && r.ba.norm()<.02,"zero synthetic sensor biases recovered");
    }
    return f.failures?1:0;
#else
    return 2;
#endif
}
