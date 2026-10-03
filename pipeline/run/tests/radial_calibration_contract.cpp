// Production-linked test of the bounded native Fisheye624 calibration edge.
// Synthetic ground truth is used only in this fixture, never in the estimator.
#include "G2oRadialCalibration.h"
#include "Settings.h"
#include "System.h"
#include "ORBextractor.h"
#include "Rig.h"
#include "Map.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace ORB_SLAM3;
int failures=0;
void check(bool ok,const std::string& label){
    std::cout<<"CHECK "<<(ok?"PASS ":"FAIL ")<<label<<std::endl;
    failures+=!ok;
}
double median(std::vector<double> a){std::sort(a.begin(),a.end());return a.empty()?1e9:a[a.size()/2];}
ImuCamPose makePose(Fisheye624* first,Fisheye624* second){
    ImuCamPose p;p.its=0;p.bf=0;
    p.Rwb=(Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.1,Eigen::Vector3d::UnitX())).toRotationMatrix();
    p.twb=Eigen::Vector3d(.2,-.1,.4);p.Rwb0=p.Rwb;p.DR.setIdentity();
    p.Rbc={Eigen::AngleAxisd(.13,Eigen::Vector3d::UnitX()).toRotationMatrix(),Eigen::AngleAxisd(1.25,Eigen::Vector3d::UnitY()).toRotationMatrix()};
    p.tbc={Eigen::Vector3d(.01,.03,-.02),Eigen::Vector3d(.14,.01,.02)};
    p.pCamera={first,second};
    for(int i=0;i<2;++i){p.Rcb.push_back(p.Rbc[i].transpose());p.tcb.push_back(-p.Rcb[i]*p.tbc[i]);p.Rcw.push_back(p.Rcb[i]*p.Rwb.transpose());p.tcw.push_back(p.tcb[i]-p.Rcw[i]*p.twb);}
    return p;
}

void derivatives(Fisheye624* camera[2],const ImuCamPose& state){
    double pointPoseError=0,radialError=0,priorError=0;
    for(int cam=0;cam<2;++cam){
        VertexRadialCalibration radial(*camera[cam],Eigen::Vector2d(.02,.01));
        radial.setDelta(Eigen::Vector2d(.004,-.001));
        const Eigen::Vector2d q=radial.estimate();
        VertexPose pose;pose.setEstimate(state);
        g2o::VertexSBAPointXYZ point;
        EdgeMonoRadial edge(cam);edge.setVertex(0,&point);edge.setVertex(1,&pose);edge.setVertex(2,&radial);edge.setMeasurement(Eigen::Vector2d(200,220));edge.setInformation(Eigen::Matrix2d::Identity());
        for(const auto& pc:{Eigen::Vector3d(0,0,3),Eigen::Vector3d(.4,-.2,2),Eigen::Vector3d(3,1.2,.8),Eigen::Vector3d(.6,.3,-.3)}){
            point.setEstimate(state.Rcw[cam].transpose()*(pc-state.tcw[cam]));
            const auto jac=edge.GetJacobian();
            check((edge.GetHessian()-jac.transpose()*jac).norm()<1e-9,"Hessian uses point/pose/radial layout before optimizer allocation");
            const Eigen::Vector3d original=point.estimate();
            for(int k=0;k<9;++k){
                const double epsilon=1e-6;Eigen::Vector2d plus,minus;
                if(k<3){Eigen::Vector3d p=original;p[k]+=epsilon;point.setEstimate(p);edge.computeError();plus=edge.error();p[k]-=2*epsilon;point.setEstimate(p);edge.computeError();minus=edge.error();point.setEstimate(original);}
                else{double u[6]={0,0,0,0,0,0};u[k-3]=epsilon;pose.setEstimate(state);pose.oplus(u);edge.computeError();plus=edge.error();u[k-3]=-epsilon;pose.setEstimate(state);pose.oplus(u);edge.computeError();minus=edge.error();pose.setEstimate(state);}
                pointPoseError=std::max(pointPoseError,((plus-minus)/(2*epsilon)-jac.col(k)).norm()/std::max(1.,jac.col(k).norm()));
            }
            // Check the analytical native coefficient derivative against the
            // exact production projector. Correct for float storage rounding.
            for(int k=0;k<2;++k){
                Fisheye624 plus(*radial.camera()),minus(*radial.camera());
                const double centre=plus.getParameter(4+k),epsilon=1e-4;
                plus.setParameter(float(centre+epsilon),4+k);minus.setParameter(float(centre-epsilon),4+k);
                const double gap=double(plus.getParameter(4+k))-minus.getParameter(4+k);
                const Eigen::Vector2d numeric=-(plus.project(pc)-minus.project(pc))/gap*radial.derivative()[k];
                radialError=std::max(radialError,(numeric-jac.col(9+k)).norm()/std::max(1.,jac.col(9+k).norm()));
            }
        }
        EdgePriorRadial prior;prior.setVertex(0,&radial);prior.setInformation(Eigen::Matrix2d::Identity());prior.computeError();
        check((prior.error()-radial.delta()).norm()<1e-15,"prior is in physical radial-coefficient units");
        for(int k=0;k<2;++k){Eigen::Vector2d plus=q,minus=q;plus[k]+=1e-6;minus[k]-=1e-6;radial.setEstimate(plus);prior.computeError();Eigen::Vector2d a=prior.error();radial.setEstimate(minus);prior.computeError();Eigen::Vector2d b=prior.error();radial.setEstimate(q);priorError=std::max(priorError,std::abs((a[k]-b[k])/2e-6-radial.derivative()[k]));}
        const float before=radial.camera()->getParameter(4);radial.push();radial.setDelta(Eigen::Vector2d(-.004,.002));(void)radial.camera();radial.pop();
        check(radial.camera()->getParameter(4)==before,"LM push/pop restores private camera projection parameters");
        radial.setEstimate(Eigen::Vector2d(100,-100));check((radial.delta().array().abs()<=radial.bounds().array()).all(),"large optimizer steps cannot exceed physical bounds");
    }
    std::cout<<"JACOBIAN point_pose_relative="<<pointPoseError<<" radial_relative="<<radialError<<" prior_absolute="<<priorError<<std::endl;
    check(pointPoseError<2e-5&&radialError<2e-5&&priorError<1e-9,"analytic point/pose/radial/prior derivatives match production numerical checks");
}

void recovery(Fisheye624* source[2],const ImuCamPose& state){
    g2o::SparseOptimizer optimizer;
    auto* linear=new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(new g2o::BlockSolverX(linear)));
    VertexPose* pose=new VertexPose();pose->setEstimate(state);pose->setId(0);pose->setFixed(true);optimizer.addVertex(pose);
    VertexRadialCalibration* radial[2];
    const Eigen::Vector2d truthDelta[2]={Eigen::Vector2d(.007,-.003),Eigen::Vector2d(-.005,.0025)};
    struct Sample {int cam;Eigen::Vector3d pc;Eigen::Vector2d pixel;bool outer;};
    std::vector<Sample,Eigen::aligned_allocator<Sample>> heldout;
    std::vector<float> saved[2];
    for(int c=0;c<2;++c)for(int i=0;i<16;++i)saved[c].push_back(source[c]->getParameter(i));
    int id=3,training=0,outer=0;
    for(int cam=0;cam<2;++cam){
        radial[cam]=new VertexRadialCalibration(*source[cam],Eigen::Vector2d(.02,.01));radial[cam]->setId(1+cam);optimizer.addVertex(radial[cam]);
        auto* prior=new EdgePriorRadial();prior->setVertex(0,radial[cam]);prior->setInformation(Eigen::Vector2d(1./(.005*.005),1./(.0025*.0025)).asDiagonal());optimizer.addEdge(prior);
        Fisheye624 truth(*source[cam]);for(int k=0;k<2;++k)truth.setParameter(saved[cam][4+k]+truthDelta[cam][k],4+k);
        int accepted=0;
        for(int radius=20;radius<=320;radius+=15)for(int a=0;a<72;++a){
            const double angle=2*M_PI*a/72;cv::Point2f pixel(source[cam]->getParameter(2)+radius*std::cos(angle),source[cam]->getParameter(3)+radius*std::sin(angle));cv::Point3f ray;
            if(!source[cam]->tryUnproject(pixel,ray))continue;
            const double depth=2+.01*((radius+a)%200);const Eigen::Vector3d pc=depth*Eigen::Vector3d(ray.x,ray.y,ray.z);
            const Eigen::Vector2d measured=truth.project(pc);
            if((accepted++%4)==0){heldout.push_back({cam,pc,measured,radius>=250});continue;}
            auto* point=new g2o::VertexSBAPointXYZ();point->setId(id++);point->setFixed(true);point->setEstimate(state.Rcw[cam].transpose()*(pc-state.tcw[cam]));optimizer.addVertex(point);
            auto* edge=new EdgeMonoRadial(cam);edge->setVertex(0,point);edge->setVertex(1,pose);edge->setVertex(2,radial[cam]);edge->setMeasurement(measured);edge->setInformation(Eigen::Matrix2d::Identity());optimizer.addEdge(edge);++training;
        }
    }
    std::vector<double> before,after,outerBefore,outerAfter;
    for(const auto& sample:heldout){const double error=(radial[sample.cam]->camera()->project(sample.pc)-sample.pixel).norm();before.push_back(error);if(sample.outer){outerBefore.push_back(error);++outer;}}
    optimizer.initializeOptimization();optimizer.computeActiveErrors();const double cost0=optimizer.activeChi2();optimizer.optimize(30);optimizer.computeActiveErrors();const double cost1=optimizer.activeChi2();
    bool recovered=true,unchanged=true,inverse=true;double maxCoefficientError=0;
    for(int cam=0;cam<2;++cam){
        const Eigen::Vector2d recoveredDelta=radial[cam]->delta();maxCoefficientError=std::max(maxCoefficientError,(recoveredDelta-truthDelta[cam]).cwiseAbs().maxCoeff());
        size_t n=0;double error=0;inverse &= ValidateRadialCameraInverse(*source[cam],*radial[cam]->camera(),640,480,&n,&error);
        std::cout<<"RECOVERY cam="<<cam<<" truth="<<truthDelta[cam].transpose()<<" recovered="<<recoveredDelta.transpose()<<" inverse_valid="<<n<<" inverse_max_px="<<error<<std::endl;
        for(int k=0;k<16;++k) unchanged &= source[cam]->getParameter(k)==saved[cam][k];
        for(int k=0;k<16;++k)if(k!=4&&k!=5)unchanged &= radial[cam]->camera()->getParameter(k)==saved[cam][k];
    }
    for(const auto& sample:heldout){const double error=(radial[sample.cam]->camera()->project(sample.pc)-sample.pixel).norm();after.push_back(error);if(sample.outer)outerAfter.push_back(error);}
    std::cout<<"RECOVERY train="<<training<<" heldout="<<heldout.size()<<" outer="<<outer<<" cost_before="<<cost0<<" cost_after="<<cost1<<" coefficient_error="<<maxCoefficientError<<" heldout_median_before="<<median(before)<<" after="<<median(after)<<" outer_median_before="<<median(outerBefore)<<" after="<<median(outerAfter)<<std::endl;
    check(training>1000&&outer>50&&heldout.size()>300,"synthetic recovery covers both native lenses and held-out valid periphery");
    check(maxCoefficientError<5e-4&&cost1<cost0*.05&&median(after)<.03&&median(outerAfter)<.05,"bounded native-edge optimization recovers injected radial error on unused samples");
    check(unchanged,"source camera and all non-radial parameters remain bitwise unchanged");
    check(inverse,"accepted private cameras preserve source-valid inverse domain");
}

#ifdef RADIAL_TEST_STEREO_REFRESH
void refreshCaches(Fisheye624* original[2]){
    Fisheye624 left(*original[0]),right(*original[1]);
    cv::Mat picture(480,640,CV_8UC1);cv::RNG random(1234);random.fill(picture,cv::RNG::UNIFORM,0,256);
    ORBextractor extractor0(100,1.2,8,20,7),extractor1(100,1.2,8,20,7);ORBVocabulary vocabulary;
    cv::Mat K=left.toK(),dist=cv::Mat::zeros(4,1,CV_32F);
    Sophus::SE3f T01(Eigen::AngleAxisf(.35f,Eigen::Vector3f::UnitY()).toRotationMatrix(),Eigen::Vector3f(.137f,0,0));
    Rig::PublishGlobals(false);
    Frame frame(picture,picture,1.,&extractor0,&extractor1,&vocabulary,K,dist,30,10,&left,&right,T01);
    check(frame.N>0,"stereo refresh fixture uses a fully initialized production Frame");
    frame.Nleft=frame.Nright=3;frame.N=6;
    const Eigen::Vector3f p0(1.5f,.2f,3.f),p1=T01.inverse()*p0;
    const Eigen::Vector2f z0=left.project(p0),z1=right.project(p1);
    frame.mvKeys={cv::KeyPoint(z0.x(),z0.y(),1),cv::KeyPoint(-10,-10,1),cv::KeyPoint(z0.x()+3,z0.y()+2,1)};
    frame.mvKeysRight={cv::KeyPoint(z1.x(),z1.y(),1),cv::KeyPoint(z1.x(),z1.y(),1),cv::KeyPoint(z1.x()+3,z1.y()+2,1)};
    frame.mvKeysUn=frame.mvKeys;frame.mvLeftToRightMatch={0,1,2};frame.mvRightToLeftMatch={0,1,-1};
    frame.mvDepth={3,4,5};frame.mvuRight.assign(3,-1);frame.mvStereo3Dpoints.assign(3,Eigen::Vector3f(9,8,7));
    frame.mDescriptors=cv::Mat(6,32,CV_8U);for(int i=0;i<6;++i)frame.mDescriptors.row(i).setTo(13*i+1);
    frame.mDescriptorsRight=frame.mDescriptors.rowRange(3,6).clone();frame.mvbOutlier.assign(6,false);
    MapPoint sentinel;frame.mvpMapPoints.assign(6,nullptr);frame.mvpMapPoints[0]=frame.mvpMapPoints[3]=&sentinel;
    frame.SetPose(Sophus::SE3f());
    for(int x=0;x<FRAME_GRID_COLS;++x)for(int y=0;y<FRAME_GRID_ROWS;++y){frame.mGrid[x][y].clear();frame.mGridRight[x][y].clear();}
    frame.mGrid[0][0]={0,1,2};frame.mGridRight[0][0]={0,1,2};
    const cv::Mat descriptors=frame.mDescriptors.clone();const auto associations=frame.mvpMapPoints;
    Map map;KeyFrame keyframe(frame,&map,nullptr);const auto keyAssociations=keyframe.GetMapPointMatches();
    Eigen::Vector3f beforePoint,expectedPoint;
    const float before=left.TriangulateMatches(&right,frame.mvKeys[0],frame.mvKeysRight[0],T01.rotationMatrix(),T01.translation(),1,1,beforePoint);
    left.setParameter(left.getParameter(4)+.001f,4);right.setParameter(right.getParameter(5)-.0005f,5);
    const float expected=left.TriangulateMatches(&right,frame.mvKeys[0],frame.mvKeysRight[0],T01.rotationMatrix(),T01.translation(),1,1,expectedPoint);
    check(before>0&&expected>0&&std::abs(expected-before)>1e-5,"radial change alters production stereo geometry for retained pair");
    frame.RefreshStereoGeometryAfterCalibration();keyframe.RefreshStereoGeometryAfterCalibration();
    check(std::abs(frame.mvDepth[0]-expected)<1e-6&&(frame.mvStereo3Dpoints[0]-expectedPoint).norm()<1e-6&&std::abs(keyframe.mvDepth[0]-expected)<1e-6,"Frame and KeyFrame cache refresh equals exact new-calibration triangulation");
    check(frame.mvLeftToRightMatch==std::vector<int>({0,-1,-1})&&frame.mvRightToLeftMatch==std::vector<int>({0,-1,-1})&&keyframe.mvLeftToRightMatch==frame.mvLeftToRightMatch&&keyframe.mvRightToLeftMatch==frame.mvRightToLeftMatch&&frame.mvDepth[1]==-1&&keyframe.mvDepth[1]==-1&&frame.mvStereo3Dpoints[1].isZero(),"invalid and non-reciprocal stereo pairs clear both directions and stale depths");
    check(frame.N==6&&frame.Nleft==3&&frame.Nright==3&&frame.mnCloseMPs==1&&frame.mvpMapPoints==associations&&keyframe.GetMapPointMatches()==keyAssociations&&cv::norm(frame.mDescriptors,descriptors,cv::NORM_INF)==0&&cv::norm(keyframe.mDescriptors,descriptors,cv::NORM_INF)==0&&frame.mvKeys[0].pt==cv::Point2f(z0.x(),z0.y()),"refresh preserves feature slots, descriptors and map associations");
    Frame unused;unused.RefreshStereoGeometryAfterCalibration();check(unused.N==0&&unused.Nleft==-1,"default unused Frame refresh is safe");
    Frame empty;empty.N=empty.Nleft=3;empty.Nright=0;empty.mvLeftToRightMatch={0,1,2};empty.mvDepth={3,4,5};empty.RefreshStereoGeometryAfterCalibration();
    check(empty.mvLeftToRightMatch==std::vector<int>({-1,-1,-1})&&empty.mvRightToLeftMatch.empty()&&empty.mvDepth==std::vector<float>({-1,-1,-1})&&empty.mnCloseMPs==0,"empty-camera refresh clears stale stereo geometry without reading unset pose");
}
#endif

int main(int argc,char** argv){
    if(argc!=2)return 2;std::cout<<std::setprecision(12);cv::setNumThreads(1);
    Settings settings(argv[1],System::IMU_STEREO);
    Fisheye624* camera[2]={dynamic_cast<Fisheye624*>(settings.camera1()),dynamic_cast<Fisheye624*>(settings.camera2())};
    if(!camera[0]||!camera[1])return 3;
    const ImuCamPose pose=makePose(camera[0],camera[1]);derivatives(camera,pose);recovery(camera,pose);
#ifdef RADIAL_TEST_STEREO_REFRESH
    refreshCaches(camera);
#endif
    std::cout<<"RESULT failures="<<failures<<std::endl;return failures?1:0;
}
