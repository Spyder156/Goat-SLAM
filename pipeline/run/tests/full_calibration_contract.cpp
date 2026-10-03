#define main old_radial_contract_main
#include "radial_calibration_contract.cpp"
#undef main
#include "G2oFullCalibration.h"
#include "TrackingStateRollback.h"

void fullDerivative(Fisheye624* cams[2],const ImuCamPose& state){
 double worst=0,geometry=0;
 for(int c=0;c<2;++c){
  std::vector<float> factory;for(int j=0;j<16;++j)factory.push_back(cams[c]->getParameter(j));
  VertexFullCalibration v(*cams[c],factory);IntrinsicVector d=v.bounds()*.02;v.setDelta(d);
  VertexPose pose;pose.setEstimate(state);g2o::VertexSBAPointXYZ point;EdgeMonoFullCalibration edge(c);
  edge.setVertex(0,&point);edge.setVertex(1,&pose);edge.setVertex(2,&v);edge.setMeasurement(Eigen::Vector2d(123,234));
  for(auto pc:{Eigen::Vector3d(0,0,3),Eigen::Vector3d(.3,-.6,2),Eigen::Vector3d(4,1,.9),Eigen::Vector3d(1,.3,-.2)}){
   point.setEstimate(state.Rcw[c].transpose()*(pc-state.tcw[c]));auto J=edge.GetJacobian();
   for(int j=0;j<15;++j){
    Fisheye624 plus(*v.camera()),minus(*v.camera());int k=j==0?0:j+1;double center=plus.getParameter(k),eps=j<3?1e-2:1e-7;
    plus.setParameter(center+eps,k);minus.setParameter(center-eps,k);if(j==0){plus.setParameter(plus.getParameter(0),1);minus.setParameter(minus.getParameter(0),1);}
    double gap=double(plus.getParameter(k))-minus.getParameter(k);Eigen::Vector2d numeric=-(plus.project(pc)-minus.project(pc))/gap*v.derivative()[j];
    worst=std::max(worst,(numeric-J.col(9+j)).norm()/std::max(1.,J.col(9+j).norm()));
   }
   auto original=point.estimate();for(int j=0;j<9;++j){
    Eigen::Vector2d a,b;double eps=1e-6;
    if(j<3){auto x=original;x[j]+=eps;point.setEstimate(x);edge.computeError();a=edge.error();x[j]-=2*eps;point.setEstimate(x);edge.computeError();b=edge.error();point.setEstimate(original);}
    else{double u[6]={};u[j-3]=eps;pose.setEstimate(state);pose.oplus(u);edge.computeError();a=edge.error();u[j-3]=-eps;pose.setEstimate(state);pose.oplus(u);edge.computeError();b=edge.error();pose.setEstimate(state);}
    geometry=std::max(geometry,((a-b)/(2*eps)-J.col(j)).norm()/std::max(1.,J.col(j).norm()));
   }
  }
  const auto original=v.delta();v.push();double enormous[15];for(double& x:enormous)x=1e6;v.oplus(enormous);
  check(((v.delta()-original).array().abs()<=v.step().array()*1.00001).all(),"every intrinsic step is physically bounded");
  for(int n=0;n<100;++n)v.oplus(enormous);
  check((v.delta().array().abs()<v.bounds().array()).all(),"repeated updates respect fixed factory lifetime bounds");
  v.pop();check((v.delta()-original).norm()<1e-12,"LM rollback restores all 15 intrinsic parameters");
  bool intact=true;for(int j=0;j<16;++j)intact &= cams[c]->getParameter(j)==factory[j];check(intact,"private optimizer never mutates source camera");
  check(v.camera()->getParameter(0)==v.camera()->getParameter(1),"focal value shared only within each physical camera");
 }
 std::cout<<"JACOBIAN full_intrinsics="<<worst<<" point_pose="<<geometry<<std::endl;
 check(worst<2e-5 && geometry<2e-5,"all native coefficient/point/pose derivatives match exact production projector");
}

void rollback(){
 Frame f;f.mImuCalib=IMU::Calib();f.SetPose(Sophus::SE3f());f.SetVelocity(Eigen::Vector3f(1,2,3));f.mImuBias=IMU::Bias(.01,.02,.03,.001,.002,.003);
 MapPoint fake;f.mvpMapPoints={&fake,nullptr};f.mvbOutlier={false,false};TrackingStateRollback snap(f);
 f.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),Eigen::Vector3f(22,3,1)));f.SetVelocity(Eigen::Vector3f(9,8,7));f.mImuBias=IMU::Bias(.3,.3,.3,.1,.1,.1);
 f.mpcpi=new ConstraintPoseImu(Eigen::Matrix3d::Identity(),Eigen::Vector3d(22,3,1),Eigen::Vector3d(9,8,7),Eigen::Vector3d(.1,.1,.1),Eigen::Vector3d(.3,.3,.3),Matrix15d::Identity());
 snap.restore(f);
 check((f.GetPose().matrix()-snap.pose.matrix()).norm()<1e-7 && (f.GetVelocity()-snap.velocity).norm()<1e-7 && f.mImuBias.bax==snap.bias.bax&&f.mImuBias.bwz==snap.bias.bwz,"rejected visual update restores pose velocity and bias together");
 check(f.mpcpi==nullptr && f.mvbOutlier[0]&&!f.mvbOutlier[1],"rejected marginal prior is removed and image associations cannot enter keyframes");
 snap.restore(f);check(f.mpcpi==nullptr,"repeated rollback is safe and never revives freed priors");
}
void refreshFocalCaches(Fisheye624* original[2]){
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
    left.setParameter(left.getParameter(0)+.1f,0);left.setParameter(left.getParameter(0),1);right.setParameter(right.getParameter(0)+.1f,0);right.setParameter(right.getParameter(0),1);left.setParameter(left.getParameter(2)+.05f,2);
    const float expected=left.TriangulateMatches(&right,frame.mvKeys[0],frame.mvKeysRight[0],T01.rotationMatrix(),T01.translation(),1,1,expectedPoint);
    check(before>0&&expected>0&&std::abs(expected-before)>1e-5,"radial change alters production stereo geometry for retained pair");
    frame.RefreshStereoGeometryAfterCalibration();keyframe.RefreshStereoGeometryAfterCalibration();
    check(frame.mK.at<float>(0,0)==left.getParameter(0) && keyframe.fx==left.getParameter(0) && keyframe.cx==left.getParameter(2) && frame.mK_(0,2)==left.getParameter(2),"focal and principal point caches follow accepted intrinsics");
    check(std::abs(frame.mvDepth[0]-expected)<1e-6&&(frame.mvStereo3Dpoints[0]-expectedPoint).norm()<1e-6&&std::abs(keyframe.mvDepth[0]-expected)<1e-6,"Frame and KeyFrame cache refresh equals exact new-calibration triangulation");
    check(frame.mvLeftToRightMatch==std::vector<int>({0,-1,-1})&&frame.mvRightToLeftMatch==std::vector<int>({0,-1,-1})&&keyframe.mvLeftToRightMatch==frame.mvLeftToRightMatch&&keyframe.mvRightToLeftMatch==frame.mvRightToLeftMatch&&frame.mvDepth[1]==-1&&keyframe.mvDepth[1]==-1&&frame.mvStereo3Dpoints[1].isZero(),"invalid and non-reciprocal stereo pairs clear both directions and stale depths");
    check(frame.N==6&&frame.Nleft==3&&frame.Nright==3&&frame.mnCloseMPs==1&&frame.mvpMapPoints==associations&&keyframe.GetMapPointMatches()==keyAssociations&&cv::norm(frame.mDescriptors,descriptors,cv::NORM_INF)==0&&cv::norm(keyframe.mDescriptors,descriptors,cv::NORM_INF)==0&&frame.mvKeys[0].pt==cv::Point2f(z0.x(),z0.y()),"refresh preserves feature slots, descriptors and map associations");
    Frame unused;unused.RefreshStereoGeometryAfterCalibration();check(unused.N==0&&unused.Nleft==-1,"default unused Frame refresh is safe");
    Frame empty;empty.N=empty.Nleft=3;empty.Nright=0;empty.mvLeftToRightMatch={0,1,2};empty.mvDepth={3,4,5};empty.RefreshStereoGeometryAfterCalibration();
    check(empty.mvLeftToRightMatch==std::vector<int>({-1,-1,-1})&&empty.mvRightToLeftMatch.empty()&&empty.mvDepth==std::vector<float>({-1,-1,-1})&&empty.mnCloseMPs==0,"empty-camera refresh clears stale stereo geometry without reading unset pose");
}

void fullRecovery(Fisheye624* cams[2],const ImuCamPose& state){
 g2o::SparseOptimizer graph;graph.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(new g2o::BlockSolverX(new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>())));
 auto* pose=new VertexPose;pose->setEstimate(state);pose->setId(0);pose->setFixed(true);graph.addVertex(pose);
 VertexFullCalibration* c[2];int id=3;struct Sample{int c;Eigen::Vector3d pc;Eigen::Vector2d pixel;};std::vector<Sample,Eigen::aligned_allocator<Sample>> heldout;
 for(int k=0;k<2;++k){
  std::vector<float> factory;for(int j=0;j<16;++j)factory.push_back(cams[k]->getParameter(j));
  c[k]=new VertexFullCalibration(*cams[k],factory);c[k]->setId(k+1);graph.addVertex(c[k]);
  auto* prior=new EdgePriorFullCalibration;prior->setVertex(0,c[k]);prior->setInformation(c[k]->prior().array().square().inverse().matrix().asDiagonal());graph.addEdge(prior);
  VertexFullCalibration truth(*cams[k],factory);IntrinsicVector d=truth.bounds()*.07;for(int j=0;j<15;++j)if((j+k)%2)d[j]*=-1;truth.setDelta(d);
  int n=0;for(int v=8;v<480;v+=12)for(int u=8;u<640;u+=12){
   cv::Point3f ray;if(!cams[k]->tryUnproject(cv::Point2f(u,v),ray))continue;
   Eigen::Vector3d pc=Eigen::Vector3d(ray.x,ray.y,ray.z)*3.;Eigen::Vector2d pixel=truth.camera()->project(pc);
   if(n++%4==0){heldout.push_back({k,pc,pixel});continue;}
   auto* p=new g2o::VertexSBAPointXYZ;p->setId(id++);p->setEstimate(state.Rcw[k].transpose()*(pc-state.tcw[k]));p->setFixed(true);graph.addVertex(p);
   auto* edge=new EdgeMonoFullCalibration(k);edge->setVertex(0,p);edge->setVertex(1,pose);edge->setVertex(2,c[k]);edge->setMeasurement(pixel);edge->setInformation(Eigen::Matrix2d::Identity());graph.addEdge(edge);
  }
 }
 std::vector<double> before,after;for(auto& x:heldout)before.push_back((c[x.c]->camera()->project(x.pc)-x.pixel).norm());
 graph.initializeOptimization();graph.computeActiveErrors();double initial=graph.activeChi2();graph.optimize(40);graph.computeActiveErrors();double final=graph.activeChi2();
 for(auto& x:heldout)after.push_back((c[x.c]->camera()->project(x.pc)-x.pixel).norm());
 std::cout<<"RECOVERY full count="<<heldout.size()<<" median_before="<<median(before)<<" median_after="<<median(after)<<" cost_before="<<initial<<" cost_after="<<final<<std::endl;
 check(heldout.size()>500&&median(after)<.04&&median(after)<.1*median(before)&&final<.1*initial,"joint 15-parameter camera optimization improves unseen native projections under production priors");
}

int main(int argc,char** argv){
 if(argc!=2)return 2;cv::setNumThreads(1);Settings settings(argv[1],System::IMU_STEREO);Fisheye624* cameras[2]={dynamic_cast<Fisheye624*>(settings.camera1()),dynamic_cast<Fisheye624*>(settings.camera2())};
 auto state=makePose(cameras[0],cameras[1]);fullDerivative(cameras,state);fullRecovery(cameras,state);rollback();
#ifdef RADIAL_TEST_STEREO_REFRESH
 refreshCaches(cameras);refreshFocalCaches(cameras);
#endif
 std::cout<<"RESULT failures="<<failures<<std::endl;return failures?1:0;
}
