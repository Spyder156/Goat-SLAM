#ifndef ORB_SLAM3_FULL_CALIBRATION_OPTIMIZER_IMPL_H
#define ORB_SLAM3_FULL_CALIBRATION_OPTIMIZER_IMPL_H
// Compiled into Optimizer.cc. Caller owns input, loop/GBA-reader and map locks.
// Every optimized value remains in g2o until all acceptance checks pass.
namespace ORB_SLAM3 {
bool Optimizer::FullCalibrationBA(Map* map,Tracking* tracker,double timestamp)
{
 auto report=[&](const std::string& reason){std::fprintf(stderr,"[CALIBRATION-TRIAL] t=%.9f accepted=0 reason=%s\n",timestamp,reason.c_str());return false;};
 auto kfs=map->GetAllKeyFrames();
 kfs.erase(std::remove_if(kfs.begin(),kfs.end(),[&](KeyFrame* k){return !k || k->isBad() || k->GetMap()!=map;}),kfs.end());
 std::sort(kfs.begin(),kfs.end(),[](KeyFrame* a,KeyFrame* b){return a->mTimeStamp<b->mTimeStamp;});
 if(kfs.size()<10 || !map->isImuInitialized())return report("insufficient initialized keyframes");
 const int version=map->GetMapChangeIndex();
 std::map<KeyFrame*,Sophus::SE3f> oldPoses;
 GeometricCamera* live[2]={kfs[0]->mpCamera,kfs[0]->mpCamera2};
 std::vector<float> source[2];
 for(int c=0;c<2;++c){
  if(!dynamic_cast<Fisheye624*>(live[c]) || tracker->mFactoryIntrinsics[c].size()!=16)return report("native pair missing");
  for(int i=0;i<16;++i)source[c].push_back(live[c]->getParameter(i));
 }
 g2o::SparseOptimizer graph;
 auto* solver=new g2o::OptimizationAlgorithmLevenberg(new g2o::BlockSolverX(new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()));
 solver->setUserLambdaInit(1e-3);graph.setAlgorithm(solver);graph.setVerbose(false);
 int id=0;std::map<KeyFrame*,VertexPose*> poses;std::map<KeyFrame*,VertexVelocity*> velocities;std::map<KeyFrame*,VertexGyroBias*> gyros;std::map<KeyFrame*,VertexAccBias*> accs;
 std::vector<std::unique_ptr<IMU::Preintegrated>> integrations;
 for(auto* k:kfs){
  if(k->mpCamera!=live[0] || k->mpCamera2!=live[1] || !k->bImu || !k->GetPose().params().allFinite() || !k->GetVelocity().allFinite())return report("inconsistent camera or inertial keyframe");
  oldPoses[k]=k->GetPose();
  auto* p=new VertexPose(k);p->setId(id++);p->setFixed(k==kfs.front());graph.addVertex(p);poses[k]=p;
  auto* v=new VertexVelocity(k);v->setId(id++);graph.addVertex(v);velocities[k]=v;
  auto* g=new VertexGyroBias(k);g->setId(id++);graph.addVertex(g);gyros[k]=g;
  auto* a=new VertexAccBias(k);a->setId(id++);graph.addVertex(a);accs[k]=a;
 }
 size_t imuCount=0;
 for(auto* k:kfs){
  KeyFrame* prev=k->mPrevKF;
  if(!prev || !poses.count(prev))continue;
  if(!k->mpImuPreintegrated)return report("missing integration in temporal chain");
  integrations.emplace_back(new IMU::Preintegrated(k->mpImuPreintegrated));auto* pi=integrations.back().get();
  if(!(pi->dT>0) || !pi->C.allFinite() || std::abs(pi->dT-(k->mTimeStamp-prev->mTimeStamp))>std::max(.003,.01*pi->dT))return report("invalid integration interval");
  pi->SetNewBias(prev->GetImuBias());
  auto* e=new EdgeInertial(pi);e->setVertex(0,poses[prev]);e->setVertex(1,velocities[prev]);e->setVertex(2,gyros[prev]);e->setVertex(3,accs[prev]);e->setVertex(4,poses[k]);e->setVertex(5,velocities[k]);
  auto* robust=new g2o::RobustKernelHuber;robust->setDelta(std::sqrt(16.92));e->setRobustKernel(robust);graph.addEdge(e);++imuCount;
  Eigen::Matrix3d cg=pi->C.block<3,3>(9,9).cast<double>(),ca=pi->C.block<3,3>(12,12).cast<double>();
  if(cg.determinant()<=0 || ca.determinant()<=0)return report("invalid bias random walk covariance");
  auto* eg=new EdgeGyroRW;eg->setVertex(0,gyros[prev]);eg->setVertex(1,gyros[k]);eg->setInformation(cg.inverse());graph.addEdge(eg);
  auto* ea=new EdgeAccRW;ea->setVertex(0,accs[prev]);ea->setVertex(1,accs[k]);ea->setInformation(ca.inverse());graph.addEdge(ea);
 }
 if(imuCount+1<kfs.size())return report("disconnected temporal chain");
 // Weak physical bias priors; per-keyframe random walks provide actual VI
 // evolution. No trajectory or ground-truth constraints enter this graph.
 auto* pg=new EdgePriorGyro(Eigen::Vector3f::Zero());pg->setVertex(0,gyros[kfs.front()]);pg->setInformation(100*Eigen::Matrix3d::Identity());graph.addEdge(pg);
 auto* pa=new EdgePriorAcc(Eigen::Vector3f::Zero());pa->setVertex(0,accs[kfs.front()]);pa->setInformation(Eigen::Matrix3d::Identity());graph.addEdge(pa);
 VertexFullCalibration* cameras[2];
 for(int c=0;c<2;++c){
  cameras[c]=new VertexFullCalibration(*static_cast<Fisheye624*>(live[c]),tracker->mFactoryIntrinsics[c]);cameras[c]->setId(id++);cameras[c]->setFixed(true);graph.addVertex(cameras[c]);
  auto* e=new EdgePriorFullCalibration;e->setVertex(0,cameras[c]);e->setInformation(cameras[c]->prior().array().square().inverse().matrix().asDiagonal());graph.addEdge(e);
 }
 struct Observation{KeyFrame* k;int c,index;cv::KeyPoint key;};
 struct Measurement{EdgeMonoFullCalibration* edge;int camera,bin;bool heldout;};
 std::vector<Measurement> measurements;std::map<MapPoint*,g2o::VertexSBAPointXYZ*> points;std::map<MapPoint*,Eigen::Vector3f> oldPoints;
 for(auto* point:map->GetAllMapPoints()){
  if(!point || point->isBad() || !point->GetWorldPos().allFinite())continue;
  std::vector<Observation> obs;
  for(auto& item:point->GetObservations()){
   auto* k=item.first;if(!poses.count(k))continue;
   int l=std::get<0>(item.second),r=std::get<1>(item.second)-k->NLeft;
   if(l>=0 && l<k->NLeft && l<(int)k->mvKeysUn.size())obs.push_back({k,0,l,k->mvKeysUn[l]});
   if(r>=0 && r<k->NRight && r<(int)k->mvKeysRight.size())obs.push_back({k,1,r,k->mvKeysRight[r]});
  }
  if(obs.size()<3)continue;
  std::sort(obs.begin(),obs.end(),[](const Observation& a,const Observation& b){return a.k->mTimeStamp==b.k->mTimeStamp?a.c<b.c:a.k->mTimeStamp<b.k->mTimeStamp;});
  if(obs.front().k==obs.back().k)continue;
  auto* p=new g2o::VertexSBAPointXYZ;p->setId(id++);p->setEstimate(point->GetWorldPos().cast<double>());p->setMarginalized(true);graph.addVertex(p);points[point]=p;oldPoints[point]=point->GetWorldPos();
  for(size_t n=0;n<obs.size();++n){
   const auto& o=obs[n];if(o.key.octave<0 || o.key.octave>=(int)o.k->mvInvLevelSigma2.size())continue;
   auto* e=new EdgeMonoFullCalibration(o.c);e->setVertex(0,p);e->setVertex(1,poses[o.k]);e->setVertex(2,cameras[o.c]);
   e->setMeasurement(Eigen::Vector2d(o.key.pt.x,o.key.pt.y));e->setInformation(o.k->mvInvLevelSigma2[o.key.octave]*Eigen::Matrix2d::Identity());
   auto* huber=new g2o::RobustKernelHuber;huber->setDelta(std::sqrt(5.991));e->setRobustKernel(huber);
   bool held=n>=3 && n%5==4; // retain first three training observations
   e->setLevel(held?1:0);graph.addEdge(e);
   double r=std::hypot(o.key.pt.x-source[o.c][2],o.key.pt.y-source[o.c][3]);measurements.push_back({e,o.c,r>=250?1:0,held});
  }
 }
 struct Summary{size_t count[2][2]={{0,0},{0,0}},inliers[2][2]={{0,0},{0,0}};double median[2][2]={{0,0},{0,0}};};
 auto summarize=[&](){Summary s;std::vector<double> values[2][2];for(auto& m:measurements)if(m.heldout){m.edge->computeError();double r=m.edge->error().norm();bool valid=m.edge->isDepthPositive()&&std::isfinite(r);values[m.camera][m.bin].push_back(valid?r:1e6);++s.count[m.camera][m.bin];if(valid&&m.edge->chi2()<5.991)++s.inliers[m.camera][m.bin];}for(int c=0;c<2;++c)for(int b=0;b<2;++b){auto& v=values[c][b];if(!v.empty()){std::sort(v.begin(),v.end());s.median[c][b]=v[v.size()/2];}}return s;};
 auto before=summarize();for(int c=0;c<2;++c)if(before.count[c][0]<30 || before.count[c][1]<10)return report("insufficient heldout center/peripheral coverage");
 std::fprintf(stderr,"[CALIBRATION-START] t=%.9f keyframes=%zu points=%zu observations=%zu imu=%zu dof_per_camera=15 iterations_fixed=10 iterations_free=20\n",timestamp,kfs.size(),points.size(),measurements.size(),imuCount);
 graph.initializeOptimization(0);graph.computeActiveErrors();double initial=graph.activeRobustChi2();graph.optimize(10);graph.computeActiveErrors();double fixed=graph.activeRobustChi2();auto reference=summarize();
 if(!std::isfinite(fixed)||fixed>initial*(1+1e-6))return report("fixed reference failed to decrease cost");
 for(int c=0;c<2;++c)cameras[c]->setFixed(false);
 graph.initializeOptimization(0);graph.optimize(20);graph.computeActiveErrors();double final=graph.activeRobustChi2();auto candidate=summarize();
 bool accept=std::isfinite(final)&&final<fixed;double improvement=0;
 for(int c=0;c<2;++c)for(int b=0;b<2;++b){
  std::fprintf(stderr,"[CALIBRATION-HELDOUT] t=%.9f cam=%d outer=%d count=%zu fixed_px=%.9g final_px=%.9g fixed_inliers=%zu final_inliers=%zu\n",timestamp,c,b,candidate.count[c][b],reference.median[c][b],candidate.median[c][b],reference.inliers[c][b],candidate.inliers[c][b]);
  if(candidate.median[c][b]>reference.median[c][b]+.03 || candidate.inliers[c][b]<.95*reference.inliers[c][b])accept=false;
  improvement+=reference.median[c][b]-candidate.median[c][b];
 }
 if(improvement<.004)accept=false;
 for(int c=0;c<2;++c){
  if((cameras[c]->delta().array().abs()>.95*cameras[c]->bounds().array()).any())accept=false;
  if(!ValidateRadialCameraInverse(*static_cast<Fisheye624*>(live[c]),*cameras[c]->camera(),kfs[0]->mnMaxX,kfs[0]->mnMaxY))accept=false;
 }
 for(auto* k:kfs){auto& p=poses[k]->estimate();if(!p.Rwb.allFinite()||!p.twb.allFinite()||!velocities[k]->estimate().allFinite()||!gyros[k]->estimate().allFinite()||!accs[k]->estimate().allFinite()||gyros[k]->estimate().norm()>.2||accs[k]->estimate().norm()>1.0)accept=false;}
 for(auto& p:points)if(!p.second->estimate().allFinite())accept=false;
 if(!accept)return report("heldout, domain, bounds or inertial acceptance failed");
 // Revalidate exact source before committing even though all readers/writers
 // are quiesced. Source calibration is fixed throughout private optimization.
 if(map->GetMapChangeIndex()!=version)return report("source map changed");
 for(auto* k:kfs)if(k->isBad()||(k->GetPose().matrix()-oldPoses[k].matrix()).norm()>1e-6)return report("source pose changed");
 for(auto& p:oldPoints)if(p.first->isBad()||(p.first->GetWorldPos()-p.second).norm()>1e-6)return report("source point changed");
 for(int c=0;c<2;++c)for(int j=0;j<16;++j)if(live[c]->getParameter(j)!=source[c][j])return report("source intrinsics changed");
 for(int c=0;c<2;++c){
  for(int j=0;j<16;++j)live[c]->setParameter(cameras[c]->camera()->getParameter(j),j);
  std::ostringstream out;out<<std::setprecision(9);for(int j=0;j<16;++j){if(j)out<<',';out<<live[c]->getParameter(j);}
  std::fprintf(stderr,"[CALIBRATION-COMMIT] t=%.9f map=%lu cam=%d params=%s\n",timestamp,map->GetId(),c,out.str().c_str());
 }
 for(auto* k:kfs){
  const auto& p=poses[k]->estimate();Eigen::Quaternionf q(p.Rcw[0].cast<float>());q.normalize();k->SetPose(Sophus::SE3f(q,p.tcw[0].cast<float>()));k->SetVelocity(velocities[k]->estimate().cast<float>());
  auto g=gyros[k]->estimate(),a=accs[k]->estimate();k->SetNewBias(IMU::Bias(a.x(),a.y(),a.z(),g.x(),g.y(),g.z()));
  k->RefreshStereoGeometryAfterCalibration();
 }
 for(auto& p:points){p.first->SetWorldPos(p.second->estimate().cast<float>());p.first->UpdateNormalAndDepth();}
 // Preserve coordinates of excluded short tracks in their reference camera.
 for(auto* p:map->GetAllMapPoints())if(p&&!p->isBad()&&!points.count(p)){
  auto* k=p->GetReferenceKeyFrame();if(k&&oldPoses.count(k)){p->SetWorldPos(k->GetPoseInverse()*(oldPoses[k]*p->GetWorldPos()));p->UpdateNormalAndDepth();}
 }
 tracker->RefreshFullCalibration(oldPoses);map->IncreaseChangeIndex();
 std::fprintf(stderr,"[CALIBRATION-TRIAL] t=%.9f accepted=1 initial_cost=%.9g fixed_cost=%.9g final_cost=%.9g keyframes=%zu points=%zu\n",timestamp,initial,fixed,final,kfs.size(),points.size());
 return true;
}
}
#endif
