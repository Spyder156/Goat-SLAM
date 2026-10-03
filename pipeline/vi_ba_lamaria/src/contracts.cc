#define main lamaria_solver_main
#include "colmap_lamaria_ba.cc"
#undef main
#include <random>

void Check(bool x,const char* label){if(!x)throw std::runtime_error(label);std::cout<<"PASS "<<label<<'\n';}
int main() try {
  std::vector<lb::Sample> raw;
  for(int i=0;i<=2000;++i)raw.push_back({1000000000LL+i*1000000LL,V3::Zero(),V3(0,0,9.81)});
  auto segment=lb::Slice(raw,1000500000,2500750000);
  auto p=lb::Integrate(segment,Bias::Zero(),.0005,.0024);
  Check(std::abs(p.dt-1.50025)<1e-12 && segment.size()==1501,"fractional endpoints retain all samples over long image interval");
  Check((p.v-V3(0,0,9.81*p.dt)).norm()<1e-10 && (p.p-V3(0,0,.5*9.81*p.dt*p.dt)).norm()<1e-10,"constant acceleration integration is analytic");
  Check((p.J.block<3,3>(3,3)+M3::Identity()*p.dt).norm()<1e-8,"accelerometer bias Jacobian sign and units");
  Check((p.W*p.covariance*p.W.transpose()-lb::M9::Identity()).norm()<1e-6,"covariance whitening identity");
  bool missing=false;try{lb::Slice(raw,999000000,1500000000);}catch(...){missing=true;}Check(missing,"missing endpoint bracket fails explicitly");
  auto gap=raw;gap.erase(gap.begin()+400,gap.begin()+450);bool rejected=false;try{lb::Slice(gap,1100000000,1700000000);}catch(...){rejected=true;}Check(rejected,"native IMU acquisition gaps rejected");
  {
    // Differentiate the actual discrete integrator in right attitude-error
    // coordinates, including rotations/specific force that a stationary
    // covariance test cannot exercise.
    Eigen::Quaterniond R=lb::Exp(V3(.3,-.2,.1));double ferror=0,gerror=0;
    const std::vector<lb::Segment> samples={{.001,V3(1,2,-3),V3(.4,-.6,9.2)},
      {.01,V3(-2,.5,1),V3(1.2,-.8,8.7)},{.004,V3(.3,-1,2),V3(-.4,.7,9.4)}};
    for(const auto& sample:samples){
      lb::M9 F,numericF;Eigen::Matrix<double,9,6> G,numericG;
      lb::MidpointJacobians(R,sample.w,sample.a,sample.dt,F,G);
      const auto nominal=lb::Integrate({sample},Bias::Zero(),.0005,.0024,false);
      auto propagate=[&](const Eigen::Matrix<double,9,1>& state,const Bias& noise){
        const auto Ri=R*lb::Exp(state.tail<3>());
        const auto step=lb::Integrate({{sample.dt,sample.w+noise.head<3>(),sample.a+noise.tail<3>()}},Bias::Zero(),.0005,.0024,false);
        Eigen::Matrix<double,9,1> e;
        e.head<3>()=state.head<3>()+state.segment<3>(3)*sample.dt+Ri*step.p-R*nominal.p;
        e.segment<3>(3)=state.segment<3>(3)+Ri*step.v-R*nominal.v;
        e.tail<3>()=lb::Log((R*nominal.q).conjugate()*(Ri*step.q));return e;
      };
      const double h=1e-5;
      for(int j=0;j<9;++j){Eigen::Matrix<double,9,1> e=Eigen::Matrix<double,9,1>::Zero();e[j]=h;numericF.col(j)=(propagate(e,Bias::Zero())-propagate(-e,Bias::Zero()))/(2*h);}
      for(int j=0;j<6;++j){Bias e=Bias::Zero();e[j]=h;numericG.col(j)=(propagate(Eigen::Matrix<double,9,1>::Zero(),e)-propagate(Eigen::Matrix<double,9,1>::Zero(),-e))/(2*h);}
      ferror=std::max(ferror,(F-numericF).norm()/F.norm());gerror=std::max(gerror,(G-numericG).norm()/G.norm());R=R*nominal.q;
    }
    Check(ferror<1e-8 && gerror<1e-8,"rotating midpoint state and gyro-force noise Jacobians match full finite differences");
    // Independently propagate covariance with numerically differentiated F/G
    // along the same actual multi-sample integration path.
    R=Eigen::Quaterniond::Identity();lb::M9 C=lb::M9::Zero();
    for(const auto& sample:samples){
      const auto nominal=lb::Integrate({sample},Bias::Zero(),.0005,.0024,false);
      auto propagate=[&](const Eigen::Matrix<double,9,1>& state,const Bias& noise){
        const auto Ri=R*lb::Exp(state.tail<3>());const auto step=lb::Integrate({{sample.dt,sample.w+noise.head<3>(),sample.a+noise.tail<3>()}},Bias::Zero(),.0005,.0024,false);
        Eigen::Matrix<double,9,1> e;e.head<3>()=state.head<3>()+state.segment<3>(3)*sample.dt+Ri*step.p-R*nominal.p;e.segment<3>(3)=state.segment<3>(3)+Ri*step.v-R*nominal.v;e.tail<3>()=lb::Log((R*nominal.q).conjugate()*(Ri*step.q));return e;
      };
      lb::M9 F;Eigen::Matrix<double,9,6> G;const double h=1e-5;
      for(int j=0;j<9;++j){Eigen::Matrix<double,9,1> e=Eigen::Matrix<double,9,1>::Zero();e[j]=h;F.col(j)=(propagate(e,Bias::Zero())-propagate(-e,Bias::Zero()))/(2*h);}
      for(int j=0;j<6;++j){Bias e=Bias::Zero();e[j]=h;G.col(j)=(propagate(Eigen::Matrix<double,9,1>::Zero(),e)-propagate(Eigen::Matrix<double,9,1>::Zero(),-e))/(2*h);}
      Bias variance;variance.head<3>().setConstant(.0005*.0005/sample.dt);variance.tail<3>().setConstant(.0024*.0024/sample.dt);
      C=F*C*F.transpose()+G*variance.asDiagonal()*G.transpose();R=R*nominal.q;
    }
    C.diagonal().array()+=1e-15;const auto propagated=lb::Integrate(samples,Bias::Zero(),.0005,.0024);
    Check((C-propagated.covariance).norm()/C.norm()<1e-8 && (propagated.W*propagated.covariance*propagated.W.transpose()-lb::M9::Identity()).norm()<1e-6,"rotating multi-sample covariance including off-diagonal blocks matches numerical transport");
    std::cout<<"MIDPOINT_JACOBIANS max_F_relative="<<ferror<<" max_G_relative="<<gerror<<'\n';
  }
  const auto qbc=lb::Exp(V3(.2,-.3,.1));const V3 tbc(.03,-.07,.09),g(1,2,-std::sqrt(9.81*9.81-5));
  const auto R0=lb::Exp(V3(.4,.2,-.1));const V3 X0(3,4,5),v0(.7,-.4,.2);
  std::vector<lb::Segment> moving(300,{.001,V3(.1,.2,-.3),V3(.4,-.6,9.2)});
  Bias b; b<<.002,-.003,.001,.04,-.03,.02;
  const auto pre=lb::Integrate(moving,b,.0005,.0024);
  const auto R1=R0*pre.q;const V3 X1=X0+v0*pre.dt+.5*g*pre.dt*pre.dt+R0*pre.p,v1=v0+g*pre.dt+R0*pre.v;
  auto CamPose=[&](const Eigen::Quaterniond& R,const V3& X){return colmap::Rigid3d(qbc.conjugate()*R.conjugate(),qbc.conjugate()*(-(R.conjugate()*X)-tbc));};
  const auto c0=CamPose(R0,X0),c1=CamPose(R1,X1);Eigen::Quaterniond recovered;V3 recovered_p;
  BodyPose(c0.params.data(),qbc,tbc,recovered,recovered_p);
  Check(lb::Log(recovered.conjugate()*R0).norm()<1e-12 && (recovered_p-X0).norm()<1e-12,"nonidentity Tbc lever arm and w2c xyzw conventions");
  ImuCost cost(&pre,qbc,tbc);Eigen::Matrix<double,9,1> residual;
  cost(c0.params.data(),c1.params.data(),v0.data(),v1.data(),b.data(),g.data(),residual.data());
  Check(residual.norm()<1e-7,"full IMU residual is zero with arbitrary world gravity and body rotation");
  const auto cfar=colmap::Rigid3d(c1.rotation(),c1.translation()+V3(.1,0,0));
  cost(c0.params.data(),cfar.params.data(),v0.data(),v1.data(),b.data(),g.data(),residual.data());Check(residual.norm()>10,"IMU factor responds to translation errors");
  // Finite differences verify AutoDiff derivatives including quaternion storage,
  // lever arm, per-frame bias correction and gravity coordinates.
  ceres::AutoDiffCostFunction<ImuCost,9,7,7,3,3,6,3> cf(new ImuCost(&pre,qbc,tbc));
  std::vector<std::vector<double>> params={std::vector<double>(c0.params.data(),c0.params.data()+7),std::vector<double>(c1.params.data(),c1.params.data()+7),std::vector<double>(v0.data(),v0.data()+3),std::vector<double>(v1.data(),v1.data()+3),std::vector<double>(b.data(),b.data()+6),std::vector<double>(g.data(),g.data()+3)};
  std::vector<const double*> pp;std::vector<std::vector<double>> jac;std::vector<double*> jp;
  for(auto& x:params){pp.push_back(x.data());jac.emplace_back(9*x.size());}for(auto& x:jac)jp.push_back(x.data());
  cf.Evaluate(pp.data(),residual.data(),jp.data());double maxrel=0;
  for(size_t block=0;block<params.size();++block)for(size_t j=0;j<params[block].size();++j){auto& x=params[block][j];double save=x,h=1e-6;Eigen::Matrix<double,9,1> rp,rm;x=save+h;cf.Evaluate(pp.data(),rp.data(),nullptr);x=save-h;cf.Evaluate(pp.data(),rm.data(),nullptr);x=save;for(int r=0;r<9;++r){double numeric=(rp[r]-rm[r])/(2*h),analytic=jac[block][r*params[block].size()+j];maxrel=std::max(maxrel,std::abs(numeric-analytic)/(1+std::abs(numeric)));}}
  Check(maxrel<2e-5,"IMU pose velocity bias gravity AutoDiff agrees with finite differences");
  std::vector<double> cp={242,242,319.3,241.,-.0277657,.104662,-.075221,.0142486,.000849837,-.000390873,.000328355,.000291279,-.00114684,-.000284116,-.000549598,-.000060646};
  CameraState camera(cp);Check(CameraDomain(camera),"factory full native 624 forward/inverse domain");
  for(int i=0;i<15;++i)camera.z[i]=.1;auto calibrated=camera.Params();Check(calibrated[0]==calibrated[1],"all-parameter calibration preserves native shared-axis focal");
  Check(calibrated[2]!=cp[2] && calibrated[15]!=cp[15] && calibrated[9]!=cp[9],"principal point radial tangential prism all enabled");
  camera.qcr=lb::Exp(V3(.02,.2,-.03));camera.tcr=V3(-.137,0,0);
  colmap::Rigid3d testpose(lb::Exp(V3(.03,-.01,.02)),V3(.1,-.2,.3));V3 landmark(1,2,8);Eigen::Vector2d pixel(340,270);
  ceres::AutoDiffCostFunction<VisualCost,2,7,3,15> free_visual(new VisualCost(camera,pixel));
  ceres::AutoDiffCostFunction<FixedVisualCost,2,7,3> fixed_visual(new FixedVisualCost(camera,pixel));
  const double* vp[3]={testpose.params.data(),landmark.data(),camera.z.data()};
  double vf[2],vx[2],JfreePose[14],JfreePoint[6],JfreeCam[30],JfixedPose[14],JfixedPoint[6];
  double* jf[3]={JfreePose,JfreePoint,JfreeCam};double* jx[2]={JfixedPose,JfixedPoint};
  free_visual.Evaluate(vp,vf,jf);fixed_visual.Evaluate(vp,vx,jx);
  double diff=std::abs(vf[0]-vx[0])+std::abs(vf[1]-vx[1]);
  for(int i=0;i<14;++i)diff+=std::abs(JfreePose[i]-JfixedPose[i]);for(int i=0;i<6;++i)diff+=std::abs(JfreePoint[i]-JfixedPoint[i]);
  Check(diff<1e-10,"memory-efficient fixed-camera factor matches full factor residuals and pose-point Jacobians");
  V3 behind(0,0,-8);VisualCost stock_semantics(camera,pixel);double hidden[2];
  Check(!stock_semantics.Project(testpose.params.data(),behind.data(),camera.z.data(),hidden) && stock_semantics(testpose.params.data(),behind.data(),camera.z.data(),hidden) && hidden[0]==0 && hidden[1]==0,"unprojectable trial observation follows stock COLMAP zero-residual contract");
  std::cout<<"ALL CONTRACTS PASS max_imu_jacobian_relative_error="<<maxrel<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<"CONTRACT_FAIL "<<e.what()<<'\n';return 1;}
