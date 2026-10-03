// Production proposal tests: withholding, private calibration, exact fallback.
#define main metric_base_fixture_main
#include "metric_inertial_contract.cpp"
#undef main

std::string proposalGeometry(const MetricInertialInitResult& result) {
    std::ostringstream out;out<<std::setprecision(17)<<result.scale<<result.Rwg<<result.bg<<result.ba;
    for(const auto& state:result.states) out<<state.optimizedPose.matrix()<<state.optimizedVelocity;
    for(const auto& point:result.points) out<<point.optimizedPosition;
    return out.str();
}
std::string cameraSnapshot(Fixture& fixture) {
    std::ostringstream out;out<<std::setprecision(17);
    for(int i=0;i<16;++i)out<<fixture.cam0.getParameter(i)<<fixture.cam1.getParameter(i);
    return out.str();
}
int main(int argc,char** argv) {
    if(argc!=2)return 2;cv::setNumThreads(1);Rig::PublishGlobals(false);
    const std::string name=argv[1];Fixture f(.86,false,60);
    MetricInertialInitOptions options;options.jointRefinement=true;options.radialCalibrationExperiment=true;
    const auto source=f.snapshot(),cameraSource=cameraSnapshot(f);
    auto control=Optimizer::ProposeMetricInertialInitialization(f.map(),Eigen::Matrix3d::Identity(),options);
    f.check(control.accepted && control.jointRefined,"fixed-intrinsics held-out control is accepted");
    f.check(!control.radialAttempted && !control.radialCalibrated,"fixed control never estimates intrinsics");
    f.check(control.radialFixedHeldout.count[0][0]>30 && control.radialFixedHeldout.count[1][0]>30,"both cameras have genuinely withheld observations");
    options.optimizeRadialCalibration=true;
    if(name=="radial_force_reject") {
        // Permit the solve on a centre-only fixture but retain the real
        // peripheral-improvement acceptance gate: calibration must roll back.
        options.minimumRadialPeripheralHeldoutPerCamera=0;
    }
    auto treatment=Optimizer::ProposeMetricInertialInitialization(f.map(),Eigen::Matrix3d::Identity(),options);
    std::cout<<"RADIAL attempted="<<treatment.radialAttempted<<" calibrated="<<treatment.radialCalibrated
        <<" reason="<<treatment.radialReason<<" sigma="<<treatment.radialLogBaselineStdDev
        <<" rank="<<treatment.radialHessianRank<<" dimension="<<treatment.radialHessianDimension
        <<" delta0="<<treatment.radialDelta[0].transpose()<<" delta1="<<treatment.radialDelta[1].transpose()<<std::endl;
    f.check(treatment.accepted && treatment.jointRefined,"rejected calibration retains viable geometry initialization");
    f.check(!treatment.radialCalibrated,"centre-only observations cannot authorize peripheral calibration");
    if(name=="radial_force_reject")f.check(treatment.radialAttempted,"production private-camera calibration solve exercised");
    else f.check(!treatment.radialAttempted,"insufficient peripheral coverage skips calibration solve");
    f.check(proposalGeometry(control)==proposalGeometry(treatment),"rejected calibration restores exact fixed-camera geometry and inertial state");
    f.check(source==f.snapshot(),"source map and preintegrations remain unchanged");
    f.check(cameraSource==cameraSnapshot(f),"both source cameras remain byte-equivalent in parameters");
    f.check(treatment.radialSourceParameters[0]==treatment.radialFinalParameters[0] && treatment.radialSourceParameters[1]==treatment.radialFinalParameters[1],"rejected proposal returns original calibration for commit");
    return f.failures?1:0;
}
