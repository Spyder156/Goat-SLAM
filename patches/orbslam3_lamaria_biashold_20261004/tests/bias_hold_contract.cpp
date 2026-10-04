// bias_hold_contract.cpp -- falsifiable checks of the bias-hold state machine and the random-walk gain.
#include "LamariaBiasHold.h"
#include "G2oTypes.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include <cstdio>
#include <cmath>
namespace {
int failures = 0;
void check(bool ok, const char* name, double value) { std::printf("%s %s (%.9g)\n", ok ? "ok  " : "FAIL", name, value); if(!ok) ++failures; }
// Pull toward a target bias with a given information: stands in for the inertial edges' bias pull.
class EdgeBiasTarget : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, ORB_SLAM3::VertexGyroBias> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual bool read(std::istream&) { return false; }
    virtual bool write(std::ostream&) const { return false; }
    virtual void computeError() { _error = static_cast<const ORB_SLAM3::VertexGyroBias*>(_vertices[0])->estimate() - _measurement; }
    virtual void linearizeOplus() { _jacobianOplusXi.setIdentity(); }
};
// One random-walk edge between a fixed previous bias and a free current bias, plus a pull of 1e6 toward
// a bias 0.02 rad/s away. Returns how far the current bias moved from the previous one.
double moved(double gain) {
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType* linear = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(new g2o::BlockSolverX(linear)));
    const Eigen::Vector3d g0(0.01, 0.0, 0.0), g1(0.03, 0.0, 0.0);
    ORB_SLAM3::VertexGyroBias* prev = new ORB_SLAM3::VertexGyroBias(); prev->setEstimate(g0); prev->setId(0); prev->setFixed(true); optimizer.addVertex(prev);
    ORB_SLAM3::VertexGyroBias* cur = new ORB_SLAM3::VertexGyroBias(); cur->setEstimate(g0); cur->setId(1); cur->setFixed(false); optimizer.addVertex(cur);
    ORB_SLAM3::EdgeGyroRW* rw = new ORB_SLAM3::EdgeGyroRW(); rw->setVertex(0, prev); rw->setVertex(1, cur);
    rw->setInformation(Eigen::Matrix3d::Identity() * 1e5 * gain); optimizer.addEdge(rw);   // typical accel-walk information times the gain
    EdgeBiasTarget* pull = new EdgeBiasTarget(); pull->setVertex(0, cur); pull->setMeasurement(g1);
    pull->setInformation(Eigen::Matrix3d::Identity() * 1e6); optimizer.addEdge(pull);
    optimizer.initializeOptimization(); optimizer.optimize(30);
    return (cur->estimate() - g0).norm();
}
}  // namespace
int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    using namespace lamaria_bias_hold;
    check(Enabled() && std::fabs(Gain() - 1e4) < 1e-6, "defaults: enabled, gain 1e4", Gain());
    ORB_SLAM3::IMU::Bias b1(0.1f, 0.2f, 0.3f, 0.01f, 0.02f, 0.03f), b2(0.5f, 0.5f, 0.5f, 0.05f, 0.05f, 0.05f), b3(0.9f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Update(false, b3, 10.0);
    check(!Active() && ClosedWindows() == 0, "no hold while vision constrains the path", 0);
    Update(true, b1, 11.0);
    check(Active() && Deviation(Held(), b1) == 0.0, "hold begins with the last observed bias", Deviation(Held(), b1));
    Update(true, b2, 11.05);
    check(Active() && Deviation(Held(), b1) == 0.0, "held bias does not follow later biases", Deviation(Held(), b2));
    Observe(b2);
    Update(false, b3, 12.0);
    check(!Active() && ClosedWindows() == 1, "hold ends when vision is back; window closed", ClosedWindows());
    check(InWindow(11.5) && !InWindow(12.5) && !InWindow(10.5), "window membership [11, 12]", 1);
    Update(true, b3, 13.0);
    check(Active() && InWindow(13.2) && Deviation(Held(), b3) == 0.0, "second hold: active window counts", 1);
    Update(false, b3, 14.0);
    const double free_move = moved(1.0), held_move = moved(Gain());
    check(free_move > 0.015, "without hold a 1e6 pull moves the bias most of the 0.02 rad/s", free_move);
    check(held_move < 1e-4, "with the hold gain the bias moves less than 1e-4 rad/s", held_move);
    if(failures) { std::printf("FAIL %d bias-hold checks\n", failures); return 1; }
    std::printf("PASS 9 bias-hold checks\n"); return 0;
}
