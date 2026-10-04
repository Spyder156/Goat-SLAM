// speed_prior_contract.cpp -- falsifiable checks of the pedestrian speed-bound edge.
// Expects: VertexVelocity estimate = body velocity, m/s, additive update; EdgeSpeedBound
// error = max(0, |v| - vmax) with vmax = 2.2 m/s and information 1/sigma^2, sigma 0.01 m/s.
#include "LamariaSpeedPrior.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/jacobian_workspace.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include <cstdio>
#include <cmath>

namespace {
int failures = 0;
void check(bool ok, const char* name, double value) {
    std::printf("%s %s (%.9g)\n", ok ? "ok  " : "FAIL", name, value);
    if(!ok) ++failures;
}
// Gaussian prior toward a target velocity: e = v - target, J = I.
class EdgeVelocityTarget : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, ORB_SLAM3::VertexVelocity> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual bool read(std::istream&) { return false; }
    virtual bool write(std::ostream&) const { return false; }
    virtual void computeError() {
        const ORB_SLAM3::VertexVelocity* v = static_cast<const ORB_SLAM3::VertexVelocity*>(_vertices[0]);
        _error = v->estimate() - _measurement;
    }
    virtual void linearizeOplus() { _jacobianOplusXi.setIdentity(); }
};
struct Problem {
    g2o::SparseOptimizer optimizer;
    ORB_SLAM3::VertexVelocity* vertex;
    explicit Problem(const Eigen::Vector3d& v0) {
        g2o::BlockSolverX::LinearSolverType* linear = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX* block = new g2o::BlockSolverX(linear);
        optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(block));
        optimizer.setVerbose(false);
        vertex = new ORB_SLAM3::VertexVelocity();
        vertex->setEstimate(v0); vertex->setId(0); vertex->setFixed(false);
        optimizer.addVertex(vertex);
    }
    void solve(int iterations) { optimizer.initializeOptimization(); optimizer.optimize(iterations); }
};
}  // namespace

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);   // keep output if a check crashes
    using lamaria_speed_prior::EdgeSpeedBound;
    const double vmax = lamaria_speed_prior::MaxSpeed();
    check(std::fabs(vmax - 2.2) < 1e-12, "default max speed 2.2 m/s", vmax);
    check(std::fabs(lamaria_speed_prior::Sigma() - 0.01) < 1e-12, "default sigma 0.01 m/s", lamaria_speed_prior::Sigma());
    check(std::fabs(lamaria_speed_prior::Information() - 1e4) < 1e-6, "information 1/sigma^2", lamaria_speed_prior::Information());
    check(lamaria_speed_prior::BabyStageEnabled() && lamaria_speed_prior::PredictStageEnabled()
          && !lamaria_speed_prior::PoseStageEnabled() && !lamaria_speed_prior::LocalBAStageEnabled(), "tail default stages = baby,predict", lamaria_speed_prior::Stages().size());

    // 1. Analytic Jacobian equals central differences in the active region.
    {
        ORB_SLAM3::VertexVelocity v; v.setEstimate(Eigen::Vector3d(2.5, -1.0, 0.5));
        EdgeSpeedBound edge(vmax); edge.setVertex(0, &v);
        edge.computeError(); const double e0 = edge.error()[0];
        check(std::fabs(e0 - (Eigen::Vector3d(2.5, -1.0, 0.5).norm() - vmax)) < 1e-12, "active error = |v| - vmax", e0);
        g2o::JacobianWorkspace jw; jw.updateSize(&edge); jw.allocate();   // Jacobian lives in the workspace map
        static_cast<g2o::BaseUnaryEdge<1, double, ORB_SLAM3::VertexVelocity>&>(edge).linearizeOplus(jw);
        double worst = 0.0;
        for(int k = 0; k < 3; ++k) {
            const double eps = 1e-6;
            Eigen::Vector3d plus(2.5, -1.0, 0.5), minus(2.5, -1.0, 0.5); plus[k] += eps; minus[k] -= eps;
            v.setEstimate(plus); edge.computeError(); const double ep = edge.error()[0];
            v.setEstimate(minus); edge.computeError(); const double em = edge.error()[0];
            worst = std::max(worst, std::fabs((ep - em) / (2 * eps) - edge.jacobianOplusXi()(0, k)));
        }
        check(worst < 1e-6, "analytic Jacobian vs central differences", worst);
    }
    // 2. Inactive region: zero error and zero Jacobian.
    {
        ORB_SLAM3::VertexVelocity v; v.setEstimate(Eigen::Vector3d(1.0, 0.5, 0.0));
        EdgeSpeedBound edge(vmax); edge.setVertex(0, &v);
        g2o::JacobianWorkspace jw; jw.updateSize(&edge); jw.allocate();
        edge.computeError(); static_cast<g2o::BaseUnaryEdge<1, double, ORB_SLAM3::VertexVelocity>&>(edge).linearizeOplus(jw);
        check(edge.error()[0] == 0.0 && edge.jacobianOplusXi().norm() == 0.0, "inactive below vmax: error 0, Jacobian 0", edge.error()[0]);
    }
    // 3. Lone vertex above the bound is pulled onto the bound along its own direction.
    {
        Problem p(Eigen::Vector3d(3.0, 0.0, 0.0));
        lamaria_speed_prior::AddSpeedBound(p.optimizer, p.vertex);
        p.solve(30);
        const Eigen::Vector3d v = p.vertex->estimate();
        check(std::fabs(v.norm() - vmax) < 1e-4, "lone 3.0 m/s vertex ends at vmax", v.norm());
        check(std::fabs(v[1]) < 1e-9 && std::fabs(v[2]) < 1e-9, "direction preserved", std::fabs(v[1]) + std::fabs(v[2]));
    }
    // 4. Lone vertex below the bound is untouched.
    {
        Problem p(Eigen::Vector3d(1.0, 0.5, 0.0));
        EdgeSpeedBound* e = lamaria_speed_prior::AddSpeedBound(p.optimizer, p.vertex);
        p.solve(30);
        const double moved = (p.vertex->estimate() - Eigen::Vector3d(1.0, 0.5, 0.0)).norm();
        e->computeError();
        check(moved < 1e-12 && e->chi2() == 0.0, "below vmax: vertex unchanged, chi2 0", moved);
    }
    // 5. Information weighting: against a Gaussian pull toward 3.0 m/s with equal information
    //    the optimum is the weighted mean (wp*3.0 + ws*2.2)/(wp+ws) = 2.6 m/s.
    {
        Problem p(Eigen::Vector3d(3.0, 0.0, 0.0));
        lamaria_speed_prior::AddSpeedBound(p.optimizer, p.vertex);
        EdgeVelocityTarget* target = new EdgeVelocityTarget();
        target->setVertex(0, p.vertex); target->setMeasurement(Eigen::Vector3d(3.0, 0.0, 0.0));
        target->setInformation(Eigen::Matrix3d::Identity() * lamaria_speed_prior::Information());
        p.optimizer.addEdge(target);
        p.solve(50);
        check(std::fabs(p.vertex->estimate()[0] - 2.6) < 1e-4, "equal-information compromise at 2.6 m/s", p.vertex->estimate()[0]);
    }
    // 6. Propagation clamp keeps direction, bounds magnitude, leaves slow velocities alone.
    {
        Eigen::Vector3f fast(3.0f, 4.0f, 0.0f), slow(1.0f, 0.0f, 0.0f);
        lamaria_speed_prior::ClampPropagated(fast, 0.0, "test");
        lamaria_speed_prior::ClampPropagated(slow, 0.0, "test");
        check((fast - Eigen::Vector3f(1.32f, 1.76f, 0.0f)).norm() < 1e-5f, "clamp 5 m/s -> 2.2 m/s same direction", fast.norm());
        check(slow == Eigen::Vector3f(1.0f, 0.0f, 0.0f), "clamp leaves 1 m/s untouched", slow.norm());
    }
    if(failures) { std::printf("FAIL %d speed-prior checks\n", failures); return 1; }
    std::printf("PASS 12 speed-prior checks\n");
    return 0;
}
