#pragma once
// LamariaSpeedPrior.h -- speedprior (2026-10-04): pedestrian speed bound for head-worn Aria.
//
// Evidence (read-only check on gt_dense, evaluation data only; nothing from GT enters here):
//   GT speed over 1 s windows never exceeds 1.84 m/s on Short/Medium/Long (p99.9 1.74-1.83).
//   Baby Medium: estimated speed 2.5-3.0 m/s during 1170-1215 s and 2.6-5.1 m/s during the
//   final SOS/coast 1220-1256 s while the wearer walked at 1.5 then 0.5-1.3 m/s; the tail
//   ended 27 m off. Local scale creeps when biases absorb it under weak vision.
// Mechanism: one-sided soft prior  e = max(0, |v| - vmax)  on body-velocity vertices
//   (gravity-aligned map frame, m/s) in pose-only inertial optimisation, local inertial BA
//   and the Baby solver; pure IMU propagation is clamped to the same bound. Silent below vmax.
// Env overrides: LAMARIA_SPEED_PRIOR_MAX (m/s; 0 disables), LAMARIA_SPEED_PRIOR_SIGMA (m/s).
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <Eigen/Core>
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "G2oTypes.h"

namespace lamaria_speed_prior {

inline double EnvOr(const char* name, double fallback) {
    const char* value = std::getenv(name);
    if(!value || !*value) return fallback;
    char* end = NULL;
    const double parsed = std::strtod(value, &end);
    return (end && *end == '\0' && std::isfinite(parsed)) ? parsed : fallback;
}
inline double MaxSpeed() { static const double v = EnvOr("LAMARIA_SPEED_PRIOR_MAX", 2.2); return v; }     // m/s
inline double Sigma() { static const double v = EnvOr("LAMARIA_SPEED_PRIOR_SIGMA", 0.01); return v; }    // m/s
inline bool Enabled() { return MaxSpeed() > 0.0 && Sigma() > 0.0; }
inline double Information() { return 1.0 / (Sigma() * Sigma()); }

// One-sided unary prior on a VertexVelocity (additive 3-vector update): e = max(0, |v| - vmax).
class EdgeSpeedBound : public g2o::BaseUnaryEdge<1, double, ORB_SLAM3::VertexVelocity> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit EdgeSpeedBound(double maxSpeed) : mMaxSpeed(maxSpeed) { setMeasurement(0.0); }
    virtual bool read(std::istream&) { return false; }
    virtual bool write(std::ostream&) const { return false; }
    virtual void computeError() {
        const ORB_SLAM3::VertexVelocity* v = static_cast<const ORB_SLAM3::VertexVelocity*>(_vertices[0]);
        const double speed = v->estimate().norm();
        _error[0] = speed > mMaxSpeed ? speed - mMaxSpeed : 0.0;
    }
    virtual void linearizeOplus() {
        const ORB_SLAM3::VertexVelocity* v = static_cast<const ORB_SLAM3::VertexVelocity*>(_vertices[0]);
        const Eigen::Vector3d vel = v->estimate();
        const double speed = vel.norm();
        _jacobianOplusXi.setZero();
        if(speed > mMaxSpeed && speed > 1e-12) _jacobianOplusXi = (vel / speed).transpose();   // d|v|/dv, 1x3
    }
    double mMaxSpeed;
};

inline EdgeSpeedBound* AddSpeedBound(g2o::SparseOptimizer& optimizer, ORB_SLAM3::VertexVelocity* vertex) {
    EdgeSpeedBound* edge = new EdgeSpeedBound(MaxSpeed());
    edge->setVertex(0, vertex);
    edge->setInformation(Eigen::Matrix<double,1,1>::Identity() * Information());
    optimizer.addEdge(edge);
    return edge;
}

// Clamp a propagated body velocity (map frame, m/s) to the bound; rate-limited log.
inline void ClampPropagated(Eigen::Vector3f& velocity, double timestamp, const char* stage) {
    const float speed = velocity.norm();
    if(speed > MaxSpeed()) {
        velocity *= static_cast<float>(MaxSpeed()) / speed;
        static long fires = 0;
        if(++fires % 20 == 1)
            std::cout << std::setprecision(12) << "[SPEED_PRIOR] t=" << timestamp << " stage=" << stage
                      << " propagated=" << speed << " clamped_to=" << MaxSpeed() << std::endl;
    }
}

// Pose-only stages: log only when the bound was active before or after the solve.
inline void ReportFrame(double timestamp, const char* stage, double before, double after, EdgeSpeedBound* edge) {
    if(before <= MaxSpeed() && after <= MaxSpeed()) return;
    edge->computeError();
    std::cout << std::setprecision(12) << "[SPEED_PRIOR] t=" << timestamp << " stage=" << stage
              << " speed_before=" << before << " speed_after=" << after << " chi2=" << edge->chi2()
              << " max=" << MaxSpeed() << std::endl;
}

struct SpeedStats {
    int over = 0;
    double max = 0.0;
    void Observe(double speed) { if(speed > max) max = speed; if(speed > MaxSpeed()) ++over; }
};

inline void ReportLocalBA(double timestamp, const SpeedStats& before, const SpeedStats& after) {
    if(before.over == 0 && after.over == 0) return;
    std::cout << std::setprecision(12) << "[SPEED_PRIOR] t=" << timestamp << " stage=local_ba"
              << " n_over_before=" << before.over << " max_before=" << before.max
              << " n_over_after=" << after.over << " max_after=" << after.max
              << " max=" << MaxSpeed() << std::endl;
}

}  // namespace lamaria_speed_prior
