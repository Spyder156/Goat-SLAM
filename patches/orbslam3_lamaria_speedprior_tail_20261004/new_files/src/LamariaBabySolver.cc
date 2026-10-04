#include "LamariaBabySolver.h"
#include "Frame.h"
#include "G2oTypes.h"
#include "GeometricTools.h"
#include "LamariaSpeedPrior.h"   // speedprior (2026-10-04)
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace ORB_SLAM3 {
namespace {
constexpr double kVisualChi2 = 5.991;
constexpr int kMinimumCurrentTracks = 15;
constexpr int kMaximumTracksPerCamera = 160;

struct StateVertices {
    VertexPose* pose;
    VertexVelocity* velocity;
    VertexGyroBias* gyro;
    VertexAccBias* acc;
};

struct TrackEdges {
    const BabyTrack* source;
    g2o::VertexSBAPointXYZ* point;
    std::vector<EdgeMono*> edges;
    std::vector<std::size_t> frames;
    std::vector<int> cameras;
};

bool ValidObservation(const BabyObservation& o,
                      const std::vector<Frame*>& frames) {
    return o.frame < frames.size() && o.camera >= 0 && o.camera <= 1 &&
           (o.camera == 0 ? frames[o.frame]->mpCamera != nullptr :
                            frames[o.frame]->mpCamera2 != nullptr) &&
           o.pixel.allFinite() && std::isfinite(o.inverseVariance) &&
           o.inverseVariance > 0.;
}

bool TriangulateSeed(const BabyTrack& track,
                     const std::vector<Frame*>& frames,
                     const std::vector<StateVertices>& states,
                     Eigen::Vector3d& point) {
    // Use the native general-ray DLT and native inverse lens model. Select the
    // widest available temporal angle; no predicted or arbitrary fixed depth.
    double bestCos = 1.;
    const BabyObservation* first = nullptr;
    const BabyObservation* second = nullptr;
    for (std::size_t a = 0; a < track.observations.size(); ++a) {
        const BabyObservation& oa = track.observations[a];
        if (!ValidObservation(oa, frames)) continue;
        const ImuCamPose& pa = states[oa.frame].pose->estimate();
        Eigen::Vector3d ra = pa.pCamera[oa.camera]->unprojectEig(
            cv::Point2f(oa.pixel.x(), oa.pixel.y())).cast<double>();
        if (!ra.allFinite() || ra.norm() < 1e-9) continue;
        ra = pa.Rcw[oa.camera].transpose() * ra.normalized();
        for (std::size_t b = a + 1; b < track.observations.size(); ++b) {
            const BabyObservation& ob = track.observations[b];
            if (!ValidObservation(ob, frames) || oa.frame == ob.frame) continue;
            const ImuCamPose& pb = states[ob.frame].pose->estimate();
            Eigen::Vector3d rb = pb.pCamera[ob.camera]->unprojectEig(
                cv::Point2f(ob.pixel.x(), ob.pixel.y())).cast<double>();
            if (!rb.allFinite() || rb.norm() < 1e-9) continue;
            rb = pb.Rcw[ob.camera].transpose() * rb.normalized();
            const double c = ra.dot(rb);
            if (c > 0. && c < bestCos) {
                bestCos = c;
                first = &oa;
                second = &ob;
            }
        }
    }
    // Below 0.25 degrees, this XYZ parameterization is too poorly conditioned.
    // This is only a temporary-point admission test, not relaxed map matching.
    if (!first || bestCos > std::cos(0.25 * M_PI / 180.)) return false;
    const ImuCamPose& pa = states[first->frame].pose->estimate();
    const ImuCamPose& pb = states[second->frame].pose->estimate();
    Eigen::Vector3f ra = pa.pCamera[first->camera]->unprojectEig(
        cv::Point2f(first->pixel.x(), first->pixel.y()));
    Eigen::Vector3f rb = pb.pCamera[second->camera]->unprojectEig(
        cv::Point2f(second->pixel.x(), second->pixel.y()));
    Eigen::Matrix<float, 3, 4> ta, tb;
    ta.leftCols<3>() = pa.Rcw[first->camera].cast<float>();
    ta.rightCols<1>() = pa.tcw[first->camera].cast<float>();
    tb.leftCols<3>() = pb.Rcw[second->camera].cast<float>();
    tb.rightCols<1>() = pb.tcw[second->camera].cast<float>();
    Eigen::Vector3f seed;
    if (!GeometricTools::Triangulate(ra, rb, ta, tb, seed) ||
        !seed.allFinite()) return false;
    point = seed.cast<double>();
    if (!pa.isDepthPositive(point, first->camera) ||
        !pb.isDepthPositive(point, second->camera)) return false;
    const double ea = (pa.Project(point, first->camera) - first->pixel).squaredNorm()
                      * first->inverseVariance;
    const double eb = (pb.Project(point, second->camera) - second->pixel).squaredNorm()
                      * second->inverseVariance;
    // A seed may be rough when the current prediction is uncertain. Acceptance
    // after robust optimization still uses the native strict 5.991 bound.
    return std::isfinite(ea) && std::isfinite(eb) && ea <= 100. && eb <= 100.;
}

bool GoodEdge(EdgeMono* edge) {
    edge->computeError();
    return std::isfinite(edge->chi2()) && edge->chi2() <= kVisualChi2 &&
           edge->isDepthPositive();
}
} // namespace

BabySolveResult SolveBabyWindow(const std::vector<Frame*>& frames,
                                const std::vector<BabyTrack>& tracks,
                                const std::vector<bool>& fixed) {
    BabySolveResult result;
    result.reason = "invalid_window";
    if (frames.size() < 2 || frames.size() > 8 || fixed.size() != frames.size())
        return result;
    for (Frame* f : frames) {
        if (!f || !f->mpCamera || !f->GetPose().matrix().allFinite() ||
            !f->GetVelocity().allFinite()) return result;
    }
    for (std::size_t i = 1; i < frames.size(); ++i) {
        IMU::Preintegrated* pre = frames[i]->mpImuPreintegratedFrame;
        const double dt = frames[i]->mTimeStamp - frames[i - 1]->mTimeStamp;
        if (!pre || !std::isfinite(dt) || dt <= 0. || dt > .2 ||
            !std::isfinite(pre->dT) || std::abs(pre->dT - dt) > .003 ||
            !pre->C.allFinite()) {
            result.reason = "invalid_imu_interval";
            return result;
        }
        const Eigen::Matrix<double, 9, 9> c = pre->C.block<9, 9>(0, 0).cast<double>();
        Eigen::LDLT<Eigen::Matrix<double, 9, 9>> ldlt(c);
        if (ldlt.info() != Eigen::Success || !ldlt.isPositive() ||
            (pre->C.block<3, 3>(9, 9).diagonal().array() <= 0.).any() ||
            (pre->C.block<3, 3>(12, 12).diagonal().array() <= 0.).any()) {
            result.reason = "invalid_imu_covariance";
            return result;
        }
    }

    g2o::SparseOptimizer optimizer;
    auto* linear = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    auto* block = new g2o::BlockSolverX(linear);
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(block));
    optimizer.setVerbose(false);
    std::vector<StateVertices> states;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        StateVertices s{new VertexPose(frames[i]), new VertexVelocity(frames[i]),
                        new VertexGyroBias(frames[i]), new VertexAccBias(frames[i])};
        const bool isFixed = i == 0 || fixed[i];
        s.pose->setId(4 * i); s.pose->setFixed(isFixed); optimizer.addVertex(s.pose);
        s.velocity->setId(4 * i + 1); s.velocity->setFixed(isFixed); optimizer.addVertex(s.velocity);
        if (!isFixed && lamaria_speed_prior::BabyStageEnabled()) lamaria_speed_prior::AddSpeedBound(optimizer, s.velocity);   // speedprior
        s.gyro->setId(4 * i + 2); s.gyro->setFixed(isFixed); optimizer.addVertex(s.gyro);
        s.acc->setId(4 * i + 3); s.acc->setFixed(isFixed); optimizer.addVertex(s.acc);
        states.push_back(s);
    }
    std::vector<EdgeInertial*> inertialEdges;
    for (std::size_t i = 1; i < frames.size(); ++i) {
        IMU::Preintegrated* pre = frames[i]->mpImuPreintegratedFrame;
        auto* imu = new EdgeInertial(pre);
        imu->setVertex(0, states[i-1].pose);
        imu->setVertex(1, states[i-1].velocity);
        imu->setVertex(2, states[i-1].gyro);
        imu->setVertex(3, states[i-1].acc);
        imu->setVertex(4, states[i].pose);
        imu->setVertex(5, states[i].velocity);
        optimizer.addEdge(imu);
        inertialEdges.push_back(imu);
        auto* gyro = new EdgeGyroRW();
        gyro->setVertex(0, states[i-1].gyro); gyro->setVertex(1, states[i].gyro);
        gyro->setInformation(pre->C.block<3,3>(9,9).cast<double>().inverse());
        optimizer.addEdge(gyro);
        auto* acc = new EdgeAccRW();
        acc->setVertex(0, states[i-1].acc); acc->setVertex(1, states[i].acc);
        acc->setInformation(pre->C.block<3,3>(12,12).cast<double>().inverse());
        optimizer.addEdge(acc);
    }

    std::vector<const BabyTrack*> ordered;
    for (const BabyTrack& t : tracks) ordered.push_back(&t);
    std::stable_sort(ordered.begin(), ordered.end(), [](const BabyTrack* a, const BabyTrack* b) {
        if (a->observations.size() != b->observations.size())
            return a->observations.size() > b->observations.size();
        return a->id < b->id;
    });
    std::vector<TrackEdges> graphTracks;
    int cameraTracks[2] = {0, 0};
    int nextId = 4 * frames.size();
    for (const BabyTrack* t : ordered) {
        int currentCamera = -1;
        std::set<std::size_t> observedFrames;
        bool duplicate = false;
        for (const BabyObservation& o : t->observations) {
            if (!ValidObservation(o, frames)) { duplicate = true; break; }
            if (!observedFrames.insert(o.frame).second) { duplicate = true; break; }
            if (o.frame == frames.size() - 1) currentCamera = o.camera;
        }
        if (duplicate || observedFrames.size() < 2 || currentCamera < 0) continue;
        bool validLensVariance = true;
        for (const BabyObservation& o : t->observations) {
            const double lensVariance = states[o.frame].pose->estimate().pCamera[o.camera]->uncertainty2(o.pixel);
            validLensVariance &= std::isfinite(lensVariance) && lensVariance > 0.;
        }
        if (!validLensVariance) continue;
        ++result.candidates;
        if (cameraTracks[currentCamera] >= kMaximumTracksPerCamera) continue;
        Eigen::Vector3d seed;
        if (!TriangulateSeed(*t, frames, states, seed)) continue;
        TrackEdges te;
        te.source = t;
        te.point = new g2o::VertexSBAPointXYZ();
        te.point->setId(nextId++);
        te.point->setEstimate(seed);
        te.point->setMarginalized(true);
        optimizer.addVertex(te.point);
        for (const BabyObservation& o : t->observations) {
            auto* edge = new EdgeMono(o.camera);
            edge->setVertex(0, te.point);
            edge->setVertex(1, states[o.frame].pose);
            edge->setMeasurement(o.pixel);
            const double lensVariance = states[o.frame].pose->estimate().pCamera[o.camera]->uncertainty2(o.pixel);
            const double inverseVariance = o.inverseVariance / std::max(1e-9, lensVariance);
            edge->setInformation(Eigen::Matrix2d::Identity() * inverseVariance);
            auto* robust = new g2o::RobustKernelHuber();
            robust->setDelta(std::sqrt(kVisualChi2));
            edge->setRobustKernel(robust);
            optimizer.addEdge(edge);
            te.edges.push_back(edge);
            te.frames.push_back(o.frame);
            te.cameras.push_back(o.camera);
        }
        graphTracks.push_back(te);
        ++cameraTracks[currentCamera];
        result.visualEdges += te.edges.size();
    }
    result.tracks = graphTracks.size();
    if (result.tracks < kMinimumCurrentTracks) {
        result.reason = "insufficient_triangulated_tracks";
        return result;
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(8);
    // Remove whole unsupported tracks so a free XYZ vertex cannot masquerade
    // as a valid single-frame visual constraint after historical rejection.
    for (TrackEdges& t : graphTracks) {
        int good = 0;
        bool current = false;
        for (std::size_t i = 0; i < t.edges.size(); ++i) {
            const bool inlier = GoodEdge(t.edges[i]);
            t.edges[i]->setLevel(inlier ? 0 : 1);
            good += inlier;
            current |= inlier && t.frames[i] == frames.size() - 1;
        }
        if (good < 2 || !current) {
            t.point->setFixed(true);
            for (EdgeMono* edge : t.edges) edge->setLevel(1);
        }
    }
    optimizer.initializeOptimization(0);
    optimizer.optimize(5);
    std::vector<double> chi2;
    for (TrackEdges& t : graphTracks) {
        int good = 0;
        EdgeMono* current = nullptr;
        int camera = -1;
        for (std::size_t i = 0; i < t.edges.size(); ++i) {
            if (t.edges[i]->level() != 0 || !GoodEdge(t.edges[i])) continue;
            ++good;
            if (t.frames[i] == frames.size() - 1) {
                current = t.edges[i]; camera = t.cameras[i];
            }
        }
        if (good >= 2 && current && t.point->estimate().allFinite()) {
            ++result.currentInliers[camera];
            chi2.push_back(current->chi2());
            BabyLandmark landmark;
            landmark.id = t.source->id;
            landmark.worldPoint = t.point->estimate();
            landmark.verifiedViews = good;
            std::vector<Eigen::Vector3d> viewingRays;
            for (std::size_t i = 0; i < t.edges.size(); ++i) {
                if (t.edges[i]->level() != 0 || !GoodEdge(t.edges[i])) continue;
                const BabyObservation& observation = t.source->observations[i];
                landmark.observations.push_back(observation);
                if (observation.frame == frames.size() - 1)
                    landmark.currentFeatureIndex = observation.featureIndex;
                const ImuCamPose& pose = states[observation.frame].pose->estimate();
                const Eigen::Vector3d center = -pose.Rcw[observation.camera].transpose() * pose.tcw[observation.camera];
                const Eigen::Vector3d ray = (landmark.worldPoint - center).normalized();
                for (const Eigen::Vector3d& previous : viewingRays)
                    landmark.parallaxDegrees = std::max(landmark.parallaxDegrees,
                        std::acos(std::max(-1., std::min(1., ray.dot(previous)))) * 180. / M_PI);
                viewingRays.push_back(ray);
            }
            result.landmarks.push_back(landmark);
        }
    }
    if (!chi2.empty()) {
        std::nth_element(chi2.begin(), chi2.begin() + chi2.size()/2, chi2.end());
        result.medianChi2 = chi2[chi2.size()/2];
    }
    if (result.currentInliers[0] + result.currentInliers[1] < kMinimumCurrentTracks) {
        result.reason = "insufficient_verified_tracks";
        return result;
    }
    for (std::size_t i = 0; i < inertialEdges.size(); ++i) {
        EdgeInertial* edge = inertialEdges[i];
        edge->computeError();
        if (!std::isfinite(edge->chi2())) {
            result.reason = "nonfinite_inertial_residual";
            return result;
        }
        result.inertialChi2 += edge->chi2();
        // Fixed historical map states can contain residuals that this solver
        // cannot alter; gate only intervals involving an optimized SOS state.
        if (!states[i].pose->fixed() || !states[i + 1].pose->fixed())
            result.maxActiveInertialChi2 = std::max(result.maxActiveInertialChi2, edge->chi2());
    }
    if (result.maxActiveInertialChi2 > 100.) {
        result.reason = "inertial_inconsistent";
        return result;
    }
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const StateVertices& s = states[i];
        if (!s.pose->estimate().Rwb.allFinite() || !s.pose->estimate().twb.allFinite() ||
            !s.velocity->estimate().allFinite() || !s.gyro->estimate().allFinite() ||
            !s.acc->estimate().allFinite()) {
            result.reason = "nonfinite_state";
            return result;
        }
        const double dp = (s.pose->estimate().twb - frames[i]->GetImuPosition().cast<double>()).norm();
        const double dv = (s.velocity->estimate() - frames[i]->GetVelocity().cast<double>()).norm();
        const Eigen::Matrix3d dr = s.pose->estimate().Rwb * frames[i]->GetImuRotation().cast<double>().transpose();
        const double angle = std::acos(std::max(-1., std::min(1., (dr.trace() - 1.) * .5)));
        if (i == frames.size() - 1) {
            result.poseDelta = dp; result.velocityDelta = dv; result.rotationDelta = angle;
        }
        if (dp > 2. || dv > 5. || s.velocity->estimate().norm() > 20. ||
            angle > .5 || s.gyro->estimate().norm() > 1. || s.acc->estimate().norm() > 5.) {
            result.reason = "excessive_state_change";
            return result;
        }
    }
    // Transaction boundary: nothing above mutates a caller frame or camera.
    for (std::size_t i = 1; i < frames.size(); ++i) {
        if (fixed[i]) continue;
        const StateVertices& s = states[i];
        frames[i]->SetImuPoseVelocity(s.pose->estimate().Rwb.cast<float>(),
            s.pose->estimate().twb.cast<float>(), s.velocity->estimate().cast<float>());
        frames[i]->mImuBias = IMU::Bias(s.acc->estimate()[0], s.acc->estimate()[1],
            s.acc->estimate()[2], s.gyro->estimate()[0], s.gyro->estimate()[1],
            s.gyro->estimate()[2]);
    }
    result.accepted = true;
    result.reason = "accepted";
    return result;
}
} // namespace ORB_SLAM3
