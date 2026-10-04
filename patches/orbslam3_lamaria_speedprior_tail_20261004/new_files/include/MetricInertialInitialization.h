#ifndef ORB_SLAM3_METRIC_INERTIAL_INITIALIZATION_H
#define ORB_SLAM3_METRIC_INERTIAL_INITIALIZATION_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>
#include <limits>
#include <string>
#include <vector>

namespace ORB_SLAM3 {
class KeyFrame;
class MapPoint;
class GeometricCamera;

struct MetricInertialInitOptions {
    size_t minimumKeyFrames = 10;
    size_t maximumKeyFrames = 80;
    double minimumDurationSeconds = 3.0;
    size_t minimumStereoLandmarks = 20;
    size_t minimumStereoKeyFrames = 3;
    double minimumTravelMetres = 0.3;
    double minimumAccelerationExcitation = 0.1;
    double maximumLogScaleStdDev = 0.05;
    double minimumScale = 0.5;
    double maximumScale = 2.0;
    double maximumGyroBiasNorm = 0.2;
    double maximumAccBiasNorm = 1.0;
    double maximumStereoMedianPixels = 2.5;
    double maximumStereoMedianIncreasePixels = 0.25;
    double minimumStereoInlierFraction = 0.6;
    double priorG = 100.0;
    double priorA = 1e5;
    int iterations = 100;
    bool jointRefinement = false;
    int jointIterations = 30;
    double minimumVisualInlierRetention = 0.9;
    // Both experiment arms use the same held-out measurements and fixed-camera
    // reference optimization. Only the second arm releases radial coefficients.
    bool radialCalibrationExperiment = false;
    bool optimizeRadialCalibration = false;
    Eigen::Vector2d radialMaximumDelta = Eigen::Vector2d(0.02,0.01);
    Eigen::Vector2d radialPriorStdDev = Eigen::Vector2d(0.005,0.0025);
    int radialIterations = 30;
    size_t minimumRadialHeldoutPerCamera = 30;
    size_t minimumRadialPeripheralHeldoutPerCamera = 10;
    double radialMaximumHeldoutIncreasePixels = 0.05;
    double radialMinimumHeldoutImprovementPixels = 0.005;
};

struct MetricRadialResidualSummary {
    // camera, radial bin (<250 pixels / >=250 pixels from principal point).
    size_t count[2][2] = {{0,0},{0,0}};
    size_t inliers[2][2] = {{0,0},{0,0}};
    double medianPixels[2][2] = {{0,0},{0,0}};
    double rmsPixels[2][2] = {{0,0},{0,0}};
};

struct MetricInertialInitState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame* keyFrame = nullptr;
    Sophus::SE3f sourcePose;
    // Map units / second. ApplyScaledRotation(..., scale, true) converts these
    // to metric velocities, rotating them into the accepted gravity frame.
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    // Only populated by jointRefinement: ready-to-commit world-to-camera pose
    // and body velocity in the final gravity-aligned metric world.
    Sophus::SE3f optimizedPose;
    Eigen::Vector3f optimizedVelocity = Eigen::Vector3f::Zero();
};

struct MetricInertialInitPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint* mapPoint = nullptr;
    Eigen::Vector3f sourcePosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f optimizedPosition = Eigen::Vector3f::Zero();
};

struct MetricInertialInitBoundary {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame* keyFrame = nullptr;
    Sophus::SE3f sourcePose;
};

struct MetricInertialInitResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool accepted = false;
    std::string reason;
    double scale = 1.0;
    Eigen::Matrix3d Rwg = Eigen::Matrix3d::Identity();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    std::vector<MetricInertialInitState, Eigen::aligned_allocator<MetricInertialInitState>> states;
    bool jointRefined = false;
    bool radialAttempted = false, radialCalibrated = false;
    bool radialCompleteMapCoverage = false;
    std::string radialReason;
    GeometricCamera* radialCameras[2] = {nullptr,nullptr};
    std::vector<float> radialSourceParameters[2], radialFinalParameters[2];
    Eigen::Vector2d radialDelta[2] = {Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
    Eigen::Vector2d radialStdDev[2] = {Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())};
    size_t radialHessianDimension = 0, radialHessianRank = 0;
    double radialLogBaselineStdDev = std::numeric_limits<double>::infinity();
    // "Final" summaries/deltas describe the attempted calibrated candidate,
    // even when rejected. radialFinalParameters and returned states always
    // describe the accepted solution (the fixed-camera fallback on rejection).
    MetricRadialResidualSummary radialInitialTraining, radialFixedTraining, radialFinalTraining;
    MetricRadialResidualSummary radialInitialHeldout, radialFixedHeldout, radialFinalHeldout;
    double radialFixedCost = std::numeric_limits<double>::infinity();
    double radialInitialCost = std::numeric_limits<double>::infinity();
    double radialFinalCost = std::numeric_limits<double>::infinity();
    std::vector<MetricInertialInitPoint, Eigen::aligned_allocator<MetricInertialInitPoint>> points;
    std::vector<MetricInertialInitBoundary, Eigen::aligned_allocator<MetricInertialInitBoundary>> boundaryStates;
    size_t visualObservations[2] = {0,0};
    size_t initialVisualInliers[2] = {0,0}, finalVisualInliers[2] = {0,0};
    double initialVisualMedianPixels[2] = {0,0}, finalVisualMedianPixels[2] = {0,0};
    double jointInitialCost = std::numeric_limits<double>::infinity();
    double jointFinalCost = std::numeric_limits<double>::infinity();
    double jointLogBaselineStdDev = std::numeric_limits<double>::infinity();
    size_t jointHessianDimension = 0, jointHessianRank = 0;
    double jointMinimumNormalisedEigenvalue = 0;
    unsigned long conditioningKeyFrameId = 0;
    unsigned long sourceMapId = 0;
    int sourceWorldFrameVersion = 0, sourceMapChangeIndex = 0;
    size_t keyFrames = 0, imuEdges = 0, stereoObservations = 0;
    size_t stereoLandmarks = 0, stereoKeyFrames = 0, stereoInliers = 0;
    double durationSeconds = 0, travelMetres = 0, accelerationExcitation = 0;
    double initialCost = std::numeric_limits<double>::infinity();
    double finalCost = std::numeric_limits<double>::infinity();
    double initialStereoMedianPixels = std::numeric_limits<double>::infinity();
    double finalStereoMedianPixels = std::numeric_limits<double>::infinity();
    // Conditional on the captured visual poses/landmarks. Nuisance velocities,
    // gravity and biases are marginalized, without their stabilizing priors.
    // This is deliberately NOT presented as a full joint-BA covariance.
    double conditionalLogScaleStdDev = std::numeric_limits<double>::infinity();
    double imuOnlyLogScaleStdDev = std::numeric_limits<double>::infinity();
};
}
#endif
