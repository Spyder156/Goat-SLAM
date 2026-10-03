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
};

struct MetricInertialInitState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame* keyFrame = nullptr;
    Sophus::SE3f sourcePose;
    // Map units / second. ApplyScaledRotation(..., scale, true) converts these
    // to metric velocities, rotating them into the accepted gravity frame.
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
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
