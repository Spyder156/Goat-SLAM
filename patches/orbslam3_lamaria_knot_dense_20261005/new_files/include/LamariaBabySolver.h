#ifndef LAMARIA_BABY_SOLVER_H
#define LAMARIA_BABY_SOLVER_H

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ORB_SLAM3 {
class Frame;

struct BabyObservation {
    std::size_t frame = 0;
    int camera = 0;
    Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
    double inverseVariance = 1.;
    int featureIndex = -1;
};

struct BabyTrack {
    std::uint64_t id = 0;
    std::vector<BabyObservation> observations;
};

struct BabyLandmark {
    std::uint64_t id = 0;
    Eigen::Vector3d worldPoint = Eigen::Vector3d::Zero();
    int currentFeatureIndex = -1;
    int verifiedViews = 0;
    double parallaxDegrees = 0.;
    std::vector<BabyObservation> observations;
};

struct BabySolveResult {
    bool accepted = false;
    std::string reason;
    int candidates = 0;
    int tracks = 0;
    int currentInliers[2] = {0, 0};
    int visualEdges = 0;
    double medianChi2 = -1.;
    double poseDelta = 0.;
    double velocityDelta = 0.;
    double inertialChi2 = 0.;
    double maxActiveInertialChi2 = 0.;
    double rotationDelta = 0.;
    std::vector<BabyLandmark> landmarks;
};

// Chronological, adjacent frames, with each frame i>0 holding the owned
// preintegration from i-1 to i. fixed identifies states already supported by
// the mature map. The oldest state is always conditioned on as a metric anchor.
// A rejected solve does not modify any Frame. An accepted solve updates only
// non-fixed pose/velocity/bias; no MapPoints, camera parameters or priors change.
// This bounded SOS experiment conditions on the oldest state rather than
// carrying a statistically complete marginalized window prior.
BabySolveResult SolveBabyWindow(const std::vector<Frame*>& frames,
                                const std::vector<BabyTrack>& tracks,
                                const std::vector<bool>& fixed);

} // namespace ORB_SLAM3
#endif
