#ifndef ORB_SLAM3_METRIC_RIG_INITIALIZATION_H
#define ORB_SLAM3_METRIC_RIG_INITIALIZATION_H

#include "CameraModels/GeometricCamera.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ORB_SLAM3 {

// The same temporal landmark may contribute measurements from both bootstrap
// frames, but must retain the same landmarkId in both observations.
struct MetricRigScaleObservation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t landmarkId = 0;
    Eigen::Vector3d bootstrapPointCam0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d metricStereoPointCam0 = Eigen::Vector3d::Zero();
    Eigen::Vector2d pixelCam0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d pixelCam1 = Eigen::Vector2d::Zero();
    Eigen::Matrix3d R10 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t10Metres = Eigen::Vector3d::Zero();
    GeometricCamera* camera0 = nullptr;
    GeometricCamera* camera1 = nullptr;
};

using MetricRigScaleObservations = std::vector<MetricRigScaleObservation,
    Eigen::aligned_allocator<MetricRigScaleObservation>>;

struct MetricRigScaleOptions {
    size_t minimumIndependentLandmarks = 3;
    double maximumReprojectionErrorPixels = 2.5;
    double minimumConsensusFraction = 0.6;
    // Linearized log-scale uncertainty for one-pixel measurement noise.
    // Reject near-zero baseline / effectively infinite-depth constraints.
    double maximumLogScaleStdDev = 0.20;
};

struct MetricRigScaleResult {
    bool valid = false;
    double metresPerBootstrapUnit = 1.0;
    size_t candidateObservations = 0;
    size_t candidateLandmarks = 0;
    size_t inlierLandmarks = 0;
    double medianReprojectionErrorPixels = std::numeric_limits<double>::infinity();
    double logScaleStdDev = std::numeric_limits<double>::infinity();
    std::vector<size_t> inlierObservationIndices;
    std::string reason;
};

namespace metric_rig_detail {
inline double Median(std::vector<double> values) {
    if(values.empty()) return std::numeric_limits<double>::infinity();
    const size_t n=values.size();
    std::nth_element(values.begin(),values.begin()+n/2,values.end());
    const double upper=values[n/2];
    if(n%2) return upper;
    return 0.5*(upper+*std::max_element(values.begin(),values.begin()+n/2));
}

inline bool ProjectScaled(const MetricRigScaleObservation& observation,
                          double scale, Eigen::Vector2d& pixel,
                          Eigen::Vector2d* derivativeLogScale=nullptr) {
    // Crucial unit contract: scale the arbitrary-unit temporal point, while
    // keeping the calibrated inter-camera translation in metres.
    const Eigen::Vector3d rotated=observation.R10*(scale*observation.bootstrapPointCam0);
    const Eigen::Vector3d point1=rotated+observation.t10Metres;
    if(!point1.allFinite() || point1.z()<=0) return false;
    pixel=observation.camera1->project(point1);
    if(!pixel.allFinite()) return false;
    if(derivativeLogScale) {
        *derivativeLogScale=observation.camera1->projectJac(point1)*rotated;
        if(!derivativeLogScale->allFinite()) return false;
    }
    return true;
}

struct Consensus {
    std::vector<size_t> indices;
    size_t landmarks=0;
    double medianError=std::numeric_limits<double>::infinity();
};

inline Consensus Score(const MetricRigScaleObservations& observations,
                       const std::vector<size_t>& candidates,double scale,
                       double maximumError) {
    Consensus result;
    std::map<uint64_t,double> errorByLandmark;
    for(size_t index:candidates) {
        const auto& observation=observations[index];
        Eigen::Vector2d pixel;
        if(!ProjectScaled(observation,scale,pixel)) continue;
        const double error=(pixel-observation.pixelCam1).norm();
        if(error>maximumError) continue;
        result.indices.push_back(index);
        auto found=errorByLandmark.find(observation.landmarkId);
        if(found==errorByLandmark.end() || error<found->second)
            errorByLandmark[observation.landmarkId]=error;
    }
    result.landmarks=errorByLandmark.size();
    std::vector<double> errors;
    for(const auto& item:errorByLandmark) errors.push_back(item.second);
    result.medianError=Median(errors);
    return result;
}
} // namespace metric_rig_detail

// No GT, IMU or state mutation. Call before map creation to reject unsupported
// initialization, then again after temporal BA with updated camera0 points.
inline MetricRigScaleResult EstimateMetricRigScale(
    const MetricRigScaleObservations& observations,
    const MetricRigScaleOptions& options=MetricRigScaleOptions()) {
    using namespace metric_rig_detail;
    MetricRigScaleResult result;
    std::vector<size_t> candidates;
    std::vector<double> hypotheses;
    std::set<uint64_t> identities;
    for(size_t index=0;index<observations.size();++index) {
        const auto& observation=observations[index];
        if(!observation.camera0 || !observation.camera1 ||
           !observation.bootstrapPointCam0.allFinite() ||
           !observation.metricStereoPointCam0.allFinite() ||
           !observation.R10.allFinite() || !observation.t10Metres.allFinite() ||
           !observation.pixelCam0.allFinite() || !observation.pixelCam1.allFinite() ||
           observation.bootstrapPointCam0.z()<=0 || observation.metricStereoPointCam0.z()<=0)
            continue;
        const Eigen::Vector3d stereo1=observation.R10*observation.metricStereoPointCam0+observation.t10Metres;
        if(stereo1.z()<=0) continue;
        const Eigen::Vector2d bootstrapPixel=observation.camera0->project(observation.bootstrapPointCam0);
        const Eigen::Vector2d stereoPixel0=observation.camera0->project(observation.metricStereoPointCam0);
        const Eigen::Vector2d stereoPixel1=observation.camera1->project(stereo1);
        const double maximum=options.maximumReprojectionErrorPixels;
        if(!bootstrapPixel.allFinite() || !stereoPixel0.allFinite() || !stereoPixel1.allFinite() ||
           (bootstrapPixel-observation.pixelCam0).norm()>maximum ||
           (stereoPixel0-observation.pixelCam0).norm()>maximum ||
           (stereoPixel1-observation.pixelCam1).norm()>maximum) continue;
        const double scale=observation.metricStereoPointCam0.z()/observation.bootstrapPointCam0.z();
        if(!std::isfinite(scale) || scale<=0) continue;
        candidates.push_back(index);hypotheses.push_back(scale);identities.insert(observation.landmarkId);
    }
    result.candidateObservations=candidates.size();
    result.candidateLandmarks=identities.size();
    if(identities.size()<options.minimumIndependentLandmarks) {
        result.reason="fewer than three independent metric stereo landmarks";return result;
    }

    Consensus best;
    double scale=1.0;
    for(double hypothesis:hypotheses) {
        Consensus candidate=Score(observations,candidates,hypothesis,options.maximumReprojectionErrorPixels);
        if(candidate.landmarks>best.landmarks ||
           (candidate.landmarks==best.landmarks && candidate.medianError<best.medianError)) {
            best=candidate;scale=hypothesis;
        }
    }
    const size_t minimumConsensus=std::max(options.minimumIndependentLandmarks,
        static_cast<size_t>(std::ceil(options.minimumConsensusFraction*identities.size())));
    if(best.landmarks<minimumConsensus) {
        result.reason="metric stereo landmarks do not agree on a shared scale";return result;
    }

    // One-dimensional refinement, on the consensus, in log scale so scale
    // remains positive. Each landmark receives at most one unit of weight.
    for(int iteration=0;iteration<12;++iteration) {
        std::map<uint64_t,size_t> repeats;
        for(size_t index:best.indices) ++repeats[observations[index].landmarkId];
        double gradient=0,information=0;
        for(size_t index:best.indices) {
            const auto& observation=observations[index];
            Eigen::Vector2d pixel,J;
            if(!ProjectScaled(observation,scale,pixel,&J)) continue;
            const Eigen::Vector2d residual=pixel-observation.pixelCam1;
            const double weight=1.0/repeats[observation.landmarkId];
            gradient+=weight*J.dot(residual);information+=weight*J.squaredNorm();
        }
        if(information<1e-12) break;
        const double step=std::max(-0.25,std::min(0.25,-gradient/information));
        if(std::abs(step)<1e-7) break;
        const double refinedScale=scale*std::exp(step);
        Consensus refined=Score(observations,candidates,refinedScale,options.maximumReprojectionErrorPixels);
        if(refined.landmarks<best.landmarks ||
           (refined.landmarks==best.landmarks && refined.medianError>best.medianError)) break;
        best=refined;scale=refinedScale;
    }

    std::map<uint64_t,double> informationByLandmark;
    for(size_t index:best.indices) {
        const auto& observation=observations[index];Eigen::Vector2d pixel,J;
        if(ProjectScaled(observation,scale,pixel,&J))
            informationByLandmark[observation.landmarkId]=std::max(informationByLandmark[observation.landmarkId],J.squaredNorm());
    }
    double information=0;
    for(const auto& item:informationByLandmark) information+=item.second;
    result.metresPerBootstrapUnit=scale;
    result.inlierLandmarks=best.landmarks;
    result.inlierObservationIndices=best.indices;
    result.medianReprojectionErrorPixels=best.medianError;
    result.logScaleStdDev=information>0?1.0/std::sqrt(information):std::numeric_limits<double>::infinity();
    if(!std::isfinite(result.logScaleStdDev) || result.logScaleStdDev>options.maximumLogScaleStdDev) {
        result.reason="stereo reprojection has insufficient metric scale sensitivity";return result;
    }
    result.valid=true;result.reason="metric scale supported by independent stereo landmarks";
    return result;
}

} // namespace ORB_SLAM3
#endif
