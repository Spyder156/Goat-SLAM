#pragma once
// LamariaBabyAssist.h -- babyassist (2026-10-04): Baby support before tracking is lost, at low weight.
//
// Evidence: Long 1765-1800 s: inliers about 100 of 1480 keypoints while the local map grows by 1000
// fresh low-parallax points in 10 s; the official error steps 3.7 -> 4.1 m and the late scale creep
// begins there. Medium 1160-1235 s: inliers 35-74, scale creep, then the tail loss. SOS never fires in
// these sections because ordinary tracking never fails.
// Mechanism: a quality monitor (previous-frame inliers below kMinInliers, or local-map matches below
// kMinMatchRatio of the detected keypoints, for kFrames consecutive frames, IMU initialised, state OK)
// triggers one Baby window solve on a COPY of the current frame with every ordinary-tracked window
// frame fixed. Its verified landmarks (>= kMinViews views, >= kMinParallaxDeg parallax, feature not
// already associated to a MapPoint) become temporaries: extra pose-only reprojection edges on the
// current frame with information scaled by Weight() (0.2) and a Huber kernel. Map edges keep full
// weight, temporaries never become MapPoints, and the set is cleared after the frame.
// Conventions: world = point in the current map frame, metres; featureIndex indexes Frame keypoints
// (index >= Nleft means camera 1). Tracking thread only.
// Env: LAMARIA_BABY_ASSIST=0 disables; LAMARIA_BABY_ASSIST_WEIGHT overrides the weight.
#include <cstdlib>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace lamaria_baby_assist {

const int kMinInliers = 80;           // previous-frame local-map inliers below this count = thin
const double kMinMatchRatio = 0.05;   // local-map matches / detected keypoints below this = thin
// Calibration (OK frames, IMU initialised): inliers below 80 are the thinnest 5 % of Long frames and 9 % of
// Medium frames; a 10 % match ratio fired on 12-16 % of frames (healthy 2900-keypoint frames at 190
// inliers), 5 % fires on 2-3 %. The Long 1765-1775 s starvation runs at 50-130 inliers, ratio 0.02-0.09.
const int kFrames = 5;                // consecutive thin frames before assisting
const int kMinViews = 3;              // Baby landmark must be verified in this many views
const double kMinParallaxDeg = 1.0;   // and carry at least this parallax
const int kMaxTemporaries = 150;      // cap per frame

struct Temporary {
    Eigen::Vector3f world;   // current map frame, metres
    int featureIndex;        // index into Frame keypoints (>= Nleft: camera 1)
};
struct Report { int used = 0; double temporaryChi2 = 0.0; double mapChi2 = 0.0; };
struct State { std::vector<Temporary> pending; int thinFrames = 0; Report report; };
inline State& Get() { static State state; return state; }

inline bool Enabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("LAMARIA_BABY_ASSIST");
        return !(value && std::string(value) == "0");
    }();
    return enabled;
}
inline double Weight() {
    static const double weight = [] {
        const char* value = std::getenv("LAMARIA_BABY_ASSIST_WEIGHT");
        const double parsed = value ? std::atof(value) : 0.0;
        return (parsed > 0.0 && parsed <= 1.0) ? parsed : 0.2;
    }();
    return weight;
}
inline bool Thin(int previousInliers, int matches, int detected) {
    return previousInliers < kMinInliers || (detected > 0 && matches < kMinMatchRatio * detected);
}
// Once per frame before the pose optimisation. Counts consecutive thin frames; healthy frames reset it.
inline bool ShouldAssist(bool imuInitialised, bool stateOk, int previousInliers, int matches, int detected, bool force = false) {
    State& s = Get();
    if(!Enabled() || !imuInitialised || !stateOk) { s.thinFrames = 0; return false; }
    if(Thin(previousInliers, matches, detected)) ++s.thinFrames; else s.thinFrames = 0;
    return force || s.thinFrames >= kFrames;   // knotregime: forced on inside a knot
}
inline int ThinFrames() { return Get().thinFrames; }
inline void Set(const std::vector<Temporary>& temporaries) { Get().pending = temporaries; Get().report = Report(); }
inline const std::vector<Temporary>& Pending() { static const std::vector<Temporary> none; return Enabled() ? Get().pending : none; }
inline void Record(int used, double temporaryChi2, double mapChi2) {
    Report& r = Get().report; r.used = used; r.temporaryChi2 = temporaryChi2; r.mapChi2 = mapChi2;
}
// Returns the last optimisation's report and clears the temporaries (always call after the frame).
inline Report Take() { State& s = Get(); Report r = s.report; s.pending.clear(); s.report = Report(); return r; }

}  // namespace lamaria_baby_assist
