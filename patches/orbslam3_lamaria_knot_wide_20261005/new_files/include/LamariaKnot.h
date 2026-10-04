#pragma once
// LamariaKnot.h -- knotregime (2026-10-05): tracking regime for control-point "knots".
//
// Evidence (read-only, IMU + ground-truth speed, evaluation data): at every control point the wearer
// slows to about 0.5 m/s and sweeps the head; gyro RMS 1.3-1.7 rad/s against 0.7 while walking. A
// 10 s-median detector (speed < 0.8 m/s, gyro RMS > 1.1 rad/s, sustained) found 17/17 Medium and 27/27
// Long control points with zero false segments. Every tracking loss we have sits inside a knot: the
// stock keyframe rule is blind to rotation, so consecutive keyframes are 35 deg apart there (p90 140),
// the next sweep finds nothing mapped, Baby bridges, and the bridge deforms the map.
// Regime inside a knot:
//   K1 detector with hysteresis (enter after kEnterSeconds, leave after kExitSeconds);
//   K2 a keyframe at every sweep reversal (|omega| falls under kSlowRate after exceeding kFastRate)
//      and after every kKeyframeDegrees of integrated rotation, at most one per kMinKeyframeGap;
//   K3 Baby landmarks verified across sweeps are promoted into the map at each knot keyframe while
//      ordinary tracking is OK (the SOS-only bootstrap, applied where the knot needs it);
//   K4 the local inertial BA window is measured in time (kWindowSeconds) and, on the first keyframe
//      after a knot ends, spans the whole knot ("knot closure"); Baby assist is forced on in a knot.
// Conventions: gyro in rad/s (IMU body), speed in m/s (estimated body velocity, map frame), times in
// native seconds. Tracking thread owns Update/OnKeyframe; the mapper thread reads the window through
// the mutex. Env: LAMARIA_KNOT=0 disables the regime (detector still logs).
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include "KeyFrame.h"

namespace lamaria_knot {

const double kGyroRms = 1.1;            // rad/s, 10 s median above this
const double kSpeed = 0.8;              // m/s, 10 s median below this
const double kMedianSeconds = 10.0;     // detector window
const double kEnterSeconds = 3.0;       // condition must hold this long to enter
const double kExitSeconds = 3.0;        // and fail this long to leave
const double kKeyframeDegrees = 10.0;   // integrated rotation between knot keyframes
const double kFastRate = 1.0;           // rad/s, a sweep is "fast" above this
const double kSlowRate = 0.3;           // rad/s, a reversal is |omega| dipping under this after a fast phase
const double kMinKeyframeGap = 0.1;     // s, never more than 10 knot keyframes per second
const double kWindowSeconds = 5.0;      // local inertial BA window in time
const int kMaxWindowKeyframes = 60;     // cap for the closure window
const bool kWideSearchDefault = true;     // K5: double the projection search radius inside knots
const bool kDenseFeaturesDefault = false;  // K6: dense keypoint cache inside knots (needs KP_DIR_DENSE/KP_DIR1_DENSE)

struct Sample { double t; double gyro; double speed; };

struct State {
    std::deque<Sample> window;          // last kMedianSeconds of per-frame samples
    bool inKnot = false;
    double conditionSince = -1.0;       // when the raw condition started holding (or failing) continuously
    bool conditionLast = false;
    double enteredAt = 0.0;
    double rotSinceKeyframe = 0.0;      // rad, integrated |omega| since the last keyframe
    double lastKeyframeAt = -1.0;
    bool fastPhase = false;             // |omega| exceeded kFastRate since the last reversal
    bool reversalPending = false;
    int knotKeyframes = 0;
    int knotPromoted = 0;
    int knots = 0;
    // closure window handed to the mapper: valid once, from knotStart to knotEnd
    bool closurePending = false;
    double closureStart = 0.0, closureEnd = 0.0;
};
inline State& Get() { static State s; return s; }
inline std::mutex& Mutex() { static std::mutex m; return m; }
inline bool Enabled() {
    static const bool e = [] { const char* v = std::getenv("LAMARIA_KNOT"); return !(v && std::string(v) == "0"); }();
    return e;
}


// Variant switches (K5 wide search, K6 dense features in knots). Each knot package sets its own default;
// the environment can override: LAMARIA_KNOT_WIDE, LAMARIA_KNOT_DENSE (values "0"/"1").
inline bool EnvSwitch(const char* name, bool fallback) {
    const char* v = std::getenv(name);
    if(!v || !*v) return fallback;
    return std::string(v) != "0";
}
inline bool WideSearch() { static const bool w = EnvSwitch("LAMARIA_KNOT_WIDE", kWideSearchDefault); return w; }
inline bool DenseFeatures() { static const bool d = EnvSwitch("LAMARIA_KNOT_DENSE", kDenseFeaturesDefault); return d; }

inline double Median(std::vector<double> v) {
    if(v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// One call per frame after IMU preintegration. gyroRms: RMS |omega| over this frame's IMU samples;
// speed: previous frame's estimated speed (NaN while the IMU is not initialised); dt: frame interval.
// Returns true when the knot state changed.
inline bool Update(double t, double gyroRms, double speed, double dt) {
    std::lock_guard<std::mutex> lock(Mutex());
    State& s = Get();
    if(std::isfinite(gyroRms) && dt > 0.0) s.rotSinceKeyframe += gyroRms * dt;
    // sweep reversal: a fast phase followed by a dip under kSlowRate
    if(std::isfinite(gyroRms)) {
        if(gyroRms > kFastRate) s.fastPhase = true;
        else if(s.fastPhase && gyroRms < kSlowRate) { s.fastPhase = false; s.reversalPending = true; }
    }
    if(std::isfinite(gyroRms) && std::isfinite(speed)) s.window.push_back(Sample{t, gyroRms, speed});
    while(!s.window.empty() && t - s.window.front().t > kMedianSeconds) s.window.pop_front();
    std::vector<double> g, v; g.reserve(s.window.size()); v.reserve(s.window.size());
    for(size_t i = 0; i < s.window.size(); ++i) { g.push_back(s.window[i].gyro); v.push_back(s.window[i].speed); }
    const bool covered = !s.window.empty() && (t - s.window.front().t) >= 0.5 * kMedianSeconds;
    const bool condition = covered && Median(g) > kGyroRms && Median(v) < kSpeed;
    if(condition != s.conditionLast) { s.conditionLast = condition; s.conditionSince = t; }
    if(s.conditionSince < 0.0) s.conditionSince = t;
    const double held = t - s.conditionSince;
    bool changed = false;
    if(!s.inKnot && condition && held >= kEnterSeconds) {
        s.inKnot = true; s.enteredAt = t; s.knotKeyframes = 0; s.knotPromoted = 0; ++s.knots; changed = true;
        std::cout << std::setprecision(12) << "[KNOT] t=" << t << " enter gyro_rms=" << Median(g) << " speed=" << Median(v) << std::endl;
    } else if(s.inKnot && !condition && held >= kExitSeconds) {
        s.inKnot = false; changed = true;
        s.closurePending = true; s.closureStart = s.enteredAt; s.closureEnd = t;
        std::cout << std::setprecision(12) << "[KNOT] t=" << t << " exit duration=" << t - s.enteredAt
                  << " keyframes=" << s.knotKeyframes << " promoted=" << s.knotPromoted << std::endl;
    }
    return changed;
}
inline bool InKnot() { std::lock_guard<std::mutex> lock(Mutex()); return Enabled() && Get().inKnot; }
inline double RotationSinceKeyframeDeg() { std::lock_guard<std::mutex> lock(Mutex()); return Get().rotSinceKeyframe * 180.0 / M_PI; }

// Knot keyframe rule (K2). Consumes the reversal flag when it fires. `reason` receives why.
inline bool KeyframeDue(double t, std::string& reason) {
    std::lock_guard<std::mutex> lock(Mutex());
    State& s = Get();
    if(!Enabled() || !s.inKnot) return false;
    if(s.lastKeyframeAt >= 0.0 && t - s.lastKeyframeAt < kMinKeyframeGap) return false;
    if(s.reversalPending) { s.reversalPending = false; reason = "reversal"; return true; }
    if(s.rotSinceKeyframe * 180.0 / M_PI >= kKeyframeDegrees) { reason = "rotation"; return true; }
    return false;
}
// Called when a keyframe is created (any reason).
inline void OnKeyframe(double t) {
    std::lock_guard<std::mutex> lock(Mutex());
    State& s = Get(); s.rotSinceKeyframe = 0.0; s.lastKeyframeAt = t; if(s.inKnot) ++s.knotKeyframes;
}
inline void CountPromoted(int n) { std::lock_guard<std::mutex> lock(Mutex()); Get().knotPromoted += n; }

// K4: number of temporal keyframes the local inertial BA should optimise for keyframe pKF, never fewer
// than defaultMax. Normally the keyframes inside the last kWindowSeconds; once after a knot ends, the
// keyframes back to the knot start (capped). Mapper thread.
inline int TemporalWindowKeyframes(ORB_SLAM3::KeyFrame* pKF, int defaultMax, bool* closure = nullptr) {
    if(!Enabled() || !pKF) return defaultMax;
    double windowSeconds = kWindowSeconds; bool usedClosure = false;
    {
        std::lock_guard<std::mutex> lock(Mutex());
        State& s = Get();
        if(s.closurePending && pKF->mTimeStamp >= s.closureEnd - 1e-6) {
            windowSeconds = std::max(kWindowSeconds, pKF->mTimeStamp - s.closureStart + 2.0);
            s.closurePending = false; usedClosure = true;
        }
    }
    int count = 1; ORB_SLAM3::KeyFrame* k = pKF;
    while(k && k->mPrevKF && pKF->mTimeStamp - k->mPrevKF->mTimeStamp <= windowSeconds && count < kMaxWindowKeyframes) { k = k->mPrevKF; ++count; }
    if(closure) *closure = usedClosure;
    return std::max(defaultMax, count);
}
// Testable core of the window rule on plain timestamps (newest first).
inline int WindowCount(const std::vector<double>& newestFirst, double windowSeconds, int cap) {
    if(newestFirst.empty()) return 0;
    int count = 1;
    while(count < static_cast<int>(newestFirst.size()) && newestFirst[0] - newestFirst[count] <= windowSeconds && count < cap) ++count;
    return count;
}
// For tests: reset everything.
inline void ResetForTest() { std::lock_guard<std::mutex> lock(Mutex()); Get() = State(); }
inline int Knots() { std::lock_guard<std::mutex> lock(Mutex()); return Get().knots; }

}  // namespace lamaria_knot
