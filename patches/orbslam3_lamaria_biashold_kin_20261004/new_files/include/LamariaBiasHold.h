#pragma once
// LamariaBiasHold.h -- biashold (2026-10-04): hold the IMU biases while vision cannot observe them.
//
// Evidence: on Medium the final coast (1220-1256 s) ran to 5-12 m/s in both raw-walk runs (71.40,
// 78.59) while the walks/10 run kept 0.5-1.2 m/s through the same section and recovered (74.41);
// coastguard_long ended 103 m off after coasts at 3-7 m/s. During a coast or a Baby SOS bridge the
// bias states are unobservable, and their random walk is what poisons the velocity.
// Mechanism: when ordinary tracking is coasting, RECENTLY_LOST or bridged by Baby, the biases are
// held at the last well-observed value (the previous frame's bias when the hold begins):
//   - Baby window solve: every bias vertex set to the held value and fixed; frames keep it;
//   - pose-only inertial optimisation: random-walk edges scaled by Gain() so the bias cannot move
//     (the marginal prior stays consistent and relaxes within one frame after the hold ends);
//   - local inertial BA: random-walk edges of keyframes created inside a hold window scaled likewise.
// Conventions: timestamps are native seconds; IMU::Bias = (bax,bay,baz) m/s^2, (bwx,bwy,bwz) rad/s.
// Env: LAMARIA_BIAS_HOLD=0 disables; LAMARIA_BIAS_HOLD_GAIN overrides the gain (default 1e4).
// Thread note: Update/Observe run on the tracking thread; InWindow is read by local mapping.
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include "ImuTypes.h"

namespace lamaria_bias_hold {

struct State {
    bool active = false;
    ORB_SLAM3::IMU::Bias held;
    double since = 0.0;
    long frames = 0;
    double maxDeviation = 0.0;                        // largest |written bias - held| during the hold
    std::vector<std::pair<double, double> > windows;  // closed hold windows [begin, end]
};
inline State& Get() { static State state; return state; }
inline std::mutex& Mutex() { static std::mutex mutex; return mutex; }

inline bool Enabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("LAMARIA_BIAS_HOLD");
        return !(value && std::string(value) == "0");
    }();
    return enabled;
}
inline double Gain() {
    static const double gain = [] {
        const char* value = std::getenv("LAMARIA_BIAS_HOLD_GAIN");
        const double parsed = value ? std::atof(value) : 0.0;
        return parsed > 1.0 ? parsed : 1e4;
    }();
    return gain;
}
inline double Deviation(const ORB_SLAM3::IMU::Bias& b, const ORB_SLAM3::IMU::Bias& h) {
    return std::sqrt((b.bax-h.bax)*(b.bax-h.bax) + (b.bay-h.bay)*(b.bay-h.bay) + (b.baz-h.baz)*(b.baz-h.baz)
                   + (b.bwx-h.bwx)*(b.bwx-h.bwx) + (b.bwy-h.bwy)*(b.bwy-h.bwy) + (b.bwz-h.bwz)*(b.bwz-h.bwz));
}

// Once per frame, before any optimisation of that frame. `starved` describes the previous frame's
// outcome (coasting, RECENTLY_LOST or Baby bridging); `lastObservedBias` is the previous frame's bias.
inline void Update(bool starved, const ORB_SLAM3::IMU::Bias& lastObservedBias, double timestamp) {
    if(!Enabled()) return;
    std::lock_guard<std::mutex> lock(Mutex());
    State& s = Get();
    if(starved && !s.active) {
        s.active = true; s.held = lastObservedBias; s.since = timestamp; s.frames = 0; s.maxDeviation = 0.0;
        std::cout << std::setprecision(12) << "[BIAS_HOLD] t=" << timestamp
                  << " begin ba=" << s.held.bax << "," << s.held.bay << "," << s.held.baz
                  << " bg=" << s.held.bwx << "," << s.held.bwy << "," << s.held.bwz << std::endl;
    } else if(!starved && s.active) {
        s.active = false; s.windows.push_back(std::make_pair(s.since, timestamp));
        std::cout << std::setprecision(12) << "[BIAS_HOLD] t=" << timestamp << " end duration=" << timestamp - s.since
                  << " frames=" << s.frames << " max_deviation=" << s.maxDeviation << std::endl;
    }
    if(s.active) ++s.frames;
}
inline bool Active() { if(!Enabled()) return false; std::lock_guard<std::mutex> lock(Mutex()); return Get().active; }
inline ORB_SLAM3::IMU::Bias Held() { std::lock_guard<std::mutex> lock(Mutex()); return Get().held; }
// A bias written to a frame while holding: records how far it strayed from the held value (expect 0).
inline void Observe(const ORB_SLAM3::IMU::Bias& written) {
    if(!Enabled()) return;
    std::lock_guard<std::mutex> lock(Mutex());
    State& s = Get(); if(!s.active) return;
    const double d = Deviation(written, s.held); if(d > s.maxDeviation) s.maxDeviation = d;
}
// True when `timestamp` lies inside the current or any closed hold window.
inline bool InWindow(double timestamp) {
    if(!Enabled()) return false;
    std::lock_guard<std::mutex> lock(Mutex());
    const State& s = Get();
    if(s.active && timestamp >= s.since) return true;
    for(size_t i = 0; i < s.windows.size(); ++i)
        if(timestamp >= s.windows[i].first && timestamp <= s.windows[i].second) return true;
    return false;
}
inline size_t ClosedWindows() { std::lock_guard<std::mutex> lock(Mutex()); return Get().windows.size(); }

}  // namespace lamaria_bias_hold
