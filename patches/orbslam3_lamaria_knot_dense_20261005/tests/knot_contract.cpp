// knot_contract.cpp -- falsifiable checks of the knot detector, the knot keyframe rule and the window rule.
#include "LamariaKnot.h"
#include <cstdio>
#include <cmath>
namespace { int failures = 0;
void check(bool ok, const char* name, double value) { std::printf("%s %s (%.9g)\n", ok ? "ok  " : "FAIL", name, value); if(!ok) ++failures; } }
int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    using namespace lamaria_knot;
    ResetForTest();
    check(Enabled(), "regime enabled by default", 1);
    // Walking for 20 s at 20 Hz: gyro 0.7 rad/s, speed 1.4 m/s -> never a knot
    double t = 100.0; const double dt = 0.05;
    for(int i = 0; i < 400; ++i) { Update(t, 0.7, 1.4, dt); t += dt; }
    check(!InKnot() && Knots() == 0, "walking never enters a knot", Knots());
    // Knot motion: gyro 1.5 rad/s, speed 0.5 m/s. Not yet after 5 s (median still walking), in after 12 s.
    for(int i = 0; i < 100; ++i) { Update(t, 1.5, 0.5, dt); t += dt; }
    const bool early = InKnot();
    for(int i = 0; i < 140; ++i) { Update(t, 1.5, 0.5, dt); t += dt; }
    check(!early && InKnot() && Knots() == 1, "knot entered after the medians flip and the condition holds", Knots());
    std::string reason;
    // Keyframe rule: rotation accumulates since OnKeyframe; 10 deg reached after 0.1745 rad / 1.5 = 0.116 s
    OnKeyframe(t); t += dt;
    Update(t, 1.5, 0.5, dt); t += dt;                       // 0.075 rad = 4.3 deg
    const bool tooEarly = KeyframeDue(t, reason);
    Update(t, 1.5, 0.5, dt); t += dt; Update(t, 1.5, 0.5, dt); t += dt;   // 0.225 rad = 12.9 deg, still under 15
    const bool stillEarly = KeyframeDue(t + 0.1, reason);
    Update(t, 1.5, 0.5, dt); t += dt;                       // 0.300 rad = 17.2 deg
    const bool due = KeyframeDue(t + 0.2, reason);
    check(!tooEarly && !stillEarly && due && reason == "rotation", "knot keyframe after 15 deg of integrated rotation", RotationSinceKeyframeDeg());
    OnKeyframe(t);
    check(!KeyframeDue(t + 0.2, reason) && RotationSinceKeyframeDeg() == 0.0, "keyframe bookkeeping resets the rotation", RotationSinceKeyframeDeg());
    // Reversal: fast phase then a dip under 0.3 rad/s
    t += 0.2; Update(t, 1.5, 0.5, dt); t += dt; Update(t, 0.2, 0.5, dt); t += dt;
    const bool reversal = KeyframeDue(t, reason);
    const bool consumed = !KeyframeDue(t + 0.01, reason);
    check(reversal && reason == "reversal" && consumed, "a sweep reversal triggers one keyframe", 1);
    // A 2 s dip in the gyro does not leave the knot (10 s median, 3 s hysteresis)
    for(int i = 0; i < 40; ++i) { Update(t, 0.5, 0.5, dt); t += dt; }
    check(InKnot(), "a 2 s dip keeps the knot", 1);
    // Walking again: knot left within 15 s, closure window recorded once
    for(int i = 0; i < 300; ++i) { Update(t, 0.7, 1.4, dt); t += dt; }
    const State& s = Get();
    check(!InKnot() && s.closurePending && s.closureStart < s.closureEnd && s.closureEnd - s.closureStart > 10.0, "knot exit records the closure window", s.closureEnd - s.closureStart);
    std::vector<double> kf{100.0, 99.7, 99.3, 94.9, 94.0};
    check(WindowCount(kf, 5.0, 60) == 3 && WindowCount(kf, 5.0, 2) == 2 && WindowCount(kf, 7.0, 60) == 5, "time-based window counts keyframes within the span, capped", WindowCount(kf, 5.0, 60));
    if(failures) { std::printf("FAIL %d knot checks\n", failures); return 1; }
    std::printf("PASS 8 knot checks\n"); return 0;
}
