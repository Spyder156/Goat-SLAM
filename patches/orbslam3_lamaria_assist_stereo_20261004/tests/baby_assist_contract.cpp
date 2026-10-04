// baby_assist_contract.cpp -- falsifiable checks of the assist monitor and the temporaries hand-over.
#include "LamariaBabyAssist.h"
#include <cstdio>
#include <cmath>
namespace { int failures = 0;
void check(bool ok, const char* name, double value) { std::printf("%s %s (%.9g)\n", ok ? "ok  " : "FAIL", name, value); if(!ok) ++failures; } }
int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    using namespace lamaria_baby_assist;
    check(Enabled() && std::fabs(Weight() - 0.2) < 1e-12, "defaults: enabled, weight 0.2", Weight());
    check(Thin(79, 500, 1000) && Thin(300, 49, 1000) && !Thin(80, 50, 1000) && !Thin(190, 246, 2913), "thin = inliers < 80 or matches < 5 % of keypoints", 1);
    bool fired = false;
    for(int k = 0; k < kFrames - 1; ++k) fired |= ShouldAssist(true, true, 40, 50, 1000);
    check(!fired && ThinFrames() == kFrames - 1, "no assist before kFrames consecutive thin frames", ThinFrames());
    const bool fifth = ShouldAssist(true, true, 40, 50, 1000);
    check(fifth && ThinFrames() == kFrames, "assist on the kFrames-th consecutive thin frame", ThinFrames());
    check(!ShouldAssist(true, true, 300, 400, 1000) && ThinFrames() == 0, "one healthy frame resets the counter", ThinFrames());
    for(int k = 0; k < kFrames; ++k) ShouldAssist(true, true, 40, 50, 1000);
    check(!ShouldAssist(false, true, 40, 50, 1000) && ThinFrames() == 0, "no assist before IMU initialisation; counter reset", ThinFrames());
    for(int k = 0; k < kFrames; ++k) ShouldAssist(true, true, 40, 50, 1000);
    check(!ShouldAssist(true, false, 40, 50, 1000), "no assist unless tracking state is OK", 0);
    std::vector<Temporary> temporaries(3); temporaries[0].featureIndex = 7; temporaries[0].world = Eigen::Vector3f(1.f, 2.f, 3.f);
    Set(temporaries);
    check(Pending().size() == 3 && Pending()[0].featureIndex == 7, "temporaries handed over for the frame", Pending().size());
    Record(2, 1.5, 40.0);
    Report r = Take();
    check(r.used == 2 && std::fabs(r.temporaryChi2 - 1.5) < 1e-12 && Pending().empty(), "report returned and temporaries cleared after the frame", r.mapChi2);
    Report r2 = Take();
    check(r2.used == 0 && Pending().empty(), "a second take is empty", 0);
    if(failures) { std::printf("FAIL %d baby-assist checks\n", failures); return 1; }
    std::printf("PASS 10 baby-assist checks\n"); return 0;
}
