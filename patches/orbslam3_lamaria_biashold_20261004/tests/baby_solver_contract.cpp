#include "LamariaBabySolver.h"
#include "Frame.h"
#include "G2oTypes.h"
#include "CameraModels/Fisheye624.h"
#include "ORBextractor.h"
#include "Rig.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>

using namespace ORB_SLAM3;

namespace {
int checks = 0;
void Check(bool condition, const char* text) {
    if (!condition) throw std::runtime_error(text);
    ++checks;
    std::cout << "CHECK " << checks << " PASS " << text << std::endl;
}

struct Fixture {
    Fisheye624 left, right;
    IMU::Calib calibration;
    ORBextractor extractorLeft, extractorRight;
    std::vector<std::unique_ptr<Frame>> owned;
    std::vector<std::unique_ptr<IMU::Preintegrated>> integrations;
    std::vector<Frame*> frames;
    std::vector<bool> fixed;
    std::vector<BabyTrack> tracks;
    std::vector<Eigen::Vector3d> points;

    Fixture(int n = 8, bool stationary = false)
        : left(std::vector<float>{250,250,320,240,.01f,-.001f,0,0,0,0,.0001f,-.0002f,0,0,0,0}),
          right(std::vector<float>{250,250,320,240,.01f,-.001f,0,0,0,0,.0001f,-.0002f,0,0,0,0}),
          calibration(Sophus::SE3f(), .01f, .2f, .001f, .01f),
          extractorLeft(100,1.2,8,20,7), extractorRight(100,1.2,8,20,7) {
        const Eigen::Matrix3f rigRotation = Eigen::AngleAxisf(75.f * M_PI / 180.f,
            Eigen::Vector3f::UnitY()).toRotationMatrix();
        Sophus::SE3f bodyFromRight(rigRotation, Eigen::Vector3f(.137749f,0,0));
        Rig::PublishGlobals(false);
        cv::Mat image(480,640,CV_8UC1);
        cv::RNG random(31); random.fill(image,cv::RNG::UNIFORM,0,256);
        cv::Mat k = left.toK(), distortion = cv::Mat::zeros(4,1,CV_32F);
        Frame prototype(image,image,0.,&extractorLeft,&extractorRight,nullptr,
            k,distortion,30.f,10.f,&left,&right,bodyFromRight,nullptr,calibration);
        const Eigen::Vector3f velocity = stationary ? Eigen::Vector3f::Zero() :
                                                       Eigen::Vector3f(1.f,0.f,0.f);
        for (int i = 0; i < n; ++i) {
            owned.emplace_back(new Frame(prototype));
            Frame* f = owned.back().get();
            f->mpCamera = &left; f->mpCamera2 = &right;
            f->mImuCalib = calibration; f->mImuBias = IMU::Bias();
            f->mTimeStamp = 1. + .05 * i; f->mnId = i;
            f->mbf = 0.f;
            f->SetImuPoseVelocity(Eigen::Matrix3f::Identity(), velocity * (.05f * i), velocity);
            if (i) {
                integrations.emplace_back(new IMU::Preintegrated(IMU::Bias(), calibration));
                for (int j = 0; j < 10; ++j)
                    integrations.back()->IntegrateNewMeasurement(
                        Eigen::Vector3f(0,0,IMU::GRAVITY_VALUE), Eigen::Vector3f::Zero(), .005f);
                f->mpImuPreintegratedFrame = integrations.back().get();
            }
            frames.push_back(f);
            fixed.push_back(i != n - 1);
        }
        // Generate the two lenses' points in their own front hemispheres.
        // All projections and optimizer factors use production Fisheye624.
        for (int camera = 0; camera < 2; ++camera) {
            const ImuCamPose initial(frames[0]);
            for (int p = 0; p < 64; ++p) {
                Eigen::Vector3d local((p % 8 - 3.5) * .36, (p / 8 - 3.5) * .28,
                                      3. + .13 * (p % 11));
                Eigen::Vector3d world = initial.Rcw[camera].transpose() *
                    (local - initial.tcw[camera]);
                BabyTrack t; t.id = tracks.size();
                for (int i = 0; i < n; ++i) {
                    BabyObservation o;
                    o.frame = i; o.camera = camera; o.featureIndex = camera * 64 + p;
                    o.pixel = ImuCamPose(frames[i]).Project(world, camera);
                    t.observations.push_back(o);
                }
                points.push_back(world);
                tracks.push_back(t);
            }
        }
    }
};

double PoseError(Frame* f, const Eigen::Vector3f& position) {
    return (f->GetImuPosition() - position).norm();
}
}

int main() {
    try {
        cv::setNumThreads(1);
        Fixture clean;
        const Eigen::Matrix4f fixedPose = clean.frames[0]->GetPose().matrix();
        BabySolveResult exact = SolveBabyWindow(clean.frames, clean.tracks, clean.fixed);
        std::cout << "exact reason=" << exact.reason << " n=" << exact.currentInliers[0]
                  << "," << exact.currentInliers[1] << " chi=" << exact.medianChi2 << std::endl;
        Check(exact.accepted && exact.currentInliers[0] >= 40 && exact.currentInliers[1] >= 40,
              "native dual-fisheye temporaries constrain both cameras");
        Check(PoseError(clean.frames.back(), Eigen::Vector3f(.35f,0,0)) < 1e-4,
              "metric anchor and correct VI motion are preserved");
        Check((clean.frames[0]->GetPose().matrix() - fixedPose).norm() == 0.,
              "fixed map anchor remains untouched");
        double maximumPointError = 0.;
        for (const BabyLandmark& p : exact.landmarks)
            maximumPointError = std::max(maximumPointError, (p.worldPoint - clean.points.at(p.id)).norm());
        Check(maximumPointError < .002, "temporary depth comes from metric native multiview geometry");

        // A deliberately noisy inertial interval has a lateral displacement
        // prediction error. The historical poses anchor visual triangulation;
        // real visual factors must improve that prediction, not merely fit
        // arbitrary point depths around a fixed wrong current pose.
        Fixture drift;
        IMU::Preintegrated* pre = drift.frames.back()->mpImuPreintegratedFrame;
        pre->dP.y() += .08f; pre->dV.y() += .1f;
        pre->C.block<9,9>(0,0).setZero();
        pre->C.block<3,3>(0,0).diagonal().setConstant(1e-4f);
        pre->C.block<3,3>(3,3).diagonal().setConstant(.1f);
        pre->C.block<3,3>(6,6).diagonal().setConstant(.001f);
        drift.frames.back()->SetImuPoseVelocity(Eigen::Matrix3f::Identity(),
            Eigen::Vector3f(.35f,.08f,0), Eigen::Vector3f(1,.1f,0));
        const double before = PoseError(drift.frames.back(), Eigen::Vector3f(.35f,0,0));
        BabySolveResult correction = SolveBabyWindow(drift.frames, drift.tracks, drift.fixed);
        const double after = PoseError(drift.frames.back(), Eigen::Vector3f(.35f,0,0));
        std::cout << "correction reason=" << correction.reason << " before=" << before
                  << " after=" << after << " n=" << correction.currentInliers[0] << ","
                  << correction.currentInliers[1] << " imu=" << correction.maxActiveInertialChi2 << std::endl;
        Check(correction.accepted && after < .02 && after < .3 * before,
              "actual visual factors correct noisy-IMU current pose");

        Fixture bad;
        std::mt19937 random(20261003);
        std::vector<Eigen::Vector2d> shuffled;
        for (const BabyTrack& t : bad.tracks) shuffled.push_back(t.observations.back().pixel);
        std::shuffle(shuffled.begin(), shuffled.end(), random);
        for (std::size_t i = 0; i < bad.tracks.size(); ++i)
            bad.tracks[i].observations.back().pixel = shuffled[i];
        const Eigen::Matrix4f beforeBad = bad.frames.back()->GetPose().matrix();
        const Eigen::Vector3f beforeVelocity = bad.frames.back()->GetVelocity();
        BabySolveResult rejected = SolveBabyWindow(bad.frames, bad.tracks, bad.fixed);
        std::cout << "shuffled reason=" << rejected.reason << " n=" << rejected.currentInliers[0]
                  << "," << rejected.currentInliers[1] << std::endl;
        Check(!rejected.accepted, "shuffled temporal correspondences are rejected");
        Check((bad.frames.back()->GetPose().matrix() - beforeBad).norm() == 0. &&
              (bad.frames.back()->GetVelocity() - beforeVelocity).norm() == 0.,
              "rejection is transactional for caller pose and velocity");

        Fixture empty;
        Check(!SolveBabyWindow(empty.frames, {}, empty.fixed).accepted,
              "IMU alone cannot be reported as BabyFeature success");
        Fixture single;
        for (BabyTrack& t : single.tracks)
            t.observations = std::vector<BabyObservation>{t.observations.back()};
        Check(!SolveBabyWindow(single.frames, single.tracks, single.fixed).accepted,
              "single-view observations cannot supply temporary landmarks");
        Fixture stationary(8, true);
        BabySolveResult noParallax = SolveBabyWindow(stationary.frames, stationary.tracks, stationary.fixed);
        Check(!noParallax.accepted && noParallax.tracks == 0,
              "zero-parallax XYZ tracks are declined without invented depth");
        Fixture wrongInterval;
        wrongInterval.frames.back()->mpImuPreintegratedFrame->dT += .05f;
        Check(SolveBabyWindow(wrongInterval.frames, wrongInterval.tracks,
                            wrongInterval.fixed).reason == "invalid_imu_interval",
              "wrong IMU interval cannot silently connect nonadjacent frames");
        Fixture two(2);
        BabySolveResult twoView = SolveBabyWindow(two.frames, two.tracks, two.fixed);
        Check(twoView.accepted && PoseError(two.frames.back(), Eigen::Vector3f(.05f,0,0)) < .001,
              "two-view temporal features can support the established metric IMU state");

        Fixture rejoin;
        rejoin.fixed.assign(8, false); rejoin.fixed[0] = true; rejoin.fixed[7] = true;
        rejoin.frames[4]->SetImuPoseVelocity(Eigen::Matrix3f::Identity(),
            Eigen::Vector3f(.2f,.02f,0), Eigen::Vector3f(1,0,0));
        const Eigen::Matrix4f rejoinedPose = rejoin.frames.back()->GetPose().matrix();
        BabySolveResult smooth = SolveBabyWindow(rejoin.frames, rejoin.tracks, rejoin.fixed);
        Check(smooth.accepted && PoseError(rejoin.frames[4], Eigen::Vector3f(.2f,0,0)) < .002,
              "verified returning endpoint refines retained intervening states");
        Check((rejoin.frames.back()->GetPose().matrix() - rejoinedPose).norm() == 0.,
              "returning mature-map pose is fixed during interval refinement");
        std::cout << "PASS " << checks << " native solver checks" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << std::endl;
        return 1;
    }
}
