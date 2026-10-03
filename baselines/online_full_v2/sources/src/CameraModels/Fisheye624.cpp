/**
 * Fisheye624 implementation. Math ported from mecka-basalt
 * basalt-headers/include/basalt/camera/fisheye624_camera.hpp (verified vs
 * pycolmap RAD_TAN_THIN_PRISM_FISHEYE to 6.4e-10 px).
 *
 * Conventions:
 *   params: [fx fy cx cy k0 k1 k2 k3 k4 k5 p0 p1 s0 s1 s2 s3]
 *   tangential pairing (Meta): u += p0*(rho+2a^2) + 2 p1 a b
 *                              v += p1*(rho+2b^2) + 2 p0 a b
 *   unproject returns a unit bearing, or NaNs for an invalid inverse.
 */
#include "Fisheye624.h"
#include "TwoViewReconstruction.h"

#include <boost/serialization/export.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

// registration id distinct from Pinhole/KannalaBrandt8
//BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::Fisheye624)

namespace ORB_SLAM3 {

void Fisheye624::setValidDomain(int width, int height, double radiusX,
                              double radiusY, double maxSolidAngle) {
    const bool noRadius = radiusX == -1.0 && radiusY == -1.0;
    if(width <= 0 || height <= 0 ||
       (!noRadius && (!std::isfinite(radiusX) || !std::isfinite(radiusY) ||
                      radiusX <= 0.0 || radiusY <= 0.0)) ||
       !std::isfinite(maxSolidAngle) || maxSolidAngle <= 0.0 ||
       maxSolidAngle > 1.5707963267948966)
        throw std::invalid_argument("Fisheye624 invalid domain metadata; this inverse supports the forward hemisphere");
    mImageWidth = width; mImageHeight = height;
    mDomainCenterX=mvParameters[2];mDomainCenterY=mvParameters[3];
    mValidRadiusX = radiusX; mValidRadiusY = radiusY;
    mMaxSolidAngle = maxSolidAngle;
}

bool Fisheye624::tryUnproject(const cv::Point2f& pixel, cv::Point3f& unitRay) const {
    return tryUnprojectImpl(pixel,unitRay,true);
}
bool Fisheye624::tryUnprojectGeometry(const cv::Point2f& pixel,cv::Point3f& unitRay) const {
    return tryUnprojectImpl(pixel,unitRay,false);
}
bool Fisheye624::tryUnprojectImpl(const cv::Point2f& pixel,cv::Point3f& unitRay,bool angularPolicy) const {
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    unitRay = cv::Point3f(invalid, invalid, invalid);
    if(!std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
       !std::isfinite(mvParameters[0]) || !std::isfinite(mvParameters[1]) ||
       mvParameters[0] <= 0.f || mvParameters[1] <= 0.f)
        return false;
    // Match the SDK's continuous pixel area, including half-pixel borders.
    if((mImageWidth > 0 && (pixel.x < -0.5 || pixel.x > mImageWidth - 0.5)) ||
       (mImageHeight > 0 && (pixel.y < -0.5 || pixel.y > mImageHeight - 0.5)))
        return false;
    const double dx = double(pixel.x) - mvParameters[2];
    const double dy = double(pixel.y) - mvParameters[3];
    const double maskX=double(pixel.x)-mDomainCenterX,maskY=double(pixel.y)-mDomainCenterY;
    if(mValidRadiusX > 0.0 &&
       maskX*maskX/(mValidRadiusX*mValidRadiusX) + maskY*maskY/(mValidRadiusY*mValidRadiusY) > 1.0)
        return false;
    double x, y;
    if(!undistort(dx/mvParameters[0], dy/mvParameters[1], x, y) ||
       !std::isfinite(x) || !std::isfinite(y))
        return false;
    const double radius = std::hypot(x, y);
    if(angularPolicy && std::atan(radius) > mMaxSolidAngle)
        return false;
    const double norm = std::sqrt(x*x + y*y + 1.0);
    if(!std::isfinite(norm) || norm <= 0.0)
        return false;
    unitRay = cv::Point3f(float(x/norm), float(y/norm), float(1.0/norm));
    return true;
}

void Fisheye624::distort(double a, double b, double& un, double& vn) const {
    const double* k = nullptr;
    // fisheye theta-polynomial on the (a,b) = (X/Z, Y/Z) plane
    const double r2 = a * a + b * b;
    const double r = std::sqrt(r2);
    double xr = a, yr = b;
    if (r > 1e-12) {
        const double theta = std::atan(r);
        const double t2 = theta * theta;
        double poly = 1.0;
        double tp = t2;
        for (int i = 0; i < 6; i++) {
            poly += double(mvParameters[4 + i]) * tp;
            tp *= t2;
        }
        const double theta_d = theta * poly;
        const double scaling = theta_d / r;
        xr = a * scaling;
        yr = b * scaling;
    }
    // tangential (Meta pairing) + thin prism on the fisheye-distorted coords
    const double p0 = mvParameters[10], p1 = mvParameters[11];
    const double s0 = mvParameters[12], s1 = mvParameters[13];
    const double s2 = mvParameters[14], s3 = mvParameters[15];
    const double rho = xr * xr + yr * yr;
    const double rho2 = rho * rho;
    un = xr + p0 * (rho + 2.0 * xr * xr) + 2.0 * p1 * xr * yr + s0 * rho +
         s1 * rho2;
    vn = yr + p1 * (rho + 2.0 * yr * yr) + 2.0 * p0 * xr * yr + s2 * rho +
         s3 * rho2;
    (void)k;
}

bool Fisheye624::undistort(double mx, double my, double& x, double& y) const {
    if(!std::isfinite(mx) || !std::isfinite(my)) return false;
    // Newton on the FULL distortion (tangential+prism+fisheye).
    // Init matters at wide angle: the measurement (mx,my) is theta-COMPRESSED,
    // so starting Newton there diverges in the corners (57deg+). Undo the
    // dominant fisheye compression first: treat r_meas as theta_d, invert the
    // theta-polynomial for theta, start at tan(theta) in the same direction.
    double a = mx, b = my;
    {
        const double rd = std::hypot(mx, my);
        if (rd > 1e-9) {
            double theta = rd;
            for (int it = 0; it < 20; it++) {
                const double t2 = theta * theta;
                double poly = 1.0, dpoly = 0.0, tp = 1.0;
                for (int i = 0; i < 6; i++) {
                    poly  += double(mvParameters[4 + i]) * tp * t2;
                    dpoly += double(mvParameters[4 + i]) * (2*i+2) * tp * theta;
                    tp *= t2;
                }
                const double f = theta * poly - rd;
                const double df = poly + theta * dpoly;
                if (std::fabs(df) < 1e-12) break;
                const double step = f / df;
                theta -= step;
                if (std::fabs(step) < 1e-12) break;
            }
            const double scale = std::tan(theta) / rd;
            a = mx * scale;
            b = my * scale;
        }
    }
    for (int it = 0; it < 50; it++) {
        double u, v;
        distort(a, b, u, v);
        const double ru = u - mx, rv = v - my;
        if (ru * ru + rv * rv < 1e-16) break;
        // numeric 2x2 Jacobian of distort at (a,b)
        const double eps = 1e-7;
        double u1, v1, u2, v2;
        distort(a + eps, b, u1, v1);
        distort(a, b + eps, u2, v2);
        const double J00 = (u1 - u) / eps, J01 = (u2 - u) / eps;
        const double J10 = (v1 - v) / eps, J11 = (v2 - v) / eps;
        const double det = J00 * J11 - J01 * J10;
        if (!std::isfinite(det) || std::fabs(det) < 1e-14) return false;
        a -= (J11 * ru - J01 * rv) / det;
        b -= (-J10 * ru + J00 * rv) / det;
    }
    // distort() maps UNDISTORTED tan-plane coords directly to the measured
    // normalized coords, so the converged Newton solution IS the answer.
    // (An extra theta-polynomial inversion here would double-invert.)
    double u, v;
    distort(a, b, u, v);
    const double res = std::hypot(u - mx, v - my);
    x = a;
    y = b;
    return std::isfinite(res) && std::isfinite(x) && std::isfinite(y) && res < 1e-8;
}

static inline void fe624_dir_to_thetad(const float* P, double x, double y,
                                       double z, double& xr, double& yr) {
    // theta = atan2(r, z): finite for EVERY direction, including z <= 0
    // (points behind the camera during BA iterations). The naive a = x/z
    // shortcut explodes there and feeds NaN into the optimizer.
    const double r = std::sqrt(x * x + y * y);
    if (r < 1e-12) { xr = 0.0; yr = 0.0; return; }
    const double theta = std::atan2(r, z);
    const double t2 = theta * theta;
    double poly = 1.0, tp = t2;
    for (int i = 0; i < 6; i++) { poly += double(P[4 + i]) * tp; tp *= t2; }
    const double theta_d = theta * poly;
    xr = theta_d * x / r;
    yr = theta_d * y / r;
}

Eigen::Vector2d Fisheye624::project(const Eigen::Vector3d& v3D) {
    double xr, yr;
    fe624_dir_to_thetad(mvParameters.data(), v3D[0], v3D[1], v3D[2], xr, yr);
    // tangential (Meta pairing) + thin prism on the theta-distorted coords
    const double p0 = mvParameters[10], p1 = mvParameters[11];
    const double s0 = mvParameters[12], s1 = mvParameters[13];
    const double s2 = mvParameters[14], s3 = mvParameters[15];
    const double rho = xr * xr + yr * yr, rho2 = rho * rho;
    const double un = xr + p0 * (rho + 2.0 * xr * xr) + 2.0 * p1 * xr * yr
                      + s0 * rho + s1 * rho2;
    const double vn = yr + p1 * (rho + 2.0 * yr * yr) + 2.0 * p0 * xr * yr
                      + s2 * rho + s3 * rho2;
    return Eigen::Vector2d(mvParameters[0] * un + mvParameters[2],
                           mvParameters[1] * vn + mvParameters[3]);
}

cv::Point2f Fisheye624::project(const cv::Point3f& p3D) {
    Eigen::Vector2d r = project(Eigen::Vector3d(p3D.x, p3D.y, p3D.z));
    return cv::Point2f(float(r[0]), float(r[1]));
}

Eigen::Vector2f Fisheye624::project(const Eigen::Vector3f& v3D) {
    Eigen::Vector2d r = project(v3D.cast<double>().eval());
    return r.cast<float>();
}

Eigen::Vector2f Fisheye624::projectMat(const cv::Point3f& p3D) {
    cv::Point2f p = project(p3D);
    return Eigen::Vector2f(p.x, p.y);
}

float Fisheye624::uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D) {
    return 1.f;
}

Eigen::Vector3f Fisheye624::unprojectEig(const cv::Point2f& p2D) {
    cv::Point3f p = unproject(p2D);
    return Eigen::Vector3f(p.x, p.y, p.z);
}

cv::Point3f Fisheye624::unproject(const cv::Point2f& p2D) {
    cv::Point3f ray;
    tryUnproject(p2D, ray);
    return ray;
}

Eigen::Matrix<double, 2, 3> Fisheye624::projectJac(const Eigen::Vector3d& v3D) {
    // central differences; step scaled to depth
    Eigen::Matrix<double, 2, 3> J;
    const double eps = 1e-6 * std::max(1.0, v3D.norm());
    for (int i = 0; i < 3; i++) {
        Eigen::Vector3d vp = v3D, vm = v3D;
        vp[i] += eps;
        vm[i] -= eps;
        J.col(i) = (project(vp) - project(vm)) / (2.0 * eps);
    }
    return J;
}

bool Fisheye624::ReconstructWithTwoViews(
    const std::vector<cv::KeyPoint>& vKeys1,
    const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int>& vMatches12,
    Sophus::SE3f& T21, std::vector<cv::Point3f>& vP3D,
    std::vector<bool>& vbTriangulated) {
    // Frame ingestion normally guarantees this. Refuse invalid direct callers
    // before handing non-finite coordinates to two-view reconstruction.
    cv::Point3f checked;
    for(const auto& key : vKeys1) if(!tryUnproject(key.pt, checked)) return false;
    for(const auto& key : vKeys2) if(!tryUnproject(key.pt, checked)) return false;
    // Focal/principal-point refinement can occur after initialization. A
    // later new-map bootstrap must not reuse an old pinhole K cache.
    if(tvr) { delete tvr; tvr=nullptr; }
    if (!tvr) {
        Eigen::Matrix3f K = this->toK_();
        tvr = new TwoViewReconstruction(K);
    }
    // undistort through the full model: pixel -> bearing -> ideal pinhole px
    auto undist = [&](const std::vector<cv::KeyPoint>& src) {
        std::vector<cv::KeyPoint> out = src;
        for (size_t i = 0; i < src.size(); i++) {
            cv::Point3f b = unproject(src[i].pt);
            out[i].pt.x = mvParameters[0] * b.x / b.z + mvParameters[2];
            out[i].pt.y = mvParameters[1] * b.y / b.z + mvParameters[3];
        }
        return out;
    };
    std::vector<cv::KeyPoint> vKeysUn1 = undist(vKeys1);
    std::vector<cv::KeyPoint> vKeysUn2 = undist(vKeys2);
    return tvr->Reconstruct(vKeysUn1, vKeysUn2, vMatches12, T21, vP3D,
                            vbTriangulated);
}

cv::Mat Fisheye624::toK() {
    cv::Mat K = (cv::Mat_<float>(3, 3) << mvParameters[0], 0.f, mvParameters[2],
                 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
    return K;
}

Eigen::Matrix3f Fisheye624::toK_() {
    Eigen::Matrix3f K;
    K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1],
        mvParameters[3], 0.f, 0.f, 1.f;
    return K;
}

bool Fisheye624::epipolarConstrain(GeometricCamera* pCamera2,
                                   const cv::KeyPoint& kp1,
                                   const cv::KeyPoint& kp2,
                                   const Eigen::Matrix3f& R12,
                                   const Eigen::Vector3f& t12,
                                   const float sigmaLevel, const float unc) {
    Eigen::Vector3f p3D;
    return this->TriangulateMatches(pCamera2, kp1, kp2, R12, t12, sigmaLevel,
                                    unc, p3D) > 0.0001f;
}

float Fisheye624::TriangulateMatches(GeometricCamera* pCamera2,
                                     const cv::KeyPoint& kp1,
                                     const cv::KeyPoint& kp2,
                                     const Eigen::Matrix3f& R12,
                                     const Eigen::Vector3f& t12,
                                     const float sigmaLevel, const float unc,
                                     Eigen::Vector3f& p3D) {
    Eigen::Vector3f r1 = this->unprojectEig(kp1.pt);
    Eigen::Vector3f r2 = pCamera2->unprojectEig(kp2.pt);
    if(!r1.allFinite() || !r2.allFinite()) return -6;

    Eigen::Vector3f r21 = R12 * r2;
    const float cosParallaxRays = r1.dot(r21) / (r1.norm() * r21.norm());
    if (cosParallaxRays > 0.9998) return -1;

    const cv::Point3f p11(r1[0], r1[1], r1[2]), p22(r2[0], r2[1], r2[2]);
    Eigen::Vector3f x3D;
    Eigen::Matrix<float, 3, 4> Tcw1;
    Tcw1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();
    Eigen::Matrix<float, 3, 4> Tcw2;
    Eigen::Matrix3f R21 = R12.transpose();
    Tcw2 << R21, -R21 * t12;
    Triangulate(p11, p22, Tcw1, Tcw2, x3D);
    if(!x3D.allFinite()) return -6;

    float z1 = x3D(2);
    if (z1 <= 0) return -2;
    float z2 = R21.row(2).dot(x3D) + Tcw2(2, 3);
    if (z2 <= 0) return -3;

    Eigen::Vector2f uv1 = this->project(x3D);
    float errX1 = uv1(0) - kp1.pt.x, errY1 = uv1(1) - kp1.pt.y;
    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel) return -4;

    Eigen::Vector3f x3D2 = R21 * x3D + Tcw2.col(3);
    Eigen::Vector2f uv2 = pCamera2->project(x3D2);
    float errX2 = uv2(0) - kp2.pt.x, errY2 = uv2(1) - kp2.pt.y;
    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * unc) return -5;

    p3D = x3D;
    return z1;
}

bool Fisheye624::matchAndtriangulate(const cv::KeyPoint& kp1,
                                     const cv::KeyPoint& kp2,
                                     GeometricCamera* pOther,
                                     Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                                     const float sigmaLevel1,
                                     const float sigmaLevel2,
                                     Eigen::Vector3f& x3Dtriangulated) {
    Eigen::Matrix<float, 3, 4> eigTcw1 = Tcw1.matrix3x4();
    Eigen::Matrix3f Rcw1 = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix3f Rwc1 = Rcw1.transpose();
    Eigen::Matrix<float, 3, 4> eigTcw2 = Tcw2.matrix3x4();
    Eigen::Matrix3f Rcw2 = eigTcw2.block<3, 3>(0, 0);
    Eigen::Matrix3f Rwc2 = Rcw2.transpose();

    cv::Point3f ray1c = this->unproject(kp1.pt);
    cv::Point3f ray2c = pOther->unproject(kp2.pt);
    Eigen::Vector3f r1(ray1c.x, ray1c.y, ray1c.z);
    Eigen::Vector3f r2(ray2c.x, ray2c.y, ray2c.z);
    if(!r1.allFinite() || !r2.allFinite()) return false;

    Eigen::Vector3f ray1 = Rwc1 * r1;
    Eigen::Vector3f ray2 = Rwc2 * r2;
    const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());
    if (cosParallaxRays > 0.9998) return false;

    Eigen::Vector3f x3D;
    Triangulate(ray1c, ray2c, eigTcw1, eigTcw2, x3D);
    if(!x3D.allFinite()) return false;

    float z1 = Rcw1.row(2).dot(x3D) + Tcw1.translation()(2);
    if (z1 <= 0) return false;
    float z2 = Rcw2.row(2).dot(x3D) + Tcw2.translation()(2);
    if (z2 <= 0) return false;

    Eigen::Vector3f x3D1 = Rcw1 * x3D + Tcw1.translation();
    Eigen::Vector2f uv1 = this->project(x3D1);
    float errX1 = uv1(0) - kp1.pt.x, errY1 = uv1(1) - kp1.pt.y;
    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel1) return false;

    Eigen::Vector3f x3D2 = Rcw2 * x3D + Tcw2.translation();
    Eigen::Vector2f uv2 = pOther->project(x3D2);
    float errX2 = uv2(0) - kp2.pt.x, errY2 = uv2(1) - kp2.pt.y;
    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaLevel2) return false;

    x3Dtriangulated = x3D;
    return true;
}

void Fisheye624::Triangulate(const cv::Point3f& p1, const cv::Point3f& p2,
                             const Eigen::Matrix<float, 3, 4>& Tcw1,
                             const Eigen::Matrix<float, 3, 4>& Tcw2,
                             Eigen::Vector3f& x3D) {
    // general-ray DLT (fork form): valid for unit bearings, no implicit z=1
    Eigen::Matrix<float, 4, 4> A;
    A.row(0) = p1.x * Tcw1.row(2) - p1.z * Tcw1.row(0);
    A.row(1) = p1.y * Tcw1.row(2) - p1.z * Tcw1.row(1);
    A.row(2) = p2.x * Tcw2.row(2) - p2.z * Tcw2.row(0);
    A.row(3) = p2.y * Tcw2.row(2) - p2.z * Tcw2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4f x3Dh = svd.matrixV().col(3);
    x3D = x3Dh.head(3) / x3Dh(3);
}

}  // namespace ORB_SLAM3
