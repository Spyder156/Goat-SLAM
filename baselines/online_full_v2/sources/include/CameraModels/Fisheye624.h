/**
 * Fisheye624 (Aria RAD_TAN_THIN_PRISM_FISHEYE) camera model for ORB-SLAM3.
 *
 * Parameters (16): fx fy cx cy k0..k5 p0 p1 s0..s3
 *   - theta-polynomial fisheye: theta_d = theta * (1 + k0 t2 + ... + k5 t2^6)
 *   - tangential (Meta pairing: p0 multiplies (rho + 2a^2) in u -- OPPOSITE
 *     of the OpenCV pairing)
 *   - thin prism: u += s0 rho + s1 rho^2 ; v += s2 rho + s3 rho^2
 * Math ported from mecka-basalt basalt-headers fisheye624_camera.hpp,
 * verified against pycolmap to 6.4e-10 px.
 *
 * projectJac is central-difference numeric (correctness over speed; fine for
 * offline benchmark runs).
 */
#ifndef CAMERAMODELS_FISHEYE624_H
#define CAMERAMODELS_FISHEYE624_H

#include <assert.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>

#include "GeometricCamera.h"
#include "TwoViewReconstruction.h"

namespace ORB_SLAM3 {

class Fisheye624 : public GeometricCamera {

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& boost::serialization::base_object<GeometricCamera>(*this);
        if(version >= 1) {
            ar & mImageWidth & mImageHeight & mValidRadiusX & mValidRadiusY & mMaxSolidAngle;
        }
        if(version>=2) ar & mDomainCenterX & mDomainCenterY;
        else if(Archive::is_loading::value && mvParameters.size()==16) {
            mDomainCenterX=mvParameters[2];mDomainCenterY=mvParameters[3];
        }
    }

public:
    Fisheye624() : tvr(nullptr) {
        mvParameters.resize(16);
        mnId = nNextId++;
        mnType = CAM_FISHEYE;
        mvLappingArea.assign(2, 0);
    }
    Fisheye624(const std::vector<float> _vParameters)
        : GeometricCamera(_vParameters), tvr(nullptr) {
        assert(mvParameters.size() == 16);
        mnId = nNextId++;
        mnType = CAM_FISHEYE;
        mvLappingArea.assign(2, 0);
    }

    cv::Point2f project(const cv::Point3f& p3D);
    Eigen::Vector2d project(const Eigen::Vector3d& v3D);
    Eigen::Vector2f project(const Eigen::Vector3f& v3D);
    Eigen::Vector2f projectMat(const cv::Point3f& p3D);

    float uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D);

    Eigen::Vector3f unprojectEig(const cv::Point2f& p2D);
    cv::Point3f unproject(const cv::Point2f& p2D);

    // Explicit inversion contract. Invalid pixels and non-converged inverses
    // return false; no substitute ray is fabricated. Radius is optional (-1),
    // centred at the principal point, and may be elliptical after image resize.
    void setValidDomain(int width, int height, double radiusX, double radiusY,
                        double maxSolidAngle);
    bool tryUnproject(const cv::Point2f& pixel, cv::Point3f& unitRay) const;
    // Calibration validation separates numerical invertibility from the
    // angular admission policy. Sensor dimensions and physical mask remain.
    bool tryUnprojectGeometry(const cv::Point2f& pixel,cv::Point3f& ray) const;
    double maximumSolidAngle() const { return mMaxSolidAngle; }
    bool isInPhysicalMask(const cv::Point2f& pixel) const {
        if(!std::isfinite(pixel.x)||!std::isfinite(pixel.y))return false;
        if((mImageWidth>0&&(pixel.x<-.5||pixel.x>mImageWidth-.5))||
           (mImageHeight>0&&(pixel.y<-.5||pixel.y>mImageHeight-.5)))return false;
        const double x=double(pixel.x)-mDomainCenterX,y=double(pixel.y)-mDomainCenterY;
        return mValidRadiusX<=0 || x*x/(mValidRadiusX*mValidRadiusX)+y*y/(mValidRadiusY*mValidRadiusY)<=1.;
    }

    Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D);

    bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1,
                                 const std::vector<cv::KeyPoint>& vKeys2,
                                 const std::vector<int>& vMatches12,
                                 Sophus::SE3f& T21,
                                 std::vector<cv::Point3f>& vP3D,
                                 std::vector<bool>& vbTriangulated);

    cv::Mat toK();
    Eigen::Matrix3f toK_();

    bool epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1,
                           const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12,
                           const Eigen::Vector3f& t12, const float sigmaLevel,
                           const float unc);

    float TriangulateMatches(GeometricCamera* pCamera2,
                             const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                             const Eigen::Matrix3f& R12,
                             const Eigen::Vector3f& t12,
                             const float sigmaLevel, const float unc,
                             Eigen::Vector3f& p3D);

    bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                             GeometricCamera* pOther, Sophus::SE3f& Tcw1,
                             Sophus::SE3f& Tcw2, const float sigmaLevel1,
                             const float sigmaLevel2,
                             Eigen::Vector3f& x3Dtriangulated);

    // same role as KannalaBrandt8::mvLappingArea (feature-extraction column
    // bounds on the stereo/rig path); default full image
    std::vector<int> mvLappingArea;

private:
    bool tryUnprojectImpl(const cv::Point2f&,cv::Point3f&,bool) const;
    double mDomainCenterX=0.0,mDomainCenterY=0.0;
    int mImageWidth = 0;
    int mImageHeight = 0;
    double mValidRadiusX = -1.0;
    double mValidRadiusY = -1.0;
    double mMaxSolidAngle = 1.5707963267948966;
    // distort normalized coords (a,b) -> (u_n, v_n), full model, double
    void distort(double a, double b, double& un, double& vn) const;
    // Newton-invert the full distortion; returns normalized bearing (x,y,1)
    bool undistort(double mx, double my, double& x, double& y) const;

    void Triangulate(const cv::Point3f& p1, const cv::Point3f& p2,
                     const Eigen::Matrix<float, 3, 4>& Tcw1,
                     const Eigen::Matrix<float, 3, 4>& Tcw2,
                     Eigen::Vector3f& x3D);

    TwoViewReconstruction* tvr;
};

}  // namespace ORB_SLAM3

BOOST_CLASS_VERSION(ORB_SLAM3::Fisheye624, 2)

#endif
