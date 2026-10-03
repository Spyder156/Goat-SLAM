#ifndef ORB_SLAM3_G2O_RADIAL_CALIBRATION_H
#define ORB_SLAM3_G2O_RADIAL_CALIBRATION_H

#include "G2oTypes.h"
#include "CameraModels/Fisheye624.h"
#include <cmath>
#include <stdexcept>

namespace ORB_SLAM3 {

inline bool ValidateRadialCameraInverse(const Fisheye624& source,
                                       Fisheye624& proposal,
                                       int width,int height,
                                       size_t* validCount=nullptr,
                                       double* maximumError=nullptr,
                                       int step=16) {
    size_t count=0;double worst=0.0;bool valid=width>0 && height>0 && step>0;
    if(valid) for(int v=0;v<height;v+=step) for(int u=0;u<width;u+=step) {
        const cv::Point2f pixel(u,v);cv::Point3f original,updated;
        if(!source.tryUnproject(pixel,original)) continue;
        ++count;
        if(!proposal.tryUnproject(pixel,updated)) {valid=false;continue;}
        const double error=cv::norm(proposal.project(updated)-pixel);
        if(!std::isfinite(error)){valid=false;worst=std::numeric_limits<double>::infinity();}
        else {worst=std::max(worst,error);valid=valid && error<=0.01;}
    }
    if(validCount) *validCount=count;
    if(maximumError) *maximumError=worst;
    return valid && count>0;
}

// One bounded radial correction per physical camera. The g2o estimate is a
// dimensionless latent value; delta() is the physical correction to k0/k1.
// The camera is a PRIVATE value copy. No shared camera parameters or global
// camera IDs are touched by optimization, numerical checks or LM rollback.
class VertexRadialCalibration : public g2o::BaseVertex<2,Eigen::Vector2d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexRadialCalibration(const Fisheye624& source,
                            const Eigen::Vector2d& maximumDeltas)
        : mCamera(source), mBounds(maximumDeltas) {
        if(!mBounds.allFinite() || (mBounds.array()<=0).any())
            throw std::invalid_argument("Radial bounds must be finite and positive");
        for(int i=0;i<2;++i) mOriginal[i]=mCamera.getParameter(4+i);
        _estimate.setZero();
    }

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
    void setToOriginImpl() override { _estimate.setZero(); }
    void oplusImpl(const double* update) override {
        ORB_FINITE_GUARD(update,2);
        _estimate+=Eigen::Map<const Eigen::Vector2d>(update);
    }

    Eigen::Vector2d delta() const {
        return mBounds.array()*_estimate.array().tanh();
    }
    Eigen::Vector2d derivative() const {
        return mBounds.array()*(1.0-_estimate.array().tanh().square());
    }
    const Eigen::Vector2d& bounds() const { return mBounds; }
    void setDelta(const Eigen::Vector2d& value) {
        if(!value.allFinite() || (value.array().abs()>=mBounds.array()).any())
            throw std::invalid_argument("Radial delta must be strictly within bounds");
        _estimate=(value.array()/mBounds.array()).unaryExpr(
            [](double x){return std::atanh(x);}).matrix();
    }

    Fisheye624* camera() const {
        // BaseVertex::pop restores _estimate directly, bypassing oplusImpl.
        // Derive the private camera on every access so rollback is exact.
        const Eigen::Vector2d correction=delta();
        for(int i=0;i<2;++i)
            mCamera.setParameter(float(mOriginal[i]+correction[i]),4+i);
        return &mCamera;
    }

    // Derivative only: residual projection remains Fisheye624::project().
    // Differentiate its existing theta polynomial and full tangential/prism
    // chain; no replacement projection implementation is used by this edge.
    Eigen::Matrix2d projectRadialJac(const Eigen::Vector3d& point) const {
        Fisheye624* model=camera();
        const double radius=std::hypot(point.x(),point.y());
        if(radius<1e-12) return Eigen::Matrix2d::Zero();
        const double theta=std::atan2(radius,point.z()), theta2=theta*theta;
        double polynomial=1.0,power=theta2;
        for(int i=0;i<6;++i){polynomial+=model->getParameter(4+i)*power;power*=theta2;}
        const double x=theta*polynomial*point.x()/radius;
        const double y=theta*polynomial*point.y()/radius;
        const double rho=x*x+y*y;
        const double p0=model->getParameter(10),p1=model->getParameter(11);
        const double s0=model->getParameter(12),s1=model->getParameter(13);
        const double s2=model->getParameter(14),s3=model->getParameter(15);
        Eigen::Matrix2d prism;
        prism << 1+6*p0*x+2*p1*y+2*s0*x+4*s1*rho*x,
                 2*p0*y+2*p1*x+2*s0*y+4*s1*rho*y,
                 2*p1*x+2*p0*y+2*s2*x+4*s3*rho*x,
                 1+6*p1*y+2*p0*x+2*s2*y+4*s3*rho*y;
        prism.row(0)*=model->getParameter(0);
        prism.row(1)*=model->getParameter(1);
        Eigen::Matrix2d polynomialJac;
        polynomialJac.col(0)=theta*theta2/radius*point.head<2>();
        polynomialJac.col(1)=theta2*polynomialJac.col(0);
        return prism*polynomialJac*derivative().asDiagonal();
    }

private:
    mutable Fisheye624 mCamera;
    Eigen::Vector2d mOriginal,mBounds;
};

class EdgePriorRadial : public g2o::BaseUnaryEdge<2,Eigen::Vector2d,VertexRadialCalibration> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorRadial(){setMeasurement(Eigen::Vector2d::Zero());}
    bool read(std::istream&) override {return false;}
    bool write(std::ostream&) const override {return false;}
    void computeError() override {
        const auto* vertex=static_cast<const VertexRadialCalibration*>(_vertices[0]);
        _error=vertex->delta()-_measurement;
    }
    void linearizeOplus() override {
        const auto* vertex=static_cast<const VertexRadialCalibration*>(_vertices[0]);
        _jacobianOplusXi=vertex->derivative().asDiagonal();
    }
};

// Vertex order and Hessian layout: point(3), body pose(6), latent radial(2).
class EdgeMonoRadial : public g2o::BaseMultiEdge<2,Eigen::Vector2d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit EdgeMonoRadial(int camera=0):cam_idx(camera){resize(3);}
    bool read(std::istream&) override {return false;}
    bool write(std::ostream&) const override {return false;}
    void computeError() override {
        const auto* point=static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const auto* pose=static_cast<const VertexPose*>(_vertices[1]);
        const auto* radial=static_cast<const VertexRadialCalibration*>(_vertices[2]);
        const Eigen::Vector3d cameraPoint=pose->estimate().Rcw[cam_idx]*point->estimate()+pose->estimate().tcw[cam_idx];
        _error=_measurement-radial->camera()->project(cameraPoint);
    }
    bool isDepthPositive() const {
        const auto* point=static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const auto* pose=static_cast<const VertexPose*>(_vertices[1]);
        return pose->estimate().isDepthPositive(point->estimate(),cam_idx);
    }
    Eigen::Matrix<double,2,11> GetJacobian() const {
        const auto* point=static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const auto* pose=static_cast<const VertexPose*>(_vertices[1]);
        const auto* radial=static_cast<const VertexRadialCalibration*>(_vertices[2]);
        const auto& state=pose->estimate();
        const Eigen::Vector3d cameraPoint=state.Rcw[cam_idx]*point->estimate()+state.tcw[cam_idx];
        const Eigen::Vector3d bodyPoint=state.Rbc[cam_idx]*cameraPoint+state.tbc[cam_idx];
        const Eigen::Matrix<double,2,3> projection=radial->camera()->projectJac(cameraPoint);
        Eigen::Matrix<double,3,6> motion;
        const double x=bodyPoint.x(),y=bodyPoint.y(),z=bodyPoint.z();
        motion << 0,z,-y,1,0,0, -z,0,x,0,1,0, y,-x,0,0,0,1;
        Eigen::Matrix<double,2,11> matrix;
        matrix.block<2,3>(0,0)=-projection*state.Rcw[cam_idx];
        matrix.block<2,6>(0,3)=projection*state.Rcb[cam_idx]*motion;
        matrix.block<2,2>(0,9)=-radial->projectRadialJac(cameraPoint);
        return matrix;
    }
    void linearizeOplus() override {
        // g2o maps these buffers through JacobianWorkspace before calling this.
        // GetJacobian/GetHessian are independently safe before initialization.
        const Eigen::Matrix<double,2,11> matrix=GetJacobian();
        _jacobianOplus[0]=matrix.block<2,3>(0,0);
        _jacobianOplus[1]=matrix.block<2,6>(0,3);
        _jacobianOplus[2]=matrix.block<2,2>(0,9);
    }
    Eigen::Matrix<double,11,11> GetHessian(){
        const Eigen::Matrix<double,2,11> matrix=GetJacobian();
        return matrix.transpose()*information()*matrix;
    }
    const int cam_idx;
};

} // namespace ORB_SLAM3
#endif
