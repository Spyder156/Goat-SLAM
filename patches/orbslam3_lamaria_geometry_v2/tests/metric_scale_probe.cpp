#include "MetricRigInitialization.h"
#include "CameraModels/Fisheye624.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <iomanip>
#include <cassert>

using namespace ORB_SLAM3;

int main(int argc,char**argv) {
    if(argc!=2) return 2;
    cv::FileStorage fs(argv[1],cv::FileStorage::READ);
    const std::vector<std::string> keys={"fx","fy","cx","cy","k1","k2","k3","k4","k5","k6","p1","p2","s1","s2","s3","s4"};
    std::vector<float> p0,p1;
    for(const auto& key:keys){p0.push_back(float(fs["Camera1."+key]));p1.push_back(float(fs["Camera2."+key]));}
    Fisheye624 c0(p0),c1(p1);
    cv::Mat T;fs["Rig.T_c0_c1"]>>T;
    Eigen::Matrix3d R01;Eigen::Vector3d t01;
    for(int r=0;r<3;++r){for(int c=0;c<3;++c)R01(r,c)=T.at<float>(r,c);t01[r]=T.at<float>(r,3);}
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R01,Eigen::ComputeFullU|Eigen::ComputeFullV);
    R01=svd.matrixU()*svd.matrixV().transpose();
    const Eigen::Matrix3d R10=R01.transpose();const Eigen::Vector3d t10=-R10*t01;
    const double trueScale=2.75;
    MetricRigScaleObservations exact;
    for(int i=0;i<12;++i){
        MetricRigScaleObservation o;o.landmarkId=i;o.camera0=&c0;o.camera1=&c1;o.R10=R10;o.t10Metres=t10;
        const double z=.7+.08*i;
        o.metricStereoPointCam0=Eigen::Vector3d(.015*(i-6),-.6*z,z);
        o.bootstrapPointCam0=o.metricStereoPointCam0/trueScale;
        o.pixelCam0=c0.project(o.metricStereoPointCam0);
        o.pixelCam1=c1.project((R10*o.metricStereoPointCam0+t10).eval());exact.push_back(o);
    }
    const auto good=EstimateMetricRigScale(exact);
    std::cout<<std::setprecision(12)<<"exact valid="<<good.valid<<" scale="<<good.metresPerBootstrapUnit<<" inliers="<<good.inlierLandmarks<<" logstd="<<good.logScaleStdDev<<std::endl;
    if(!good.valid||std::abs(good.metresPerBootstrapUnit-trueScale)>1e-6) return 10;
    auto contaminated=exact;for(size_t i=8;i<contaminated.size();++i)contaminated[i].bootstrapPointCam0/=3;
    const auto robust=EstimateMetricRigScale(contaminated);
    std::cout<<"outliers valid="<<robust.valid<<" scale="<<robust.metresPerBootstrapUnit<<" inliers="<<robust.inlierLandmarks<<std::endl;
    if(!robust.valid||std::abs(robust.metresPerBootstrapUnit-trueScale)>1e-6||robust.inlierLandmarks!=8) return 11;
    auto duplicate=exact;for(auto& o:duplicate)o.landmarkId=7;
    const auto dup=EstimateMetricRigScale(duplicate);
    std::cout<<"duplicate_identity valid="<<dup.valid<<" reason="<<dup.reason<<std::endl;
    if(dup.valid) return 12;
    auto distant=exact;for(auto& o:distant){o.t10Metres*=1e-7;o.pixelCam1=c1.project((o.R10*o.metricStereoPointCam0+o.t10Metres).eval());}
    const auto weak=EstimateMetricRigScale(distant);
    std::cout<<"unobservable valid="<<weak.valid<<" reason="<<weak.reason<<std::endl;
    if(weak.valid) return 13;
    auto ambiguous=exact;for(size_t i=6;i<ambiguous.size();++i)ambiguous[i].bootstrapPointCam0/=3;
    const auto amb=EstimateMetricRigScale(ambiguous);
    std::cout<<"split_consensus valid="<<amb.valid<<" reason="<<amb.reason<<std::endl;
    if(amb.valid) return 14;
    auto noisy=exact;for(size_t i=0;i<noisy.size();++i){noisy[i].pixelCam1+=Eigen::Vector2d(.25*std::sin(i),.25*std::cos(i));noisy[i].bootstrapPointCam0*=1+.025*std::sin(i*1.7);}
    const auto noise=EstimateMetricRigScale(noisy);
    std::cout<<"small_noise valid="<<noise.valid<<" scale="<<noise.metresPerBootstrapUnit<<" inliers="<<noise.inlierLandmarks<<std::endl;
    if(!noise.valid||std::abs(noise.metresPerBootstrapUnit/trueScale-1)>.05) return 15;
    std::cout<<"ALL_CHECKS_PASSED"<<std::endl;
}
