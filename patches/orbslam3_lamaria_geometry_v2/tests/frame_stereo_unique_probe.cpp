#include "Frame.h"
#include "CameraModels/Fisheye624.h"
#include "ORBextractor.h"
#include "Rig.h"
#include <opencv2/core.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

using namespace ORB_SLAM3;

int main(int argc,char**argv) {
    if(argc!=3) return 2;
    cv::setNumThreads(1);
    Rig::PublishGlobals(false);
    cv::FileStorage fs(argv[1],cv::FileStorage::READ);
    const std::vector<std::string> keys={"fx","fy","cx","cy","k1","k2","k3","k4","k5","k6","p1","p2","s1","s2","s3","s4"};
    std::vector<float> p0,p1;
    for(const auto& key:keys){p0.push_back(float(fs["Camera1."+key]));p1.push_back(float(fs["Camera2."+key]));}
    Fisheye624 c0(p0),c1(p1);c0.mvLappingArea={0,640};c1.mvLappingArea={0,640};
    cv::Mat matrix;fs["Rig.T_c0_c1"]>>matrix;
    Eigen::Matrix3f R;Eigen::Vector3f t;
    for(int r=0;r<3;++r){for(int c=0;c<3;++c)R(r,c)=matrix.at<float>(r,c);t[r]=matrix.at<float>(r,3);}
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(R,Eigen::ComputeFullU|Eigen::ComputeFullV);
    R=svd.matrixU()*svd.matrixV().transpose();Sophus::SE3f T01(R,t);
    cv::Mat K=c0.toK(),distortion=cv::Mat::zeros(4,1,CV_32F),image=cv::Mat::zeros(480,640,CV_8UC1);
    ORBextractor ext0(1500,1.2,8,20,7),ext1(1500,1.2,8,20,7);
    std::ifstream input(argv[2]);std::string line;std::getline(input,line);
    int frames=0,points=0,reciprocalFailures=0;
    while(std::getline(input,line)) {
        const long long stamp=std::stoll(line.substr(0,line.find(',')));
        Frame f(image,image,double(stamp)/1e9,&ext0,&ext1,nullptr,K,distortion,t.norm()*p0[0],60,&c0,&c1,T01);
        int accepted=0;
        for(size_t left=0;left<f.mvLeftToRightMatch.size();++left) {
            const int right=f.mvLeftToRightMatch[left];
            if(right<0) continue;
            ++accepted;
            if(right>=int(f.mvRightToLeftMatch.size()) || f.mvRightToLeftMatch[right]!=int(left) || !(f.mvDepth[left]>0)) ++reciprocalFailures;
        }
        for(size_t right=0;right<f.mvRightToLeftMatch.size();++right) {
            const int left=f.mvRightToLeftMatch[right];
            if(left>=0 && (left>=int(f.mvLeftToRightMatch.size()) || f.mvLeftToRightMatch[left]!=int(right))) ++reciprocalFailures;
        }
        points+=accepted;++frames;
    }
    int emptyCases=0;
    for(auto counts:std::vector<std::pair<int,int>>{{0,0},{0,2},{2,0},{2,1}}) {
        Frame f;f.Nleft=counts.first;f.Nright=counts.second;f.monoLeft=0;f.monoRight=0;
        if(f.Nleft)f.mDescriptors=cv::Mat::zeros(f.Nleft,32,CV_8U);
        if(f.Nright)f.mDescriptorsRight=cv::Mat::zeros(f.Nright,32,CV_8U);
        f.ComputeStereoFishEyeMatches();
        if(f.mvLeftToRightMatch.size()!=size_t(f.Nleft) || f.mvRightToLeftMatch.size()!=size_t(f.Nright) || f.mvDepth.size()!=size_t(f.Nleft))return 20;
        for(int right:f.mvLeftToRightMatch)if(right!=-1)return 21;
        for(int left:f.mvRightToLeftMatch)if(left!=-1)return 22;
        for(float depth:f.mvDepth)if(depth!=-1)return 23;
        ++emptyCases;
    }
    std::cout<<"frames="<<frames<<" accepted_points="<<points<<" reciprocal_failures="<<reciprocalFailures<<" empty_cases="<<emptyCases<<std::endl;
    return reciprocalFailures?30:0;
}
