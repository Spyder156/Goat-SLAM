// Link this fixture to the selected production library. No inverse/filter math
// is copied here: native SDK supplies reference rays in a separate input file.
#include "CameraModels/Fisheye624.h"
#include "Frame.h"
#include "Settings.h"
#include "System.h"
#include "ORBextractor.h"
#include "Rig.h"
#include <opencv2/core.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <sys/stat.h>

using namespace ORB_SLAM3;
int failures=0;
void check(bool value,const char* name) {
    std::cout<<"CHECK "<<(value?"PASS ":"FAIL ")<<name<<std::endl;
    if(!value) ++failures;
}
bool finite(const cv::Point3f& ray) {
    return std::isfinite(ray.x)&&std::isfinite(ray.y)&&std::isfinite(ray.z);
}
void writeFeatures(const std::string& directory,const std::vector<cv::Point2f>& points) {
    mkdir(directory.c_str(),0755);
    FILE* file=fopen((directory+"/1000000000.kp").c_str(),"wb");
    if(!file)throw std::runtime_error("Cannot write fixture cache");
    int32_t n=points.size();fwrite(&n,4,1,file);
    for(const auto& point:points) {fwrite(&point.x,4,1,file);fwrite(&point.y,4,1,file);}
    std::vector<float> response(n,1.f);fwrite(response.data(),4,n,file);
    for(int i=0;i<n;++i){unsigned char row[32];for(int j=0;j<32;++j)row[j]=static_cast<unsigned char>(i*41+j);fwrite(row,1,32,file);}
    fclose(file);
}
int main(int argc,char** argv) {
    if(argc!=5)return 2;
    const std::string config=argv[1],references=argv[2],mode=argv[3],folder=argv[4];
    cv::setNumThreads(1); Rig::PublishGlobals(false);
    Settings settings(config,System::IMU_STEREO);
    Fisheye624* camera[2]={dynamic_cast<Fisheye624*>(settings.camera1()),dynamic_cast<Fisheye624*>(settings.camera2())};
    if(!camera[0]||!camera[1])return 3;
    int total=0,expectedValid=0,expectedInvalid=0,validLost=0,invalidAccepted=0,outerKept=0;
    double maxAngle=0,maxRoundtrip=0;
    std::ifstream input(references);int cam,valid;double u,v,x,y,z;
    while(input>>cam>>u>>v>>valid>>x>>y>>z) {
        ++total; const cv::Point2f pixel(u,v);const cv::Point3f ray=camera[cam]->unproject(pixel);
        if(!valid){++expectedInvalid;if(finite(ray))++invalidAccepted;continue;}
        ++expectedValid;if(!finite(ray)){++validLost;continue;}
        Eigen::Vector3d actual(ray.x,ray.y,ray.z),truth(x,y,z);actual.normalize();truth.normalize();
        maxAngle=std::max(maxAngle,std::atan2(actual.cross(truth).norm(),actual.dot(truth)));
        cv::Point2f projected=camera[cam]->project(ray);
        maxRoundtrip=std::max(maxRoundtrip,double(cv::norm(projected-pixel)));
        if(std::hypot(u-camera[cam]->getParameter(2),v-camera[cam]->getParameter(3))>=300)++outerKept;
    }
    std::cout<<"DOMAIN total="<<total<<" valid="<<expectedValid<<" invalid="<<expectedInvalid
             <<" valid_lost="<<validLost<<" invalid_accepted="<<invalidAccepted<<" outer_ge300_kept="<<outerKept
             <<" max_angle_rad="<<maxAngle<<" max_roundtrip_px="<<maxRoundtrip<<std::endl;
    check(total>1000&&expectedInvalid>100&&outerKept>100,"SDK reference covers centre, valid periphery, invalid corners");
    check(validLost==0,"all SDK-valid sampled pixels retained");
    check(invalidAccepted==0,"all SDK-invalid sampled pixels rejected");
    check(maxAngle<1e-6&&maxRoundtrip<1e-3,"valid bearing and projection agree with native SDK");
    const cv::Point2f bad0(46.6204567f,474.3709106f),bad1(29.9041233f,453.6503601f);
    check(!finite(camera[1]->unproject(bad0))&&!finite(camera[1]->unproject(bad1)),"both real122px fallback pixels rejected");
    std::vector<float> parameters;for(int k=0;k<16;++k)parameters.push_back(camera[1]->getParameter(k));
    Fisheye624 unconfigured(parameters);
    bool honestInverse=true;
    for(const auto& pixel:{bad0,bad1}) {
        const cv::Point3f ray=unconfigured.unproject(pixel);
        honestInverse&=!finite(ray)||cv::norm(unconfigured.project(ray)-pixel)<1e-3;
    }
    check(honestInverse,"inverse without metadata is accurate or explicitly invalid, never a fabricated fallback");
    check(!finite(camera[0]->unproject(cv::Point2f(std::numeric_limits<float>::quiet_NaN(),200))),"nonfinite pixel rejected");
    Eigen::Vector3f point;Sophus::SE3f pose0,pose1(Eigen::Matrix3f::Identity(),Eigen::Vector3f(.14,0,0));
    check(!(camera[1]->TriangulateMatches(camera[0],cv::KeyPoint(bad0,1),cv::KeyPoint(320,240,1),Eigen::Matrix3f::Identity(),Eigen::Vector3f(.14,0,0),1,1,point)>0),"invalid stereo point cannot triangulate");
    check(!camera[1]->matchAndtriangulate(cv::KeyPoint(bad0,1),cv::KeyPoint(320,240,1),camera[0],pose0,pose1,1,1,point),"invalid temporal point cannot triangulate");

    ORBextractor extractor0(400,1.2,8,20,7),extractor1(400,1.2,8,20,7);ORBVocabulary vocabulary;
    cv::Mat texture(480,640,CV_8UC1);cv::RNG random(12345);random.fill(texture,cv::RNG::UNIFORM,0,256);
    if(mode=="cached"||mode=="empty"||mode=="empty_left"||mode=="empty_right") {
        std::vector<cv::Point2f> points={cv::Point2f(100,240),bad0,cv::Point2f(320,240),bad1,cv::Point2f(620,240)};
        if(mode=="empty")points={bad0,bad1};
        const std::vector<cv::Point2f> invalid={bad0,bad1};
        writeFeatures(folder+"/left",mode=="empty_left"?invalid:points);
        writeFeatures(folder+"/right",mode=="empty_right"?invalid:points);
        setenv("KP_DIR",(folder+"/left").c_str(),1);setenv("KP_DIR1",(folder+"/right").c_str(),1);
    }
    camera[0]->mvLappingArea={200,500};camera[1]->mvLappingArea={200,500};
    cv::Mat K=camera[0]->toK(),dist=cv::Mat::zeros(4,1,CV_32F);
    Sophus::SE3f T01(Eigen::Matrix3f::Identity(),Eigen::Vector3f(.14,0,0));
    Frame frame(texture,texture,1.,&extractor0,&extractor1,&vocabulary,K,dist,33.6,60,camera[0],camera[1],T01);
    if(mode=="empty") {
        check(frame.N==0&&frame.Nleft==0&&frame.Nright==0,"all invalid cache entries produce coherent empty frame");
    } else {
        check(frame.N==frame.Nleft+frame.Nright&&frame.mDescriptors.rows==frame.N,"pooled descriptors and camera counts remain consistent");
        bool validKeys=true,gridSafe=true;
        for(int side=0;side<2;++side) {
            const auto& keys=side?frame.mvKeysRight:frame.mvKeys;
            for(const auto& key:keys)validKeys&=finite(camera[side]->unproject(key.pt));
            for(int gx=0;gx<FRAME_GRID_COLS;++gx)for(int gy=0;gy<FRAME_GRID_ROWS;++gy)
                for(size_t index:(side?frame.mGridRight[gx][gy]:frame.mGrid[gx][gy]))gridSafe&=index<keys.size();
        }
        check(validKeys,"all retained extracted features have valid native bearings");check(gridSafe,"no stale pre-filter grid indices");
        check(frame.mvpMapPoints.size()==static_cast<size_t>(frame.N)&&frame.mvbOutlier.size()==static_cast<size_t>(frame.N),"map/outlier arrays allocated after filtering");
        if(mode=="cached") {
            check(frame.Nleft==3&&frame.Nright==3&&frame.N==6,"invalid cached features removed in both cameras");
            check(frame.monoLeft==2&&frame.monoRight==2,"overlap partition boundaries adjusted");
            bool paired=frame.N==6;const int expected[3]={0,4,2};
            if(paired)for(int side=0;side<2;++side)for(int i=0;i<3;++i)for(int col=0;col<32;++col)
                paired&=frame.mDescriptors.at<unsigned char>(side*3+i,col)==static_cast<unsigned char>(expected[i]*41+col);
            check(paired,"keypoint and descriptor identities preserved after filter and pooling");
        } else if(mode=="empty_left"||mode=="empty_right") {
            const int expectedLeft=mode=="empty_left"?0:3;
            const int expectedRight=mode=="empty_right"?0:3;
            check(frame.Nleft==expectedLeft&&frame.Nright==expectedRight&&frame.N==3,
                  "one fully filtered camera preserves the other camera and pooled offsets");
            check(frame.mvLeftToRightMatch.size()==static_cast<size_t>(expectedLeft)&&
                  frame.mvRightToLeftMatch.size()==static_cast<size_t>(expectedRight),
                  "one empty camera has correctly sized stereo correspondence arrays");
        } else check(frame.N>100,"native ORB retains substantial valid features");
    }
    std::cout<<"RESULT failures="<<failures<<std::endl;return failures?1:0;
}
