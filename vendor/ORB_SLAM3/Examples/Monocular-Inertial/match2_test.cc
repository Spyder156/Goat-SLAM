// Two-frame test of the FORK's OWN front-end: its ORBextractor + its
// ORBmatcher::SearchForInitialization, on real fork Frame objects with the
// Fisheye624 camera. Nothing OpenCV-generic in the loop.
//   match2_test voc.txt settings.yaml imgA imgB out_matches.csv
#include <opencv2/opencv.hpp>
#include "System.h"
#include "Frame.h"
#include "ORBmatcher.h"
#include "ORBextractor.h"
#include "Fisheye624.h"
#include <fstream>
using namespace ORB_SLAM3;

int main(int argc, char** argv){
    cv::FileStorage fs(argv[2], cv::FileStorage::READ);
    std::vector<float> P(16);
    const char* keys[16] = {"fx","fy","cx","cy","k1","k2","k3","k4","k5","k6",
                            "p1","p2","s1","s2","s3","s4"};
    for(int i=0;i<16;i++){ std::string k = std::string("Camera1.")+keys[i]; P[i]=(float)fs[k]; }
    GeometricCamera* cam = new Fisheye624(P);
    ORBVocabulary* voc = new ORBVocabulary();
    voc->loadFromTextFile(argv[1]);
    ORBextractor ext(1500, 1.2f, 8, 20, 7);   // Tracking's init extractor params
    cv::Mat dist = cv::Mat::zeros(4,1,CV_32F);
    cv::Mat a = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
    cv::Mat b = cv::imread(argv[4], cv::IMREAD_GRAYSCALE);
    IMU::Calib imuCalib;
    Frame f1(a, 0.0, &ext, voc, cam, dist, 0.f, 40.f, nullptr, imuCalib);
    Frame f2(b, 0.05, &ext, voc, cam, dist, 0.f, 40.f, nullptr, imuCalib);
    printf("extracted: F1=%d F2=%d\n", f1.N, f2.N);

    std::vector<cv::Point2f> prev(f1.mvKeysUn.size());
    for(size_t i=0;i<f1.mvKeysUn.size();i++) prev[i]=f1.mvKeysUn[i].pt;
    std::vector<int> m12;
    for(int win : {10, 30, 100}){
        ORBmatcher matcher(0.9f, true);
        std::vector<cv::Point2f> p = prev;
        int n = matcher.SearchForInitialization(f1, f2, p, m12, win);
        printf("SearchForInitialization window=%d -> %d matches\n", win, n);
    }
    std::ofstream out(argv[5]);
    for(size_t i=0;i<m12.size();i++)
        if(m12[i]>=0)
            out << f1.mvKeys[i].pt.x << "," << f1.mvKeys[i].pt.y << ","
                << f2.mvKeys[m12[i]].pt.x << "," << f2.mvKeys[m12[i]].pt.y << "\n";
    printf("csv written (window=100 matches)\n");
    return 0;
}
