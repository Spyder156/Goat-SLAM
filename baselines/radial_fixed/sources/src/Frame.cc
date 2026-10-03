/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Frame.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "Fisheye624.h"
#include "Rig.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"

#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>

namespace ORB_SLAM3
{

// ALIKED external keypoints are all octave 0, so a MapPoint's scale-invariance
// distance window derives from a single level and the ORB-derived
// [0.8*min, 1.2*max] frustum gate rejects points on modest distance change.
static const bool sbExternalKpFrustum = (getenv("KP_DIR") != nullptr);


// Diagnostic-only counters. No visibility decision or projection is changed.
namespace {
struct FrustumAudit {
    unsigned long frame = 0;
    double time = 0;
    bool initialized = false;
    int counts[2][5] = {};
    int bypassed_distance[2] = {};
};
void auditFrustum(const Frame& frame, int camera, int reason, bool distanceOutside=false) {
    static const bool enabled = std::getenv("LAMARIA_MATCH_DIAGNOSTICS") != nullptr;
    if (!enabled) return;
    static thread_local FrustumAudit a;
    if(a.initialized && a.frame != frame.mnId) {
        const int accepted = a.counts[0][4]+a.counts[1][4];
        if(a.frame % 20 == 0 || accepted < 30) {
            std::fprintf(stderr,
                "[FRUSTUM_DIAG] {\"frame\":%lu,\"t\":%.9f,\"cam0\":[%d,%d,%d,%d,%d],\"cam1\":[%d,%d,%d,%d,%d],\"distance_bypassed\":[%d,%d]}\n",
                a.frame,a.time,a.counts[0][0],a.counts[0][1],a.counts[0][2],a.counts[0][3],a.counts[0][4],
                a.counts[1][0],a.counts[1][1],a.counts[1][2],a.counts[1][3],a.counts[1][4],
                a.bypassed_distance[0],a.bypassed_distance[1]);
        }
        a = FrustumAudit();
    }
    a.initialized=true; a.frame=frame.mnId; a.time=frame.mTimeStamp;
    ++a.counts[camera][reason];
    if(distanceOutside) ++a.bypassed_distance[camera];
}
}

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

//For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

Frame::Frame(): mpcpi(NULL), mpImuPreintegrated(NULL), mpPrevFrame(NULL), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mbHasPose(false), mbHasVelocity(false)
{
    // A default/unused frame can be included in a calibration refresh batch.
    N = 0;
    Nleft = Nright = -1;
    mpCamera = mpCamera2 = nullptr;
    mnCloseMPs = 0;
#ifdef REGISTER_TIMES
    mTimeStereoMatch = 0;
    mTimeORB_Ext = 0;
#endif
}


//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpcpi(frame.mpcpi),mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mK_(Converter::toMatrix3f(frame.mK)), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mImuCalib(frame.mImuCalib), mnCloseMPs(frame.mnCloseMPs),
     mpImuPreintegrated(frame.mpImuPreintegrated), mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame), mImuBias(frame.mImuBias),
     mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors), mNameFile(frame.mNameFile), mnDataset(frame.mnDataset),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mpPrevFrame(frame.mpPrevFrame), mpLastKeyFrame(frame.mpLastKeyFrame),
     mbIsSet(frame.mbIsSet), mbImuPreintegrated(frame.mbImuPreintegrated), mpMutexImu(frame.mpMutexImu),
     mpCamera(frame.mpCamera), mpCamera2(frame.mpCamera2), Nleft(frame.Nleft), Nright(frame.Nright),
     monoLeft(frame.monoLeft), monoRight(frame.monoRight), mvLeftToRightMatch(frame.mvLeftToRightMatch),
     mvRightToLeftMatch(frame.mvRightToLeftMatch), mvStereo3Dpoints(frame.mvStereo3Dpoints),
     mTlr(frame.mTlr), mRlr(frame.mRlr), mtlr(frame.mtlr), mTrl(frame.mTrl),
     mTcw(frame.mTcw), mbHasPose(false), mbHasVelocity(false)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++){
            mGrid[i][j]=frame.mGrid[i][j];
            if(frame.Nleft > 0){
                mGridRight[i][j] = frame.mGridRight[i][j];
            }
        }

    if(frame.mbHasPose)
        SetPose(frame.GetPose());

    if(frame.HasVelocity())
    {
        SetVelocity(frame.GetVelocity());
    }

    mmProjectPoints = frame.mmProjectPoints;
    mmMatchedInImage = frame.mmMatchedInImage;

#ifdef REGISTER_TIMES
    mTimeStereoMatch = frame.mTimeStereoMatch;
    mTimeORB_Ext = frame.mTimeORB_Ext;
#endif
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL), mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
     mpCamera(pCamera) ,mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,0,0);
    thread threadRight(&Frame::ExtractORB,this,1,imRight,0,0);
    threadLeft.join();
    threadRight.join();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif

    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
    ComputeStereoMatches();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    mmProjectPoints.clear();
    mmMatchedInImage.clear();


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);



        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    if(pPrevF)
    {
        if(pPrevF->HasVelocity())
            SetVelocity(pPrevF->GetVelocity());
    }
    else
    {
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
     mpCamera(pCamera),mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    ExtractORB(0,imGray,0,0);
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();
    mmMatchedInImage.clear();

    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    if(pPrevF){
        if(pPrevF->HasVelocity())
            SetVelocity(pPrevF->GetVelocity());
    }
    else{
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(static_cast<Pinhole*>(pCamera)->toK()), mK_(static_cast<Pinhole*>(pCamera)->toK_()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL),mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera),
     mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    ExtractORB(0,imGray,0,1000);
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);
    mnCloseMPs = 0;

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();

    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,0);
        fy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,1);
        cx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,2);
        cy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }


    mb = mbf/fx;

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();

    if(pPrevF)
    {
        if(pPrevF->HasVelocity())
        {
            SetVelocity(pPrevF->GetVelocity());
        }
    }
    else
    {
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();
}


void Frame::AssignFeaturesToGrid()
{
    // Fill matrix with points
    const int nCells = FRAME_GRID_COLS*FRAME_GRID_ROWS;

    int nReserve = 0.5f*N/(nCells);

    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
            mGrid[i][j].reserve(nReserve);
            if(Nleft != -1){
                mGridRight[i][j].reserve(nReserve);
            }
        }



    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                                 : (i < Nleft) ? mvKeys[i]
                                                                 : mvKeysRight[i - Nleft];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY)){
            if(Nleft == -1 || i < Nleft)
                mGrid[nGridPosX][nGridPosY].push_back(i);
            else
                mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
        }
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1)
{
    // Filter before Nleft/Nright, stereo correspondences, pooled descriptors,
    // map associations and grids exist. Both extraction paths use the same
    // contract, preserving descriptor rows and the mono/overlap partition.
    auto filterDomain = [&]() {
        GeometricCamera* camera = flag == 0 || !mpCamera2 ? mpCamera : mpCamera2;
        Fisheye624* fisheye = dynamic_cast<Fisheye624*>(camera);
        if(!fisheye) return;
        std::vector<cv::KeyPoint>& keys = flag == 0 ? mvKeys : mvKeysRight;
        cv::Mat& descriptors = flag == 0 ? mDescriptors : mDescriptorsRight;
        int& mono = flag == 0 ? monoLeft : monoRight;
        const size_t input = keys.size();
        if(!input) return;
        std::vector<int> keep; keep.reserve(input);
        int keptMono = 0;
        for(size_t i = 0; i < input; ++i) {
            cv::Point3f ray;
            const cv::Point2f& pixel = keys[i].pt;
            if(pixel.x < -0.5f || pixel.x > im.cols - 0.5f ||
               pixel.y < -0.5f || pixel.y > im.rows - 0.5f ||
               !fisheye->tryUnproject(pixel, ray))
                continue;
            keep.push_back(static_cast<int>(i));
            if(static_cast<int>(i) < mono) ++keptMono;
        }
        if(keep.size() == input) return;
        std::vector<cv::KeyPoint> validKeys; validKeys.reserve(keep.size());
        cv::Mat validDescriptors(static_cast<int>(keep.size()), descriptors.cols, descriptors.type());
        for(size_t i = 0; i < keep.size(); ++i) {
            validKeys.push_back(keys[keep[i]]);
            descriptors.row(keep[i]).copyTo(validDescriptors.row(static_cast<int>(i)));
        }
        keys.swap(validKeys); descriptors = validDescriptors; mono = keptMono;
        // A single C stdio call avoids interleaved per-camera extraction lines.
        std::fprintf(stdout, "[FISHEYE-DOMAIN] t=%.9f cam=%d input=%zu rejected=%zu kept=%zu mono=%d\n",
                     mTimeStamp, flag, input, input-keep.size(), keep.size(), mono);
    };
    // External keypoints (descriptor transplant experiment): when KP_DIR is
    // set, load precomputed ALIKED keypoints + 256-bit binarized descriptors
    // for this frame's timestamp instead of running the ORB extractor. The
    // binarization keeps Hamming distances in the range the matcher's
    // TH_LOW/TH_HIGH thresholds expect, so all matching code runs unchanged.
    static const char* kpdir  = getenv("KP_DIR");
    static const char* kpdir1 = getenv("KP_DIR1");   // rig right lens
    const char* dir = (flag==0) ? kpdir : kpdir1;
    if(dir){
        static bool announced = false;
        char path[512];
        snprintf(path, sizeof(path), "%s/%lld.kp", dir,
                 (long long)llround(mTimeStamp*1e9));
        FILE* f = fopen(path, "rb");
        if(f){
            int32_t n = 0;
            size_t rd = fread(&n, 4, 1, f);
            std::vector<float> xy(2*n), resp(n);
            std::vector<uint8_t> d(32*n);
            rd += fread(xy.data(), 4, 2*n, f);
            rd += fread(resp.data(), 4, n, f);
            rd += fread(d.data(), 1, 32*n, f);
            fclose(f); (void)rd;
            std::vector<cv::KeyPoint>& K = (flag==0) ? mvKeys : mvKeysRight;
            cv::Mat& Dm = (flag==0) ? mDescriptors : mDescriptorsRight;
            K.clear(); K.reserve(n);
            for(int i=0;i<n;i++)
                K.emplace_back(xy[2*i], xy[2*i+1], 31.f, 0.f, resp[i], 0);
            Dm = cv::Mat(n, 32, CV_8U, d.data()).clone();
            // ORB's overlap contract: [0, mono) is outside the overlap;
            // [mono, N) is eligible for calibrated stereo triangulation.
            // Keep keypoint and descriptor order identical when partitioning.
            int mono = 0;
            std::vector<int> order;
            order.reserve(n);
            for(int i=0; i<n; ++i)
                if(x1 <= x0 || K[i].pt.x < x0 || K[i].pt.x > x1)
                    order.push_back(i);
            mono = static_cast<int>(order.size());
            for(int i=0; i<n; ++i)
                if(x1 > x0 && K[i].pt.x >= x0 && K[i].pt.x <= x1)
                    order.push_back(i);
            std::vector<cv::KeyPoint> orderedKeys;
            orderedKeys.reserve(n);
            cv::Mat orderedDescriptors(n, 32, CV_8U);
            for(int i=0; i<n; ++i) {
                orderedKeys.push_back(K[order[i]]);
                Dm.row(order[i]).copyTo(orderedDescriptors.row(i));
            }
            K.swap(orderedKeys);
            Dm = orderedDescriptors;
            if(flag==0) monoLeft = mono; else monoRight = mono;
            filterDomain();
            if(!announced){
                std::cout << "[KP-EXT] external keypoints active: " << kpdir
                          << " (first frame n=" << n << ")" << std::endl;
                announced = true;
            }
            return;
        }
        std::cerr << "[KP-EXT] MISSING " << path << " - falling back to ORB"
                  << std::endl;
    }
    vector<int> vLapping = {x0,x1};
    if(flag==0)
        monoLeft = (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors,vLapping);
    else
        monoRight = (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight,vLapping);
    filterDomain();
}

bool Frame::isSet() const {
    return mbIsSet;
}

void Frame::SetPose(const Sophus::SE3<float> &Tcw) {
    mTcw = Tcw;

    UpdatePoseMatrices();
    mbIsSet = true;
    mbHasPose = true;
}

void Frame::SetNewBias(const IMU::Bias &b)
{
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

void Frame::SetVelocity(Eigen::Vector3f Vwb)
{
    mVw = Vwb;
    mbHasVelocity = true;
}

Eigen::Vector3f Frame::GetVelocity() const
{
    return mVw;
}

void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb)
{
    mVw = Vwb;
    mbHasVelocity = true;

    Sophus::SE3f Twb(Rwb, twb);
    Sophus::SE3f Tbw = Twb.inverse();

    mTcw = mImuCalib.mTcb * Tbw;

    UpdatePoseMatrices();
    mbIsSet = true;
    mbHasPose = true;
}

void Frame::UpdatePoseMatrices()
{
    Sophus::SE3<float> Twc = mTcw.inverse();
    mRwc = Twc.rotationMatrix();
    mOw = Twc.translation();
    mRcw = mTcw.rotationMatrix();
    mtcw = mTcw.translation();
}

Eigen::Matrix<float,3,1> Frame::GetImuPosition() const {
    return mRwc * mImuCalib.mTcb.translation() + mOw;
}

Eigen::Matrix<float,3,3> Frame::GetImuRotation() {
    return mRwc * mImuCalib.mTcb.rotationMatrix();
}

Sophus::SE3<float> Frame::GetImuPose() {
    return mTcw.inverse() * mImuCalib.mTcb;
}

Sophus::SE3f Frame::GetRelativePoseTrl()
{
    return mTrl;
}

Sophus::SE3f Frame::GetRelativePoseTlr()
{
    return mTlr;
}

Eigen::Matrix3f Frame::GetRelativePoseTlr_rotation(){
    return mTlr.rotationMatrix();
}

Eigen::Vector3f Frame::GetRelativePoseTlr_translation() {
    return mTlr.translation();
}


bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    if(Nleft == -1){
        pMP->mbTrackInView = false;
        pMP->mTrackProjX = -1;
        pMP->mTrackProjY = -1;

        // 3D in absolute coordinates
        Eigen::Matrix<float,3,1> P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Eigen::Matrix<float,3,1> Pc = mRcw * P + mtcw;
        const float Pc_dist = Pc.norm();

        // Check positive depth
        const float &PcZ = Pc(2);
        const float invz = 1.0f/PcZ;
        if(PcZ<0.0f)
            { auditFrustum(*this, 0, 0); return false; }

        const Eigen::Vector2f uv = mpCamera->project(Pc);

        if(uv(0)<mnMinX || uv(0)>mnMaxX)
            { auditFrustum(*this, 0, 1); return false; }
        if(uv(1)<mnMinY || uv(1)>mnMaxY)
            { auditFrustum(*this, 0, 1); return false; }

        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const Eigen::Vector3f PO = P - mOw;
        const float dist = PO.norm();

        if(!sbExternalKpFrustum && (dist<minDistance || dist>maxDistance))
            { auditFrustum(*this, 0, 2); return false; }

        // Check viewing angle
        Eigen::Vector3f Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<viewingCosLimit)
            { auditFrustum(*this, 0, 3); return false; }

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjXR = uv(0) - mbf*invz;

        pMP->mTrackDepth = Pc_dist;

        pMP->mTrackProjY = uv(1);
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        auditFrustum(*this, 0, 4, sbExternalKpFrustum && (dist<minDistance || dist>maxDistance));
        return true;
    }
    else{
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
        pMP -> mnTrackScaleLevel = -1;
        pMP -> mnTrackScaleLevelR = -1;

        pMP->mbTrackInView = isInFrustumChecks(pMP,viewingCosLimit);
        pMP->mbTrackInViewR = isInFrustumChecks(pMP,viewingCosLimit,true);

        return pMP->mbTrackInView || pMP->mbTrackInViewR;
    }
}

bool Frame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const Eigen::Vector3f Pc = mRcw * P + mtcw;
    const float &PcX = Pc(0);
    const float &PcY= Pc(1);
    const float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float u_distort, v_distort;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    u_distort = x_distort * fx + cx;
    v_distort = y_distort * fy + cy;


    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

Eigen::Vector3f Frame::inRefCoordinates(Eigen::Vector3f pCw)
{
    return mRcw * pCw + mtcw;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    // ROOT NaN guard: a non-finite query (from a poisoned pose projecting to
    // NaN) would cast to a garbage grid index below -> out-of-bounds SEGV.
    // Guarding here covers ALL ~15 call sites at once. Empty result = no match.
    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(r))
        return vIndices;

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
    {
        return vIndices;
    }

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
    {
        return vIndices;
    }

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
    {
        return vIndices;
    }

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
    {
        return vIndices;
    }

    // External keypoints (ALIKED via KP_DIR) are all octave 0 while callers
    // pass ORB pyramid-level windows (e.g. PredictScale -> levels 1-2), which
    // would exclude every feature. Level filtering is meaningless there.
    static const bool bExternalKp = (getenv("KP_DIR") != nullptr);
    const bool bCheckLevels = !bExternalKp && ((minLevel>0) || (maxLevel>=0));

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<factorX && fabs(disty)<factorY)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);

    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat, static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);


    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }

}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Undistort corners
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

bool Frame::UnprojectStereo(const int &i, Eigen::Vector3f &x3D)
{
    const float z = mvDepth[i];
    if(z>0) {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        Eigen::Vector3f x3Dc(x, y, z);
        x3D = mRwc * x3Dc + mOw;
        return true;
    } else
        return false;
}

bool Frame::imuIsPreintegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);
    return mbImuPreintegrated;
}

void Frame::setIntegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);
    mbImuPreintegrated = true;
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, Sophus::SE3f& Tlr,Frame* pPrevF, const IMU::Calib &ImuCalib)
        :mpcpi(NULL), mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)),  mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
         mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(pCamera2),
         mbHasPose(false), mbHasVelocity(false)

{
    imgLeft = imLeft.clone();
    imgRight = imRight.clone();

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    auto lap = [](GeometricCamera* c, int cols) -> std::pair<int,int> {
        if(auto kb = dynamic_cast<KannalaBrandt8*>(c))
            return {kb->mvLappingArea[0], kb->mvLappingArea[1]};
        if(auto fe = dynamic_cast<Fisheye624*>(c)){
            int e = fe->mvLappingArea[1];
            return {fe->mvLappingArea[0], e < 0 ? cols : e};
        }
        return {0, cols};
    };
    auto lapL = lap(mpCamera,  imLeft.cols), lapR = lap(mpCamera2, imRight.cols);
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,lapL.first,lapL.second);
    thread threadRight(&Frame::ExtractORB,this,1,imRight,lapR.first,lapR.second);
    threadLeft.join();
    threadRight.join();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif

    Nleft = mvKeys.size();
    Nright = mvKeysRight.size();
    N = Nleft + Nright;

    if(N == 0)
        return;

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf / fx;

    // Sophus/Eigen
    mTlr = Tlr;
    mTrl = mTlr.inverse();
    mRlr = mTlr.rotationMatrix();
    mtlr = mTlr.translation();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
    ComputeStereoFishEyeMatches();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

    //Put all descriptors in the same matrix. Either side can be EMPTY on a
    //dark/blurred frame (zero ORB features) - vconcat asserts on that.
    if(mDescriptors.empty())
        mDescriptors = mDescriptorsRight.clone();
    else if(!mDescriptorsRight.empty())
        cv::vconcat(mDescriptors,mDescriptorsRight,mDescriptors);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(nullptr));
    mvbOutlier = vector<bool>(N,false);

    AssignFeaturesToGrid();

    mpMutexImu = new std::mutex();

    UndistortKeyPoints();

}

void Frame::ComputeStereoFishEyeMatches() {
    // Always publish well-formed empty correspondence/depth arrays, including
    // frames with no descriptors or too few right candidates for a ratio test.
    mvLeftToRightMatch = vector<int>(Nleft,-1);
    mvRightToLeftMatch = vector<int>(Nright,-1);
    mvDepth = vector<float>(Nleft,-1.0f);
    mvuRight = vector<float>(Nleft,-1);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(Nleft);
    mnCloseMPs = 0;

    if (Rig::NonOverlappingGlobal()) {
        Rig::StageOnce("frame", "non-overlapping rig: cross-camera stereo "
                                "matching SKIPPED; both cameras are monocular");
        return;
    }

    if(mDescriptors.empty() || mDescriptorsRight.empty() ||
       monoLeft < 0 || monoLeft >= mDescriptors.rows ||
       monoRight < 0 || monoRight > mDescriptorsRight.rows-2)
        return;

    // Only descriptors in the calibrated overlap region participate.
    cv::Mat stereoDescLeft = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
    cv::Mat stereoDescRight = mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);
    vector<vector<cv::DMatch>> matches;
    BFmatcher.knnMatch(stereoDescLeft,stereoDescRight,matches,2);

    // Best descriptor claims are considered first. An unsuccessful geometric
    // test does not reserve the right feature for that candidate.
    vector<cv::DMatch> candidates;
    candidates.reserve(matches.size());
    for(const auto& neighbors : matches)
        if(neighbors.size() >= 2 && neighbors[0].distance < neighbors[1].distance*0.7f)
            candidates.push_back(neighbors[0]);
    std::stable_sort(candidates.begin(),candidates.end(),
        [](const cv::DMatch& a,const cv::DMatch& b){ return a.distance < b.distance; });

    for(const cv::DMatch& match : candidates) {
        const int leftIndex = match.queryIdx + monoLeft;
        const int rightIndex = match.trainIdx + monoRight;
        if(mvRightToLeftMatch[rightIndex] != -1)
            continue;

        Eigen::Vector3f p3D;
        const float sigma1 = mvLevelSigma2[mvKeys[leftIndex].octave];
        const float sigma2 = mvLevelSigma2[mvKeysRight[rightIndex].octave];
        const float depth = mpCamera->TriangulateMatches(mpCamera2,
            mvKeys[leftIndex],mvKeysRight[rightIndex],mRlr,mtlr,sigma1,sigma2,p3D);
        if(depth > 0.0001f) {
            mvLeftToRightMatch[leftIndex] = rightIndex;
            mvRightToLeftMatch[rightIndex] = leftIndex;
            mvStereo3Dpoints[leftIndex] = p3D;
            mvDepth[leftIndex] = depth;
        }
    }
}

void Frame::RefreshStereoGeometryAfterCalibration()
{
    // Conventional stereo/RGB-D depths do not use this two-camera cache.
    if(Nleft < 0 || Nright < 0)
        return;

    const vector<int> oldLeftToRight = mvLeftToRightMatch;
    const vector<int> oldRightToLeft = mvRightToLeftMatch;
    mvLeftToRightMatch.assign(Nleft, -1);
    mvRightToLeftMatch.assign(Nright, -1);
    mvDepth.assign(Nleft, -1.0f);
    mvuRight.assign(Nleft, -1.0f);
    mvStereo3Dpoints.assign(Nleft, Eigen::Vector3f::Zero());
    mnCloseMPs = 0;

    // Empty frame construction can return before the rig transform is set.
    if(Nleft == 0 || Nright == 0 || !mpCamera || !mpCamera2)
        return;

    for(int left = 0; left < Nleft; ++left)
    {
        if(static_cast<size_t>(left) >= oldLeftToRight.size() ||
           static_cast<size_t>(left) >= mvKeys.size())
            continue;
        const int right = oldLeftToRight[left];
        if(right < 0 || right >= Nright ||
           static_cast<size_t>(right) >= oldRightToLeft.size() ||
           static_cast<size_t>(right) >= mvKeysRight.size() ||
           oldRightToLeft[right] != left)
            continue;

        const int levelLeft = mvKeys[left].octave;
        const int levelRight = mvKeysRight[right].octave;
        if(levelLeft < 0 || levelRight < 0 ||
           static_cast<size_t>(levelLeft) >= mvLevelSigma2.size() ||
           static_cast<size_t>(levelRight) >= mvLevelSigma2.size())
            continue;

        Eigen::Vector3f point;
        const float depth = mpCamera->TriangulateMatches(mpCamera2,
            mvKeys[left], mvKeysRight[right], mRlr, mtlr,
            mvLevelSigma2[levelLeft], mvLevelSigma2[levelRight], point);
        if(depth > 0.0001f && std::isfinite(depth) && point.allFinite())
        {
            mvLeftToRightMatch[left] = right;
            mvRightToLeftMatch[right] = left;
            mvDepth[left] = depth;
            mvStereo3Dpoints[left] = point;
            if(depth < mThDepth)
                ++mnCloseMPs;
        }
    }
}

bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight) {
    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    Eigen::Matrix3f mR;
    Eigen::Vector3f mt, twc;
    if(bRight){
        Eigen::Matrix3f Rrl = mTrl.rotationMatrix();
        Eigen::Vector3f trl = mTrl.translation();
        mR = Rrl * mRcw;
        mt = Rrl * mtcw + trl;
        twc = mRwc * mTlr.translation() + mOw;
    }
    else{
        mR = mRcw;
        mt = mtcw;
        twc = mOw;
    }

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mR * P + mt;
    const float Pc_dist = Pc.norm();
    const float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
        { auditFrustum(*this, bRight ? 1 : 0, 0); return false; }

    // Project in image and check it is not outside
    Eigen::Vector2f uv;
    if(bRight) uv = mpCamera2->project(Pc);
    else uv = mpCamera->project(Pc);

    if(uv(0)<mnMinX || uv(0)>mnMaxX)
        { auditFrustum(*this, bRight ? 1 : 0, 1); return false; }
    if(uv(1)<mnMinY || uv(1)>mnMaxY)
        { auditFrustum(*this, bRight ? 1 : 0, 1); return false; }

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const Eigen::Vector3f PO = P - twc;
    const float dist = PO.norm();

    if(!sbExternalKpFrustum && (dist<minDistance || dist>maxDistance))
        { auditFrustum(*this, bRight ? 1 : 0, 2); return false; }

    // Check viewing angle
    Eigen::Vector3f Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn) / dist;

    if(viewCos<viewingCosLimit)
        { auditFrustum(*this, bRight ? 1 : 0, 3); return false; }

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    if(bRight){
        pMP->mTrackProjXR = uv(0);
        pMP->mTrackProjYR = uv(1);
        pMP->mnTrackScaleLevelR= nPredictedLevel;
        pMP->mTrackViewCosR = viewCos;
        pMP->mTrackDepthR = Pc_dist;
    }
    else{
        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;
        pMP->mTrackDepth = Pc_dist;
    }

    auditFrustum(*this, bRight ? 1 : 0, 4, sbExternalKpFrustum && (dist<minDistance || dist>maxDistance));
        return true;
}

Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i){
    return mRwc * mvStereo3Dpoints[i] + mOw;
}

} //namespace ORB_SLAM
