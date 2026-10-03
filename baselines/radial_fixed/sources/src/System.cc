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



#include "System.h"
#include "MapLine.h"
#include "Converter.h"
#include <thread>
#include <stdexcept>
#include <cmath>
#include <map>
#include <set>
#include <algorithm>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int initFr, const string &strSequence):
    mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;
    else if(mSensor==IMU_RGBD)
        cout << "RGB-D-Inertial" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if(!node.empty() && node.isString() && node.string() == "1.0"){
        settings_ = new Settings(strSettingsFile,mSensor);

        mStrLoadAtlasFromFile = settings_->atlasLoadFile();
        mStrSaveAtlasToFile = settings_->atlasSaveFile();

        cout << (*settings_) << endl;
    }
    else{
        settings_ = nullptr;
        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if(!node.empty() && node.isString())
        {
            mStrLoadAtlasFromFile = (string)node;
        }

        node = fsSettings["System.SaveAtlasToFile"];
        if(!node.empty() && node.isString())
        {
            mStrSaveAtlasToFile = (string)node;
        }
    }

    node = fsSettings["loopClosing"];
    bool activeLC = true;
    if(!node.empty())
    {
        activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    mStrVocabularyFilePath = strVocFile;

    bool loadedAtlas = false;

    if(mStrLoadAtlasFromFile.empty())
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Atlas
        cout << "Initialization of Atlas from scratch " << endl;
        mpAtlas = new Atlas(0);
    }
    else
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        cout << "Load File" << endl;

        // Load the file with an earlier session
        //clock_t start = clock();
        cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if(!isRead)
        {
            cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
            exit(-1);
        }
        //mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);


        //cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

        loadedAtlas = true;

        mpAtlas->CreateNewMap();

        //clock_t timeElapsed = clock() - start;
        //unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
        //cout << "Binary file read in " << msElapsed << " ms" << endl;

        //usleep(10*1000*1000);
    }


    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR || mSensor==IMU_RGBD)
        mpAtlas->SetInertialSensor();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "Seq. Name: " << strSequence << endl;
    // ---- rig: load BEFORE the threads start, so every consumer sees the same
    // extrinsics and the log shows the rig state before any tracking output.
    mRig.LoadFromSettings(strSettingsFile);
    if(mRig.IsEnabled())
        Rig::Stage("system", "rig available to Tracking / LocalMapping / LoopClosing");

    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_, strSequence);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR,
                                     mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD, strSequence);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);
    mpLocalMapper->mInitFr = initFr;
    if(settings_)
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if(mpLocalMapper->mThFarPoints!=0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR, activeLC); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    //usleep(10*1000*1000);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    //if(false) // TODO
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}

Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if(settings_ && settings_->needToRectify()){
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if(settings_ && settings_->needToResize()){
        cv::resize(imLeft,imLeftToFeed,settings_->newImSize());
        cv::resize(imRight,imRightToFeed,settings_->newImSize());
    }
    else{
        imLeftToFeed = imLeft.clone();
        imRightToFeed = imRight.clone();
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    // std::cout << "start GrabImageStereo" << std::endl;
    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed,imRightToFeed,timestamp,filename);

    // std::cout << "out grabber" << std::endl;

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=RGBD  && mSensor!=IMU_RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;

        cv::resize(depthmap,imDepthToFeed,settings_->newImSize());
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_RGBD)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed,imDepthToFeed,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Sophus::SE3f System::TrackMonoRig(const cv::Mat &im0, const cv::Mat &im1, const double &timestamp,
                                  const vector<IMU::Point>& vImuMeas, string filename)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbShutDown) return Sophus::SE3f();
    }
    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "[Debug] TrackMonoRig FATAL: sensor is not Monocular/Monocular-Inertial. "
                "Experiment B runs the RIG on the MONO path on purpose -- see "
                "SLAM/patches/orbslam3_rigB/README.md" << endl;
        exit(-1);
    }
    if(!mRig.IsEnabled())
    {
        cerr << "[Debug] TrackMonoRig FATAL: Rig.enabled is 0 but a 2-camera "
                "frame was fed" << endl;
        exit(-1);
    }

    cv::Mat i0 = im0.clone(), i1 = im1.clone();
    if(settings_ && settings_->needToResize()){
        cv::resize(i0,i0,settings_->newImSize());
        cv::resize(i1,i1,settings_->newImSize());
    }

    // mode / reset handling, identical to TrackMonocular
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();
            while(!mpLocalMapper->isStopped()) usleep(1000);
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset){ mpTracker->Reset(); mbReset=false; mbResetActiveMap=false; }
        else if(mbResetActiveMap){ mpTracker->ResetActiveMap(); mbResetActiveMap=false; }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i=0;i<vImuMeas.size();i++) mpTracker->GrabImuData(vImuMeas[i]);

    Sophus::SE3f Tcw = mpTracker->GrabImageMonoRig(i0,i1,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{

    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbShutDown)
            return Sophus::SE3f();
    }

    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}



void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    cout << "Shutdown" << endl;

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer) mpViewer->RequestFinish();
    // Finish flags alone are not a worker-lifetime barrier. Joining also
    // handles a shutdown issued before a worker has entered Run().
    if(mptLocalMapping && mptLocalMapping->joinable()) mptLocalMapping->join();
    if(mptLoopClosing && mptLoopClosing->joinable()) mptLoopClosing->join();
    mpLoopCloser->WaitForGlobalBundleAdjustmentWorkers();
    if(mpViewer && mptViewer && mptViewer->joinable()) mptViewer->join();

    if(!mStrSaveAtlasToFile.empty())
    {
        Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
        SaveAtlas(FileType::BINARY_FILE);
    }

    /*if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif


}

bool System::isShutDown() {
    unique_lock<mutex> lock(mMutexReset);
    return mbShutDown;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
          << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

    }

    f.close();
}

void System::OpenKeypointDump(const string &filename)
{
    mKpDump.open(filename);
    if(!mKpDump.is_open()){
        cerr << "[Debug] OpenKeypointDump FATAL: cannot write " << filename << endl;
        exit(-1);
    }
    mKpDump << "t,cam,id,u,v,tracked" << endl;
    cout << "[Debug] keypoint dump -> " << filename << endl;
}

void System::DumpFrameKeypoints()
{
    if(!mKpDump.is_open()) return;
    Frame &F = mpTracker->mCurrentFrame;
    // After a map reset mCurrentFrame is a default Frame(): N holds stale
    // garbage while the keypoint vectors are empty. Indexing them segfaulted
    // (gdb: DumpFrameKeypoints, first frame after CreateMapInAtlas).
    const int nAvail = (F.Nleft == -1) ? (int)F.mvKeysUn.size()
                                       : (int)(F.mvKeys.size() + F.mvKeysRight.size());
    if(F.N <= 0 || F.N > nAvail) return;
    const int nL = (F.Nleft == -1) ? (int)F.mvKeys.size() : F.Nleft;
    for(int i = 0; i < F.N; i++)
    {
        // camera index is IMPLICIT in ORB-SLAM3: idx < Nleft is the front
        // camera, the rest belong to the rear one.
        const int cam = (F.Nleft == -1 || i < nL) ? 0 : 1;
        const cv::KeyPoint &kp = (F.Nleft == -1) ? F.mvKeysUn[i]
                                : (i < nL ? F.mvKeys[i] : F.mvKeysRight[i - nL]);
        long id = -1; int tracked = 0;
        if(i < (int)F.mvpMapPoints.size() && F.mvpMapPoints[i]){
            id = (long)F.mvpMapPoints[i]->mnId;
            tracked = (i < (int)F.mvbOutlier.size() && F.mvbOutlier[i]) ? 0 : 1;
        }
        mKpDump << std::fixed << std::setprecision(9) << F.mTimeStamp << ","
                << cam << "," << id << ","
                << std::setprecision(2) << kp.pt.x << "," << kp.pt.y << ","
                << tracked << "\n";
    }
}

void System::CloseKeypointDump()
{
    if(mKpDump.is_open()){ mKpDump.flush(); mKpDump.close(); }
}

// Additive diagnostics/export only. No estimate is modified or stitched to GT.
void System::OpenFramePoseDump(const string &filename)
{
    mFramePoseDump.open(filename);
    if(!mFramePoseDump) throw std::runtime_error("Cannot open frame pose dump: " + filename);
    mFramePoseDump << "input_t_s,frame_id,map_id,map_init_kf_id,world_version,state,coasting,inliers,imu_initialized,pose_available,pose_kind,tx,ty,tz,qx,qy,qz,qw\n";
}

void System::DumpFramePose(const double inputTime)
{
    if(!mFramePoseDump.is_open()) return;
    Frame &frame = mpTracker->mCurrentFrame;
    Map *map = mpAtlas->GetCurrentMap();
    // A reset may have replaced the current frame with a default Frame whose
    // timestamp/ID are uninitialized. Read them only after isSet() succeeds.
    const bool hasPose = frame.isSet() && std::abs(frame.mTimeStamp-inputTime)<1e-6;
    const bool inertial = mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD;
    mFramePoseDump << fixed << setprecision(9) << inputTime << ",";
    if(hasPose) mFramePoseDump << frame.mnId; else mFramePoseDump << -1;
    mFramePoseDump << "," << (map ? static_cast<long long>(map->GetId()) : -1)
                  << "," << (map ? static_cast<long long>(map->GetInitKFid()) : -1)
                  << "," << (map ? map->GetWorldFrameVersion() : -1)
                  << "," << static_cast<int>(mpTracker->mState)
                  << "," << mpTracker->ExportIsCoasting()
                  << "," << (hasPose ? mpTracker->ExportInliers() : -1)
                  << "," << (map && map->isImuInitialized())
                  << "," << hasPose << "," << (inertial ? "body_to_current_map" : "cam0_to_current_map");
    if(hasPose) {
        const Sophus::SE3f pose = inertial ? frame.GetImuPose() : frame.GetPose().inverse();
        const Eigen::Vector3f p = pose.translation();
        const Eigen::Quaternionf q = pose.unit_quaternion();
        mFramePoseDump << "," << p.x() << "," << p.y() << "," << p.z()
                      << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w();
    } else mFramePoseDump << ",nan,nan,nan,nan,nan,nan,nan";
    // Small diagnostic stream: preserve its final row even if a later frame fails.
    mFramePoseDump << endl;
}

void System::CloseFramePoseDump()
{
    if(mFramePoseDump.is_open()) { mFramePoseDump.flush(); mFramePoseDump.close(); }
    mpTracker->ExportCloseConfidenceDump();
}

void System::SaveAtlasSegments(const string &prefix)
{
    // This fork's Shutdown currently requests finish but does not wait. The
    // atlas snapshot must precede no outstanding backend mutation.
    if(!isShutDown()) throw std::runtime_error("SaveAtlasSegments requires Shutdown first");
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()) usleep(1000);
    mpLoopCloser->WaitForGlobalBundleAdjustmentWorkers();
    const bool inertial = mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD;
    ofstream manifest(prefix+"_maps.csv"), trajectory(prefix+"_trajectory.csv"), points(prefix+"_points.csv"),
             keyframes(prefix+"_keyframes.csv"), history(prefix+"_history.csv"), contract(prefix+"_contract.txt");
    if(!manifest || !trajectory || !points || !keyframes || !history || !contract)
        throw std::runtime_error("Cannot open atlas export: " + prefix);
    manifest << "map_id,map_init_kf_id,coordinate_frame,pose_object,keyframes,map_points,exported_poses,first_t_s,last_t_s,imu_initialized,global_anchor_available,map_T_export_tx,map_T_export_ty,map_T_export_tz,map_T_export_qx,map_T_export_qy,map_T_export_qz,map_T_export_qw\n";
    trajectory << "t_s,map_id,map_init_kf_id,coordinate_frame,history_index,reference_kf_id,tx,ty,tz,qx,qy,qz,qw\n";
    keyframes << "t_s,map_id,map_init_kf_id,coordinate_frame,keyframe_id,tx,ty,tz,qx,qy,qz,qw\n";
    points << "t_s,map_id,map_init_kf_id,coordinate_frame,point_id,x,y,z,cam\n";
    history << "history_index,t_s,status,map_id,reference_kf_id\n";
    contract << "Each coordinate_frame is independent. No cross-map transform or ground-truth alignment is supplied.\n"
             << "Poses are object-to-export-frame, quaternion xyzw, timestamps seconds, lengths in estimator map units.\n"
             << "For IMU_STEREO with metric initialization those map units are metres. IMU initialization status is listed per map.\n"
             << "The export origin is that map's first surviving keyframe body (inertial) or cam0 (visual).\n"
             << "map_T_export is the INTERNAL map-to-export-origin relation (export coordinates to internal map); it is NOT a shared physical world anchor.\n"
             << "Trajectory includes stored history not marked lost; RECENTLY_LOST/coasting is NOT excluded by that legacy flag. Join online diagnostics by timestamp to distinguish it.\n"
             << "Online snapshots are not retrospectively optimized; map world_version may change during inertial initialization.\n"
             << "Cleared map history and frames never given a pose remain unavailable. Missing history is not filled or interpolated.\n"
             << "Do not draw connecting edges between maps, reset epochs, missing frames, or unknown transforms.\n";
    auto writePose = [](ofstream &stream, const Sophus::SE3f &pose) {
        const Eigen::Vector3f p=pose.translation(); const Eigen::Quaternionf q=pose.unit_quaternion();
        stream << setprecision(9) << "," << p.x() << "," << p.y() << "," << p.z()
               << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w();
    };
    for(auto stream : {&manifest,&trajectory,&points,&keyframes,&history}) *stream << fixed;
    struct Counters { size_t poses=0, points=0; double first=0,last=0; };
    std::map<Map*,Counters> counts;
    const vector<Map*> maps=mpAtlas->GetAllMaps();
    // Final, quiescent graph audit. Camera-presence flags alone cannot expose a
    // missing observation in one particular keyframe. Check both directions.
    ofstream observationAudit(prefix+"_observation_audit.csv");
    ofstream observationErrors(prefix+"_observation_errors.csv");
    if(!observationAudit || !observationErrors)
        throw std::runtime_error("Cannot open observation audit: " + prefix);
    observationAudit << "map_id,metric,count\n";
    observationErrors << "map_id,kind,keyframe_id,point_id,index,recorded_index\n";
    for(Map *map : maps) {
        if(!map || map->IsBad()) continue;
        vector<KeyFrame*> kfs=map->GetAllKeyFrames();
        kfs.erase(std::remove_if(kfs.begin(),kfs.end(),[](KeyFrame* k){return !k || k->isBad();}), kfs.end());
        if(kfs.empty()) continue;
        std::map<std::string,size_t> audit;
        std::map<KeyFrame*,std::vector<MapPoint*>> auditSlots;
        for(const char* metric : {"cam0_slots", "cam1_slots", "cam0_observations", "cam1_observations",
                                  "missing_backlinks", "wrong_reverse_slots", "invalid_observation_indices",
                                  "bad_observation_keyframes", "observation_count_mismatches", "bad_point_slots"})
            audit[metric]=0;
        auto violation = [&](const char* kind, KeyFrame* kf, MapPoint* point, int index, int recorded) {
            ++audit[kind];
            observationErrors << map->GetId() << "," << kind << ","
                              << (kf?static_cast<long long>(kf->mnId):-1) << "," << point->mnId
                              << "," << index << "," << recorded << '\n';
        };
        for(KeyFrame* kf : kfs) {
            const auto& slots=auditSlots.emplace(kf,kf->GetMapPointMatches()).first->second;
            for(size_t i=0; i<slots.size(); ++i) {
                MapPoint* point=slots[i];
                if(!point) continue;
                if(point->isBad()) { ++audit["bad_point_slots"]; continue; }
                const bool right=kf->NLeft!=-1 && static_cast<int>(i)>=kf->NLeft;
                ++audit[right?"cam1_slots":"cam0_slots"];
                const auto indices=point->GetIndexInKeyFrame(kf);
                const int registered=right?std::get<1>(indices):std::get<0>(indices);
                if(registered!=static_cast<int>(i))
                    violation("missing_backlinks",kf,point,static_cast<int>(i),registered);
            }
        }
        for(MapPoint* point : map->GetAllMapPoints()) {
            if(!point || point->isBad()) continue;
            int weightedObservations=0;
            for(const auto& observation : point->GetObservations()) {
                KeyFrame* kf=observation.first;
                if(!kf || kf->isBad()) {
                    violation("bad_observation_keyframes",kf,point,-1,-1);
                    continue;
                }
                auto cached=auditSlots.find(kf);
                if(cached==auditSlots.end()) cached=auditSlots.emplace(kf,kf->GetMapPointMatches()).first;
                const auto& slots=cached->second;
                const int indices[2]={std::get<0>(observation.second),std::get<1>(observation.second)};
                for(int camera=0; camera<2; ++camera) {
                    const int index=indices[camera];
                    if(index<0) continue;
                    ++audit[camera?"cam1_observations":"cam0_observations"];
                    const bool valid=index<static_cast<int>(slots.size()) &&
                        (camera ? kf->NLeft!=-1 && index>=kf->NLeft : kf->NLeft==-1 || index<kf->NLeft);
                    if(!valid) { violation("invalid_observation_indices",kf,point,index,-1); continue; }
                    weightedObservations += !kf->mpCamera2 && index<static_cast<int>(kf->mvuRight.size()) &&
                                            kf->mvuRight[index]>=0 ? 2 : 1;
                    if(slots[index]!=point)
                        violation("wrong_reverse_slots",kf,point,index,-1);
                }
            }
            if(point->Observations()!=weightedObservations)
                violation("observation_count_mismatches",nullptr,point,point->Observations(),weightedObservations);
        }
        for(const auto& metric : audit)
            observationAudit << map->GetId() << "," << metric.first << "," << metric.second << '\n';
        sort(kfs.begin(),kfs.end(),KeyFrame::lId);
        const Sophus::SE3f TmapExport=inertial ? kfs.front()->GetImuPose() : kfs.front()->GetPoseInverse();
        const Sophus::SE3f TexMap=TmapExport.inverse();
        const string frameName="map_"+to_string(map->GetId())+"_init_"+to_string(map->GetInitKFid())+(inertial?"_body0":"_cam0");
        auto prefixRow = [&](ofstream &stream,double t) {
            stream << setprecision(9) << t << "," << map->GetId() << "," << map->GetInitKFid() << "," << frameName;
        };
        for(KeyFrame *kf:kfs) {
            prefixRow(keyframes,kf->mTimeStamp); keyframes << "," << kf->mnId;
            writePose(keyframes,TexMap*(inertial?kf->GetImuPose():kf->GetPoseInverse())); keyframes << '\n';
        }
        for(MapPoint *point:map->GetAllMapPoints()) {
            if(!point || point->isBad()) continue;
            KeyFrame *ref=point->GetReferenceKeyFrame();
            const Eigen::Vector3f p=TexMap*point->GetWorldPos();
            bool left=false,right=false;
            for(const auto &ob:point->GetObservations()) { left |= std::get<0>(ob.second)>=0; right |= std::get<1>(ob.second)>=0; }
            prefixRow(points,ref?ref->mTimeStamp:0.); points << "," << point->mnId << "," << p.x() << "," << p.y() << "," << p.z()
                                                       << "," << (left&&right?2:(right?1:0)) << '\n';
            ++counts[map].points;
        }
        auto ref=mpTracker->mlpReferences.begin(); auto stamp=mpTracker->mlFrameTimes.begin(); auto lost=mpTracker->mlbLost.begin();
        size_t index=0;
        for(auto relative=mpTracker->mlRelativeFramePoses.begin(); relative!=mpTracker->mlRelativeFramePoses.end(); ++relative,++ref,++stamp,++lost,++index) {
            if(ref==mpTracker->mlpReferences.end() || stamp==mpTracker->mlFrameTimes.end() || lost==mpTracker->mlbLost.end())
                throw std::runtime_error("Inconsistent tracking history lengths");
            if(*lost) continue;
            KeyFrame *kf=*ref; Sophus::SE3f TrefAncestor; std::set<KeyFrame*> visited;
            while(kf && kf->isBad()) {
                if(!visited.insert(kf).second) { kf=nullptr; break; }
                TrefAncestor=TrefAncestor*kf->mTcp; kf=kf->GetParent();
            }
            if(!kf || kf->GetMap()!=map) continue;
            // Exact legacy export composition, using this map's own origin.
            const Sophus::SE3f TcamExport=(*relative)*TrefAncestor*kf->GetPose()*TmapExport;
            const Sophus::SE3f pose=inertial ? (kf->mImuCalib.mTbc*TcamExport).inverse() : TcamExport.inverse();
            prefixRow(trajectory,*stamp); trajectory << "," << index << "," << kf->mnId; writePose(trajectory,pose); trajectory << '\n';
            if(counts[map].poses++==0) counts[map].first=*stamp; counts[map].last=*stamp;
        }
        manifest << map->GetId() << "," << map->GetInitKFid() << "," << frameName << "," << (inertial?"body":"cam0")
                 << "," << kfs.size() << "," << counts[map].points << "," << counts[map].poses << "," << setprecision(9)
                 << counts[map].first << "," << counts[map].last << "," << map->isImuInitialized() << ",0";
        writePose(manifest,TmapExport); manifest << '\n';
    }
    // Audit every retained history slot once, including exclusions. This is
    // separate from the native-input trace because history omits some frames.
    auto ref=mpTracker->mlpReferences.begin(); auto stamp=mpTracker->mlFrameTimes.begin(); auto lost=mpTracker->mlbLost.begin();
    size_t index=0;
    for(auto pose=mpTracker->mlRelativeFramePoses.begin(); pose!=mpTracker->mlRelativeFramePoses.end(); ++pose,++ref,++stamp,++lost,++index) {
        if(ref==mpTracker->mlpReferences.end() || stamp==mpTracker->mlFrameTimes.end() || lost==mpTracker->mlbLost.end())
            throw std::runtime_error("Inconsistent tracking history lengths");
        KeyFrame *kf=*ref; string status="exported"; std::set<KeyFrame*> visited;
        if(*lost) { status="marked_lost"; kf=nullptr; }
        else { while(kf && kf->isBad()) { if(!visited.insert(kf).second) { kf=nullptr; break; } kf=kf->GetParent(); }
            if(!kf) status="missing_reference_or_cycle";
            else if(!kf->GetMap() || !counts.count(kf->GetMap())) status="cleared_or_unretained_map";
        }
        history << index << "," << setprecision(9) << *stamp << "," << status << ","
                << (kf && kf->GetMap()?static_cast<long long>(kf->GetMap()->GetId()):-1)
                << "," << (kf?static_cast<long long>(kf->mnId):-1) << '\n';
    }
    cout << "[Atlas export] Saved " << counts.size() << " independent map segments to " << prefix << "_*" << endl;
}

void System::SaveMapPoints(const string &filename)
{
    cout << endl << "Saving map points to " << filename << " ..." << endl;
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    // largest map = the one that actually tracked
    Map* pBiggerMap = nullptr;
    size_t numMax = 0;
    for(Map* pMap : vpMaps)
        if(pMap->GetAllKeyFrames().size() > numMax){ numMax = pMap->GetAllKeyFrames().size(); pBiggerMap = pMap; }
    if(!pBiggerMap){ cout << "  no map to save" << endl; return; }

    // SAME REFERENCE FRAME AS SaveTrajectoryEuRoC.
    // That function re-expresses the trajectory relative to the FIRST keyframe
    // ("b0 is the new world reference"), because after a loop closure the first
    // keyframe is no longer at the origin. Dumping landmarks in the raw world
    // frame instead left the map and the trajectory in two different frames --
    // the map looked plausible on its own and the camera never passed through
    // it. Anything written next to that trajectory must use the same frame.
    vector<KeyFrame*> vpKFsOrd = pBiggerMap->GetAllKeyFrames();
    sort(vpKFsOrd.begin(), vpKFsOrd.end(), KeyFrame::lId);
    Sophus::SE3f Twb0;
    if(!vpKFsOrd.empty())
        Twb0 = (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
                 ? vpKFsOrd[0]->GetImuPose() : vpKFsOrd[0]->GetPoseInverse();
    const Sophus::SE3f Tb0w = Twb0.inverse();

    ofstream f(filename);
    // cam column: which camera the landmark was seen by. On the rig this is the
    // direct test of whether the geometry is right -- cam0 points should sit
    // AHEAD of the camera and cam1 points BEHIND it.
    //   0 = front only, 1 = rear only, 2 = both cameras observed it
    f << fixed << "t,x,y,z,cam,id" << endl;
    int n = 0, nbad = 0, n0 = 0, n1 = 0, nboth = 0;
    for(MapPoint* pMP : pBiggerMap->GetAllMapPoints())
    {
        if(!pMP || pMP->isBad()){ nbad++; continue; }
        Eigen::Vector3f P = Tb0w * pMP->GetWorldPos();   // world -> b0
        double t = 0.0;
        KeyFrame* pRef = pMP->GetReferenceKeyFrame();
        if(pRef) t = pRef->mTimeStamp;          // first-seen time -> growing cloud

        bool seen0 = false, seen1 = false;
        for(auto &ob : pMP->GetObservations())
        {
            KeyFrame* pKF = ob.first;
            if(!pKF) continue;
            const int li = get<0>(ob.second);
            const int ri = get<1>(ob.second);
            if(li >= 0) seen0 = true;   // left/front observation
            if(ri >= 0) seen1 = true;   // right/rear observation
        }
        int cam = seen0 && seen1 ? 2 : (seen1 ? 1 : 0);
        if(cam==0) n0++; else if(cam==1) n1++; else nboth++;

        f << setprecision(9) << t << ","
          << setprecision(6) << P(0) << "," << P(1) << "," << P(2) << ","
          << cam << "," << pMP->mnId << endl;
        n++;
    }
    cout << "  by camera: front-only " << n0 << ", rear-only " << n1
         << ", both " << nboth << endl;
    f.close();
    cout << "  wrote " << n << " map points (" << nbad << " bad/culled skipped)" << endl;
}

void System::SaveMapLines(const string &filename)
{
    cout << endl << "Saving map lines to " << filename << " ..." << endl;
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;
    size_t numMax = 0;
    for(Map* pMap : vpMaps)
        if(pMap->GetAllKeyFrames().size() > numMax){
            numMax = pMap->GetAllKeyFrames().size(); pBiggerMap = pMap; }
    if(!pBiggerMap){ cout << "  no map to save" << endl; return; }

    // Final consistency sweep, poses now final and all threads down: repair
    // every line against its observations, DELETE what cannot be explained.
    // Without this the saved map keeps lines whose keyframes moved after the
    // last neighbourhood sweep -- the audit's worst offenders.
    LocalMapping::RevalidateMapLines(pBiggerMap, true);

    // SAME FRAME AS THE POINTS AND THE TRAJECTORY (see SaveMapPoints).
    vector<KeyFrame*> vpKFsOrd = pBiggerMap->GetAllKeyFrames();
    sort(vpKFsOrd.begin(), vpKFsOrd.end(), KeyFrame::lId);
    Sophus::SE3f Twb0;
    if(!vpKFsOrd.empty())
        Twb0 = (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
                 ? vpKFsOrd[0]->GetImuPose() : vpKFsOrd[0]->GetPoseInverse();
    const Sophus::SE3f Tb0w = Twb0.inverse();

    ofstream f(filename);
    f << fixed << "t,x1,y1,z1,x2,y2,z2,validated,id,support" << endl;
    // UNVERIFIED candidates (insufficient observations for the multiview
    // check) are exported SEPARATELY: the map file holds only geometry that
    // passed reprojection against all its observations.
    ofstream fu(filename.substr(0, filename.find_last_of('.')) + "_unverified.csv");
    fu << fixed << "t,x1,y1,z1,x2,y2,z2,validated,id,support" << endl;
    long n = 0, noext = 0, nUnv = 0;
    for(MapLine* pML : pBiggerMap->GetAllMapLines())
    {
        if(!pML || pML->isBad()) continue;
        // Use the segment the camera ACTUALLY SAW, recovered by intersecting
        // the observation's endpoint bearings with the line. The previous
        // version drew a fixed-length stub at the line's point of closest
        // approach to the camera -- which is the perpendicular foot, so every
        // line landed at roughly constant radius around the trajectory and the
        // map rendered as a spherical shell of 10 cm dashes (51% of segments
        // were pinned at the 0.10 m floor). A Plucker line is infinite; the
        // only honest finite piece is the observed one.
        // Only landmarks that actually CONTRIBUTED to tracking: probation
        // (mnValidated >= 2) is the bar for entering the pose optimizers, so
        // everything below it never voted and does not belong in the map.
        if(pML->mnValidated < 2){ noext++; continue; }
        if(!pML->mbHasExtent){ noext++; continue; }
        Eigen::Vector3f e1 = pML->mEnd1, e2 = pML->mEnd2;
        if(!e1.allFinite() || !e2.allFinite()) continue;
        if((e2 - e1).norm() < 1e-3f) continue;
        e1 = Tb0w * e1;  e2 = Tb0w * e2;
        double t = 0.0;
        if(KeyFrame* pRef = pML->GetReferenceKeyFrame()) t = pRef->mTimeStamp;
        ofstream &dst = (pML->mnGeomVerdict == 1) ? f : fu;   // verified vs candidate
        dst << setprecision(9) << t << "," << setprecision(6)
          << e1(0) << "," << e1(1) << "," << e1(2) << ","
          << e2(0) << "," << e2(1) << "," << e2(2) << ","
          << pML->mnValidated << "," << pML->mnId << ","
          << pML->SupportCount() << endl;
        if(pML->mnGeomVerdict == 1) n++; else nUnv++;
    }
    f.close();
    fu.close();
    cout << "  " << n << " verified lines saved; " << nUnv
         << " unverified candidates -> _unverified.csv" << endl;

    {   // GROUND TRUTH FOR OFFLINE ANALYSIS, all at FINAL state and all in the
        // SAME b0 frame as the map above. Two files:
        //   kf_pose_b0.csv  T_c_b0 per keyframe per lens (b0 world -> lens),
        //                   computed here by the estimator's own composition.
        //   kf_line_obs.csv the segments that keyframe actually observed, with
        //                   the landmark id each is bound to.
        // Projecting a map line (by id) with the matching T_c_b0 must land on
        // its segment. Any offline reconstruction of T_c_b0 can be diffed
        // against this instead of assumed correct.
        const Sophus::SE3f Twb0_ = Tb0w.inverse();
        ofstream pf(filename.substr(0, filename.find_last_of('.')) + "_kfpose_b0.csv");
        ofstream of(filename.substr(0, filename.find_last_of('.')) + "_kfobs.csv");
        pf << fixed << "kfid,t,cam,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz\n";
        of << fixed << "kfid,t,cam,lineid,x1,y1,x2,y2\n";
        ofstream pof(filename.substr(0, filename.find_last_of('.')) + "_kfpts.csv");
        pof << fixed << "kfid,t,cam,pointid,u,v\n";
        const Sophus::SE3f T_c1_c0 = mRig.IsEnabled() ? mRig.T_c1_c0() : Sophus::SE3f();
        for(KeyFrame* pKF : vpKFsOrd)
        {
            if(!pKF || pKF->isBad()) continue;
            const Sophus::SE3f Tcw_raw = pKF->GetPose();          // raw world -> cam0
            for(int c = 0; c < 2; c++)
            {
                const Sophus::SE3f Tc = (c == 1 ? T_c1_c0 * Tcw_raw : Tcw_raw) * Twb0_;
                const Eigen::Matrix3f R = Tc.rotationMatrix();
                const Eigen::Vector3f t = Tc.translation();
                pf << pKF->mnId << "," << setprecision(9) << pKF->mTimeStamp
                   << "," << c << setprecision(6);
                for(int r = 0; r < 3; r++) for(int cc = 0; cc < 3; cc++) pf << "," << R(r,cc);
                pf << "," << t(0) << "," << t(1) << "," << t(2) << "\n";
            }
            for(size_t i = 0; i < pKF->mvLines.size() && i < pKF->mvpMapLines.size(); i++)
            {
                MapLine* pML = pKF->mvpMapLines[i];
                if(!pML || pML->isBad()) continue;
                const LineObs &lo = pKF->mvLines[i];
                of << pKF->mnId << "," << setprecision(9) << pKF->mTimeStamp << ","
                   << lo.cam << "," << pML->mnId << setprecision(3) << ","
                   << lo.p1.x << "," << lo.p1.y << "," << lo.p2.x << "," << lo.p2.y << "\n";
            }
            // THE REFERENCE PATH: the same thing for POINTS. Points are known
            // to behave; measuring them through the identical keyframe, pose
            // and projection turns "lines look wrong" into a bisect.
            {
                const vector<MapPoint*> vpMP = pKF->GetMapPointMatches();
                for(size_t i = 0; i < vpMP.size(); i++)
                {
                    MapPoint* pMP = vpMP[i];
                    if(!pMP || pMP->isBad()) continue;
                    const bool right = (pKF->NLeft != -1 && (int)i >= pKF->NLeft);
                    if(right && (int)(i - pKF->NLeft) >= (int)pKF->mvKeysRight.size()) continue;
                    if(!right && i >= pKF->mvKeysUn.size()) continue;
                    const cv::KeyPoint &kp = right ? pKF->mvKeysRight[i - pKF->NLeft]
                                                   : pKF->mvKeysUn[i];
                    pof << pKF->mnId << "," << setprecision(9) << pKF->mTimeStamp << ","
                        << (right ? 1 : 0) << "," << pMP->mnId << setprecision(3) << ","
                        << kp.pt.x << "," << kp.pt.y << "\n";
                }
            }
        }
    }

    if(noext) cout << "  skipped " << noext << " lines with no observed extent" << endl;
    {
        const long acc = MapLine::nAccepted, par = MapLine::nRejParallel,
                   dep = MapLine::nRejDepth, rat = MapLine::nRejRatio,
                   lng = MapLine::nRejLong;
        const long tot = acc + par + dep + rat + lng;
        if(tot) cout << "  extent gates: accepted " << 100.0*acc/tot << "%  |"
                     << " near-parallel(8deg) " << 100.0*par/tot << "%  "
                     << " depth-range " << 100.0*dep/tot << "%  "
                     << " endpoint-depth-ratio>5 " << 100.0*rat/tot << "%  "
                     << " longer-than-20m " << 100.0*lng/tot << "%" << endl;
    }
    cout << "  wrote " << n << " map lines" << endl;
    {   // Per-line diagnostics for the depth-correction question: does a
        // PERSISTENT line ever get its depth re-solved, or does it keep its
        // 2-deg birth geometry while happily re-binding on the plane gate?
        // Columns: id, validated sightings, creation parallax [deg], number
        // of KF observations, widest parallax available between any two of
        // its KF observation planes [deg] (0 if <2 KFs).
        ofstream fs(filename.substr(0, filename.find_last_of('.')) + "_stats.csv");
        // extent_off: distance of the stored endpoints from the landmark's OWN
        // line, |p x d - m| (0 if consistent). Non-zero means the extent was
        // left behind when (d,m) moved -- the drawn segment is not on the line.
        fs << "id,validated,create_par_deg,nkf,widest_kf_par_deg,extent_off\n";
        for(MapLine* pML : pBiggerMap->GetAllMapLines())
        {
            if(!pML || pML->isBad()) continue;
            float widest = 0.f;
            auto obs = pML->GetObservations();
            std::vector<Eigen::Vector3f> nw;
            for(auto& ob : obs)
            {
                KeyFrame* pKFi = ob.first;
                if(!pKFi || pKFi->isBad()) continue;
                for(int idx : ob.second)
                {
                    if(idx < 0 || idx >= (int)pKFi->mvLines.size()) continue;
                    const LineObs& lo = pKFi->mvLines[idx];
                    // plane normal in world: R_wc * n_c (lens pose = body here
                    // is WRONG for cam1: cam0-only stat; rear lines report 0)
                    if(lo.cam != 0) continue;
                    Sophus::SE3f Tcw = pKFi->GetPose();
                    nw.push_back(Tcw.rotationMatrix().transpose() * lo.n);
                }
            }
            for(size_t i2 = 0; i2 < nw.size(); i2++)
                for(size_t j2 = i2+1; j2 < nw.size(); j2++)
                    widest = std::max(widest,
                        std::asin(std::min(1.f, nw[i2].cross(nw[j2]).norm())));
            float eoff = -1.f;
            if(pML->mbHasExtent){
                const Eigen::Vector3f dd = pML->GetDirection(), mm = pML->GetMoment();
                eoff = std::max((pML->mEnd1.cross(dd) - mm).norm(),
                                (pML->mEnd2.cross(dd) - mm).norm());
            }
            fs << pML->mnId << "," << pML->mnValidated << ","
               << 180.0/M_PI*pML->mCreateParallax << "," << obs.size() << ","
               << 180.0/M_PI*widest << "," << eoff << "\n";
        }
        fs.close();
    }
    {   // Track-length metric (the starvation number): validated sightings
        // per landmark, over ALL landmarks ever created. Median 2.3 = starved.
        std::vector<int> v;
        for(MapLine* pML : pBiggerMap->GetAllMapLines())
            if(pML && !pML->isBad()) v.push_back(pML->mnValidated);
        if(!v.empty()){
            std::sort(v.begin(), v.end());
            double mean = 0; for(int x : v) mean += x; mean /= v.size();
            cout << "  line track length (validated sightings): median "
                 << v[v.size()/2] << "  mean " << mean
                 << "  p90 " << v[(size_t)(0.9*v.size())]
                 << "  landmarks alive " << v.size()
                 << "  ever created " << MapLine::nNextId << endl;
        }
    }
}

void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    int numMaxKFs = 0;
    // Uninitialised upstream: when no map has any keyframes the loop below
    // never assigns it, and the dereference that follows is undefined -- a
    // segfault at shutdown whenever a run ends just after a map reset.
    Map* pBiggerMap = nullptr;
    std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
    for(Map* pMap :vpMaps)
    {
        std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        cout << "  no map has any keyframes -- nothing to save" << endl;
        return;
    }
    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    if(vpKFs.empty())
    {
        cout << "  map has no keyframes -- nothing to save" << endl;
        return;
    }

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

void System::SaveTrajectoryEuRoC(const string &filename, Map* pMap)
{

    cout << endl << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    int numMaxKFs = 0;

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

/*void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    // Uninitialised upstream: when no map has any keyframes the loop below
    // never assigns it, and the dereference that follows is undefined -- a
    // segfault at shutdown whenever a run ends just after a map reset.
    Map* pBiggerMap = nullptr;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        cout << "  no map has any keyframes -- nothing to save" << endl;
        return;
    }
    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    if(vpKFs.empty())
    {
        cout << "  map has no keyframes -- nothing to save" << endl;
        return;
    }

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose_();
    else
        Twb = vpKFs[0]->GetPoseInverse_();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;


        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
            Sophus::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}*/


/*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    // Uninitialised upstream: when no map has any keyframes the loop below
    // never assigns it, and the dereference that follows is undefined -- a
    // segfault at shutdown whenever a run ends just after a map reset.
    Map* pBiggerMap = nullptr;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        cout << "  no map has any keyframes -- nothing to save" << endl;
        return;
    }
    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    if(vpKFs.empty())
    {
        cout << "  map has no keyframes -- nothing to save" << endl;
        return;
    }

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    // Uninitialised upstream: when no map has any keyframes the loop below
    // never assigns it, and the dereference that follows is undefined -- a
    // segfault at shutdown whenever a run ends just after a map reset.
    Map* pBiggerMap = nullptr;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    if(!pBiggerMap)
    {
        cout << "  no map has any keyframes -- nothing to save" << endl;
        return;
    }
    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    if(vpKFs.empty())
    {
        cout << "  map has no keyframes -- nothing to save" << endl;
        return;
    }

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap)
{
    cout << endl << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

/*void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        if(!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Tow;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << setprecision(9) << Rwc(0,0) << " " << Rwc(0,1)  << " " << Rwc(0,2) << " "  << twc(0) << " " <<
             Rwc(1,0) << " " << Rwc(1,1)  << " " << Rwc(1,2) << " "  << twc(1) << " " <<
             Rwc(2,0) << " " << Rwc(2,1)  << " " << Rwc(2,2) << " "  << twc(2) << endl;
    }
    f.close();
}


void System::SaveDebugData(const int &initIdx)
{
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_" +to_string(mpLocalMapper->mInitSect)+ "_" + to_string(initIdx)+".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mScale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mRwg(0,0) << "," << mpLocalMapper->mRwg(0,1) << "," << mpLocalMapper->mRwg(0,2) << endl;
    f << mpLocalMapper->mRwg(1,0) << "," << mpLocalMapper->mRwg(1,1) << "," << mpLocalMapper->mRwg(1,2) << endl;
    f << mpLocalMapper->mRwg(2,0) << "," << mpLocalMapper->mRwg(2,1) << "," << mpLocalMapper->mRwg(2,2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mCostTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" +to_string(mpLocalMapper->mInitSect)+ "_" +to_string(initIdx)+".txt", ios_base::app);
    f << fixed;
    for(int i=0; i<mpLocalMapper->mcovInertial.rows(); i++)
    {
        for(int j=0; j<mpLocalMapper->mcovInertial.cols(); j++)
        {
            if(j!=0)
                f << ",";
            f << setprecision(15) << mpLocalMapper->mcovInertial(i,j);
        }
        f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mInitTime << endl;
    f.close();
}


int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState==Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void System::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

float System::GetImageScale()
{
    return mpTracker->GetImageScale();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time)
{
    mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

void System::SaveAtlas(int type){
    if(!mStrSaveAtlasToFile.empty())
    {
        //clock_t start = clock();

        // Save the current session
        mpAtlas->PreSave();

        string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
        pathSaveFileName = pathSaveFileName.append(".osa");

        string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);
        std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
        string strVocabularyName = mStrVocabularyFilePath.substr(found+1);

        if(type == TEXT_FILE) // File text
        {
            cout << "Starting to write the save text file " << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::text_oarchive oa(ofs);

            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            cout << "End to write the save text file" << endl;
        }
        else if(type == BINARY_FILE) // File binary
        {
            cout << "Starting to write the save binary file" << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            cout << "End to write save binary file" << endl;
        }
    }
}

bool System::LoadAtlas(int type)
{
    string strFileVoc, strVocChecksum;
    bool isRead = false;

    string pathLoadFileName = "./";
    pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
    pathLoadFileName = pathLoadFileName.append(".osa");

    if(type == TEXT_FILE) // File text
    {
        cout << "Starting to read the save text file " << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        cout << "End to load the save text file " << endl;
        isRead = true;
    }
    else if(type == BINARY_FILE) // File binary
    {
        cout << "Starting to read the save binary file"  << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        cout << "End to load the save binary file" << endl;
        isRead = true;
    }

    if(isRead)
    {
        //Check if the vocabulary is the same
        string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);

        if(strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            cout << "The vocabulary load isn't the same which the load session was created " << endl;
            cout << "-Vocabulary name: " << strFileVoc << endl;
            return false; // Both are differents
        }

        mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
        mpAtlas->SetORBVocabulary(mpVocabulary);
        mpAtlas->PostLoad();

        return true;
    }
    return false;
}

string System::CalculateCheckSum(string filename, int type)
{
    string checksum = "";

    unsigned char c[MD5_DIGEST_LENGTH];

    std::ios_base::openmode flags = std::ios::in;
    if(type == BINARY_FILE) // Binary file
        flags = std::ios::in | std::ios::binary;

    ifstream f(filename.c_str(), flags);
    if ( !f.is_open() )
    {
        cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
        return checksum;
    }

    MD5_CTX md5Context;
    char buffer[1024];

    MD5_Init (&md5Context);
    while ( int count = f.readsome(buffer, sizeof(buffer)))
    {
        MD5_Update(&md5Context, buffer, count);
    }

    f.close();

    MD5_Final(c, &md5Context );

    for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
        char aux[10];
        sprintf(aux,"%02x", c[i]);
        checksum = checksum + aux;
    }

    return checksum;
}

} //namespace ORB_SLAM
