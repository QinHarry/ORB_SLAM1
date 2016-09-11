
#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "System.h"
#include "Frame.h"

using namespace std;

namespace RGBD_QIN {

class System;


class Tracker{

public:

    Tracker(System* pSys,const std::string &strSettingPath);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;


protected:
    // Main tracking function. It is independent of the input sensor.
    void Run();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    //ORB
    ORBextractor* mpORBextractorLeft;

    // System
    System* mpSystem;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    Frame mLastFrame;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

};

}//RGBD_QIN

#endif
