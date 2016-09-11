#include "Tracker.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<iostream>
#include<mutex>

namespace RGBD_QIN {

Tracker::Tracker(System* pSys,const std::string &strSettingPath):
    mState(NO_IMAGES_YET){

    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
    cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;

    mDepthMapFactor = fSettings["DepthMapFactor"];
    if(mDepthMapFactor==0)
        mDepthMapFactor=1;
    else
        mDepthMapFactor = 1.0f/mDepthMapFactor;

}

cv::Mat Tracker::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp){
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);
    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mK,mDistCoef,mbf,mThDepth);

    Run();

    return mCurrentFrame.mTcw.clone();
}

void Tracker::Run(){
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    if(mState==NOT_INITIALIZED){
        StereoInitialization();
        if(mState!=OK)
                return ;
    }else{

        ORBmatcher matcher(0.9,true);

        if(mCurrentFrame.mnId > 1){
            mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
        }else{
            mCurrentFrame.SetPose(mLastFrame.mTcw);
        }

        if(!mLastFrame.mTcw.empty()){
            cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
            mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
            mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
            mVelocity = mCurrentFrame.mTcw*LastTwc;
        }else
            mVelocity = cv::Mat();

        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,15);

        // If few matches, uses a wider window search
        if(nmatches<20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,30);
        }

        std::cout << "The image Id is: " << mCurrentFrame.mnId << "And the number of mathcers is " << nmatches << std::endl;
    }

    mLastFrame = Frame(mCurrentFrame);


}

void Tracker::StereoInitialization(){
    if(mCurrentFrame.N>500){
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,&mCurrentFrame,i);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        mState=OK;
    }
}

}
