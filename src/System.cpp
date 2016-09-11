#include "System.h"

namespace RGBD_QIN {


//System::System(const int a){

//    std::cout<<a<<std::endl;
//}

System::System(const std::string &strSettingsFile):mbReset(false),mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false){

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Create Drawers. These are used by the Viewer
    //mpFrameDrawer = new FrameDrawer(mpMap);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracker(this,strSettingsFile);
}


cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp){

    return mpTracker->GrabImageRGBD(im,depthmap,timestamp);
}

}
