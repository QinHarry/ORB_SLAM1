#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "ORBextractor.h"
#include "Timer.h"

int main(){



    cv::Mat im = cv::imread("/home/slam/Documents/qin/ORB_slam_qin1/data/1.png",0);
    std::vector<cv::KeyPoint> mv_keys;
    cv::Mat mDescriptors,mv_keys_show;
    RGBD_QIN::Timer t;
    RGBD_QIN::ORBextractor* test;
    test = new RGBD_QIN::ORBextractor(1000,1.2f,8,20,7);
    (*test)(im,cv::Mat(),mv_keys,mDescriptors);
    std::cout << "This take :" << t.stop()*10 << "ms" << std::endl;
    std::cout << "The number of keys: " << mv_keys.size() << std::endl;
    char filename[] ="temp.txt";
    std::ofstream fout(filename);
    for(int i=0;i<mv_keys.size();i++){
        fout << "x: " << mv_keys[i].pt.x << " y: " << mv_keys[i].pt.y << "    \n";
    }
    cv::drawKeypoints(im,mv_keys,mv_keys_show,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);

    cv::imshow("show",mv_keys_show);



    cv::waitKey(0);
    return 0;


}
