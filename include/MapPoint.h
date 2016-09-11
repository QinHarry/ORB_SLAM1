/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"Frame.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace RGBD_QIN
{

class Frame;


class MapPoint
{
public:

    MapPoint(const cv::Mat &Pos,Frame* pFrame, const int &idxF);

    cv::Mat GetWorldPos();

    cv::Mat GetDescriptor();

    void ComputeDistinctiveDescriptors();



public:
    long unsigned int mnId;
    static long unsigned int nNextId;


protected:
    // Position in absolute coordinates
    cv::Mat mWorldPos;

    // Mean viewing direction
    cv::Mat mNormalVector;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;


};

} //namespace RGBD_QIN

#endif // MAPPOINT_H
