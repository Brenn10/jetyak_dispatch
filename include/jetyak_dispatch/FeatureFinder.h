/**********************************************************
Searches for features of interest for the quad to explore

Author: Brennan Cain
Modified: Jan 23 2018
*/

#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "geometry_msgs/Pose.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#ifndef JETYAK_DISPATCH_FEATUREFINDER_H_
#define JETYAK_DISPATCH_FEATUREFINDER_H_

// Searches for features to explore
class FeatureFinder
{
  public:
    FeatureFinder(ros::NodeHandle& nh);
    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
  private:
    ros::Publisher featurePub;
    ros::Subscriber tagSub;
};
#endif
