#include "jetyak_dispatch/FeatureFinder.h"

FeatureFinder::FeatureFinder(ros::NodeHandle& nh)
{
  featurePub = nh.advertise<geometry_msgs::Pose>("feature_finder",1);
  tagSub = nh.subscribe("feature_tag_tracker/ar_pose_marker",5,&FeatureFinder::arTagCallback,this);
}

void FeatureFinder::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  if(!msg->markers.empty())
  {
    geometry_msgs::Pose ftPose = msg->markers[0].pose.pose;
    ROS_WARN("Tag Found: {x=%.2f, y=%.2f, z=%.2f}",ftPose.position.x,ftPose.position.y,ftPose.position.z);
    featurePub.publish(ftPose);
  }
}
main(int argc, char** argv) {
  ros::init(argc,argv,"feature_finder");
  ros::NodeHandle nh;
  FeatureFinder feature_finder(nh);
  ros::spin();

}
