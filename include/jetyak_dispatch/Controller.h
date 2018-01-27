/**********************************************************
Controller for the quadcopter which takes input
from the ground station camera, feature detectors, and
GPS

Author: Brennan Cain
Modified: Jan 23 2018
*/


#ifndef JETYAK_DISPATCH_CONTROLLER_H_
#define JETYAK_DISPATCH_CONTROLLER_H_

#include "jetyak_dispatch/FeatureFinder.h"
#include "jetyak_dispatch/PIDController.h"

#include "ros/ros.h"

//message types
#include "std_msgs/Empty.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/NavSatFix.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h"

// This class controls the quadcopter
class Controller
{
  public:
    enum State {landed, dispatched, returning, landing};
    Controller(ros::NodeHandle& nh);
    void control_loop();
    void shutdown();

    //Callbacks
    void baseCamCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void featureFinderCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  private:
    const double FAKE_AR_DIST = .2;
    //State tracker

    State state = landed;
    //Node methods
    void setDispatchState(State s);
    void takeoff();
    void land();
    void sendCmd(double vx,double vy, double vz, double vyaw);

    //Publishers
    ros::Publisher landPub,
      takeOffPub,
      cmdVelPub;

    //Subscribers
    ros::Subscriber baseCamSub,
      featureSub,
      gpsSub;

    //PID Controllers
    PIDController *yaw_pid,
      *z_pid,
      *x_pid,
      *y_pid,
      *landingy_pid,
      *landingx_pid;

    double landStartAlt;
    double currAltitude;
    double maxDispatchVel;
    double landingHeight;
    double landingHeightDecrement;
    double currLandingGoal;

    //keeps track of where the quad is while landing
    geometry_msgs::Point lastSpottedLanding;
    geometry_msgs::Point lastFeaturePoint;

    //empty message so i dont have to recreate constantly
    std_msgs::Empty empty_msg;
};


#endif
