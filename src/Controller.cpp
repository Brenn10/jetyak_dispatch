#include "jetyak_dispatch/Controller.h"

#include <cstdlib>
#include <vector>
#include <ctime>
#include <sstream>

#include <math.h>

#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


/*Creates the Controller object and initializes all of the

*/
Controller::Controller(ros::NodeHandle& nh)
{
  //PID Controllers
  yaw_pid=new PIDController(.2,.05,0.0);
  z_pid=new PIDController(.4,-.1,0.0);
  x_pid=new PIDController(.25,.8,0.0);
  y_pid=new PIDController(.25,.8,0.0);

  //Publishers
  landPub = nh.advertise<std_msgs::Empty>("/bebop/land",1);
  takeOffPub = nh.advertise<std_msgs::Empty>("/bebop/takeoff",1);
  cmdVelPub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);

  //Subscribers
  baseCamSub = nh.subscribe("bebop_tag_tracker/ar_pose_marker",5,&Controller::baseCamCallback, this);
  featureSub = nh.subscribe("feature_finder",5,&Controller::featureFinderCallback, this);
  gpsSub = nh.subscribe("/bebop/fix",5,&Controller::gpsCallback,this);

  //assorted variable initializations
  lastSpottedLanding.x=lastSpottedLanding.y=lastSpottedLanding.z=0.0;

  //ros parameters
  nh.param<double>("maxDispatchVel",maxDispatchVel,.5);
  nh.param<double>("landStartAlt",landStartAlt,2.0);
  nh.param<double>("landingHeight",landingHeight,.05);
  nh.param<double>("landingHeightDecrement",landingHeightDecrement,.005);

  usleep(1000*1000); //wait to ensure everything is good
}

/**Controls the operation of the quadcopter when it has been spotted by the base
If no markers and was landing and was landing:
  retry returning mode if too high
  land if low enough
if markers present and returning, enter landing mode
if markers present and in landing mode, attempt landing procedure
TODO: test
*/
void Controller::baseCamCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  //If it was landing and we lost track
  if(msg->markers.empty())
  {
    if(state==landing)
    {
      //if it was just within landing distance
      if(lastSpottedLanding.z<=landingHeight)
        land();
      //if we lost it high up (most likely due to side to side movement)
      else
        setDispatchState(returning);
    }
  }
  else//tag found
  {
    if(state==returning)//if was returning, now landing
    {
      setDispatchState(landing);

    }
    if(state=landing)
    {
      currLandingGoal= currLandingGoal-landingHeightDecrement;
      lastSpottedLanding.x = msg->markers[0].pose.pose.position.x;
      lastSpottedLanding.y = msg->markers[0].pose.pose.position.y;
      lastSpottedLanding.z = msg->markers[0].pose.pose.position.z;

      tf::Quaternion q(
        msg->markers[0].pose.pose.orientation.x,
        msg->markers[0].pose.pose.orientation.y,
        msg->markers[0].pose.pose.orientation.z,
        msg->markers[0].pose.pose.orientation.w
      );
      tf::Matrix3x3 m(q);
      double t_r, t_p, t_y;
      m.getRPY(t_r, t_p, t_y);

      t_y = (t_y-M_PI/2);
      if(t_y<-M_PI) {
        t_y = 2*M_PI+t_y;
      }
      yaw_pid->update(t_y);
      x_pid->update(lastSpottedLanding.x);
      y_pid->update(lastSpottedLanding.y);
      z_pid->update(currLandingGoal-lastSpottedLanding.z);

      sendCmd(x_pid->signal,y_pid->signal,z_pid->signal,yaw_pid->signal);
    }
  }
}
/** Control the behavior of the robot for when the quad when a feature is noticed
If on the boat, take off and enter dispatch mode
if in dispatch adn more than a meter away, move toward it
if in dispatch and less than a meter away, enter returning mode
TODO: Test
*/
void Controller::featureFinderCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  if(state==landed)
    setDispatchState(dispatched);
  else if(state==dispatched)
  {
    lastFeaturePoint.x=msg->position.x;
    lastFeaturePoint.y=msg->position.y;
    lastFeaturePoint.z=msg->position.z;

    if(msg->position.x<=1)
    {
      setDispatchState(returning);
    }
    else
    {
      z_pid->update(msg->position.z);
      yaw_pid->update(msg->position.y);

      double vel = std::min(maxDispatchVel,msg->position.x/5-.5);
      double angle = std::atan2(msg->position.y,msg->position.x);

      sendCmd(vel*std::cos(angle),vel*std::sin(angle),z_pid->signal,yaw_pid->signal);
    }
  }

}

/**get the gps coords for when returning
get coords of boat and set as endpoint of a flightplan
upload flightplan
call flightplan
TODO: implement
*/
void Controller::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ROS_WARN("Long: %.6f    Lat: %.6f",msg->longitude, msg->latitude);
}

void Controller::setDispatchState(State s)
{
  yaw_pid->reset();
  x_pid->reset();
  y_pid->reset();
  z_pid->reset();
  switch (s)
  {
    case landing:{
      ROS_WARN("%s","Landing");
      sendCmd(0,0,0,0);
      currLandingGoal=landStartAlt;
      break;
    }
    case landed:{
      ROS_WARN("%s","Landed");
      break;
    }
    case dispatched:{
      ROS_WARN("%s","Distpatched");
      z_pid->reset();
      takeoff();
      break;
    }
    case returning:{
      ROS_WARN("%s","Returning");
      break;
    }
  }
  this->state=s;
}
void Controller::takeoff()
{
  //takeOffPub.publish(empty_msg);
  ROS_WARN("%s","I would take off now");
}
void Controller::land()
{
  landPub.publish(empty_msg);
}
void Controller::sendCmd(double vx,double vy, double vz, double vyaw)
{
  geometry_msgs::Twist cmdT;
  cmdT.linear.x=std::min(vx,maxDispatchVel);
  cmdT.linear.y=std::min(vy,maxDispatchVel);
  cmdT.linear.z=std::min(vz,maxDispatchVel);
  cmdT.angular.z=std::min(vyaw,maxDispatchVel);
  cmdT.angular.x=cmdT.angular.y=0;

  //cmdVelPub.publish(cmdT);
  ROS_WARN("CMD: x:%.2f, y:%.2f, z:%.2f, yaw:%.2f",cmdT.linear.x,cmdT.linear.y,cmdT.linear.z,cmdT.angular.z);
}
void Controller::shutdown()
{

}

main(int argc, char** argv) {
  ros::init(argc,argv, "quad_pilot");
  ros::NodeHandle nh;
  Controller quad_pilot(nh);
  ros::spin();

  quad_pilot.shutdown();
}
