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
  yaw_pid=new PIDController(.04,.01,0.0);
  z_pid=new PIDController(.4,-.1,0.0);
  x_pid=new PIDController(.2,.1,0.0);
  y_pid=new PIDController(.2,.1,0.0);
  landingy_pid = new PIDController(.25,.8,0.0);
  landingx_pid = new PIDController(.25,.8,0.0);

  //Publishers
  landPub = nh.advertise<std_msgs::Empty>("land",1);
  takeOffPub = nh.advertise<std_msgs::Empty>("takeoff",1);
  cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

  //Subscribers
  baseCamSub = nh.subscribe("quad_tag_tracker/ar_pose_marker",5,&Controller::baseCamCallback, this);
  featureSub = nh.subscribe("feature_finder",5,&Controller::featureFinderCallback, this);
  gpsSub = nh.subscribe("fix",5,&Controller::gpsCallback,this);

  //assorted variable initializations
  lastSpottedLanding.x=lastSpottedLanding.y=lastSpottedLanding.z=0.0;

  //ros parameters
  ros::param::get("~maxDispatchVel",maxDispatchVel);
  ros::param::get("~landStartAlt",landStartAlt);
  ros::param::get("~landingHeight",landingHeight);
  ros::param::get("~landingHeightDecrement",landingHeightDecrement);
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
      ROS_WARN("Curr: %.2f, goal: %.2f",lastSpottedLanding.z,landingHeight);
      //if it was just within landing distance
      if(lastSpottedLanding.z<=landingHeight)
      {
        land();
        setDispatchState(landed);
      }
      //if we lost it high up (most likely due to side to side movement)
      else
        setDispatchState(returning);
    }
  }
  else//tag found
  {
    //If it is an atrociously small number, it can be thrown out
    if(std::abs(msg->markers[0].pose.pose.position.x)+std::abs(msg->markers[0].pose.pose.position.y)+std::abs(msg->markers[0].pose.pose.position.z)<FAKE_AR_DIST)
      return;
    ROS_INFO("Quad spotted at: %f %f %f",msg->markers[0].pose.pose.position.x,msg->markers[0].pose.pose.position.y,msg->markers[0].pose.pose.position.z);
    if(state==returning)//if was returning, now landing
    {
      setDispatchState(landing);
    }
    if(state==landing)  //if was landing
    {
      if(msg->markers[0].pose.pose.position.z<landingHeight) // if height is small, land
      {
        land();
        setDispatchState(landed);
      }
      else //otherwise decrement goal height and fix x,y position
      {
        currLandingGoal= currLandingGoal-landingHeightDecrement;
        lastSpottedLanding.x = -msg->markers[0].pose.pose.position.y;
        lastSpottedLanding.y = msg->markers[0].pose.pose.position.x;
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
        landingx_pid->update(-lastSpottedLanding.x);
        landingy_pid->update(-lastSpottedLanding.y);
        z_pid->update(currLandingGoal-lastSpottedLanding.z);

        //Rotate velocities by the yaw of the quad
        Eigen::Vector2d v(landinxx_pid->signal,landingy_pid->signal);
        Eigen::Matrix2d T;
        T << cos(t_y),sin(t_y),
            -sin(t_y),cos(t_y);
        v = T*v;

        ROS_INFO("Height: %.2f",lastSpottedLanding.z);
        sendCmd(v[0],v[1],z_pid->signal,yaw_pid->signal);
      }
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
  if(std::abs(msg->position.x)+std::abs(msg->position.y)+std::abs(msg->position.z)<=FAKE_AR_DIST)
    return;
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
      x_pid->update(msg->position.x-.5);
      y_pid->update(msg->position.y);
      z_pid->update(msg->position.z);
      yaw_pid->update(std::atan2(-msg->position.y,msg->position.x));

      double xVel = std::min(x_pid->signal,maxDispatchVel);
      double yVel = std::min(y_pid->signal,maxDispatchVel);
      sendCmd(xVel,yVel,z_pid->signal,yaw_pid->signal);
    }
  }

}

/**get the gps coords of quad for when returning
get coords of boat and set as endpoint of a flightplan
upload flightplan
call flightplan
TODO: implement
*/
void Controller::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ROS_INFO("Long: %.6f    Lat: %.6f",msg->longitude, msg->latitude);
}

void Controller::setDispatchState(State s)
{
  yaw_pid->reset();
  x_pid->reset();
  y_pid->reset();
  z_pid->reset();
  landingy_pid->reset();
  landingx_pid->reset();
  switch (s)
  {
    case landing:{
      ROS_WARN("%s","Landing");
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
  takeOffPub.publish(empty_msg);
  //ROS_WARN("%s","I would take off now");
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
  cmdT.angular.z=vyaw;
  cmdT.angular.x=cmdT.angular.y=0;

  cmdVelPub.publish(cmdT);
  //ROS_WARN("CMD: x:%.2f, y:%.2f, z:%.2f, yaw:%.2f",cmdT.linear.x,cmdT.linear.y,cmdT.linear.z,cmdT.angular.z);
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
