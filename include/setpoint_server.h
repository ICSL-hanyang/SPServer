#ifndef SPSERVER_H
#define SPSERVER_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <setpoint_server/SetPoint.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

class SPServer
{
  public:
    SPServer();
    SPServer(std::string _name);
    SPServer(SPServer &&) = default;
    SPServer(const SPServer &) = default;
    SPServer &operator=(SPServer &&) = default;
    SPServer &operator=(const SPServer &) = default;
    ~SPServer();

    void SPInit();

    void stateCB(const mavros_msgs::State::ConstPtr &msg);
    void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &msg);

    bool targetCB(setpoint_server::SetPoint::Request &req,
               setpoint_server::SetPoint::Response &res);
    
    void gotoLocal();
    void gotoVel();

    bool check();
    bool pub();

  private:
    std::string name;

    ros::NodeHandle nh;
    ros::NodeHandle nh_global;

    /* ros subscriber*/
    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber global_pos_sub;

    /* ros publisher*/
    ros::Publisher setpoint_vel_pub;
    ros::Publisher setpoint_local_pub;
    ros::Publisher reached_pub;

    /* ros service server */
    ros::ServiceServer setpoint_server;

    /*drone state*/
    mavros_msgs::State cur_state;

    /* local coordinate*/
    geometry_msgs::PoseStamped cur_local;
    geometry_msgs::PoseStamped tar_local;

    /* global coordinate*/
    sensor_msgs::NavSatFix cur_global;
    sensor_msgs::NavSatFix tar_global;

    bool reached;
};

#endif