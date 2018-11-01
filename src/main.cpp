#include <iostream>
#include <ros/ros.h>
#include <setpoint_server.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sp_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
    std::string name;
    nh.getParam("name", name);

    SPServer server(name);

    while(ros::ok()){
        server.pub();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}