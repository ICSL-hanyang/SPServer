#include <setpoint_server.h>

SPServer::SPServer() : name("camila"),
                       nh(ros::NodeHandle(name)),
                       nh_global(ros::NodeHandle("~"))
{
    SPInit();
}

SPServer::SPServer(std::string _name) : name(_name),
                                        nh(ros::NodeHandle(name)),
                                        nh_global(ros::NodeHandle("~"))
{
    SPInit();
}

SPServer::~SPServer()
{
}

void SPServer::SPInit()
{
    tar_local.pose.position.x = 0;
    tar_local.pose.position.y = 0;
    tar_local.pose.position.z = 2.5;

    state_sub = nh.subscribe("state", 10, &SPServer::stateCB, this);
    local_pos_sub = nh.subscribe("local_position/pose", 10, &SPServer::localPositionCB, this);
    global_pos_sub = nh.subscribe("global_position/global", 10, &SPServer::globalPositionCB, this);

    setpoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 10);
    setpoint_vel_pub = nh.advertise<geometry_msgs::Twist>("setpoint_velocity/cmd_vel_unstamped", 10);
    reached_pub = nh.advertise<std_msgs::Int32>("reached", 10);

    setpoint_server = nh.advertiseService("sp_server", &SPServer::targetCB, this);
}

void SPServer::stateCB(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}

void SPServer::localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    cur_local = *msg;
}

void SPServer::globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    cur_global = *msg;
}

bool SPServer::targetCB(setpoint_server::SetPoint::Request &req,
                        setpoint_server::SetPoint::Response &res)
{
    if ((tar_local.pose.position.x != req.x) ||
        (tar_local.pose.position.y != req.y) ||
        (tar_local.pose.position.z != req.z))
    {
        tar_local.pose.position.x = req.x;
        tar_local.pose.position.y = req.y;
        tar_local.pose.position.z = req.z;

        ROS_INFO("%s receive target_local_pos(x : %lf, y : %lf, z : %lf)", name.c_str(),
                 tar_local.pose.position.x, tar_local.pose.position.y, tar_local.pose.position.z);
    }

    res.success = true;

    return res.success;
}

void SPServer::gotoLocal()
{
    tar_local.header.seq += 1;
    tar_local.header.stamp = ros::Time::now();
    setpoint_local_pub.publish(tar_local);
}

void SPServer::gotoVel()
{
    double kp;
    nh_global.getParam("pid/kp", kp);
    geometry_msgs::Twist vel;

    vel.linear.x = (tar_local.pose.position.x - cur_local.pose.position.x) * kp;
    vel.linear.y = (tar_local.pose.position.y - cur_local.pose.position.y) * kp;
    vel.linear.z = (tar_local.pose.position.z - cur_local.pose.position.z) * kp;

    setpoint_vel_pub.publish(vel);
}

bool SPServer::check()
{
    double dist;
    bool pre_reached = reached;

    dist = sqrt(pow(tar_local.pose.position.x - cur_local.pose.position.x, 2) +
                pow(tar_local.pose.position.y - cur_local.pose.position.y, 2) +
                pow(tar_local.pose.position.z - cur_local.pose.position.z, 2));

    if (dist < 0.5)
    {
        reached = true;
        if (pre_reached != reached)
        {
            std_msgs::Int32 msg;
            nh_global.getParam("ID", msg.data);
            reached_pub.publish(msg);
        }
    }
    else
        reached = false;

    return reached;
}

bool SPServer::pub()
{
    bool control_method;
    nh_global.getParam("use_vel", control_method);
    if (control_method)
        gotoVel();
    else
        gotoLocal();
}