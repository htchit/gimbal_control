#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int16.h"
#include "../include/MiniPID.hpp"

geometry_msgs::Vector3 angle_exp;
std_msgs::Float32MultiArray angle_fb;
float x_fb = 320;
float y_fb = 240;

void expected_cb(const geometry_msgs::Vector3 &msg)
{
    angle_exp = msg;
}

void feedback_cb(const std_msgs::Float32MultiArray &msg)
{
    angle_fb = msg;
    x_fb = angle_fb.data[0];
    y_fb = angle_fb.data[1];
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dron_position_control");

    ros::NodeHandle n;

    ros::Subscriber gimbal_angle_exp = n.subscribe("/expected_pos", 1000, expected_cb);
    ros::Subscriber gimbal_angle_fb = n.subscribe("/track_result", 1000, feedback_cb);

    ros::Publisher gimbal_p_exp = n.advertise<std_msgs::Int16>("/gimbal/setpoint/angular_vel/p", 1000);
    ros::Publisher gimbal_y_exp = n.advertise<std_msgs::Int16>("/gimbal/setpoint/angular_vel/y", 1000);

    ros::Rate loop_rate(10);
    double x_P;
    double x_I;
    double x_D;
    double y_P;
    double y_I;
    double y_D;

    n.getParam("x_P", x_P);
    n.getParam("x_I", x_I);
    n.getParam("x_D", x_D);
    n.getParam("y_P", y_P);
    n.getParam("y_I", y_I);
    n.getParam("y_D", y_D);

    std_msgs::Int16 exp_p;
    std_msgs::Int16 exp_y;

    MiniPID PID_X;
    MiniPID PID_Y;
    // MiniPID PID_Z;
    PID_X.Init(x_P, x_I, x_D, 10, 50, 100);
    PID_Y.Init(y_P, y_I, y_D, 10, 50, 100);
    // PID_Z.Init(1, 0, 0, 10, 2, 5);

    while (ros::ok())
    {
        exp_y.data = PID_X.run(angle_exp.x, x_fb);
        exp_p.data = PID_Y.run(angle_exp.y, y_fb);
        // vel_exp.z = PID_Z.run(angle_exp.z, angle_fb.z);

        gimbal_p_exp.publish(exp_p);
        gimbal_y_exp.publish(exp_y);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
