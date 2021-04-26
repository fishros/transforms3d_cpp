#include "iostream"
#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
using namespace std;

void send_opendoor_tf(tf::TransformBroadcaster &br, float x, float y, float z, float roll, float pitch, float yall, const std::string &frame_id, const std::string &child_frame_id)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x/1000, y/1000, z/1000));

    tf::Quaternion q;
    q.setRPY(roll/180.0*M_PI, pitch/180.0*M_PI, yall/180.0*M_PI);

    transform.setRotation(q);

    // 发布坐标变换
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_operate");
    ros::NodeHandle n;
    tf::TransformBroadcaster br;
    ros::Rate rate = ros::Rate(10);
    while (ros::ok())
    {
        send_opendoor_tf(br, 0,0, 0, 0, 0, 45, "world", "base");
        send_opendoor_tf(br, 3.000006, -279.999992, 375, 180, 0.000001, -90, "base", "grapper");
        send_opendoor_tf(br, 55.5,-55.5,-105.97,0,0,45, "grapper","camera");
        send_opendoor_tf(br, -2.48,15.66,300,0,0,-45, "camera","bootle");
        // send_opendoor_tf(br, 3,-343,163.6,180,0,-135, "base", "bootle");
        rate.sleep();
    }
    ros::spin();
    return 0;
}