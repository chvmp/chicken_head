#ifndef CHICKEN_HEAD_H
#define CHICKEN_HEAD_H

#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <champ_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

class ChickenHead
{
    using Vector3d = Eigen::Vector3d;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cmd_pose_subscriber_;
    ros::Publisher joint_state_publisher_;

    champ_msgs::Pose req_pose_;
    ros::Timer loop_timer_;

    float nominal_height_;

    Vector3d initial_pos_ = Vector3d(0.391, 0.0, 0.165); 
    Vector3d base_to_lower_arm_ = Vector3d(0.070, 0.000, 0.100); 
    Vector3d lower_to_upper_arm_ = Vector3d(0.195, 0.000, 0.000); 
    Vector3d upper_arm_to_wrist1_ = Vector3d(0.195, 0.000, 0.000); 
    Vector3d wrist1_to_wrist2_ = Vector3d(0.065, 0.000, 0.000);

    float l1_;
    float l2_;

    Vector3d rotate(const Vector3d, const float alpha, const float phi, const float beta);
    void cmdPoseCallback_(const champ_msgs::Pose::ConstPtr& msg);
    void controlLoop_(const ros::TimerEvent& event);
    public:
        ChickenHead(const ros::NodeHandle &node_handle,
                    const ros::NodeHandle &private_node_handle);
};

#endif