#pragma once
#include <ros/ros.h>
#include <iostream>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include "iostream"
#include "ctime"
#include "cstdlib"
#include <eigen3/Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <fstream>

using namespace std;
using namespace Eigen;
#define RIGHT_ENDEFFECTOR 0
#define LEFT_ENDEFFECTOR 1

#define MainOSNAMECODE 0
#define FullNAMECODE 1
#define LeftArmNAMECODE 2

class LaborKinematics
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient GetPositionFKClient;
    ros::ServiceClient GetPositionIKClient;
    ros::Publisher marker_pub;
    moveit_msgs::GetPositionFK fk;
    moveit_msgs::GetPositionIK ik;
    string right_endeffector_center="right_endeffector_center_link";
    string left_endeffector_center="left_endeffector_center_link";
    string right_endeffector="right_wrist_link";
    string left_endeffector="left_wrist_link";
    string base="base_link";
    string realsense="realsense_link";
    vector<string> mainos_joint_name;
    
public:
    LaborKinematics();
    ~LaborKinematics();
    Eigen::Matrix4f Pose2Matrix(const geometry_msgs::Pose &pose);
    geometry_msgs::Pose Matrix2Pose(const Eigen::Matrix4f &matrix);
    bool MainOS_FK(const vector<double> &config , geometry_msgs::Pose &fk_pose);
    bool MainOS_IK(const geometry_msgs::Pose &fk_pose,vector<double> &config);
    bool MainOS_EE_FK(const vector<double> &config , geometry_msgs::Pose &fk_pose);
    bool LeftArm_EE_FK(const vector<double> &config , geometry_msgs::Pose &fk_pose);
    bool LeftArm_FK(const vector<double> &config , geometry_msgs::Pose &fk_pose);
    bool RightArm_FK(const vector<double> &config , geometry_msgs::Pose &fk_pose);
    bool RightArm_EE_FK(const vector<double> &config , geometry_msgs::Pose &fk_pose);
    bool LeftArm_IK(const geometry_msgs::Pose &fk_pose,vector<double> &config);
    bool Perception_FK(const vector<double> &config , geometry_msgs::Pose &fk_pose);
    geometry_msgs::Pose PoseMulti(const geometry_msgs::Pose pose1,const geometry_msgs::Pose pose2);
    geometry_msgs::Pose EEtrans(bool left_or_right,const geometry_msgs::Pose &pose);
    vector<double> MainOSExtract(const vector<double> &ik_res);
    vector<double> LeftArmExtract(const vector<double> &ik_res);
    vector<double> FullExtract(const vector<double> &ik_res);
    Eigen::Matrix4f left_arm_EEtrans;
    Eigen::Matrix4f right_arm_EEtrans;
    void MainOS_DrawLine(vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory);
    void Full_DrawLine(vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory);
    void EEPositionPrintf(vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory,int planning_group_name_code);
};


template <class T>
void vector_show(const vector<T> &t)
{
    for(auto it=t.begin();it!=t.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
}
