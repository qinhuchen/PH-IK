#pragma once
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <iostream>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/io/io.h>
#include <pcl-1.10/pcl/io/ply_io.h>
#include <pcl-1.10/pcl/common/transforms.h>
#include <pcl-1.10/pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <thread>
#include <fstream>
using namespace std;

#define MAINOS_9_PLANNINGGROUPNAMECODE 0
#define MAINOS_PLANNINGGROUPNAMECODE 1
#define RIGHT_ARM_PLANNINGGROUPNAMECODE 2
#define LEFT_ARM_PLANNINGGROUPNAMECODE 3
#define DUAL_ARM_PLANNINGGROUPNAMECODE 4
#define HEAD_PLANNINGGROUPNAMECODE 5
#define FULL_PLANNINGGROUPNAMECODE 6
#define PRESENTATION_PLANNINGGROUPNAMECODE 7
#define HEAD_PLANNINGGROUPNAME "head"
#define FULL_PLANNINGGROUPNAME "Full"
#define MAINOS_9_PLANNINGGROUPNAME "MainOS_9"
#define MAINOS_PLANNINGGROUPNAME "MainOS"
#define RIGHT_ARM_PLANNINGGROUPNAME "right_arm"
#define LEFT_ARM_PLANNINGGROUPNAME "left_arm"
#define DUAL_ARM_PLANNINGGROUPNAME "dual_arm"
#define ROBOT_DESCRIPTION "robot_description"
#define PRESENTATION_PLANNINGGROUPNAME "presentation"

#define CAR_FORWARD_JOINT_NAME "car_forward_joint"
#define CAR_LEFT_JOINT_NAME "car_left_joint"
#define CAR_ROTATE_JOINT_NAME "car_rotate_joint"

#define PLAN_SUCCESS 1
#define PLAN_FAILED -1

#define JOINTCODE_CQH 0
#define JOINTCODE_CZY 1
#define JOINTCODE_CXZ 2
#define JOINTCODE_ZB 3
#define JOINTCODE_SS 4
#define JOINTCODE_WY 5
#define JOINTCODE_CY 6
#define JOINTCODE_YT 7
#define JOINTCODE_DT 8
#define JOINTCODE_YJQH 9
#define JOINTCODE_YJCT 10
#define JOINTCODE_YDB 11
#define JOINTCODE_YZB 12
#define JOINTCODE_YXB 13
#define JOINTCODE_ZJQH 14
#define JOINTCODE_ZJCT 15
#define JOINTCODE_ZDB 16
#define JOINTCODE_ZZB 17
#define JOINTCODE_ZXB 18
#define JOINTCODE_ZSW 19

// #define DEBUG_OUTPUT
const double tau = 2 * M_PI;
class LaborPlan
{
private:
  ofstream fout;
  ros::NodeHandle nh;
  ros::Subscriber plan_result_sub;
  int PlanningGroupNameCode;
  vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory;
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  planning_interface::PlanningContextPtr context;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;
  moveit_msgs::RobotState trajectory_start;
  ros::Publisher display_pub;
  moveit_msgs::MoveGroupActionResult result;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_state;
  const robot_state::JointModelGroup* joint_model_group;
  void result_CB(const moveit_msgs::MoveGroupActionResult &res);
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model::RobotModelPtr robot_model;
  moveit::planning_interface::MoveGroupInterface *move_group;
  std::string planning_group_name;
  ros::Publisher pcl_pub;
public:
  vector<string> full_joint_name;
  sensor_msgs::JointState state_control;
  ros::Publisher state_control_pub;
  void getJointTrajectoy(vector<trajectory_msgs::JointTrajectoryPoint> &joint_trajectory_);
  void getTrajectoryStart(moveit_msgs::RobotState &trajectory_start);
  int StartPlan(vector<double> joint_position,vector<trajectory_msgs::JointTrajectoryPoint> &joint_trajectory);
  int Move();
  int Move(const sensor_msgs::JointState &goal);
  int Plan(const vector<double>& joint_goal_position,int maxCount,std::string Planner);
  int getMinCountVector(const vector<vector<trajectory_msgs::JointTrajectoryPoint>>& plan_res);
  void AddBigScene(string filename);
  void AddScene();
  void AddRealWorld();
  void CreateWorld();
  void CreateDynamicWorld();
  void pick();
  bool CheckSelfCollision(vector<double> &robot_configuration);
  bool CheckCollision(vector<double> &robot_configuration);
  void AddFullBigTree();
  void Add362();
  void AddBigTree(string filename);
  void MAINOS_JointPositionPrintf(vector<trajectory_msgs::JointTrajectoryPoint> &joint_trajectory);
  void DualArm_JointPositionPrintf(vector<trajectory_msgs::JointTrajectoryPoint> &joint_trajectory);
  void JointStatePrintf(vector<trajectory_msgs::JointTrajectoryPoint> &joint_trajectory);
  void VectorPrintf(const vector<double>& v);
  void PlanResultPrintf(int plan_count,bool result,vector<trajectory_msgs::JointTrajectoryPoint> &joint_trajectory);
  LaborPlan(int PlanningGroupNameCode);
  ~LaborPlan();
};


