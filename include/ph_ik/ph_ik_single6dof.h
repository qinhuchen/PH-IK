#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/octree/octree.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <iostream>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "ph_ik/labor_plan.h"
#include "ph_ik/labor_kinematics.h"

// #define DEBUG_SHOW
using namespace std;
using namespace fcl;
#define P1P2 1
#define P2P3 2
#define P3P4 3
#define P3GoalP4 4
#define N 9999

void stepDebug(std::string info)
{
    std::cout << info << "  please input any number" << std::endl;
    int a;
    std::cin >> a;
    if(a==0)
        exit(0);
}

bool FileRead(const string &path, vector<vector<double>> &data)
{
    ifstream file(path);
    if (!file.is_open())
    {
        cout << "Failed to open the file." << endl;
        return false;
    }
    string line;
    while (getline(file, line))
    {
        istringstream iss(line);
        double value;
        vector<double> row;

        while (iss >> value)
        {
            row.push_back(value);
        }
        data.push_back(row);
    }
    file.close();
    // 打印读取的数据
    #ifdef DEBUG_OUTPUT
        for (const auto &row : data)
        {
            for (const auto &value : row)
            {
                cout << value << " ";
            }
            cout << endl;
        }
    #endif
    return true;
}

void vector_print(const std::vector<double> &v,std::string filename)
{
    std::ofstream fout;
    for(int i=0;i<v.size();i++)
    {
        fout.open(filename,ios::app);
        fout<<v.at(i)<<endl;
        fout.close();
    }
}

void vector_vector_print(const std::vector<std::vector<double>> &v,std::string filename)
{
    std::ofstream fout;
    for(int i=0;i<v.size();i++)
    {
        fout.open(filename,ios::app);
        for(int j=0;j<v.at(i).size();j++)
        {
            fout<<v.at(i).at(j)<<" ";
        }
        fout<<std::endl;
        fout.close();
    }
}

void ee_pose_print(const vector<geometry_msgs::Pose> &pose,std::string filename)
{
    std::ofstream fout;
    fout.open(filename,ios::app);
    for(auto it=pose.begin();it!=pose.end();it++)
    {
        fout<<it->orientation.x<<" "<<it->orientation.y<<" "<<it->orientation.z<<" "<<it->orientation.w<<" "
        <<it->position.x<<" "<<it->position.y<<" "<<it->position.z<<" \n";
    }
    fout<<std::endl;
    fout.close();
}

void file_clear(std::string filename)
{
    std::ofstream fout;
    fout.open(filename);
    fout.clear();
    fout.close();
}

class labor_fabrik
{
private:
    ros::NodeHandle nh;
    Eigen::Vector3f pw; //point_waist
    Eigen::Vector3f pf; //point_foot
    Eigen::Vector3f po; //point_chest
    Eigen::Vector3f p1; //point_shoulder
    Eigen::Vector3f p2; //point_elbow
    Eigen::Vector3f p3; //point_wrist
    Eigen::Vector3f p4; //point_gripper
    Eigen::Vector3f p1_start;   //肩部初始位置
    Eigen::Vector3f p2_start;   //肘部初始位置
    Eigen::Vector3f p3_start;   //手腕初始位置
    Eigen::Matrix4f TgoalB; //目标位姿
    Eigen::Vector3f p3_goal;    //手腕目标位置
    visualization_msgs::Marker box1_marker;
    visualization_msgs::Marker box2_marker;
    visualization_msgs::Marker box3_marker;
    visualization_msgs::Marker body_marker;
    visualization_msgs::Marker waist_marker;
    vector<double> success_time;
    vector<double> success_attempts;
    std::thread *thread_marker;
    int maxFailedAttempts; //最大错误次数
    double car_to_foot = 237;
    double foot_to_ground = 349.18;
    double waist_to_foot = 667.57; // 加动平台厚度 1.0168-0.34918
    double waist_to_chest = 351.5; //351.5, 1.3676-1.0168
    double chest_to_shoulder = 265; //
    double shoulder_to_elbow = 242;
    double elbow_to_wrist = 249-1.5937;
    double wrist_to_gripper = 198.8392;
    double waist_to_ground = foot_to_ground + waist_to_foot; 
    double chest_to_ground = waist_to_ground + waist_to_chest;
    double d1 = chest_to_shoulder; // 胸部中心与肩部的距离
    double d2 = shoulder_to_elbow; // 肩部与肘部的距离
    double d3 = elbow_to_wrist; // 肘部与手爪中心的距离
    vector<Eigen::Vector3f> sphere_obs;
    visualization_msgs::MarkerArray sphere_obs_marker;//虚拟障碍物marker数组
    double sphere_radius=0.01;//虚拟障碍物半径
    ros::Publisher marker_pub;
    ros::Publisher marker_array_pub;
    ros::Publisher right_arm_plan_req;
    // fcl::Box<float> box_obs(marker.scale.x,marker.scale.y,marker.scale.z);
    fcl::Box<float> box1;
    fcl::Box<float> box2;
    fcl::Box<float> box3;
    // fcl::Cylinder<float> *cylinder1;
    // fcl::Cylinder<float> *cylinder2;
    // fcl::Cylinder<float> *cylinder3;
    fcl::Box<float> body;
    
    CollisionRequest<float> request;
    CollisionResult<float> result;
    OcTree<float>* tree1;
    octomap::OcTree *tree; // 设置八叉树分辨率，这里设置为3mm
    std::shared_ptr<fcl::CollisionGeometryf> shared_tree;
    CollisionObject<float> *tree_obj;
    CollisionObject<float> *box_obj1;
    CollisionObject<float> *box_obj2;
    CollisionObject<float> *box_obj3;
    CollisionObject<float> *box_obj_body;
    sensor_msgs::PointCloud2 self_pointcloud;
    ros::Publisher self_pointcloud_pub;
    // LaborPlan *plan_;
    LaborKinematics LK;
    vector<double> config;
    double theta6;
public:

    labor_fabrik();
    ~labor_fabrik();
    Eigen::Matrix4f Tend;

    //超参数设置
    int segment;
    double epsilon;
    int little_count;
    double rand_range;
    int max_count;
    double eps;
    int pc_use;
    int show;
    int test_count;
    //功能函数
    void setConfig(); //从外部文件获取参数
    void box_update(); //更新碰撞盒位置
    void pub_marker(); //发布碰撞盒marker
    void pub_p1p2p3p4(); //发布关键点marker
    void thread_marker_CB(); //发布marker线程回调函数
    void MultiTest(int test_count,int &successs_count); //多次测试
    void LeftArmMultiTest(int count,int &success_count); //左臂多次测试
    void show_points(const std::string &valid_filename,const std::string &invalid_filename); //显示成功与失败点
    void body_collision_test(); //测试躯干碰撞
    void pubEndEffectorMarker();//发布末端执行器中心点的marker
    bool set_p3_goal(); //设置手腕目标位置
    bool setEndEffectorDesiredPose(Eigen::Vector3f apple_position); //设置末端执行器目标位姿
    bool isCollision(int index); //碰撞检测
    bool solve(int maxFailedAttempts, double eps, const Eigen::Matrix4f &pose_matrix, int &iter_count); //求解函数
    bool solveSelfRotation(int maxFailedAttempts, double eps, const Eigen::Matrix4f &T13G, int &iter_count); //自旋求解函数
    float distanceCost3(const Eigen::Vector3f &p1,Eigen::Vector3f &p2); //距离函数
    float rand01(); //生成0-1随机数
    double ConfigCalculate(Eigen::Vector3f p1,Eigen::Vector3f p2,Eigen::Vector3f p3,Eigen::Vector3f p4); //计算当前构型对应的关节角度
    
    Eigen::Matrix3f rotate_x(float rad); //绕x轴旋转矩阵
    Eigen::Matrix3f rotate_y(float rad); //绕y轴旋转矩阵
    Eigen::Matrix3f rotate_z(float rad); //绕z轴旋转矩阵
    
    Eigen::Matrix4f calculate_T1G(const double& theta8, const double& theta9); //计算T9G
    Eigen::Matrix4f calculate_T2G(const double& theta8, const double& theta9, const double& theta10); //计算T10G
    Eigen::Matrix4f calculate_T3G(const double& theta8, const double& theta9, const double& theta10, 
                                    const double& theta11); //计算T11G
    Eigen::Matrix4f calculate_T4G(const double& theta8, const double& theta9, const double& theta10, 
                                    const double& theta11, const double& theta12); //计算T12G
    Eigen::Matrix4f calculate_T5G(const double& theta8, const double& theta9, const double& theta10, 
                                    const double& theta11, const double& theta12, const double& theta13); //计算T13G
    Eigen::Vector2f pivot_hinge_solve(const double& pstart_x, const double& pstart_y, const double& pstart_z,
                                        const double& pmid_x, const double& pmid_y, const double& pmid_z,
                                        const double& pend_x, const double& pend_y, const double& pend_z,
                                        const double& pend_pre_x, const double& pend_pre_y, const double& pend_pre_z); //枢轴铰链求解
    Eigen::Vector2f calculate_dabi_zhoubu_verify(
        Eigen::Vector3f p_wrist, double theta_wanyao, double theta_ceyao,
        double theta_jiaobu, double theta_qianhoutai, double theta_cetai,
        double theta_dabi, double theta_zhoubu); //计算大臂肘部验证正负
    Eigen::Vector2f calculate_qianhoutai_cetai_verify(
        Eigen::Vector3f p3, double theta_wanyao, double theta_ceyao,
        double theta_jiaobu, double theta_qianhoutai, double theta_cetai); //计算前后抬侧抬验证正负
    Eigen::Vector3f calculate_xiaobi_shouwan_verify(Eigen::Vector3f p4,
                                                 double theta_qianhoutai,
                                                 double theta_cetai,
                                                 double theta_dabi,
                                                 double theta_zhoubu,
                                                 double theta_xiaobi,
                                                 double theta_shouwan); //计算小臂手腕验证正负
};