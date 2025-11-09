#include <ph_ik/ph_ik_single6dof.h>

void solve()
{
    labor_fabrik fabrik;
    int count=0;
    /*urdf与模型不同的是：侧抬，小臂是相反数*/
    Eigen::Matrix4f end_link_pose_matrix=fabrik.calculate_T5G(60.0*M_PI/180.0,-5.0*M_PI/180.0,56.0*M_PI/180.0,62.0*M_PI/180.0,-0.2345,0.321); //抓苹果
    int iter_count=0;
    double elapsed_secs;
    clock_t end;
    clock_t start = clock();
    if(fabrik.solve(fabrik.max_count,fabrik.eps,end_link_pose_matrix,iter_count))
    {
        count++;
        end = clock();
        elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
        ROS_INFO("Time taken to solve: %.6f seconds", elapsed_secs);
    }
    
    end_link_pose_matrix=fabrik.calculate_T5G(65.0*M_PI/180.0,-5.0*M_PI/180.0,56.0*M_PI/180.0,62.0*M_PI/180.0,-0.2345,0.321);
    start = clock();
    if(fabrik.solve(fabrik.max_count,fabrik.eps,end_link_pose_matrix,iter_count))
    {
        count++;
        end = clock();
        elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
        ROS_INFO("Time taken to solve: %.6f seconds", elapsed_secs);
    }

    end_link_pose_matrix=fabrik.calculate_T5G(60.0*M_PI/180.0,-15.0*M_PI/180.0,86.0*M_PI/180.0,62.0*M_PI/180.0,0.52,0.236);
    start = clock();
    if(fabrik.solve(fabrik.max_count,fabrik.eps,end_link_pose_matrix,iter_count))
    {
        count++;
        end = clock();
        elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
        ROS_INFO("Time taken to solve: %.6f seconds", elapsed_secs);
    }

    end_link_pose_matrix=fabrik.calculate_T5G(86.0*M_PI/180.0,-25.0*M_PI/180.0,34.0*M_PI/180.0,72.0*M_PI/180.0,-0.21,0.132);
    start = clock();
    if(fabrik.solve(fabrik.max_count,fabrik.eps,end_link_pose_matrix,iter_count))
    {
        count++;
        end = clock();
        elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
        ROS_INFO("Time taken to solve: %.6f seconds", elapsed_secs);
    }

    end_link_pose_matrix=fabrik.calculate_T5G(90.0*M_PI/180.0,-35.0*M_PI/180.0,27.0*M_PI/180.0,32.0*M_PI/180.0,0.312,0.051);
    start = clock();
    if(fabrik.solve(fabrik.max_count,fabrik.eps,end_link_pose_matrix,iter_count))
    {
        count++;
        end = clock();
        elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
        ROS_INFO("Time taken to solve: %.6f seconds", elapsed_secs);
    }
    ROS_INFO("total success count:%d",count);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "right_arm");
    solve();
    return 0; 
}