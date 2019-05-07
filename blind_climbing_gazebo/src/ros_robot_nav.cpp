#include <ros/ros.h>
#include <map>
#include <vector>
#include <ros_robot_nav.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <multisensor_information_fusion/pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <map.h>
#include <obstacle_avoid.h>
#include <control_var.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <unistd.h>
using namespace std;
const float Pi = 3.1415926535898;


Robot_Nav::Robot_Nav()
{
    run_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
    scan_sub = nh.subscribe("/scan", 1, &Robot_Nav::scan_callback,this);
    //pose_sub = nh.subscribe("/pose", 1, &Robot_Nav::pose_callback,this);
    gps_sub = nh.subscribe("/sensor_msgs/NavSatFix", 1, &Robot_Nav::gps_callback,this);
    imu_sub = nh.subscribe("/mobile_base/sensors/imu_data", 1, &Robot_Nav::imu_callback,this);
    robo_pose = vector<float>(3,0);
    sleep(2); //wait 2s
    cout << " robot navigation initial success... " << endl;
}
/*
robot coordinate
about scan_angle
    ^
    |
    |
-   | +
————|————————————————>
    |  
    |
*/
void Robot_Nav::scan_callback(const sensor_msgs::LaserScan& scan_data)
{
   int num_scan = (scan_data.angle_max - scan_data.angle_min) / scan_data.angle_increment + 1;
   for(int i = 0; i < num_scan; ++i)
   {
       if(isnan(scan_data.ranges[i]) == false)
       {
           float angle = scan_data.range_min + i * scan_data.angle_increment;
           float dis = scan_data.ranges[i];
           scan_angle_dis[angle] = dis;
       }
       else
       {
           float angle = scan_data.range_min + i * scan_data.angle_increment;
           float dis = 100;
           scan_angle_dis[angle] = dis;
       }

   } 
}

/*
#pose: x:m
       y:m
       yaw:rad
Global coordinate 
about yaw:
    ^
    |
    |
    |  0-pi
————|————————————————>
    |  -pi-0
    |
*/
void Robot_Nav::pose_callback(const multisensor_information_fusion::pose& pose_data)
{
    robo_pose[0] = pose_data.x;
    robo_pose[1] = pose_data.y;
    float yaw;
    if(pose_data.psi > Pi && pose_data.psi <= 2 * Pi)
        yaw = pose_data.psi - 2 * Pi;
    if(pose_data.psi >= 0 && pose_data.psi <= Pi)
        yaw = pose_data.psi;
    if(pose_data.psi >= 2 * Pi) yaw = 0;
    robo_pose[2] = yaw;
}

void Robot_Nav::gps_callback(const sensor_msgs::NavSatFix& gps_data)
{
    robo_pose[0] = (gps_data.latitude - 49.9) * 111000;
    robo_pose[1] = -(gps_data.longitude - 8.9) * 71950;
}

void Robot_Nav::imu_callback(const sensor_msgs::Imu& imu_data)
{
    tf::Quaternion bq(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
    double roll,pitch,yaw;
    tf::Matrix3x3(bq).getRPY(roll, pitch, yaw);
    robo_pose[2] = yaw;
}

bool Robot_Nav::reach_node(vector<float> &aim_node)
{
    float dis;
    dis = pow(aim_node[0] - robo_pose[0],2) + pow(aim_node[1] - robo_pose[1],2);
    if( dis >= 0.01)  return false;
    return true;
}

/*
#robot control
speed_u:+ forward
yaw_u:+ anticlockwise
*/
bool Robot_Nav::run_node(vector<float> &aim_node)
{
    geometry_msgs::Twist vel;
    Obstacle_Avoidance oa;
    Control_Var cv(aim_node);
    //ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        if(reach_node(aim_node))  
        {
            cout << "break! reach the node" << endl;
            break;
        }
        if(oa.exist_obstacle(scan_angle_dis) == false) 
        {
            cout << "break!there are obstacles!" << endl;
            return false;
        }
        float speed_u, yaw_u;
        speed_u = cv.speed_var(robo_pose);
        yaw_u = cv.yaw_var(robo_pose);
        //cout <<"speed_yaw:" <<speed_u <<" " << yaw_u  <<endl;
        //cout <<"robot_pose:" <<robo_pose[0] << " " << robo_pose[1]<<" " <<robo_pose[2] << endl;
        vel.linear.x = speed_u;
        vel.angular.z =  yaw_u;
        run_pub.publish(vel);
        //loop_rate.sleep();
        usleep(20000);
    }
    return true;
}

bool Robot_Nav::turn_node(vector<float> &aim_node)
{
    geometry_msgs::Twist vel;
    Control_Var cv(aim_node);
    while(ros::ok())
    {
        ros::spinOnce();
        if(reach_node(aim_node))  
        {
            cout <<"break! reach the node" << endl;
            break;
        }
        float speed_u, yaw_u;
        yaw_u = cv.yaw_var(robo_pose);
        //cout <<"yaw:" << yaw_u  <<endl;
        //cout <<"robot_pose:" <<robo_pose[0] << " " << robo_pose[1]<<" " <<robo_pose[2] << endl;
        vel.angular.z =  yaw_u;
        if(fabs(yaw_u) < 0.01) break;
        run_pub.publish(vel);
        //loop_rate.sleep();
        usleep(20000);
    }
    return true;
}

bool Robot_Nav::climb_goal(vector<float> &goal_pose)
{
    Map gm(100, 100, 0.5, -25.25, -25.25);
    int local_min = 1, local_max = 1;
    //ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        vector<vector<int> > local_node_index;
        vector<vector<int> > local_node_index_filter;
        local_node_index = gm.square_region_node(local_min, local_max, robo_pose);
        local_node_index_filter = gm.filter_node(local_node_index);
        cout << "local_node_num:" << local_node_index_filter[0].size() << endl;
        vector<float> op_node;
        op_node = gm.optimal_node(local_node_index_filter, goal_pose);
        cout <<"optimal:" <<op_node[0] <<" " <<op_node[1] << endl;
        bool turn = turn_node(op_node);
        bool reach = run_node(op_node);
        if(reach) 
        {
            cout << "reach the optimal node:"  <<op_node[0] <<" " <<op_node[1] <<  endl;
            gm.add_history_node(op_node);
        }
        else
        {
            cout << "can't reach!!"  << endl;
            gm.add_unreach_node(op_node);
        }
        usleep(20000);
       //loop_rate.sleep();
        if(reach_node(goal_pose)) break;
    }
    return true;
}



