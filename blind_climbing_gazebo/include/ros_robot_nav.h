#ifndef _ROS_ROBOT_NAV_H
#define _ROS_ROBOT_NVA_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <multisensor_information_fusion/pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <map>
#include <vector>
using namespace std;

class Robot_Nav
{
public:
    Robot_Nav();
    void scan_callback(const sensor_msgs::LaserScan& scan);
    void pose_callback(const multisensor_information_fusion::pose& pose);
    void gps_callback(const sensor_msgs::NavSatFix& gps_data);
    void imu_callback(const sensor_msgs::Imu& imu_data);
    bool run_node(vector<float> &aim_node);
    bool turn_node(vector<float> &aim_node);
    bool reach_node(vector<float> &aim_node);
    bool climb_goal(vector<float> &goal_pose);

private:
    map<float,float> scan_angle_dis;
    vector<float> robo_pose;
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Publisher run_pub;
};


#endif
