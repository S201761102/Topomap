#include <control_var.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
using namespace std;

const float Pi = 3.1415926535898;

Control_Var::Control_Var(vector<float> &aim_node)
{
    aim_node_x = aim_node[0];
    aim_node_y = aim_node[1];
    aim_node_yaw = aim_node[2];
} 

float Control_Var::speed_var(vector<float> &robo_pose)
{
    return 0.2; 
}
/*
#use vector angle formula
    x1 = (aim_node_x - robo_x, aim_node_y - robo_y);
    x2 = (1, tan(robo_yaw));? (-1,-tan(robo_yaw))
    angle = (x1_1 * x2_1 + x1_2 * x2_2) / [sqrt(x1_1^2 + x1_2^2) * sqrt(x2_1^2 + x2_2^2)]
*/
float Control_Var::yaw_var(vector<float> &robo_pose)
{
    float robo_x = robo_pose[0];
    float robo_y = robo_pose[1];
    float robo_yaw = robo_pose[2];
    float u = 0;
    float x1_dis = sqrt(pow(aim_node_x - robo_x , 2) + pow(aim_node_y - robo_y , 2));
    float x2_dis = sqrt( 1  + pow( tan(robo_yaw) , 2));
    float x1_x2_mult = aim_node_x - robo_x + (aim_node_y - robo_y) * tan(robo_yaw);
    float raw_err_yaw_1 = acos(x1_x2_mult / (x1_dis * x2_dis));
    float raw_err_yaw_2 = acos(-x1_x2_mult / (x1_dis * x2_dis));

    float err_yaw;
    if( robo_yaw > -Pi/2 && robo_yaw < Pi/2)
    {
        if(tan(robo_yaw) * (aim_node_x - robo_x) + robo_y < aim_node_y)
        {
            err_yaw = raw_err_yaw_1;
        }
        if(tan(robo_yaw) * (aim_node_x - robo_x) + robo_y > aim_node_y)
        {
            err_yaw = -raw_err_yaw_1; 
        }
    }    
 
    if( (robo_yaw < -Pi/2 && robo_yaw > -Pi) || (robo_yaw > Pi/2 
                                                 && robo_yaw <Pi))
    {
        if (tan(robo_yaw) * (aim_node_x - robo_x) + robo_y < aim_node_y)
        {
            err_yaw = -raw_err_yaw_2; 
        }
        if (tan(robo_yaw) * (aim_node_x - robo_x) + robo_y > aim_node_y)
        {
            err_yaw = raw_err_yaw_2; 
        }
    }
    if(isnan(err_yaw) == true) err_yaw = 0;
    u = 2 * err_yaw;
    //cout << " err_yaw " << err_yaw  << endl;
    return u;  
}
   
