#ifndef _CONTROL_VAR_H_
#define _CONTROL_VAR_H
#include <ros/ros.h>
#include <vector>
using namespace std;


class Control_Var
{
public:
    Control_Var(vector<float> &aim_node);
    float speed_var(vector<float> &robo_pose);
    float yaw_var(vector<float> &robo_pose);
    
private:
    float aim_node_x;
    float aim_node_y;
    float aim_node_yaw;
};

#endif
