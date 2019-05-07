#include <iostream>
#include "obstacle_avoid.h"
#include <vector>
#include <map>
const float OBSTACLE_DISTANCE = 0.6;

bool Obstacle_Avoidance::exist_obstacle(std::map<float, float> &angle_dis)
{
    for(std::map<float, float>::iterator i = angle_dis.begin(); i != angle_dis.end(); ++i)
    {
        //std::cout << i->second << std::endl;
        if( i->second < OBSTACLE_DISTANCE) return false;  
    }
    return true;
}
