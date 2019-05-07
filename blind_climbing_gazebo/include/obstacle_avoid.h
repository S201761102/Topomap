#ifndef _OBSTACLE_AVOID_H
#define _OBSTACLE_AVOID_H
#include <vector>
#include <map>

class Obstacle_Avoidance
{
public:
    bool exist_obstacle(std::map<float,float> &angle_dis);
};

#endif
