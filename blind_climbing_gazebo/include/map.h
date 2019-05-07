#ifndef _MAP_H
#define _MAP_H
#include <vector>
#include <map>
using namespace std;

/*
   m * n
   m:number of row  n:number of col
   -------------
   |*|*|*|*|*|*|
   -------------
   |*|*|*|*|*|*|  value = 0:initial status 
   -------------  value = 1:sign of history optimal node 
   |*|*|*|*|*|*|  value = 2:sign of unreach_node
   -------------
   x_start y_start
*/

class Map
{
public:
    Map(int m, int n, float i, float x_start, float y_start);
    map<int, float> build_index_val(float &start, int &num, float &i);
    map<float, int> build_val_index(float &start, int &num, float &i);
    int nearest_node_index(float &val, map<float, int> &val_index);
    vector<vector<int> > square_region_node(int &min, int &max, vector<float> &robo_pose);
    vector<vector<int> > circle_region_node(float &min, float &max, vector<float> &robo_pose);
    float evaluation_fun(float &x, float &y, float &aim_x, float &aim_y); 
    vector<vector<int> > filter_node(vector<vector<int> > &region_node);
    vector<float> optimal_node(vector<vector<int> > &node, vector<float> &goal_pose);
    void add_unreach_node(vector<float> &node_val);
    void add_history_node(vector<float> &node_val);

private:
    vector<vector<int> > grid_map;
    map<int, float> x_index_val;
    map<int, float> y_index_val;
    map<float, int> x_val_index;
    map<float, int> y_val_index;
    int grid_row;
    int grid_col;
    float grid_inter;
   };

#endif
