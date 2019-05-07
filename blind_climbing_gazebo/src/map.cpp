#include <iostream>
#include <map.h>
#include <vector>
#include <math.h>
using namespace std;

Map::Map(int m, int n, float i, float x_start, float y_start)
{
    grid_row = m;
    grid_col = n;
    grid_inter = i;
    grid_map = vector<vector<int> > (m, vector<int>(n,0));
    x_index_val = build_index_val(x_start, n, i);
    y_index_val = build_index_val(y_start, m, i);
    x_val_index = build_val_index(x_start, n, i);
    y_val_index = build_val_index(y_start, m ,i);
    cout <<"build empty map sucess... ..."<< endl;
}

map<int, float> Map::build_index_val(float &start, int &num, float &i)
{
    map<int, float> index_val;
    for(int j = 0;j < num; ++j)
    {
        index_val[j] = start + i / 2 + i * j;  
    }
    return index_val;
}

map<float, int> Map::build_val_index(float &start, int &num, float &i)
{
    map<float, int> val_index;
    for(int j = 0;j < num; ++j)
    {
        float val = start + i / 2 + i * j;
        int index = j;
        //cout <<"val:" <<val <<" " << "index:"<< index << endl;
        val_index[start + i/2 + i * j] = j;
    }
    return val_index;
}

int Map::nearest_node_index(float &val, map<float, int> &val_index)
{
    vector<int> index_vec;
    float temp_err = fabs(val - val_index.begin()->first);
    int temp_index = val_index.begin()->second;
    for(map<float, int>::iterator i = val_index.begin(); 
        i != val_index.end(); ++i)
    {
        if( temp_err > fabs(val - i->first))
        {
            temp_err = fabs(val - i->first);
            temp_index = i->second;
        }
    }
    return temp_index;
}
 
    /*
    x[0][0] x[0][1]
    x[1][0] x[1][1]
    ... ...
    x[n][0] x[n][1]
    */

vector<vector<int> > Map::square_region_node(int &min, int &max, vector<float> &robo_pose)
{
    float x = robo_pose[0];
    float y = robo_pose[1];
    int x_index = nearest_node_index(x, x_val_index);
    int y_index = nearest_node_index(y, y_val_index);
    vector<int> x_index_vec;
    vector<int> y_index_vec;
    vector<vector<int> > x_y_index;
    for(int i = x_index - max; i <= x_index + max; ++i)
        for(int j = y_index - max; j <= y_index + max; ++j)
        {
            if(i > x_index - min && i < x_index + min && 
               j > y_index - min && j < y_index + min)
               continue;
            else
            {
                x_index_vec.push_back(i);
                y_index_vec.push_back(j);
            }
        }
    x_y_index.push_back(x_index_vec);
    x_y_index.push_back(y_index_vec);
    return x_y_index;
}

vector<vector<int> > Map::circle_region_node(float &min, float &max, vector<float> &robo_pose)
{
    
}

vector<vector<int> > Map::filter_node(vector<vector<int> > &region_node)
{
    vector<int> x_index_vec;
    vector<int> y_index_vec;
    vector<vector<int> > x_y_index(2);
    for(int i = 0;i < region_node[0].size(); ++i)
    {
        int x_index = region_node[0][i];
        int y_index = region_node[1][i];
        if(grid_map[x_index][y_index] == 0) 
        {
            x_index_vec.push_back(x_index);
            y_index_vec.push_back(y_index);
        }
    }
    x_y_index[0] = x_index_vec;
    x_y_index[1] = y_index_vec;
    return x_y_index;
}

float Map::evaluation_fun(float &x, float &y, float &aim_x, float &aim_y)
{
    float val;
    val = pow(x - aim_x, 2) + pow(y - aim_y, 2);
    return val;
}

vector<float> Map::optimal_node(vector<vector<int> > &node, vector<float> &goal_pose)
{
    float aim_x = goal_pose[0];
    float aim_y = goal_pose[1];
    vector<float> op_node(2);
    float min_val = evaluation_fun(x_index_val[node[0][0]],
                                   y_index_val[node[1][0]],
                                   aim_x,aim_y);
    int x_index = node[0][0], y_index = node[1][0];
    for(int i = 0;i < node[0].size();++i)
    {
        float temp_x = x_index_val[node[0][i]];
        float temp_y = y_index_val[node[1][i]];
        float val = evaluation_fun(temp_x, temp_y, aim_x, aim_y);
        if(val < min_val)
        {
            min_val = val;
            x_index = node[0][i];
            y_index = node[1][i];
        }
    }
    op_node[0] = x_index_val[x_index];
    op_node[1] = y_index_val[y_index];
    return op_node;
}

void Map::add_history_node(vector<float> &node_val)
{
    int x_index = x_val_index[node_val[0]];
    int y_index = y_val_index[node_val[1]];
    grid_map[x_index][y_index] = 1;
}

void Map::add_unreach_node(vector<float> &node_val)
{
    int x_index = x_val_index[node_val[0]];
    int y_index = y_val_index[node_val[1]];
    grid_map[x_index][y_index] = 2;
}



