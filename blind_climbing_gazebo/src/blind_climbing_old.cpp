#include <ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<multisensor_information_fusion/pose.h>
#include<unistd.h> //暂停函数，把进程挂起一段时间

using namespace std;

//全局变量区域
float yaw;//机器人朝向
const float Pi = 3.1415926535898;
float x = 0;
float y = 0;
float psi = 0;
float dx = 0;
float dpsi = 0;


void RobotPos(const multisensor_information_fusion::pose& pos)
{
  x = pos.x;
  y = pos.y;
  psi = pos.psi;
  if(psi >Pi && psi <= 2 * Pi)   yaw =psi - 2 * Pi ;
  if(psi >=0 && psi <= Pi) yaw = psi;
  if(psi > 2 * Pi) yaw = 0;
  dx = pos.dx;
  dpsi = pos.dpsi;
}

float node_x[100];
float node_y[100];
float line_interval = 0.5;


//build nodes function
void BuildNode( )
{
	
	for(int i = 0;i < 50;++i)
	{
		node_x[i+50] =  line_interval * i;
		node_y[i+50] =  line_interval * i;
		node_x[49-i] = -line_interval - line_interval * i;
		node_y[49-i] = -line_interval - line_interval * i;
	}
}


float scan_ranges[20];
void ScanDataRecieved(const sensor_msgs::LaserScan& ScanData)
{
	for(int i=0;i < 20 ;++i)
	{
		scan_ranges[i] = ScanData.ranges[32 * i];
		if(isnan(scan_ranges[i]) != 0 )  scan_ranges[i] = 100;
		//cout << scan_ranges[i] << "  ";
		//if(i == 19)  cout << endl;
	}
}



int index_nearest_x;
int index_nearest_y;
float dist_min_x;
float dist_min_y;
void FindNearestNode( )
{
	float dist_x[100];
	float dist_y[100];
	for(int i=0;i<100;++i)
	{
		dist_x[i] = fabs(x - node_x[i]);
		dist_y[i] = fabs(y- node_y[i]);
	}
	dist_min_x = dist_x[0];
	dist_min_y = dist_y[0];
	for(int i=0;i<100;++i)
	{
		if(dist_min_x > dist_x[i])
		{
			dist_min_x = dist_x[i];
			index_nearest_x = i;
		}

		if(dist_min_y > dist_y[i])
		{
			dist_min_y = dist_y[i];
			index_nearest_y = i;
		}
	}
}

const float scan_robot_length = 0.083;
float x_scan_origin;
float y_scan_origin;
void GetScanCoordinate()
{
	x_scan_origin = x - scan_robot_length * cos(yaw);
	y_scan_origin = y - scan_robot_length * sin(yaw);
}


float local_radius_min = 0.5;
float local_radius_max = 1;
vector<int> radius_max_min_x;
vector<int> radius_max_min_y;
int num_min_max_node;
int intj = 0;
vector<int> UnableReachNodeX;
vector<int> UnableReachNodeY;
int numUnableReachNode;
int numUnableReachNodeMax = 20;
//.3  find the circle node; shrinking number of nodes in local_radius_min and  local_radius_max

void FindCircleNode()
{
	numUnableReachNode = UnableReachNodeX.size();
	if(numUnableReachNode > 1)
	{
	  if(UnableReachNodeX[numUnableReachNode - 1] == UnableReachNodeX[numUnableReachNode - 2] && UnableReachNodeY[numUnableReachNode - 1] == UnableReachNodeY[numUnableReachNode - 2])
	  {
	    local_radius_max+=0.5;
	    local_radius_min+=0.5;
	  }
	  else
	  {
	    local_radius_max-=0.5;
	    local_radius_min-=0.5;
	    if(local_radius_min<=0.5)   local_radius_min=0.5;
	    if(local_radius_max<=1)  local_radius_max=1;
	  }
	}
	
	for(int i=0;i<local_radius_max/line_interval + 1;++i)
		for(int j=0;j<local_radius_max/line_interval+1;++j)
		{
			int mark1 = 0;
			if(pow((node_x[index_nearest_x+i]-x_scan_origin),2)+pow((node_y[index_nearest_y+j]-y_scan_origin),2) <= pow(local_radius_max,2) && pow((node_x[index_nearest_x+i]-x_scan_origin),2)+pow((node_y[index_nearest_y+j]-y_scan_origin),2) >= pow(local_radius_min,2))
			{
			  //cout <<"发现与不可达区域一致" <<index_nearest_x + i<<"    "<<index_nearest_y +j << endl;
			  for(int ij = 0;ij < numUnableReachNode;++ij)
			  {
			    if(((index_nearest_x + i) == UnableReachNodeX[ij]) && index_nearest_y + j == UnableReachNodeY[ij])
			    {
			     // cout <<"发现与不可达区域一致" <<index_nearest_x + i<<"    "<<index_nearest_y +j << endl;
			      mark1++;
			      //break;
			    }
			  }
			  if(mark1 == 0)
			  {
				radius_max_min_x.push_back(index_nearest_x+i);
				radius_max_min_y.push_back(index_nearest_y+j);
				//cout << index_nearest_x + i <<"   " <<index_nearest_y+j
			  }
			}
			if(pow((node_x[index_nearest_x-i-1]-x_scan_origin),2)+pow((node_y[index_nearest_y+j]-y_scan_origin),2) <= pow(local_radius_max,2) && pow((node_x[index_nearest_x-i-1]-x_scan_origin),2)+pow((node_y[index_nearest_y+j]-y_scan_origin),2) >= pow(local_radius_min,2))
			{
			   for(int ij = 0;ij < numUnableReachNode;++ij)
			  {
			    if(index_nearest_x - i - 1 == UnableReachNodeX[ij] && index_nearest_y + j==UnableReachNodeY[ij])
			    {
			       //cout <<"发现与不可达区域一致" <<index_nearest_x + i<<"    "<<index_nearest_y +j << endl;
			      mark1 = 1;
			     // break;
			    }
			      
			  }
			  if(mark1 == 0)
			  {
				radius_max_min_x.push_back(index_nearest_x-i-1);
				radius_max_min_y.push_back(index_nearest_y+j);
			  }
			}
			if(pow((node_x[index_nearest_x-i-1]-x_scan_origin),2)+pow((node_y[index_nearest_y-j-1]-y_scan_origin),2) <= pow(local_radius_max,2) && pow((node_x[index_nearest_x-i-1]-x_scan_origin),2)+pow((node_y[index_nearest_y-j-1]-y_scan_origin),2) >= pow(local_radius_min,2))
			{
			  for(int ij = 0;ij < numUnableReachNode;++ij)
			  {
			    if(index_nearest_x - i -1 == UnableReachNodeX[ij] && index_nearest_y - j - 1==UnableReachNodeY[ij])
			    {
			      //cout <<"发现与不可达区域一致" <<index_nearest_x + i<<"    "<<index_nearest_y +j << endl;
			      mark1 = 1;
			      //break;
			    }
			  }
			  if(mark1 == 0)
			  {
			    radius_max_min_x.push_back(index_nearest_x-i-1);
			    radius_max_min_y.push_back(index_nearest_y-j-1);
			  }
			}
			if(pow((node_x[index_nearest_x+i]-x_scan_origin),2)+pow((node_y[index_nearest_y-j-1]-y_scan_origin),2) <= pow(local_radius_max,2) && pow((node_x[index_nearest_x+i]-x_scan_origin),2)+pow((node_y[index_nearest_y-j-1]-y_scan_origin),2) >= pow(local_radius_min,2))
			{
			   for(int ij = 0;ij < numUnableReachNode;++ij)
			  {
			    if(index_nearest_x + i == UnableReachNodeX[ij] && index_nearest_y-j-1 == UnableReachNodeY[ij])
			    {
			     //cout <<"发现与不可达区域一致" <<index_nearest_x + i<<"    "<<index_nearest_y +j << endl;
			      mark1 = 1;
			      //break;
			    }
			  }
			  if(mark1 == 0)
			  {
				radius_max_min_x.push_back(index_nearest_x+i);
				radius_max_min_y.push_back(index_nearest_y-j-1);
			  }
			}
		}
	num_min_max_node = radius_max_min_x.size();
}

float scan_min = 0.45;//minmum  value:0.45m
float scan_max = 1;//maxmum value:5m
vector<float> dist_scan_node_aim;
//记录机器人已经到达的节点
vector<int>  RobotReachNodeX;
vector<int>  RobotReachNodeY;
float min_scan_node_aim;
int index_min_scan_node_aim; 
float CurAimX;
float CurAimY;
float lastCurAimX;
float lastCurAimY;


void FindOptimalNode(float aim_x, float aim_y)
{
	lastCurAimX = CurAimX;
	lastCurAimY = CurAimY;
	if(pow(aim_x - x,2) + pow(aim_y - y,2) >= pow(scan_max , 2))
	{
		for(int i = 0;i < num_min_max_node;++i)
		{
			dist_scan_node_aim.push_back(pow(node_x[radius_max_min_x[i]] - aim_x,2) + pow(node_y[radius_max_min_y[i]] - aim_y,2));
		}
		min_scan_node_aim = dist_scan_node_aim[0];
		index_min_scan_node_aim = 0;
		for(int i = 0;i < num_min_max_node;++i)
		{
			if(dist_scan_node_aim[i] < min_scan_node_aim)
			{
				index_min_scan_node_aim = i;
				min_scan_node_aim = dist_scan_node_aim[i];
			}
		}
		CurAimX =node_x[radius_max_min_x[index_min_scan_node_aim]];
		CurAimY = node_y[radius_max_min_y[index_min_scan_node_aim]];
	}
	if(pow(aim_x - x,2) + pow(aim_y - y,2) < pow(scan_max , 2))
	{
	  	for(int i = 0;i < num_min_max_node;++i)
		{
			dist_scan_node_aim.push_back(pow(node_x[radius_max_min_x[i]] - aim_x,2) + pow(node_y[radius_max_min_y[i]] - aim_y,2));

		}
		CurAimX =aim_x;
		CurAimY = aim_y;
	}
}



void CoutToScreen(void)
{
	cout <<"-------------------cout  to screen---------------------------"<<endl;
	cout << "Robot current position: " << "x : "<< x << "  y : " << y <<" yaw : "<<yaw<<endl;
	cout << "激光传感器原点："  << "x : " <<x_scan_origin <<" y :  " << y_scan_origin << endl;
	cout << "激光传感器角度与测量的长度："<< endl;
	//for(int i = 0; i < 20; ++i)
	//{
		//cout <<"                   序号： "<< i<<"   角度："<<scan_angle[i]*180/Pi <<"   测量长度："<<scan_ranges[i] <<endl;
	//}  
	cout <<"扇形区域节点角度："<<endl;
	//for(int i = 0; i < num_sector; ++i)
	//{
		//cout <<"                   节点序号： "<< sector_x[i] <<" "<<sector_y[i]<<"   节点坐标："<< node_x[sector_x[i]] <<" "<<node_y[sector_y[i]]<<"  节点角度："  << sectorAngle[i] * 180 / Pi <<endl;
	//}  
	cout <<"环形区域节点个数："<< num_min_max_node << endl;
	cout<<"当前局部目标点"<<"x: "<<CurAimX<<"   y: "<<CurAimY<<endl;
	//if(num_scan == 0)
	//{
		//cout << " no scan's node " << endl;
		//cout << "starting turning!" << endl;
		//fflush(stdout);
	//}
	//else
	//{
		for(int i=0;i<num_min_max_node;++i)
		{
			//cout<<"04"<<"num_scan: "<<num_scan<<endl;
			cout<<radius_max_min_x[i]<<"   "<<radius_max_min_y[i]<<"  距离目标点距离: "<<dist_scan_node_aim[i]<<endl;
			//cout << scan_x[i] << "  "<< scan_y[i]<<"  f(x): "<<dist_scan_node_aim[i]<< "  which scan: "<<scan_min_index[i]<<"  length: " <<scan_ranges[scan_min_index[i]]<<endl;
		  
		}	
		cout <<"机器人不可到达的节点："<<endl;
		for(int i=0;i<numUnableReachNode;++i)
		{
		  cout << UnableReachNodeX[i] << "   " <<UnableReachNodeY[i] <<" 坐标：" <<node_x[UnableReachNodeX[i]]<<"   "<<node_y[UnableReachNodeY[i]]<<endl;
		}
	//}

	cout << endl;  
	cout <<"-------------------end---------------------------"<<endl;
}


//定义函数，让机器人指向目标点
void PointToTarget(float aim_x,float aim_y)
{
	ros::NodeHandle nh;
	ros::Publisher pub_run = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1000);
	geometry_msgs::Twist run;
	float erro_angle  = 3.14;
	//cout << "start point to target!"<< endl;
	//cout << "warting"<<endl;
	//ros::Rate rate(50);
	while(fabs(erro_angle) * 180/Pi>= 3)
	{
		//cout << "." << " " ;
		fflush(stdout);
		usleep(100000);//100ms
		ros::spinOnce();
		if( yaw > -Pi/2 && yaw < Pi/2)
		{
			if (tan(yaw) * (aim_x - x) + y < aim_y)
			{
				erro_angle = acos((aim_x-x + tan(yaw) * (aim_y - y)) / ( sqrt( pow(aim_x - x , 2)  + pow( aim_y - y , 2)) * sqrt( 1  + pow( tan(yaw) , 2))));
			}
			if (tan(yaw) * (aim_x - x) + y > aim_y)
			{
				erro_angle = -acos((aim_x-x + tan(yaw) * (aim_y - y)) / ( sqrt( pow(aim_x - x , 2)  + pow( aim_y - y , 2)) * sqrt( 1  + pow( tan(yaw) , 2))));
			}
		}
		if( (yaw < -Pi/2 && yaw > -Pi) || (yaw > Pi/2 && yaw <Pi))
		{
			if (tan(yaw) * (aim_x - x) + y < aim_y)
			{
				erro_angle = -acos((-aim_x+x - tan(yaw) * (aim_y - y)) / ( sqrt( pow(aim_x - x , 2)  + pow( aim_y - y , 2)) * sqrt( 1  + pow( tan(yaw) , 2))));
			}
			if (tan(yaw) * (aim_x - x) + y > aim_y)
			{
				erro_angle = acos((-aim_x+x - tan(yaw) * (aim_y - y)) / ( sqrt( pow(aim_x - x , 2)  + pow( aim_y - y , 2)) * sqrt( 1  + pow( tan(yaw) , 2))));
			}
		}
		//cout <<erro_angle <<endl;
		run.angular.z = 1*erro_angle;
		//run.linear.x = speed;
		pub_run.publish(run);
		//rate.sleep();
	}
	//cout<<endl;
}

double NextPointAngleControl(double aim_x,double aim_y)
{
	double erro_angle;
	if( yaw > -Pi/2 && yaw < Pi/2)
	{
		if (tan(yaw) * (aim_x - x) + y < aim_y)
		{
			erro_angle = acos((aim_x-x + tan(yaw) * (aim_y - y)) / ( sqrt( pow(aim_x - x , 2)  + pow( aim_y - y , 2)) * sqrt( 1  + pow( tan(yaw) , 2))));
		}
		if (tan(yaw) * (aim_x - x) + y > aim_y)
		{
			erro_angle = -acos((aim_x-x + tan(yaw) * (aim_y - y)) / ( sqrt( pow(aim_x - x , 2)  + pow( aim_y - y , 2)) * sqrt( 1  + pow( tan(yaw) , 2))));
		}
	}

	if( (yaw < -Pi/2 && yaw > -Pi) || (yaw > Pi/2 && yaw <Pi))
	{
		if (tan(yaw) * (aim_x - x) + y < aim_y)
		{
			erro_angle = -acos((-aim_x+x - tan(yaw) * (aim_y - y)) / ( sqrt( pow(aim_x - x , 2)  + pow( aim_y - y , 2)) * sqrt( 1  + pow( tan(yaw) , 2))));
		}
		if (tan(yaw) * (aim_x - x) + y > aim_y)
		{
			erro_angle = acos((-aim_x+x - tan(yaw) * (aim_y - y)) / ( sqrt( pow(aim_x - x , 2)  + pow( aim_y - y , 2)) * sqrt( 1  + pow( tan(yaw) , 2))));
		}
	}
	return erro_angle;
} 

void DeleVector()
{
	for(int i=0;i<num_min_max_node;++i)
	{
		radius_max_min_x.pop_back();
		radius_max_min_y.pop_back();
	}
// 	for(int i=0;i<num_sector;++i)
// 	{
// 		sector_x.pop_back();
// 		sector_y.pop_back();
// 		sectorAngle.pop_back();
// 	}
 	for(int i=0;i<num_min_max_node;++i)
 	{
// 		scan_x.pop_back();
// 		scan_y.pop_back();
 		dist_scan_node_aim.pop_back();
// 		scan_min_index.pop_back();
 	}	

}


int main(int argc ,char** argv)
{
  ros::init(argc,argv,"uwbblindclimbing");
  ros::NodeHandle nh;
  //ros::Subscriber sub = nh.subscribe("/head",1,&RobotHead);
  ros::Publisher pub_run = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1000);
  ros::Subscriber sub_scan = nh.subscribe("/scan",1,&ScanDataRecieved);
  //ros::Subscriber sub_imu = nh.subscribe("/mobile_base/sensors/imu_data",1,&ImuDataRecieved);
  ros::Subscriber sub_pos = nh.subscribe("/pos",1,&RobotPos);
  geometry_msgs::Twist run;
 
  cout << "Please input the aim position(like:x  y ):";
  float Aim[2];
  for(int i = 0;i < 2;++i)
  {
    cin >> Aim[i];
  }
  PointToTarget(Aim[0],Aim[1]);
  
  BuildNode();
 
  CurAimX = x;//Aim[0];
  CurAimY = y;//Aim[1];
  cout << " 开始走向目标  " <<endl;
  
  
  while(ros::ok() && pow(Aim[0]- x,2) + pow(Aim[1] - y,2) >= 0.01)
  {
    ros::spinOnce();
    FindNearestNode();
    GetScanCoordinate();
    FindCircleNode();
    FindOptimalNode(Aim[0],Aim[1]);
    CoutToScreen();
    //cout << CurAimX << "   " << CurAimY<< endl;
    cout << "开始进入内部循环！" << pow(CurAimX- x,2) + pow(CurAimY - y,2) <<"  "<< x << " "<<y << "  "<< CurAimX<<"  "<<CurAimY<<endl;
    PointToTarget(CurAimX,CurAimY);
    UnableReachNodeX.push_back(radius_max_min_x[index_min_scan_node_aim]);
    UnableReachNodeY.push_back(radius_max_min_y[index_min_scan_node_aim]);
   
    while(pow(CurAimX- x,2) + pow(CurAimY - y,2) >= 0.01)
    {
	ros::spinOnce();
	int intj = 0;
	for(int inti = 0;inti < 20; ++inti)
	{
		
		 if(scan_ranges[inti] < 0.5)
		 {
			    intj = 1;
			    //将不可到达的节点记录在动态矩阵中
			    cout <<"距离太小，将重新进行规划" << endl;
			    break;
		 }
		 
	}
	if(intj == 1)  break;
	
	float erro_angle = NextPointAngleControl(CurAimX,CurAimY);
	float speed = 0.2;
	run.angular.z = 2*erro_angle;
	run.linear.x = speed;
	
	//cout << speed << "   "  << erro_angle << endl;
	pub_run.publish(run);
	usleep(20000);//20ms
    }


    DeleVector();
  }
}
