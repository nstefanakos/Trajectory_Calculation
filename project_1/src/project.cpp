# include "ros/ros.h"
# include "sensor_msgs/LaserScan.h"
# include "nav_msgs/OccupancyGrid.h"
# include <tf2/LinearMath/Quaternion.h>
# include <tf2/LinearMath/Scalar.h>
# include <tf2_ros/transform_listener.h>
# include <geometry_msgs/TransformStamped.h>
# include <visualization_msgs/MarkerArray.h>
# include <geometry_msgs/PoseArray.h>
# include <cmath>
# include <vector>
# include <map_ray_caster/map_ray_caster.h>

double KALMAN_X(double U)
{
	static const double R = 0.5;
	static const double H = 1.00;
	static double Q = 0.001;
	static double P = 0;
	static double U_old = 0;
	static double K = 0;

	K = P*H/(H*P*H+R);

	U_old = U_old + K*(U-H*U_old);

	P = (1-K*H)*P+Q;

	return U_old;
}

double KALMAN_Y(double U)
{
	static const double R = 1;
	static const double H = 1.00;
	static double Q = 0.001;
	static double P = 0;
	static double U_old = 0;
	static double K = 0;

	K = P*H/(H*P*H+R);

	U_old = U_old + K*(U-H*U_old);

	P = (1-K*H)*P+Q;

	return U_old;
}

int count_1 = 0;
int count_2 = 0;
float max_x = 0;
float min_y = 30;
float max_y = 0;
float old_x;
float old_y;
float f_x;
float f_y;
float vel;
float vel_param;
float orient;

ros::Publisher pub_1;
ros::Publisher pub_2;
ros::Publisher pub_3;
std::vector<float> old_ranges;
ros::Time old_time;
geometry_msgs::PoseArray future;
// map_ray_caster::MapRayCaster ray_caster;
nav_msgs::OccupancyGrid map;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped transformstamped;


void Callback_1(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan new_msg(*msg);

	map.header.frame_id = "/hokuyo_base_laser_link";
	map.header.stamp = ros::Time::now();

	if ( count_1 == 0 )
	{
		map.info.map_load_time = ros::Time::now();
		map.info.resolution = 0.2;
		map.info.width = 80;
		map.info.height = 80;
		map.info.origin.position.x = 8;
		map.info.origin.position.y = 8;
		map.info.origin.position.z = 0;
		map.info.origin.orientation.x = 0.7071066;
		map.info.origin.orientation.y = -0.7071066;
		map.info.origin.orientation.z = 0;
		map.info.origin.orientation.w = 0;
	}

	// map.header.frame_id = "Human";
	

	for (int i = 0 ; i < (map.info.width*map.info.height); i++)
	{
		map.data.push_back(0);
	}


	if (count_1 == 0)
	{
		old_ranges = new_msg.ranges;
		count_1++;
	}

	for (int i = 0 ; i < new_msg.ranges.size(); i++)
	{
		float th = new_msg.angle_min + i * new_msg.angle_increment;
		float x0 = new_msg.ranges[i] * cos(th);
		float y0 = new_msg.ranges[i] * sin(th);
		int x;
		int y;

		x = map.info.width/2 + x0/map.info.resolution;
		y = map.info.width*map.info.height/2 - int(-y0/map.info.resolution)*map.info.height - (map.info.width);

		int k = floor(x+y);

		if (k >= 0 && k < map.data.size())
		{
			map.data[k] = 100;
		}

		if (abs(old_ranges[i] - new_msg.ranges[i]) <= 0.5 || abs(old_ranges[i] - new_msg.ranges[i]) > 10)
		{    
			new_msg.ranges[i] = 0;
		}

 		orient = atan2((y - old_y), (x - old_x));
	}

	pub_3.publish(map);

	sensor_msgs::LaserScan scan;
   	scan.angle_min = -90*3.14159/180;
  	scan.angle_max = 90*3.14159/180;
  	scan.angle_increment = (scan.angle_max - scan.angle_min) / (720 - 1);
  	scan.range_max = 50;
  	scan.header = map.header;
 	scan.header.frame_id = "Human";

 	try {
 		transformstamped = tfBuffer.lookupTransform("hokuyo_base_laser_link", "Human", ros::Time(0));
 	}	
 	catch (tf2::TransformException &ex) {
 		ROS_WARN("%s", ex.what());
 	}

 	float x_t = transformstamped.transform.translation.x;
 	float y_t = transformstamped.transform.translation.y;
 	float z_t = transformstamped.transform.translation.z;

 	tf2Scalar q_x = transformstamped.transform.rotation.x;
 	tf2Scalar q_y = transformstamped.transform.rotation.y;
 	tf2Scalar q_z = transformstamped.transform.rotation.z;
 	tf2Scalar q_w = transformstamped.transform.rotation.w;

 	tf2::Quaternion qu(q_x, q_y, q_z, q_w);

  	float orient_3 = qu.getAngle();

 	std::cout << x_t << "\n";
 	
	map_ray_caster::MapRayCaster ray_caster;

	ray_caster.laserScanCast(map, scan, x_t, y_t);

	pub_2.publish(scan);

	int max = 0;
	int min = new_msg.ranges.size();

	for (int i = 0 ; i < new_msg.ranges.size(); i++)
	{
		if (new_msg.ranges[i] > 0)
		{
			if (max < i)
			{
				max = i;
			}
			if (min > i)
			{
				min = i;
			}
		}
	}

	float th_1 = new_msg.angle_min + max * new_msg.angle_increment;
	float th_2 = new_msg.angle_min + min * new_msg.angle_increment;
	float x_1 = new_msg.ranges[max] * cos(th_1);
	float x_2 = new_msg.ranges[min] * cos(th_2);
	float y_1 = new_msg.ranges[max] * sin(th_1);
	float y_2 = new_msg.ranges[min] * sin(th_2);
	float x = KALMAN_X((x_1 + x_2)/2);
	float y = KALMAN_Y((y_1 + y_2)/2);

	if (count_2 >= 1)
	{
		ros::Time time_now = msg->header.stamp;
		ros::Duration t_d(time_now - old_time);

		vel = sqrt(pow((x - old_x), 2.) + pow((y - old_y), 2.))/(t_d.toSec()); 
		
		// vel_param = vel * 2; // Change input from YAML
		
		old_time = time_now;
	}

	if (x > 0)
	{	
		if (count_2 > 0)
		{
			for (float j=0 ; j <= 1; j = j + 0.5)
			{
				float old_pos_x = 0;
				float old_pos_y = 0;
				
				vel_param = vel * j; // Change input from YAML

				for (int i=-90; i <=90 ; i = i + 10 )
				{
					geometry_msgs::Pose p;	
					// Add Pose Array for future points
					future.header.stamp = ros::Time::now();
					future.header.frame_id = "Human";

					p.position.x = vel_param*cos(i*3.14159/180);
					p.position.y = vel_param*sin(i*3.14159/180);
					p.position.z = 0.3;

					// int l = (i<0) ? -45 : 45;
					// int k = i + l;

					// float orient_2 = (j==0) ? i*3.14159/180 : k*3.14159/180;
					float orient_2 = i*3.14159/180;

					tf2::Quaternion q;
		  			q.setRPY(0, 0, orient_2);

					p.orientation.x = q.x();
				 	p.orientation.y = q.y();
				 	p.orientation.z = q.z();
				 	p.orientation.w = q.w();

					// int l = (x - old_x < 0) ? -1 : 1;
					future.poses.push_back(p);
				}
			}
			pub_1.publish(future);
		}

		count_2 = count_2 + 1;
		old_x = x;
		old_y = y;
	}


	future.poses.erase(future.poses.begin(), future.poses.end());
	map.data.erase(map.data.begin(), map.data.end());
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Project");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/scan", 1000, Callback_1);

	tf2_ros::TransformListener tfListener(tfBuffer);

	pub_1 = n.advertise<geometry_msgs::PoseArray>("Future_Pose", 1000);
	pub_2 = n.advertise<sensor_msgs::LaserScan>("ray_caster", 1000);
	pub_3 = n.advertise<nav_msgs::OccupancyGrid>("grid", 1000);

	ros::spin();

	return 0;
}