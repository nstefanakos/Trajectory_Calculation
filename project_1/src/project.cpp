# include "ros/ros.h"
# include "sensor_msgs/LaserScan.h"
# include "nav_msgs/OccupancyGrid.h"
# include <tf2/LinearMath/Quaternion.h>
# include <visualization_msgs/MarkerArray.h>
# include <geometry_msgs/PoseArray.h>
# include <cmath>
# include <vector>

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

ros::Publisher pub_1;
std::vector<float> old_ranges;
ros::Time old_time;
geometry_msgs::PoseArray future;

void Callback_1(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan new_msg(*msg);
	
	if (count_1 == 0)
	{
		old_ranges = new_msg.ranges;
		count_1++;
	}

	for (int i = 0 ; i < new_msg.ranges.size(); i++)
	{
		if (abs(old_ranges[i] - new_msg.ranges[i]) <= 0.5 || abs(old_ranges[i] - new_msg.ranges[i]) > 10)
		{    
			new_msg.ranges[i] = 0;
		}
	}

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

	int i_human = (max+min)/2;
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
			float slope = (y - old_y)/(x - old_x);
			float orient = atan2((y - old_y), (x - old_x));

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
					
					float c = sqrt(pow(old_ranges[i_human], 2.) + pow(vel_param, 2.) - 2*old_ranges[i_human]*vel_param*cos(3.14159 - orient_2));

					float B = acos((pow(old_ranges[i_human], 2.) - pow(vel_param, 2.) + pow(c, 2.))/2*old_ranges[i_human]*c);
					
					tf2::Quaternion q;
		  			q.setRPY(0, 0, orient_2);

					p.orientation.x = q.x();
				 	p.orientation.y = q.y();
				 	p.orientation.z = q.z();
				 	p.orientation.w = q.w();

					// int l = (x - old_x < 0) ? -1 : 1;

				 	int deg = i_human - (B*new_msg.angle_increment);

				 	if (i > 0)
				 	{
					 	if (c < old_ranges[i_human + deg])
					 	{	
							future.poses.push_back(p);
					 	}
				 	}
				 	else
				 	{
				 		if (c < old_ranges[i_human - deg])
					 	{	
							future.poses.push_back(p);
					 	}
				 	}
				}
			}
			pub_1.publish(future);
		}

		count_2 = count_2 + 1;
		old_x = x;
		old_y = y;
	}


	future.poses.erase(future.poses.begin(), future.poses.end());
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Project");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/scan", 1000, Callback_1);

	pub_1 = n.advertise<geometry_msgs::PoseArray>("Future_Pose", 1000);

	ros::spin();

	return 0;
}