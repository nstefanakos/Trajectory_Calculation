# include <ros/ros.h>
# include <tf2/LinearMath/Quaternion.h>
# include <tf2_ros/transform_broadcaster.h>
# include <geometry_msgs/TransformStamped.h>
# include "sensor_msgs/LaserScan.h"
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

std::vector<float> old_ranges;
int count_1 = 0;
int count_2 = 0;
float old_x;
float old_y;

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

	float th_1 = new_msg.angle_min + max * new_msg.angle_increment;
	float th_2 = new_msg.angle_min + min * new_msg.angle_increment;
	float x_1 = new_msg.ranges[max] * cos(th_1);
	float x_2 = new_msg.ranges[min] * cos(th_2);
	float y_1 = new_msg.ranges[max] * sin(th_1);
	float y_2 = new_msg.ranges[min] * sin(th_2);
	float x = KALMAN_X((x_1 + x_2)/2);
	float y = KALMAN_Y((y_1 + y_2)/2);

	if (x > 0)
	{	
		if (count_2 > 0)
		{
			float orient = atan2((y - old_y), (x - old_x));
			
			static tf2_ros::TransformBroadcaster br;
			geometry_msgs::TransformStamped transformStamped;

			transformStamped.header.stamp = ros::Time::now();
		  	transformStamped.header.frame_id = "/hokuyo_base_laser_link";
		  	transformStamped.child_frame_id = "Human";
		  	transformStamped.transform.translation.x = x;
		  	transformStamped.transform.translation.y = y;
		  	transformStamped.transform.translation.z = 0.0;

		  	tf2::Quaternion q;
		  	q.setRPY(0, 0, orient);

		  	transformStamped.transform.rotation.x = q.x();
		  	transformStamped.transform.rotation.y = q.y();
		  	transformStamped.transform.rotation.z = q.z();
		  	transformStamped.transform.rotation.w = q.w();

		  	br.sendTransform(transformStamped);
		}

		count_2 = count_2 + 1;
		old_x = x;
		old_y = y;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Frame");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/scan", 1000, Callback_1);

	ros::spin();

	return 0;
}