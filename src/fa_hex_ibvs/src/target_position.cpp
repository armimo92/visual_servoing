#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>


float pos_x;
float pos_y;
float xp;
float yp;
float xpp;
float ypp;
float pos_z = 0;
float yaw;
float yawRate;
float yawAccel;
float t;
float step = 0.01;
float arg;
float x_traj;
float y_traj;

float tgt_roll;
float tgt_pitch;

Eigen::Vector3f quad_pos;
Eigen::Vector3f error;


void quadPosCallback(const geometry_msgs::Vector3::ConstPtr& quadPos)
{
	quad_pos(0) = quadPos->x;
    quad_pos(1) = quadPos->y;
    quad_pos(2) = quadPos->z;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tgt_pos");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Publisher tgt_pos_pub = nh.advertise<geometry_msgs::Vector3>("tgt_position",100);
	ros::Publisher tgt_yaw_pub = nh.advertise<std_msgs::Float64>("tgt_yaw",100);
	ros::Publisher tgt_vel_pub = nh.advertise<geometry_msgs::Vector3>("tgt_velocity",100);
	ros::Publisher tgt_yaw_rate_pub = nh.advertise<std_msgs::Float64>("tgt_yaw_rate",100);
	ros::Publisher tgt_accel_pub = nh.advertise<geometry_msgs::Vector3>("tgt_acceleration",100);
	ros::Publisher tgt_yaw_acceleration_pub = nh.advertise<std_msgs::Float64>("tgt_yaw_acceleration",100);

	ros::Publisher tgt_attitude_pub = nh.advertise<geometry_msgs::Vector3>("tgt_attitude",100);

	ros::Publisher error_lin_pub = nh.advertise<geometry_msgs::Vector3>("position_error",100);
	

	ros::Subscriber quad_pos_sub = nh.subscribe("quad_position", 100, &quadPosCallback);

	

	geometry_msgs::Vector3 tgt_position;
	geometry_msgs::Vector3 tgt_attitude_var;
	geometry_msgs::Vector3 positionError;
	std_msgs::Float64 tgt_yaw;
	
	geometry_msgs::Vector3 tgt_vel;
	std_msgs::Float64 tgt_yaw_vel;
	std_msgs::Float64 tgt_psi_rate;

	geometry_msgs::Vector3 tgt_accel;
	std_msgs::Float64 tgt_yaw_accel;	
	
	
	int i = 0;
	int sim_time = 100/step; //Seconds / step

	pos_x = 0;
	pos_y = -4;
	xp = 0;
	yp = 0;
	yaw = 0;
	yawRate = 0;
	tgt_position.x = pos_x;
	tgt_position.y = pos_y;
	tgt_position.z = pos_z;
	tgt_yaw.data = yaw;
	tgt_vel.x = xp;
	tgt_vel.y = yp;
	tgt_vel.z = 0;
	tgt_psi_rate.data = yawRate;
	ros::Duration(0.05).sleep();
	tgt_pos_pub.publish(tgt_position);
	tgt_yaw_pub.publish(tgt_yaw);
	tgt_vel_pub.publish(tgt_vel);
	tgt_yaw_rate_pub.publish(tgt_psi_rate);
	ros::Duration(2).sleep();
	tgt_pos_pub.publish(tgt_position);
	tgt_yaw_pub.publish(tgt_yaw);
	tgt_vel_pub.publish(tgt_vel);
	tgt_yaw_rate_pub.publish(tgt_psi_rate);

	ros::Duration(0.1).sleep();
	while(ros::ok())
	{
		t = i*step;
		
		
		xp = 1 * cos(0.1*t);
		yp = 1 * sin(0.1*t);
		yawRate = 0.1;

		pos_x = pos_x + xp*step;
		pos_y = pos_y + yp*step;
		yaw = 0.1*t;	
		tgt_pitch = 0;
		tgt_roll = 0;	
		
		/*
		
		pos_x = 0;
		pos_y = 0;
			
		xp = 0;
		yp = 0;

		xpp = 0;
		ypp = 0;
			
		yaw = 0;
		yawRate = 0;
		yawAccel = 0;

		tgt_pitch = 0;
		tgt_roll = 0;
		*/

		/*
		pos_x = 0.3*t;
		pos_y = -3;
			
		xp = 0.3;
		yp = 0;

		xpp = 0;
		ypp = 0;
			
		yaw = 0;
		yawRate = 0;
		yawAccel = 0;
		*/

		error(0) = quad_pos(0) - pos_x;
		error(1) = quad_pos(1) - pos_y;
		error(2) = quad_pos(2) + 2.5;

		tgt_position.x = pos_x;
		tgt_position.y = pos_y;
		tgt_position.z = pos_z;
		tgt_yaw.data = yaw;

		tgt_vel.x = xp;
		tgt_vel.y = yp;
		tgt_vel.z = 0;
		tgt_psi_rate.data = yawRate;

		tgt_accel.x = xpp;
		tgt_accel.y = ypp;
		tgt_accel.z = 0;
		tgt_yaw_accel.data = 0;

		tgt_attitude_var.x = tgt_roll;
		tgt_attitude_var.y = tgt_pitch;
		tgt_attitude_var.z = yaw;

		positionError.x = error(0);
		positionError.y = error(1);
		positionError.z = error(2);

		tgt_pos_pub.publish(tgt_position);
		tgt_yaw_pub.publish(tgt_yaw);
		
		tgt_vel_pub.publish(tgt_vel);
		tgt_yaw_rate_pub.publish(tgt_psi_rate);

		tgt_accel_pub.publish(tgt_accel);

		tgt_attitude_pub.publish(tgt_attitude_var);

		error_lin_pub.publish(positionError);
		

		i = i+1;

		std::cout << yawRate << std::endl;

		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	return 0;	
}
