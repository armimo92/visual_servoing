//Including ROS libraries
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

mavros_msgs::State current_state;
float thrust = 0;
float thrust_normalized = 0;
float thrust_min = 0;
float thrust_max = 30;

Eigen::Vector3f attitude_vel_des(0,0,0);
int count = 1;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void attDesEstCallback(const geometry_msgs::Twist::ConstPtr& attD)
{
	attitude_vel_des(0) = attD->angular.x;
	attitude_vel_des(1) = attD->angular.y;
	attitude_vel_des(2) = attD->angular.z;
}

void thrustCallback(const std_msgs::Float64::ConstPtr& th)
{
	thrust = th->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_setpoints");
    ros::NodeHandle nh;
    ros::Rate rate(100);

////////////////////////////////ROS Publishers////////////////////////////////////////////////// 
    ros::Publisher attVel_des_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel",100);
    ros::Publisher thrust_des_pub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust",100);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

////////////////////////////////ROS Subscribers//////////////////////////////////////////////////
		 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, &state_cb);
    ros::Subscriber attitude_des_est_sub = nh.subscribe<geometry_msgs::Twist>("attitude_desired_estimates",100,&attDesEstCallback);
    ros::Subscriber thrust_sub = nh.subscribe<std_msgs::Float64>("quad_thrust",100,&thrustCallback);

////////////////////////////////ROS Services/////////////////////////////////////////////////////
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
////////////////////////////////ROS Variables////////////////////////////////////////////////////
	geometry_msgs::TwistStamped attVel_des_var;
	mavros_msgs::Thrust thrust_des_var;
	geometry_msgs::PoseStamped pose;

	attVel_des_var.twist.angular.x = 0;
	attVel_des_var.twist.angular.y = 0;
	attVel_des_var.twist.angular.z = 0;

	thrust_des_var.thrust = 0;

	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 2;

	while(ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
    	rate.sleep();
	    std::cout<<"Connecting"<<std::endl;
	}

	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i)
	{        
		local_pos_pub.publish(pose);
		ros::spinOnce();
    	rate.sleep();
	}
    
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	while(ros::ok())
	{
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > 		ros::Duration(5.0)))
    	{
        	if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        	{
        	    ROS_INFO("Offboard enabled");
	        }
	        last_request = ros::Time::now();
	    } 
	    else 
	    {
	        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
	        {
	            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
	            {
	                ROS_INFO("Vehicle armed");
	            }
	        last_request = ros::Time::now();
	        }
	    } 
    
    	thrust_des_var.header.seq = count;
		thrust_des_var.header.stamp = ros::Time::now();
		thrust_des_var.header.frame_id = 1;
		
		attVel_des_var.header.seq = count;
		attVel_des_var.header.stamp = ros::Time::now();
		attVel_des_var.header.frame_id = 1;
		
    	thrust_normalized = (thrust - thrust_min) / (thrust_max - thrust_min);
    	thrust_des_var.thrust = thrust_normalized;
		
    	attVel_des_var.twist.angular.x = attitude_vel_des(0);
		attVel_des_var.twist.angular.y = attitude_vel_des(1);
		attVel_des_var.twist.angular.z = attitude_vel_des(2);
		
		count++;
		
    	attVel_des_pub.publish(attVel_des_var);	
    	thrust_des_pub.publish(thrust_des_var);
    	
    	ros::spinOnce();
    	rate.sleep();
    	
	}
	return 0;
}
 
