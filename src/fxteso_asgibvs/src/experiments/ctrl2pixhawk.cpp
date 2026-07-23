//Including ROS libraries
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
//Including mavros messages
#include <mavros_msgs/Thrust.h> 
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

float thrust;
float thrust_norm = 0.5; //Normalized thrust value for hover flight 
int counter = 1;
int rc_CH_7 = 0;    //Channel for flight mode changing ---> State 1 and 2 -> STABILIZED MODE ---------State 3 -> OFFBOARD MODE
int rc_CH_10 = 0;   //Channel for IBVS Activation ------> State 1 -> LOCAL_POSITION_CONTROL WITH VICON----------State 2 -> IBVS
bool flag = false;

Eigen::Vector3f desired_attitude;
Eigen::Vector3f desired_attVel;
mavros_msgs::State current_state;

void thrustCallback(const std_msgs::Float32::ConstPtr& msg)
{
	thrust = msg->data;
}

void desAttEstCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	desired_attitude(0) = msg->linear.x;
    desired_attitude(1) = msg->linear.y;
    desired_attitude(2) = msg->linear.z;

    desired_attVel(0) = msg->angular.x;
    desired_attVel(1) = msg->angular.y;
    desired_attVel(2) = msg->angular.z;

}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg){
    rc_CH_7 = msg->channels[6];
    rc_CH_10 = msg->channels[9];
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ctrl2pixhawk");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	

    //ROS publishers, subscribers, and services
    ros::Subscriber des_att_est_sub = nh.subscribe("attitude_desired_estimates", 100, &desAttEstCallback);  
    ros::Subscriber thrust_sub = nh.subscribe("quad_thrust", 100, &thrustCallback);  
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, &state_cb);
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 100, &rcInCallback);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);
    ros::Publisher cmdVel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",100);
    ros::Publisher normThrust_pub = nh.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust",100);
    ros::Publisher flag_pub = nh.advertise<std_msgs::Bool>("flag_topic",100);

    mavros_msgs::Thrust thrust_normalized;
    geometry_msgs::TwistStamped attVel2pix;
    geometry_msgs::PoseStamped pose;
    std_msgs::Bool flag_var;
    //ros::Duration(4.5).sleep();

    flag_var.data =  false;
    flag_pub.publish(flag_var);

    //ROS Variables initialization
    attVel2pix.twist.angular.x = 0;
    attVel2pix.twist.angular.y = 0;
    attVel2pix.twist.angular.z = 0;

    thrust_normalized.header.seq = counter;
    thrust_normalized.header.stamp = ros::Time::now();
    thrust_normalized.header.frame_id = 1;
    thrust_normalized.thrust = thrust_norm;

    cmdVel_pub.publish(attVel2pix);
    normThrust_pub.publish(thrust_normalized);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.80;
   
    while(ros::ok() && !current_state.connected)
    {
       ros::spinOnce();
       loop_rate.sleep();
       std::cout<<"Connecting"<<std::endl;
    }
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){        
        local_pos_pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode stabilized_set_mode;
    stabilized_set_mode.request.custom_mode = "STABILIZED";


    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {   
        if (rc_CH_7<1930 && current_state.mode != "STABILIZED" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(stabilized_set_mode) && stabilized_set_mode.response.mode_sent)
            {
                ROS_INFO("Stabilized enabled");
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
        
        
        if (rc_CH_7>=1930 && current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
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
        
        
        if (rc_CH_10 <1930)
        {
            flag = true;
            thrust_norm = thrust/30;    //Thrust in N / Maximum thrust
    
            attVel2pix.header.seq = counter;
            attVel2pix.header.stamp = ros::Time::now();
            attVel2pix.header.frame_id = 1;

            attVel2pix.twist.angular.x = desired_attVel(0);
            attVel2pix.twist.angular.y = desired_attVel(1);
            attVel2pix.twist.angular.z = desired_attVel(2);

            thrust_normalized.header.seq = counter;
            thrust_normalized.header.stamp = ros::Time::now();
            thrust_normalized.header.frame_id = 1;
            thrust_normalized.thrust = thrust_norm;

            cmdVel_pub.publish(attVel2pix);
            normThrust_pub.publish(thrust_normalized);

            
            counter++;
        }
        else
        {
            flag = false;
            local_pos_pub.publish(pose);
            counter = 1;
        }
        
        flag_var.data =  flag;
        flag_pub.publish(flag_var);     
        
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
