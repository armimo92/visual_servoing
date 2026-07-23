#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <math.h>

bool flag = false;
int sw_value = 0; 

void rcCallback(const mavros_msgs::RCIn::ConstPtr& rci);
{
	sw_value = rci->channels[0];	//Choose the appropiate RC channel to start/stop the IBVS algorithm
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rc_flag");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	
    
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in",100,&rcCallback);
	ros::Publisher flag_pub = nh.advertise<std_msgs::Bool>("flag_ibvs",100);
	
	std_msgs::Bool flag_var;
	flag_var.data = flag;
	flag_pub.publish(flag_var);
	
    while (ros::ok())
	{
		if(sw_value>1600)
		{
			flag = true;
		}
		else
		{
			flag = false;
		}
		
		flag_var.data = flag;
		flag_pub.publish(flag_var);
		
		ros::spinOnce();
		loop_rate.sleep();  		
	}

    return 0;
}
