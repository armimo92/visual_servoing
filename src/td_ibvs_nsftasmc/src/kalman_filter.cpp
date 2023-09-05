#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>

float step = 0.02;
Eigen::Vector4f ImFeat_dot_est(0,0,0,0);
Eigen::Vector4f kf_ImFeat_dot_est(0,0,0,0);

float kalman(float input)
{
    static const float r = 40;  //noise covariance
    static const float h = 1;   //measurement map scalar
    static float q = 10;    //initial estimates covariance
    static float p = 0;     //initial error covariance (must be zero)
    static float est = 0;   //initial estimated state
    static float k = 0 ;     //initial kalman gain 

    k = (p*h)/( h * p * h + r); //update kalman gain
    est = est + k * (input - h * est);  //update estimation
    p = (1 - k * h) * p + q;    //update error covariance

    return est;
}

void ImFeatEstDotCallback(const geometry_msgs::Quaternion::ConstPtr& imfd)
{
	ImFeat_dot_est(0) = imfd->x;
    ImFeat_dot_est(1) = imfd->y;
    ImFeat_dot_est(2) = imfd->z;
    ImFeat_dot_est(3) = imfd->w;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "kalman_filter");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);
	
	ros::Subscriber ImFeat_dot_est_sub = nh.subscribe("ImFeat_dot_estimates_fxt", 100, &ImFeatEstDotCallback);
	ros::Publisher kf_output_pub = nh.advertise<geometry_msgs::Quaternion>("kf_output",100);

	geometry_msgs::Quaternion kf_output_var;

    kf_output_var.x = 0;
    kf_output_var.y = 0;
    kf_output_var.z = 0;
    kf_output_var.w = 0;
    kf_output_pub.publish(kf_output_var);
	
	while(ros::ok())
	{
		for (int i = 0; i<=3; i++)
        {
            kf_ImFeat_dot_est(i) = kalman(ImFeat_dot_est(i));
        }

        kf_output_var.x = kf_ImFeat_dot_est(0);
        kf_output_var.y = kf_ImFeat_dot_est(1);
        kf_output_var.z = kf_ImFeat_dot_est(2);
        kf_output_var.w = kf_ImFeat_dot_est(3);
        kf_output_pub.publish(kf_output_var);

		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	return 0;	
}
