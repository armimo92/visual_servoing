//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector4f imFeat; //qx, qy, qz, qpsi
Eigen::Vector4f imFeat_estimate;
Eigen::Vector4f imFeat_estimate_dot;
Eigen::Vector4f x1_hat_dot;
Eigen::Vector4f x2_hat_dot;
Eigen::Vector4f alpha;
Eigen::Vector4f TD_gain_1;
Eigen::Vector4f TD_gain_2;
Eigen::Vector4f estimation_error;

float step = 0.02;

float sign(float var)
{
    float result;
    if(var>0)
    {
        result = 1;
    }
    else if(var<0)
    {
        result = -1;
    }
    else if (var == 0)
    {
        result = 0;
    }
    return result;
}

float sig(float a, float b) //sig = |a|^b * sign(a)
{
    float s;
    s = powf(std::abs(a), b) * sign(a);
    return s;
}

void imFeatCallback(const geometry_msgs::Quaternion::ConstPtr& img_features)
{
	imFeat(0) = img_features->x;
	imFeat(1) = img_features->y;
	imFeat(2) = img_features->z;
    imFeat(3) = img_features->w;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "ibvs_pos_ctrl");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	
    
    //ROS publishers and subscribers
    ros::Subscriber im_feat_sub = nh.subscribe("ImFeat_vector", 100, &imFeatCallback);  


    ros::Publisher im_feat_est_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_estimates",100);
    ros::Publisher im_feat_est_err_pub = nh.advertise<geometry_msgs::Quaternion>("estimation_error",100);
    ros::Publisher im_feat_est_dot_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_dot_estimates",100);

    geometry_msgs::Quaternion im_feat_est_var;
    geometry_msgs::Quaternion im_feat_est_dot_var;
    geometry_msgs::Quaternion im_feat_est_err_var;

    imFeat_estimate << 0, 0, 1, 0; 
    imFeat_estimate_dot << 0,0,0,0;
    alpha << 0.5, 0.5, 0.5, 0.5;
    TD_gain_1 << 12, 12, 12, 12;
    TD_gain_2 << 35, 35, 35, 35;

    im_feat_est_var.x = imFeat_estimate(0);
    im_feat_est_var.y = imFeat_estimate(1);
    im_feat_est_var.z = imFeat_estimate(2);
    im_feat_est_var.w = imFeat_estimate(3);

    im_feat_est_dot_var.x = imFeat_estimate_dot(0);
    im_feat_est_dot_var.y = imFeat_estimate_dot(1);
    im_feat_est_dot_var.z = imFeat_estimate_dot(2);
    im_feat_est_dot_var.w = imFeat_estimate_dot(3);

    im_feat_est_err_var.x = estimation_error(0);
    im_feat_est_err_var.y = estimation_error(1);
    im_feat_est_err_var.z = estimation_error(2);
    im_feat_est_err_var.w = estimation_error(3);

    im_feat_est_pub.publish(im_feat_est_var);
    im_feat_est_dot_pub.publish(im_feat_est_dot_var);
    im_feat_est_err_pub.publish(im_feat_est_err_var);
    ros::Duration(0.01).sleep();


    while(ros::ok())
    {   
        /////// Tracking Differentiator ///////

        for(int i = 0; i<=3; i++)
        {
            estimation_error(i) = imFeat(i) - imFeat_estimate(i);
        
            x2_hat_dot(i) = TD_gain_2(i)*sig(estimation_error(i), alpha(i));
            imFeat_estimate_dot(i) = imFeat_estimate_dot(i) + x2_hat_dot(i)*step;

            x1_hat_dot(i) = imFeat_estimate_dot(i) + TD_gain_1(i)*sig(estimation_error(i), ((alpha(i)+1)/2));
            imFeat_estimate(i) = imFeat_estimate(i) + x1_hat_dot(i)*step;
        
        }
        
        im_feat_est_var.x = imFeat_estimate(0);
        im_feat_est_var.y = imFeat_estimate(1);
        im_feat_est_var.z = imFeat_estimate(2);
        im_feat_est_var.w = imFeat_estimate(3);

        im_feat_est_dot_var.x = imFeat_estimate_dot(0);
        im_feat_est_dot_var.y = imFeat_estimate_dot(1);
        im_feat_est_dot_var.z = imFeat_estimate_dot(2);
        im_feat_est_dot_var.w = imFeat_estimate_dot(3);

        im_feat_est_err_var.x = estimation_error(0);
        im_feat_est_err_var.y = estimation_error(1);
        im_feat_est_err_var.z = estimation_error(2);
        im_feat_est_err_var.w = estimation_error(3);

        im_feat_est_pub.publish(im_feat_est_var);
        im_feat_est_dot_pub.publish(im_feat_est_dot_var);
        im_feat_est_err_pub.publish(im_feat_est_err_var);

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}