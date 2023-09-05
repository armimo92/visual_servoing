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

float step = 0.02;

Eigen::Vector4f imFeat; //qx, qy, qz, qpsi
Eigen::Vector4f imFeat_estimate(0,0,1.4,0);
Eigen::Vector4f imFeat_estimate_dot(0,0,0,0);
Eigen::Vector4f x3_hat(0,0,0,0);
Eigen::Vector4f x4_hat(0,0,0,0);
Eigen::Vector4f x1_hat_dot(0,0,0,0);
Eigen::Vector4f x2_hat_dot(0,0,0,0);
Eigen::Vector4f x3_hat_dot(0,0,0,0);
Eigen::Vector4f x4_hat_dot(0,0,0,0);
Eigen::Vector4f estimation_error(0,0,0,0);

float alpha, R, delta, G1, G2, G3, G4;


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

float fal(float var, float pot, float delta)
{
    float res;
    if (std::abs(var) > delta)
    {
        res = powf(std::abs(var), pot) * sign(var);
    }
    if (std::abs(var) <= delta)
    {
        res = var/powf(delta, (1-pot));
    }

    return res;

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
    ros::init(argc, argv, "HanDiff");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	

    //ROS publishers and subscribers
    ros::Subscriber im_feat_sub = nh.subscribe("ImFeat_vector", 100, &imFeatCallback);  


    ros::Publisher im_feat_est_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_estimates_han",100);
    ros::Publisher im_feat_est_err_pub = nh.advertise<geometry_msgs::Quaternion>("estimation_error_han",100);
    ros::Publisher im_feat_est_dot_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_dot_estimates_han",100);

    geometry_msgs::Quaternion im_feat_est_var;
    geometry_msgs::Quaternion im_feat_est_dot_var;
    geometry_msgs::Quaternion im_feat_est_err_var;

    imFeat_estimate << 0, 0, 1.4, 0; 
    imFeat_estimate_dot << 0,0,0,0;
    x3_hat << 0,0,0,0;
    x4_hat << 0,0,0,0;
    estimation_error << 0, 0, 0, 0;
    x1_hat_dot << 0, 0, 0, 0;
    x2_hat_dot << 0, 0, 0, 0;
    
    alpha = 0.94;
    R = 12;
    delta = 0.05;

    G1 = 1;
    G2 = 3;
    G3 = 2;
    G4 = 2;


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
    ros::Duration(2.1).sleep();



    while(ros::ok())
    {
        /////// Tracking Differentiator ///////

        //fal(float var, float pot, float delta)

        for(int i = 0; i<=3; i++)
        {
            estimation_error(i) =  imFeat_estimate(i) - imFeat(i);

            x4_hat_dot(i) = powf(R, 4) * (-fal(estimation_error(i)*G1, alpha, delta) - fal((imFeat_estimate_dot(i)*G2/R), alpha, delta) - fal((x3_hat(i)*G3/(R*R)), alpha, delta) - fal((x4_hat(i)*G4/(R*R*R)), alpha, delta));
            x4_hat(i) = x4_hat(i) + x4_hat_dot(i) * step;

            x3_hat_dot(i) = x4_hat(i);
            x3_hat(i) = x3_hat(i) + x3_hat_dot(i) * step;

            x2_hat_dot(i) = x3_hat(i); 
            imFeat_estimate_dot(i) = imFeat_estimate_dot(i) + x2_hat_dot(i) * step;

            x1_hat_dot(i) = imFeat_estimate_dot(i);
            imFeat_estimate(i) = imFeat_estimate(i) + x1_hat_dot(i) * step;
            
        
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