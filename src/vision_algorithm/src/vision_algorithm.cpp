#include <iostream>
#include <math.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>

int ng1;
int ug1;
int ug2;
int ng2;
int ug3;
int ng3;
int ug4;
int ng4;

int ug;
int ng;

int keypress = cv::waitKey(25);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vision_algorithm");

	ros::NodeHandle nh;
	ros::Rate loop_rate(30);	
	
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);
	
    cv::Mat frame;

    //DETECT ARUCO MARKER
	//OPEN THE CAMERA
	cv::VideoCapture cap(0);

	//CHECK IF CAMERA OPENED SUCCESFULLY
	if (!cap.isOpened())
	{
		std::cout << "Error abriendo la cámara" << std::endl;
		return -1;
	}

	//LOAD THE DICTIONARY TO USE
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);

     sensor_msgs::ImagePtr msg;

	while(ros::ok())
	{
		
		//CAPTURE FRAME
		cap >> frame;

		//If frame is empty, break
		if (frame.empty())
		{
			break;
		}



		//INITIALIZE THE DETECTOR PARAMETERS USING DEFAULT VALUES
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();


		//VECTORS THAT WOULD CONTAIN THE DETECTED MARKER CORNERS AND THE REJECTED CANDIDATES
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectCandidates;

		//SAVE THE MARKER ID
		std::vector<int> markerIds;

		//Detect the markers in the image
		cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectCandidates);


		if (markerIds.empty())
		{
			std::cout << "No hay marca" << std::endl;
		}

		if (markerIds.size() == 4)
		{
			//Assignment of Point 1
			if (markerIds[0] == 4)
			{
				//Extracción de esquinas
				cv::Point p11 = markerCorners[0].at(0);
				cv::Point p21 = markerCorners[0].at(1);
				cv::Point p31 = markerCorners[0].at(2);
				cv::Point p41 = markerCorners[0].at(3);

				//Centroide para 4 puntos
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else if (markerIds[1] == 4)
			{
				//Extracción de esquinas
				cv::Point p11 = markerCorners[1].at(0);
				cv::Point p21 = markerCorners[1].at(1);
				cv::Point p31 = markerCorners[1].at(2);
				cv::Point p41 = markerCorners[1].at(3);

				//Centroide para 4 puntos
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else if (markerIds[2] == 4)
			{
				//Extracción de esquinas
				cv::Point p11 = markerCorners[2].at(0);
				cv::Point p21 = markerCorners[2].at(1);
				cv::Point p31 = markerCorners[2].at(2);
				cv::Point p41 = markerCorners[2].at(3);

				//Centroide para 4 puntos
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else
			{
				//Extracción de esquinas
				cv::Point p11 = markerCorners[3].at(0);
				cv::Point p21 = markerCorners[3].at(1);
				cv::Point p31 = markerCorners[3].at(2);
				cv::Point p41 = markerCorners[3].at(3);

				//Centroide para 4 puntos
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			//Assignment of Point 2
			if (markerIds[0] == 6)
			{
				//Extracción de esquinas
				cv::Point p12 = markerCorners[0].at(0);
				cv::Point p22 = markerCorners[0].at(1);
				cv::Point p32 = markerCorners[0].at(2);
				cv::Point p42 = markerCorners[0].at(3);

				//Centroide para 4 puntos
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else if (markerIds[1] == 6)
			{
				//Extracción de esquinas
				cv::Point p12 = markerCorners[1].at(0);
				cv::Point p22 = markerCorners[1].at(1);
				cv::Point p32 = markerCorners[1].at(2);
				cv::Point p42 = markerCorners[1].at(3);

				//Centroide para 4 puntos
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else if (markerIds[2] == 6)
			{
				//Extracción de esquinas
				cv::Point p12 = markerCorners[2].at(0);
				cv::Point p22 = markerCorners[2].at(1);
				cv::Point p32 = markerCorners[2].at(2);
				cv::Point p42 = markerCorners[2].at(3);

				//Centroide para 4 puntos
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else
			{
				//Extracción de esquinas
				cv::Point p12 = markerCorners[3].at(0);
				cv::Point p22 = markerCorners[3].at(1);
				cv::Point p32 = markerCorners[3].at(2);
				cv::Point p42 = markerCorners[3].at(3);
			}
			///////////////////////////////////////////////////////////////////////////////77
			//Assignment of Point 3
			if (markerIds[0] == 8)
			{
				//Extracción de esquinas
				cv::Point p13 = markerCorners[0].at(0);
				cv::Point p23 = markerCorners[0].at(1);
				cv::Point p33 = markerCorners[0].at(2);
				cv::Point p43 = markerCorners[0].at(3);

				//Centroide para 4 puntos
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else if (markerIds[1] == 8)
			{
				//Extracción de esquinas
				cv::Point p13 = markerCorners[1].at(0);
				cv::Point p23 = markerCorners[1].at(1);
				cv::Point p33 = markerCorners[1].at(2);
				cv::Point p43 = markerCorners[1].at(3);

				//Centroide para 4 puntos
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else if (markerIds[2] == 8)
			{
				//Extracción de esquinas
				cv::Point p13 = markerCorners[2].at(0);
				cv::Point p23 = markerCorners[2].at(1);
				cv::Point p33 = markerCorners[2].at(2);
				cv::Point p43 = markerCorners[2].at(3);

				//Centroide para 4 puntos
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else
			{
				//Extracción de esquinas
				cv::Point p13 = markerCorners[3].at(0);
				cv::Point p23 = markerCorners[3].at(1);
				cv::Point p33 = markerCorners[3].at(2);
				cv::Point p43 = markerCorners[3].at(3);

				//Centroide para 4 puntos
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
		////////////////////////////////////////////////////////////////////////////////////7
			//Assignment of Point 4
			if (markerIds[0] == 10)
			{
				//Extracción de esquinas
				cv::Point p14 = markerCorners[0].at(0);
				cv::Point p24 = markerCorners[0].at(1);
				cv::Point p34 = markerCorners[0].at(2);
				cv::Point p44 = markerCorners[0].at(3);

				//Centroide para 4 puntos
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else if (markerIds[1] == 10)
			{
				//Extracción de esquinas
				cv::Point p14 = markerCorners[1].at(0);
				cv::Point p24 = markerCorners[1].at(1);
				cv::Point p34 = markerCorners[1].at(2);
				cv::Point p44 = markerCorners[1].at(3);

				//Centroide para 4 puntos
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else if (markerIds[2] == 10)
			{
				//Extracción de esquinas
				cv::Point p14 = markerCorners[2].at(0);
				cv::Point p24 = markerCorners[2].at(1);
				cv::Point p34 = markerCorners[2].at(2);
				cv::Point p44 = markerCorners[2].at(3);

				//Centroide para 4 puntos
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else
			{
				//Extracción de esquinas
				cv::Point p14 = markerCorners[3].at(0);
				cv::Point p24 = markerCorners[3].at(1);
				cv::Point p34 = markerCorners[3].at(2);
				cv::Point p44 = markerCorners[3].at(3);

				//Centroide para 4 puntos
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			

			ug = (ug1 + ug2 + ug3 + ug4) / 4;
			ng = (ng1 + ng2 + ng3 + ng4) / 4;
			
			cv::circle(frame, cv::Point(ug, ng), 8, cv::Scalar(255, 0, 255), -1);
			cv::circle(frame, cv::Point(ug1, ng1), 8, cv::Scalar(255, 0, 0), -1);
			cv::circle(frame, cv::Point(ug2, ng2), 8, cv::Scalar(0, 255, 0), -1);
			cv::circle(frame, cv::Point(ug3, ng3), 8, cv::Scalar(0, 0, 255), -1);
			cv::circle(frame, cv::Point(ug4, ng4), 8, cv::Scalar(255, 255, 0), -1);
			
			std::cout << "OK" << std::endl;
		}

		if (markerIds.size() == 3)
		{
			std::cout << "3 marcas" << std::endl;
		}

		if (markerIds.size() == 2)
		{
			std::cout << "2 marcas" << std::endl;
		}

		if (markerIds.size() == 1)
		{
			std::cout << "1 marca" << std::endl;
		}


        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
	
		if (keypress == 27)
		{
			break;
		}

        ros::spinOnce();
        loop_rate.sleep();

	}

	cap.release();
	cv::destroyAllWindows();
	return 0;
}


// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file	