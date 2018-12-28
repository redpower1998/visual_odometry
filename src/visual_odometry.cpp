/*
BSD 2-Clause License

Copyright (c) 2018, redpower
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"

int visual_odometry(std::vector<cv::Point2f> & prevFeatures, std::vector<cv::Point2f> & currFeatures, 
	cv::Mat &rotation_vectors, cv::Mat &tranlation_vectors, double focal, cv::Point2d pp)
{
	if (rotation_vectors.data == NULL)
	{
		cv::Mat E, R, t, mask;
		//get essential matrix
		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
		//recovering the pose
		cv::recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

		rotation_vectors = R;
		tranlation_vectors = t;
	}
	else
	{
		cv::Mat R, t, mask;
		//get essential matrix
		cv::Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
		//recovering the pose
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask); 

		if ((t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))
		{
			tranlation_vectors = tranlation_vectors + rotation_vectors * t;
			rotation_vectors = R * rotation_vectors;
		}

		std::swap(prevFeatures, currFeatures);
	}
	return 0;
}


int main(int argc, char *argv[])
{
	std::cout << "Visual Odometry"<<std::endl;
	cv::Mat prev_image;     //the previous image
	cv::Mat rotation_vectors;//the final rotation vectors
	cv::Mat tranlation_vectors; //the final tranlation vectors
	std::vector<cv::Point> path_points;

	
	//the case is 2011_09_30_drive_0027_sync , width =1241 height=376
	//forcal = 780.1981  
	//intrinsic parameters  =660.1406, 272.1004
	double focal = 780.1981;
	cv::Point2d pp(660.1406, 272.1004);   //intrinsic parameters
	cv::VideoCapture capture("../2011_09_30_drive_0027_sync_tiny.mp4");  //in order to save space, a reduced version of the video was used here.

	while (true)
	{
		cv::Mat raw_image, gray_image;
		long long t = cv::getTickCount();
		if (!capture.read(raw_image))
			break;
		cv::resize(raw_image, raw_image, cv::Size(1241, 376));  //restore original size
		cv::cvtColor(raw_image, gray_image, cv::COLOR_RGB2GRAY); 
		std::vector<cv::Point2f> points[2];
		if (prev_image.data) 
		{
			std::vector<uchar> status;
			std::vector<float> err;
			goodFeaturesToTrack(prev_image, points[0], 10000, 0.01, 10);
			calcOpticalFlowPyrLK(prev_image, gray_image, points[0], points[1], status, err);
			
			visual_odometry(points[0], points[1], rotation_vectors, tranlation_vectors, focal, pp);
			std::cout << "Position: x = "
				<< tranlation_vectors.at<double>(0)
				<< "m y = "
				<< tranlation_vectors.at<double>(1)
				<< "m z = "
				<< tranlation_vectors.at<double>(2)
				<<"m" << std::endl;

			//display path 
			int x = tranlation_vectors.at<double>(0) + raw_image.cols / 2;
			int y = tranlation_vectors.at<double>(2) + raw_image.rows / 4;
			path_points.push_back(cv::Point(x, y));
			for (cv::Point point : path_points)
			{
				circle(raw_image, point, 1, CV_RGB(255, 255, 0), 2);
				circle(raw_image, point, 1, CV_RGB(255, 0, 0), 1);
			}
			cv::imshow("Path", raw_image);

		}
		std::swap(prev_image, gray_image);
		std::swap(points[1], points[0]);

		cv::waitKey(1);
	}
	std::cout << "Done!" << std::endl;
	cv::waitKey();
	return 0;
}

