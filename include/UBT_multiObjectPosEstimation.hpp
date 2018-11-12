/*******************************************************************************
Copyright(c) UBTech, All right reserved.

This file is UBTech's property. It contains UBTech's trade secret, proprietary
and confidential information.

The information and code contained in this file is only for authorized UBTech
employees to design, create, modify, or review.

DO NOT DISTRIBUTE, DO NOT DUPLICATE OR TRANSMIT IN ANY FORM WITHOUT PROPER
AUTHORIZATION.

If you are not an intended recipient of this file, you must not copy,
distribute, modify, or take any action in reliance on it.

If you have received this file in error, please immediately notify UBTech and
permanently delete the original and any copy of any file and any printout thereof.
*******************************************************************************/

#pragma once

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/*******************************************************************************************
	Function:    initPoseDetector
	Description: Create and initialize the pose estimation engine

	params:
    models_path[IN]:  The path that contains the 3D model file and caffemodel for decode ID
	camera_info[IN]:  Camera internal parameters and distortion parameters, double

	Return:
	true                 - succeed
	false                - failed
	*******************************************************************************************/
bool initPoseDetector(string models_path, string camera_info);


/*******************************************************************************************
	Function:    applyPoseDetector
	Description: Estimating Pose Parameters of 3D Objects in the input image buffer

	params:
	frame[IN]:  image data

	Return:
	vector< pair<string, Mat> >    -Pose of an object,[orientation.x orientation.y orientation.z orientation.w position.x position.y position.z] 
								    The first four is quaternions
	*******************************************************************************************/
vector< pair<string, Mat> > applyPoseDetector(Mat frame);


/*******************************************************************************************
	Function:    destroyPoseDetector
	Description: release the pose estimation engine

	*******************************************************************************************/
void destroyPoseDetector();
