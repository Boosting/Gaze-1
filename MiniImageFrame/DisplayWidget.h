#pragma once
#include <GTEnginePCH.h>

#include <opencv2/imgproc.hpp>

//This Class mainly solves the problem that given gaze angle what the dot will
//be shown on the display(laptop).
//The unit in this program is 1=1mm

//In OpenFace, the frame is in camera coo:x is screen left;y is up;z is direct to you.

using namespace gte;

class DisplayWidget
{
public:
	DisplayWidget();
	~DisplayWidget();

	bool DotEstimate(const cv::Point3f gazeDirection0, const cv::Point3f gazeDirection1, const cv::Point3f left_eyeball_center,
		const cv::Point3f right_eyeball_center,cv::Point2d &dot);

private:
	bool Detect(const cv::Point3f gazeDirection, const cv::Point3f eyeball_center, cv::Point3f &dot);
	bool Display(const cv::Point3f left_dot, const cv::Point3f right_dot, cv::Point2d &dot);
private:
	double display_width;
	double display_height;
	double camera2displapy;
	gte::Triangle<3, float> display0, display1;
	
};

