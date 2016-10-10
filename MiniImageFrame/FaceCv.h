#pragma once
#include <QObject>
#include <stdio.h>
#include <cv.h>  
#include <cxcore.h>  
#include <highgui.h>  
#include "opencv2/opencv.hpp"
#include "FaceTrackingDll.h"
#include"aam_interface_struct.h"

#pragma comment(lib,"../bin/FaceTrackingDll.lib")


class FaceCv : public QObject
{
	Q_OBJECT

public:
	FaceCv(QObject *parent);
	FaceCv(void);

	~FaceCv();

	public slots:
	void Init();
	double GetCameraFocal() { return camera_focal_length_; };
	double GetCameraCols() { return camera_cols_; };
	double GetCameraRows() { return camera_rows_; };


private:
	void HeadCompute(const std::vector<cv::Point2d>control_point, const cv::Point2d nose_tip, std::vector<cv::Point2d>& nose_end_point2D);
	void Point_AAM2CV(const AAM_OUTPUT_STRUCT aam_ret, std::vector<cv::Point2d>& control_point); //tranfer struct
	void Average_Point(std::vector<cv::Point2d>& draw_point);
	void Draw_point(cv::Mat &frame, const std::vector<cv::Point2d> nose_end_point2D);


private:
	double camera_focal_length_;
	double camera_cols_;
	double camera_rows_;

	const int per_frame_=5; //control draw point frame rate;
	std::vector<std::vector<cv::Point2d>> control_point_seq_;
	std::vector<cv::Point2d>draw_points_;
	cv::Point2d nose_tip_;
};
