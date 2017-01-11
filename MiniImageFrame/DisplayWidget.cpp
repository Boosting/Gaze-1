#include "DisplayWidget.h"



DisplayWidget::DisplayWidget()
{
	display_height = 389;
	display_width = 559;
	camera2displapy = 25;

	display0.v[0][0] = display_width / 2.0; display0.v[0][1] = camera2displapy; display0.v[0][2] = 0.0;
	display0.v[1][0] = -display_width / 2.0; display0.v[1][1] = camera2displapy; display0.v[1][2] = 0.0;
	display0.v[2][0] = display_width / 2.0; display0.v[2][1] = camera2displapy + display_height; display0.v[2][2] = 0.0;

	display1.v[0][0] = -display_width / 2.0; display1.v[0][1] = camera2displapy + display_height; display1.v[0][2] = 0.0;
	display1.v[1][0] = -display_width / 2.0; display1.v[1][1] = camera2displapy; display1.v[1][2] = 0.0;
	display1.v[2][0] = display_width / 2.0; display1.v[2][1] = camera2displapy + display_height; display1.v[2][2] = 0.0;
	image_orgin = cv::Point3f(display0.v[0][0], display0.v[0][1], display0.v[0][2]);
	u = cv::Vec3f(display0.v[1][0], display0.v[1][1], display0.v[1][2]) - cv::Vec3f(image_orgin);
	u = cv::normalize(u);
	v = cv::Vec3f(display0.v[2][0], display0.v[2][1], display0.v[2][2]) - cv::Vec3f(image_orgin);
	v = cv::normalize(v);

	resolution_width = 1920;
	resolution_height = 1080;
	std::vector<cv::Point3f>tmp;
	cv::Point3f tmp_point;
	for (int i = 0; i < 5; i++)
	{
		display_calibration_point_.push_back(tmp_point);
		display_calibration_vector_.push_back(tmp);
	}
}


DisplayWidget::~DisplayWidget()
{
}

bool DisplayWidget::DotEstimate(const cv::Point3f gazeDirection0, const cv::Point3f gazeDirection1, const cv::Point3f left_eyeball_center,
	const cv::Point3f right_eyeball_center, cv::Point2d &dot)
{

	cv::Point3f left_dot,right_dot;
	if (!Detect(gazeDirection0, left_eyeball_center, left_dot))
		return false;

	if (!Detect(gazeDirection1, right_eyeball_center, right_dot))
		return false;

	return Display(left_dot, right_dot, dot);
}

bool DisplayWidget::Detect(const cv::Point3f gazeDirection, const cv::Point3f eyeball_center, cv::Point3f &dot)
{

	//design plane for test
	gte::Plane3<float> display;
	display.normal[0] = 0; display.normal[1] = 0; display.normal[2]=1;
	display.constant = 0;


	gte::Ray<3, float> gaze_ray;
	gaze_ray.origin[0] = eyeball_center.x; gaze_ray.origin[1] = eyeball_center.y; gaze_ray.origin[2] = eyeball_center.z;
	gaze_ray.direction[0] = gazeDirection.x; gaze_ray.direction[1] = gazeDirection.y; gaze_ray.direction[2] = gazeDirection.z;

	gte::FIQuery<float, Ray3<float>, Plane3<float>> intersection;
	auto result = intersection(gaze_ray, display);
	dot = cv::Point3d(result.point[0], result.point[1], result.point[2]);


	return result.intersect;
}

bool DisplayWidget::Display(const cv::Point3f left_dot, const cv::Point3f right_dot, cv::Point2d &dot)
{
	cv::Point2d tmp;
	cv::Point3f mid = (left_dot + right_dot) / 2.0;
	//cv::Point3f mid = (left_dot + left_dot) / 2.0;

	tmp = cv::Point2f(int(u.dot(mid - image_orgin)*resolution_width/display_width), int(v.dot(mid - image_orgin)*resolution_height/display_height));
	//tmp.y *= 4;
	//tmp.x = (tmp.x - resolution_width / 2.0) * 8 + resolution_width / 2.0;
	if (tmp.x > 0 && tmp.x < resolution_width&&tmp.y>0 && tmp.y < resolution_width)
	{
		std::cout << tmp.x << " " << tmp.y << std::endl;

		dot = tmp;
		return true;
	}
	return false;
	
}


Ray<3, float> DisplayWidget::CreateRay(const cv::Point3f gazeDirection, const cv::Point3f eyeball_center)
{

	gte::Ray<3, float> gaze_ray;
	gaze_ray.origin[0] = eyeball_center.x; gaze_ray.origin[1] = eyeball_center.y; gaze_ray.origin[2] = eyeball_center.z;
	gaze_ray.direction[0] = gazeDirection.x; gaze_ray.direction[1] = gazeDirection.y; gaze_ray.direction[2] = gazeDirection.z;
	return gaze_ray;
}

void DisplayWidget::Calibration(const cv::Point3f gazeDirection0, const cv::Point3f gazeDirection1, const cv::Point3f left_eyeball_center,
	const cv::Point3f right_eyeball_center,int type)
{
	left_gaze_ray=CreateRay(gazeDirection0, left_eyeball_center);
	right_gaze_ray = CreateRay(gazeDirection1, right_eyeball_center);
	DCPQuery<float, Ray3<float>, Ray3<float>> distance;
	auto result = distance(left_gaze_ray, right_gaze_ray);
	if (result.distance < 50)
	{
		Vector<3, float> mid = (result.closestPoint[0] + result.closestPoint[1]);
		cv::Point3f dot = cv::Point3f(mid[0] * 0.5, mid[1] * 0.5, mid[2] * 0.5);
		std::cout << dot.x << " " << dot.y << " " << dot.z << std::endl;
		display_calibration_vector_[type].push_back(dot);
	}
}

void DisplayWidget::UpdateCalibration() {
	
	for (int i = 0; i < display_calibration_point_.size(); i++)
	{
		cv::Point3f tmp = cv::Point3f(0, 0, 0);
		for (int j = 0; j < display_calibration_vector_[i].size(); j++)
			tmp += display_calibration_vector_[i][j];
		tmp /= float(display_calibration_vector_[i].size());
		display_calibration_point_[i] = tmp;
		std::cout << i << " " << tmp.x << " " << tmp.y << " " << tmp.z << std::endl;
		display_calibration_vector_[i].clear();
	}
};