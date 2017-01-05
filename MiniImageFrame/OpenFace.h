#pragma once

#include "LandmarkCoreIncludes.h"
#include "GazeEstimation.h"
#include <fstream>
#include <sstream>

// OpenCV includes
#include <opencv2/videoio/videoio.hpp>  // Video write
#include <opencv2/videoio/videoio_c.h>  // Video write
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost includes
#include <filesystem.hpp>
#include <filesystem/fstream.hpp>
using namespace std;

class OpenFace 
{

public:

	~OpenFace();

	OpenFace(void);

	
	void Init(int argc, char **argv);

private:
	static void printErrorAndAbort(const std::string & error);
	vector<string> get_arguments(int argc, char **argv);

	// Some globals for tracking timing information for visualisation
	double fps_tracker = -1.0;
	int64 t0 = 0;
	void visualise_tracking(cv::Mat& captured_image, cv::Mat_<float>& depth_image, const LandmarkDetector::CLNF& face_model, const LandmarkDetector::FaceModelParameters& det_parameters, cv::Point3f gazeDirection0, cv::Point3f gazeDirection1, int frame_count, double fx, double fy, double cx, double cy);
	int main(int argc, char **argv);

};