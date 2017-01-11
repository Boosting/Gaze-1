#include "OpenFace.h"
// Libraries for landmark detection (includes CLNF and CLM modules)


#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl

void OpenFace::printErrorAndAbort(const std::string & error)
{
	std::cout << error << std::endl;
	abort();
}




OpenFace::OpenFace(void)
{
	displaywidget_ = new DisplayWidget();
}

OpenFace::~OpenFace()
{
}

void OpenFace::Init(int argc, char **argv)
{
	#define FATAL_STREAM( stream ) \
	printErrorAndAbort( std::string( "Fatal error: " ) + stream )

	main(argc, argv);

}

vector<string> OpenFace::get_arguments(int argc, char **argv)
{

	vector<string> arguments;

	for (int i = 0; i < argc; ++i)
	{
		arguments.push_back(string(argv[i]));
	}
	return arguments;
}

// Visualising the results
void OpenFace::visualise_tracking(cv::Mat& captured_image, cv::Mat_<float>& depth_image, const LandmarkDetector::CLNF& face_model, const LandmarkDetector::FaceModelParameters& det_parameters, cv::Point3f gazeDirection0, cv::Point3f gazeDirection1, int frame_count, double fx, double fy, double cx, double cy)
{

	// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
	double detection_certainty = face_model.detection_certainty;
	bool detection_success = face_model.detection_success;

	double visualisation_boundary = 0.2;

	// Only draw if the reliability is reasonable, the value is slightly ad-hoc
	if (detection_certainty < visualisation_boundary)
	{
		LandmarkDetector::Draw(captured_image, face_model);

		double vis_certainty = detection_certainty;
		if (vis_certainty > 1)
			vis_certainty = 1;
		if (vis_certainty < -1)
			vis_certainty = -1;

		vis_certainty = (vis_certainty + 1) / (visualisation_boundary + 1);

		// A rough heuristic for box around the face width
		int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

		cv::Vec6d pose_estimate_to_draw = LandmarkDetector::GetCorrectedPoseWorld(face_model, fx, fy, cx, cy);

		// Draw it in reddish if uncertain, blueish if certain
		//LandmarkDetector::DrawBox(captured_image, pose_estimate_to_draw, cv::Scalar((1 - vis_certainty)*255.0, 0, vis_certainty * 255), thickness, fx, fy, cx, cy);

		if (det_parameters.track_gaze && detection_success && face_model.eye_model)
		{
			FaceAnalysis::DrawGaze(captured_image, face_model, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
		}
	}

	// Work out the framerate
	if (frame_count % 10 == 0)
	{
		double t1 = cv::getTickCount();
		fps_tracker = 10.0 / (double(t1 - t0) / cv::getTickFrequency());
		t0 = t1;
	}

	// Write out the framerate on the image before displaying it
	char fpsC[255];
	std::sprintf(fpsC, "%d", (int)fps_tracker);
	string fpsSt("FPS:");
	fpsSt += fpsC;
	cv::putText(captured_image, fpsSt, cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));

	if (!det_parameters.quiet_mode)
	{
		cv::namedWindow("tracking_result", 1);
		cv::imshow("tracking_result", captured_image);

		if (!depth_image.empty())
		{
			// Division needed for visualisation purposes
			imshow("depth", depth_image / 2000.0);
		}

	}
}

int OpenFace::main(int argc, char **argv)
{

	vector<string> arguments = get_arguments(argc, argv);
	vector<string> files, depth_directories, output_video_files, out_dummy;
	LandmarkDetector::FaceModelParameters det_parameters(arguments);
	bool u;
	string output_codec;
	LandmarkDetector::get_video_input_output_params(files, depth_directories, out_dummy, output_video_files, u, output_codec, arguments);
	LandmarkDetector::CLNF clnf_model(det_parameters.model_location);


	float fx = 6.1962475585937500*100, fy = 6.1962475585937500*100, cx = 3.1217816162109375*100, cy = 2.4584956359863281*100;

	det_parameters.track_gaze = true;
	int frame_count = 0;



	//Define some parameters for the camera
	cv::Size frameSize = cv::Size(640, 480);

	cv::Size depthSize = cv::Size(640, 480);
	float frameRate = 30;

	//Draw dot on the screen
	cv::namedWindow("Estimate", CV_WINDOW_NORMAL);
	cv::Mat img(cv::Mat(1080, 1920, CV_8U));
	img = cv::Scalar(50);    // or the desired uint8_t value from 0-255
	//Create the OpenCV windows and images

	//cv::namedWindow("Color", cv::WINDOW_NORMAL);
	//cv::namedWindow("Depth", cv::WINDOW_NORMAL);

	cv::Mat frameColor = cv::Mat::zeros(frameSize, CV_8UC3);
	cv::Mat frameDepth = cv::Mat::zeros(depthSize, CV_8UC1);


	//Initialize the RealSense Manager
	PXCSenseManager* pxcSenseManager = PXCSenseManager::CreateInstance();

	//Enable the streams to be used

	pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, frameSize.width, frameSize.height, frameRate);
	pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, depthSize.width, depthSize.height, frameRate);

	//Initialize the pipeline
	pxcSenseManager->Init();
	bool keepRunning = true;
	while (keepRunning)
	{
		//Acquire all the frames from the camera
		pxcSenseManager->AcquireFrame();
		PXCCapture::Sample *sample = pxcSenseManager->QuerySample();

		//Convert each frame into an OpenCV image

		frameColor = PXCImage2CVMat(sample->color, PXCImage::PIXEL_FORMAT_RGB24);
		cv::Mat frameDepth_u16 = PXCImage2CVMat(sample->depth, PXCImage::PIXEL_FORMAT_DEPTH);
		//frameDepth_u16.convertTo(frameDepth, CV_8UC1);

		cv::Mat_<float> frameDisplay;
		frameDisplay = frameDepth_u16;
		//cv::equalizeHist(frameDepth, frameDisplay);

		//Display the images

		//cv::imshow("Color", frameColor);
		//cv::imshow("Depth", frameDisplay);

		//Check for user input
		int key = cv::waitKey(1);
		if (key == 27)
			keepRunning = false;

		//Release the memory from the frames
		pxcSenseManager->ReleaseFrame();


		// Reading the images

		cv::Mat_<uchar> grayscale_image;

		if (frameColor.channels() == 3)
		{
			cv::cvtColor(frameColor, grayscale_image, CV_BGR2GRAY);
		}
		else
		{
			grayscale_image = frameColor.clone();
		}


		// The actual facial landmark detection / tracking
		bool detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, cv::Mat_<float>(), clnf_model, det_parameters);

		// Visualising the results
		// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
		double detection_certainty = clnf_model.detection_certainty;

		// Gaze tracking, absolute gaze direction, eyeball center
		cv::Point3f gazeDirection0(0, 0, -1);
		cv::Point3f gazeDirection1(0, 0, -1);


		cv::Point3f left_eyeball_center(0, 0, -1);
		cv::Point3f right_eyeball_center(0, 0, -1);

		if (det_parameters.track_gaze && detection_success && clnf_model.eye_model)
		{
			FaceAnalysis::EstimateGaze(left_eyeball_center, clnf_model, gazeDirection0, fx, fy, cx, cy, true); //Left Eye
			FaceAnalysis::EstimateGaze(right_eyeball_center, clnf_model, gazeDirection1, fx, fy, cx, cy, false); //Right Eye
		}
		frame_count++;
		visualise_tracking(frameColor, frameDisplay, clnf_model, det_parameters, gazeDirection0, gazeDirection1, frame_count, fx, fy, cx, cy);

		cv::Point2d dot;
		if (displaywidget_->DotEstimate(gazeDirection0, gazeDirection1, left_eyeball_center, right_eyeball_center, dot))
		{
			circle(img, dot, 32.0, cv::Scalar(0, 0, 255), -1, 8);
			cv::imshow("Estimate", img);
			img = cv::Scalar(50);
		}

	}

	//Release the memory from the RealSense manager
	pxcSenseManager->Release();
	return 0;
}

int OpenFace::img_track(int argc, char **argv)
{
	return 0;

	vector<string> arguments = get_arguments(argc, argv);
	vector<string> files, depth_directories, output_video_files, out_dummy;
	LandmarkDetector::FaceModelParameters det_parameters(arguments);
	bool u;
	string output_codec;
	LandmarkDetector::get_video_input_output_params(files, depth_directories, out_dummy, output_video_files, u, output_codec, arguments);
	LandmarkDetector::CLNF clnf_model(det_parameters.model_location);


	float fx = 500, fy = 500, cx = 320, cy = 240;
	
	det_parameters.track_gaze = true;

	cv::Mat captured_image;
	cv::Mat image;
	image = cv::imread("F:\\Work\\Git\\Gaze\\img\\me.jpg", 0);
	captured_image = image;
	

	int frame_count = 0;

	// Reading the images
	cv::Mat_<float> depth_image;
	cv::Mat_<uchar> grayscale_image;

	if (captured_image.channels() == 3)
	{
		cv::cvtColor(captured_image, grayscale_image, CV_BGR2GRAY);
	}
	else
	{
		grayscale_image = captured_image.clone();
	}


	// The actual facial landmark detection / tracking
	bool detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, depth_image, clnf_model, det_parameters);

	// Visualising the results
	// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
	double detection_certainty = clnf_model.detection_certainty;

	// Gaze tracking, absolute gaze direction, eyeball center
	cv::Point3f gazeDirection0(0, 0, -1);
	cv::Point3f gazeDirection1(0, 0, -1);

	
	cv::Point3f left_eyeball_center(0, 0, -1);
	cv::Point3f right_eyeball_center(0, 0, -1);

	if (det_parameters.track_gaze && detection_success && clnf_model.eye_model)
	{
		FaceAnalysis::EstimateGaze(left_eyeball_center, clnf_model, gazeDirection0, fx, fy, cx, cy, true); //Left Eye
		FaceAnalysis::EstimateGaze(right_eyeball_center, clnf_model, gazeDirection1, fx, fy, cx, cy, false); //Right Eye
	}

	visualise_tracking(captured_image, depth_image, clnf_model, det_parameters, gazeDirection0, gazeDirection1, frame_count, fx, fy, cx, cy);
	
	cv::Point2d dot;
	if (displaywidget_->DotEstimate(gazeDirection0, gazeDirection1, left_eyeball_center, right_eyeball_center, dot))
	{
		draw_point(dot);
	}
	return 0;
}

void OpenFace::Debug()
{
	//draw_point(cv::Point2d(960, 540));

	//vector<string> arguments = get_arguments(argc, argv);
	//vector<string> files, depth_directories, output_video_files, out_dummy;
	//LandmarkDetector::FaceModelParameters det_parameters(arguments);
	//bool u;
	//string output_codec;
	//LandmarkDetector::get_video_input_output_params(files, depth_directories, out_dummy, output_video_files, u, output_codec, arguments);
	//LandmarkDetector::CLNF clnf_model(det_parameters.model_location);


	//float fx = 500, fy = 500, cx = 320, cy = 240;

	//det_parameters.track_gaze = true;
	//int frame_count = 0;

	//

	////Define some parameters for the camera
	//cv::Size frameSize = cv::Size(1920, 1080);

	//cv::Size depthSize = cv::Size(640, 480);
	////frameSize = depthSize;
	//float frameRate = 30;

	////Create the OpenCV windows and images

	//cv::namedWindow("Color", cv::WINDOW_NORMAL);
	//cv::namedWindow("Depth", cv::WINDOW_NORMAL);

	//cv::Mat frameColor = cv::Mat::zeros(frameSize, CV_8UC3);
	//cv::Mat frameDepth = cv::Mat::zeros(depthSize, CV_8UC1);


	////Initialize the RealSense Manager
	//PXCSenseManager* pxcSenseManager = PXCSenseManager::CreateInstance();

	////Enable the streams to be used

	//pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, frameSize.width, frameSize.height, frameRate);
	//pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, depthSize.width, depthSize.height, frameRate);

	////Initialize the pipeline
	//pxcSenseManager->Init();

	//bool keepRunning = true;
	//while (keepRunning)
	//{
	//	//Acquire all the frames from the camera
	//	pxcSenseManager->AcquireFrame();
	//	PXCCapture::Sample *sample = pxcSenseManager->QuerySample();

	//	//Convert each frame into an OpenCV image

	//	frameColor = PXCImage2CVMat(sample->color, PXCImage::PIXEL_FORMAT_RGB24);
	//	cv::Mat frameDepth_u16 = PXCImage2CVMat(sample->depth, PXCImage::PIXEL_FORMAT_DEPTH);
	//	frameDepth_u16.convertTo(frameDepth, CV_8UC1);

	//	cv::Mat_<float> frameDisplay;
	//	cv::equalizeHist(frameDepth, frameDisplay);

	//	//Display the images

	//	cv::imshow("Color", frameColor);
	//	cv::imshow("Depth", frameDisplay);

	//	//Check for user input
	//	int key = cv::waitKey(1);
	//	if (key == 27)
	//		keepRunning = false;

	//	//Release the memory from the frames
	//	pxcSenseManager->ReleaseFrame();


	//	// Reading the images

	//	cv::Mat_<uchar> grayscale_image;

	//	if (frameColor.channels() == 3)
	//	{
	//		cv::cvtColor(frameColor, grayscale_image, CV_BGR2GRAY);
	//	}
	//	else
	//	{
	//		grayscale_image = frameColor.clone();
	//	}


	//	// The actual facial landmark detection / tracking
	//	bool detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, frameDisplay, clnf_model, det_parameters);

	//	// Visualising the results
	//	// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
	//	double detection_certainty = clnf_model.detection_certainty;

	//	// Gaze tracking, absolute gaze direction, eyeball center
	//	cv::Point3f gazeDirection0(0, 0, -1);
	//	cv::Point3f gazeDirection1(0, 0, -1);


	//	cv::Point3f left_eyeball_center(0, 0, -1);
	//	cv::Point3f right_eyeball_center(0, 0, -1);

	//	if (det_parameters.track_gaze && detection_success && clnf_model.eye_model)
	//	{
	//		FaceAnalysis::EstimateGaze(left_eyeball_center, clnf_model, gazeDirection0, fx, fy, cx, cy, true); //Left Eye
	//		FaceAnalysis::EstimateGaze(right_eyeball_center, clnf_model, gazeDirection1, fx, fy, cx, cy, false); //Right Eye
	//	}
	//	frame_count++;
	//	visualise_tracking(frameColor, frameDisplay, clnf_model, det_parameters, gazeDirection0, gazeDirection1, frame_count, fx, fy, cx, cy);

	//	cv::Point2d dot;
	//	if (displaywidget_->DotEstimate(gazeDirection0, gazeDirection1, left_eyeball_center, right_eyeball_center, dot))
	//	{
	//		draw_point(dot);
	//	}

	//}

	////Release the memory from the RealSense manager
	//pxcSenseManager->Release();
}

void OpenFace::draw_point(cv::Point center)
{
	cv::namedWindow("estimate",CV_WINDOW_NORMAL);
	cv::Mat img = cv::cvarrToMat(cvCreateImage(cvSize(1920, 1080), 8, 3));
	circle(img, center, 32.0, cv::Scalar(0, 0, 255), -1, 8);
	cv::imshow("estimate", img);

}


void OpenFace::IntelRealSence()
{
	//Define some parameters for the camera
	cv::Size frameSize = cv::Size(1920, 1080);

	cv::Size depthSize = cv::Size(640, 480);
	//frameSize = depthSize;
	float frameRate = 30;

	//Create the OpenCV windows and images

	cv::namedWindow("Color", cv::WINDOW_NORMAL);
	cv::namedWindow("Depth", cv::WINDOW_NORMAL);

	cv::Mat frameColor = cv::Mat::zeros(frameSize, CV_8UC3);
	cv::Mat frameDepth = cv::Mat::zeros(depthSize, CV_8UC1);


	//Initialize the RealSense Manager
	PXCSenseManager* pxcSenseManager = PXCSenseManager::CreateInstance();

	//Enable the streams to be used

	pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, frameSize.width, frameSize.height, frameRate);
	pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, depthSize.width, depthSize.height, frameRate);

	//Initialize the pipeline
	pxcSenseManager->Init();

	bool keepRunning = true;
	while (keepRunning)
	{
		//Acquire all the frames from the camera
		pxcSenseManager->AcquireFrame();
		PXCCapture::Sample *sample = pxcSenseManager->QuerySample();

		//Convert each frame into an OpenCV image

		frameColor = PXCImage2CVMat(sample->color, PXCImage::PIXEL_FORMAT_RGB24);
		cv::Mat frameDepth_u16 = PXCImage2CVMat(sample->depth, PXCImage::PIXEL_FORMAT_DEPTH);
		frameDepth_u16.convertTo(frameDepth, CV_8UC1);

		cv::Mat frameDisplay;
		cv::equalizeHist(frameDepth, frameDisplay);

		//Display the images

		cv::imshow("Color", frameColor);
		cv::imshow("Depth", frameDisplay);

		//Check for user input
		int key = cv::waitKey(1);
		if (key == 27)
			keepRunning = false;

		//Release the memory from the frames
		pxcSenseManager->ReleaseFrame();
	}

	//Release the memory from the RealSense manager
	pxcSenseManager->Release();
}


void OpenFace::InitOpenface(int argc, char **argv)
{


}


cv::Mat OpenFace::PXCImage2CVMat(PXCImage *pxcImage, PXCImage::PixelFormat format)
{
	PXCImage::ImageData data;
	pxcImage->AcquireAccess(PXCImage::ACCESS_READ, format, &data);

	int width = pxcImage->QueryInfo().width;
	int height = pxcImage->QueryInfo().height;
	if (!format)
		format = pxcImage->QueryInfo().format;

	int type;
	if (format == PXCImage::PIXEL_FORMAT_Y8)
		type = CV_8UC1;
	else if (format == PXCImage::PIXEL_FORMAT_RGB24)
		type = CV_8UC3;
	else if (format == PXCImage::PIXEL_FORMAT_DEPTH_F32)
		type = CV_32FC1;
	else if (format == PXCImage::PIXEL_FORMAT_DEPTH)
		type = CV_16UC1;

	cv::Mat ocvImage = cv::Mat(cv::Size(width, height), type, data.planes[0]);


	pxcImage->ReleaseAccess(&data);
	return ocvImage;
}

void OpenFace::Calibration(int argc, char **argv,int type)
{

	vector<string> arguments = get_arguments(argc, argv);
	vector<string> files, depth_directories, output_video_files, out_dummy;
	LandmarkDetector::FaceModelParameters det_parameters(arguments);
	bool u;
	string output_codec;
	LandmarkDetector::get_video_input_output_params(files, depth_directories, out_dummy, output_video_files, u, output_codec, arguments);
	LandmarkDetector::CLNF clnf_model(det_parameters.model_location);
	float fx = 6.1962475585937500 * 100, fy = 6.1962475585937500 * 100, cx = 3.1217816162109375 * 100, cy = 2.4584956359863281 * 100;
	det_parameters.track_gaze = true;
	int frame_count = 0;
	cv::Size frameSize = cv::Size(640, 480);
	float frameRate = 30;


	cv::Mat frameColor = cv::Mat::zeros(frameSize, CV_8UC3);
	PXCSenseManager* pxcSenseManager = PXCSenseManager::CreateInstance();
	pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, frameSize.width, frameSize.height, frameRate);
	pxcSenseManager->Init();
	bool keepRunning = true;

	int start_time = std::time(NULL);
	int run_time = start_time;
	while (keepRunning)
	{
		run_time = std::time(NULL) - start_time;

		if (run_time < 3)
			continue;
		if (run_time > 6)
			keepRunning = false;

		pxcSenseManager->AcquireFrame();
		PXCCapture::Sample *sample = pxcSenseManager->QuerySample();
		frameColor = PXCImage2CVMat(sample->color, PXCImage::PIXEL_FORMAT_RGB24);
		cv::Mat_<float> frameDisplay;


		pxcSenseManager->ReleaseFrame();
		cv::Mat_<uchar> grayscale_image;

		if (frameColor.channels() == 3)
		{
			cv::cvtColor(frameColor, grayscale_image, CV_BGR2GRAY);
		}
		else
		{
			grayscale_image = frameColor.clone();
		}
		bool detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, cv::Mat_<float>(), clnf_model, det_parameters);

		double detection_certainty = clnf_model.detection_certainty;

		// Gaze tracking, absolute gaze direction, eyeball center
		cv::Point3f gazeDirection0(0, 0, -1);
		cv::Point3f gazeDirection1(0, 0, -1);
		cv::Point3f left_eyeball_center(0, 0, -1);
		cv::Point3f right_eyeball_center(0, 0, -1);

		if (det_parameters.track_gaze && detection_success && clnf_model.eye_model)
		{
			FaceAnalysis::EstimateGaze(left_eyeball_center, clnf_model, gazeDirection0, fx, fy, cx, cy, true); //Left Eye
			FaceAnalysis::EstimateGaze(right_eyeball_center, clnf_model, gazeDirection1, fx, fy, cx, cy, false); //Right Eye
		}
		frame_count++;
		//visualise_tracking(frameColor, frameDisplay, clnf_model, det_parameters, gazeDirection0, gazeDirection1, frame_count, fx, fy, cx, cy);
		displaywidget_->Calibration(gazeDirection0, gazeDirection1, left_eyeball_center,right_eyeball_center,type);

	}

	
	pxcSenseManager->Release();
	cv::destroyWindow("estimate");
}

void OpenFace::Cali(int argc, char **argv)
{
	draw_point(cv::Point2d(960, 540));
	Calibration(argc, argv,0);
	draw_point(cv::Point2d(960, 50));
	Calibration(argc, argv, 1);
	draw_point(cv::Point2d(960, 1040));
	Calibration(argc, argv, 2);
	draw_point(cv::Point2d(50, 540));
	Calibration(argc, argv, 3);
	draw_point(cv::Point2d(1870, 540));
	Calibration(argc, argv, 4);

	displaywidget_->UpdateCalibration();
}