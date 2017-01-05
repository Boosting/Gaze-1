#include"FaceCv.h"
using namespace cv;

FaceCv::FaceCv(QObject *parent)
	: QObject(parent)
{

}

FaceCv::FaceCv(void)
{


}


FaceCv::~FaceCv()
{
}


void FaceCv::Init()
{

	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return ;

	Mat edges;
	namedWindow("gaze", 1);

	static int nImgWidth = 0;
	static int nImgHeight = 0;
	nImgWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	nImgHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	//----camera----
	camera_focal_length_= cap.get(CV_CAP_PROP_FRAME_HEIGHT);// Approximate focal length.
	camera_cols_= cap.get(CV_CAP_PROP_FRAME_WIDTH);
	camera_rows_= cap.get(CV_CAP_PROP_FRAME_HEIGHT);


	//-----face lib----
	char pcMdlPath[512];
	sprintf(pcMdlPath, "./data/model_map_2d_data.bin");
	FILE* fp = fopen(pcMdlPath, "rb");
	fseek(fp, 0, SEEK_SET);
	fseek(fp, 0, SEEK_END);
	long len_bytes = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	unsigned char* pLoadData = (unsigned char*)malloc(len_bytes);
	fread(pLoadData, sizeof(unsigned char), len_bytes, fp);
	if (!EiInitialize(nImgWidth, nImgHeight, pLoadData, len_bytes) ||
		!EiInitialize_NewReso(nImgWidth, nImgHeight))
	{
		printf("initialize failed\r\n");
		free(pLoadData);
		exit(-1);
	}
	free(pLoadData);



	//----init----
	int count = 0;
	AAM_OUTPUT_STRUCT aam_ret = { 0 };
	aam_ret.nstate = -1;
	std::vector<Point2d> control_points;
	std::vector<Point2d>nose_end_point2D;

	//----frame----
	for (;;)
	{
		Mat frame;
		cap.read(frame); // get a new frame from camera
		cv::flip(frame, frame, 1);
		cvtColor(frame, edges, COLOR_BGR2GRAY);	
		IplImage img = IplImage(edges);

		//detection
		aam_ret.nstate = EiGetExpression((unsigned char*)img.imageData, &aam_ret, true);
		if (aam_ret.nstate == 1)
		{
			Point_AAM2CV(aam_ret, control_points);
			if (control_point_seq_.size() != per_frame_)
			{
				do {
					control_point_seq_.push_back(control_points);
				} while (control_point_seq_.size() < per_frame_);
			}
			//circle
			control_point_seq_[count%per_frame_] = control_points;

			//Head Compute
			Average_Point(draw_points_);
			nose_tip_ = ((draw_points_[41] + draw_points_[42]) / 2 +draw_points_[38] + draw_points_[45]) / 3;
			//HeadCompute(draw_points_, nose_tip_, nose_end_point2D);			
			//Draw
			Draw_point(frame, nose_end_point2D);
			count++;
		}
	
		cv::imshow("gaze", frame);
		if (waitKey(30) >= 0) break;
	}
	cvDestroyWindow("gaze");
}

void FaceCv::HeadCompute(const std::vector<cv::Point2d>control_point, const cv::Point2d nose_tip, std::vector<cv::Point2d> & nose_end_point2D)
{
	nose_end_point2D.clear();

	// 2D image points. If you change the image, you need to change vector
	std::vector<cv::Point2d> image_points;
	image_points.push_back(nose_tip);    // Nose tip
	image_points.push_back(control_point[77]);    // Chin
	image_points.push_back(control_point[4]);     // Left eye left corner
	image_points.push_back(control_point[12]);    // Right eye right corner
	image_points.push_back(control_point[48]);    // Left Mouth corner
	image_points.push_back(control_point[54]);    // Right mouth corner

													  // 3D model points.
	std::vector<cv::Point3d> model_points;
	model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
	model_points.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
	model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
	model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
	model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
	model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner

																		 // Camera internals
	double focal_length = GetCameraFocal(); 
	Point2d center = cv::Point2d(GetCameraCols()/ 2, GetCameraRows()/ 2);
	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

	std::cout << "Camera Matrix " << std::endl << camera_matrix << std::endl;
	// Output rotation and translation
	cv::Mat rotation_vector; // Rotation in axis-angle form
	cv::Mat translation_vector;

	// Solve for pose
	cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);


	// Project a 3D point (0, 0, 1000.0) onto the image plane.
	// We use this to draw a line sticking out of the nose

	std::vector<cv::Point3d> nose_end_point3D;
	nose_end_point3D.push_back(Point3d(0, 0, 1000.0));
	projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);


	//std::cout << "Rotation Vector " << std::endl << rotation_vector << std::endl;
	//std::cout << "Translation Vector" << std::endl << translation_vector << std::endl;
	//std::cout << nose_end_point2D << std::endl;

}



void  FaceCv::Point_AAM2CV(const AAM_OUTPUT_STRUCT aam_ret, std::vector<cv::Point2d>& control_point)
{
	control_point.clear();
	for (int i = 0; i < 87; ++i)  //87 is a magic num
	{		
		control_point.push_back(cvPoint(int(aam_ret.pKeyPoint2DOut[i].x + 0.5f), int(aam_ret.pKeyPoint2DOut[i].y + 0.5f)));		
	}
}

void FaceCv::Average_Point(std::vector<cv::Point2d>& draw_point)
{
	draw_point.clear();
	for (int i = 0; i < control_point_seq_[0].size(); i++)
	{
		Point2d temp= Point2d(0.0,0.0);
		for (int j = 0; j < per_frame_; j++)
		{
			temp += control_point_seq_[j][i];
		}
		draw_point.push_back(temp/per_frame_);
	}

}

void FaceCv::Draw_point(cv::Mat &frame,const std::vector<cv::Point2d> nose_end_point2D)
{
	for (int i = 0; i < draw_points_.size(); ++i)  //aam_ret.n2DNum
	{
		//if(i==4||i==12||i==38 || i == 41 || i == 42 || i == 45 || i == 48 || i == 54 || i == 77)
		circle(frame, draw_points_[i], 1, cvScalar(0, 255, 0), -1);
	}
	//cv::line(frame, nose_tip_, nose_end_point2D[0], cv::Scalar(255, 0, 0), 2);
}