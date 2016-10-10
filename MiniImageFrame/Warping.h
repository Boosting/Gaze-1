#pragma once
#include <vector>
using std::vector;
class QImage;
class QPoint;

class Warping
{
public:
	Warping(void);
	virtual ~Warping(void);
	
	double Distance(QPoint p, QPoint q);
	void InitControlPoints(vector<QPoint>& points_start, vector<QPoint>& points_end);		// Get control points
	int IsInControlPoints(int x, int y);
	virtual void CalculateWarping(QImage &image);

protected:
	void SetFixedPoints(QImage &image);			//	set four corner points to be fixed

protected:
	vector<double>		points_start_x_;
	vector<double>		points_start_y_;
	vector<double>		points_end_x_;
	vector<double>		points_end_y_;
	vector<bool>		is_points_change_;
};

