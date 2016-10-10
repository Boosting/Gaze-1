#include "Warping.h"
#include <QPoint>
#include <QImage>
#include <iostream>

using namespace std;

Warping::Warping(void)
{
}


Warping::~Warping(void)
{
}

double Warping::Distance(QPoint p, QPoint q)
{
	double dtmp = (p.x()-q.x())*(p.x()-q.x())
		+(p.y()-q.y())*(p.y()-q.y());
	return sqrt(dtmp);
}

void Warping::InitControlPoints(vector<QPoint>& points_start, vector<QPoint>& points_end)
{
	int length = points_start.size();
	points_start_x_.resize(length);
	points_start_y_.resize(length);
	points_end_x_.resize(length);
	points_end_y_.resize(length);

	for (size_t i=0; i<points_start.size(); i++)
	{
		points_start_x_[i] = double( points_start[i].x() );
		points_start_y_[i] = double( points_start[i].y() );
		points_end_x_[i] = double( points_end[i].x() );
		points_end_y_[i] = double( points_end[i].y() );
	}
}

void Warping::CalculateWarping(QImage &image)
{

}

void Warping::SetFixedPoints(QImage &image)
{
	points_start_x_.push_back(0);
	points_start_y_.push_back(0);
	points_end_x_.push_back(0);
	points_end_y_.push_back(0);

	points_start_x_.push_back(0);
	points_start_y_.push_back(image.height()-1);
	points_end_x_.push_back(0);
	points_end_y_.push_back(image.height()-1);

	points_start_x_.push_back(image.width()-1);
	points_start_y_.push_back(0);
	points_end_x_.push_back(image.width()-1);
	points_end_y_.push_back(0);

	points_start_x_.push_back(image.width()-1);
	points_start_y_.push_back(image.height()-1);
	points_end_x_.push_back(image.width()-1);
	points_end_y_.push_back(image.height()-1);
}

int Warping::IsInControlPoints(int x, int y)
{
	for (size_t i=0; i<points_start_x_.size(); i++)
	{
	//	cout<<"test: "<<int(points_start_x_[i])<<' '<<int(points_start_y_[i])<<endl;
		if ( (x==int(points_start_x_[i])) && (y==int(points_start_y_[i])) )
		{
			return i;
		}
	}
	return -1;
}
