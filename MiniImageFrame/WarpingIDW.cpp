#include "WarpingIDW.h"
#include <QImage>
#include <QColor>
#include <iostream>

using namespace std;

WarpingIDW::WarpingIDW(void)
{
}


WarpingIDW::~WarpingIDW(void)
{
}

void WarpingIDW::CalculateWarping(QImage &image)
{
	SetFixedPoints(image);

	QImage image_backup(image);
	int num = points_start_x_.size();

	for (int i=0; i<image.width(); i++)
	{
		for (int j=0; j<image.height(); j++)
		{
			image.setPixel(i, j, qRgb(193, 193, 193));		//	change backgroud color to gray
		}
	}

	for (int i=0; i<image.width(); i++)
	{
		for (int j=0; j<image.height(); j++)
		{
			vector<double> distance(num);
			double dist_sum = 0;
			
			int indice = IsInControlPoints(i, j);
			
			if (-1 != indice)
			{
				image.setPixel(points_end_x_[indice], points_end_y_[indice], image_backup.pixel(i, j));
			}
			else
			{
				for (int k=0; k<num; k++)
				{
					QPoint p_origin(i, j);
					QPoint p_result(points_start_x_[k], points_start_y_[k]);
					double dtmp = 1.0/Distance(p_origin, p_result);
					double dist_single = pow(dtmp, 2);
	
					distance[k] = dist_single;
					dist_sum += dist_single;
				}

				double dx = 0;
				double dy = 0;
				for (int k=0; k<num; k++)
				{
					dx += distance[k]/dist_sum*(points_end_x_[k]-points_start_x_[k]);
					dy += distance[k]/dist_sum*(points_end_y_[k]-points_start_y_[k]);
				}

				int pos_x = int(dx)+i;
				int pos_y = int(dy)+j;

				if ( pos_x>image.width()-1 || pos_y>image.height()-1 || pos_x<0 || pos_y<0)
				{
					continue;
				}
				image.setPixel(pos_x, pos_y, image_backup.pixel(i, j));
			}		
		}
	}
}

void WarpingIDW::FillHoles(QImage &image)
{
	int width = image.width();
	int height = image.height();
	for (int i=0; i<width; i++)
	{
		for (int j=0; j<height; j++)
		{
			if (!is_points_change_[width*j+i])
			{
				continue;
			}
		}
	}
}
