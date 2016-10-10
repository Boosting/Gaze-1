#include "WarpingRBF.h"
#include <QImage>
#include <QColor>

#include <iostream>

using namespace std;


WarpingRBF::WarpingRBF(void)
{
}


WarpingRBF::~WarpingRBF(void)
{
}


void WarpingRBF::CalculateWarping(QImage &image)
{
	SetFixedPoints(image);

	int num = points_start_x_.size();
	
	// Assignment for coefficient matrix
	MatrixXd coe_matrix(num, num);
	InitMatrix(coe_matrix);

	// Solve coefficients
	SolveCoefficient(coe_matrix);

	// Calculate pixel
	QImage image_backup(image);
	is_points_change_.resize(image.width()*image.height(), false);
	for (int i=0; i<image.width(); i++)
	{
		for (int j=0; j<image.height(); j++)
		{
			image.setPixel(i, j, qRgb(193, 193, 193));
		}
	}
	for (int i=0; i<image.width(); i++)
	{
		for (int j=0; j<image.height(); j++)
		{
			VectorXd dis_vec(num);
			for (int k=0; k<num; k++)
			{
				double dtmp = pow(i-points_start_x_[k], 2)
					+ pow(j-points_start_y_[k], 2);
				dis_vec(k) = pow(dtmp+v_r_[k], 0.5);
			}

			int pos_x = int(alpha_x_.dot(dis_vec))+i;
			int pos_y = int(alpha_y_.dot(dis_vec))+j;

			if ( pos_x>image.width()-1 || pos_y>image.height()-1 || pos_x<0 || pos_y<0)
			{
				continue;
			}
			image.setPixel(pos_x, pos_y, image_backup.pixel(i, j));
		}
	}
}

void WarpingRBF::InitMatrix(MatrixXd& coe_matrix)
{
	int num = points_start_x_.size();
	MatrixXd dis_matrix(num, num);
	v_r_.resize(num);

	// Calculate distance matrix
	for (int i=0; i<num; i++)
	{
		for (int j=0; j<num; j++)
		{
			double dis = pow(points_start_x_[i]-points_start_x_[j], 2)
				+ pow(points_start_y_[i]-points_start_y_[j], 2);
			dis_matrix(i, j) = dis;
		}
		dis_matrix(i, i) = 2147483647;
	}

	// Calculate vector r
	for (int i=0; i<num; i++)
	{
		double r = 999999;
		for (int j=0; j<num; j++)
		{
			if (dis_matrix(i, j)<r)
			{
				r = dis_matrix(i, j);
			}
		}
		v_r_[i] = r;
	}

	// Assignment for coefficient matrix
	for (int i=0; i<num; i++)
	{
		dis_matrix(i, i) = 0;
		for (int j=0; j<num; j++)
		{
			coe_matrix(i, j) = pow(dis_matrix(i, j)+v_r_[j], 0.5);
		}
	}
}

void WarpingRBF::SolveCoefficient(const MatrixXd &coe_matrix)
{
	// Assignment for value vector
	int num = points_start_x_.size();
	VectorXd value_x(num);
	VectorXd value_y(num);
	for (int i=0; i<num; i++)
	{
		value_x[i] = points_end_x_[i]-points_start_x_[i];
		value_y[i] = points_end_y_[i]-points_start_y_[i];
	}

	// Solve coefficient
	Eigen::ColPivHouseholderQR<MatrixXd> decomposition(coe_matrix);
	alpha_x_ = decomposition.solve(value_x);
	alpha_y_ = decomposition.solve(value_y);
}
