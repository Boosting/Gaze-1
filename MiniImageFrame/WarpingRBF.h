#pragma once
#include "warping.h"
#include <Eigen/dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class QImage;
class WarpingRBF :
	public Warping
{
public:
	WarpingRBF(void);
	~WarpingRBF(void);

	void CalculateWarping(QImage &image);

private:
	void InitMatrix(MatrixXd &coe_matrix);
	void SolveCoefficient(const MatrixXd &coe_matrix);

private:
	VectorXd alpha_x_;
	VectorXd alpha_y_;
	std::vector<double> v_r_;
};

