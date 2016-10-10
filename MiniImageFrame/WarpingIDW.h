#pragma once
#include "warping.h"

class QImage;

class WarpingIDW :
	public Warping
{
public:
	WarpingIDW(void);
	~WarpingIDW(void);

	void CalculateWarping(QImage &image);
	void FillHoles(QImage &image);
	
};

