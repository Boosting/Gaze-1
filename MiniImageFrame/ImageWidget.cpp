#include "ImageWidget.h"
#include <QImage>
#include <QPainter>
#include <QtWidgets> 
#include <iostream>
#include "WarpingIDW.h"
#include "WarpingRBF.h"

using std::cout;
using std::endl;

ImageWidget::ImageWidget(void)
{
	ptr_image_ = new QImage();
	ptr_image_backup_ = new QImage();
	is_select_points_mode_ = false;
	is_draw_lines_ = false;
	warp_class_ = None;
}


ImageWidget::~ImageWidget(void)
{
}

void ImageWidget::paintEvent(QPaintEvent *paintevent)
{
	QPainter painter;
	painter.begin(this);

	// Draw background
	painter.setBrush(Qt::lightGray);
	QRect back_rect(0, 0, width(), height());
	painter.drawRect(back_rect);

	// Draw image
	//QRect rect = QRect( (width()-image_->width())/2, (height()-image_->height())/2, image_->width(), image_->height());
	QRect rect = QRect(0, 0, ptr_image_->width(), ptr_image_->height());
	painter.drawImage(rect, *ptr_image_); 

	// Draw control line
	QPen pen(Qt::red, 2);
	painter.setPen(pen);
	painter.drawLine(point_origin_, point_end_);
	if (points_start_.size() != 0)
	{
		for (size_t i=0; i<points_start_.size(); i++)
		{
			painter.drawLine(points_start_[i], points_end_[i]);
		}
	}
	
	painter.end();
}

void ImageWidget::mousePressEvent(QMouseEvent *mouseevent)
{
	if (is_select_points_mode_ && (mouseevent->button() == Qt::LeftButton))
	{
		is_draw_lines_ = true;
		point_origin_ = point_end_ = mouseevent->pos();
	}
}

void ImageWidget::mouseMoveEvent(QMouseEvent *mouseevent)
{
	if (is_draw_lines_)
	{
		point_end_ = mouseevent->pos();
	}
	update();
}

void ImageWidget::mouseReleaseEvent(QMouseEvent *mouseevent)
{
	if (is_draw_lines_)
	{
		is_draw_lines_ = false;
		point_end_ = mouseevent->pos();
		points_start_.push_back(point_origin_);
		points_end_.push_back(point_end_);
	}
}

void ImageWidget::Open()
{
	// Open file
	QString fileName = QFileDialog::getOpenFileName(this, tr("Read Image"), ".", tr("Images(*.bmp *.png *.jpg)"));

	// Load file
	if (!fileName.isEmpty())
	{
		ptr_image_->load(fileName);
		*(ptr_image_backup_) = *(ptr_image_);
	}

	//image_->invertPixels(QImage::InvertRgb);
	//*(image_) = image_->mirrored(true, true);
	//*(image_) = image_->rgbSwapped();
	cout<<"image size: "<<ptr_image_->width()<<' '<<ptr_image_->height()<<endl;
	update();
}

void ImageWidget::Save()
{
	SaveAs();
}

void ImageWidget::SaveAs()
{
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Image"), ".", tr("Images(*.bmp)"));
	if (filename.isNull())
	{
		return;
	}	

	ptr_image_->save(filename);
}

void ImageWidget::Invert()
{
	for (int i=0; i<ptr_image_->width(); i++)
	{
		for (int j=0; j<ptr_image_->height(); j++)
		{
			QRgb color = ptr_image_->pixel(i, j);
			ptr_image_->setPixel(i, j, qRgb(255-qRed(color), 255-qGreen(color), 255-qBlue(color)) );
		}
	}

	// equivalent member function of class QImage
	// image_->invertPixels(QImage::InvertRgb);
	update();
}

void ImageWidget::Mirror(bool ishorizontal, bool isvertical)
{
	QImage image_tmp(*(ptr_image_));
	int width = ptr_image_->width();
	int height = ptr_image_->height();

	if (ishorizontal)
	{
		if (isvertical)
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					ptr_image_->setPixel(i, j, image_tmp.pixel(width-1-i, height-1-j));
				}
			}
		} 
		else
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					ptr_image_->setPixel(i, j, image_tmp.pixel(i, height-1-j));
				}
			}
		}
		
	}
	else
	{
		if (isvertical)
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					ptr_image_->setPixel(i, j, image_tmp.pixel(width-1-i, j));
				}
			}
		}
	}

	// equivalent member function of class QImage
	//*(image_) = image_->mirrored(true, true);
	update();
}

void ImageWidget::TurnGray()
{
	for (int i=0; i<ptr_image_->width(); i++)
	{
		for (int j=0; j<ptr_image_->height(); j++)
		{
			QRgb color = ptr_image_->pixel(i, j);
			int gray_value = (qRed(color)+qGreen(color)+qBlue(color))/3;
			ptr_image_->setPixel(i, j, qRgb(gray_value, gray_value, gray_value) );
		}
	}

	update();
}

void ImageWidget::Restore()
{
	*(ptr_image_) = *(ptr_image_backup_);
	points_start_.clear();
	points_end_.clear();
	point_origin_ = point_end_ = QPoint(0, 0);
	update();
}

void ImageWidget::SelectControlPointsMode()
{
	is_select_points_mode_ = true;
}

void ImageWidget::WarpIDW()
{
	warp_class_ = IDW;
	DoWarp();
}

void ImageWidget::WarpRBF()
{
	warp_class_ =  RBF;
	DoWarp();
}

void ImageWidget::DoWarp()
{
	Warping *pwarp;
	switch (warp_class_)
	{
	case IDW: 
		pwarp = new WarpingIDW();
		break;
		
	case RBF:
		pwarp = new WarpingRBF();
		break;

	default:
		break;
	}

	pwarp->InitControlPoints(points_start_, points_end_);
	pwarp->CalculateWarping(*(ptr_image_));
	update();
}