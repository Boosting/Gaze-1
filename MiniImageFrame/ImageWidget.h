#pragma once
#include <QWidget>
#include <QPoint>
QT_BEGIN_NAMESPACE
class QImage;
class QPainter;
QT_END_NAMESPACE

enum warpclass {IDW, RBF, None} ;
class ImageWidget :
	public QWidget
{
	Q_OBJECT

public:
	ImageWidget(void);
	~ImageWidget(void);

protected:
	void paintEvent(QPaintEvent *paintevent);
	void mousePressEvent(QMouseEvent *mouseevent);
	void mouseMoveEvent(QMouseEvent *mouseevent);
	void mouseReleaseEvent(QMouseEvent *mouseevent);

public slots:
	// File IO
	void Open();												// Open an image file, support ".bmp, .png, .jpg" format
	void Save();												
	void SaveAs();												// Save image to another file

	// Image processing
	void Invert();												// Invert pixel value in image
	void Mirror(bool horizontal=false, bool vertical=true);		// Mirror image vertically or horizontally
	void TurnGray();											// Turn image to gray-scale map
	void Restore();												// Restore image to origin

	// Image Warping
	void SelectControlPointsMode();								// Change to select mode 
	void WarpIDW();
	void WarpRBF();

private:
	void DoWarp();

private:
	QImage								*ptr_image_;				// image 
	QImage								*ptr_image_backup_;

	bool								is_select_points_mode_;
	bool								is_draw_lines_;
	std::vector<QPoint>					points_start_;		// vector of control points for warping algorithm,
	std::vector<QPoint>					points_end_;

	QPoint								point_origin_;
	QPoint								point_end_;
	warpclass							warp_class_;
};

