#include "mainwindow.h"
#include <QtWidgets>
#include <QImage>
#include <QPainter>
#include "ImageWidget.h"


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	//ui.setupUi(this);

	setGeometry(300, 150, 800, 450);

	imagewidget_ = new ImageWidget();
	setCentralWidget(imagewidget_);

	facecv_ = new FaceCv();
	openface_ = new OpenFace();

	CreateActions();
	CreateMenus();
	CreateToolBars();
	CreateStatusBar();
}

MainWindow::~MainWindow()
{

}

void MainWindow::closeEvent(QCloseEvent *e)
{

}

void MainWindow::paintEvent(QPaintEvent* paintevent)
{
	
}

void MainWindow::CreateActions()
{
	action_new_ = new QAction(QIcon(":/MainWindow/Resources/images/new.png"), tr("&New"), this);
	action_new_->setShortcut(QKeySequence::New);
	action_new_->setStatusTip(tr("Create a new file"));

	action_open_ = new QAction(QIcon(":/MainWindow/Resources/images/open.png"), tr("&Open..."), this);
	action_open_->setShortcuts(QKeySequence::Open);
	action_open_->setStatusTip(tr("Open an existing file"));
	connect(action_open_, SIGNAL(triggered()), imagewidget_, SLOT(Open()));

	action_save_ = new QAction(QIcon(":/MainWindow/Resources/images/save.png"), tr("&Save"), this);
	action_save_->setShortcuts(QKeySequence::Save);
	action_save_->setStatusTip(tr("Save the document to disk"));

	action_saveas_ = new QAction(tr("Save &As..."), this);
	action_saveas_->setShortcuts(QKeySequence::SaveAs);
	action_saveas_->setStatusTip(tr("Save the document under a new name"));
	connect(action_saveas_, SIGNAL(triggered()), imagewidget_, SLOT(SaveAs()));

	action_invert_ = new QAction(tr("Inverse"), this);
	action_invert_->setStatusTip(tr("Invert all pixel value in the image"));
	connect(action_invert_, SIGNAL(triggered()), imagewidget_, SLOT(Invert()));

	action_mirror_ = new QAction(tr("Mirror"), this);
	action_mirror_->setStatusTip(tr("Mirror image vertically or horizontally"));
	connect(action_mirror_, SIGNAL(triggered()), imagewidget_, SLOT(Mirror()));

	action_gray_ = new QAction(tr("Grayscale"), this);
	action_gray_->setStatusTip(tr("Gray-scale map"));
	connect(action_gray_, SIGNAL(triggered()), imagewidget_, SLOT(TurnGray()));

	action_restore_ = new QAction(tr("Restore"), this);
	action_restore_->setStatusTip(tr("Show origin image"));
	connect(action_restore_, SIGNAL(triggered()), imagewidget_, SLOT(Restore()));

	action_controlpoints_ = new QAction(tr("ControlPoints"), this);
	action_controlpoints_->setStatusTip(tr("Select control points"));
	connect(action_controlpoints_, SIGNAL(triggered()), imagewidget_, SLOT(SelectControlPointsMode()));

	action_warping_idw_ = new QAction(tr("WarpingIDW"), this);
	action_warping_idw_->setStatusTip(tr("Warp image with IDW method"));
	connect(action_warping_idw_, SIGNAL(triggered()), imagewidget_, SLOT(WarpIDW()));

	action_warping_rbf_ = new QAction(tr("WarpingRBF"), this);
	action_warping_rbf_->setStatusTip(tr("Warp iamge with RBF method"));
	connect(action_warping_rbf_, SIGNAL(triggered()), imagewidget_, SLOT(WarpRBF()));

	action_vision_ = new QAction(tr("Vision"), this);
	action_vision_->setStatusTip(tr("Gaze Detection"));
	connect(action_vision_, SIGNAL(triggered()), facecv_, SLOT(Init()));

	action_openface_ = new QAction(tr("OpenFace"), this);
	action_openface_->setStatusTip(tr("Test Gaze"));
	connect(action_openface_, SIGNAL(triggered()), this, SLOT(OpenFaceData()));
}

void MainWindow::CreateMenus()
{
	menu_file_ = menuBar()->addMenu(tr("&File"));
	menu_file_->setStatusTip(tr("File menu"));
	menu_file_->addAction(action_new_);
	menu_file_->addAction(action_open_);
	menu_file_->addAction(action_save_);
	menu_file_->addAction(action_saveas_);




	menu_edit_ = menuBar()->addMenu(tr("&Edit"));
	menu_edit_->setStatusTip(tr("Edit menu"));
	menu_edit_->addAction(action_invert_);
	menu_edit_->addAction(action_mirror_);
	menu_edit_->addAction(action_gray_);
	menu_edit_->addAction(action_restore_);

	menu_warping_ = menuBar()->addMenu(tr("&Warping"));
	menu_warping_->setStatusTip(tr("Warping menu"));
	menu_warping_->addAction(action_controlpoints_);

	menu_vision_ = menuBar()->addMenu(tr("&Vision"));
	menu_vision_->setStatusTip(tr("Face Stream"));
	menu_vision_->addAction(action_vision_);

	menu_vision_ = menuBar()->addMenu(tr("&OpenFace"));
	menu_vision_->setStatusTip(tr("OpenFace"));
	menu_vision_->addAction(action_openface_);
}

void MainWindow::CreateToolBars()
{
	toolbar_file_ = addToolBar(tr("File"));
	toolbar_file_->addAction(action_new_);
	toolbar_file_->addAction(action_open_);
	toolbar_file_->addAction(action_save_);

	// Add separator in toolbar 
	toolbar_file_->addSeparator();
	toolbar_file_->addAction(action_invert_);
	toolbar_file_->addAction(action_mirror_);
	toolbar_file_->addAction(action_gray_);
	toolbar_file_->addAction(action_restore_);

	// Warping 
	toolbar_file_->addSeparator();
	toolbar_file_->addAction(action_controlpoints_);
	toolbar_file_->addAction(action_warping_idw_);
	toolbar_file_->addAction(action_warping_rbf_);

	//Vision
	toolbar_file_->addSeparator();
	toolbar_file_->addAction(action_vision_);
	toolbar_file_->addAction(action_openface_);
}

void MainWindow::CreateStatusBar()
{
	statusBar()->showMessage(tr("Ready"));
}



