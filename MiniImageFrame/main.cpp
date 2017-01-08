#include "mainwindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainWindow w;
	w.SetArgc(argc, argv);
	w.show();
	w.Test();
	return a.exec();
}
