/*
 * ImageViewer.cpp
 *
 *  Created on: Dec 15, 2016
 *      Author: linh
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
using namespace std;

#include <QtGui/QApplication>
#include <QtGui/QScrollBar>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QPrintDialog>
#include <QtGui/QAction>
#include <QtGui/QMenuBar>
#include <QtGui/QPainter>
#include <QtGui/QCloseEvent>
#include <QtCore/QSettings>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QMessageBox>
#include <QtCore/QDebug>
#include <QtGui/QIcon>
#include <QtGui/qwidget.h>

#include "../imageModel/Point.h"
#include "../imageModel/Line.h"
#include "../imageModel/Edge.h"
#include "../imageModel/Matrix.h"
#include "../io/Reader.h"
#include "../segmentation/Thresholds.h"
#include "../segmentation/Canny.h"
#include "../segmentation/Suzuki.h"
#include "../imageModel/Image.h"

#include "../pht/PHTEntry.h"
#include "../pht/PHoughTransform.h"

#include "../histograms/ShapeHistogram.h"
#include "../pointInterest/Treatments.h"
#include "../pointInterest/Segmentation.h"
#include "../pointInterest/ProHoughTransform.h"
#include "../pointInterest/LandmarkDetection.h"
#include "../utils/ImageConvert.h"
#include "../utils/Drawing.h"

#include "../MAELab.h"

#include "ImageViewer.h"

void ImageViewer::createFileMenu()
{
	openAct = new QAction(QIcon("./resources/ico/open.png"), tr("&Open..."),
			this);
	openAct->setShortcuts(QKeySequence::Open);
	openAct->setStatusTip(tr("Open an existing file"));
	connect(openAct, SIGNAL(triggered()), this, SLOT(open()));

	saveAct = new QAction(QIcon("./resources/ico/save.png"), tr("&Save"), this);
	saveAct->setShortcuts(QKeySequence::Save);
	saveAct->setStatusTip(tr("Save the document to disk"));
	saveAct->setEnabled(false);
	connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));

	saveAsAct = new QAction(tr("Save &As..."), this);
	saveAsAct->setShortcuts(QKeySequence::SaveAs);
	saveAsAct->setStatusTip(tr("Save the document under a new name"));
	saveAsAct->setEnabled(false);
	connect(saveAsAct, SIGNAL(triggered()), this, SLOT(saveAs()));

	closeAct = new QAction(tr("&Close"), this);
	closeAct->setShortcut(tr("Ctrl+W"));
	closeAct->setStatusTip(tr("Close this window"));
	connect(closeAct, SIGNAL(triggered()), this, SLOT(close()));

	exitAct = new QAction(tr("E&xit"), this);
	exitAct->setShortcuts(QKeySequence::Quit);
	exitAct->setStatusTip(tr("Exit the application"));
	connect(exitAct, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));

}
void ImageViewer::createViewMenu()
{
	zoomInAct = new QAction(QIcon("./resources/ico/1uparrow.png"),
			tr("Zoom &In (25%)"), this);
	zoomInAct->setShortcut(tr("Ctrl++"));
	zoomInAct->setEnabled(false);
	connect(zoomInAct, SIGNAL(triggered()), this, SLOT(zoomIn()));

	zoomOutAct = new QAction(QIcon("./resources/ico/1downarrow.png"),
			tr("Zoom &Out (25%)"), this);
	zoomOutAct->setShortcut(tr("Ctrl+-"));
	zoomOutAct->setEnabled(false);
	connect(zoomOutAct, SIGNAL(triggered()), this, SLOT(zoomOut()));

	normalSizeAct = new QAction(tr("&Normal Size"), this);
	normalSizeAct->setShortcut(tr("Ctrl+N"));
	normalSizeAct->setEnabled(false);
	connect(normalSizeAct, SIGNAL(triggered()), this, SLOT(normalSize()));

	fitToWindowAct = new QAction(tr("&Fit to Window"), this);
	fitToWindowAct->setEnabled(false);
	fitToWindowAct->setCheckable(true);
	fitToWindowAct->setShortcut(tr("Ctrl+J"));
	connect(fitToWindowAct, SIGNAL(triggered()), this, SLOT(fitToWindow()));

	displayMLandmarksAct = new QAction(tr("&Display manual landmarks"), this);
	displayMLandmarksAct->setEnabled(false);
	displayMLandmarksAct->setCheckable(true);
	displayMLandmarksAct->setChecked(false);
	connect(displayMLandmarksAct, SIGNAL(triggered()), this,
			SLOT(displayManualLandmarks()));

	displayALandmarksAct = new QAction(tr("&Display estimated landmarks"),
			this);
	displayALandmarksAct->setEnabled(false);
	displayALandmarksAct->setCheckable(true);
	connect(displayALandmarksAct, SIGNAL(triggered()), this,
			SLOT(displayAutoLandmarks()));

	displayBoundingBoxAct = new QAction(tr("&Bounding box detection"), this);
	displayBoundingBoxAct->setEnabled(false);
	displayBoundingBoxAct->setCheckable(true);
	connect(displayBoundingBoxAct, SIGNAL(triggered()), this,
			SLOT(detectBoundingBox()));

}
void ImageViewer::viewMenuUpdateActions()
{
	zoomInAct->setEnabled(!fitToWindowAct->isChecked());
	zoomOutAct->setEnabled(!fitToWindowAct->isChecked());
	normalSizeAct->setEnabled(!fitToWindowAct->isChecked());
}

void ImageViewer::scaleImage(double factor)
{
	Q_ASSERT(imageLabel->pixmap());
	scaleFactor *= factor;
	imageLabel->resize(scaleFactor * imageLabel->pixmap()->size());

	adjustScrollBar(scrollArea->horizontalScrollBar(), factor);
	adjustScrollBar(scrollArea->verticalScrollBar(), factor);

	zoomInAct->setEnabled(scaleFactor < 3.0);
	zoomOutAct->setEnabled(scaleFactor > 0.333);
}
void ImageViewer::adjustScrollBar(QScrollBar *scrollBar, double factor)
{
	scrollBar->setValue(
			int(
					factor * scrollBar->value()
							+ ((factor - 1) * scrollBar->pageStep() / 2)));
}

void ImageViewer::createHelpMenu()
{
	aboutAct = new QAction(tr("&About MAELab"), this);
	connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
}
void ImageViewer::createSegmentationMenu()
{
	binaryThresholdAct = new QAction(tr("&Binary threshold"), this);
	binaryThresholdAct->setEnabled(false);
	connect(binaryThresholdAct, SIGNAL(triggered()), this,
			SLOT(binThreshold()));

	cannyAct = new QAction(tr("&Canny"), this);
	cannyAct->setEnabled(false);
	connect(cannyAct, SIGNAL(triggered()), this, SLOT(cannyAlgorithm()));
}
void ImageViewer::createLandmarksMenu()
{
	phtAct = new QAction(tr("&Probabilistic Hough Transform"), this);
	phtAct->setEnabled(false);
	connect(phtAct, SIGNAL(triggered()), this, SLOT(estimatedLandmarks()));

	autoLandmarksAct = new QAction(tr("&Compute automatic landmarks"), this);
	autoLandmarksAct->setEnabled(false);
	connect(autoLandmarksAct, SIGNAL(triggered()), this,
			SLOT(extractLandmarks()));

	measureMBaryAct = new QAction(tr("&Measure manual centroid"), this);
	measureMBaryAct->setEnabled(false);
	connect(measureMBaryAct, SIGNAL(triggered()), this, SLOT(measureMBary()));

	measureEBaryAct = new QAction(tr("&Measure estimated centroid"), this);
	measureEBaryAct->setEnabled(false);
	connect(measureEBaryAct, SIGNAL(triggered()), this, SLOT(measureEBary()));

	dirAutoLandmarksAct = new QAction(
			tr("Compute automatic landmarks on folder"), this);
	dirAutoLandmarksAct->setEnabled(false);
	connect(dirAutoLandmarksAct, SIGNAL(triggered()), this,
			SLOT(dirAutoLandmarks()));

	dirCentroidMeasureAct = new QAction(tr("Measure centroid on folder"), this);
	dirCentroidMeasureAct->setEnabled(false);
	connect(dirCentroidMeasureAct, SIGNAL(triggered()), this,
			SLOT(dirCentroidMeasure()));
}
void ImageViewer::createActions()
{
	createFileMenu();
	createViewMenu();
	createHelpMenu();
	createSegmentationMenu();
	createLandmarksMenu();
}
void ImageViewer::createMenus()
{
	fileMenu = new QMenu(tr("&File"), this);
	fileMenu->addAction(openAct);
	fileMenu->addAction(saveAct);
	fileMenu->addAction(saveAsAct);
	fileMenu->addSeparator();
	fileMenu->addAction(closeAct);
	fileMenu->addAction(exitAct);

	viewMenu = new QMenu(tr("&View"), this);
	viewMenu->addAction(zoomInAct);
	viewMenu->addAction(zoomOutAct);
	viewMenu->addSeparator();
	viewMenu->addAction(normalSizeAct);
	viewMenu->addAction(fitToWindowAct);
	viewMenu->addSeparator();
	viewMenu->addAction(displayMLandmarksAct);
	viewMenu->addAction(displayALandmarksAct);
	viewMenu->addSeparator();
	viewMenu->addAction(displayBoundingBoxAct);

	helpMenu = new QMenu(tr("&Helps"), this);
	helpMenu->addAction(aboutAct);

	segmentationMenu = new QMenu(tr("&Segmentation"), this);
	segmentationMenu->addAction(binaryThresholdAct);
	segmentationMenu->addAction(cannyAct);

	dominantPointMenu = new QMenu(tr("&Landmarks"), this);
	dominantPointMenu->addAction(phtAct);
	dominantPointMenu->addAction(autoLandmarksAct);
	dominantPointMenu->addSeparator();
	dominantPointMenu->addAction(measureMBaryAct);
	dominantPointMenu->addAction(measureEBaryAct);
	dominantPointMenu->addSeparator();
	QMenu* menuDirectory = dominantPointMenu->addMenu(
			tr("Working on directory"));
	menuDirectory->addAction(dirAutoLandmarksAct);
	menuDirectory->addAction(dirCentroidMeasureAct);

	// add menus to GUI
	menuBar()->addMenu(fileMenu);
	menuBar()->addMenu(viewMenu);
	menuBar()->addMenu(segmentationMenu);
	menuBar()->addMenu(dominantPointMenu);
	menuBar()->addMenu(helpMenu);
}
void ImageViewer::createToolBars()
{
	fileToolBar = addToolBar(tr("File"));
	fileToolBar->addAction(openAct);
	fileToolBar->addAction(saveAct);

	viewToolBar = addToolBar(tr("View"));
	viewToolBar->addAction(zoomInAct);
	viewToolBar->addAction(zoomOutAct);
}
void ImageViewer::createStatusBar()
{
	statusBar()->showMessage(tr("Ready"));
}
void ImageViewer::activeFunction()
{
	openAct->setEnabled(true);
	saveAct->setEnabled(true);
	saveAsAct->setEnabled(true);

	zoomInAct->setEnabled(true);
	zoomOutAct->setEnabled(true);
	fitToWindowAct->setEnabled(true);
	normalSizeAct->setEnabled(true);
	displayMLandmarksAct->setEnabled(true);
	if (matImage->getListOfAutoLandmarks().size() > 0)
		displayALandmarksAct->setEnabled(true);
	displayBoundingBoxAct->setEnabled(true);

	binaryThresholdAct->setEnabled(true);
	cannyAct->setEnabled(true);

	phtAct->setEnabled(true);
	autoLandmarksAct->setEnabled(true);
	if (matImage->getListOfManualLandmarks().size() > 0)
		measureMBaryAct->setEnabled(true);
	if (matImage->getListOfAutoLandmarks().size() > 0)
		measureEBaryAct->setEnabled(true);
	dirAutoLandmarksAct->setEnabled(true);
	dirCentroidMeasureAct->setEnabled(true);

	viewMenuUpdateActions();
	if (!fitToWindowAct->isChecked())
		imageLabel->adjustSize();
}

ImageViewer::ImageViewer()
{
	imageLabel = new QLabel;
	imageLabel->setBackgroundRole(QPalette::Base);
	imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	imageLabel->setScaledContents(true);

	scrollArea = new QScrollArea;
	scrollArea->setBackgroundRole(QPalette::Dark);
	scrollArea->setWidget(imageLabel);
	setCentralWidget(scrollArea);

	createActions();
	createMenus();
	createToolBars();
	createStatusBar();

	setWindowTitle(tr(".: MAELab :."));
	resize(900, 700);

	setWindowIcon(QIcon("./resources/ico/ip.ico"));
	//parameterPanel = NULL;

}

ImageViewer::~ImageViewer()
{

}
void ImageViewer::loadImage(QString fn)
{

	matImage = new Image(fn.toStdString());
	qImage.load(fn);

	imageLabel->setPixmap(QPixmap::fromImage(qImage));
	scaleFactor = 1.0;

	saveAsAct->setEnabled(true);
	activeFunction();

	this->fileName = fn;
	setWindowTitle(tr("Image Viewer - ") + fileName);
	statusBar()->showMessage(tr("File loaded"), 2000);
}
void ImageViewer::loadImage(Image *_matImage, QImage _qImage, QString tt)
{
	matImage = _matImage;
	qImage = _qImage;
	imageLabel->setPixmap(QPixmap::fromImage(qImage));
	scaleFactor = 1.0;

	saveAct->setEnabled(true);
	activeFunction();

	setWindowTitle(tr("Image Viewer - ") + tt);
	statusBar()->showMessage(tr("Finished"), 2000);
}

void ImageViewer::displayLandmarks(Image *image, vector<ptr_Point> lms,
		RGB color)
{
	for (int i = 0; i < lms.size(); i++)
	{
		ptr_Point lm = lms.at(i);
		vector<ptr_Point> dPoints = drawingCircle(lm, 5, color);
		for (int k = 0; k < dPoints.size(); k++)
		{
			ptr_Point p = dPoints.at(k);
			image->getRGBMatrix()->setAtPosition(p->getY(), p->getX(),
					p->getColor());
		}
	}

}
// ========================================= File and help menu actions =================================================
void ImageViewer::about()
{
	QMessageBox::about(this, tr("About MAELab"),
			tr(
					"<p><b>MAELab</b> is a software in Computer Vision. It provides the function to "
							"segmentation and detection dominant points.</p>"));
}
void ImageViewer::open()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
			QDir::currentPath());
	if (!fileName.isEmpty())
	{
		QImage image(fileName);
		if (image.isNull())
		{
			QMessageBox::information(this, tr("MAELab"),
					tr("Cannot load %1.").arg(fileName));
			return;
		}
		if (!this->fileName.isEmpty())
		{
			ImageViewer* other = new ImageViewer;
			other->loadImage(fileName);
			other->move(x() + 40, y() + 40);
			other->show();
		} else
		{
			this->loadImage(fileName);
		}
	}
}
void ImageViewer::save()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save image"), ".",
			tr("Image Files (*.jpg)"));
	if (fileName.isEmpty())
	{
		cout << "\nCan not save the image !!";
		return;
	}
	QApplication::setOverrideCursor(Qt::WaitCursor);
	qImage.save(fileName, "JPEG");

	QApplication::restoreOverrideCursor();

	this->fileName = fileName;
	setWindowTitle(tr("MAELab - ") + fileName);

	saveAct->setEnabled(false);
	saveAsAct->setEnabled(true);

	statusBar()->showMessage(tr("File saved"), 2000);
	return;
}
void ImageViewer::saveAs()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save image"), ".",
			tr("Image Files (*.jpg)"));
	if (fileName.isEmpty())
	{
		cout << "\nCan not save the image !!";
		return;
	}
	QApplication::setOverrideCursor(Qt::WaitCursor);
	qImage.save(fileName, "JPEG");
	QApplication::restoreOverrideCursor();

	saveAct->setEnabled(true);

	statusBar()->showMessage(tr("File saved"), 2000);
	return;
}

// ========================================= View menu actions =================================================
void ImageViewer::zoomIn()
{
	scaleImage(1.25);
}
void ImageViewer::zoomOut()
{
	scaleImage(0.8);
}
void ImageViewer::normalSize()
{
	imageLabel->adjustSize();
	scaleFactor = 1.0;
}
void ImageViewer::fitToWindow()
{

	bool fitToWindow = fitToWindowAct->isChecked();
	scrollArea->setWidgetResizable(fitToWindow);
	if (!fitToWindow)
	{
		normalSize();
	}
	viewMenuUpdateActions();
}
void ImageViewer::displayManualLandmarks()
{
	cout << "\n Display the manual landmarks.\n";

	bool currentState = displayMLandmarksAct->isChecked();
	if (currentState)
	{
		if (matImage->getListOfManualLandmarks().size() <= 0)
		{
			QMessageBox msgbox;
			msgbox.setText("Select the manual landmarks file.");
			msgbox.exec();

			QString reflmPath = QFileDialog::getOpenFileName(this);
			matImage->readManualLandmarks(reflmPath.toStdString());

		}
		vector<ptr_Point> mLandmarks = matImage->getListOfManualLandmarks();
		RGB color;
		color.R = 255;
		color.G = 0;
		color.B = 0;
		displayLandmarks(matImage, mLandmarks, color);
		displayMLandmarksAct->setChecked(true);
		measureMBaryAct->setEnabled(true);
	} else
	{
		Image *img = new Image(fileName.toStdString());
		matImage->setRGBMatrix(img->getRGBMatrix());
		displayMLandmarksAct->setChecked(false);
	}

	if (displayALandmarksAct->isChecked())
	{
		vector<ptr_Point> aLM = matImage->getListOfAutoLandmarks();
		if (aLM.size() > 0)
		{
			RGB color;
			color.R = 255;
			color.G = 255;
			color.B = 0;
			displayLandmarks(matImage, aLM, color);
			displayALandmarksAct->setChecked(true);
		}
	}

	this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
			"Display manual landmarks.");
	this->show();
	cout << "\nFinish.\n";
}
void ImageViewer::displayAutoLandmarks()
{
	vector<ptr_Point> autoLM = matImage->getListOfAutoLandmarks();
	QMessageBox message;
	if (autoLM.size() <= 0)
	{
		message.setText(
				"Automatic landmarks do not exists. You need to compute them.");
		message.exec();
	} else
	{
		bool currentState = displayALandmarksAct->isChecked();
		if (currentState)
		{
			RGB color;
			color.R = 255;
			color.G = 255;
			color.B = 0;
			displayLandmarks(matImage, autoLM, color);
			displayALandmarksAct->setChecked(true);
		} else
		{
			Image *img = new Image(fileName.toStdString());
			matImage->setRGBMatrix(img->getRGBMatrix());
			displayALandmarksAct->setChecked(false);
		}

		if (displayMLandmarksAct->isChecked())
		{
			vector<ptr_Point> mLM = matImage->getListOfManualLandmarks();
			if (mLM.size() > 0)
			{
				RGB color;
				color.R = 255;
				color.G = 0;
				color.B = 0;
				displayLandmarks(matImage, mLM, color);
				displayMLandmarksAct->setChecked(true);
			}
		}
		this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
				"Display estimated landmarks.");
	}

}

void ImageViewer::detectBoundingBox()
{
	cout << "\nBounding Box detection...";
	Segmentation tr;
	tr.setRefImage(*matImage);
	vector<ptr_Point> corners = tr.boundingBox();
	if (corners.size() == 4)
	{
		RGB color;
		color.R = 255;
		color.G = color.B = 0;

		ptr_Point tl = corners.at(0);
		ptr_Point tr = corners.at(1);
		ptr_Point bl = corners.at(2);
		ptr_Point br = corners.at(3);
		for (int c = tl->getX(); c < tr->getX(); c++)
		{
			matImage->getRGBMatrix()->setAtPosition(tl->getY(), c, color);
		}
		for (int c = bl->getX(); c < br->getX(); c++)
		{
			matImage->getRGBMatrix()->setAtPosition(bl->getY(), c, color);
		}
		for (int r = tl->getY(); r < bl->getY(); r++)
		{
			matImage->getRGBMatrix()->setAtPosition(r, tl->getX(), color);
		}
		for (int r = tr->getY(); r < br->getY(); r++)
		{
			matImage->getRGBMatrix()->setAtPosition(r, tr->getX(), color);
		}
	}
	this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
			"Bounding box");
	this->show();
	QMessageBox message;
	message.setText("Finish");
	message.exec();
}
// ========================================= Segmentation menu actions =================================================
void ImageViewer::binThreshold()
{
	cout << "\nBinary thresholding...\n";
	float tValue = matImage->getThresholdValue();
	Segmentation tr; // = new Segmentation();
	tr.setRefImage(*matImage);
	ptr_IntMatrix rsMatrix = tr.threshold(tValue, 255); //binaryThreshold(matImage->getGrayMatrix(), tValue,
	//255);

	ImageViewer *other = new ImageViewer;
	other->loadImage(matImage, ptrIntToQImage(rsMatrix), "Thresholding result");
	other->show();

}
void ImageViewer::cannyAlgorithm()
{
	cout << "\nCanny Algorithm...\n";
	Segmentation tr;
	tr.setRefImage(*matImage);
	vector<ptr_Edge> edges = tr.canny();

	RGB color;
	color.R = 255;
	color.G = color.B = 0;
	for (size_t i = 0; i < edges.size(); i++)
	{
		ptr_Edge edgei = edges.at(i);
		for (int k = 0; k < edgei->getPoints().size(); k++)
		{
			ptr_Point pi = edgei->getPoints().at(k);
			matImage->getRGBMatrix()->setAtPosition(pi->getY(), pi->getX(),
					color);
		}
	}
	ImageViewer *other = new ImageViewer;
	other->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
			"Canny result");
	other->move(x() - 40, y() - 40);
	other->show();
}

// ======================================================= Landmarks Menu actions =============================================
void ImageViewer::estimatedLandmarks()
{
	cout << "\n Estimated landmarks by probabilistic hough transform.\n";
	QMessageBox msgbox;

	msgbox.setText("Select the model image.");
	msgbox.exec();

	QString fileName2 = QFileDialog::getOpenFileName(this);
	if (fileName2.isEmpty())
		return;
	cout << endl << fileName2.toStdString() << endl;

	Image *modelImage = new Image(fileName2.toStdString());

	msgbox.setText("Select the landmark file of model image.");
	msgbox.exec();

	QString reflmPath = QFileDialog::getOpenFileName(this);
	modelImage->readManualLandmarks(reflmPath.toStdString());

	ProHoughTransform tr;
	tr.setRefImage(*modelImage);

	ptr_Point ePoint;
	double angleDiff;
	vector<ptr_Point> lms = tr.estimateLandmarks(*matImage, angleDiff, ePoint);
	cout << "\nNumber of the landmarks: " << lms.size();
	RGB color;
	color.R = 255;
	color.G = 0;
	color.B = 0;

	vector<ptr_Point> ePoints = drawingCircle(ePoint, 5, color);
	for (int k = 0; k < ePoints.size(); k++)
	{
		ptr_Point p = ePoints.at(k);
		matImage->getRGBMatrix()->setAtPosition(p->getY(), p->getX(),
				p->getColor());
	}
	color.R = 0;
	color.B = 255;
	for (int i = 0; i < lms.size(); i++)
	{
		ptr_Point lm = lms.at(i);
		vector<ptr_Point> dPoints = drawingCircle(lm, 5, color);
		for (int k = 0; k < dPoints.size(); k++)
		{
			ptr_Point p = dPoints.at(k);
			matImage->getRGBMatrix()->setAtPosition(p->getY(), p->getX(),
					p->getColor());
		}
	}
	this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
			"PHT result");
	this->show();
	msgbox.setText("Finish");
	msgbox.exec();
}

void ImageViewer::extractLandmarks()
{
	cout << "\n Automatic extraction the landmarks.\n";
	QMessageBox msgbox;

	msgbox.setText("Select the model image.");
	msgbox.exec();

	QString fileName2 = QFileDialog::getOpenFileName(this);
	if (fileName2.isEmpty())
		return;
	cout << endl << fileName2.toStdString() << endl;

	Image *modelImage = new Image(fileName2.toStdString());

	msgbox.setText("Select the landmark file of model image.");
	msgbox.exec();

	QString reflmPath = QFileDialog::getOpenFileName(this);
	modelImage->readManualLandmarks(reflmPath.toStdString());

	LandmarkDetection tr;
	tr.setRefImage(*modelImage);

	ptr_Point ePoint;
	double angleDiff;
	vector<ptr_Point> lms = tr.landmarksAutoDectect(*matImage, Degree, 500, 400,
			500, ePoint, angleDiff);
	cout << "\nNumber of the landmarks: " << lms.size();
	RGB color;
	color.R = 255;
	color.G = 255;
	color.B = 0;

	matImage->rotate(ePoint, angleDiff, 1);
	for (int i = 0; i < lms.size(); i++)
	{
		ptr_Point lm = lms.at(i);
		vector<ptr_Point> dPoints = drawingCircle(lm, 5, color);
		for (int k = 0; k < dPoints.size(); k++)
		{
			ptr_Point p = dPoints.at(k);
			matImage->getRGBMatrix()->setAtPosition(p->getY(), p->getX(),
					p->getColor());
		}
	}
	matImage->setAutoLandmarks(lms);
	displayALandmarksAct->setEnabled(true);
	displayALandmarksAct->setChecked(true);
	measureEBaryAct->setEnabled(true);
	this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
			"Landmarks result");
	this->show();
	cout << "\nFinish.\n";
}
void ImageViewer::measureMBary()
{
	QMessageBox qmessage;
	vector<ptr_Point> mLandmarks = matImage->getListOfManualLandmarks();
	if (mLandmarks.size() > 0)
	{
		ptr_Point ebary = new Point(0, 0);
		double mCentroid = measureCentroidPoint(mLandmarks, ebary);

		qmessage.setText(
				"<p>Coordinate of bary point: ("
						+ QString::number(ebary->getX()) + ", "
						+ QString::number(ebary->getY()) + ")</p>"
								"<p>Centroid value: "
						+ QString::number(mCentroid) + "</p");
	} else
	{
		qmessage.setText("The image has not the manual landmarks.");
	}
	qmessage.exec();
}
void ImageViewer::measureEBary()
{
	QMessageBox qmessage;
	vector<ptr_Point> mLandmarks = matImage->getListOfAutoLandmarks();
	if (mLandmarks.size() > 0)
	{
		ptr_Point ebary = new Point(0, 0);
		double mCentroid = measureCentroidPoint(mLandmarks, ebary);

		qmessage.setText(
				"<p>Coordinate of bary point: ("
						+ QString::number(ebary->getX()) + ", "
						+ QString::number(ebary->getY()) + ")</p>"
								"<p>Centroid value: "
						+ QString::number(mCentroid) + "</p");
	} else
	{
		qmessage.setText("The image has not the automatic landmarks.");
	}
	qmessage.exec();
}
void ImageViewer::dirAutoLandmarks()
{
	cout << "\n Automatic estimated landmarks on directory." << endl;
	QMessageBox msgbox;

	msgbox.setText("Select the model's landmarks file");
	msgbox.exec();
	QString lpath = QFileDialog::getOpenFileName(this);
	matImage->readManualLandmarks(lpath.toStdString());

	msgbox.setText("Selecte the scene images folder.");
	msgbox.exec();
	QString folder = QFileDialog::getExistingDirectory(this);

	msgbox.setText("Selecte the saving folder.");
	msgbox.exec();
	QString savefolder = QFileDialog::getExistingDirectory(this);

	DIR *pDir;
	struct dirent *entry;
	string filePath;
	pDir = opendir(folder.toStdString().c_str());
	if (pDir == NULL)
	{
		cout << "\n Error when reading the folder";
		return;
	}
	while (entry = readdir(pDir))
	{
		if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
		{
			filePath = folder.toStdString() + "/" + entry->d_name;
			cout << "\n" << filePath;
			Image sceneimage(filePath);

			ptr_Point ePoint = new Point(0, 0);
			double angleDiff = 0;
			///ptr_Treatments tr;
			//tr->setRefImage(*matImage);

			LandmarkDetection tr;
			tr.setRefImage(*matImage);

			vector<ptr_Point> esLandmarks = tr.landmarksAutoDectect(sceneimage,
					Degree, 500, 400, 500, ePoint, angleDiff);
			if (savefolder != NULL || savefolder != "")
			{
				string saveFile = savefolder.toStdString() + "/" + entry->d_name
						+ ".TPS";
				ofstream inFile(saveFile.c_str());
				inFile << "LM=" << esLandmarks.size() << "\n";
				for (size_t k = 0; k < esLandmarks.size(); k++)
				{
					ptr_Point pk = esLandmarks.at(k);
					inFile << pk->getX() << "\t" << pk->getY() << "\n";
				}
				inFile << "IMAGE=" << saveFile << "\n";
				inFile.close();
			}
		}
	}
	closedir(pDir);
	msgbox.setText("Finish");
	msgbox.exec();

}
void ImageViewer::dirCentroidMeasure()
{
	cout << "\n Compute centroid on directory." << endl;

	QMessageBox qmessage;
	qmessage.setText("Select the landmarks folder.");
	qmessage.exec();

	QString lmfolder = QFileDialog::getExistingDirectory(this);

	QString fileName = QFileDialog::getSaveFileName(this, tr("Save file"), ".",
			tr("Measure file (*.txt)"));
	/*qmessage.setText("Select the saving folder.");
	 qmessage.exec();

	 QString savefolder = QFileDialog::getExistingDirectory(this);*/

	DIR *pDir;
	struct dirent *entry;
	string filePath;
	pDir = opendir(lmfolder.toStdString().c_str());
	if (pDir == NULL)
	{
		cout << "\n Error when reading the folder";
		return;
	}
	string saveFile;
	ofstream inFile;
	if (fileName != NULL || fileName != "")
	{
		string saveFile = fileName.toStdString();
		inFile.open(saveFile.c_str(), std::ofstream::out);
	}
	while (entry = readdir(pDir))
	{
		if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
		{
			filePath = lmfolder.toStdString() + "/" + entry->d_name;
			cout << "\n" << filePath;
			matImage->readManualLandmarks(filePath);
			vector<ptr_Point> mLandmarks = matImage->getListOfManualLandmarks();
			if (mLandmarks.size() > 0)
			{
				cout << "\nHow";
				ptr_Point ebary = new Point(0, 0);
				double mCentroid = measureCentroidPoint(mLandmarks, ebary);

				if (fileName != NULL || fileName != "")
				{
					inFile << entry->d_name << "\t" << ebary->getX() << "\t"
							<< ebary->getY() << "\t" << mCentroid << "\n";

				}
			}
		}
	}
	inFile.close();
	closedir(pDir);
	qmessage.setText("Finish.");
	qmessage.exec();
}

