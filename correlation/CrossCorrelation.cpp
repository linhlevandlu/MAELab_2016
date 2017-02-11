/*
 * CrossCorrelation.cpp
 *
 *  Created on: Dec 2, 2016
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
#include <iomanip>
using namespace std;

#include "../imageModel/Point.h"
#include "../imageModel/Line.h"
#include "../imageModel/Matrix.h"
#include "../imageModel/Image.h"

#include "CrossCorrelation.h"

CrossCorrelation::CrossCorrelation()
{
	// TODO Auto-generated constructor stub

}

CrossCorrelation::~CrossCorrelation()
{
	// TODO Auto-generated destructor stub
}

ptr_IntMatrix createTemplate(ptr_IntMatrix inputImage, Point centerPoint,
	int tsize, Point &location, Point &distance)
{
	int hsize = tsize / 2;
	int cx = centerPoint.getX(); //col
	int cy = centerPoint.getY(); // row
	if (cx < 0 || cy < 0)
		return inputImage;
	int rows = inputImage->getRows();
	int cols = inputImage->getCols();

	int lx = (cx - hsize) < 0 ? 0 : (cx - hsize);
	int ly = (cy - hsize) < 0 ? 0 : (cy - hsize);
	location.setX(lx);
	location.setY(ly);
	distance.setX(cx - lx);
	distance.setY(cy - ly);

	int rx = (cx + hsize) >= cols ? cols - 1 : (cx + hsize);
	int ry = (cy + hsize) >= rows ? rows - 1 : (cy + hsize);

	ptr_IntMatrix result = new Matrix<int>(ry - ly, rx - lx);
	int i = -1, j = -1;
	for (int row = ly; row < ry; row++)
	{
		i++;
		j = -1;
		for (int col = lx; col < rx; col++)
		{
			j++;
			if (i >= 0 && i < ry - ly && j >= 0 && j < rx - lx)
			{
				result->setAtPosition(i, j, inputImage->getAtPosition(row, col));
			}
		}
	}
	return result;

}

Point matCrossCorrelation(ptr_IntMatrix templ, ptr_IntMatrix image)
{
	int width = image->getCols() - templ->getCols() + 1;
	int height = image->getRows() - templ->getRows() + 1;

	int imgrows = image->getRows();
	int imgcols = image->getCols();
	int tmprows = templ->getRows();
	int tmpcols = templ->getCols();
	Point location(0, 0);

	if (width > 0 && height > 0)
	{
		double maxCoeff = 0;
		for (int row = 0; row <= imgrows - tmprows; row++)
		{
			for (int col = 0; col <= imgcols - tmpcols; col++)
			{
				double acoeff = 0, bcoeff1 = 0, bcoeff2 = 0;
				for (int r = 0; r < tmprows; r++)
				{
					for (int c = 0; c < tmpcols; c++)
					{
						double te = templ->getAtPosition(r, c);
						double im = image->getAtPosition(row + r, col + c);

						acoeff += (te * im);
						bcoeff1 += (te * te);
						bcoeff2 += (im * im);
					}
				}
				double coeff;
				if (bcoeff1 == 0 || bcoeff2 == 0)
				{
					coeff = 0;
				}
				else
				{
					coeff = acoeff / sqrt(bcoeff1 * bcoeff2);
				}
				if (coeff > maxCoeff)
				{
					maxCoeff = coeff;
					location.setX(col);
					location.setY(row);
				}
			}
		}
	}
	return location;
}

vector<Point> verifyLandmarks(Image mImage, Image sImage,
	vector<Point> manualLM, vector<Point> esLandmarks, int templSize,
	int sceneSize, double angleDiff, Point ePoint)
{

	ptr_IntMatrix mMatrix = mImage.getGrayMatrix();
	//sImage.rotate(ePoint, angleDiff, 1);
	ptr_IntMatrix sRotateImg = sImage.getGrayMatrix();
	int width = mMatrix->getCols();
	int height = mMatrix->getRows();

	std::vector<Point> mcResult;
	Point epi;
	Point mpi;
	Point tLocation(0, 0);
	Point tDistance(0, 0);
	Point iLocation(0, 0);
	Point iDistance(0, 0);
	ptr_IntMatrix templ;
	ptr_IntMatrix sceneM;
	Point maxLoc(0, 0);
	for (size_t i = 0; i < esLandmarks.size(); i++)
	{
		epi = esLandmarks.at(i);
		mpi = manualLM.at(i);
		if (epi.getX() > 0 && epi.getY() > 0 && epi.getX() < width
			&& epi.getY() < height)
		{
			tLocation.setX(0);
			tLocation.setY(0);
			tDistance.setX(0);
			tDistance.setY(0);
			iLocation.setX(0);
			iLocation.setY(0);
			iDistance.setX(0);
			iDistance.setY(0);
			maxLoc.setX(0);
			maxLoc.setY(0);

			templ = createTemplate(mMatrix, mpi, templSize, tLocation, tDistance);
			sceneM = createTemplate(sRotateImg, epi, sceneSize, iLocation, iDistance);

			maxLoc = matCrossCorrelation(templ, sceneM);
			int lmx = iLocation.getX() + maxLoc.getX() + tDistance.getX();
			int lmy = iLocation.getY() + maxLoc.getY() + tDistance.getY();
			mcResult.push_back(Point(lmx, lmy));
		}

	}

	Point p;
	for (size_t k = 0; k < mcResult.size(); k++)
	{
		p = mcResult.at(k);
		cout << "\nEstimated landmark : " << p.getX() << "\t" << p.getY();

	}

	return mcResult;
}
vector<Point> verifyLandmarks2(ptr_IntMatrix mImage, ptr_IntMatrix sImage,
	vector<Point> manualLM, vector<Point> esLandmarks, int templSize,
	int sceneSize)
{
	int width = mImage->getCols();
	int height = mImage->getRows();
	ptr_IntMatrix mMatrix = new Matrix<int>(height, width, 0);
	*mMatrix = *(mImage);
	ptr_IntMatrix sMatrix = new Matrix<int>(height, width, 0);
	*sMatrix = *(sImage);

	std::vector<Point> mcResult;
	Point epi;
	Point mpi;
	Point tLocation(0, 0);
	Point tDistance(0, 0);
	Point iLocation(0, 0);
	Point iDistance(0, 0);
	ptr_IntMatrix templ;
	ptr_IntMatrix sceneM;
	Point maxLoc(0, 0);
	size_t limit = 0;
	if (manualLM.size() < esLandmarks.size())
		limit = manualLM.size();
	else
		limit = esLandmarks.size();
	for (size_t i = 0; i < limit; i++)
	{
		epi = esLandmarks.at(i);
		mpi = manualLM.at(i);
		if (epi.getX() >= 0 && epi.getY() >= 0 && epi.getX() < width
			&& epi.getY() < height)
		{
			tLocation.setX(0);
			tLocation.setY(0);
			tDistance.setX(0);
			tDistance.setY(0);
			iLocation.setX(0);
			iLocation.setY(0);
			iDistance.setX(0);
			iDistance.setY(0);
			maxLoc.setX(0);
			maxLoc.setY(0);

			templ = new Matrix<int>(
				*createTemplate(mMatrix, mpi, templSize, tLocation, tDistance));
			sceneM = new Matrix<int>(
				*createTemplate(sMatrix, epi, sceneSize, iLocation, iDistance));

			maxLoc = matCrossCorrelation(templ, sceneM);
			int lmx = iLocation.getX() + maxLoc.getX() + tDistance.getX();
			int lmy = iLocation.getY() + maxLoc.getY() + tDistance.getY();
			mcResult.push_back(Point(lmx, lmy));
			delete templ;
			delete sceneM;
		}else
		{
			mcResult.push_back(epi);
		}

	}
	delete mMatrix;
	delete sMatrix;
	return mcResult;
}

// ============================================ Test new way ===============================================================
/*
 ptr_IntMatrix createTemplate2(ptr_IntMatrix inputImage, Point centerPoint,
 int tsize, Point &tlCorn, Point &brCorner, Point &distance)
 {
 int hsize = tsize / 2;
 int cx = centerPoint.getX(); //col
 int cy = centerPoint.getY(); // row
 if (cx < 0 || cy < 0)
 return inputImage;
 int rows = inputImage->getRows();
 int cols = inputImage->getCols();

 int lx = (cx - hsize) < 0 ? 0 : (cx - hsize);
 int ly = (cy - hsize) < 0 ? 0 : (cy - hsize);
 tlCorn.setX(lx);
 tlCorn.setY(ly);
 distance.setX(cx - lx);
 distance.setY(cy - ly);
 int rx = (cx + hsize) >= cols ? cols - 1 : (cx + hsize);
 int ry = (cy + hsize) >= rows ? rows - 1 : (cy + hsize);
 brCorner.setX(rx);
 brCorner.setY(ry);
 ptr_IntMatrix result = new Matrix<int>(ry - ly, rx - lx);
 int i = -1, j = -1;
 for (int row = ly; row < ry; row++)
 {
 i++;
 j = -1;
 for (int col = lx; col < rx; col++)
 {
 j++;
 if (i >= 0 && i < ry - ly && j >= 0 && j < rx - lx)
 {
 result->setAtPosition(i, j, inputImage->getAtPosition(row, col));
 }
 }
 }
 return result;

 }
 vector<Point> getPoints(vector<Point> points, Point lCorner, Point rCorner)
 {
 vector<Point> ls;
 for (size_t i = 0; i < points.size(); i++)
 {
 int x = points.at(i).getX();
 int y = points.at(i).getY();
 if (x >= lCorner.getX() && x <= rCorner.getX() && y >= lCorner.getY()
 && y <= rCorner.getY())
 ls.push_back(Point(x, y));

 }
 return ls;
 }

 Point matCrossCorrelation2(ptr_IntMatrix templ, ptr_IntMatrix image,
 vector<Point> edgePoints,Point iLocation)
 {
 int width = image->getCols() - templ->getCols() + 1;
 int height = image->getRows() - templ->getRows() + 1;

 int imgrows = image->getRows();
 int imgcols = image->getCols();
 int tmprows = templ->getRows();
 int tmpcols = templ->getCols();
 Point location(0, 0);

 if (width > 0 && height > 0)
 {
 double maxCoeff = 0;
 for (int row = 0; row <= imgrows; row++)
 {
 for (int col = 0; col <= imgcols; col++)
 {
 if (checkPointInList(edgePoints, Point(iLocation.getX() + col,iLocation.getY() + row)))
 {
 double acoeff = 0, bcoeff1 = 0, bcoeff2 = 0;
 for (int r = 0; r < tmprows; r++)
 {
 for (int c = 0; c < tmpcols; c++)
 {
 double te = templ->getAtPosition(r, c);
 double im = 0;
 if (row + r < imgrows && col + c < imgcols)
 im = image->getAtPosition(row + r, col + c);

 acoeff += (te * im);
 bcoeff1 += (te * te);
 bcoeff2 += (im * im);
 }
 }
 double coeff=0;
 if (bcoeff1 == 0 || bcoeff2 == 0)
 {
 coeff = 0;
 }
 else
 {
 coeff = acoeff / sqrt(bcoeff1 * bcoeff2);
 }
 if (coeff > maxCoeff)
 {
 maxCoeff = coeff;
 location.setX(col);
 location.setY(row);
 }
 }
 }
 }
 }
 return location;
 }
 vector<Point> verifyLandmarks3(ptr_IntMatrix mImage, ptr_IntMatrix sImage,
 vector<Point> manualLM, vector<Point> esLandmarks, int templSize,
 int sceneSize, vector<Point> lsEdgePoints)
 {
 int width = mImage->getCols();
 int height = mImage->getRows();
 ptr_IntMatrix mMatrix = new Matrix<int>(height, width, 0);
 *mMatrix = *(mImage);
 ptr_IntMatrix sMatrix = new Matrix<int>(height, width, 0);
 *sMatrix = *(sImage);

 std::vector<Point> mcResult;
 Point epi;
 Point mpi;
 Point tLocation(0, 0);
 Point tDistance(0, 0);
 Point tCorner(0, 0);
 Point iLocation(0, 0);
 Point iDistance(0, 0);
 Point iCorner(0, 0);
 ptr_IntMatrix templ;
 ptr_IntMatrix sceneM;
 Point maxLoc(0, 0);
 size_t limit = 0;
 if (manualLM.size() < esLandmarks.size())
 limit = manualLM.size();
 else
 limit = esLandmarks.size();
 for (size_t i = 0; i < limit; i++)
 {
 epi = esLandmarks.at(i);
 mpi = manualLM.at(i);
 if (epi.getX() >= 0 && epi.getY() >= 0 && epi.getX() < width
 && epi.getY() < height)
 {
 tLocation.setX(0);
 tLocation.setY(0);
 tDistance.setX(0);
 tDistance.setY(0);
 iLocation.setX(0);
 iLocation.setY(0);
 iDistance.setX(0);
 iDistance.setY(0);
 maxLoc.setX(0);
 maxLoc.setY(0);

 templ = new Matrix<int>(
 *createTemplate2(mMatrix, mpi, templSize, tLocation, tCorner,
 tDistance));
 sceneM = new Matrix<int>(
 *createTemplate2(sMatrix, epi, sceneSize, iLocation, iCorner,
 iDistance));
 vector<Point> edgePoints = getPoints(lsEdgePoints, iLocation, iCorner);
 cout << "\nNumber of points: " << edgePoints.size() << endl;
 maxLoc = matCrossCorrelation2(templ, sceneM, edgePoints,iLocation);
 int lmx, lmy;
 if (maxLoc.getX() != 0 && maxLoc.getY() != 0)
 {
 lmx = iLocation.getX() + maxLoc.getX() ;
 lmy = iLocation.getY() + maxLoc.getY();
 }
 else
 {
 cout<<"\nTeo..."<<endl;
 lmx = epi.getX();
 lmy = epi.getY();
 }
 mcResult.push_back(Point(lmx, lmy));
 delete templ;
 delete sceneM;
 }

 }
 delete mMatrix;
 delete sMatrix;
 return mcResult;
 }*/
