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

ptr_IntMatrix createTemplate(ptr_IntMatrix inputImage, ptr_Point centerPoint,
	int tsize, ptr_Point &location, ptr_Point &distance)
{
	int hsize = tsize / 2;
	int cx = centerPoint->getX(); //col
	int cy = centerPoint->getY(); // row
	if (cx < 0 || cy < 0)
	 return inputImage;
	int rows = inputImage->getRows();
	int cols = inputImage->getCols();

	int lx = (cx - hsize) < 0 ? 0 : (cx - hsize);
	int ly = (cy - hsize) < 0 ? 0 : (cy - hsize);
	location = new Point(lx, ly);
	distance = new Point(cx - lx, cy - ly);

	int rx = (cx + hsize) > cols ? cols : (cx + hsize);
	int ry = (cy + hsize) > rows ? rows : (cy + hsize);

	ptr_IntMatrix result = new Matrix<int>(ry - ly, rx - lx);
	int i = -1, j = -1;
	for (int row = ly; row < ry; row++)
	{
		i++;
		j = -1;
		for (int col = lx; col < rx; col++)
		{
			j++;
			result->setAtPosition(i, j, inputImage->getAtPosition(row, col));
		}
	}
	return result;

}
ptr_Point matCrossCorrelation(ptr_IntMatrix templ, ptr_IntMatrix image)
{
	int width = image->getCols() - templ->getCols() + 1;
	int height = image->getRows() - templ->getRows() + 1;

	int imgrows = image->getRows();
	int imgcols = image->getCols();
	int tmprows = templ->getRows();
	int tmpcols = templ->getCols();
	ptr_Point location = new Point(0, 0);

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
					location->setX(col);
					location->setY(row);
				}
			}
		}
	}
	return location;
}

vector<ptr_Point> verifyLandmarks(Image mImage, Image sImage,
	vector<ptr_Point> manualLM, vector<ptr_Point> esLandmarks, int templSize,
	int sceneSize, double angleDiff, ptr_Point ePoint)
{
	ptr_IntMatrix mMatrix = mImage.getGrayMatrix();
	ptr_IntMatrix sRotateImg = sImage.rotate(ePoint, angleDiff, 1);
	vector<ptr_Point> mcResult;
	for (size_t i = 0; i < esLandmarks.size(); i++)
	{
		//ptr_Point lm = new Point(manualLM.at(i)->getX(), manualLM.at(i)->getY());
		ptr_Point pi = esLandmarks.at(i);
		if (pi->getX() > 0 && pi->getY() > 0)
		{
			ptr_Point tLocation, tDistance, iLocation, iDistance;
			ptr_IntMatrix templ = createTemplate(mMatrix, manualLM.at(i), templSize,
				tLocation, tDistance);
			ptr_IntMatrix sceneM = createTemplate(sRotateImg, esLandmarks.at(i),
				sceneSize, iLocation, iDistance);

			ptr_Point maxLoc = matCrossCorrelation(templ, sceneM);

			int lmx = iLocation->getX() + maxLoc->getX() + tDistance->getX();
			int lmy = iLocation->getY() + maxLoc->getY() + tDistance->getY();
			mcResult.push_back(new Point(lmx, lmy));
		}
	}
	for (size_t k = 0; k < mcResult.size(); k++)
	{
		ptr_Point p = mcResult.at(k);
		cout << "\nEstimated landmark : " << p->getX() << "\t" << p->getY();
	}

	return mcResult;
}
