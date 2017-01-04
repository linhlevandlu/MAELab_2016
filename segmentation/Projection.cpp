/*
 * Projection.cpp
 *
 *  Created on: Jan 2, 2017
 *      Author: linh
 */
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <string.h>

using namespace std;

#include "../imageModel/Point.h"
#include "../imageModel/Matrix.h"
#include "Projection.h"

/*
 * viet 4 ham
 * 0. Quantization image
 * 1. horizontal projection
 * 2. vetical projection
 * 3. analysis histogram
 * 4. detecting bounding box
 */

// gray level -> number of bits which used to store value 1<= graylevel <= 8
ptr_IntMatrix quantization(ptr_IntMatrix grayMatrix, int graylevel)
{
	int rows = grayMatrix->getRows();
	int cols = grayMatrix->getCols();
	ptr_IntMatrix result = new Matrix<int>(rows, cols, 0);
	int mask = pow(2.0, 8 - graylevel);
	if (graylevel == 8)
		return grayMatrix;
	int tvalue = 0;
	int newvalue = 0;
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			tvalue = grayMatrix->getAtPosition(r, c);
			newvalue = tvalue / mask;
			result->setAtPosition(r, c, newvalue);
		}
	}
	return result;
}

// projection on columns
int* histogramProjection(ptr_IntMatrix binaryMatrix, PROJECTION_TYPE pType,
		int &size)
{
	int brows = binaryMatrix->getRows(); // number of rows of input matrix
	int bcols = binaryMatrix->getCols(); // number of columns of input matrix

	int rows, cols; // rows and cols for process
	if (pType == Vertical) // vertical projection
	{
		rows = bcols;
		cols = brows;
	} else // horizontal projection (default)
	{
		rows = brows;
		cols = bcols;
	}

	int* colProj = new int[cols];
	memset(colProj, 0, sizeof(colProj));
	size = cols;
	int tvalue = 0;
	int count = 0;
	for (int c = 0; c < cols; ++c)
	{
		count = 0;
		for (int r = 0; r < rows; ++r)
		{
			if (pType == Horizontal)
			{
				tvalue = binaryMatrix->getAtPosition(r, c);

			} else
			{
				tvalue = binaryMatrix->getAtPosition(c, r);
			}
			if (tvalue == 0)
			{
				count++;
			}
		}
		colProj[c] = count;
	}
	return colProj;
}

void analysisProjection(int* projection, int psize, int &leftLimit,
		int &rightLimit)
{
	int maxValue = projection[0];
	int imax = 0;
	int pValue = 0;
	// find the max location and value
	for (int i = 1; i < psize; i++)
	{
		pValue = projection[i];
		if (pValue > maxValue)
		{
			maxValue = pValue;
			imax = i;
		}
	}
	//cout<<"\nMax value: "<<maxValue;
	// find the left part
	int currentMax = maxValue;
	int lIndex = imax;
	int flIndex = imax; // store the left limti position
	for (int l = imax - 1; l >= 0; l--)
	{
		pValue = projection[l];
		if (pValue < currentMax)
		{
			currentMax = pValue;
			lIndex = l;
		} else
		{
			if (pValue == currentMax)
			{
				flIndex = lIndex;
			}
		}
	}
	//cout << "\nLeft limit: " << flIndex;
	leftLimit = flIndex;
	// find the right part
	currentMax = maxValue;
	int rIndex = imax;
	int frIndex = imax; // store the right limit position
	for (int r = imax; r < psize; r++)
	{
		pValue = projection[r];
		if (pValue < currentMax)
		{
			currentMax = pValue;
			rIndex = r;
		} else
		{
			if (pValue == currentMax)
			{
				frIndex = rIndex;
			}
		}
	}
	//cout << "\nRight limit: " << frIndex;
	rightLimit = frIndex;
}

vector<ptr_Point> boundingBoxDetection(ptr_IntMatrix grayMatrix)
{
	vector<ptr_Point> corners;
	ptr_IntMatrix quanMatrix = quantization(grayMatrix, 1);
	int hpsize = 0;
	int* hProjection = histogramProjection(quanMatrix, Horizontal, hpsize);
	int left = 0, right = 0;
	analysisProjection(hProjection,hpsize,left,right);

	int vpsize = 0;
	int* vProjection = histogramProjection(quanMatrix, Vertical, vpsize);
	int top = 0, bottom = 0;
	analysisProjection(vProjection,vpsize,top,bottom);

	ptr_Point topleft = new Point(left,top);
	ptr_Point topright = new Point(right,top);
	ptr_Point bottomleft = new Point(left,bottom);
	ptr_Point bottomright = new Point(right,bottom);
	corners.push_back(topleft);
	corners.push_back(topright);
	corners.push_back(bottomleft);
	corners.push_back(bottomright);

	// free memory
	delete quanMatrix;
	delete topleft;
	delete topright;
	delete bottomleft;
	delete bottomright;

	return corners;
}
vector<ptr_Point> boundingBoxDetection(int* hProjection, int hSize, int* vProjection, int vSize)
{
	vector<ptr_Point> corners;

	int left = 0, right = 0;
	analysisProjection(hProjection,hSize,left,right);

	int top = 0, bottom = 0;
	analysisProjection(vProjection,vSize,top,bottom);

	ptr_Point topleft = new Point(left,top);
	ptr_Point topright = new Point(right,top);
	ptr_Point bottomleft = new Point(left,bottom);
	ptr_Point bottomright = new Point(right,bottom);
	corners.push_back(topleft);
	corners.push_back(topright);
	corners.push_back(bottomleft);
	corners.push_back(bottomright);

	// free memory
	delete topleft;
	delete topright;
	delete bottomleft;
	delete bottomright;

	return corners;
}
