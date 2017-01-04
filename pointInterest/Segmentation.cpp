/*
 * Segmentation.cpp
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

using namespace std;

#include "../imageModel/Point.h"
#include "../imageModel/Line.h"
#include "../imageModel/Edge.h"
#include "../imageModel/Matrix.h"
#include "../imageModel/Image.h"

#include "../segmentation/Thresholds.h"
#include "../segmentation/Canny.h"
#include "../segmentation/Suzuki.h"
#include "../segmentation/Projection.h"
#include "../histograms/ShapeHistogram.h"

// nho xoa
#include "../io/Reader.h"
//===
#include "Treatments.h"
#include "Segmentation.h"

Segmentation::Segmentation()
{
	// TODO Auto-generated constructor stub

}

Segmentation::~Segmentation()
{
	// TODO Auto-generated destructor stub
}

ptr_IntMatrix Segmentation::threshold(int tValue, int maxValue)
{
	ptr_IntMatrix inputMatrix = Treatments::refImage.getGrayMatrix();
	return binaryThreshold(inputMatrix, tValue, maxValue);
}

vector<ptr_Edge> Segmentation::canny()
{
	return Treatments::refImage.cannyAlgorithm();
}
vector<ptr_Line> Segmentation::segment(int minDistance = 3)
{
	return Treatments::refImage.getApproximateLines(minDistance);
}

// return coordinates of 4 bounding points of object.
vector<ptr_Point> Segmentation::boundingBox()
{
	vector<ptr_Point> boundingPoints;
	ptr_IntMatrix grayMatrix = Treatments::refImage.getGrayMatrix();

	// nho xoa
	ptr_IntMatrix quanMatrix = quantization(grayMatrix, 3);

	saveGrayScale("quanMatrix.jpg", quanMatrix);
	int rows = grayMatrix->getRows();
	int cols = grayMatrix->getCols();
	ptr_IntMatrix output = new Matrix<int>(rows, cols, 0);
	int tk1 = 0, tk2 = 0, tk3 = 0, tk4 = 0;
	int tValue;
	for (int r = 0; r < rows; ++r)
	{
		for (int c = 0; c < cols; ++c)
		{
			tValue = quanMatrix->getAtPosition(r, c);
			if (tValue <= 1)
			{
				output->setAtPosition(r, c, 0);
			}
			else
			{
				output->setAtPosition(r, c, 255);
			}
			if (tValue == 0)
				tk1++;
			if (tValue == 1)
				tk2++;
			if (tValue == 2)
				tk3++;
			if (tValue == 3)
				tk4++;
		}
	}
	cout << "\t 0 pixells: " << tk1 << endl;
	cout << "\t 1 pixells: " << tk2 << endl;
	cout << "\t 2 pixells: " << tk3 << endl;
	cout << "\t 3 pixells: " << tk4 << endl;
	saveGrayScale("2GrayLevels.jpg", output);
	//int rows = grayMatrix->getRows();
	//int cols = grayMatrix->getCols();

	int psize = 0;
	int* hHistogram = histogramProjection(output, Horizontal, psize);
	cout << "\n" << psize << endl;

	ptr_IntMatrix houtput = new Matrix<int>(rows, cols, 0);
	int pvalue = 0;
	for (int i = 0; i < psize; i++)
	{
		pvalue = hHistogram[i];
		if (pvalue < 0)
			pvalue = 0;
		else
			if (pvalue >= rows)
				pvalue = rows - 1;
		houtput->setAtPosition(rows - pvalue - 1, i, 255);
	}
	int left = 0, right = 0;
	analysisProjection(hHistogram, psize, left, right);
	for (int k = 0; k < rows; ++k)
	{
		houtput->setAtPosition(k, left, 255);
		houtput->setAtPosition(k, right, 255);
	}
	saveGrayScale("hProjection.jpg", houtput);

	int vpsize = 0;
	int* vHistogram = histogramProjection(output, Vertical, vpsize);
	cout << "\n" << vpsize << endl;
	ptr_IntMatrix voutput = new Matrix<int>(rows, cols, 0);
	int vpvalue = 0;
	for (int i = 0; i < vpsize; i++)
	{
		vpvalue = vHistogram[i];
		if (vpvalue < 0)
			vpvalue = 0;
		else
			if (vpvalue >= cols)
				vpvalue = cols - 1;
		voutput->setAtPosition(i, vpvalue, 255);
	}
	int top = 0, bottom = 0;
	analysisProjection(vHistogram, vpsize, top, bottom);
	for (int k = 0; k < cols; k++)
	{
		voutput->setAtPosition(top, k, 255);
		voutput->setAtPosition(bottom, k, 255);
	}
	saveGrayScale("vProjection.jpg", voutput);

	 //boundingPoints = boundingBoxDetection(grayMatrix);
	boundingPoints = boundingBoxDetection(hHistogram,psize,vHistogram,vpsize);
	return boundingPoints;
}
