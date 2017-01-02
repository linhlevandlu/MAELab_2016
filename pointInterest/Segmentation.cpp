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
	/*ptr_IntMatrix quanMatrix = quantization(grayMatrix, 1);

	 saveGrayScale("quanMatrix.jpg", quanMatrix);

	 int rows = grayMatrix->getRows();
	 int cols = grayMatrix->getCols();

	 int psize = 0;
	 int* hHistogram = histogramProjection(quanMatrix, Horizontal, psize);
	 cout << "\n" << psize << endl;

	 ptr_IntMatrix output = new Matrix<int>(rows, cols, 0);
	 int pvalue = 0;
	 for (int i = 0; i < psize; i++)
	 {
	 pvalue = hHistogram[i];
	 if (pvalue < 0)
	 pvalue = 0;
	 else
	 if (pvalue >= rows)
	 pvalue = rows - 1;
	 output->setAtPosition(rows - pvalue - 1, i, 255);
	 }
	 int left = 0, right = 0;
	 analysisProjection(hHistogram, psize, left, right);
	 for (int k = 0; k < rows; ++k)
	 {
	 output->setAtPosition(k, left, 255);
	 output->setAtPosition(k, right, 255);
	 }
	 saveGrayScale("hProjection.jpg", output);

	 int vpsize = 0;
	 int* vHistogram = histogramProjection(quanMatrix, Vertical, vpsize);
	 cout << "\n" << vpsize << endl;
	 ptr_IntMatrix output2 = new Matrix<int>(rows, cols, 0);
	 int vpvalue = 0;
	 for (int i = 0; i < vpsize; i++)
	 {
	 vpvalue = vHistogram[i];
	 if (vpvalue < 0)
	 vpvalue = 0;
	 else
	 if (vpvalue >= cols)
	 vpvalue = cols - 1;
	 output2->setAtPosition(i, vpvalue, 255);
	 }
	 int top = 0, bottom = 0;
	 analysisProjection(vHistogram, vpsize, top, bottom);
	 for (int k = 0; k < cols; k++)
	 {
	 output2->setAtPosition(top, k, 255);
	 output2->setAtPosition(bottom, k, 255);
	 }
	 saveGrayScale("vProjection.jpg", output2);*/

	boundingPoints = boundingBoxDetection(grayMatrix);

	cout << "\n Finish";
	return boundingPoints;
}
