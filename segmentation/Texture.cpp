/*
 * Texture.cpp
 *
 *  Created on: Jan 13, 2017
 *      Author: linh
 */
#include <iostream>
#include <vector>
using namespace std;

#include "../imageModel/Point.h"
#include "../imageModel/Matrix.h"
#include "Texture.h"

vector<ptr_IntMatrix> splitImage(ptr_IntMatrix grayImage)
{
	int rows = grayImage->getRows();
	int cols = grayImage->getCols();

	int subRow = rows / 2;
	int subCols = cols / 2;
	ptr_IntMatrix region1 = new Matrix<int>(subRow, subCols);
	ptr_IntMatrix region2 = new Matrix<int>(subRow, cols - subCols);
	ptr_IntMatrix region3 = new Matrix<int>(rows - subRow, subCols);
	ptr_IntMatrix region4 = new Matrix<int>(rows - subRow, cols - subCols);

	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			int value = grayImage->getAtPosition(r, c);
			if (r < subRow)
			{
				if (c < subCols)
				{
					region1->setAtPosition(r, c, value);
				}
				else
				{
					region2->setAtPosition(r, c - subCols, value);
				}
			}
			else
			{
				if (c < subCols)
				{
					region3->setAtPosition(r - subRow, c, value);
				}
				else
				{
					region4->setAtPosition(r- subRow, c - subCols, value);
				}
			}
		}
	}
	vector<ptr_IntMatrix> regions;
	regions.push_back(region1);
	regions.push_back(region2);
	regions.push_back(region3);
	regions.push_back(region4);
	return regions;
}


double contrastLBP(ptr_IntMatrix region,double &lbp)
{
	lbp = 0;
	double contrast = 0;
	int rows = region->getRows();
	int cols = region->getCols();

	int rCenter = rows/2;
	int cCenter = cols/2;
	int vCenter = region->getAtPosition(rCenter,cCenter);

	int sumUpper =0;
	int countUpper =0;
	int sumLower =0;
	int countLower = 0;
	double lbpTemp = 0;

	//Matrix<int> binMatrix = new Matrix<int>(rows,cols,0);
	//Matrix<int> binomialMatrix = new Matrix<int>(rows,cols,0);
	int vi = 0;
	int binomial = 1;
	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			vi = region->getAtPosition(r,c);
			if(r != rCenter && c != cCenter)
			{
				if(vi >= vCenter)
				{
					//binMatrix.setAtPosition(r,c,1);
					sumUpper +=vi;
					countUpper++;
					lbpTemp += (binomial * 1);
				}
				else
				{
					//binMatrix.setAtPosition(r,c,0);
					sumLower += vi;
					countLower++;
					lbpTemp += (binomial * 0);
				}
				binomial *= 2;
			}
		}
	}

	lbp = lbpTemp;
	contrast = (double)((double)sumUpper/countUpper) - (double)((double)sumLower/countLower);
	return contrast;
}
