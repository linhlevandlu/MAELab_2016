/*
 * Texture.cpp
 *
 *  Created on: Jan 13, 2017
 *      Author: linh
 */
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
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
				} else
				{
					region2->setAtPosition(r, c - subCols, value);
				}
			} else
			{
				if (c < subCols)
				{
					region3->setAtPosition(r - subRow, c, value);
				} else
				{
					region4->setAtPosition(r - subRow, c - subCols, value);
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
ptr_IntMatrix getPack(ptr_IntMatrix inputImage, int rIndex, int cIndex,
		int rSize, int cSize)
{
	ptr_IntMatrix result = new Matrix<int>(rSize, cSize, 0);

	int rows = inputImage->getRows();
	int cols = inputImage->getCols();
	if (rIndex + rSize <= rows && cIndex + cSize <= cols)
	{
		int value = 0;
		for (int r = rIndex; r < rIndex + rSize; r++)
		{
			for (int c = cIndex; c < cIndex + cSize; c++)
			{
				value = inputImage->getAtPosition(r, c);
				result->setAtPosition(r - rIndex, c - cIndex, value);
			}
		}
	}
	return result;
}
// split image into blocks (16x16)
vector<ptr_IntMatrix> splitImage16x16(ptr_IntMatrix grayImage)
{
	int rows = grayImage->getRows();
	int cols = grayImage->getCols();
	int sizew = 16, sizeh = 16;
	vector<ptr_IntMatrix> packs;
	for (int r = 0; r < rows; r += 16)
	{
		for (int c = 0; c < cols; c += 16)
		{
			packs.push_back(getPack(grayImage, r, c, 16, 16));
		}
	}
	cout << "\nNumber of packs: " << packs.size() << endl;
	return packs;
}

// compute LBP and contrast on a cell 3x3
double contrastLBP(ptr_IntMatrix region3x3, double &lbp)
{
	lbp = 0;
	double contrast = 0;
	int vCenter = region3x3->getAtPosition(1, 1);

	int sumUpper = 0;
	int countUpper = 0;
	int sumLower = 0;
	int countLower = 0;
	double lbpTemp = 0;

	int vi = 0;
	int binomial = 1;
	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			vi = region3x3->getAtPosition(r, c);
			if (r != 1 || c != 1)
			{
				if (vi >= vCenter)
				{
					sumUpper += vi;
					countUpper++;
					lbpTemp += (binomial * 1);
				} else
				{
					sumLower += vi;
					countLower++;
					lbpTemp += (binomial * 0);
				}
				binomial *= 2;
			}
		}
	}

	lbp = lbpTemp;
	if (countUpper != 0 && countLower != 0)
	{
		contrast = (double) ((double) sumUpper / countUpper)
				- (double) ((double) sumLower / countLower);
	} else
	{
		if (countUpper == 0 && countLower != 0)
		{
			contrast = (double) ((double) sumLower / countLower);
		} else
		{
			if (countUpper != 0 && countLower == 0)
			{
				contrast = (double) ((double) sumUpper / countUpper);
			}
		}
	}
	return contrast;
}

ptr_IntMatrix get3x3Region(ptr_IntMatrix region, int rindex, int cindex)
{
	int rows = region->getRows();
	int cols = region->getCols();
	ptr_IntMatrix region3x3 = new Matrix<int>(3, 3, 0);
	for (int r = rindex - 1; r < rindex + 2; r++)
	{
		for (int c = cindex - 1; c < cindex + 2; c++)
		{
			if (r >= 0 && r < rows && c >= 0 && c < cols)
			{
				int value = region->getAtPosition(r, c);
				region3x3->setAtPosition(r - rindex + 1, c - cindex + 1, value);
			}
		}
	}
	return region3x3;
}

/*ptr_IntMatrix lbpcDistribution(ptr_IntMatrix region)
 {
 int rows = region->getRows();
 int cols = region->getCols();
 int exrows = 0, excols = 0;
 if (rows % 3 == 0)
 {
 exrows = rows;
 } else
 {
 if (rows % 3 == 1)
 {
 exrows = rows - 1;
 } else
 {
 exrows = rows + 1;
 }
 }
 if (cols % 3 == 0)
 {
 excols = cols;
 } else
 {
 if (cols % 3 == 1)
 {
 excols = cols - 1;
 } else
 {
 excols = cols + 1;
 }
 }
 ptr_IntMatrix tempRegion = new Matrix<int>(exrows, excols);
 for (int r = 0; r < exrows; r++)
 {
 for (int c = 0; c < excols; c++)
 {
 if (r < rows && c < cols)
 {
 tempRegion->setAtPosition(r, c, region->getAtPosition(r, c));

 } else
 {
 tempRegion->setAtPosition(r, c, 0);
 }
 }
 }

 double contrast = 0, maxConst = 0;
 double lbp = 0;
 vector<double> listContrast;
 vector<double> listLBP;
 ofstream of("contrast.txt");
 for (int r = 0; r < exrows; r += 3)
 {
 for (int c = 0; c < excols; c += 3)
 {
 ptr_IntMatrix rg3x3 = new Matrix<int>(3, 3, 0);
 for (int i = 0; i < 3; i++)
 {
 for (int j = 0; j < 3; j++)
 {
 int value = tempRegion->getAtPosition(r + i, c + j);
 rg3x3->setAtPosition(i, j, value);
 }
 }
 contrast = contrastLBP(rg3x3, lbp);
 if (contrast > maxConst)
 maxConst = contrast;
 listContrast.push_back(contrast);
 listLBP.push_back(lbp);
 of << lbp << "\t" << contrast << endl;
 delete rg3x3;
 }
 }
 of.close();
 //cout << "\nMaximum contrast: " << maxConst;
 ptr_IntMatrix lbpcDist = new Matrix<int>(256, 8, 0);
 double entriesInBin = maxConst / 8;
 if (listContrast.size() == listLBP.size())
 {
 double contrasti, lbpi;
 int index = 0, oldValue = 0;
 for (size_t i = 0; i < listContrast.size(); i++)
 {
 contrasti = listContrast.at(i);
 lbpi = listLBP.at(i);
 index = round(contrasti / entriesInBin);
 oldValue = lbpcDist->getAtPosition(lbpi, index);
 lbpcDist->setAtPosition(lbpi, index, oldValue + 1);
 }
 }
 listContrast.clear();
 listLBP.clear();
 delete tempRegion;

 return lbpcDist;
 }*/

ptr_IntMatrix lbpcDistribution(ptr_IntMatrix region)
{
	int rows = region->getRows();
	int cols = region->getCols();

	double contrast = 0, totalConst = 0;
	double lbp = 0;
	vector<double> listContrast;
	vector<double> listLBP;

	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			ptr_IntMatrix rg3x3 = get3x3Region(region, r, c);
			contrast = contrastLBP(rg3x3, lbp);
			totalConst += contrast;
			listContrast.push_back(contrast);
			listLBP.push_back(lbp);
			delete rg3x3;
		}
	}
	cout << "\nTotal contrast: " << totalConst << "\t" << totalConst / 32
			<< endl;
	ptr_IntMatrix lbpcDist = new Matrix<int>(256, 32, 0);
	double entriesInBin = totalConst / 32;
	if (listContrast.size() == listLBP.size())
	{
		double contrasti, lbpi;
		int index = 0, oldValue = 0;
		for (size_t i = 0; i < listContrast.size(); i++)
		{
			contrasti = listContrast.at(i);
			lbpi = listLBP.at(i);
			index = ceil(contrasti / entriesInBin);
			oldValue = lbpcDist->getAtPosition(lbpi, index);
			lbpcDist->setAtPosition(lbpi, index, oldValue + 1);
		}
	}
	listContrast.clear();
	listLBP.clear();
	return lbpcDist;
}

int totalValueDist(ptr_IntMatrix lbpcDist)
{
	int rows = lbpcDist->getRows();
	int cols = lbpcDist->getCols();
	int total = 0;
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			total += lbpcDist->getAtPosition(r, c);
		}
	}
	return total;
}

// calculate the frequency at bin n of LBP/C distribution
int freqDistributionAtBin(ptr_IntMatrix lbpcDist, int binn)
{
	int rows = lbpcDist->getRows();
	int cols = lbpcDist->getCols();
	int count = 0;
	for (int r = 0; r < rows; r++)
	{
		count += lbpcDist->getAtPosition(r, binn);
	}

	return count;
}
double likelihoodRatio(ptr_IntMatrix sample, ptr_IntMatrix model)
{
	int srows = sample->getRows();
	int scols = sample->getCols();
	int mrows = model->getRows();
	int mcols = model->getCols();
	int stotal = totalValueDist(sample);
	int mtotal = totalValueDist(model);
	double measure = 0;
	if (srows == mrows && scols == mcols)
	{
		//double si = 0, mi = 0;
		for (int c = 0; c < scols; c++)
		{
			/*double si = sqrt(
			 (double) freqDistributionAtBin(sample,c) / (double) stotal);
			 double mi = sqrt(
			 (double) freqDistributionAtBin(model,c) / (double) mtotal);
			 measure += (si * mi);*/
			double si = (double) freqDistributionAtBin(sample, c)
					/ (double) stotal;
			double mi = (double) freqDistributionAtBin(model, c)
					/ (double) mtotal;
			if (mi != 0 && si != 0)
				measure += (si * log(si / mi));
		}
	}
	return measure;
}
