/*
 * MAELAB_PostCNN.cpp
 *
 *  Created on: Jun 28, 2018
 *      Author: root
 *  This cpp file implements the following idea to improve the quality
 *  of the predicted landmarks in CNN process.
 *  1. Compute the General model of manual landmarks (G): General landmarks
 *  2. Compare the predicted landmarks and General landmarks
 *  3. Transform the predicted landmarks
 */

#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string.h>
#include <string>
#include <fstream>
#include <time.h>
#include <cstdlib>
#include <dirent.h>
#include <algorithm>
#include <iostream>
#include <float.h>
#include <numeric>

using namespace std;

#include "imageModel/Point.h"
#include "imageModel/Line.h"
#include "imageModel/Edge.h"
#include "imageModel/Matrix.h"
#include "imageModel/Image.h"

vector<string> Read_Directory(string folder_path)
{
	DIR *dir;
	struct dirent *ent;
	std::vector < string > files;
	if ((dir = opendir(folder_path.c_str())) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
				files.push_back(folder_path + "/" + ent->d_name);
		}
	}
	closedir(dir);
	std::sort(files.begin(), files.end());
	return files;
}

/**
 * nLandmarks: number of landmarks i.e. 8 for pronotum, 10 for tete, 11 for elytre
 * nImages: number of images that we would like to compute the general model
 * imageFolder, mlmFolder: where we store the images and manual landmarks (TPS file)
 */
vector<Point> General_Landmarks(int nLandmarks, int nImages,
		std::string imageFolder, string mlmFolder)
{
	vector<Point> result;
	vector < string > list_images = Read_Directory(imageFolder);
	vector < string > list_tpsfiles = Read_Directory(mlmFolder);
	if (list_tpsfiles.size() == list_images.size())
	{
		if (nImages > list_images.size())
			nImages = list_images.size();
		string imgfile, lmfile;
		int xArray[nLandmarks] =
		{ 0 };
		int yArray[nLandmarks] =
		{ 0 };
		for (int i = 0; i < nImages; i++)
		{
			imgfile = list_images.at(i);
			lmfile = list_tpsfiles.at(i);
			Image image(imgfile);
			vector<Point> mLandmarks = image.readManualLandmarks(lmfile);
			for (int j = 0; j < mLandmarks.size(); j++)
			{
				Point pi = mLandmarks.at(j);
				xArray[j] += pi.getX();
				yArray[j] += pi.getY();
			}
		}
		for (int k = 0; k < nLandmarks; k++)
		{
			int x = xArray[k];
			int y = yArray[k];
			result.push_back(Point(x / nImages, y / nImages));
		}
	}
	return result;
}

/*
 * Print the coordinates of landmarks in a list
 */
void Print_List_Of_Landmarks(vector<Point> list)
{
	for (size_t i = 0; i < list.size(); i++)
	{
		Point pi = list.at(i);
		cout << "Point " << (i + 1) << ": " << pi.getX() << " - " << pi.getY()
				<< endl;
	}
}

/*
 * Compute the constrast of a patch of 3x3 around a point pcenter
 */
int LBP_C_General(Image image, Point pcenter, double &ct)
{
	Matrix<int> grayImage = image.getGrayMatrix();
	int cValue = grayImage.getAtPosition(pcenter.getY(), pcenter.getX());
	int i = 0, cg = 0, cl = 0;
	double lbp = 0, vg = 0, vl = 0;
	for (int y = pcenter.getY() - 1; y <= pcenter.getY() + 1; y++)
	{
		for (int x = pcenter.getX() - 1; x <= pcenter.getX() + 1; x++)
		{
			if (y != pcenter.getY() || x != pcenter.getX())
			{
				int tValue = grayImage.getAtPosition(y, x);
				double ivalue = pow(2, i);
				if (tValue >= cValue)
				{
					lbp += ivalue;
					vg += tValue;
					cg++;
				}
				else
				{
					vl += tValue;
					cl++;
				}
				i++;
			}
		}
	}
	double highContrast = (cg != 0) ? vg / cg : 0;
	double lowContrast = (cl != 0) ? vl / cl : 0;
	double contrast = highContrast - lowContrast;
	if (contrast < 0)
		cout << endl << lbp << "\t" << contrast;
	ct = contrast;
	return lbp;
}

int LBP_C_General_ImagePath(string imgPath, Point pcenter, double &ct)
{
	Image image(imgPath);
	return LBP_C_General(image, pcenter, ct);
}

Matrix<int> Create_Distribution(Image image, Point pcenter, int radius = 10,
		int nBins = 8)
{
	Matrix<int> distribution(256, nBins, 0);
	int px = pcenter.getX();
	int py = pcenter.getY();
	//Image image(imgPath);
//Matrix<int> grayImage = image.getGrayMatrix();
	vector<int> lbpArray;
	vector<double> contrastArray;
	for (int r = py - radius; r < py + radius; r++)
	{
		for (int c = px - radius; c < px + radius; c++)
		{
			Point pi(r, c);
			int lbp = 0;
			double ct = 0;
			lbp = LBP_C_General(image, pi, ct);
			lbpArray.push_back(lbp);
			contrastArray.push_back(ct);
		}
	}
	if (lbpArray.size() == contrastArray.size())
	{
		double sumContrast = std::accumulate(contrastArray.begin(),
				contrastArray.end(), 0.0);
		double sizeOfBin = sumContrast / nBins;
		for (int i = 0; i < lbpArray.size(); i++)
		{
			int lbpV = lbpArray.at(i);
			double cV = contrastArray.at(i);
			int cIndex = -1;
			if (cV >= 0)
			{
				if (std::fmod(cV, sizeOfBin) == 0.0)
				{
					cIndex = (int) (cV / sizeOfBin) - 1;
				}
				else
				{
					cIndex = (int) (cV / sizeOfBin);
				}
				int oldValue = distribution.getAtPosition(lbpV, cIndex);
				distribution.setAtPosition(lbpV, cIndex, oldValue + 1);
			}
		}
	}
	return distribution;
}

double BhattacharyyaMetric(Matrix<int> model, Matrix<int> sample)
{
	double distance = 0.0;
	int rows = model.getRows();
	int cols = model.getCols();
	for (int c = 0; c < cols; c++)
	{
		for (int r = 0; r < rows; r++)
		{
			int mi = model.getAtPosition(r, c);
			int si = sample.getAtPosition(r, c);
			distance += sqrt(mi * si);
		}
	}
	return distance;
}
/*
 * Searching based on a pair of (LBP, Contrast)
 */
Point Searching_Landmark(string imgPath, Point plandmark, int radius, int lbp,
		int contrast)
{
	int beginX = plandmark.getX() - radius;
	int endX = plandmark.getX() + radius;
	int beginY = plandmark.getY() - radius;
	int endY = plandmark.getY() + radius;
	int temp_lbp = 0;
	double ctrast = 0.0;
	double distance = 0, maxDistance = 3264;
	Point result(0, 0);
	Image image(imgPath);
	for (int r = beginY; r < endY; r++)
	{
		for (int c = beginX; c < endX; c++)
		{
			Point pi(c, r);
			temp_lbp = LBP_C_General(image, pi, ctrast);
			distance = sqrt(pow(temp_lbp - lbp, 2) + pow(contrast - ctrast, 2));
			if (distance <= maxDistance)
			{
				maxDistance = distance;
				result.setX(c);
				result.setY(r);
			}
		}
	}
	return result;
}
Point Searching_Landmark_Distribution(string imgPath, Point plandmark,
		int radiusPatch, int radius, int nBins, Matrix<int> modelDistr)
{
	int beginX = plandmark.getX() - radiusPatch;
	int endX = plandmark.getX() + radiusPatch;
	int beginY = plandmark.getY() - radiusPatch;
	int endY = plandmark.getY() + radiusPatch;
	Matrix<int> tempDistr;

	double distance = 0, maxDistance = 3264;
	Point result(0, 0);
	Image image(imgPath);
	for (int r = beginY; r < endY; r++)
	{
		for (int c = beginX; c < endX; c++)
		{
			Point pi(c, r);
			tempDistr = Create_Distribution(image, pi, radius, nBins);
			distance = BhattacharyyaMetric(modelDistr, tempDistr);
			if (distance <= maxDistance)
			{
				maxDistance = distance;
				result.setX(c);
				result.setY(r);
			}
		}
	}
	cout << endl << "Max distance: " << maxDistance;
	return result;
}

/*
 * Split and merge patch 25x25
 */
void Split_Patches(Image image, int wSize, int hSize)
{
	Matrix<RGB> rgbImage = image.getRGBMatrix();
	int rows = rgbImage.getRows();
	int cols = rgbImage.getCols();
	int nCPatch = cols / wSize;
	int nRPatch = rows / hSize;
	cout<<endl<<nRPatch<<"\t"<<nCPatch;
	int rbegin = 0, cbegin = 0;
	int rend = 0, cend = 0;
	vector<Matrix<RGB> > patches;
	for (int rPatch = 0; rPatch < nRPatch; rPatch++)
	{
		rbegin = (rPatch * hSize);
		rend = rbegin + hSize;
		for (int cPatch = 0; cPatch < nCPatch; cPatch++)
		{
			cbegin = (cPatch * wSize);
			cend = cbegin + wSize;
			Matrix<RGB> patch(hSize, wSize);
			int i = 0, j = 0;
			for (int r = rbegin; r < rend; r++)
			{
				j = 0;
				for (int c = cbegin; c < cend; c++)
				{
					RGB color = rgbImage.getAtPosition(r, c);
					patch.setAtPosition(i, j, color);
					j++;
				}
				i++;
			}
			patches.push_back(patch);
		}
	}
	cout << "\nnumber of patches: " << patches.size()<<endl;
}

int main(int argc, char* argv[])
{
	string imagePath =
			"results/rgb/lm1/Prono_001.jpg";
	int wSize = 25, hSize = 25;
	Image inputImg(imagePath);
	Split_Patches(inputImg,wSize,hSize);

	return 0;
}

