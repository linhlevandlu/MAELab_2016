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
	for (int i = 0; i < list.size(); i++)
	{
		Point pi = list.at(i);
		cout << "Point " << (i + 1) << ": " << pi.getX() << " - " << pi.getY()
				<< endl;
	}
}
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
	double contrast = vg / cg - vl / cl;
	cout << endl << lbp << "\t" << contrast;
	ct = contrast;
	return lbp;
}

int LBP_C_General_ImagePath(string imgPath, Point pcenter, double &ct)
{
	Image image(imgPath);
	return LBP_C_General(image, pcenter, ct);
}


/*void Create_Distribution(int lbp, double contrast, int b=8) {
 Matrix<int> distribution(256,b);
 }*/

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

int main(int argc, char* argv[])
{
	/*
	 * The first test --> not work
	 */
	/*string imagefolder =
	 "/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/original";
	 string lmfolder = "/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/landmarks";
	 int nLandmarks = 8;
	 int nImages = 200;
	 vector<Point> general_list = General_Landmarks(nLandmarks, nImages,
	 imagefolder, lmfolder);
	 Print_List_Of_Landmarks(general_list);*/
	// end the first test
	// The second test (LBP/C)
	string imagePath =
			"/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/original/Prono_001.JPG";
	//string imagePath = "/media/vanlinh/Data/Biogical_Images/tdata/i3264x2448/original/train/Prono_001.JPG";
	string lmPath =
			"/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/landmarks/p_001.TPS";
	//string lmPath ="/media/vanlinh/Data/Biogical_Images/tdata/i3264x2448/landmarks/p_001.TPS";
	Point pcenter(2136, 400);
	int radius = 10 0, lbp;
	double contrast;
	lbp = LBP_C_General_ImagePath(imagePath, pcenter, contrast);
	Point pLandmark(2088, 660), result;

	result = Searching_Landmark(imagePath, pLandmark, radius, lbp, contrast);
	result.toString();
	return 0;

}
