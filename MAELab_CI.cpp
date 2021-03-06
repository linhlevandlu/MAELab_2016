/*
 *
 *
 * Test file
 */
#include <iostream>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string.h>
#include <fstream>
#include <time.h>
#include <cstdlib>
#include <dirent.h>

using namespace std;

#include "imageModel/Point.h"
#include "imageModel/Line.h"
#include "imageModel/Edge.h"
#include "imageModel/Matrix.h"
#include "imageModel/Image.h"

#include "histograms/ShapeHistogram.h"
#include "pht/PHTEntry.h"
#include "pht/PHoughTransform.h"
#include "correlation/CrossCorrelation.h"
#include "pointInterest/Treatments.h"
#include "pointInterest/Segmentation.h"
#include "pointInterest/GeometricHistgoram.h"
#include "pointInterest/ProHoughTransform.h"
#include "pointInterest/LandmarkDetection.h"

#include "MAELab.h"

void workOnDir(Image mImage, string sceneFolder, int distanceAcc, int tempSize,
	int sceneSize)
{
	DIR *pDir;
	struct dirent *entry;
	string filePath;
	pDir = opendir(sceneFolder.c_str());
	if (pDir == NULL)
	{
		cout << "\n Error when reading the folder";
		return;
	}
	while (entry = readdir(pDir))
	{
		if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
		{
			filePath = sceneFolder + "/" + entry->d_name;
			cout << "\n" << filePath;
			Image sceneimage(filePath);

			LandmarkDetection lm;
			ptr_Treatments tr = new LandmarkDetection();
			tr->setRefImage(mImage);

			vector<ptr_Point> esLandmarks = estimatedLandmarks(tr, sceneimage, Degree,
				distanceAcc, tempSize, sceneSize);

			cout << "\nTotal landmarks: " << esLandmarks.size();
		}
	}
	closedir(pDir);
}
int main(int argc, char* argv[])
{
	cout << "MAELab test on the folder" << endl << endl;
	string mImagePath, mLMPath, sImageFolder;
	int distanceAcc = 500; // distance accuracy to compute the geometric histgoram
	int tempSize = 400; // template size uses in template matching
	int sceneSize = 500; // image size uses in template matching
	if (argc != 3)
	{
		mImagePath = "data/Md039.JPG";
		mLMPath = "data/Md 039.TPS";
		sImageFolder = "data/sceneImages";
	}
	else
	{
		mImagePath = argv[0];
		mLMPath = argv[1];
		sImageFolder = argv[2];
	}

	Image modelImage(mImagePath);
	modelImage.readManualLandmarks(mLMPath);
	workOnDir(modelImage, sImageFolder, distanceAcc, tempSize, sceneSize);
	cout << endl << "finish\n";
	return 0;
}
