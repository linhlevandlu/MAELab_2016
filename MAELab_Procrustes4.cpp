/*
 * MAELab_Procrustes4.cpp
 *
 *  Created on: Feb 16, 2018
 *      Author: root
 *
 * Improving the coordinates of the landmarks
 *
 * Step 1: Estimated the landmarks by CNN
 * Step 2: Segmentation the images
 * Step 3: Finding the point on curve that has the minimum distance
 * 		   with the estimated landmarks
 *
 * 		   Landmark chinh la diem tren curve gan diem duoc chon nhat
 */

#include <iostream>
#include <sstream>
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
#include <string>
#include <algorithm>
#include <pthread.h>
#include <queue>
#include <float.h>

using namespace std;

#include "imageModel/Point.h"
#include "imageModel/Line.h"
#include "imageModel/Edge.h"
#include "imageModel/Matrix.h"
#include "imageModel/Image.h"
#include "utils/Drawing.h"
#include "io/Reader.h"

#include "histograms/ShapeHistogram.h"
#include "pht/PHTEntry.h"
#include "pht/PHoughTransform.h"
#include "correlation/CrossCorrelation.h"

#include "segmentation/Thresholds.h"
#include "segmentation/Canny.h"
#include "segmentation/Suzuki.h"
#include "segmentation/Filters.h"

#include "pointInterest/Treatments.h"
#include "pointInterest/Segmentation.h"
#include "pointInterest/GeometricHistgoram.h"
#include "pointInterest/ProHoughTransform.h"
#include "pointInterest/LandmarkDetection.h"

#include "MAELab.h"

Point Nearest_Point(vector<Point> sourcePoints, Point lmPoint)
{
	double minDistance = DBL_MAX;
	Point landmark = lmPoint;
	for (size_t i = 0; i < sourcePoints.size(); i++)
	{
		Point pi = sourcePoints.at(i);
		Line line(lmPoint, pi);
		if (line.getLength() < minDistance)
		{
			landmark = pi;
			minDistance = line.getLength();
		}
	}
	return landmark;
}

Point Improve_Coordinates(Image image, Point pPoint)
{
	vector<Point> cPoints = mae_Canny_Algorithm(&image);
	Point landmark = Nearest_Point(cPoints, pPoint);
	landmark.toString();
	return landmark;
}
int main(int argc, char* argv[])
{
	string imagePath =
			"/home/linhpc/data_CNN/linhlv/pronotum/v1/original/Prono_011.JPG";
	Image image(imagePath);
	Point pPoint(244, 145);
	Improve_Coordinates(image,pPoint);
	return 0;
}

