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
#include <sstream>

using namespace std;

#include "imageModel/Point.h"
#include "imageModel/Line.h"
#include "imageModel/Edge.h"
#include "imageModel/Matrix.h"
#include "imageModel/Image.h"
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
/*
 Steps of this test.
 1. Extract the patch around the preditected landmark
 2. Binary (threshold)
 3. Projection on X=0, y =0 or x=y
 4. Try with segmentation
 */

Matrix<int> extractLandmarkPatch(string image_file, string landmark_file,
		int lmIndex, int width, int height, string save_folder)
{
	Image matImage(image_file);
	matImage.readManualLandmarks(landmark_file);
	vector<Point> landmarks = matImage.getListOfManualLandmarks();
	string name = matImage.getName();
	size_t found2 = name.find_last_of(".");
	string sname = name.substr(0, found2);
	Point pi = landmarks.at(lmIndex);
	Matrix<int> grayImage = matImage.getGrayMatrix();
	Matrix<int> patch = grayImage.extractPatch(width, height, pi.getY(),
			pi.getX(), 0);

	std::stringstream ssname;
	ssname << sname;
	ssname << "_patch.jpg";
	string savename = save_folder + "/" + ssname.str();
	saveGrayScale(savename.c_str(), &patch);

	return patch;
}

Point Exact_Landmark(Matrix<int> projection)
{
	int maximum = 0;
	int lastc = 0, lastr;
	for (int c = 0; c < projection.getCols(); c++)
	{
		int countBlack = 0;
		int temp_lastr = -1;
		for (int r = 0; r < projection.getRows(); r++)
		{
			if (projection.getAtPosition(r, c) == 0)
			{
				countBlack++;
				if (temp_lastr == -1)
					temp_lastr = r;
			}
		}
		//cout << endl << maximum << " - " << countBlack;
		if (countBlack >= maximum)
		{
			maximum = countBlack;
			lastc = c;
			lastr = temp_lastr;
		}
	}
	return Point(lastc, lastr);
}

int main(int argc, char* argv[])
{

	//string imagePath = "/home/linhpc/Data/images/Prono_032.JPG";
	//string landmarkPath = "/home/linhpc/Data/predicted_landmarks/prono_32.TPS";
	string imagePath =
			"/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/original/Prono_001.JPG";
	string landmarkPath =
			"/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/predicted_landmarks/prono_001.TPS";
	if (argc == 3)
	{
		imagePath = argv[1];
		landmarkPath = argv[2];
	}
	else
	{
		cout << "Default!" << endl;
	}
	Image orgImage(imagePath);
	vector<Point> list_Landmarks = orgImage.readManualLandmarks(landmarkPath);

	int lmIndex = 5, wPatch = 150, hPatch = 200;
	Point cLandmark = list_Landmarks.at(lmIndex);
	Point originPatch(cLandmark.getX() - wPatch / 2,
			cLandmark.getY() - hPatch / 2);

	extractLandmarkPatch(imagePath, landmarkPath, lmIndex, wPatch, hPatch,
			"results/lm6");
	//string step2Image = "results/patch.jpg";
	//Image patch(step2Image);

	/* Apply a Gaussian filter before computing */
	/*Matrix<double> kernel = getGaussianKernel(5, 1.0);
	Matrix<RGB> imageGBlur = mae_Gaussian_Filter(&patch, kernel);
	patch.setRGBMatrix(imageGBlur);*/

	/* Extract and compare by projection */
	/*ptr_IntMatrix thresh_matrix = mae_Binary_Threshold(&patch);
	saveGrayScale("results/patch_bin.jpg", thresh_matrix);
	Point p = Exact_Landmark(*thresh_matrix);
	cout << p.getX() + originPatch.getX() << "\t"
			<< p.getY() + originPatch.getY() << endl;*/

	/* Extract and compare by line segment*/

	/*Point exLandmark(cLandmark.getX() - originPatch.getX(),
	 cLandmark.getY() - originPatch.getY());
	 vector<Point> cannyPoints = mae_Canny_Algorithm(&patch);
	 double minDistance = DBL_MAX;
	 Point result(0, 0);
	 for (size_t i = 0; i < cannyPoints.size(); i++)
	 {
	 Point pi = cannyPoints.at(i);
	 double distance = distancePoints(exLandmark, pi);
	 if (distance < minDistance)
	 {
	 minDistance = distance;
	 result.setX(pi.getX());
	 result.setY(pi.getY());
	 }
	 }
	 cout << result.getX() + originPatch.getX() << "\t"
	 << result.getY() + originPatch.getY() << endl;*/

	return 0;

}
