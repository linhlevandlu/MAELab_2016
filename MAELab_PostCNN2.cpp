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
#include "utils/Drawing.h"

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
/*
 * Extract patch from RGB matrix
 */
Matrix<RGB> Extract_RGB(Image image, Point pi, int width, int height,
		string save_folder)
{
	Matrix<RGB> rgbImage = image.getRGBMatrix();
	RGB color;
	color.R = color.G = color.B = 255;
	Matrix<RGB> patch = rgbImage.extractPatch(width, height, pi.getY(),
			pi.getX(), color);

	string name = image.getName();
	size_t found2 = name.find_last_of(".");
	string sname = name.substr(0, found2);
	std::stringstream ssname;
	ssname << sname;
	ssname << ".jpg";
	string savename = save_folder + "/" + ssname.str();
	saveRGB(savename.c_str(), &patch);
	return patch;
}
/*
 * Extract patch from int matrix
 */
Matrix<int> Extract(Image image, Point pi, int width, int height,
		string save_folder)
{
	Matrix<int> grayImage = image.getGrayMatrix();
	Matrix<int> patch = grayImage.extractPatch(width, height, pi.getY(),
			pi.getX(), 0);

	string name = image.getName();
	size_t found2 = name.find_last_of(".");
	string sname = name.substr(0, found2);
	std::stringstream ssname;
	//ssname << sname;
	ssname << "patch.jpg";
	string savename = save_folder + "/" + ssname.str();
	saveGrayScale(savename.c_str(), &patch);
	return patch;
}
Matrix<int> Extract_Landmark_Patch(string image_file, string landmark_file,
		int lmIndex, int width, int height, string save_folder)
{
	Image matImage(image_file);
	matImage.readManualLandmarks(landmark_file);
	vector<Point> landmarks = matImage.getListOfManualLandmarks();
	Point pi = landmarks.at(lmIndex);
	Matrix<int> patch = Extract(matImage, pi, width, height, save_folder);
	/*

	 string name = matImage.getName();
	 size_t found2 = name.find_last_of(".");
	 string sname = name.substr(0, found2);

	 Matrix<int> grayImage = matImage.getGrayMatrix();
	 Matrix<int> patch = grayImage.extractPatch(width, height, pi.getY(),
	 pi.getX(), 0);

	 std::stringstream ssname;
	 ssname << sname;
	 ssname << "_patch.jpg";
	 string savename = save_folder + "/" + ssname.str();
	 saveGrayScale(savename.c_str(), &patch);*/

	return patch;
}

void ReadandSaveLandmarks(string imgpath, string lmPath, string savefolder)
{
	Image image(imgpath);
	vector<Point> mLandmarks = image.readManualLandmarks(lmPath);
	Matrix<RGB> imgWithLM = image.getRGBMatrix();
	RGB color;
	color.R = 255;
	color.G = 0;
	color.B = 0;
	for (size_t i = 0; i < mLandmarks.size(); i++)
	{
		Point lm = mLandmarks.at(i);
		imgWithLM = fillCircle(imgWithLM, lm, 7, color);
	}
	string name = image.getName();
	size_t found2 = name.find_last_of(".");
	string sname = name.substr(0, found2);
	std::stringstream ssname;
	ssname << sname;
	ssname << "_plandmarks.jpg";
	string savename = savefolder + "/" + ssname.str();
	saveRGB(savename.c_str(), &imgWithLM);
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

double SQDIFF(Matrix<int> temp, Matrix<int> image)
{
	int rows_temp = temp.getRows();
	int cols_temp = temp.getCols();
	int rows_image = image.getRows();
	int cols_image = image.getCols();
	//Matrix<double> result(rows_result,cols_result,0.0);
	double metric = 0;

	for (int r = 0; r < rows_temp; r++)
	{
		for (int c = 0; c < cols_temp; c++)
		{
			int t_value = temp.getAtPosition(r, c);
			int i_value = image.getAtPosition(r, c);
			metric += pow((t_value - i_value), 2);
		}
	}

	return metric;
}

void Comparing_Histogram(Image sImage, Point pLandmark, int width1, int height1,
		Matrix<int> model)
{
	Point begin(pLandmark.getX() - width1 / 2, pLandmark.getY() - height1 / 2);
	Point end(pLandmark.getX() + width1 / 2, pLandmark.getY() + height1 / 2);
	double maxMetric = 0;
	Point presult(0, 0);
	double minMetric = DBL_MAX;
	Matrix<double> kernel = getGaussianKernel(5, 1.0);
	Matrix<int> gray_matrix;
	Matrix<RGB> imageGBlur;
	for (int r = begin.getY(); r < end.getY(); r++)
	{
		for (int c = begin.getX(); c < end.getX(); c++)
		{
			Point pi(c, r);
			Extract(sImage, pi, 50, 50, "results");
			string refPatch = "results/patch.jpg";
			Image patch(refPatch);
			imageGBlur = mae_Gaussian_Filter(&patch, kernel);
			patch.setRGBMatrix(imageGBlur);
			gray_matrix = patch.getGrayMatrix();
			double metric = SQDIFF(model, gray_matrix);
			if (metric < minMetric)
			{
				minMetric = metric;
				presult.setX(c);
				presult.setY(r);
			}
		}
	}
	presult.toString();
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

	int lmIndex = 7, wPatch = 300, hPatch = 300;
	Point cLandmark = list_Landmarks.at(lmIndex);
	Point originPatch(cLandmark.getX() - wPatch / 2,
			cLandmark.getY() - hPatch / 2);
	Extract_RGB(orgImage,cLandmark,wPatch,hPatch,"results/rgb/lm8");

	/*Extract_Landmark_Patch(imagePath, landmarkPath, lmIndex, wPatch, hPatch,
			"results");

	string step2Image = "results/patch.jpg";
	Image patch(step2Image);*/

	/* Apply a Gaussian filter before computing */
	/*Matrix<double> kernel = getGaussianKernel(3, 1.0);
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

	/* Load predicted landmarks to the images and saving to the file */
	/*string save_folder = "/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/pLandmarksOnImages";
	 ReadandSaveLandmarks(imagePath,landmarkPath,save_folder);*/

	/*
	 * Combining test: gaussian blur + nearest points + projection
	 */
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

	 result.setX(result.getX() + originPatch.getX());
	 result.setY(result.getY() + originPatch.getY());
	 if (result.getX() != 0 && result.getY() != 0)
	 {
	 //cout<<"Step 2..."<<endl;
	 wPatch = 200;
	 Extract(orgImage, result, wPatch, hPatch, "results");
	 Point originPatch2(result.getX() - wPatch / 2,
	 result.getY() - hPatch / 2);
	 string step22Image = "results/patch.jpg";
	 Image patch2(step22Image);
	 ptr_IntMatrix thresh_matrix = mae_Binary_Threshold(&patch2);
	 saveGrayScale("results/patch_bin.jpg", thresh_matrix);
	 Point p = Exact_Landmark(*thresh_matrix);
	 cout << p.getX() + originPatch2.getX() << "\t"
	 << p.getY() + originPatch2.getY() << endl;
	 }*/

	/*
	 * Other test:
	 * 1. Choose a reference image -> extract a reference patch of 50 x 50 around a manual landmark ??
	 * 2. For each predicted landmark, create a patch of w x h.
	 *    For each pixel, extract a patch of 150 x 100, then extract the small patch.
	 *    Comparing with reference patch.
	 *
	 * Question: Size of patch?
	 */

	/*string refImgPath =
	 "/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/original/Prono_044.JPG";
	 Image refImage(refImgPath);
	 Point refLandmark(1443, 355);
	 Extract(orgImage, refLandmark, 50, 50, "results"); // extract reference patch
	 string refPatch = "results/patch.jpg";
	 Image patch(refPatch);
	 Matrix<double> kernel = getGaussianKernel(5, 1.0);
	 Matrix<RGB> imageGBlur = mae_Gaussian_Filter(&patch, kernel);
	 patch.setRGBMatrix(imageGBlur);
	 Matrix<int> gray_patch_matrix = patch.getGrayMatrix();
	 Comparing_Histogram(orgImage, cLandmark, wPatch, hPatch, gray_patch_matrix);*/

	/*
	 * Apply DoG to extract the edge
	 */
	/*Image patch2(step2Image);
	 Matrix<double> kernel2 = getGaussianKernel(5, 1.0);
	 Matrix<RGB> imageGBlur2 = mae_Gaussian_Filter(&patch2, kernel2);
	 patch2.setRGBMatrix(imageGBlur2);
	 Matrix<int> patch2Int = patch2.getGrayMatrix();
	 saveGrayScale("results/patch_2.jpg", &patch2Int);
	 for (int r = 0; r < patch2Int.getCols(); r++) {
	 for (int c = 0; c < patch2Int.getCols(); c++) {
	 cout<<patch2Int.getAtPosition(r,c) <<"\t";
	 }
	 cout<<endl;
	 }


	 Matrix<int> dog = patch.getGrayMatrix().subtract(patch2.getGrayMatrix(),0);
	 saveGrayScale("results/dog.jpg", &dog);*/

	return 0;

}
