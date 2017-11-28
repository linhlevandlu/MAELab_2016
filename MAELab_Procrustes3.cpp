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
struct comparexTest
{
	bool operator()(Point p1, Point p2)
	{
		return p1.getX() < p2.getX();
	}
} xComparationTest;
struct compareyTest
{
	bool operator()(Point p1, Point p2)
	{
		return p1.getY() < p2.getY();
	}
} yComparationTest;

/*
 * Find the closet point of p on list points cPoints
 */
Point landmark_On_Curve(Point lmPoint, vector<Point> cPoints)
{
	double minDistance = DBL_MAX;
	Point landmark = lmPoint;
	for (size_t i = 0; i < cPoints.size(); i++)
	{
		Point pi = cPoints.at(i);
		Line line(lmPoint, pi);
		if (line.getLength() < minDistance)
		{
			landmark = pi;
			minDistance = line.getLength();
		}
	}
	return landmark;

}

/**
 * Extract the pixels belong to the contours and store into the list
 */
/*vector<Point> extract_Contours(Matrix<int> mapContours)
 {
 vector<Point> ctPoints;
 int rows = mapContours.getRows();
 int cols = mapContours.getCols();
 for (int r = 0; r < rows; r++)
 {
 for (int c = 0; c < cols; c++)
 {
 if (mapContours.getAtPosition(r, c) != 0)
 {
 ctPoints.push_back(Point(c, r));
 }
 }
 }
 if (ctPoints.size() > 0)
 {
 ctPoints = reduce_Contours_Points(ctPoints);
 }
 std::sort(ctPoints.begin(), ctPoints.end(), xComparationTest);
 return ctPoints;
 }*/
/*
 * Applied for 4th, 6th landmark
 */
/*vector<Point> extract_Contours2(Matrix<int> mapContours)
 {
 vector<Point> ctPoints;
 int rows = mapContours.getRows();
 int cols = mapContours.getCols();
 for (int r = 0; r <= rows / 2; r++)
 {
 for (int c = 0; c < cols; c++)
 {
 if (mapContours.getAtPosition(r, c) != 0)
 {
 ctPoints.push_back(Point(c, r));
 }
 }
 }
 if (ctPoints.size() > 0)
 {
 ctPoints = reduce_Contours_Points2(ctPoints);
 }
 cout << "\nNumber of points: " << ctPoints.size();
 std::sort(ctPoints.begin(), ctPoints.end(), yComparationTest);
 return ctPoints;
 }*/

//================================================================================
/*vector<Point> parse_Folder(string folderPath)
 {
 // read directory
 vector<string> txtFiles = readDirectory(folderPath.c_str());
 int xarr[7] =
 { 0 };
 int yarr[7] =
 { 0 };

 int count = 0;
 for (size_t i = 0; i < txtFiles.size(); i++)
 {
 string fileName = folderPath + "/" + txtFiles.at(i);
 //cout << fileName << endl;
 Matrix<int> pMatrix = parse_Matrix_From_File(fileName);
 vector<Point> cPoints = extract_Contours(pMatrix);
 //cout << "\nNumber of contour points: " << cPoints.size() << endl;
 if (cPoints.size() == 7)
 {
 count++;

 for (size_t j = 0; j < cPoints.size(); j++)
 {
 Point pj = cPoints.at(j);
 xarr[j] += pj.getX();
 yarr[j] += pj.getY();
 }
 }
 }
 cout << "\n Count: " << count;
 vector<Point> result;
 int x = 0, y = 0;
 for (int k = 0; k < 7; k++)
 {
 x = xarr[k] / count;
 y = yarr[k] / count;
 result.push_back(Point(x, y));
 //cout << "\n" << x << "\t" << y;
 }
 return result;
 }*/

vector<Point> extract_BBox(vector<Point> cPoints, Point center, int bsize,
	Point &origin)
{
	if (bsize % 2 == 0)
		bsize += 1; // make the size is odd number
	int hsize = bsize / 2;
	int xbegin = center.getX() - hsize;
	int xend = center.getX() + hsize;
	int ybegin = center.getY() - hsize;
	int yend = center.getY() + hsize;

	vector<Point> pinBox;
	for (size_t t = 0; t < cPoints.size(); t++)
	{
		Point pi = cPoints.at(t);
		if (pi.getX() >= xbegin && pi.getX() <= xend && pi.getY() >= ybegin
			&& pi.getY() <= yend)
			pinBox.push_back(pi);
	}
	origin.setX(xbegin);
	origin.setY(ybegin);
	return pinBox;
}

Matrix<int> list_Points_To_Matrix(vector<Point> listPoints, int size,
	Point origin)
{
	Matrix<int> result(size, size, 0);
	for (size_t i = 0; i < listPoints.size(); i++)
	{
		Point pi = listPoints.at(i);
		int r = pi.getY() - origin.getY();
		int c = pi.getX() - origin.getX();
		result.setAtPosition(r, c, 1);
	}
	return result;
}
Matrix<int> list_Points_To_Matrix(vector<Point> listPoints)
{
	Matrix<int> result(7, 7, 0);
	for (size_t i = 0; i < listPoints.size(); i++)
	{
		Point pi = listPoints.at(i);
		result.setAtPosition(pi.getY(), pi.getX(), 1);
	}
	return result;
}

/*
 * bsize1: size of bouding box arround considered landmark
 * bsize2: size to extract the point and to compare with mean curve
 * bsize1 >= bsize2
 */
vector<vector<Point> > extract_Shape_Manual_Landmark(Image matImage,
	Point estLandmark, int bsize1, int bsize2)
{
	vector<Point> cPoints = mae_Canny_Algorithm(&matImage);

	// find the point on the curve that have the minimum distance with estLandmark
	//double minDistance = DBL_MAX;
	Point landmark = landmark_On_Curve(estLandmark, cPoints);

	landmark.toString();

	vector<vector<Point> > ppPoints;
	Point org1, org2;
	vector<Point> pinBox = extract_BBox(cPoints, landmark, bsize1, org1);
	//cout << "\n Danh sach cac diem trong bounding box lon: " << endl;
	for (size_t i = 0; i < pinBox.size(); i++)
	{
		Point pi = pinBox.at(i);
		//pi.toString();
		vector<Point> piPinBox = extract_BBox(cPoints, pi, bsize2, org2);
		for (size_t k = 0; k < piPinBox.size(); k++)
		{
			Point pk = piPinBox.at(k);
			(piPinBox.at(k)).setX(pk.getX() - org2.getX());
			(piPinBox.at(k)).setY(pk.getY() - org2.getY());
		}
		//std::sort(piPinBox.begin(), piPinBox.end(), xComparationTest);
		//piPinBox = reduce_Contours_Points(piPinBox);
		piPinBox.push_back(pi); // them diem cuoi cung de biet vi tri ban dau trong contour points
		ppPoints.push_back(piPinBox);
	}
	return ppPoints;
}

double bhattacharyya_coefficient(Matrix<int> mt1, Matrix<int> mt2)
{
	if (mt1.getRows() != mt2.getRows() || mt1.getCols() != mt2.getCols())
		return 0; // not match
	double sum = 0;
	for (int r = 0; r < mt1.getRows(); r++)
	{
		for (int c = 0; c < mt1.getCols(); c++)
		{
			double p1 = (double) mt1.getAtPosition(r, c);
			double p2 = (double) mt2.getAtPosition(r, c);
			sum += sqrt(p1 * p2);
		}
	}
	return sum;
}
Point checkList_Matrices(vector<vector<Point> > lcpPoints,
	Matrix<int> meanMatrix)
{
	vector<Point> result;
	double maxDistance = DBL_MIN;
	int index = -1;
	vector<Point> originalList;
	for (size_t i = 0; i < lcpPoints.size(); i++)
	{
		vector<Point> lpi = lcpPoints.at(i);
		Point pi = lpi.at(lpi.size() - 1);
		lcpPoints.at(i).erase(lcpPoints.at(i).begin() + lpi.size() - 1);
		originalList.push_back(pi);
	}
	if (lcpPoints.size() == originalList.size())
	{
		for (size_t i = 0; i < lcpPoints.size(); i++)
		{
			vector<Point> icpPoints = lcpPoints.at(i);
			Matrix<int> piMatrix = list_Points_To_Matrix(icpPoints);
			double distance = bhattacharyya_coefficient(meanMatrix, piMatrix);
			if (distance > maxDistance)
			{
				result = icpPoints;
				maxDistance = distance;
				index = i;
			}
			//break;
		}
	}
	Point pResult;
	cout << "\nIndex: " << index << endl;
	if (index != -1)
	{
		pResult = originalList.at(index);
		originalList.at(index).toString();
	}
	else
	{
		cout << "\n0\t0";
		pResult.setX(0);
		pResult.setY(0);
	}
	return pResult;

}

int main(int argc, char* argv[])
{
	/*
	 * Chon 1 hinh anh lam model, vd Prono_077.JPG
	 * Chon vi tri landmark can kiem tra, vd index = 7
	 */
	cout << "\n Procrustes analysis helper !!!" << endl;

	string imagePath =
		"/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original/Prono_077.JPG";
	string mLandmarksPath =
		"/home/linh/Desktop/data/pronotum_data_5/data_aug/landmarks/p_077.TPS";
	int lmIndex = 6; // landmark 7
	Image image(imagePath);
	vector<Point> mLandmarks = image.readManualLandmarks(mLandmarksPath);
	vector<Point> curvePoints;
	image.cannyAlgorithm(curvePoints);
	Point origin(0, 0);
	vector<Point> meanCurve = extract_BBox(curvePoints, mLandmarks.at(lmIndex), 7,
		origin);
	Matrix<int> meanMatrix = list_Points_To_Matrix(meanCurve, 7, origin);

	//

	cout << "\n";
	for (int r = 0; r < meanMatrix.getRows(); r++)
	{
		for (int c = 0; c < meanMatrix.getCols(); c++)
		{
			int v = meanMatrix.getAtPosition(r, c);
			cout << "\t" << v;
		}
		cout << "\n";
	}

	// run on the list
	vector<Point> listPoints;
	listPoints.push_back(Point(109,45));
	listPoints.push_back(Point(104,41));
	listPoints.push_back(Point(103,35));
	listPoints.push_back(Point(106,30));
	listPoints.push_back(Point(98,41));
	listPoints.push_back(Point(106,27));
	listPoints.push_back(Point(101,35));
	listPoints.push_back(Point(108,39));
	listPoints.push_back(Point(106,47));
	listPoints.push_back(Point(106,46));
	listPoints.push_back(Point(108,39));
	listPoints.push_back(Point(111,41));
	listPoints.push_back(Point(102,37));
	listPoints.push_back(Point(110,42));
	listPoints.push_back(Point(107,23));
	listPoints.push_back(Point(107,20));
	listPoints.push_back(Point(103,37));
	listPoints.push_back(Point(110,33));
	listPoints.push_back(Point(109,43));
	listPoints.push_back(Point(101,44));
	listPoints.push_back(Point(105,23));
	listPoints.push_back(Point(107,21));
	listPoints.push_back(Point(107,38));
	listPoints.push_back(Point(107,31));
	listPoints.push_back(Point(102,35));
	listPoints.push_back(Point(115,49));
	listPoints.push_back(Point(99,34));
	listPoints.push_back(Point(107,38));
	listPoints.push_back(Point(107,44));
	listPoints.push_back(Point(107,33));
	listPoints.push_back(Point(100,42));
	listPoints.push_back(Point(101,37));
	listPoints.push_back(Point(110,42));
	listPoints.push_back(Point(108,34));
	listPoints.push_back(Point(113,22));
	listPoints.push_back(Point(107,47));
	listPoints.push_back(Point(102,34));
	listPoints.push_back(Point(104,36));
	listPoints.push_back(Point(109,41));
	listPoints.push_back(Point(104,42));
	listPoints.push_back(Point(102,46));
	listPoints.push_back(Point(113,37));
	listPoints.push_back(Point(111,49));






	string imgsFolder =
		"/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original/";
	vector<string> listImages;
	listImages.push_back("Prono_251.JPG");
	listImages.push_back("Prono_252.JPG");
	listImages.push_back("Prono_253.JPG");
	listImages.push_back("Prono_254.JPG");
	listImages.push_back("Prono_255.JPG");
	listImages.push_back("Prono_256.JPG");
	listImages.push_back("Prono_257.JPG");
	listImages.push_back("Prono_258.JPG");
	listImages.push_back("Prono_259.JPG");
	listImages.push_back("Prono_260.JPG");
	listImages.push_back("Prono_261.JPG");
	listImages.push_back("Prono_262.JPG");
	listImages.push_back("Prono_263.JPG");
	listImages.push_back("Prono_264.JPG");
	listImages.push_back("Prono_265.JPG");
	listImages.push_back("Prono_266.JPG");
	listImages.push_back("Prono_267.JPG");
	listImages.push_back("Prono_268.JPG");
	listImages.push_back("Prono_269.JPG");
	listImages.push_back("Prono_270.JPG");
	listImages.push_back("Prono_271.JPG");
	listImages.push_back("Prono_272.JPG");
	listImages.push_back("Prono_273.JPG");
	listImages.push_back("Prono_274.JPG");
	listImages.push_back("Prono_275.JPG");
	listImages.push_back("Prono_276.JPG");
	listImages.push_back("Prono_277.JPG");
	listImages.push_back("Prono_278.JPG");
	listImages.push_back("Prono_279.JPG");
	listImages.push_back("Prono_280.JPG");
	listImages.push_back("Prono_281.JPG");
	listImages.push_back("Prono_282.JPG");
	listImages.push_back("Prono_283.JPG");
	listImages.push_back("Prono_284.JPG");
	listImages.push_back("Prono_285.JPG");
	listImages.push_back("Prono_286.JPG");
	listImages.push_back("Prono_287.JPG");
	listImages.push_back("Prono_288.JPG");
	listImages.push_back("Prono_289.JPG");
	listImages.push_back("Prono_290.JPG");
	listImages.push_back("Prono_291.JPG");
	listImages.push_back("Prono_292.JPG");
	listImages.push_back("Prono_293.JPG");






	vector<Point> result;
	for (int i = 0; i < listPoints.size(); i++)
	{
		Point estLM = listPoints.at(i);
		string filename = imgsFolder + listImages.at(i);
		ptr_Image img = new Image(filename);
		vector<vector<Point> > points = extract_Shape_Manual_Landmark(*img, estLM,
			12, 7);
		Point rs = checkList_Matrices(points, meanMatrix);
		result.push_back(rs);
		delete img;
	}
	cout << "\nResults: \n";
	for (int i = 0; i < result.size(); i++)
	{
		(result.at(i)).toString();
	}

}
