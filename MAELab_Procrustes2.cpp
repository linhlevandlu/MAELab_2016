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
 * Parse the matrix from a text file
 */
Matrix<int> parse_Matrix_From_File(string txtfile)
{
	ifstream openFile(txtfile.c_str());
	vector<char*> temp;
	string lineText;
	int rows = 0, cols = 0;
	if (openFile.is_open())
	{

		while (getline(openFile, lineText))
		{
			rows++;
			char * pch;
			pch = strtok(strdup(lineText.c_str()), "\t");
			cols = 0;
			while (pch != NULL)
			{
				cols++;
				char* s = pch;
				temp.push_back(s);
				pch = strtok(NULL, "\t");
			}
		}
	}
	else
	{
		cout << endl << "Cannot open this file !";
	}
	openFile.close();

	Matrix<int> pMatrix(rows, cols);
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			int index = r * cols + c;
			int vindex = atoi(temp.at(index));
			pMatrix.setAtPosition(r, c, vindex);
		}
	}
	return pMatrix;
}
void printMatrix(Matrix<double> mMatrix)
{
	cout << "\nThe values of the matrix: \n";
	for (int r = 0; r < mMatrix.getRows(); r++)
	{
		cout << "\n";
		for (int c = 0; c < mMatrix.getCols(); c++)
		{
			cout << "\t" << mMatrix.getAtPosition(r, c);
		}
	}
	cout << "\n";
}
/*
 * Finding the points in cPoints that have the same x-coordinate with p
 */
vector<Point> point_Equal_X(Point p, vector<Point> &cPoints)
{
	vector<Point> result;
	for (size_t i = 0; i < cPoints.size(); i++)
	{
		Point pi = cPoints.at(i);
		if (pi.getX() == p.getX() && pi.getY() != p.getY())
		{
			result.push_back(pi);
			cPoints.erase(cPoints.begin() + i);
		}
	}
	return result;
}
vector<Point> reduce_Contours_Points(vector<Point> cPoints)
{
	vector<Point> result;
	size_t nPoints = cPoints.size();
	for (size_t i = 0; i < cPoints.size(); i++)
	{
		Point pi = cPoints.at(i);
		vector<Point> equalPoints = point_Equal_X(pi, cPoints);
		if (equalPoints.size() == 0)
		{
			result.push_back(pi);
		}
		else
		{
			int totalY = pi.getY();
			for (size_t j = 0; j < equalPoints.size(); j++)
			{
				Point pj = equalPoints.at(j);
				totalY += pj.getY();
			}
			result.push_back(
				Point(pi.getX(),
					round((double) totalY / ((double) equalPoints.size() + 1))));
		}
	}
	/*for (int i = 0; i < result.size(); ++i) {
	 cout<<"\n"<<result.at(i).getX()<<"\t"<<result.at(i).getY();
	 }*/
	return result;
}
/**
 * Extract the pixels belong to the contours and store into the list
 */
vector<Point> extract_Contours(Matrix<int> mapContours)
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
}
/*
 * Calculate the centroid of a list of contour points
 */
Point cal_Centroid(vector<Point> ctPoints)
{
	Point ct(0, 0);
	int totalX = 0, totalY = 0;
	int totalPoints = (int) ctPoints.size();
	for (int t = 0; t < totalPoints; t++)
	{
		Point p = ctPoints.at(t);
		totalX += p.getX();
		totalY += p.getY();
	}
	ct.setX(totalX / totalPoints);
	ct.setY(totalY / totalPoints);
	return ct;
}
//================================================================================

vector<Point> parse_Folder(string folderPath)
{
	// read directory
	vector<string> txtFiles = readDirectory(folderPath.c_str());
	int xarr[7] =
	{ 0 };
	int yarr[7] =
	{ 0 };

	int count = 0;
	for (size_t i = 0; i < txtFiles.size(); ++i)
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
}

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
	double minDistance = DBL_MAX;
	Point landmark = estLandmark;
	for (size_t i = 0; i < cPoints.size(); i++)
	{
		Point pi = cPoints.at(i);
		Line line(estLandmark, pi);
		if (line.getLength() < minDistance)
		{
			landmark = pi;
			minDistance = line.getLength();
		}
	}

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
		std::sort(piPinBox.begin(), piPinBox.end(), xComparationTest);
		piPinBox = reduce_Contours_Points(piPinBox);
		piPinBox.push_back(pi); // them diem cuoi cung de biet vi tri ban dau trong contour points
		ppPoints.push_back(piPinBox);
	}
	return ppPoints;
}

double measure_Distance(vector<Point> meanPoints, vector<Point> cpPoints)
{
	//cout << "\n Number of points: " << cpPoints.size();
	if (meanPoints.size() != cpPoints.size())
		return DBL_MAX;
	double totalDistance = 0;
	for (size_t i = 0; i < meanPoints.size(); i++)
	{
		Point pi = meanPoints.at(i);
		Point pj = cpPoints.at(i);
		totalDistance += Line(pi, pj).getLength();
	}
	return totalDistance;
}
Point checkList(vector<vector<Point> > lcpPoints,
	vector<Point> meanPoints)
{
	vector<Point> result;
	double minDistance = DBL_MAX;
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
			double distance = measure_Distance(meanPoints, icpPoints);
			if (distance < minDistance)
			{
				result = icpPoints;
				minDistance = distance;
				index = i;
			}
		}
	}
	Point pResult;
	cout << "\nIndex: " << index << endl;
	if (index != -1){
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
	cout << "\n Procrustes analysis helper !!!" << endl;
	/**
	 * Paser a file which contains the contours into a matrix
	 *
	 */
	/*string filename =
	 "/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct/Prono_001.txt";
	 Matrix<int> mParse = parse_Matrix_From_File(filename);
	 vector<Point> cMap = extract_Contours(mParse);
	 cout << "\nNumber of contour points: " << cMap.size() << endl;
	 Point centroid = cal_Centroid(cMap);
	 cout << "\n" << centroid.getX() << "\t" << centroid.getY() << "\n";

	 // file 2
	 string filename2 =
	 "/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct/Prono_006.txt";
	 Matrix<int> mParse2 = parse_Matrix_From_File(filename2);
	 vector<Point> cMap2 = extract_Contours(mParse2);
	 cout << "\nNumber of contours points 2: " << cMap2.size() << endl;
	 Point centroid2 = cal_Centroid(cMap2);
	 cout << "\n" << centroid2.getX() << "\t" << centroid2.getY() << "\n";*/
	string folderpath =
		"/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct_lm3";
	vector<Point> meanCurve = parse_Folder(folderpath);

	/*string imagePath =
	 "/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original/Prono_003.JPG";
	 Image image(imagePath);
	 Point estLandmark(110, 47);
	 vector<vector<Point> > pPoints = extract_Shape_Manual_Landmark(image,
	 estLandmark, 12, 7);
	 vector<Point> rs = checkList(pPoints, meanCurve);
	 for (int i = 0; i < rs.size(); ++i)
	 {
	 Point pi = rs.at(i);
	 cout << "\n" << pi.getX() << "\t" << pi.getY();
	 }*/

	// run on the list
	vector<Point> listPoints;
	listPoints.push_back(Point(214,23));
	listPoints.push_back(Point(219,22));
	listPoints.push_back(Point(216,34));
	listPoints.push_back(Point(217,32));
	listPoints.push_back(Point(218,46));
	listPoints.push_back(Point(202,46));
	listPoints.push_back(Point(209,18));
	listPoints.push_back(Point(217,13));
	listPoints.push_back(Point(219,40));
	listPoints.push_back(Point(212,34));
	listPoints.push_back(Point(216,28));
	listPoints.push_back(Point(213,61));
	listPoints.push_back(Point(207,28));
	listPoints.push_back(Point(215,44));
	listPoints.push_back(Point(212,51));
	listPoints.push_back(Point(211,33));
	listPoints.push_back(Point(207,41));
	listPoints.push_back(Point(210,38));
	listPoints.push_back(Point(213,46));
	listPoints.push_back(Point(219,34));
	listPoints.push_back(Point(234,30));
	listPoints.push_back(Point(214,44));
	listPoints.push_back(Point(218,36));
	listPoints.push_back(Point(209,36));
	listPoints.push_back(Point(217,43));
	listPoints.push_back(Point(223,50));
	listPoints.push_back(Point(218,46));
	listPoints.push_back(Point(225,45));
	listPoints.push_back(Point(221,52));


	string imgsFolder =
		"/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original/";
	vector<string> listImages;
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
		Image img(filename);
		vector<vector<Point> > points = extract_Shape_Manual_Landmark(img, estLM,
			12, 7);
		Point rs = checkList(points, meanCurve);
		result.push_back(rs);
	}
	cout<<"\nResults: \n";
	for (int i = 0; i < result.size(); i++) {
		(result.at(i)).toString();
	}
}
