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
vector<Point> point_Equal_Y(Point p, vector<Point> &cPoints)
{
	vector<Point> result;
	for (size_t i = 0; i < cPoints.size(); i++)
	{
		Point pi = cPoints.at(i);
		if (pi.getX() != p.getX() && pi.getY() == p.getY())
		{
			result.push_back(pi);
			cPoints.erase(cPoints.begin() + i);
		}
	}
	return result;
}
vector<Point> reduce_Contours_Points2(vector<Point> cPoints)
{
	vector<Point> result;
	size_t nPoints = cPoints.size();
	for (size_t i = 0; i < cPoints.size(); i++)
	{
		Point pi = cPoints.at(i);
		vector<Point> equalPoints = point_Equal_Y(pi, cPoints);
		if (equalPoints.size() == 0)
		{
			result.push_back(pi);
		}
		else
		{
			int totalX = pi.getX();
			for (size_t j = 0; j < equalPoints.size(); j++)
			{
				Point pj = equalPoints.at(j);
				totalX += pj.getX();
			}
			result.push_back(
				Point(round((double) totalX / ((double) equalPoints.size() + 1)),
					pi.getY()));
		}
	}

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
 * Applied for 4th, 6th landmark
 */
vector<Point> extract_Contours2(Matrix<int> mapContours)
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
		std::sort(piPinBox.begin(), piPinBox.end(), xComparationTest);
		piPinBox = reduce_Contours_Points(piPinBox);
		piPinBox.push_back(pi); // them diem cuoi cung de biet vi tri ban dau trong contour points
		ppPoints.push_back(piPinBox);
	}
	return ppPoints;
}

/*
 * Root mean square distance (RMSD)
 */
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
		Line l(pi, pj);
		totalDistance += (l.getLength() * l.getLength());
	}

	return sqrt(totalDistance / (double) meanPoints.size());
}
//vector<Point>
Point checkList(vector<vector<Point> > lcpPoints, vector<Point> meanPoints)
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
/*
 * Calculate the distance between the mean curve and each curve in training set
 */
void mean_Distance_MeanCurve(string folderPath, vector<Point> meanPoints)
{
	vector<string> txtFiles = readDirectory(folderPath.c_str());
	int count = 0;
	cout << "\nDistance beween each model to mean model" << endl;
	for (size_t i = 0; i < txtFiles.size(); i++)
	{
		string fileName = folderPath + "/" + txtFiles.at(i);
		Matrix<int> pMatrix = parse_Matrix_From_File(fileName);
		vector<Point> cPoints = extract_Contours(pMatrix);
		if (cPoints.size() == meanPoints.size())
		{
			double mdistance = measure_Distance(meanPoints, cPoints);
			cout << "\n" << mdistance;
		}
	}
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

Point checkList_Matrices(vector<vector<Point> > lcpPoints,
	Matrix<int> meanMatrix)
{
	/*double maxDistance = DBL_MIN;
	 Matrix<int> maxMatrix;
	 for (int i = 0; i < listMatrices.size(); i++)
	 {
	 Matrix<int> mi = listMatrices.at(i);
	 double distance = bhattacharyya_coefficient(meanMatrix, mi);
	 if (distance > maxDistance)
	 {
	 maxDistance = distance;
	 maxMatrix = mi;
	 }
	 }*/

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

void segmentation_Folder(string folderPath, string saveFolder)
{
	// read directory
	vector<string> txtFiles = readDirectory(folderPath.c_str());

	int count = 0;
	for (size_t i = 0; i < txtFiles.size(); i++)
	{
		string fileName = folderPath + "/" + txtFiles.at(i);
		ptr_Image imgPtr = new Image(fileName);
		vector<Point> cPoints;

		imgPtr->cannyAlgorithm(cPoints); //= mae_Canny_Algorithm(imgPtr);
		Matrix<RGB> rgbMatrix = imgPtr->getRGBMatrix();

		ptr_RGBMatrix saveMatrix = new Matrix<RGB>(rgbMatrix.getRows(),
			rgbMatrix.getCols(), rgbMatrix.getAtPosition(0, 0));
		*saveMatrix = rgbMatrix;
		RGB red;
		red.R = 255;
		red.G = red.B = 0;
		for (size_t i = 0; i < cPoints.size(); i++)
		{
			Point pi = cPoints.at(i);
			saveMatrix->setAtPosition(pi.getY(), pi.getX(), red);
		}
		string savefile = saveFolder + "/" + txtFiles.at(i);
		saveRGB(savefile.c_str(), saveMatrix);
		//delete saveMatrix;
		//delete imgPtr;
	}
	cout << "\nDone!\n";
}
/*
 * This program try to improve the location of a specific landmark (i.e lm7, lm3)
 * Step 1: Extract the patch around the manual landmark of all images
 * Step 2: Calculate the mean curve of the manual landmarks (mean patch)
 * Step 3: Extract a patch around predicted landmark, extract the curve inside the patch
 * Step 4: For each pixel of curve, extract a patch and calculate the measure with mean patch
 */

int main(int argc, char* argv[])
{
	cout << "\n Procrustes analysis helper !!!" << endl;
// ================================================================================================
// Try to extract contours for every images
	/*string imageFolder = "/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original";
	 string saveFolder = "/home/linh/Desktop/results/2017/pronotum/256x192_segmentation";
	 segmentation_Folder(imageFolder,saveFolder);*/
// ================================================================================================
	string folderpath =
		"/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct_lm7";
	vector<Point> meanCurve = parse_Folder(folderpath);
	cout << "\nMean curve: " << meanCurve.size();
	for (int i = 0; i < meanCurve.size(); ++i)
	{
		Point pi = meanCurve.at(i);
		pi.toString();
	}
	//mean_Distance_MeanCurve(folderpath, meanCurve);
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

	// =================================== the mean curve is selected by a user ========================================
	/*string fileName =
	 "/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct_lm3/Prono_084.txt";
	 Matrix<int> pMatrix = parse_Matrix_From_File(fileName);
	 vector<Point> meanCurve = extract_Contours(pMatrix);
	 cout<<"\nNumber of points: "<<meanCurve.size();*/

	// ============================================= run on the list ============================================================
	vector<Point> listPoints;

	listPoints.push_back(Point(107, 51));
	listPoints.push_back(Point(104, 52));
	listPoints.push_back(Point(110, 47));
	listPoints.push_back(Point(106, 36));
	listPoints.push_back(Point(111, 51));
	listPoints.push_back(Point(108, 40));
	listPoints.push_back(Point(111, 41));
	listPoints.push_back(Point(111, 28));
	listPoints.push_back(Point(109, 49));
	listPoints.push_back(Point(105, 49));
	listPoints.push_back(Point(101, 19));
	listPoints.push_back(Point(110, 41));
	listPoints.push_back(Point(107, 48));
	listPoints.push_back(Point(111, 42));
	listPoints.push_back(Point(107, 38));
	listPoints.push_back(Point(107, 47));
	listPoints.push_back(Point(106, 51));
	listPoints.push_back(Point(105, 53));
	listPoints.push_back(Point(108, 49));
	listPoints.push_back(Point(106, 35));

	string imgsFolder =
		"/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original/";
	vector<string> listImages;

	listImages.push_back("Prono_001.JPG");
	listImages.push_back("Prono_002.JPG");
	listImages.push_back("Prono_003.JPG");
	listImages.push_back("Prono_004.JPG");
	listImages.push_back("Prono_005.JPG");
	listImages.push_back("Prono_006.JPG");
	listImages.push_back("Prono_007.JPG");
	listImages.push_back("Prono_008.JPG");
	listImages.push_back("Prono_009.JPG");
	listImages.push_back("Prono_010.JPG");
	listImages.push_back("Prono_011.JPG");
	listImages.push_back("Prono_012.JPG");
	listImages.push_back("Prono_013.JPG");
	listImages.push_back("Prono_014.JPG");
	listImages.push_back("Prono_015.JPG");
	listImages.push_back("Prono_016.JPG");
	listImages.push_back("Prono_017.JPG");
	listImages.push_back("Prono_018.JPG");
	listImages.push_back("Prono_019.JPG");
	listImages.push_back("Prono_020.JPG");

	vector<Point> result;
	for (int i = 0; i < listPoints.size(); i++)
	{
		Point estLM = listPoints.at(i);
		string filename = imgsFolder + listImages.at(i);
		ptr_Image img = new Image(filename);
		vector<vector<Point> > points = extract_Shape_Manual_Landmark(*img, estLM,
			16, 7);
		Point rs = checkList(points, meanCurve);
		result.push_back(rs);
		delete img;
	}
	cout << "\nResults: \n";
	for (int i = 0; i < result.size(); i++)
	{
		(result.at(i)).toString();
	}
}
