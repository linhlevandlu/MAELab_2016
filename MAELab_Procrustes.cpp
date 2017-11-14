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

/**
 * Extract the border shape via a manual landmark inside a box
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

void extract_Shape_Manual_Landmark(Image matImage, string lmark, int lmIndex,
	int bsize, string fSave)
{
	vector<Point> cPoints = mae_Canny_Algorithm(&matImage);
	matImage.readManualLandmarks(lmark);
	Point landmark = matImage.getListOfManualLandmarks().at(lmIndex);

	landmark = landmark_On_Curve(landmark,cPoints);
	//cout << "\nContours points: " << cPoints.size() << endl;
	if (bsize % 2 == 0)
		bsize += 1; // make the size is odd number
	int hsize = bsize / 2;
	int xbegin = landmark.getX() - hsize;
	int xend = landmark.getX() + hsize;
	int ybegin = landmark.getY() - hsize;
	int yend = landmark.getY() + hsize;

	vector<Point> pinBox;
	for (size_t t = 0; t < cPoints.size(); t++)
	{
		Point pi = cPoints.at(t);
		if (pi.getX() >= xbegin && pi.getX() <= xend && pi.getY() >= ybegin
			&& pi.getY() <= yend)
			pinBox.push_back(pi);
	}
	cout << "\nNumber of pixel: " << pinBox.size() << endl;
	Matrix<int> patch(bsize, bsize, 0);
	for (size_t i = 0; i < pinBox.size(); i++)
	{
		Point pi = pinBox.at(i);
		int x = pi.getX() - xbegin;
		int y = pi.getY() - ybegin;
		patch.setAtPosition(y, x, 1);
	}
	string name = matImage.getName();
	name = name.substr(0, name.length() - 3);
	string savefile = fSave + name + "txt";
	ofstream outfile(savefile.c_str());
	for (int i = 0; i < bsize; i++)
	{
		for (int j = 0; j < bsize; j++)
		{
			outfile << patch.getAtPosition(i, j) << "\t";
		}
		outfile << endl;
	}
	outfile.close();
}

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
/*
 * Matching two shapes
 *
 */
Matrix<double> identityMatrix(int size)
{
	Matrix<double> iden(size, size);
	for (int r = 0; r < size; r++)
	{
		for (int c = 0; c < size; c++)
		{
			if (r == c)
			{
				iden.setAtPosition(r, c, 1);
			}
			else
			{
				iden.setAtPosition(r, c, 0);
			}
		}
	}
	return iden;
}
Matrix<double> ipMatrix(int size)
{
	Matrix<double> ipMatrix(size, size);
	for (int r = 0; r < size; r++)
	{
		for (int c = 0; c < size; c++)
		{
			ipMatrix.setAtPosition(r, c, 1 / (double) size);
		}
	}
	return ipMatrix;
}
double traceMatrix(Matrix<double> mMatrix)
{

	double tvalue = 0;
	for (int r = 0; r < mMatrix.getRows(); r++)
	{
		for (int c = 0; c < mMatrix.getCols(); c++)
		{
			if (r == c)
			{
				tvalue += mMatrix.getAtPosition(r, c);
			}
		}
	}
	return tvalue;
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
 * Vector points to matrix
 */

Matrix<double> pointsToMatrix(vector<Point> cPoints)
{
	int ncols = 2;
	int nrows = (int) cPoints.size();
	Matrix<double> pMatrix(nrows, ncols);
	for (int c = 0; c < nrows; c++)
	{
		Point p = cPoints.at(c);
		pMatrix.setAtPosition(c, ncols - 2, (double) p.getX());
		pMatrix.setAtPosition(c, ncols - 1, (double) p.getY());
	}
	return pMatrix;
}
/*
 * Center the coordinates of one object on other
 */
Matrix<double> centerCoordinates(vector<Point> cPoints)
{
	int ncols = 2;
	int nrows = (int) cPoints.size();
	Matrix<double> pMatrix = pointsToMatrix(cPoints);

	Matrix<double> iMatrix = identityMatrix(nrows);
	Matrix<double> piMatrix = ipMatrix(nrows);
	Matrix<double> subtraction = iMatrix.subtract(piMatrix, 0.0);
	Matrix<double> centerMatrix = subtraction.multiply(pMatrix, 0.0);

	//cout<<"\nSize of pMatrix: "<<pMatrix.getRows()<<"\t"<<pMatrix.getCols()<<endl;
	//cout<<"\nSize of iMatrix: "<<iMatrix.getRows()<<"\t"<<iMatrix.getCols()<<endl;
	//cout<<"\nSize of piMatrix: "<<piMatrix.getRows()<<"\t"<<piMatrix.getCols()<<endl;
	//cout<<"\nSize of subtraction: "<<subtraction.getRows()<<"\t"<<subtraction.getCols()<<endl;
	//cout<<"\nSize of centerMatrix: "<<centerMatrix.getRows()<<"\t"<<centerMatrix.getCols()<<endl;
	return centerMatrix;
}

double squareDistance(vector<Point> cPoints)
{
	Point centroid = cal_Centroid(cPoints);
	size_t nPoints = cPoints.size();
	double totalDistance = 0;
	for (int i = 0; i < nPoints; i++)
	{
		Point pi = cPoints.at(i);
		Line l(centroid, pi);
		totalDistance += (l.getLength() * l.getLength());
	}
	return sqrt(totalDistance / (double) nPoints);
}

Matrix<double> scaleObject(vector<Point> cPoints)
{
	Matrix<double> cCoordinates = centerCoordinates(cPoints);
	double sDistance = squareDistance(cPoints);
	Matrix<double> scaleMatrix = cCoordinates.multiplyScalar(0.0, 1 / sDistance);
	cout << "\nSize of scaled Matrix: " << scaleMatrix.getRows() << "\t"
		<< scaleMatrix.getCols() << endl;
	//printMatrix(scaleMatrix);
	return scaleMatrix;
}

Matrix<double> scaleObject2(vector<Point> cPoints)
{
	int ncols = 2;
	int nrows = (int) cPoints.size();
	Matrix<double> pMatrix = pointsToMatrix(cPoints);
	Matrix<double> pMatrixT = pMatrix.transposition(0);

	// I, P, I-P matrices
	Matrix<double> iMatrix = identityMatrix(nrows);
	Matrix<double> piMatrix = ipMatrix(nrows);
	Matrix<double> ipiMatrix = iMatrix.subtract(piMatrix, 0);

	Matrix<double> vt = ipiMatrix.multiply(pMatrix, 0);
	//cout << "\n Vt rows: " << vt.getRows() << "\t cols: " << vt.getCols();
	Matrix<double> vp = pMatrixT.multiply(ipiMatrix, 0);
	//cout << "\n Vp rows: " << vp.getRows() << "\t cols: " << vp.getCols();
	Matrix<double> all = vt.multiply(vp, 0.0);
	double scaleValue = 1 / sqrt(traceMatrix(all));
	//cout << "\nScale value: " << scaleValue;
	Matrix<double> result = vt.multiplyScalar(0.0, scaleValue);

	printMatrix(result);
	return result;
}

/*
 * return: angle in radians
 */
double calAngleRotation(vector<Point> cPoint1, vector<Point> cPoints2)
{
	Matrix<double> mt1 = scaleObject2(cPoint1);
	Matrix<double> mt1t = mt1.transposition(0.0);

	Matrix<double> mt2 = scaleObject2(cPoints2);
	Matrix<double> mt2t = mt2.transposition(0.0);

	/*Matrix<double> x21t(1, mt2t.getCols());
	 for (int i = 0; i < mt2t.getCols(); i++)
	 {
	 double value = mt2t.getAtPosition(0, i);
	 x21t.setAtPosition(0, i, value);
	 }
	 Matrix<double> x11t(1, mt1t.getCols());
	 for (int i = 0; i < mt1t.getCols(); i++)
	 {
	 double value = mt1t.getAtPosition(0, i);
	 x11t.setAtPosition(0, i, value);
	 }
	 Matrix<double> x12t(1, mt1t.getCols());
	 for (int i = 0; i < mt1t.getCols(); i++)
	 {
	 double value = mt1t.getAtPosition(1, i);
	 x11t.setAtPosition(0, i, value);
	 }
	 Matrix<double> x12(mt1.getRows(), 1);
	 for (int i = 0; i < mt1.getRows(); i++)
	 {
	 double value = mt1.getAtPosition(i, 2);
	 x12.setAtPosition(i, 0, value);
	 }
	 Matrix<double> x21(mt2.getRows(), 1);
	 for (int i = 0; i < mt2.getRows(); i++)
	 {
	 double value = mt2.getAtPosition(i, 0);
	 x21.setAtPosition(i, 0, value);
	 }
	 Matrix<double> x22(mt2.getRows(), 1);
	 for (int i = 0; i < mt2.getRows(); i++)
	 {
	 double value = mt2.getAtPosition(i, 1);
	 x22.setAtPosition(i, 0, value);
	 }

	 double a = 0, b = 0, c = 0, d = 0;
	 for (int i = 0; i < x21t.getCols(); i++)
	 {
	 a += (x21t.getAtPosition(0, i) * x12.getAtPosition(i, 0));
	 b += (x11t.getAtPosition(0, i) * x22.getAtPosition(i, 0));
	 c += (x11t.getAtPosition(0, i) * x21.getAtPosition(i, 0));
	 d += (x12t.getAtPosition(0, i) * x21.getAtPosition(i, 0));
	 }
	 double tanx = -(a - b) / (c + d);
	 double x = atan(tanx);
	 cout << "\nAngle: " << x << endl;*/

	double a = 0, b = 0, c = 0, d = 0;
	Matrix<double> x11(mt1.getRows(), 1);
	Matrix<double> x12(mt1.getRows(), 1);
	Matrix<double> x21(mt1.getRows(), 1);
	Matrix<double> x22(mt1.getRows(), 1);
	for (int r = 0; r < mt1.getRows(); r++)
	{
		double v10 = mt1.getAtPosition(r, 0);
		double v11 = mt1.getAtPosition(r, 1);
		x11.setAtPosition(r, 0, v10);
		x12.setAtPosition(r, 0, v11);

		double v20 = mt2.getAtPosition(r, 0);
		double v21 = mt2.getAtPosition(r, 1);
		x21.setAtPosition(r, 0, v20);
		x22.setAtPosition(r, 0, v21);
	}
	Matrix<double> x21t = x21.transposition(0);
	Matrix<double> x11t = x11.transposition(0);
	Matrix<double> x12t = x12.transposition(0);
	for (int i = 0; i < x21t.getCols(); i++)
	{
		a += (x21.getAtPosition(i, 0) * x12.getAtPosition(i, 0));
		b += (x11.getAtPosition(i, 0) * x22.getAtPosition(i, 0));
		c += (x11.getAtPosition(i, 0) * x21.getAtPosition(i, 0));
		d += (x12.getAtPosition(i, 0) * x21.getAtPosition(i, 0));
	}
	double tanx = -(a - b) / (c + d);
	double x = atan(tanx);
	cout << "\nAngle: " << x << endl;
	return x;
}

double innerProductMatrices(Matrix<double> mt1, Matrix<double> mt2)
{
	double rs = 0.0;
	for (int r = 0; r < mt1.getRows(); r++)
	{
		for (int c = 0; c < mt1.getCols(); c++)
		{
			double vl1 = mt1.getAtPosition(r, c);
			double vl2 = mt2.getAtPosition(r, c);
			rs += (vl1 * vl2);
		}
	}
	return rs;
}
/*
 * Calculate angle in radians
 */
double calAngleRotation2(vector<Point> cPoints1, vector<Point> cPoints2)
{
	if (cPoints1.size() == cPoints2.size())
	{
		Matrix<double> mt1 = pointsToMatrix(cPoints1);
		Matrix<double> mt2 = pointsToMatrix(cPoints2);
		double ab = innerProductMatrices(mt1, mt2);
		double aa = innerProductMatrices(mt1, mt1);
		double bb = innerProductMatrices(mt2, mt2);
		double cosValue = ab / (sqrt(aa) * sqrt(bb));
		double angle = acos(cosValue);
		cout << "\nAngle 2: " << angle << endl;
		return angle;
	}
	else
	{
		cout << "\n The matching cannot finish.\n";
	}
	return -1;
}

Matrix<double> rotate(vector<Point> cPoint, double angleR)
{
	Matrix<double> scaleMatrix = scaleObject2(cPoint);

	Matrix<double> rotationMatrix(2, 2, 0);
	rotationMatrix.setAtPosition(0, 0, cos(angleR));
	rotationMatrix.setAtPosition(0, 1, -sin(angleR));
	rotationMatrix.setAtPosition(1, 0, sin(angleR));
	rotationMatrix.setAtPosition(1, 1, cos(angleR));
	Matrix<double> result = scaleMatrix.multiply(rotationMatrix, 0.0);
	printMatrix(result);
	return result;
}

double measureDifference(Matrix<double> mt1, Matrix<double> mt2)
{
	Matrix<double> subtraction = mt1.subtract(mt2, 0.0);
	Matrix<double> subtractiont = subtraction.transposition(0.0);
	Matrix<double> mul = subtraction.multiply(subtractiont, 0.0);
	cout << "\nSize of mul: " << mul.getRows() << "\t" << mul.getCols() << endl;
	double trace = 0.0;
	for (int r = 0; r < mul.getRows(); r++)
	{
		for (int c = 0; c < mul.getCols(); c++)
		{
			if (r == c)
			{
				trace += mul.getAtPosition(r, c);
			}
		}
	}
	return trace;
}

int main(int argc, char* argv[])
{
	cout << "\n Procrustes analysis helper !!!" << endl;
	/**
	 * Extract the contours around the landmark and save to the file
	 */
	string imagePath =
		"/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original/Prono_001.JPG";
	string lmPath =
		"/home/linh/Desktop/data/pronotum_data_5/landmarks/train/p_001.TPS";
	int lmIndex = 3;
	string fSave = "/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct_lm2/";
	int bsize = 7;
	if (argc == 6)
	{
		cout << "\nWith parameters...";
		imagePath = argv[1];
		lmPath = argv[2];
		lmIndex = atoi(argv[3]);
		bsize = atoi(argv[4]);
		fSave = argv[5];
	}
	Image image(imagePath);
	extract_Shape_Manual_Landmark(image, lmPath, lmIndex, bsize, fSave);

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
	 "/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct/Prono_003.txt";
	 Matrix<int> mParse2 = parse_Matrix_From_File(filename2);
	 vector<Point> cMap2 = extract_Contours(mParse2);
	 cout << "\nNumber of contours points 2: " << cMap2.size() << endl;
	 Point centroid2 = cal_Centroid(cMap2);
	 cout << "\n" << centroid2.getX() << "\t" << centroid2.getY() << "\n";

	 //scaleObject(cMap);
	 //scaleObject2(cMap);
	 Matrix<double> sMatrix1 = scaleObject2(cMap);
	 double angleR = calAngleRotation(cMap, cMap2);
	 //calAngleRotation2(cMap, cMap2);
	 Matrix<double> rotatedMatrix = rotate(cMap2, angleR);
	 double measure = measureDifference(sMatrix1, rotatedMatrix);
	 cout << "\nDifference measure: " << sqrt(measure/2) << endl;*/
}
