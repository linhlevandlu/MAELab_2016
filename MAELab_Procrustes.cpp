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
/*
 * Read the list of landmarks from txt file
 */
vector<Point> parse_Landmarks_From_File(string txtfile)
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
	vector<Point> cPoints;
	for (int r = 0; r < rows; r++)
	{
		int index1 = r * cols;
		int index2 = r * cols + 1;
		int vindex = atoi(temp.at(index1));
		int vindex2 = atoi(temp.at(index2));
		cPoints.push_back(Point(vindex, vindex2));
	}
	//cout << "\nNumber of points: " << cPoints.size();
	return cPoints;
}
vector<vector<Point> > parse_Landmarks_From_Folder(string folderPath)
{
	vector<string> txtFiles = readDirectory(folderPath.c_str());
	vector<vector<Point> > results;
	for (size_t i = 0; i < txtFiles.size(); i++)
	{
		string fileName = folderPath + "/" + txtFiles.at(i);
		vector<Point> parse = parse_Landmarks_From_File(fileName);
		results.push_back(parse);
	}
	return results;
}
/*
 * Vector points to matrix
 */

Matrix<double> points_To_Matrix(vector<Point> cPoints)
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

//================================================================================
/*
 * Matching two shapes
 *
 */
/*
 * Generating the identity matrix
 */
Matrix<double> identity_Matrix(int size)
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

/*
 * Genenrate the "deviation" matrix: all elements equal to 1/p
 */
Matrix<double> ip_Matrix(int size)
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

/*
 * Calculate the trace of a matrix.
 * It equals to the sum of all elements belong to the main diagonal
 */
double trace_Matrix(Matrix<double> mMatrix)
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
void print_Matrix(Matrix<double> mMatrix)
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
 * Center the coordinates of one object on other
 */
Matrix<double> center_Coordinates(vector<Point> cPoints)
{
	int ncols = 2;
	int nrows = (int) cPoints.size();
	Matrix<double> pMatrix = points_To_Matrix(cPoints);

	Matrix<double> iMatrix = identity_Matrix(nrows);
	Matrix<double> piMatrix = ip_Matrix(nrows);
	Matrix<double> subtraction = iMatrix.subtract(piMatrix, 0.0);
	Matrix<double> centerMatrix = subtraction.multiply(pMatrix, 0.0);
	return centerMatrix;
}

/*double square_Distance(vector<Point> cPoints)
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
 }*/

/*Matrix<double> scale_Object(vector<Point> cPoints)
 {
 Matrix<double> cCoordinates = center_Coordinates(cPoints);
 double sDistance = square_Distance(cPoints);
 Matrix<double> scaleMatrix = cCoordinates.multiplyScalar(0.0, 1 / sDistance);
 cout << "\nSize of scaled Matrix: " << scaleMatrix.getRows() << "\t"
 << scaleMatrix.getCols() << endl;
 //printMatrix(scaleMatrix);
 return scaleMatrix;
 }*/
double scale_Value(Matrix<double> objMatrix)
{
	int ncols = objMatrix.getCols();
	int nrows = objMatrix.getRows();
	Matrix<double> objMatrixT = objMatrix.transposition(0);
	// I, P, I-P matrices
	Matrix<double> iMatrix = identity_Matrix(nrows);
	Matrix<double> piMatrix = ip_Matrix(nrows);
	Matrix<double> ipiMatrix = iMatrix.subtract(piMatrix, 0);
	Matrix<double> vt = ipiMatrix.multiply(objMatrix, 0);
	Matrix<double> vp = objMatrixT.multiply(ipiMatrix, 0);
	Matrix<double> all = vt.multiply(vp, 0.0);
	double scaleValue = 1 / sqrt(trace_Matrix(all));
	return scaleValue;
}
/*
 * Scale the object
 */
Matrix<double> scale_Object(vector<Point> cPoints)
{
	Matrix<double> objMatrix = points_To_Matrix(cPoints);
	Matrix<double> cMatrix = center_Coordinates(cPoints);
	double sValue = scale_Value(objMatrix);
	Matrix<double> result = cMatrix.multiplyScalar(0.0, sValue);

	return result;
}

/*
 * return: angle in radians
 */
double calculate_Angle_Rotation(Matrix<double> mt1, Matrix<double> mt2)
{
	//Matrix<double> mt1 = scale_Object(cPoint1);
	Matrix<double> mt1t = mt1.transposition(0.0);

	//Matrix<double> mt2 = scale_Object(cPoints2);
	Matrix<double> mt2t = mt2.transposition(0.0);

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
	Matrix<double> mta = x21t.multiply(x12, 0.0);
	Matrix<double> mtb = x11t.multiply(x12, 0.0);
	Matrix<double> mtc = x11t.multiply(x21, 0.0);
	Matrix<double> mtd = x12t.multiply(x21, 0.0);

	Matrix<double> mtab = mta.subtract(mtb, 0.0);
	Matrix<double> mtcd = mtc.subtract(mtd, 0.0);

	double angle = 0;
	if (mtab.getRows() == mtcd.getRows() && mtab.getCols() == mtcd.getCols()
		&& mtab.getRows() == mtab.getCols() && mtab.getRows() == 1)
	{
		//cout << "\n Right";
		double x = mtab.getAtPosition(0, 0);
		double y = mtcd.getAtPosition(0, 0);
		double tanx = -x / y;
		angle = atan(tanx);
	}
	else
	{
		cout << "\n Error: ";
		angle = atan(0);
	}
	//cout << "\nAngle: " << angle << endl;
	return angle;
}

/*double inner_Product_Matrices(Matrix<double> mt1, Matrix<double> mt2)
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

 double cal_Angle_Rotation2(vector<Point> cPoints1, vector<Point> cPoints2)
 {
 if (cPoints1.size() == cPoints2.size())
 {
 Matrix<double> mt1 = points_To_Matrix(cPoints1);
 Matrix<double> mt2 = points_To_Matrix(cPoints2);
 double ab = inner_Product_Matrices(mt1, mt2);
 double aa = inner_Product_Matrices(mt1, mt1);
 double bb = inner_Product_Matrices(mt2, mt2);
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
 }*/

Matrix<double> create_Rotated_Matrix(double angle)
{
	Matrix<double> rotationMatrix(2, 2, 0);
	rotationMatrix.setAtPosition(0, 0, cos(angle));
	rotationMatrix.setAtPosition(0, 1, -sin(angle));
	rotationMatrix.setAtPosition(1, 0, sin(angle));
	rotationMatrix.setAtPosition(1, 1, cos(angle));
	return rotationMatrix;
}
/*
 * Rotate a matrix by an angle (in radian)
 */
Matrix<double> rotate(Matrix<double> mt, double angleR)
{
	//Matrix<double> scaleMatrix = scale_Object(cPoint);
	Matrix<double> rotatedMatrix = create_Rotated_Matrix(angleR);
	Matrix<double> result = mt.multiply(rotatedMatrix, 0.0);
	//print_Matrix(result);
	return result;
}

double measure_Difference(Matrix<double> mt1, Matrix<double> mt2)
{
	Matrix<double> subtraction = mt1.subtract(mt2, 0.0);
	Matrix<double> subtractiont = subtraction.transposition(0.0);
	Matrix<double> mul = subtraction.multiply(subtractiont, 0.0);
	//cout << "\nSize of mul: " << mul.getRows() << "\t" << mul.getCols() << endl;
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

Matrix<double> matching_Two_Matrices(Matrix<double> mt1, Matrix<double> mt2)
{
	double angle = calculate_Angle_Rotation(mt1, mt2);
	Matrix<double> mt2r = rotate(mt2, angle);
	double mDistance = measure_Difference(mt1, mt2r);
	cout << "\nDistance: " << mDistance;
	return mt2r;
}
void matching_Two_Lists(vector<Point> cPoints1, vector<Point> cPoints2)
{
	Matrix<double> mt1 = scale_Object(cPoints1);
	Matrix<double> mt2 = scale_Object(cPoints2);
	matching_Two_Matrices(mt1, mt2);

}

/*
 * ======================================================== GENERALIZED ORTHOGONAL PROCRUSTES ANALYSIS ===============================
 */
vector<Matrix<double> > step1(vector<vector<Point> > listCPoints)
{
	vector<Matrix<double> > listMatrices;
	for (size_t i = 0; i < listCPoints.size(); i++)
	{
		vector<Point> listi = listCPoints.at(i);
		Matrix<double> mti = scale_Object(listi);
		listMatrices.push_back(mti);
	}
	return listMatrices;
}

Matrix<double> step23(vector<Matrix<double> > listMatrices)
{
	Matrix<double> meany = listMatrices.at(0);
	for (size_t i = 1; i < listMatrices.size(); i++)
	{
		Matrix<double> xi = listMatrices.at(i);
		Matrix<double> xie = matching_Two_Matrices(meany, xi);

		// calculate the mean of Y and Xi rotated
		meany = meany.add(xie, 0.0);
		meany = meany.multiplyScalar(0.0, 0.5);
	}
	return meany;
}

/*
 * Step 4
 * compute the residual sum of squares as Sr
 */
double residual_Sum_Of_Square(Matrix<double> meanMatrix, int n)
{
	Matrix<double> mMatrixT = meanMatrix.transposition(0.0);
	Matrix<double> mmMatrix = meanMatrix.multiply(mMatrixT, 0.0);
	double tr = trace_Matrix(mmMatrix);
	return n * (1 - tr);
}

/*
 * Step 5
 */
vector<double> scale_Initialize(int n)
{
	vector<double> scales;
	for (int i = 0; i < n; i++)
	{
		scales.push_back(1.0);
	}
	return scales;
}

Matrix<double> mean_Matrices(vector<Matrix<double> > listMatrices)
{

	Matrix<double> result = listMatrices.at(0);
	for (size_t i = 0; i < listMatrices.size(); i++)
	{
		Matrix<double> mti = listMatrices.at(i);
		result = result.add(mti, 0.0);
		result = result.multiplyScalar(0.0, 0.5);
	}
	return result;
}

double recompute_Scale(Matrix<double> mt1, Matrix<double> mt2)
{
	Matrix<double> mt1t = mt1.transposition(0.0);
	Matrix<double> mt2t = mt2.transposition(0.0);

	Matrix<double> t1 = mt2.multiply(mt1t, 0.0);
	Matrix<double> t2 = mt2.multiply(mt2t, 0.0);
	Matrix<double> t3 = mt1.multiply(mt1t, 0.0);

	double trace1 = trace_Matrix(t1);
	double trace2 = trace_Matrix(t2);
	double trace3 = trace_Matrix(t3);
	return sqrt(trace1 / (trace2 * trace3));
}

void GPA(vector<vector<Point> > listPoints)
{
	vector<Matrix<double> > listMatrices = step1(listPoints);
	Matrix<double> meany1 = step23(listMatrices);
	double s1 = residual_Sum_Of_Square(meany1, (int) listMatrices.size());

	vector<double> sfactors = scale_Initialize((int) listMatrices.size());
	bool stop = true;
	do
	{
		vector<Matrix<double> > list2;
		for (size_t i = 0; i < listMatrices.size(); i++)
		{
			Matrix<double> xi = listMatrices.at(i);
			double factor = sfactors.at(i);
			double angle = calculate_Angle_Rotation(meany1, xi);
			xi = xi.multiplyScalar(0.0, factor);
			Matrix<double> xie = rotate(xi, angle);
			list2.push_back(xie);
		}

		// step 6.2
		Matrix<double> meanY2 = mean_Matrices(list2);

		// step 6.3
		vector<Matrix<double> > list22;
		vector<double> sfactor2;
		for (size_t i = 0; i < list2.size(); i++)
		{
			Matrix<double> mti = list2.at(i);
			double sc = recompute_Scale(meanY2, mti);
			Matrix<double> mti2 = mti.multiplyScalar(0.0, sc);
			list22.push_back(mti2);
			sfactor2.push_back(sc * (double) sfactors.at(i));
		}
		// step 6.4
		Matrix<double> meanY22 = mean_Matrices(list22);
		double s2 = residual_Sum_Of_Square(meanY22, (int) list22.size());

		double cond = s1 - s2;
		if (cond >= 0.001)
		{
			stop = false;
			meany1 = meanY22;
			sfactors = sfactor2;
			listMatrices = list22;
		}
		else
		{
			meany1 = meanY22;
			stop = true;
		}
	}
	while (!stop);
	print_Matrix(meany1);
}

Matrix<int> read_All_Landmarks(string txtfile)
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
	vector<Point> cPoints;
	for (int r = 0; r < rows; r++)
	{
		int index1 = r * cols;
		int index2 = r * cols + 1;
		int vindex = atoi(temp.at(index1));
		int vindex2 = atoi(temp.at(index2));
		cPoints.push_back(Point(vindex, vindex2));
	}
	cout << "\nNumber of points: " << cPoints.size();
	vector<vector<Point> > vpPoints;
	for (int i = 0; i < cPoints.size(); i += 8)
	{
		vector<Point> pjPoints;
		for (int j = i; j < i + 8; j++)
		{
			Point pj = cPoints.at(j);
			pjPoints.push_back(pj);
		}
		vpPoints.push_back(pjPoints);
	}
	cout << "\nNumber of vpPoints: " << vpPoints.size();
	string folder =
		"/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct_all_IMG";
	for (size_t i = 0; i < vpPoints.size(); i++)
	{
		stringstream strs;
		strs << (i+1);
		string temp_str = strs.str();
		string filename = folder + "/Prono_" + temp_str + ".txt";
		ofstream outfile(filename.c_str());
		vector<Point> pjs = vpPoints.at(i);
		outfile<<"LM="<<pjs.size()<<"\n";
		for (int j = 0; j < pjs.size(); j++)
		{
			Point pj = pjs.at(j);
			outfile << pj.getX() << " " << (192-pj.getY()) << "\n";
		}
		outfile.close();
	}
	return pMatrix;
}

/*
 * This program works on Generalized Procrutes Analysis
 * With a set of landmark points:
 * Step 1: We generate the mean model (checked)
 * Step 2: Adapt the mean model to each image, try to scale, rotate, translate
 * 				 to fit the model with the image (How? Active contours ?)
 * Step 3: Determine the location of the landmarks
 */
int main(int argc, char* argv[])
{
	cout << "\n Procrustes analysis helper !!!" << endl;
	/**
	 * Extract the contours around the landmark and save to the file
	 */
	/*string imagePath =
	 "/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original/Prono_001.JPG";
	 string lmPath =
	 "/home/linh/Desktop/data/pronotum_data_5/landmarks/train/p_001.TPS";
	 int lmIndex = 3;
	 string fSave =
	 "/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct_lm2/";
	 int bsize = 7;
	 if (argc == 6)
	 {
	 cout << "\nWith parameters...";
	 imagePath = argv[1];
	 lmPath = argv[2];
	 lmIndex = atoi(argv[3]);
	 bsize = atoi(argv[4]);
	 fSave = argv[5];
	 }*/

	string lmPath = "/home/linh/Desktop/all.txt";
	read_All_Landmarks(lmPath);
	/**
	 * General Procrustes Analysis
	 *
	 */

	/*string fLandmarks =
		"/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct_all";
	vector<vector<Point> > listOfLandmarks = parse_Landmarks_From_Folder(
		fLandmarks);
	GPA(listOfLandmarks);*/
}
