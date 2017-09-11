/*
 *
 *
 * Test file
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

using namespace std;

#include "imageModel/Point.h"
#include "imageModel/Line.h"
#include "imageModel/Edge.h"
#include "imageModel/Matrix.h"
#include "imageModel/Image.h"
#include "utils/Drawing.h"
#include "io/Reader.h"

#include "segmentation/Thresholds.h"
#include "segmentation/Canny.h"
#include "segmentation/Filters.h"
#include "segmentation/Projection.h"
#include "correlation/DescriptorDistance.h"

#include "utils/Converter.h"
/**
 * This function is used to fill the hole inside the object (pronotum)
 * Input: file path of rgb image
 * Output: the output image under Binary format.
 */

ptr_IntMatrix holeFill(string filename, string savename)
{
	Image image(filename);
	Matrix<double> gauKernel = getGaussianKernel(5, 1);
	Matrix<int> result = gaussianBlur(image.getGrayMatrix(), gauKernel);
	ptr_IntMatrix binMatrix = binaryThreshold(&result, image.getThresholdValue(),
		255);
	binMatrix = postProcess(binMatrix, 255);
	saveGrayScale(savename.c_str(), binMatrix);
	return binMatrix;
}

Matrix<int> removelegMain(string filename, string savename)
{
	ptr_IntMatrix binMatrix = holeFill(filename, savename);
	Matrix<int> result = splitImage(binMatrix, 150); // 150 --> height of sub-image
	saveGrayScale(savename.c_str(), &result);
	return result;
}

void getProjections(string filename, string savename)
{
	Matrix<int> binMatrix = removelegMain(filename, savename);
	ptr_IntMatrix hProjection = new Matrix<int>(binMatrix.getRows(),
		binMatrix.getCols(), 255);
	ptr_IntMatrix vProjection(hProjection);
	binProjection(&binMatrix, hProjection, vProjection);
	analysisHistogram(hProjection, 0, 20);
	saveGrayScale(savename.c_str(), hProjection);
}

void colorThreshold(string filename, string savename)
{
	Image matImage(filename);
	Matrix<RGB> temp = matImage.getRGBHistogram();
	ptr_RGBMatrix histogram = &temp;
	double totalPixels = matImage.getGrayMatrix().getRows()
		* matImage.getGrayMatrix().getCols();
	Matrix<RGB> temp2 = matImage.getRGBMatrix();
	ptr_RGBMatrix result = colorThreshold(&temp2, histogram);
	saveRGB(savename.c_str(), result);
}

void extractLandmarkPatch(string image_file, string landmark_file, int width,
	int height, string save_folder)
{
	Image matImage(image_file);
	matImage.readManualLandmarks(landmark_file);
	vector<Point> landmarks = matImage.getListOfManualLandmarks();
	string name = matImage.getName();
	size_t found2 = name.find_last_of(".");
	string sname = name.substr(0, found2);

	RGB color;
	color.R = color.G = color.B = 0;
	for (int i = 0; i < landmarks.size(); i++)
	{
		Point pi = landmarks.at(i);
		Matrix<int> patch = matImage.getGrayMatrix().extractPatch(width, height,
			pi.getY(), pi.getX(), 0);
		std::stringstream ssname;
		ssname << sname;
		ssname << "_p" << i << ".jpg";
		string savename = save_folder + "/" + ssname.str();
		saveGrayScale(savename.c_str(), &patch);
	}
}

void calSIFT(ptr_IntMatrix imageGray, Point point, int patchsize,
	string save_file)
{
	//ofstream inFile(savetps.c_str());

	ofstream inFile(save_file.c_str());
	//Image matImage(image_file);
	vector<double> SIFTVector = SIFTDescriptor(imageGray, point, patchsize);
	for (size_t i = 0; i < SIFTVector.size(); i++)
	{
		if (i % 8 == 0)
		{
			if (i != 0)
				inFile << "\n";
			inFile << SIFTVector.at(i);
			//cout << endl << SIFTVector.at(i);
		}
		else
		{
			//cout << "\t" << SIFTVector.at(i);
			inFile << "\t" << SIFTVector.at(i);
		}
	}
	//cout << "\n";
	inFile.close();
}
void calculateSIFT(string image_file, string lm_file, int patchsize,
	string save_folder)
{
	Image matImage(image_file);
	matImage.readManualLandmarks(lm_file);
	vector<Point> landmarks = matImage.getListOfManualLandmarks();
	string name = matImage.getName();
	size_t found2 = name.find_last_of(".");
	string sname = name.substr(0, found2);
	//vector<Matrix<int> > channels = matImage.splitChannels();
	for (int i = 0; i < landmarks.size(); ++i)
	{
		Point pi = landmarks.at(i);
		pi.toString();
		std::stringstream ssname;
		ssname << sname;
		ssname << "_p" << i << ".txt";
		string savename = save_folder + "/" + ssname.str();
		Matrix<int> temp = matImage.getGrayMatrix();
		calSIFT(&temp, pi, patchsize, savename);
	}
}

vector<Point> resize_Landmarks(string file_name, string lm_file, double xRatio,
	double yRatio, string save_file)
{
	Image image(file_name);
	image.readManualLandmarks(lm_file);
	vector<Point> landmarks = image.getListOfManualLandmarks();
	vector<Point> result;
	ofstream outfile(save_file.c_str());
	outfile << "LM=" << landmarks.size() << "\n";
	for (int i = 0; i < landmarks.size(); i++)
	{
		Point pi = landmarks.at(i);
		int x_new = pi.getX() / xRatio;
		int y_new = pi.getY() / yRatio;
		result.push_back(Point(x_new, y_new));
		outfile << x_new << " "
			<< (image.getGrayMatrix().getRows() / yRatio) - y_new << "\n";
	}
	outfile << "IMAGE=" << image.getName();
	outfile.close();
	return result;
}
enum AUGMENTATION
{
	INCREASE_RED = 1, INCREASE_GREEN = 2, INCREASE_BLUE = 3
};
void data_Augmentation(string filename, string lm_file, AUGMENTATION aug,
	int v_increase, string save_file)
{
	Image image(filename);
	image.readManualLandmarks(lm_file);
	Matrix<RGB> rgbImage = image.getRGBMatrix();
	RGB color;
	color.R = color.G = color.B = 255;
	int rows = rgbImage.getRows();
	int cols = rgbImage.getCols();
	Matrix<RGB> result(rows, cols, color);
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			color = rgbImage.getAtPosition(r, c);
			switch (aug)
			{
			case INCREASE_RED:
				if (color.R < 255 - v_increase)
					color.R += v_increase;
				else
					color.R = 255;
				break;
			case INCREASE_GREEN:
				if (color.G < 255 - v_increase)
					color.G += v_increase;
				else
					color.G = 255;
				break;
			case INCREASE_BLUE:
				if (color.B < 255 - v_increase)
					color.B += v_increase;
				else
					color.B = 255;
				break;
			default:
				break;
			}
			result.setAtPosition(r, c, color);
		}
	}
	saveRGB(save_file.c_str(), &result);
}

void get_Bounding_Box(vector<Point> mLandmarks, Point &lt, Point &rb)
{
	lt.setX(0);
	lt.setY(0);
	rb.setX(0);
	rb.setY(0);
	
	Point p0 = mLandmarks.at(0);
	int left = p0.getX(), right = p0.getX(), top = p0.getY(), bottom = p0.getY();
	for (int k = 1; k < mLandmarks.size(); ++k) {
		Point p = mLandmarks.at(k);
		if(p.getX() < left)
			left = p.getX();
		if(p.getX() > right)
			right = p.getX();
		if(p.getY() < top)
			top = p.getY();
		if(p.getY() > bottom)
			bottom = p.getY();
	}
	left -= 5;
	right += 5;
	top -= 5;
	bottom += 5;
	lt.setX(left);
	lt.setY(top);
	rb.setX(right);
	rb.setY(bottom);
}
/*
	Read the list of images, list of landmarks and store the coordinate of 
	the landmarks on txt file. This file is used as the input of CNN.
*/
void write_Image_Landmarks_txt(vector<string> images, vector<string> landmarks, 
	string savename)
{
	size_t found = savename.find_last_of(".");
	string ext = savename.substr(found+1,3);
	if(strcmp(ext.c_str(),"txt") != 0)
	{
		cout<<"\n The saving extension is not TXT format!!!\n";
		return;
	}
	ofstream outfile(savename.c_str());
		for (int i = 0; i < images.size(); ++i)
		{
			string imgfile = images.at(i);
			string lmfile = landmarks.at(i);
			Image image(imgfile);
			image.readManualLandmarks(lmfile);
			vector<Point> mLandmarks = image.getListOfManualLandmarks();
			outfile << image.getFileName();
			Point lt, rb;
			get_Bounding_Box(mLandmarks,lt,rb);
			outfile <<" "<<lt.getX()<<" "<<rb.getX()<<" "<<lt.getY()<<" "<<rb.getY();
			for (int k = 0; k < mLandmarks.size(); ++k) {
				Point p = mLandmarks.at(k);
				outfile<<" "<<p.getX()<<" "<<p.getY();
			}
			outfile << "\n";
		}
		outfile.close();
}

string image_Values(Matrix<int> inputMatrix)
{
	int rows = inputMatrix.getRows();
	int cols = inputMatrix.getCols();
	string result = "";
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			int pvalue = inputMatrix.getAtPosition(r,c);
			result += numberToString(pvalue).c_str();
			result += " ";
		}
	}
	return result.substr(0,result.length() - 1);
}

void write_Image_Landmarks_csv(vector<string> images, vector<string> landmarks, 
	string savename)
{
	size_t found = savename.find_last_of(".");
	string ext = savename.substr(found+1,3);
	if(strcmp(ext.c_str(),"csv") != 0)
	{
		cout<<"\n The saving extension is not CSV format!!!\n";
		return;
	}

	ofstream outfile(savename.c_str());
	string title ="lm1_x;lm1_y;lm2_x;lm2_y;lm3_x;lm3_y;lm4_x;lm4_y;lm5_x;lm5_y;lm6_x;lm6_y;lm7_x;lm7_y;lm8_x;lm8_y;image;\n";
	outfile<<title;
	for (int i = 0; i < images.size(); ++i)
	{
		string imgfile = images.at(i);
		string lmfile = landmarks.at(i);
		Image image(imgfile);
		image.readManualLandmarks(lmfile);
		vector<Point> mLandmarks = image.getListOfManualLandmarks();
		string iValue = image_Values( image.getGrayMatrix());
		//outfile << image.getFileName();
		
		//Point lt, rb;
		//get_Bounding_Box(mLandmarks,lt,rb);
		//outfile <<" "<<lt.getX()<<" "<<rb.getX()<<" "<<lt.getY()<<" "<<rb.getY();
		string lmCoors = "";
		for (int k = 0; k < mLandmarks.size(); ++k) {
			Point p = mLandmarks.at(k);
			lmCoors += numberToString(p.getX()).c_str();
			lmCoors += ";";
			lmCoors += numberToString(p.getY()).c_str();
			lmCoors += ";";
			//outfile<<" "<<p.getX()<<" "<<p.getY();
		}
		string swrite = lmCoors + iValue + ";\n";
		outfile << swrite;
		
	}
	outfile.close();
}

void read_Image_Landmarks(string image_folder, string lm_folder,
	string savename)
{
	//Image image(filename);
	//image.readManualLandmarks(lm_file);
	DIR *dir;
	struct dirent *ent;

	// read image directory
	vector<string> images;
	if ((dir = opendir(image_folder.c_str())) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
				images.push_back(image_folder + "/" + ent->d_name);
			//cout<<"\n"<<ent->d_name;
		}
	}
	std::sort(images.begin(), images.end());
	for (int i = 0; i < images.size(); ++i)
	{
		cout << "\n" << images.at(i);
	}
	closedir(dir);
	// read landmark directory
	vector<string> landmarks;
	if ((dir = opendir(lm_folder.c_str())) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
				landmarks.push_back(lm_folder + "/" + ent->d_name);
		}
	}
	std::sort(landmarks.begin(), landmarks.end());
	for (int i = 0; i < landmarks.size(); ++i)
	{
		cout << "\n" << landmarks.at(i);
	}
	closedir(dir);
	if (images.size() == landmarks.size())
	{
		//write_Image_Landmarks_txt(images,landmarks,savename);
		write_Image_Landmarks_csv(images,landmarks,savename);
	}
}




int main(int argc, char* argv[])
{
	cout << "\n Test a function !!!" << endl;

	// ================================================================ Test hole fill =================================================
	string filename, savename, lm_file;
	int width, height;
	string save_folder;
	if (argc == 1)
	{
		cout << "\nWithout parameters !!" << endl;
		filename = "data/md028.jpg";
		savename = "results/md028.jpg";
		lm_file = "data/landmarks/Md 028.TPS";
		width = 121;
		height = 121;
		save_folder = "results";
	}
	else
	{
		cout << "\nWith parameters !!" << endl;
		filename = argv[1];
		//savename = argv[2];
		lm_file = argv[2];
		width = atoi(argv[3]);
		height = atoi(argv[4]);
		save_folder = argv[5];
	}
	//holeFill(filename,savename);
	//removelegMain(filename, savename);
	//getProjections(filename,savename);
	//colorThreshold(filename, savename);
	//extractLandmarkPatch(filename, lm_file, width, height, save_folder);
	//calculateSIFT(filename,lm_file,9,save_folder);
	//resize_Landmarks(filename,lm_file,12.75,12.75,save_folder);
	//data_Augmentation(filename,lm_file,INCREASE_RED,10,save_folder);
	read_Image_Landmarks("/home/vanlinh/Downloads/pronotum_data_5/train_green",
		"/home/vanlinh/Downloads/pronotum_data_5/landmarks/train", 
		"/home/vanlinh/Downloads/pronotum_data_5/train_green.csv");
}
