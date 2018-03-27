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

#include "segmentation/Thresholds.h"
#include "segmentation/Canny.h"
#include "segmentation/Filters.h"
#include "segmentation/Projection.h"
#include "correlation/DescriptorDistance.h"

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
/**
 * This function is used to fill the hole inside the object (pronotum)
 * Input: file path of rgb image
 * Output: the output image under Binary format.
 */
ptr_IntMatrix refillHole(ptr_IntMatrix inputMatrix)
{
	int rows = inputMatrix->getRows();
	int cols = inputMatrix->getCols();
	ptr_IntMatrix pMatrix = new Matrix<int>(rows - 20, cols - 20, 255);
	// copy matrix
	for (int r = 10; r < rows - 10; r++)
	{
		for (int c = 10; c < cols - 10; c++)
		{
			int value = inputMatrix->getAtPosition(r, c);
			pMatrix->setAtPosition(r - 10, c - 10, value);
		}
	}

	rows = pMatrix->getRows();
	cols = pMatrix->getCols();
	Point left(0, 0), right(0, 0);
	vector<Line> lines;
	for (int r = 1; r < rows; r++)
	{
		for (int c = 1; c < cols; c++)
		{
			int value = pMatrix->getAtPosition(r, c);
			int value_l = pMatrix->getAtPosition(r, c - 1);
			if (value == 255 && value_l == 0)
			{
				// set the first point
				left.setX(c - 1);
				left.setY(r);
				if (c - 1 != 0)
				{
					for (int k = c; k < cols; k++)
					{
						if (pMatrix->getAtPosition(r, k) == 0
								&& pMatrix->getAtPosition(r, k - 1) == 255)
						{
							right.setX(k);
							right.setY(r);
							break;
						}
					}
					if (right.getX() != 0 && right.getX() > left.getX()
							&& right.getX() < cols - 1)
					{

						bool beginHole = true;
						if (r - 1 < 0 || r - 1 >= rows)
							beginHole = false;
						else
						{
							for (int l = left.getX() + 1; l <= right.getX() - 1;
									l++)
							{
								if (pMatrix->getAtPosition(r - 1, l) == 255)
								{
									beginHole = false;
									break;
								}
							}
							if (!beginHole)
							{
								left.setX(0);
								left.setY(0);
								right.setX(0);
								right.setY(0);
							}
							if (left != 0 && right != 0)
							{
								bool inhole = true;
								int rnew = r + 1;
								int clnew = 0, crnew = 0;
								lines.push_back(Line(left, right));
								do
								{
									clnew = 0;
									crnew = 0;
									if (rnew < rows)
									{
										int lValue = pMatrix->getAtPosition(
												rnew, left.getX());
										if (lValue == 0)
										{
											clnew = left.getX();
										}
										else // lValue == 255
										{
											for (int cl = left.getX(); cl > 0;
													cl--)
											{
												if (pMatrix->getAtPosition(rnew,
														cl) == 0)
												{
													clnew = cl;
													break;
												}
											}
										}
										int rValue = pMatrix->getAtPosition(
												rnew, right.getX());
										if (rValue == 0)
										{
											crnew = right.getX();
										}
										else // lValue == 255
										{
											for (int cr = right.getX();
													cr < cols; cr++)
											{
												if (pMatrix->getAtPosition(rnew,
														cr) == 0)
												{
													crnew = cr;
													break;
												}
											}
										}
										if (clnew != 0 && crnew != 0)
										{
											int count = 0;
											for (int cl = clnew; cl < crnew;
													cl++)
											{
												if (pMatrix->getAtPosition(rnew,
														cl) == 255)
												{
													count++;
												}
											}
											if (count == 0) // end of hole
											{
												inhole = false;
											}
											else
											{
												left.setX(clnew);
												left.setY(rnew);
												right.setX(crnew);
												right.setY(rnew);
												lines.push_back(
														Line(left, right));
											}

										}
										else
										{
											lines.clear();
											inhole = false;
										}
									}
									else
									{
										//lines.clear();
										inhole = false;
									}
									rnew++;
								} while (inhole);
								for (size_t li = 0; li < lines.size(); li++)
								{
									Line line = lines.at(li);
									if (line.getBegin().getY()
											== line.getEnd().getY())
									{
										for (int x = line.getBegin().getX();
												x < line.getEnd().getX(); x++)
										{
											pMatrix->setAtPosition(
													line.getBegin().getY(), x,
													0);
										}
									}
								}
								lines.clear();
							}

						} // end else
					}

				}
				else
				{
					left.setX(0);
					left.setY(0);
				}
			}
		}
	}

	rows = inputMatrix->getRows();
	cols = inputMatrix->getCols();
	ptr_IntMatrix result = new Matrix<int>(rows, cols, 255);
	for (int r = 0; r < pMatrix->getRows(); r++)
	{
		for (int c = 0; c < pMatrix->getCols(); c++)
		{
			int value = pMatrix->getAtPosition(r, c);
			result->setAtPosition(r + 10, c + 10, value);
		}
	}

	for (int r = 0; r <= 10; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			int value = pMatrix->getAtPosition(11, c);
			result->setAtPosition(r, c, value);
		}
	}
	for (int r = rows - 10; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			int value = pMatrix->getAtPosition(rows - 21, c);
			result->setAtPosition(r, c, value);
		}
	}
	delete pMatrix;
	return result;
}
ptr_IntMatrix holeFill(string filename, string savefolder)
{
	Image image(filename);
	Matrix<double> gauKernel = getGaussianKernel(5, 1);
	Matrix<int> result = gaussianBlur(image.getGrayMatrix(), gauKernel);
	ptr_IntMatrix binMatrix = binaryThreshold(&result,
			image.getThresholdValue(), 255);
	//binMatrix = postProcess(binMatrix, 255);
	binMatrix = dilate(dilate(dilate(binMatrix, 5), 7), 5);

	binMatrix = refillHole(binMatrix);

	string savefile = savefolder + "/" + image.getName();
	cout << "\n" << savefile;
	saveGrayScale(savefile.c_str(), binMatrix);
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
	Matrix<RGB> histogram = matImage.getRGBHistogram();
	double totalPixels = matImage.getGrayMatrix().getRows()
			* matImage.getGrayMatrix().getCols();
	Matrix<RGB> rgbMatrix = matImage.getRGBMatrix();
	ptr_RGBMatrix result = colorThreshold(&rgbMatrix, &histogram);
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
	Matrix<int> grayMatrix = matImage.getGrayMatrix();
	for (int i = 0; i < landmarks.size(); ++i)
	{
		Point pi = landmarks.at(i);
		pi.toString();
		std::stringstream ssname;
		ssname << sname;
		ssname << "_p" << i << ".txt";
		string savename = save_folder + "/" + ssname.str();
		calSIFT(&grayMatrix, pi, patchsize, savename);
	}
}

vector<Point> resize_Landmarks(string file_name, string lm_file, double xRatio,
		double yRatio, string save_file)
{
	Image image(file_name);
	image.readManualLandmarks(lm_file);
	vector<Point> landmarks = image.getListOfManualLandmarks();
	cout << "\nNumber of landmarks: " << landmarks.size();
	vector<Point> result;
	ofstream outfile(save_file.c_str());
	outfile << "LM=" << landmarks.size() << "\n";
	for (int i = 0; i < landmarks.size(); i++)
	{
		Point pi = landmarks.at(i);
		double x_new = (double) pi.getX() / xRatio;
		double y_new = (double) pi.getY() / yRatio;
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
	INCREASE_RED = 1, INCREASE_GREEN = 2, INCREASE_BLUE = 3, GRAY_SCALE = 4,
};
void data_Augmentation(string filename, AUGMENTATION aug, int v_increase,
		string save_file)
{
	cout << "\nData augmentation";
	Image image(filename);
	Matrix<RGB> rgbImage = image.getRGBMatrix();
	if (aug == GRAY_SCALE)
	{
		Matrix<int> gray = image.getGrayMatrix();
		saveGrayScale(save_file.c_str(), &gray);
	}
	else
	{
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
}

// calculating the bounding box from manual landmarks
Line manual_BBox(vector<Point> mLandmarks, int marginSize = 180)
{
	cout << "\nMargin: " << marginSize << "\n";
	int minX = DBL_MAX, maxX = DBL_MIN, minY = DBL_MAX, maxY = DBL_MIN;
	for (size_t i = 0; i < mLandmarks.size(); i++)
	{
		Point pi = mLandmarks.at(i);
		if (pi.getX() < minX)
			minX = pi.getX();
		if (pi.getX() > maxX)
			maxX = pi.getX();
		if (pi.getY() < minY)
			minY = pi.getY();
		if (pi.getY() > maxY)
			maxY = pi.getY();
	}
	int dx = abs(minX - maxX);
	int dy = abs(minY - maxY);
	int distance = 0, move = 0;
	/*if (dx > dy)
	 {
	 distance = dx - dy;
	 move = distance / 2;
	 minY -= move;
	 maxY += move;
	 }
	 else
	 {
	 distance = dy - dx;
	 move = distance / 2;
	 minX -= move;
	 maxX += move;
	 }*/

	move = (marginSize - dx) / 2;

	minX -= move;
	maxX += move;

	move = (marginSize - dy) / 2;
	minY -= move;
	if (minY < 0)
	{
		distance = 0 - minY;
		minY = 0;
	}
	if (distance != 0)
		move += distance;
	maxY += move;
	return Line(Point(minX, maxX), Point(minY, maxY));
}
// read image and landmark folders, then write the image and corresponding landmarks
// into file
void read_Image_Landmarks(string image_folder, string lm_folder,
		string savename)
{
	//Image image(filename);
	//image.readManualLandmarks(lm_file);
	DIR *dir;
	struct dirent *ent;

	// read image directory
	vector < string > images;
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
	vector < string > landmarks;
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
		ofstream outfile(savename.c_str());
		for (int i = 0; i < images.size(); ++i)
		{
			string imgfile = images.at(i);
			string lmfile = landmarks.at(i);
			Image image(imgfile);
			image.readManualLandmarks(lmfile);
			vector<Point> mLandmarks = image.getListOfManualLandmarks();
			// write file name
			outfile << image.getName();
			for (int k = 0; k < mLandmarks.size(); k++)
			{
				Point p = mLandmarks.at(k);
				outfile << "\t" << p.getX() << "\t" << p.getY();
			}
			outfile << "\n";
			// write the coordinate of landmarks
			/*for (int k = 0; k < mLandmarks.size(); ++k)
			 {
			 Point p = mLandmarks.at(k);
			 outfile << p.getX() << "\t" << p.getY() << "\t";
			 }
			 outfile << image.getFileName();
			 outfile << "\n";*/
		}
		outfile.close();
	}
}

void split_Save_Channels(string folderPath, string saveFolder, int cIndex)
{
	DIR *dir;
	struct dirent *ent;

	// read image directory
	vector < string > images;
	if ((dir = opendir(folderPath.c_str())) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
				images.push_back(folderPath + "/" + ent->d_name);
		}
	}
	std::sort(images.begin(), images.end());
	vector<Matrix<int> > channels;
	Matrix<int> channel;
	for (size_t i = 0; i < images.size(); ++i)
	{
		string imgfile = images.at(i);
		Image *image = new Image(imgfile);
		//gray = image->getGrayMatrix();
		string filename = image->getName();
		size_t loff = filename.find_last_of('.');
		filename = filename.substr(0, loff);
		channels = image->splitChannels();
		string cname = "";
		if (cIndex <= 2)
			channel = channels.at(cIndex);
		else
			channel = image->getGrayMatrix();
		switch (cIndex)
		{
		case 0:
			cname = "_red";
			break;
		case 1:
			cname = "_green";
			break;
		case 2:
			cname = "_blue";
			break;
		case 3:
			cname = "_gray";
			break;
		default:
			cname = "_red";
			break;
		}
		filename += cname + ".JPG";
		saveGrayScale((saveFolder + "/" + filename).c_str(), &channel);
		delete image;
	}
}

// ******************************************************************************************************* //
Point check_Contour_Point(Point origin, Point begin, Point end,
		ptr_IntMatrix matrix)
{
	int cols = matrix->getCols();
	int rows = matrix->getRows();
	Point mid(abs(begin.getX() + end.getX()) / 2,
			abs(begin.getY() + end.getY()) / 2);
	Line midLine(origin, mid);
	vector<Point> linePoints = detectLine(midLine);
	std::sort(linePoints.begin(), linePoints.end(), xComparationTest);
	Point index(-1, -1);
	if (mid.getX() > origin.getX())
	{
		for (size_t i = 0; i < linePoints.size(); i++)
		{
			Point pi = linePoints.at(i);
			if (matrix->getAtPosition(pi.getY(), pi.getX()) == 255)
			{
				index = pi;
				break;
			}
		}
	}
	else
	{
		for (size_t i = linePoints.size() - 1; i > 0; i--)
		{
			Point pi = linePoints.at(i);
			if (matrix->getAtPosition(pi.getY(), pi.getX()) == 255)
			{
				index = pi;
				break;
			}
		}
	}
	Line ox1(Point(0, 0), Point(cols - 1, 0));
	Line ox2(Point(0, rows - 1), Point(cols - 1, rows - 1));
	Line oy1(Point(0, 0), Point(0, rows - 1));
	Line oy2(Point(cols - 1, 0), Point(cols - 1, rows - 1));
	Point p1, p2;
	if (index.getX() == -1 || index.getY() == -1)
	{
		if (mid.getX() > origin.getX()) // right part
		{
			if (mid.getY() > origin.getY()) // below part
			{
				p1 = midLine.intersection(ox2);
				p2 = midLine.intersection(oy2);
			}
			else // above part
			{
				p1 = midLine.intersection(ox1);
				p2 = midLine.intersection(oy2);
			}
		}
		else
		{
			if (mid.getY() > origin.getY()) // below part
			{
				p1 = midLine.intersection(ox2);
				p2 = midLine.intersection(oy1);
			}
			else // above part
			{
				p1 = midLine.intersection(ox1);
				p2 = midLine.intersection(oy1);
			}
		}
		//cout << "\nP1: " << p1.getX() << "\t" << p1.getY() << endl;
		//cout << "\nP2: " << p2.getX() << "\t" << p2.getY() << endl;
		if (p1.getX() >= 0 && p1.getX() < cols && p1.getY() >= 0
				&& p1.getY() < rows)
			index = p1;
		else
			index = p2;
	}
	return index;
}

void bounding_Box(string imagePath, string savefolder)
{
	ptr_IntMatrix binImage = holeFill(imagePath, savefolder);

	int width = 600, height = 400;
	Image inputImage(imagePath);
	Matrix<RGB> rgb = inputImage.getRGBMatrix();
	int rows = rgb.getRows();
	int cols = rgb.getCols();
	string imgName = inputImage.getName();
	// extra
	Point origin(cols / 2, rows / 2);
	Point p0(origin.getX() + width / 2, origin.getY());
	Point p1(origin.getX() + width / 2, origin.getY() - height / 2);
	Point p2(origin.getX(), origin.getY() - height / 2);
	Point p3(origin.getX() - width / 2, origin.getY() - height / 2);
	Point p4(origin.getX() - width / 2, origin.getY());
	Point p5(origin.getX() - width / 2, origin.getY() + height / 2);
	Point p6(origin.getX(), origin.getY() + height / 2);
	Point p7(origin.getX() + width / 2, origin.getY() + height / 2);
	int index = -1;
	for (int c1 = origin.getX(); c1 < cols; c1++)
	{
		if (binImage->getAtPosition(origin.getY(), c1) == 255)
		{
			index = c1;
			break;
		}
	}
	if (index == -1)
		index = cols - 1;
	p0.setX(index);

	index = -1;
	for (int c1 = origin.getX(); c1 >= 0; c1--)
	{
		if (binImage->getAtPosition(origin.getY(), c1) == 255)
		{
			index = c1;
			break;
		}
	}
	if (index == -1)
		index = 0;
	p4.setX(index);

	index = -1;
	for (int r1 = origin.getX(); r1 >= 0; r1--)
	{
		if (binImage->getAtPosition(r1, origin.getX()) == 255)
		{
			index = r1;
			break;
		}
	}
	if (index == -1)
		index = 0;
	p2.setY(index);

	index = -1;
	for (int r1 = origin.getX(); r1 < rows; r1++)
	{
		if (binImage->getAtPosition(r1, origin.getX()) == 255)
		{
			index = r1;
			break;
		}
	}
	if (index == -1)
		index = rows - 1;
	p6.setY(index);

	p1.setX(p0.getX());
	p1.setY(p2.getY());
	p3.setX(p4.getX());
	p3.setY(p2.getY());
	p5.setX(p4.getX());
	p5.setY(p6.getY());
	p7.setX(p0.getX());
	p7.setY(p6.getY());

	vector<Point> line1 = detectLine(Line(origin, p1));
	std::sort(line1.begin(), line1.end(), xComparationTest);
	for (size_t i = 0; i < line1.size(); i++)
	{
		Point pi = line1.at(i);
		if (binImage->getAtPosition(pi.getY(), pi.getX()) == 255)
		{
			p1.setX(pi.getX());
			p1.setY(pi.getY());
			break;
		}
	}
	vector<Point> line3 = detectLine(Line(origin, p3));
	std::sort(line3.begin(), line3.end(), xComparationTest);
	for (size_t i = line3.size() - 1; i >= 0; i--)
	{
		Point pi = line3.at(i);
		if (binImage->getAtPosition(pi.getY(), pi.getX()) == 255)
		{
			p3.setX(pi.getX());
			p3.setY(pi.getY());
			break;
		}
	}
	/*if(p1.getY() < p3.getY())
	 p2.setY(p1.getY());
	 else
	 p2.setY(p3.getY());*/

	vector<Point> line5 = detectLine(Line(origin, p5));
	std::sort(line5.begin(), line5.end(), xComparationTest);
	for (size_t i = line5.size() - 1; i >= 0; i--)
	{
		Point pi = line5.at(i);
		if (binImage->getAtPosition(pi.getY(), pi.getX()) == 255)
		{
			p5.setX(pi.getX());
			p5.setY(pi.getY());
			break;
		}
	}
	vector<Point> line7 = detectLine(Line(origin, p7));
	std::sort(line7.begin(), line7.end(), xComparationTest);
	for (size_t i = 0; i < line1.size(); i++)
	{
		Point pi = line7.at(i);
		if (binImage->getAtPosition(pi.getY(), pi.getX()) == 255)
		{
			p7.setX(pi.getX());
			p7.setY(pi.getY());
			break;
		}
	}
	/*if(p5.getY() > p7.getY())
	 p6.setY(p5.getY());
	 else
	 p6.setY(p7.getY());*/
	RGB color;
	color.R = 255;
	color.G = color.B = 0;
	drawingLine(rgb, Line(Point(0, rows / 2), Point(cols - 1, rows / 2)),
			color);
	drawingLine(rgb, Line(Point(cols / 2, 0), Point(cols / 2, rows - 1)),
			color);
	color.G = 255;
	drawingLine(rgb, Line(p0, p1), color);
	drawingLine(rgb, Line(p1, p2), color);
	drawingLine(rgb, Line(p2, p3), color);
	drawingLine(rgb, Line(p3, p4), color);
	drawingLine(rgb, Line(p4, p5), color);
	drawingLine(rgb, Line(p5, p6), color);
	drawingLine(rgb, Line(p6, p7), color);
	drawingLine(rgb, Line(p7, p0), color);
	color.G = 0;
	drawingLine(rgb, Line(origin, p1), color);
	drawingLine(rgb, Line(origin, p3), color);
	drawingLine(rgb, Line(origin, p5), color);
	drawingLine(rgb, Line(origin, p7), color);

	color.R = 0;
	color.B = 255;

	Point mid01 = check_Contour_Point(origin, p0, p1, binImage);
	if (mid01.getX() != -1 && mid01.getY() != -1)
	{
		drawingLine(rgb, Line(p0, mid01), color);
		drawingLine(rgb, Line(mid01, p1), color);
	}

	Point mid12 = check_Contour_Point(origin, p1, p2, binImage);
	if (mid12.getX() != -1 && mid12.getY() != -1)
	{
		drawingLine(rgb, Line(p1, mid12), color);
		drawingLine(rgb, Line(mid12, p2), color);
	}
	Point mid23 = check_Contour_Point(origin, p2, p3, binImage);
	if (mid23.getX() != -1 && mid23.getY() != -1)
	{
		drawingLine(rgb, Line(p2, mid23), color);
		drawingLine(rgb, Line(mid23, p3), color);
	}
	Point mid34 = check_Contour_Point(origin, p3, p4, binImage);
	if (mid34.getX() != -1 && mid34.getY() != -1)
	{
		drawingLine(rgb, Line(p3, mid34), color);
		drawingLine(rgb, Line(mid34, p4), color);
	}
	Point mid45 = check_Contour_Point(origin, p4, p5, binImage);
	if (mid45.getX() != -1 && mid45.getY() != -1)
	{
		drawingLine(rgb, Line(p4, mid45), color);
		drawingLine(rgb, Line(mid45, p5), color);
	}
	Point mid56 = check_Contour_Point(origin, p5, p6, binImage);
	if (mid56.getX() != -1 && mid56.getY() != -1)
	{
		drawingLine(rgb, Line(p5, mid56), color);
		drawingLine(rgb, Line(mid56, p6), color);
	}
	Point mid67 = check_Contour_Point(origin, p6, p7, binImage);
	if (mid67.getX() != -1 && mid67.getY() != -1)
	{
		drawingLine(rgb, Line(p6, mid67), color);
		drawingLine(rgb, Line(mid67, p7), color);
	}
	Point mid70 = check_Contour_Point(origin, p7, p0, binImage);
	if (mid70.getX() != -1 && mid70.getY() != -1)
	{
		drawingLine(rgb, Line(p7, mid70), color);
		drawingLine(rgb, Line(mid70, p0), color);
	}
	string savefile = savefolder + "/" + imgName;
	saveRGB(savefile.c_str(), &rgb);
}
void move_Queue(std::queue<Line> &queue1, std::queue<Line> &queue2)
{
	while (!queue1.empty())
	{
		Line l = queue1.front();
		queue2.push(l);
		queue1.pop();
	}
}
Line corners_Detection(vector<Point> pQueue, Line axis)
{

	double dmax = -1;
	int imax = -1, imin1 = -1, imin2 = -1;
	std::sort(pQueue.begin(), pQueue.end(), yComparationTest);

	for (size_t i = 0; i < pQueue.size(); i++)
	{
		Point pi = pQueue.at(i);
		double d = axis.perpendicularDistance(pi);
		if (d > dmax)
		{
			dmax = d;
			imax = i;
		}
	}
	double dmin1 = dmax + 100, dmin2 = dmax + 100;
	for (size_t i = 0; i < imax; i++)
	{
		Point pi = pQueue.at(i);
		double d = axis.perpendicularDistance(pi);
		if (d < dmin1)
		{
			dmin1 = d;
			imin1 = i;
		}
	}
	for (size_t i = imax + 1; i < pQueue.size(); i++)
	{
		Point pi = pQueue.at(i);
		double d = axis.perpendicularDistance(pi);
		if (d < dmin2)
		{
			dmin2 = d;
			imin2 = i;
		}
	}
	Point p1 = pQueue.at(imax);
	Point p2 = pQueue.at(imin1);
	Point p3 = pQueue.at(imin2);
	int x1 = (p1.getX() + p2.getX()) / 2;
	int y1 = p2.getY();
	int x2 = (p1.getX() + p3.getX()) / 2;
	int y2 = p3.getY();
	return Line(Point(x1, y1), Point(x2, y2));

}
int checkPosition(vector<double> listDistance, int range)
{
	if (range % 2 == 0)
		range += 1;
	int mid = range / 2;

	vector<int> vIndex;
	for (int i = mid; i < listDistance.size() - mid; i++)
	{
		double ivalue = listDistance.at(i);

		bool lowest = true;
		double mv1 = DBL_MAX, mv2 = DBL_MAX;
		int im1, im2;
		for (int j = i - mid; j < i + mid; j++)
		{
			double jvalue = listDistance.at(j);
			if (ivalue > jvalue)
			{
				lowest = false;
				break;
			}
		}

		if (lowest)
		{
			vIndex.push_back(i);
		}
	}
	cout << "\nVindex size: " << vIndex.size();
	double minValue = listDistance.at(vIndex.at(0));
	int minIndex = 0;
	for (int k = 1; k < vIndex.size(); k++)
	{
		double kvalue = listDistance.at(vIndex.at(k));
		if (kvalue < minValue)
		{
			minValue = kvalue;
			minIndex = k;
		}
	}
	return vIndex.at(minIndex);
}
Line corners_Detection2(vector<Point> pQueue, Line axis, Point origin)
{

	double dmax = -1;
	int imax = -1, imin1 = -1, imin2 = -1;
	std::sort(pQueue.begin(), pQueue.end(), yComparationTest);
	//auto last = std::unique(pQueue.begin(),pQueue.end());
	pQueue.erase(std::unique(pQueue.begin(), pQueue.end()), pQueue.end());

	for (size_t i = 0; i < pQueue.size(); i++)
	{
		Point pi = pQueue.at(i);
		if (pi.getY() > origin.getY())
		{
			imax = i;
			break;
		}
	}

	vector<double> list1;
	for (size_t i = 0; i < imax; i++)
	{
		Point pi = pQueue.at(i);
		double d = abs(axis.getBegin().getX() - pi.getX());
		list1.push_back(d);
	}

	imin1 = checkPosition(list1, 5);

	vector<double> list2;
	for (size_t i = imax; i < pQueue.size(); i++)
	{
		Point pi = pQueue.at(i);
		double d = axis.perpendicularDistance(pi);
		list2.push_back(d);
	}
	imin2 = checkPosition(list2, 5);
	return Line(pQueue.at(imin1), pQueue.at(list1.size() + imin2));
}
/*void bbox_Detection(std::queue<Point> pQueue, Point origin, int rows)
 {
 Line oy(Point(origin.getX(), 0), Point(origin.getX(), rows - 1));
 vector<Point> pqueue1, pqueue2;
 while (!pQueue.empty())
 {
 Point pi = pQueue.front();
 if (pi.getX() < origin.getX())
 pqueue1.push_back(pi);
 else
 pqueue2.push_back(pi);
 pQueue.pop();
 }
 Line left = corners_Detection(pqueue1,oy);
 Line right = corners_Detection(pqueue2,oy);
 }*/

Point origin_Detect(ptr_IntMatrix binMatrix)
{
	Point origin(0, 0);
	int hsize = 0, vsize = 0;
	int* hProjection = histogramProjection(binMatrix, Horizontal_Projection,
			hsize);
	cout << "\n";
	ptr_IntMatrix hMatrix = new Matrix<int>(binMatrix->getRows(),
			binMatrix->getCols(), 255);
	int max = 0;
	for (int i = 0; i < hsize; ++i)
	{
		int rows = hProjection[i];
		for (int ro = binMatrix->getRows() - 1;
				ro > binMatrix->getRows() - rows - 1; ro--)
		{
			hMatrix->setAtPosition(ro, i, 0);
		}
		if (rows > max)
		{
			max = rows;
		}
		//cout << hProjection[i] << "\t";
	}
	saveGrayScale("results/hprojection.jpg", hMatrix);

	ptr_IntMatrix vMatrix = new Matrix<int>(binMatrix->getRows(),
			binMatrix->getCols(), 255);
	int* vProjection = histogramProjection(binMatrix, Vertical_Projection,
			vsize);
	cout << "\n";

	for (int i = 0; i < vsize; ++i)
	{
		int cols = vProjection[i];
		for (int c = 0; c < cols; c++)
		{
			vMatrix->setAtPosition(i, c, 0);
		}

		//cout << vProjection[i] << "\t";
	}
	int sum = 0, count = 0;
	for (int i = 0; i < hsize; ++i)
	{
		int cols = hProjection[i];
		if (cols == max)
		{
			sum += i;
			count++;
		}
	}
	origin.setY((binMatrix->getCols()) / 2);
	origin.setX(sum / count);
	saveGrayScale("results/vprojection.jpg", vMatrix);
	delete hMatrix;
	delete vMatrix;
	cout << "\nx = " << origin.getX() << ", y = " << origin.getY();
	return origin;
}

vector<Point> bounding_Box2(string imagePath, string savefolder,
		vector<Point> listPoints, int num_iter)
{
	ptr_IntMatrix binImage = holeFill(imagePath, savefolder);
	//origin_Detect(binImage);
	Image inputImage(imagePath);
	Matrix<RGB> rgb = inputImage.getRGBMatrix();
	int rows = rgb.getRows();
	int cols = rgb.getCols();
	string imgName = inputImage.getName();
	Point origin(0, 0);
	vector<Point> initPoints;
	if (listPoints.size() == 0)
	{
		//origin.setX((cols / 2) + 500);
		//origin.setY((rows / 2) + 500);
		origin = origin_Detect(binImage);
		initPoints.push_back(Point(cols - 1, rows / 2));
		//initPoints.push_back(Point(cols - 1, 0));
		initPoints.push_back(Point(cols / 2, 0));
		//initPoints.push_back(Point(0, 0));
		initPoints.push_back(Point(0, rows / 2));
		//initPoints.push_back(Point(0, rows - 1));
		initPoints.push_back(Point(cols / 2, rows - 1));
		//initPoints.push_back(Point(cols - 1, rows - 1));
	}
	else
	{
		origin = listPoints.at(listPoints.size() - 1);
		initPoints.assign(listPoints.begin(), listPoints.end());
		int index = initPoints.size() - 1;
		initPoints.erase(initPoints.begin() + index);
	}
	Point rightLM(0, origin.getY());
	Point leftLM(0, origin.getY());
	for (int i = origin.getX(); i < cols; i++)
	{
		if (binImage->getAtPosition(origin.getY(), i) == 255)
		{
			rightLM.setX(i);
			break;
		}
	}
	if (rightLM.getX() == 0)
		rightLM.setX(cols - 1);
	for (int i = origin.getX(); i > 0; i--)
	{
		if (binImage->getAtPosition(origin.getY(), i) == 255)
		{
			leftLM.setX(i);
			break;
		}
	}

	// init the queue of lines
	std::queue<Line> initQueue;
	for (size_t i = 0; i < initPoints.size() - 1; i++)
	{
		Line l(initPoints.at(i), initPoints.at(i + 1));
		initQueue.push(l);
	}
	initQueue.push(
			Line(initPoints.at(initPoints.size() - 1), initPoints.at(0)));
	std::queue<Line> queue2;
	int i = 0;
	while (i < num_iter)
	{
		while (!initQueue.empty())
		{
			Line l = initQueue.front();
			Point p = check_Contour_Point(origin, l.getBegin(), l.getEnd(),
					binImage);
			Line l1(l.getBegin(), p);
			Line l2(p, l.getEnd());
			queue2.push(l1);
			queue2.push(l2);
			initQueue.pop();
		}
		move_Queue(queue2, initQueue);
		//cout << "\nSize of queue: " << initQueue.size() << "\t" << queue2.size()
		//	<< "\n";
		i++;
	}

	// drawing the result
	RGB color;
	color.R = 255;
	color.G = color.B = 0;
	int count = 0;
	fillCircle(rgb, origin, 7, color);
	std::queue<Point> pQueue;
	while (!initQueue.empty())
	{
		Line l1 = initQueue.front();
		initQueue.pop();
		Point bPoint = l1.getBegin();
		Point ePoint = l1.getEnd();
		if (bPoint.getX() != 0 && bPoint.getX() != cols - 1
				&& bPoint.getY() != 0 && bPoint.getY() != rows - 1)
			pQueue.push(bPoint);

		if (ePoint.getX() != 0 && ePoint.getX() != cols - 1
				&& ePoint.getY() != 0 && ePoint.getY() != rows - 1)
			pQueue.push(ePoint);
	}

	///////////////////////////////////////////////////////////////////////////////
	//bbox detection
	Line oy(Point(origin.getX(), 0), Point(origin.getX(), rows - 1));
	vector<Point> pqueue1, pqueue2;
	while (!pQueue.empty())
	{
		Point pi = pQueue.front();
		if (pi.getX() < origin.getX())
			pqueue1.push_back(pi);
		else
			pqueue2.push_back(pi);
		pQueue.pop();
	}

	// drawing the contours points
	std::sort(pqueue1.begin(), pqueue1.end(), yComparationTest);
	color.R = 0;
	color.B = 255;
	for (int i = 0; i < pqueue1.size() - 1; i++)
	{
		Point p1 = pqueue1.at(i);
		Point p2 = pqueue1.at(i + 1);
		drawingLine(rgb, Line(p1, p2), color);
	}
	color.R = 255;
	color.B = 0;
	std::sort(pqueue2.begin(), pqueue2.end(), yComparationTest);
	for (int i = 0; i < pqueue2.size() - 1; i++)
	{
		Point p1 = pqueue2.at(i);
		Point p2 = pqueue2.at(i + 1);
		drawingLine(rgb, Line(p1, p2), color);
	}

	//Line left = corners_Detection(pqueue1, oy);
	//Line right = corners_Detection(pqueue2, oy);
	Line left = corners_Detection2(pqueue1, oy, origin);
	Line right = corners_Detection2(pqueue2, oy, origin);
	fillCircle(rgb, left.getBegin(), 5, color);
	fillCircle(rgb, left.getEnd(), 5, color);
	fillCircle(rgb, right.getBegin(), 5, color);
	fillCircle(rgb, right.getEnd(), 5, color);

	vector<Point> result;
	int vmargin = 70, hmargin = 70;
	int leftx = leftLM.getX() - hmargin;
	if (leftx < 0)
		leftx = 0;
	int rightx = rightLM.getX() + hmargin;
	if (rightx >= cols)
		rightx = cols - 1;
	int topy = (
			(left.getBegin().getY() > right.getBegin().getY()) ?
					left.getBegin().getY() : right.getBegin().getY()) - vmargin;
	if (topy < 0)
		topy = 0;

	int bottomy = 0;

	if (left.getEnd().getY() > right.getEnd().getY())
	{
		bottomy = left.getEnd().getY();
	}
	else
	{
		bottomy = right.getEnd().getY();
	}
	bottomy += vmargin;
	if (bottomy >= rows)
		bottomy = rows - 1;

	drawingLine(rgb, Line(Point(leftx, topy), Point(leftx, bottomy)), color);
	drawingLine(rgb, Line(Point(leftx, bottomy), Point(rightx, bottomy)),
			color);
	drawingLine(rgb, Line(Point(rightx, bottomy), Point(rightx, topy)), color);
	drawingLine(rgb, Line(Point(rightx, topy), Point(leftx, topy)), color);

	result.push_back(Point(leftx, topy));
	result.push_back(Point(leftx, bottomy));
	result.push_back(Point(rightx, bottomy));
	result.push_back(Point(rightx, topy));

	int xMid = (leftx + rightx) / 2;
	int yMid = (bottomy + topy) / 2;

	color.G = 255;
	fillCircle(rgb, Point(xMid, yMid), 7, color);
	result.push_back(Point(xMid, yMid));

//////////////////////////////////////////////////////////////////////////////
	/*int size = pQueue.size();
	 int totalX = 0, totalY =0;
	 Point p1 = pQueue.front();
	 pQueue.pop();
	 totalX += p1.getX();
	 totalY += p1.getY();
	 while (!pQueue.empty())
	 {
	 Point p2 = pQueue.front();
	 pQueue.pop();

	 totalX += p2.getX();
	 totalY += p2.getY();
	 Line l(p1, p2);
	 drawingLine(rgb, l, color);
	 p1 = p2;
	 }
	 fillCircle(rgb,Point(totalX/size, totalY/size),5,color);*/

	drawingLine(rgb, Line(Point(0, rows / 2), Point(cols - 1, rows / 2)),
			color);
	drawingLine(rgb, Line(Point(cols / 2, 0), Point(cols / 2, rows - 1)),
			color);

	drawingLine(rgb,
			Line(Point(0, (rows / 2) - 300), Point(cols - 1, (rows / 2) - 300)),
			color);
	drawingLine(rgb,
			Line(Point(0, (rows / 2) + 300), Point(cols - 1, (rows / 2) + 300)),
			color);
	cout << "\nCount: " << count;
	string savefile = savefolder + "/" + imgName;
	saveRGB(savefile.c_str(), &rgb);

	return result;
}

void load_Landmarks_Save(string fimage, string flandmarks, string fsave)
{
	Image image(fimage);
	image.readManualLandmarks(flandmarks);
	Matrix<RGB> rgbImage = image.getRGBMatrix();
	int width = rgbImage.getCols();
	int height = rgbImage.getRows();

	RGB color;
	color.R = 255;
	color.G = 0;
	color.B = 0;
	vector<Point> landmarks = image.getListOfManualLandmarks();
	// drawing the landmarks on RGB
	int sumX = 0, sumY = 0;
	for (int i = 0; i < landmarks.size(); i++)
	{
		Point pi = landmarks.at(i);
		sumX += pi.getX();
		sumY += pi.getY();
		//rgbImage = fillCircle(rgbImage, pi, 2, color);
	}

	// centroid coordinates
	int cX = sumX / landmarks.size();
	int cY = sumY / landmarks.size();
	int fix_number = 112;
	int x1_limit = cX - fix_number;
	int x2_limit = cX + fix_number;
	int x_aug = 0;
	if (x1_limit < 0)
	{
		x_aug = -x1_limit;
		if (x2_limit >= width)
		{
			return;
		}
		else
		{
			x2_limit += x_aug;
			x1_limit = 0;
			if (x2_limit >= width)
			{
				return;
			}
		}
	}
	else //x1_limit >=0
	{
		if (x2_limit >= width)
		{
			x_aug = x2_limit - width;
			x1_limit -= x_aug;
			x2_limit = width;
			if (x1_limit < 0)
				return;

		}
	}
	cout << "\nx1 -x2: " << x1_limit << "\t" << x2_limit;

	int y1_limit = cY - fix_number;
	int y2_limit = cY + fix_number;
	int y_aug = 0;
	if (y1_limit < 0)
	{
		y_aug = -y1_limit;
		if (y2_limit >= height)
		{
			return;
		}
		else
		{
			y2_limit += y_aug;
			y1_limit = 0;
			if (y2_limit >= height)
			{
				return;
			}
		}
	}
	else //y1_limit >=0
	{
		if (y2_limit >= height)
		{
			y_aug = y2_limit - height;
			y1_limit -= y_aug;
			y2_limit = height;
			if (y1_limit < 0)
				return;

		}
	}
	cout << "\ny1 -y2: " << y1_limit << "\t" << y2_limit;
	color.G = color.B = 255;
	Matrix<RGB> saveImg(224, 224, color);
	int i = -1, j = 0;
	for (int row = y1_limit; row < y2_limit; row++)
	{
		i++;
		j = 0;
		for (int col = x1_limit; col < x2_limit; col++)
		{
			RGB value = rgbImage.getAtPosition(row, col);
			saveImg.setAtPosition(i, j, value);
			j++;
		}
	}
	ofstream outfile(fsave.c_str());
	outfile << "LM=" << landmarks.size() << "\n";
	for (int i = 0; i < landmarks.size(); ++i)
	{
		Point pi = landmarks.at(i);
		int newX = pi.getX() - x1_limit;
		int newY = pi.getY() - y1_limit;
		//saveImg = fillCircle(saveImg, Point(newX, newY), 2, color);
		outfile << newX << " " << 224 - newY << "\n";
	}
	//saveRGB(fsave.c_str(), &saveImg);

	/*Display landmarks and crop the image*/

	outfile << "IMAGE=" << image.getName();
	outfile.close();
}
void crop_Image(string filename, int n_width, int n_height, string savepath)
{
	Image image(filename);
	Matrix<RGB> rgbImage = image.getRGBMatrix();
	int rows = rgbImage.getRows();
	int cols = rgbImage.getCols();
	cout<<"\n rows - cols: "<<rows <<"\t" << cols;
	RGB color;
	color.R=color.G=color.B = 0;
	int i=0, j=0;
	Matrix<RGB> newImage(n_height,n_width,color);
	for (int r = rows - n_height; r < rows; r++) {
		j = 0;
		for (int c = cols - n_width; c < cols; c++) {
			color = rgbImage.getAtPosition(r,c);
			newImage.setAtPosition(i,j,color);
			j++;
		}
		i++;
	}
	saveRGB(savepath.c_str(), &newImage);
}

vector<Point> crop_Landmarks(string file_name, string lm_file, int cropX,
		double cropY, string save_file)
{
	Image image(file_name);
	image.readManualLandmarks(lm_file);
	vector<Point> landmarks = image.getListOfManualLandmarks();
	cout << "\nNumber of landmarks: " << landmarks.size();
	vector<Point> result;
	ofstream outfile(save_file.c_str());
	outfile << "LM=" << landmarks.size() << "\n";
	for (int i = 0; i < landmarks.size(); i++)
	{
		Point pi = landmarks.at(i);
		int x_new =  pi.getX() - cropX;
		int y_new = pi.getY() - cropY;
		result.push_back(Point(x_new, y_new));
		outfile << x_new << " "
				<< 2448 - y_new << "\n";
	}
	outfile << "IMAGE=" << image.getName();
	outfile.close();
	return result;
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
		filename = "/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/original/Prono_001.JPG";
		//filename="/media/vanlinh/Data/Biogical_Images/pronotum/Images_without_grid_2/Prono_001.JPG";
		savename = "results/Prono_001_resize.jpg";
		lm_file = "/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/landmarks/p_001.TPS";
		//lm_file = "/media/vanlinh/Data/Biogical_Images/pronotum/landmarks/p 001.TPS";
		width = 121;
		height = 121;
		save_folder = "results/test.tps";
	}
	else
	{
		cout << "\nWith parameters !!" << endl;
		filename = argv[1];
//savename = argv[2];
		lm_file = argv[2];
//save_folder = argv[2];
//width = atoi(argv[3]);
//height = atoi(argv[4]);
save_folder = argv[2];
	}
	//holeFill(filename, save_folder);
//removelegMain(filename, savename);
//getProjections(filename,savename);
//colorThreshold(filename, savename);
//extractLandmarkPatch(filename, lm_file, width, height, save_folder);
//calculateSIFT(filename,lm_file,9,save_folder);
	//resize_Landmarks(filename, lm_file, 25.5, 25.5, save_folder);
	//data_Augmentation(filename, INCREASE_GREEN, 10, save_folder);
	/*
	 * read two folders (image and landamrk) to export data for CNN
	 */
	read_Image_Landmarks("/home/vanlinh/data_CNN/i96x96/original",
		"/home/vanlinh/data_CNN/i96x96/landmarks",
		"results/cnn_data_i96x96.txt");
	//split_Save_Channels(
	//	"/home/vanlinh/data_CNN/i96x96/original",
	//	"/home/vanlinh/data_CNN/i96x96/split_blue", 2);


	//crop_Image(filename,2448,2448,lm_file);
	//crop_Landmarks(filename, lm_file, 3264-2448,0, save_folder);

	//vector<Point> list;
	//list = bounding_Box2(filename, save_folder, list, 10); // lm_file parameter is the save folder path
	//list = bounding_Box2(filename, save_folder, list, 10);
	/*Test load manual landmarks and save into file*/
	//load_Landmarks_Save(filename, lm_file, save_folder);
}
