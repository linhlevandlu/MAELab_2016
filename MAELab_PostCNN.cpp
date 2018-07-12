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

using namespace std;

#include "imageModel/Point.h"
#include "imageModel/Line.h"
#include "imageModel/Edge.h"
#include "imageModel/Matrix.h"
#include "imageModel/Image.h"

vector<string> Read_Directory(string folder_path)
{
	DIR *dir;
	struct dirent *ent;
	std::vector < string > files;
	if ((dir = opendir(folder_path.c_str())) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
				files.push_back(folder_path + "/" + ent->d_name);
		}
	}
	closedir(dir);
	std::sort(files.begin(), files.end());
	return files;
}

/**
 * nLandmarks: number of landmarks i.e. 8 for pronotum, 10 for tete, 11 for elytre
 * nImages: number of images that we would like to compute the general model
 * imageFolder, mlmFolder: where we store the images and manual landmarks (TPS file)
 */
vector<Point> General_Landmarks(int nLandmarks, int nImages,
		std::string imageFolder, string mlmFolder)
{
	vector<Point> result;
	vector < string > list_images = Read_Directory(imageFolder);
	vector < string > list_tpsfiles = Read_Directory(mlmFolder);
	if (list_tpsfiles.size() == list_images.size())
	{
		if (nImages > list_images.size())
			nImages = list_images.size();
		string imgfile, lmfile;
		int xArray[nLandmarks] =
		{ 0 };
		int yArray[nLandmarks] =
		{ 0 };
		for (int i = 0; i < nImages; i++)
		{
			imgfile = list_images.at(i);
			lmfile = list_tpsfiles.at(i);
			Image image(imgfile);
			vector<Point> mLandmarks = image.readManualLandmarks(lmfile);
			for (int j = 0; j < mLandmarks.size(); j++)
			{
				Point pi = mLandmarks.at(j);
				xArray[j] += pi.getX();
				yArray[j] += pi.getY();
			}
		}
		for (int k = 0; k < nLandmarks; k++)
		{
			int x = xArray[k];
			int y = yArray[k];
			result.push_back(Point(x / nImages, y / nImages));
		}
	}
	return result;
}

/*
 * Print the coordinates of landmarks in a list
 */
void Print_List_Of_Landmarks(vector<Point> list)
{
	for (size_t i = 0; i < list.size(); i++)
	{
		Point pi = list.at(i);
		cout << "Point " << (i + 1) << ": " << pi.getX() << " - " << pi.getY()
				<< endl;
	}
}

/*
 * Compute the constrast of a patch of 3x3 around a point pcenter
 */
int LBP_C_General(Matrix<int> grayImage, Point pcenter, double &ct)
{
	//Matrix<int> grayImage = image.getGrayMatrix();
	int cValue = grayImage.getAtPosition(pcenter.getY(), pcenter.getX());
	int i = 0, cg = 0, cl = 0;
	double lbp = 0, vg = 0, vl = 0;
	for (int y = pcenter.getY() - 1; y <= pcenter.getY() + 1; y++)
	{
		for (int x = pcenter.getX() - 1; x <= pcenter.getX() + 1; x++)
		{
			if (y != pcenter.getY() || x != pcenter.getX())
			{
				int tValue;
				if (x < 0 || y < 0 || x >= grayImage.getCols()
						|| y >= grayImage.getRows())
				{
					tValue = 0;
				}
				else
				{
					tValue = grayImage.getAtPosition(y, x);
				}
				double ivalue = pow(2, i);
				if (tValue >= cValue)
				{
					lbp += ivalue;
					vg += tValue;
					cg++;
				}
				else
				{
					vl += tValue;
					cl++;
				}
				i++;
			}
		}
	}
	double highContrast = (cg != 0) ? vg / cg : 0;
	double lowContrast = (cl != 0) ? vl / cl : 0;
	double contrast = highContrast - lowContrast;
	//if (contrast < 0)
	//	cout << endl << lbp << "\t" << contrast;
	ct = contrast;
	return lbp;
}

int LBP_C_General_ImagePath(string imgPath, Point pcenter, double &ct)
{
	Image image(imgPath);
	return LBP_C_General(image.getGrayMatrix(), pcenter, ct);
}

Matrix<int> Create_Distribution(Image image, Point pcenter, int radius = 10,
		int nBins = 8)
{
	Matrix<int> distribution(256, nBins, 0);
	int px = pcenter.getX();
	int py = pcenter.getY();
	//Image image(imgPath);
//Matrix<int> grayImage = image.getGrayMatrix();
	vector<int> lbpArray;
	vector<double> contrastArray;
	for (int r = py - radius; r < py + radius; r++)
	{
		for (int c = px - radius; c < px + radius; c++)
		{
			Point pi(r, c);
			int lbp = 0;
			double ct = 0;
			lbp = LBP_C_General(image.getGrayMatrix(), pi, ct);
			lbpArray.push_back(lbp);
			contrastArray.push_back(ct);
		}
	}
	if (lbpArray.size() == contrastArray.size())
	{
		double sumContrast = std::accumulate(contrastArray.begin(),
				contrastArray.end(), 0.0);
		double sizeOfBin = sumContrast / nBins;
		for (int i = 0; i < lbpArray.size(); i++)
		{
			int lbpV = lbpArray.at(i);
			double cV = contrastArray.at(i);
			int cIndex = -1;
			if (cV >= 0)
			{
				if (std::fmod(cV, sizeOfBin) == 0.0)
				{
					cIndex = (int) (cV / sizeOfBin) - 1;
				}
				else
				{
					cIndex = (int) (cV / sizeOfBin);
				}
				int oldValue = distribution.getAtPosition(lbpV, cIndex);
				distribution.setAtPosition(lbpV, cIndex, oldValue + 1);
			}
		}
	}
	return distribution;
}

double BhattacharyyaMetric(Matrix<int> model, Matrix<int> sample)
{
	double distance = 0.0;
	int rows = model.getRows();
	int cols = model.getCols();
	for (int c = 0; c < cols; c++)
	{
		for (int r = 0; r < rows; r++)
		{
			int mi = model.getAtPosition(r, c);
			int si = sample.getAtPosition(r, c);
			distance += sqrt(mi * si);
		}
	}
	return distance;
}
double LBP_C_ChiSquare_Metric(Matrix<int> model, Matrix<int> sample)
{
	int rows = model.getRows();
	int cols = model.getCols();
	double distance = 0;
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			int mValue = model.getAtPosition(r, c);
			int sValue = sample.getAtPosition(r, c);
			if (mValue == 0)
				distance += 0;
			else
				distance += pow(mValue - sValue, 2)
						/ (double) (mValue + sValue);
		}
	}
	return distance / 2.0;
}
/*
 * Searching based on a pair of (LBP, Contrast)
 */
Point Searching_Landmark(string imgPath, Point plandmark, int radius, int lbp,
		int contrast)
{
	int beginX = plandmark.getX() - radius;
	int endX = plandmark.getX() + radius;
	int beginY = plandmark.getY() - radius;
	int endY = plandmark.getY() + radius;
	int temp_lbp = 0;
	double ctrast = 0.0;
	double distance = 0, maxDistance = 3264;
	Point result(0, 0);
	Image image(imgPath);
	for (int r = beginY; r < endY; r++)
	{
		for (int c = beginX; c < endX; c++)
		{
			Point pi(c, r);
			temp_lbp = LBP_C_General(image.getGrayMatrix(), pi, ctrast);
			distance = sqrt(pow(temp_lbp - lbp, 2) + pow(contrast - ctrast, 2));
			if (distance <= maxDistance)
			{
				maxDistance = distance;
				result.setX(c);
				result.setY(r);
			}
		}
	}
	return result;
}
Point Searching_Landmark_Distribution(string imgPath, Point plandmark,
		int radiusPatch, int radius, int nBins, Matrix<int> modelDistr)
{
	int beginX = plandmark.getX() - radiusPatch;
	int endX = plandmark.getX() + radiusPatch;
	int beginY = plandmark.getY() - radiusPatch;
	int endY = plandmark.getY() + radiusPatch;
	Matrix<int> tempDistr;

	double distance = 0, maxDistance = 3264;
	Point result(0, 0);
	Image image(imgPath);
	for (int r = beginY; r < endY; r++)
	{
		for (int c = beginX; c < endX; c++)
		{
			Point pi(c, r);
			tempDistr = Create_Distribution(image, pi, radius, nBins);
			distance = BhattacharyyaMetric(modelDistr, tempDistr);
			if (distance <= maxDistance)
			{
				maxDistance = distance;
				result.setX(c);
				result.setY(r);
			}
		}
	}
	cout << endl << "Max distance: " << maxDistance;
	return result;
}

/*
 * Split and merge patch 25x25
 */
vector<Matrix<int> > Split_Patches(Image image, int wSize, int hSize)
{
	Matrix<int> rgbImage = image.getGrayMatrix();
	int rows = rgbImage.getRows();
	int cols = rgbImage.getCols();
	int nCPatch = cols / wSize;
	int nRPatch = rows / hSize;
	cout << endl << nRPatch << "\t" << nCPatch << endl;
	int rbegin = 0, cbegin = 0;
	int rend = 0, cend = 0;
	vector<Matrix<int> > patches;
	for (int rPatch = 0; rPatch < nRPatch; rPatch++)
	{
		rbegin = (rPatch * hSize);
		rend = rbegin + hSize;
		for (int cPatch = 0; cPatch < nCPatch; cPatch++)
		{
			cbegin = (cPatch * wSize);
			cend = cbegin + wSize;
			Matrix<int> patch(hSize, wSize);
			int i = 0, j = 0;
			for (int r = rbegin; r < rend; r++)
			{
				j = 0;
				for (int c = cbegin; c < cend; c++)
				{
					int color = rgbImage.getAtPosition(r, c);
					patch.setAtPosition(i, j, color);
					j++;
				}
				i++;
			}
			patches.push_back(patch);
		}
	}
	return patches;
	//cout << "\nnumber of patches: " << patches.size()<<endl;
}

Matrix<int> Compute_Features(Matrix<int> patch, int lbps = 256, int nBins = 8)
{
	Matrix<int> feature(lbps, nBins, 0);
	int patchSize = patch.getRows() * patch.getCols();
	int lbpArray[patchSize] =
	{ 0 };
	double contrastArray[patchSize] =
	{ 0.0 };
	double contrast_min = DBL_MAX, contrast_max = DBL_MAX;
	int k = 0;
	//cout<<endl<<patch.getRows()<<"\t"<<patch.getCols();
	for (int r = 0; r < patch.getRows(); r++)
	{
		for (int c = 0; c < patch.getCols(); c++)
		{
			double contrast = 0.0;
			int lbp = LBP_C_General(patch, Point(c, r), contrast);
			lbpArray[k] = lbp;
			contrastArray[k] = contrast;
			if (contrast > contrast_max)
			{
				contrast_max = contrast;
			}
			if (contrast < contrast_min)
			{
				contrast_min = contrast;
			}
		}
	}
	double quan_step = (contrast_max - contrast_min) / nBins;
	for (int m = 0; m < patchSize; m++)
	{
		int lbp = lbpArray[m];
		double contrs = contrastArray[m];
		int contrsIndex = (contrs - contrast_min) / quan_step;
		int oldValue = feature.getAtPosition(lbp, contrsIndex);
		feature.setAtPosition(lbp, contrsIndex, oldValue + 1);
	}
	return feature;
}

/*
 * direction:
 * 	-> 1
 * 	^  2
 * 	<- 3
 * 	v  4
 */
Matrix<int> Merge_Two_Patches(Matrix<int> patch1, Matrix<int> feature1,
		Matrix<int> patch2, Matrix<int> feature2, int direction,
		Matrix<int> &mergeMatrix)
{
	int rowsP1 = patch1.getRows();
	int colsP1 = patch1.getCols();
	int rowsP2 = patch2.getRows();
	int colsP2 = patch2.getCols();
	int rows = 0, cols = 0;
	//Matrix<int> mergeMatrix;
	if (direction == 1 || direction == 3)
	{
		rows = rowsP1;
		cols = colsP1 + colsP2;
		mergeMatrix.setRows(rows);
		mergeMatrix.setCols(cols);
		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < cols; c++)
			{
				int value;
				if (c >= patch1.getCols())
				{
					int c1 = c - patch1.getCols();
					if (direction == 1)
					{
						value = patch2.getAtPosition(r, c1);
					}
					else // direction == 3
					{
						value = patch1.getAtPosition(r, c1);
					}
				}
				else
				{
					if (direction == 1)
					{
						value = patch1.getAtPosition(r, c);
					}
					else // direction == 3
					{
						value = patch2.getAtPosition(r, c);
					}
				}
				mergeMatrix.setAtPosition(r, c, value);
			}
		}
	}
	if (direction == 2 || direction == 4)
	{
		rows = rowsP1 + rowsP2;
		cols = colsP1;
		mergeMatrix.setRows(rows);
		mergeMatrix.setCols(cols);
		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < cols; c++)
			{
				int value;
				if (r >= patch1.getRows())
				{
					int r1 = r - patch1.getRows();
					if (direction == 2)
					{
						value = patch1.getAtPosition(r1, c);
					}
					else // direction == 4
					{
						value = patch2.getAtPosition(r1, c);
					}
				}
				else
				{
					if (direction == 2)
					{
						value = patch2.getAtPosition(r, c);
					}
					else // direction == 4
					{
						value = patch1.getAtPosition(r, c);
					}
				}
				mergeMatrix.setAtPosition(r, c, value);
			}
		}

	}

	return feature1.add(feature2, 0);
}

Matrix<int> Merge_Two_Features(Matrix<int> index_feature,
		Matrix<int> merge_feature)
{
	return index_feature.add(merge_feature, 0);
}

vector<int> Get_4_Adjacents(vector<bool> merged, int index, int size = 12)
{
	vector<int> adjacents;
	int adj = index + 1;
	if (adj >= 0 && adj < merged.size())
	{
		if (merged.at(adj) == false)
			adjacents.push_back(adj);
	}
	adj = index - size;
	if (adj >= 0 && adj < merged.size())
	{
		if (merged.at(adj) == false)
			adjacents.push_back(adj);
	}
	adj = index - 1;
	if (adj >= 0 && adj < merged.size())
	{
		if (merged.at(adj) == false)
			adjacents.push_back(adj);
	}
	adj = index + size;
	if (adj >= 0 && adj < merged.size())
	{
		if (merged.at(adj) == false)
			adjacents.push_back(adj);
	}
	return adjacents;
}
vector<int> Merge_Features(vector<Matrix<int> > patches,
		vector<Matrix<int> > features, vector<bool> &merged, int index,
		double &mi_max)
{
	vector<int> mergedList;
	mergedList.push_back(index);
	double mir = 0.0;
	vector<int> adjacents = Get_4_Adjacents(merged, index, 12);
	Matrix<int> center = patches.at(index);
	int centerPixels = center.getRows() * center.getCols();
	Matrix<int> centerFeature = features.at(index);
	Matrix<int> adjFeature, adj;
	double smallest_MI, mi = 0.0;
	int merged_index;
	int direction;
	do
	{
		//cout<<endl<<"Neighbors: "<<adjacents.size();
		merged_index = -1;
		smallest_MI = DBL_MAX;
		direction = 0;
		for (size_t i = 0; i < adjacents.size(); i++)
		{
			int pos = adjacents.at(i);
			//cout<<"\n"<<pos;
			if (pos >= 0 && pos < patches.size())
			{
				adjFeature = features.at(pos); // or compute the feature
				adj = patches.at(pos);
				int pi = 0;
				if (adj.getRows() * adj.getCols() < centerPixels)
				{
					pi = adj.getRows() * adj.getCols();
				}
				else
				{
					pi = centerPixels;
				}
				mi = (double) pi
						* LBP_C_ChiSquare_Metric(centerFeature, adjFeature);
				if (mi < smallest_MI)
				{
					smallest_MI = mi;
					merged_index = pos;
					direction = i;
				}
				if (mi > mi_max)
				{
					mi_max = mi;
				}
			}
		} // found the patch to merged

		mir = smallest_MI / mi_max;
		//cout<<endl<<mir<<"\tindex: "<<merged_index<<endl;

		if (mir < 2 && merged.at(merged_index) == false)
		{
			//cout<<endl<<mir<<"\tindex: "<<merged_index<<endl;
			adjFeature = features.at(merged_index); // get feature of best merged
			// compute the features of merged patches and merge the patches
			mergedList.push_back(merged_index);
			adj = patches.at(merged_index);
			Matrix<int> newFeature = Merge_Two_Features(centerFeature,
					adjFeature);
			centerFeature = newFeature;
			centerPixels += (adj.getRows() + adj.getCols());
			merged.at(merged_index) = true; // set the patches to merged

			vector<int> adj2 = Get_4_Adjacents(merged, merged_index, 12); // get adjacents of merged patches
			adjacents.insert(adjacents.end(), adj2.begin(), adj2.end());

		}
		adjacents.erase(adjacents.begin() + direction);


	} while (adjacents.size() > 0);
	return mergedList;
}

void Merge_Process(vector<Matrix<int> > patches, vector<Matrix<int> > features,
		vector<bool> merged)
{
	int k = 0;
	double mi_max = 0;
	vector<int> mergedList;
	while (k < patches.size())
	{
		if (merged[k] == false) // manh chua ghep
		{
			merged.at(k) = true;
			mergedList = Merge_Features(patches, features, merged, k, mi_max);
			for (int m = 0; m < mergedList.size(); ++m)
			{
				cout << "\t" << mergedList.at(m);
			}
			cout << endl<<endl;
		}
		k++;
	}
}

int main(int argc, char* argv[])
{
	string imagePath = "results/rgb/lm2/Prono_001.jpg";
	int wSize = 25, hSize = 25;
	int lbp_limit = 256, nBins = 8;
	Image inputImg(imagePath);
	vector<Matrix<int> > patches = Split_Patches(inputImg, wSize, hSize);

	cout << "\nNumber of patches: " << patches.size();
// compute the features of patches
	vector<Matrix<int> > features;
	for (size_t i = 0; i < patches.size(); i++)
	{
		Matrix<int> patchi = patches.at(i);
		Matrix<int> feature = Compute_Features(patchi, lbp_limit, nBins);
		features.push_back(feature);
		/*for (int r = 0; r < feature.getRows(); r++) {
		 for (int c = 0; c < feature.getCols(); c++) {
		 cout<<feature.getAtPosition(r,c)<<"\t";
		 }
		 cout<<endl;
		 }
		 cout<<endl;*/
	}
	cout << "\nNumber of features: " << patches.size();
	vector<bool> merged;
	for (int i = 0; i < features.size(); i++)
	{
		merged.push_back(false);
	}

	cout << "\nNumber of merged: " << patches.size();

	Merge_Process(patches, features, merged);

	// compute the metric between patches
	/*vector<double> metrics;
	 for (int k = 0; k < features.size() - 1; k++)
	 {
	 Matrix<int> mfeature = features.at(k);

	 for (int m = k + 1; m < features.size(); m++)
	 {
	 Matrix<int> sfeature = features.at(m);
	 double distance = LBP_C_ChiSquare_Metric(mfeature, sfeature);
	 cout << endl << distance;
	 metrics.push_back(distance);
	 }
	 }
	 cout << endl << metrics.size();*/

	return 0;
}

