/*
 * Image.h
 *
 *  Created on: Sep 16, 2016
 *      Author: linh
 */

#ifndef IMAGE_H_
#define IMAGE_H_
#include "Edge.h"
class Image
{
private:
	std::string fileName;
	vector<ptr_Line> listOfLines;
	vector<ptr_Point> manualLandmarks;
	ptr_IntMatrix grayMatrix;
	ptr_RGBMatrix imgMatrix;
	ptr_IntMatrix grayHistogram;
	float medianHistogram;
	float meanHistogram;
	float thresholdValue;

	void calcGrayHistogram();
	void calThresholdValue();

public:
	Image();
	virtual ~Image();
	Image(std::string);
	void setFileName(std::string);
	std::string getFileName();
	void setMLandmarks(string);
	ptr_IntMatrix getGrayMatrix();
	ptr_RGBMatrix getRGBMatrix();
	float getMedianHistogram();
	float getMeanHistogram();
	float getThresholdValue();
	vector<ptr_Line> getListOfLines();
	vector<ptr_Point> getListOfManualLandmarks();

	vector<ptr_Edge> cannyAlgorithm();
	vector<ptr_Line> getApproximateLines(int minDistance);
	vector<ptr_Point> readManualLandmarks(string fileName);

	ptr_DoubleMatrix getRotationMatrix2D(ptr_Point center, double angle,
		double scale);
	ptr_IntMatrix rotate(ptr_Point center, double angle, double scale);
};

#endif /* IMAGE_H_ */
