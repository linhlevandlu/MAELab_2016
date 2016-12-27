/*
 * LandmarkDetection.cpp
 *
 *  Created on: Dec 7, 2016
 *      Author: linh
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#include "../imageModel/Point.h"
#include "../imageModel/Line.h"
#include "../imageModel/Edge.h"
#include "../imageModel/Matrix.h"
#include "../imageModel/Image.h"

#include "../histograms/ShapeHistogram.h"
#include "../pht/PHTEntry.h"
#include "../pht/PHoughTransform.h"
#include "../correlation/CrossCorrelation.h"

#include "Treatments.h"
#include "Segmentation.h"
#include "GeometricHistgoram.h"
#include "ProHoughTransform.h"
#include "LandmarkDetection.h"

LandmarkDetection::LandmarkDetection()
{
	// TODO Auto-generated constructor stub

}

LandmarkDetection::~LandmarkDetection()
{
	// TODO Auto-generated destructor stub
}

vector<ptr_Point> LandmarkDetection::landmarksAutoDectect(Image sceneImage,
	AngleAccuracy acc, int cols, int templSize, int sceneSize,ptr_Point &ePoint, double &angleDiff)
{
	vector<ptr_Point> result;
	Image modelImage = Treatments::refImage;
	vector<ptr_Point> manualLMs = modelImage.getListOfManualLandmarks();
	vector<ptr_Line> mLines = modelImage.getListOfLines();
	vector<ptr_Line> sLines = sceneImage.getListOfLines();
	int width = modelImage.getGrayMatrix()->getCols();
	int height = modelImage.getGrayMatrix()->getRows();

	ShapeHistogram mHistogram;
	vector<LocalHistogram> mLocalHist = mHistogram.constructPGH(mLines);
	mHistogram.constructPGHMatrix(mLocalHist, acc, cols);

	ShapeHistogram sHistogram;
	vector<LocalHistogram> sLocalHist = sHistogram.constructPGH(sLines);
	sHistogram.constructPGHMatrix(sLocalHist, acc, cols);

	double bhatt = bhattacharyyaMetric(mHistogram, sHistogram);

	if (bhatt > 0.9)
	{
		//double angleDiff;
		//ptr_Point ePoint;
		PHoughTransform mpht;
		mpht.setRefPoint(new Point(width / 2, height / 2));
		vector<ptr_PHTEntry> entriesTabel = mpht.constructPHTTable(mLines);
		vector<ptr_Point> phtEsLM = phtLandmarks(entriesTabel, mpht.getRefPoint(),
			sLines, width, height, manualLMs, angleDiff, ePoint);

		cout<<"\nAngle difference: "<<angleDiff;

		result = verifyLandmarks(modelImage, sceneImage, manualLMs, phtEsLM,
			templSize, sceneSize, angleDiff, ePoint);
	}

	return result;
}
