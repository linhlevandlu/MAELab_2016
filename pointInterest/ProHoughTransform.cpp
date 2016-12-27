/*
 * ProHoughTransform.cpp
 *
 *  Created on: Dec 2, 2016
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

#include "../pht/PHTEntry.h"
#include "../pht/PHoughTransform.h"

#include "Treatments.h"
#include "ProHoughTransform.h"

vector<ptr_Point> findLandmarks(ptr_Point refPoint, ptr_Point esPoint,
	vector<ptr_Point> refLandmarks, int width, int height, int &positive)
{
	vector<ptr_Point> esLandmarks;
	positive = 0;
	for (size_t t = 0; t < refLandmarks.size(); t++)
	{
		ptr_Point temp = refLandmarks.at(t);
		ptr_Point lm = new Point();
		lm->setX(temp->getX());
		lm->setY(temp->getY());

		int px = refPoint->getX() - esPoint->getX();
		int py = refPoint->getY() - esPoint->getY();
		int x;
		int y;
		x = lm->getX() - px;
		if (py > 0)
			y = lm->getY() - py;
		else
			y = lm->getY() + py;
		esLandmarks.push_back(new Point(x, y));
		if (x >= 0 && x <= width && y >= 0 && y <= height)
		{
			positive += 1;
		}
	}
	return esLandmarks;
}

ptr_Point refPointInScene(ptr_PHTEntry entry, vector<ptr_Line> matchLines,
	double &angleDiff, vector<ptr_Point> refLandmarks, int width, int height)
{

	ptr_Point inter = new Point(0, 0);
	ptr_Point refPoint = new Point(width / 2, height / 2);

	if (matchLines.size() <= 0)
		return inter;
	ptr_Line objl1 = matchLines.at(0);
	ptr_Line objl2 = matchLines.at(1);

	ptr_Line lineEntry1 = entry->getRefLine();
	ptr_Line lineEntry2 = entry->getObjLine();

	HoughSpace hs1, hs2;
	hs1 = entry->getListHoughSpace().at(0);
	hs2 = entry->getListHoughSpace().at(1);

	// display the angle
	double angle3 = lineEntry1->angleLines(*objl1);
	double angle4 = lineEntry1->angleLines(*objl2);

	vector<ptr_Point> intersects1 = objl1->interParallel(*objl1, *objl2,
		hs1.distance, hs2.distance, width, height);
	int max = 0;
	vector<ptr_Point> estLM;
	vector<double> angles;
	for (size_t i = 0; i < intersects1.size(); i++)
	{
		ptr_Point esPoint = intersects1.at(i);
		int positive = 0;
		vector<ptr_Point> lms = findLandmarks(refPoint, esPoint, refLandmarks,
			width, height, positive);
		if (positive > max)
		{
			estLM.clear();
			estLM.push_back(esPoint);
			angles.clear();
			angles.push_back(angle3);
			max = positive;
		}
		else
		{
			if (positive == max)
			{
				estLM.push_back(esPoint);
				angles.push_back(angle3);
			}
		}
	}

	vector<ptr_Point> intersects2 = objl1->interParallel(*objl1, *objl2,
		hs2.distance, hs1.distance, width, height);
	for (size_t i = 0; i < intersects2.size(); i++)
	{
		ptr_Point esPoint = intersects2.at(i);
		int positive = 0;
		vector<ptr_Point> lms = findLandmarks(refPoint, esPoint, refLandmarks,
			width, height, positive);
		if (positive > max)
		{
			estLM.clear();
			estLM.push_back(esPoint);
			angles.clear();
			angles.push_back(angle4);
			max = positive;
		}
		else
		{
			if (positive == max)
			{
				estLM.push_back(esPoint);
				angles.push_back(angle4);
			}
		}
	}

	if (estLM.size() == 1)
	{
		inter = estLM.at(0);
		angleDiff = angles.at(0);
	}
	else
	{
		double angleds = 180;
		for (size_t i = 0; i < estLM.size(); i++)
		{
			ptr_Point es = estLM.at(i);
			double angle = angles.at(i);
			if (angle <= angleds)
			{
				angleds = angle;
				angleDiff = angleds;
				inter = es;
			}
		}
	}
	cout << "\n Reference point in scene: " << inter->getX() << ", "
		<< inter->getY();

	return inter;
}
bool similarPairLines(ptr_Line ref1, ptr_Line ref2, ptr_Line scene1,
	ptr_Line scene2)
{
	int cond1 = 1;
	int cond2 = 1;
	int cond3 = 2;

	double refAngle = ref1->angleLines(*ref2);
	double rd1 = ref1->perpendicularDistance(ref2->getBegin());
	double rd2 = ref1->perpendicularDistance(ref2->getEnd());
	double rd = rd1 + rd2;

	double sceneAngle = scene1->angleLines(*scene2);
	double sd1 = scene1->perpendicularDistance(scene2->getBegin());
	double sd2 = scene1->perpendicularDistance(scene2->getEnd());
	double sd = sd1 + sd2;

	if (abs(refAngle - sceneAngle) < cond1
		&& (abs(
			(ref1->getLength() / scene1->getLength())
				- (ref2->getLength() / scene2->getLength())) < cond2)
		&& (abs(rd - sd) < cond3))
	{
		return true;
	}
	return false;
}

ptr_PHTEntry findHoughSpace(vector<ptr_PHTEntry> entryTable, ptr_Line line1,
	ptr_Line line2)
{
	ptr_PHTEntry entry = NULL;
	for (size_t i = 0; i < entryTable.size(); i++)
	{
		ptr_Line ref1 = entryTable.at(i)->getRefLine();
		ptr_Line ref2 = entryTable.at(i)->getObjLine();
		if (similarPairLines(ref1, ref2, line1, line2))
		{
			entry = entryTable.at(i);
		}
	}
	return entry;
}

ptr_PHTEntry matchingInScene(vector<ptr_PHTEntry> entryTable,
	vector<ptr_Line> sceneLines, int width, int height,
	vector<ptr_Line> &maxVector)
{
	ptr_IntMatrix accumulator = new Matrix<int>(
		floor(sqrt(width * width + height * height)), 361);
	int maxValue = 0;
	ptr_PHTEntry maxEntry;
	for (size_t i = 0; i < sceneLines.size(); i++)
	{
		ptr_Line objLine1 = sceneLines.at(i);
		for (size_t j = 0; j < sceneLines.size(); j++)
		{
			ptr_Line objLine2 = sceneLines.at(j);
			if (i != j && closetLine(objLine2, objLine1))
			{
				ptr_PHTEntry entry = findHoughSpace(entryTable, objLine1, objLine2);
				if (entry != NULL)
				{
					vector<HoughSpace> hspace = entry->getListHoughSpace();
					for (size_t k = 0; k < hspace.size(); k++)
					{
						HoughSpace hsp = hspace.at(k);
						int angle = round(hsp.angle);
						int distance = round(hsp.distance);
						if (!isnan(angle) && !isnan(distance) && angle >= 0
							&& distance >= 0)
						{
							int value = accumulator->getAtPosition(distance, angle);
							accumulator->setAtPosition(distance, angle, value + 1);
							if (accumulator->getAtPosition(distance, angle) > maxValue)
							{
								maxVector.clear();
								maxVector.push_back(objLine1);
								maxVector.push_back(objLine2);
								maxValue = accumulator->getAtPosition(distance, angle);
								maxEntry = entry;
							}
							else
							{
								if (k == 0
									&& accumulator->getAtPosition(distance, angle) == maxValue)
								{
									maxVector.push_back(objLine1);
									maxVector.push_back(objLine2);
								}
							}
						}
					}
				}
			}
		}
	}
	return maxEntry;
}
vector<ptr_Point> phtLandmarks(vector<ptr_PHTEntry> entriesTable,
	ptr_Point refPoint, vector<ptr_Line> sceneLines, int width, int height,
	vector<ptr_Point> mLandmarks, double &angleDiff, ptr_Point &ePoint)
{
	vector<ptr_Point> eLandmarks;
	vector<ptr_Line> maxVector;
	ptr_PHTEntry entry = matchingInScene(entriesTable, sceneLines, width, height,
		maxVector);
	if (maxVector.size() > 0)
	{
		ePoint = refPointInScene(entry, maxVector, angleDiff, mLandmarks, width,
			height);
		double angle1 = entry->getRefLine()->angleLines(*entry->getObjLine());
		double angle2 = maxVector.at(0)->angleLines(*maxVector.at(1));
		angleDiff += abs(angle1 - angle2);
		int positive = 0;
		eLandmarks = findLandmarks(refPoint, ePoint, mLandmarks, width, height,
			positive);
	}
	return eLandmarks;
}
ProHoughTransform::ProHoughTransform()
{
	// TODO Auto-generated constructor stub

}

ProHoughTransform::~ProHoughTransform()
{
	// TODO Auto-generated destructor stub
}
PHoughTransform ProHoughTransform::constructPHT()
{
	ptr_IntMatrix grayImage = Treatments::refImage.getGrayMatrix();
	int width = grayImage->getCols();
	int height = grayImage->getRows();

	PHoughTransform pht;
	pht.setRefPoint(new Point(width / 2, height / 2));
	pht.constructPHTTable(Treatments::refImage.getListOfLines());
	return pht;
}

vector<ptr_Point> ProHoughTransform::estimateLandmarks(Image sImage,
	double &angleDiff, ptr_Point &ePoint)
{
	vector<ptr_Point> eLandmarks;
	ptr_IntMatrix mMatrix = Treatments::refImage.getGrayMatrix();
	int width = mMatrix->getCols();
	int height = mMatrix->getRows();

	vector<ptr_Point> mLandmarks =
		Treatments::refImage.getListOfManualLandmarks();
	vector<ptr_Line> mLines = Treatments::refImage.getListOfLines();
	vector<ptr_Line> sLines = sImage.getListOfLines();

	ptr_Point mPoint = new Point(width / 2, height / 2);

	PHoughTransform pht;
	pht.setRefPoint(mPoint);
	vector<ptr_PHTEntry> entryTable = pht.constructPHTTable(mLines);

	eLandmarks = phtLandmarks(entryTable, mPoint, sLines, width, height,
		mLandmarks, angleDiff, ePoint);

	return eLandmarks;

}
