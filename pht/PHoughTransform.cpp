/*
 * PHoughTransform.cpp
 *
 *  Created on: Nov 30, 2016
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
#include "../imageModel/Matrix.h"
#include "../imageModel/Image.h"

#include "PHTEntry.h"

#include "PHoughTransform.h"

bool closetLine(ptr_Line line1, ptr_Line line2)
{
	int cond1 = 60;
	int cond2 = 15;
	int cond3 = 5;

	double distance1 = line2->perpendicularDistance(line1->getBegin());
	double distance2 = line2->perpendicularDistance(line1->getEnd());
	if (line1->getLength() > cond1 && line2->getLength() > cond1
		&& line1->angleLines(*line2) >= cond2
		&& (distance1 <= cond3 || distance2 <= cond3))
		return true;
	return false;
}
double computeDistance(ptr_Line objLine, ptr_Point rPoint)
{
	return objLine->perpendicularDistance(rPoint);
}

ptr_Point closestPoint(ptr_Line objLine, ptr_Point origin)
{
	vector<double> equation = objLine->getEquation();
	double a = equation.at(0);
	double b = equation.at(1);
	double c = equation.at(2);
	double x, y;

	if (a == 0 && b == 1)
	{
		x = origin->getX();
		y = c;
	}
	else
	{
		if (a == 1 && b == 0)
		{
			x = c;
			y = origin->getY();
		}
		else
		{
			x = (b * (b * origin->getX() - a * origin->getY()) - (a * c))
				/ (a * a + b * b);
			y = (a * (-b * origin->getX() + a * origin->getY()) - (b * c))
				/ (a * a + b * b);
		}
	}
	return new Point(x, y);
}

double computeAngle(ptr_Line objLine, ptr_Point rPoint)
{
	ptr_Line oX = new Line(rPoint,
		new Point(rPoint->getX() + 100, rPoint->getY()));
	ptr_Point pCloset = closestPoint(objLine, rPoint);
	ptr_Line distanceLine = new Line(rPoint, pCloset);
	double theta = oX->angleLines(*distanceLine);
	if (pCloset->getY() > rPoint->getY())
		return (360 - theta);
	return theta;
}

//==================================== Methods in class ==================================================
PHoughTransform::PHoughTransform()
{
	// TODO Auto-generated constructor stub

}

PHoughTransform::~PHoughTransform()
{
	// TODO Auto-generated destructor stub
}

void PHoughTransform::setRefPoint(ptr_Point rpoint)
{
	refPoint = rpoint;
}
ptr_Point PHoughTransform::getRefPoint()
{
	return refPoint;
}
vector<ptr_PHTEntry> PHoughTransform::getPHTEntries()
{
	//if (!phtEntries.empty())
	return phtEntries;
	//return null;
}
vector<ptr_PHTEntry> PHoughTransform::constructPHTTable(vector<ptr_Line> lines)
{
	vector<ptr_PHTEntry> entries;
	for (size_t i = 0; i < lines.size(); i++)
	{
		ptr_Line line1 = lines.at(i);
		HoughSpace hs1;
		hs1.angle = computeAngle(line1, refPoint);
		hs1.distance = computeDistance(line1, refPoint);
		for (size_t j = 0; j < lines.size(); j++)
		{
			ptr_Line line2 = lines.at(j);

			if (i != j && closetLine(line1, line2))
			{
				HoughSpace hs2;
				hs2.angle = computeAngle(line2, refPoint);
				hs2.distance = computeDistance(line2, refPoint);

				ptr_PHTEntry entry = new PHTEntry();
				entry->setRefLine(line1);
				entry->setObjLine(line2);
				entry->addHoughSpace(hs1);
				entry->addHoughSpace(hs2);

				entries.push_back(entry);
			}
		}
	}
	phtEntries = entries;
	//cout << "\nSize of pht table: " << entries.size() << endl;
	return entries;
}
