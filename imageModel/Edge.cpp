/*
 * Edge.cpp
 *
 *  Created on: Sep 15, 2016
 *      Author: linh
 */
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string.h>
#include <fstream>
#include <time.h>
#include <istream>
#include <algorithm>

using namespace std;

#include "Point.h"
#include "Line.h"
#include "Edge.h"

//=================================================== Constructor ===========================================

Edge::Edge()
{
	//listOfLines = NULL;
	//listOfPoints = NULL;
	//listOfBreakPoints = NULL;

}

Edge::~Edge()
{
	// TODO Auto-generated destructor stub
}

Edge::Edge(std::vector<ptr_Point> points)
{
	listOfPoints = points;
}
//=================================================== Get and set methods ===========================================

std::vector<ptr_Point> Edge::getPoints()
{
	return listOfPoints;
}

void Edge::setPoints(std::vector<ptr_Point> points)
{
	listOfPoints = points;
}

//=================================================== Private methods ===========================================

/*
 *
 * Check a point is exist in list of Break Point or not.
 *
 */
bool Edge::checkPointInList(std::vector<ptr_Point> listPoints, ptr_Point point)
{
	for (size_t i = 0; i < listPoints.size(); i++)
	{
		ptr_Point p = listPoints.at(i);
		if (point->getX() == p->getX() && point->getY() == p->getY())
			return true;
	}
	return false;
}



static std::vector<ptr_Point> vvp; // used to keep the break points after recursive time
void Edge::breakEdge(double minDistance)
{ // in old program, minDistance is constant with value is 3
	if (listOfPoints.size() <= 0)
		return;
	ptr_Point p0;
	ptr_Point pend;
	int size = listOfPoints.size();

	p0 = listOfPoints.at(0);
	pend = listOfPoints.at(size - 1);

	if (size > 2)
	{
		ptr_Line line = new Line(p0, pend);
		double distance = 0, maxDistance = 0;
		size_t imax = 0;
		for (int i = 1; i < size - 1; i++)
		{
			ptr_Point pi = listOfPoints.at(i);
			distance = line->perpendicularDistance(pi);
			if (distance > maxDistance)
			{
				maxDistance = distance;
				imax = i;
			}
		}
		if (maxDistance > minDistance)
		{ // continue break the edge
			std::vector<ptr_Point> part1(listOfPoints.begin(),
				listOfPoints.begin() + imax + 1);
			std::vector<ptr_Point> part2(listOfPoints.begin() + imax,
				listOfPoints.end());
			ptr_Edge edge1 = new Edge(part1);
			ptr_Edge edge2 = new Edge(part2);
			edge1->breakEdge(minDistance);
			edge2->breakEdge(minDistance);
		}
	}

	if (!checkPointInList(vvp, p0))
		vvp.push_back(p0);
	if (!checkPointInList(vvp, pend))
		vvp.push_back(pend);

}

//=================================================== Public methods ===========================================
std::vector<ptr_Point> Edge::segment(double minDistance)
{
	listOfBreakPoints.clear();
	vvp.clear();
	breakEdge(minDistance);
	listOfBreakPoints = vvp;

	return listOfBreakPoints;
}
vector<ptr_Line> Edge::getLines(vector<ptr_Point> listPoints) {
	vector<ptr_Line> listLines;
	ptr_Point p0;
	ptr_Point p1;

	if (listPoints.size() > 0) {
		p0 = listPoints.at(0);
		for (size_t i = 1; i < listPoints.size(); i++) {
			p1 = listPoints.at(i);
			ptr_Line l = new Line(p0, p1);
			listLines.push_back(l);
			p0 = p1;
		}
	}
	return listLines;
}
void Edge::sortByX()
{
	std::sort(listOfPoints.begin(), listOfPoints.end(), xComparation);
}
void Edge::sortByY()
{
	std::sort(listOfPoints.begin(), listOfPoints.end(), yComparation);
}

