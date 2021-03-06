/*
 * Line.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: linh
 */
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string.h>
#include <fstream>
#include <time.h>

using namespace std;

#include "Point.h"
#include "Line.h"

//=================================================== Constructor ===========================================
Line::Line()
{
}
Line::Line(const Line& otherLine)
{
	begin = otherLine.begin;
	end = otherLine.end;
	dx = otherLine.dx;
	dy = otherLine.dy;
	length = otherLine.length;
	equation = otherLine.equation;
}

Line::Line(ptr_Point ep1, ptr_Point ep2)
{
	begin = ep1;
	end = ep2;

	dx = end->getX() - begin->getX();
	dy = end->getY() - begin->getY();

	length = lengthOfLine();

	equation = equationOfLine();
}

Line::~Line()
{
	// TODO Auto-generated destructor stub
}
//=================================================== Get and set ===========================================
ptr_Point Line::getBegin()
{
	return begin;
}
ptr_Point Line::getEnd()
{
	return end;
}

void Line::setBegin(ptr_Point ep1)
{
	begin = ep1;
}
void Line::setEnd(ptr_Point ep2)
{
	end = ep2;
}

double Line::getLength()
{
	return length;
}
std::vector<double> Line::getEquation()
{
	return equation;
}

//=================================================== Private methods ===========================================
/*
 * Check the line is a point or not
 */
bool Line::isPoint()
{
	if (begin->getX() == end->getX() && begin->getY() == end->getY())
		return true;
	return false;
}

/*
 * Calculate the length of line
 */
double Line::lengthOfLine()
{
	return sqrt(pow((double) dx, 2) + pow((double) dy, 2));
}

/*
 * Get the equation of line (ax + by + c = 0)
 */
std::vector<double> Line::equationOfLine()
{
	std::vector<double> equa;
	if (dx == 0)
	{ //line x = n
		equa.push_back(1.0);
		equa.push_back(0.0);
		equa.push_back((double) (end->getX()));
		return equa;
	}
	else
	{
		if (dy == 0)
		{ // line y = m
			equa.push_back(0);
			equa.push_back(1);
			equa.push_back((end->getY()));
			return equa;
		}
		else
		{
			if (dx != 0 && dy != 0)
			{ // normal line
				double m = (double) dy / (double) dx;

				equa.push_back(m);
				equa.push_back(-1);
				equa.push_back((double) (begin->getY() - (m * begin->getX())));
				return equa;
			}
		}
	}

	return equa; // NULL
}
//=================================================== Public Methods ===========================================
double Line::perpendicularDistance(ptr_Point point)
{
	double a = equation.at(0);
	double b = equation.at(1);
	double c = equation.at(2);
	double distance = 0;
	if (b != 0 && a != 0)
	{
		distance = abs(
			((a * point->getX()) + (b * point->getY()) + c)
				/ (sqrt(pow(a, 2) + pow(b, 2))));
	}
	if (b == 0 && a == 1)
		distance = abs(point->getX() - c);
	if (a == 0 && b == 1)
		distance = abs(point->getY() - c);
	return distance;
}

double Line::angleLines(Line otherLine)
{

	double slope1, slope2;
	if (dx == 0 && otherLine.dx == 0)
		return 0;
	if (dx == 0 && otherLine.dx != 0)
	{ // line 1 is parallel with Oy
		slope2 = (double) otherLine.dy / (double) otherLine.dx;
		return atan(abs(1 / slope2)) * 180 / M_PI;
	}
	if (dx != 0 && otherLine.dx == 0)
	{ //otherLine is parallel with Oy
		slope1 = (double) dy / (double) dx;
		return atan(abs(1 / slope1)) * 180 / M_PI;
	}

	slope1 = (double) dy / (double) dx;
	slope2 = (double) otherLine.dy / (double) otherLine.dx;

	if ((slope1 == 0 && slope2 == 0)) // parallel lines
		return 0;
	if (slope1 * slope2 == -1) // perpendicular lines
		return 90;
	if (slope1 == 0 && slope2 != 0)
	{ // line 1 is parallel with Ox
		return atan(abs(slope2)) * 180 / M_PI;
	}
	if (slope1 != 0 && slope2 == 0)
	{ // otherLine is parallel with Ox
		return atan(abs(slope1)) * 180 / M_PI;
	}
	return atan(abs((slope1 - slope2) / (1 + slope1 * slope2))) * 180 / M_PI;
}

ptr_Point Line::intersection(Line otherLine)
{

	double a1 = equation.at(0);
	double b1 = equation.at(1);
	double c1 = equation.at(2);

	double a2 = otherLine.equation.at(0);
	double b2 = otherLine.equation.at(1);
	double c2 = otherLine.equation.at(2);

	if ((b1 == 0 && b2 == 0) || (a1 == a2)) // parallel lines
		return new Point(-1, -1);

	double a; // = a1 - a2;
	double c; // = c2 - c1;
	double x = 0; // = c / a;
	double y = 0; // = (a1 * x) + c1;

	if (b1 == 0 && b2 != 0)
	{
		x = c1;
		y = (a2 * x) + c2;
	}
	else
	{
		if (b1 != 0 && b2 == 0)
		{
			x = c2;
			y = (a1 * x) + c1;
		}
	}
	if (b1 != 0 && b2 != 0)
	{
		a = a1 - a2;
		c = c2 - c1;
		x = c / a;
		y = (a1 * x) + c1;
	}
	return new Point(round(x), round(y));
}

bool Line::checkBelongPoint(ptr_Point point)
{
	if (perpendicularDistance(point) == 0)
		return true;
	return false;
}
vector<ptr_Point> Line::interParallel(Line line1, Line line2, double distance1,
	double distance2, int width, int height)
{
	vector<double> equation1 = line1.equationOfLine();
	vector<double> equation2 = line2.equationOfLine();
	double a = equation1[0];
	double b = equation1[2];
	double c = equation2[0];
	double d = equation2[2];
	double e = distance1 * (sqrt((a * a) + 1));
	double f = distance2 * (sqrt((c * c) + 1));

	vector<ptr_Point> result;

	double x0 = 0, y0 = 0;
	x0 = (f + b - d - e) / (c - a);
	if (x0 >= 0 && x0 < width)
	{
		y0 = a * x0 + b - e;
		if (y0 >= 0 && y0 < height)
		{
			result.push_back(new Point(x0, y0));
		}
	}
	x0 = (b - e - f - d) / (c - a);
	if (x0 >= 0 && x0 < width)
	{
		y0 = a * x0 + b - e;
		if (y0 >= 0 && y0 < height)
		{
			result.push_back(new Point(x0, y0));
		}
	}
	x0 = (f + b - d + e) / (c - a);
	if (x0 >= 0 && x0 < width)
	{
		y0 = a * x0 + b + e;
		if (y0 >= 0 && y0 < height)
		{
			result.push_back(new Point(x0, y0));
		}
	}
	x0 = (b + e - f - d) / (c - a);
	if (x0 >= 0 && x0 < width)
	{
		y0 = a * x0 + b + e;
		if (y0 >= 0 && y0 < height)
		{
			result.push_back(new Point(x0, y0));
		}
	}
	return result;
}
