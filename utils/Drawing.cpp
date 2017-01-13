/*
 * Drawing.cpp
 *
 *  Created on: Dec 18, 2016
 *      Author: linh
 */
#include <iostream>
#include <vector>
#include <stdlib.h>

using namespace std;
#include "../imageModel/Point.h"
#include "../imageModel/Line.h"

#include "Drawing.h"

// draw a line using Breshenham algorithm
vector<ptr_Point> drawingLine(ptr_Line line, RGB color)
{
	ptr_Point begin = line->getBegin();
	ptr_Point end = line->getEnd();
	begin->setColor(color);
	end->setColor(color);
	int dx = abs(end->getX() - begin->getX());
	int dy = abs(end->getY() - begin->getY());

	int x = begin->getX();
	int y = begin->getY();
	int c = dx - dy;
	int c2 = 2 * c;

	int x_unit = 1, y_unit = 1;

	if (end->getX() - begin->getX() < 0)
		x_unit = -x_unit;
	if (end->getY() - begin->getY() < 0)
		y_unit = -y_unit;
	vector<ptr_Point> listOfPoints;
	listOfPoints.push_back(new Point(x, y, color));
	// vertical line
	if (begin->getX() == end->getX())
	{
		while (y != end->getY())
		{
			y += y_unit;
			listOfPoints.push_back(new Point(x, y, color));
		}
	}
	else
	{ // horizontal line
		if (begin->getY() == end->getY())
		{
			while (x != end->getX())
			{
				x += x_unit;
				listOfPoints.push_back(new Point(x, y, color));
			}
		}
		else
		{
			if (begin->getX() != end->getX() && begin->getY() != end->getY())
			{
				while (x != end->getX())
				{
					c2 = 2*c;
					if(c2 > -dy)
					{
						c = c- dy;
						x +=x_unit;
					}
					if(c2 <dx)
					{
						c +=dx;
						y += y_unit;
					}

					listOfPoints.push_back(new Point(x, y, color));
				}
			}
		}
	}
	return listOfPoints;
}

vector<ptr_Point> put8pixel(ptr_Point center, ptr_Point p, RGB color)
{
	int xc = center->getX();
	int yc = center->getY();
	int x = p->getX();
	int y = p->getY();
	vector<ptr_Point> eightPoints;
	eightPoints.push_back(new Point(x + xc, y + yc, color));
	eightPoints.push_back(new Point(-x + xc, y + yc, color));
	eightPoints.push_back(new Point(x + xc, -y + yc, color));
	eightPoints.push_back(new Point(-x + xc, -y + yc, color));
	eightPoints.push_back(new Point(y + xc, x + yc, color));
	eightPoints.push_back(new Point(-y + xc, x + yc, color));
	eightPoints.push_back(new Point(y + xc, -x + yc, color));
	eightPoints.push_back(new Point(-y + xc, -x + yc, color));
	return eightPoints;
}
vector<ptr_Point> drawingCircle(ptr_Point center, int radius, RGB color)
{
	int x = 0;
	int y = radius;
	int f = 1 - radius;
	vector<ptr_Point> result;
	vector<ptr_Point> drawPoints = put8pixel(center, new Point(x, y), color);
	result.insert(result.end(), drawPoints.begin(), drawPoints.end());
	while (x < y)
	{
		if (f < 0)
		{
			f += 2 * x + 3;
		}
		else
		{
			f += 2 * (x - y) + 5;
			y--;
		}
		x++;
		drawPoints = put8pixel(center, new Point(x, y), color);
		result.insert(result.end(), drawPoints.begin(), drawPoints.end());
	}
	return result;
}
vector<ptr_Point> fillCircle(ptr_Point center, int radius, RGB color) {
	vector<ptr_Point> result;
	for (int y = -radius; y <= radius; y++){
		for (int x = -radius; x <= radius; x++){
			if (x * x + y * y <= radius * radius){
				result.push_back(new Point(center->getX() + x, center->getY()+y,color));
			}
		}
	}
	return result;
}
