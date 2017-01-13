/*
 * Drawing.h
 *
 *  Created on: Dec 18, 2016
 *      Author: linh
 */

#ifndef DRAWING_H_
#define DRAWING_H_

vector<ptr_Point> drawingLine(ptr_Line line, RGB color);
vector<ptr_Point> drawingCircle(ptr_Point center,int radius, RGB color);
vector<ptr_Point> fillCircle(ptr_Point center, int radius, RGB color) ;
#endif /* DRAWING_H_ */
