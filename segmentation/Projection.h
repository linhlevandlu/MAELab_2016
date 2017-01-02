/*
 * Projection.h
 *
 *  Created on: Jan 2, 2017
 *      Author: linh
 */

#ifndef PROJECTION_H_
#define PROJECTION_H_
enum PROJECTION_TYPE
{
	Horizontal = 1,
	Vertical = 2
};
ptr_IntMatrix quantization(ptr_IntMatrix grayMatrix, int graylevel);
int* histogramProjection(ptr_IntMatrix binaryMatrix, PROJECTION_TYPE pType,
		int &size);
void analysisProjection(int* projection, int psize,int &leftLimit, int &rightLimit);
vector<ptr_Point> boundingBoxDetection(ptr_IntMatrix grayMatrix);
#endif /* PROJECTION_H_ */
