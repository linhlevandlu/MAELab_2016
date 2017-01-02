/*
 * Segmentation.h
 *
 *  Created on: Dec 2, 2016
 *      Author: linh
 */

#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

class Segmentation: public Treatments
{
public:
	Segmentation();
	virtual ~Segmentation();
	vector<ptr_Line> segment(int minDistance);
	ptr_IntMatrix threshold(int tValue, int maxValue);
	vector<ptr_Edge> canny();
	vector<ptr_Point> boundingBox();
};

#endif /* SEGMENTATION_H_ */
