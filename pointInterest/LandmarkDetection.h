/*
 * LandmarkDetection.h
 *
 *  Created on: Dec 7, 2016
 *      Author: linh
 */

#ifndef LANDMARKDETECTION_H_
#define LANDMARKDETECTION_H_

class LandmarkDetection: public Treatments
{
public:
	LandmarkDetection();
	virtual ~LandmarkDetection();
	vector<ptr_Point> landmarksAutoDectect(Image sceneImage, AngleAccuracy acc,
		int cols, int templSize, int sceneSize, ptr_Point &ePoint,
		double &angleDiff);
	void landmarksOnDir(string modelName,string folderScene,
		vector<string> sceneImages, AngleAccuracy acc, int cols, int templSize,
		int sceneSize, ptr_Point &ePoint, double &angleDiff,string saveFolder);
};

#endif /* LANDMARKDETECTION_H_ */
