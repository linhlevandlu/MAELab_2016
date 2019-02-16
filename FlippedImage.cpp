#include <iostream>
#include <string>
#include <vector>
#include <fstream>
using namespace std;

#include "imageModel/Point.h"
#include "imageModel/Line.h"
#include "imageModel/Edge.h"
#include "imageModel/Matrix.h"
#include "imageModel/Image.h"
#include "io/Reader.h"

void saveLandmarks(vector<Point> lmlist, string fileName, int imgHeight) {
	ofstream outfile(fileName.c_str());
	outfile << "LM=" << lmlist.size() << "\n";
	for (size_t i = 0; i < lmlist.size(); ++i) {
		Point pi = lmlist.at(i);

		outfile << pi.getX() << " " << imgHeight - pi.getY() << "\n";
	}
	//outfile << "IMAGE=" << image.getName();
	outfile.close();
}
Matrix<RGB> flipX_images(Image inImage, string lmFile, string saveImg,
		string savelm) {

	RGB color;
	color.R = 0;
	color.G = 128;
	color.B = 0;

	Matrix<RGB> rgbMatrix = inImage.getRGBMatrix();
	int rows = rgbMatrix.getRows();
	int cols = rgbMatrix.getCols();
	Matrix<RGB> rgbOutput(rows, cols, color);
	int x0 = cols / 2;
	int x2, y2;
	// Flipped landmarks
	vector<Point> mLandmarks = inImage.readManualLandmarks(lmFile);
	vector<Point> lmFlipped;
	for (size_t i = 0; i < mLandmarks.size(); i++) {
		Point pi = mLandmarks.at(i);
		x2 = -pi.getX() + (2 * x0);
		y2 = pi.getY();
		lmFlipped.push_back(Point(x2, y2));
		//cout << x2 << " - " << y2 << endl;
	}
	saveLandmarks(lmFlipped, savelm, rows);

	// Flipped image

	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			//cout<<"x - y: " << c <<"\t "<< r<<endl;
			RGB value = rgbMatrix.getAtPosition(r, c);
			x2 = -c + (2 * x0) - 1;
			y2 = r;
			//cout<<"x1 - y1: " << x2 <<"\t "<< y2<<endl;
			rgbOutput.setAtPosition(y2, x2, value);
		}
	}
	saveRGB(saveImg.c_str(), &rgbOutput);
	return rgbOutput;
}
Matrix<RGB> flipY_images(Image inImage, string lmFile, string saveImg,
		string savelm) {

	RGB color;
	color.R = 0;
	color.G = 128;
	color.B = 0;

	Matrix<RGB> rgbMatrix = inImage.getRGBMatrix();
	int rows = rgbMatrix.getRows();
	int cols = rgbMatrix.getCols();
	Matrix<RGB> rgbOutput(rows, cols, color);
	int y0 = rows / 2;
	int x2, y2;
	// Flipped landmarks
	vector<Point> mLandmarks = inImage.readManualLandmarks(lmFile);
	vector<Point> lmFlipped;
	for (size_t i = 0; i < mLandmarks.size(); i++) {
		Point pi = mLandmarks.at(i);
		x2 = pi.getX();
		y2 = -pi.getY() + (2 * y0);
		lmFlipped.push_back(Point(x2, y2));
		//cout << x2 << " - " << y2 << endl;
	}
	saveLandmarks(lmFlipped, savelm, rows);

	// Flipped image

	for (int r = 0; r < rows; r++) { // y coordinate
		for (int c = 0; c < cols; c++) { // x coordinate
			//cout<<"x - y: " << c <<"\t "<< r<<endl;
			RGB value = rgbMatrix.getAtPosition(r, c);
			x2 = c;
			y2 = -r + (2 * y0) - 1;
			//cout<<"x1 - y1: " << x2 <<"\t "<< y2<<endl;
			rgbOutput.setAtPosition(y2, x2, value);
		}
	}
	saveRGB(saveImg.c_str(), &rgbOutput);
	return rgbOutput;
}

Matrix<RGB> flipXY_images(Image inImage, string lmFile, string saveImg,
		string savelm) {

	RGB color;
	color.R = 0;
	color.G = 128;
	color.B = 0;

	Matrix<RGB> rgbMatrix = inImage.getRGBMatrix();
	int rows = rgbMatrix.getRows();
	int cols = rgbMatrix.getCols();
	Matrix<RGB> rgbOutput(rows, cols, color);
	int x0 = cols / 2;
	int y0 = rows / 2;
	int x2, y2;
	// Flipped landmarks
	vector<Point> mLandmarks = inImage.readManualLandmarks(lmFile);
	vector<Point> lmFlipped;
	for (size_t i = 0; i < mLandmarks.size(); i++) {
		Point pi = mLandmarks.at(i);
		x2 = -pi.getX() + (2 * x0);
		y2 = -pi.getY() + (2 * y0);
		lmFlipped.push_back(Point(x2, y2));
		//cout << x2 << " - " << y2 << endl;
	}
	saveLandmarks(lmFlipped, savelm, rows);

	// Flipped image

	for (int r = 0; r < rows; r++) { // y coordinate
		for (int c = 0; c < cols; c++) { // x coordinate
			//cout<<"x - y: " << c <<"\t "<< r<<endl;
			RGB value = rgbMatrix.getAtPosition(r, c);
			x2 = -c + (2 * x0) - 1;
			y2 = -r + (2 * y0) - 1;
			//cout<<"x1 - y1: " << x2 <<"\t "<< y2<<endl;
			rgbOutput.setAtPosition(y2, x2, value);
		}
	}
	saveRGB(saveImg.c_str(), &rgbOutput);
	return rgbOutput;
}

int main(int argc, char* argv[]) {
	cout << "\n Flip images function" << endl;
	string imgPath, lmFile, saveImg, saveLM;
	if (argc == 0) {
		imgPath =
				"/home/linhpc/CNN_data/pronotum/v1_abc/original/Prono_001.JPG";
		lmFile = "/home/linhpc/CNN_data/pronotum/v1_abc/landmarks/p_001.TPS";
		saveImg = "";
		saveLM = "";
	} else {
		if (argc == 5) {
			cout << argc << endl;
			imgPath = argv[1];
			lmFile = argv[2];
			saveImg = argv[3];
			saveLM = argv[4];
		}
	}
	Image inImage(imgPath);
	flipXY_images(inImage, lmFile, saveImg, saveLM);
	return 0;
}
