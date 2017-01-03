/*
 * Reader.h
 *
 *  Created on: Oct 6, 2016
 *      Author: linh
 */

#ifndef READER_H_
#define READER_H_

ptr_RGBMatrix readJPGToRGB(const char* filename);
void saveRGB(const char* filename, ptr_RGBMatrix rgbMatrix);
void saveGrayScale(const char* filename, ptr_IntMatrix grayMatrix);
vector<string> readDirectory(const char* dirPath);
vector<ptr_Point> readTPSFile(const char* filename);
#endif /* READER_H_ */
