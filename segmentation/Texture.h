/*
 * Texture.h
 *
 *  Created on: Jan 13, 2017
 *      Author: linh
 */

#ifndef TEXTURE_H_
#define TEXTURE_H_
vector<ptr_IntMatrix> splitImage(ptr_IntMatrix grayImage);
double contrastLBP(ptr_IntMatrix region,double &lbp);

#endif /* TEXTURE_H_ */
