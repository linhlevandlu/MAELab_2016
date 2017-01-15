/*
 * Texture.h
 *
 *  Created on: Jan 13, 2017
 *      Author: linh
 */

#ifndef TEXTURE_H_
#define TEXTURE_H_
vector<ptr_IntMatrix> splitImage(ptr_IntMatrix grayImage);
ptr_IntMatrix lbpcDistribution(ptr_IntMatrix region);
double likelihoodRatio(ptr_IntMatrix sample, ptr_IntMatrix model);
vector<ptr_IntMatrix> splitImage16x16(ptr_IntMatrix grayImage);
#endif /* TEXTURE_H_ */
