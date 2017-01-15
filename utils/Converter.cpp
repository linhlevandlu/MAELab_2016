/*
 * Converter.cpp
 *
 *  Created on: Jan 5, 2017
 *      Author: linh
 */
#include <cstdlib>
#include <ctime>
#include <string>
using namespace std;

#include "Converter.h"

// get a random number between min and max
int random(int min,int max)
{
	srand(time(NULL));
	return rand() % (max - min + 1) + min;
}

// get name from a full name
string getName(string fullPath,string split)
{
	int len = fullPath.length();
	int lIndex = fullPath.find_last_of(split);
	string temp = fullPath.substr(lIndex+1,len - lIndex-1);
	return temp.replace(3,1,"_");
}
