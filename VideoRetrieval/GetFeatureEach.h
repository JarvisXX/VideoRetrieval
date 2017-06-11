#ifndef GETFEATUREEACH_H
#define GETFEATUREEACH_H

#include <vector>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h> 
#include <ml.h>
#include <math.h> 

using namespace std;

void GetFeatureEach(vector<CvPoint> &Traj, double *w, vector<double> &Feat);

void getFeatureEachRespective(vector<CvPoint> &Traj, double *w, vector<double> &FeatV, vector<double> &FeatTraj);
#endif //GETFEATUREEACH_H
