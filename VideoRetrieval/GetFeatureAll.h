#ifndef GETFEATUREALL_H
#define GETFEATUREALL_H

#include <vector>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h> 
#include <ml.h>

using namespace std;

void GetFeatureALL(vector <vector <CvPoint>> &complete_trajectory, double *w, vector <vector <double>> &FeatAll);

void GetFeatureAllRespective(vector<vector<CvPoint>> &complete_trajectory, double *w, vector<vector<double>> &FeatAllV, vector<vector<double>> &FeatAllTraj);
#endif