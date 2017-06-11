#ifndef CORNERFINDER_H
#define CORNERFINDER_H

#include "Object.h"

class CornerFinder
{
public:
	int featureCount;
	vector<Feature*> featureList;
	CornerFinder(void);
	~CornerFinder(void);
	Feature** featureMap;
	void initialize(int iW, int iH);
	Feature** getFeatureMap();
	vector<Feature*>* getFeatures();
	void findCorner(IplImage* frame, IplImage* pFG);
	int F_listsize;
};

float compare(Feature* a, Feature* b);

#endif