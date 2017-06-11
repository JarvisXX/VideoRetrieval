#include "clude.h"
#include "CornerFinder.h"

CornerFinder::CornerFinder(void)
	: F_listsize(0)
{
	featureCount = 0;
}

CornerFinder::~CornerFinder(void)
{
	free(featureMap);
	F_listsize = featureList.size();
	for (int i = 0; i < F_listsize; i++)
	{
		if (featureList[i] != NULL)
		{
			cvReleaseMat(&(featureList[i]->patch));
			delete featureList[i];
		}
	}
	featureList.erase(featureList.begin(), featureList.end());
	featureList.clear();
}


void CornerFinder::findCorner(IplImage* frame, IplImage* pFG)
{


	F_listsize = 0;

	F_listsize = featureList.size();

	for (int i = 0; i < F_listsize; i++)
	{
		if (featureList[i] != NULL)
		{
			cvReleaseMat(&(featureList[i]->patch));
			delete featureList[i];
		}
	}
	featureList.erase(featureList.begin(), featureList.end());
	featureList.clear();
	F_listsize = featureList.size();
	int iW = frame->width;
	int iH = frame->height;
	int stride = frame->widthStep;
	CvSize S = cvSize(iW, iH);

	memset(featureMap, 0, iW * iH * sizeof(Feature*));

	IplImage* gray = cvCreateImage(S, IPL_DEPTH_8U, 1);//
	cvCvtColor(frame, gray, CV_RGB2GRAY);//
	IplImage* eigImage = cvCreateImage(S, IPL_DEPTH_32F, 1);//
	IplImage* tempImage = cvCreateImage(S, IPL_DEPTH_32F, 1);//
	CvPoint2D32f* corners = (CvPoint2D32f*)calloc(1000, sizeof(CvPoint2D32f));///////

	int corner_count = 1000;
	double quality_level = 0.01;
	double min_distance = 2;
	int block_size = 3;
	int use_harris = 0;
	double k = 0.1;
	cvGoodFeaturesToTrack(gray, eigImage, tempImage, corners, &corner_count, quality_level, min_distance, pFG, block_size, use_harris, k);

	featureCount = 0;
	CvPoint2D32f* pCorner = corners;


	for (int i = 0; i < min(corner_count, 1000); i++)
	{
		if (pCorner->x < a_feature || pCorner->y < a_feature || pCorner->x > iW - 1 - a_feature || pCorner->y > iH - 1 - a_feature)
		{
			pCorner++;
			continue;
		}
		Feature* pF = new Feature;
		pF->x = pCorner->x;
		pF->y = pCorner->y;
		pF->rx = -1;
		pF->ry = -1;
		CvMat*fpf = cvCreateMat(l_feature, l_feature, CV_16SC3);///////////
		pF->patch = fpf;
		CvRect blob = cvRect(pF->x - a_feature, pF->y - a_feature, l_feature, l_feature);
		CvMat* sub = cvCreateMatHeader(l_feature, l_feature, CV_8UC3);/////////////////////////
		cvGetSubRect(frame, sub, blob);
		short * pD = pF->patch->data.s;
		uchar * pS;
		float sum = 0;
		long sumXX = 0;
		for (int k = 0; k < l_feature; k++)
		{
			pS = sub->data.ptr + sub->step * k;
			for (int l = 0; l < l_feature; l++)
			{
				*(pD++) = *(pS++);
				*(pD++) = *(pS++);
				*(pD++) = *(pS++);
			}
		}
		cvReleaseMatHeader(&sub);///////////////////////////
		pF->meanRGB = cvAvg(pF->patch);
		pF->mean = (pF->meanRGB.val[0] + pF->meanRGB.val[1] + pF->meanRGB.val[2]) / 3;
		CvMat* norm = cvCreateMat(l_feature, l_feature, CV_16SC3);
		cvSubS(pF->patch, cvScalarAll(pF->mean), norm);
		//cvReleaseMat(&pF->patch);/////////////////////////
		cvReleaseMat(&fpf);
		pF->patch = norm;
		short* pn = norm->data.s;
		for (int i = 0; i < l_feature * l_feature * 3; i++)
		{
			sumXX += *pn * *pn;
			pn++;
		}
		pF->sumXX = sqrt(double(sumXX));
		featureList.push_back(pF);
		featureMap[int(pF->y) * iW + int(pF->x)] = pF;

		pCorner++;
		featureCount++;
	}
	F_listsize = featureList.size();
	cvReleaseImage(&gray);
	cvReleaseImage(&eigImage);
	cvReleaseImage(&tempImage);
	free(corners);////
}

void CornerFinder::initialize(int iW, int iH)
{
	featureMap = (Feature**)malloc(iW * iH * sizeof(Feature*));
}

float compare(Feature* a, Feature* b)
{
	float ab = a->meanRGB.val[0], ag = a->meanRGB.val[1], ar = a->meanRGB.val[2], bb = b->meanRGB.val[0], bg = b->meanRGB.val[1], br = b->meanRGB.val[2];
	float BR, GR, BG;
	if (ab > 20 && ag > 20 && ar > 20 && bb > 20 && bg > 20 && br > 20)
	{
		BR = ab * br / ar / bb;
		GR = ag * br / ar / bg;
		BG = ab * bg / ag / bb;
		if (BR < 0.8 || BR > 1.2 || GR < 0.8 || GR > 1.2 || BG < 0.8 || BG > 1.2)
			return 0.1;
	}

	short* pa = a->patch->data.s;
	short* pb = b->patch->data.s;

	long Sab = 0;
	for (int i = 0; i < l_feature * l_feature * 3; i++)
	{
		Sab += *pa * *pb;
		pa++;
		pb++;
	}
	float nzc = Sab / a->sumXX / b->sumXX;
	return nzc;
}

Feature** CornerFinder::getFeatureMap()
{
	return featureMap;
}