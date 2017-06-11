#include "clude.h"
#include "Object.h"


ObjectList::ObjectList()
{

}

ObjectList::~ObjectList()
{
	Objvec.clear();
}


myObject::~myObject()
{
	cvReleaseMat(&current_map);
	cvReleaseMat(&current_weight);
	if (map != NULL)
		cvReleaseMat(&map);
	cvReleaseMat(&weight);

	for (int i = 0; i < (int)featureList.size(); i++)
	{
		//	cvReleaseMat(&featureList[i]->patch);
		delete featureList[i];
	}
	featureList.clear();

	if (trajectory != NULL)
	{
		delete trajectory;
	}
	if (stateSeq != NULL)
	{
		delete stateSeq;
	}
	if (curwhSeq != NULL)
	{
		delete curwhSeq;
	}

}

void myObject::deObject(void)
{
	cvReleaseMatHeader(&current_map);
	cvReleaseMatHeader(&current_weight);
	cvReleaseMat(&map);
	cvReleaseMat(&weight);
	for (int i = 0; i < (int)featureList.size(); i++)
	{
		cvReleaseMat(&featureList[i]->patch);
		delete featureList[i];
	}
	featureList.clear();
	/*
	if(trajectory!=NULL)
	{
	trajectory->clear();
	delete trajectory;
	}
	if(stateSeq!=NULL)
	{
	stateSeq->clear();
	delete stateSeq;
	}
	if(curwhSeq!=NULL)
	{
	curwhSeq->clear();
	delete curwhSeq;
	}*/
}
