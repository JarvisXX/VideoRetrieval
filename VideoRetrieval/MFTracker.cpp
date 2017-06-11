//跟踪算法模块

//毕竟是基于自己设想的算法尝试实现的，其中带“×”的注释 表示此部分只是尝试性的实现方法或策略，不一定会带来稳定的效果，对某些实验情况可能不利。
//算法参考 两篇paper
//调用请参考Monitor.cpp
#include "clude.h"
#include "MFTracker.h"
#include "math.h"
#include "iostream"
#include "stdlib.h"
#include "CornerFinder.h"

extern 	CvScalar* FakeRGB;
extern int checkID;
int fps=25;
vector<closePair> vPair;

void addPair(int ID1, int ID2)
{
	if (ID1 < 0 || ID2 < 0) return;
	for (int i = 0; i < vPair.size(); i++)
	{
		if (ID1 == vPair[i].ID1 && ID2 == vPair[i].ID2)
		{
			vPair[i].count += 1;
			return;
		}
	}
	closePair cp;
	cp.ID1 = ID1;
	cp.ID2 = ID2;
	cp.count = 0;
	vPair.push_back(cp);
	return;
}


MFTracker::MFTracker()
	: testnum(0)
	, testbnum(0)
{
	trackLabel = NULL;///////////////////////////////////////////////////////////////////////////////////////////////
	frame = NULL;////////////////////////////////////////////////////////////////////////////////////////////////////
	FGmask = NULL;///////////////////////////////////////////////////////////////
	BG = NULL;
	newBlobList = NULL;
}


MFTracker::~MFTracker(void)
{
	if (trackLabel != NULL)//////////////////////////////////////////////////////////////////////////////////////////////////////
		cvReleaseImage(&trackLabel);
	std::vector   <myObject*> ::iterator   iterbtn;
	for (iterbtn = trackList.begin(); iterbtn != trackList.end(); iterbtn++)
	{

		delete   *iterbtn;
	}
	trackList.clear();
	//newBlobList->clear();
	for (int i = 0; i < int(matchGroups.size()); i++)
	{
		delete matchGroups[i];
	}
	matchGroups.clear();

}

void MFTracker::initialize(int w, int h)
{

	//////////////////////////////////////////
	nextID = 1;
	nextnewID = -1;
	iW = w;
	iH = h;
	trackLabel = cvCreateImage(cvSize(iW, iH), IPL_DEPTH_8U, 1);
	cvSetZero(trackLabel);
	flagNewing = 0;
	testnum = 0;
	testbnum = 0;
}
/////////////////////           当前帧         当前帧的前景掩模     NULL               前景块序列                     前景特征点  
void MFTracker::track(IplImage* framein, IplImage* FGmaskin, IplImage* BGin, vector<myBlob>* newBlobListin, Feature** featureMapin)
{

	IplImage* gray = cvCreateImage(cvSize(iW, iH), IPL_DEPTH_8U, 1);//  创建处理图像一样大小的灰度图
	frame = framein;
	FGmask = FGmaskin;
	BG = BGin;

	newBlobList = newBlobListin;    //
	featureMap = featureMapin;      //特征块
	w = frame->width;               //图像的宽度
	h = frame->height;              //图像的长度
	int blobnum, tracknum;
	tracknum = trackList.size();    //获取跟踪序列中跟踪块的个数
	blobnum = newBlobList->size();  //前景块的个数
	testnum = trackList.size();       //跟踪块的个数 即前面的帧中产生的跟踪目标块
	testbnum = newBlobList->size();
	trackclose = NULL;
	one2oneBestMatch = NULL;
	blobclose = NULL;
	distanceMatrix = NULL;
	closeMatrix = NULL;

	if (tracknum > 0)
	{
		trackclose = (int*)calloc(tracknum, sizeof(int));         //整型数组trackclose   存储与该模块相近的模块的个数
		one2oneBestMatch = (int*)calloc(tracknum, sizeof(int));   //整型数组one2oneBestMatch
	}
	if (blobnum > 0)
	{
		blobclose = (int*)calloc(blobnum, sizeof(int));            //前景块对应的整型数组
	}
	if (blobnum > 0 && tracknum > 0)		//目的将前景块与track列表进行一一匹配
	{
		distanceMatrix = (float*)calloc(tracknum * blobnum, sizeof(float));     //生成距离矩阵    ,用于判断两个块之间的关系，是否是同一个物体用的
		closeMatrix = (int*)calloc(tracknum * blobnum, sizeof(int));
	}
	cvZero(trackLabel);        //对图像数据清0
	calBoundingBoxDistance();  //计算所有目标和前景块之间的Bounding Box Distance       产生距离矩阵distanceMatriX 和closeMatrx
	makeMatchGroups();         //分组目标和前景块
	//main track procedure
	groupMatch();////////////////组间的匹配/
	conclude();                //跟踪结果分析函数


	if (tracknum > 0)
	{
		free(trackclose);
		free(one2oneBestMatch);
	}
	if (blobnum > 0)
	{
		free(blobclose);
	}
	if (blobnum > 0 && tracknum > 0)
	{
		free(distanceMatrix);
		free(closeMatrix);
	}
	if (tracknum > 0)
	{
		delete[] one2groupBestConfig;
		free(one2groupBestScore);
	}
	for (int i = 0; i < int(matchGroups.size()); i++)
	{
		delete matchGroups[i];
	}
	matchGroups.clear();
	testnum = trackList.size();
	testbnum = newBlobList->size();

	cvReleaseImage(&gray);
}

void MFTracker::calBoundingBoxDistance()          //产生前景块到跟踪块的距离矩阵，和最近块标识矩阵
{
	int i, j, blobnum, tracknum;
	tracknum = trackList.size();
	blobnum = newBlobList->size();
	for (i = 0; i < tracknum; i++)   //遍历前一次的跟踪模块
	{
		CvRect rect1 = trackList[i]->blob.outRect;            //获取跟踪模块的外矩框
		int rangeX = (int)trackList[i]->searchRangeX;         //获取该模块的搜索范围
		int rangeY = (int)trackList[i]->searchRangeY;
		//rect1为搜索矩形范围，由下面4行代码确定	
		rect1.x = rect1.x - rangeX;
		rect1.y = rect1.y - rangeY;
		rect1.width = rect1.width + rangeX * 2;
		rect1.height = rect1.height + rangeY * 2;
		for (j = 0; j < blobnum; j++)       //遍历一次当前前景模块
		{
			CvRect& rect2 = (*newBlobList)[j].outRect;         //获取该前景块的矩形框
			float d1 = 0, d2 = 0;
			int dx, dy;        //dx,dy分别为两矩形框在想x和y上的最近距离
			dx = rect1.x > rect2.x + rect2.width ? (rect1.x - rect2.x - rect2.width) : 0;
			dx = rect1.x + rect1.width < rect2.x ? (rect2.x - rect1.x - rect1.width) : dx;
			dy = rect1.y > rect2.y + rect2.height ? (rect1.y - rect2.y - rect2.height) : 0;
			dy = rect2.y > rect1.y + rect1.height ? (rect2.y - rect1.y - rect1.height) : dy;   //求取两矩形框距离
			d1 = (dx != 0 || dy != 0) ? sqrt(float(dx * dx + dy * dy)) : 0;
			distanceMatrix[i * blobnum + j] = d1;
		}
	}
	//sum close target
	for (i = 0; i < tracknum; i++)
	{
		for (j = 0; j < blobnum; j++)
		{
			if (distanceMatrix[i * blobnum + j] <= 0)//  trackList[i]->blob.outRect.width/2)// 0)//if(distanceMatrix[i * blobnum + j] <= (trackList[i]->blob.outRect.width+trackList[i]->blob.outRect.height)/2/3)
			{
				closeMatrix[i * blobnum + j] = 1;   //接近时，对应矩阵为1否则为0
				trackclose[i]++;                    //对于第i个跟踪块与其接近的前景块个数计数
				blobclose[j]++;                     //对于第j个前景块与其接近的背景块的个数计数
			}
			else
			{
				closeMatrix[i * blobnum + j] = 0;    //若对应的前景，跟踪块不相邻则标识为0

			}
		}
	}
}

float MFTracker::getBoundingBoxDistance(int cx, int cy, int bleft, int bright, int bbottom, int btop)
{
	int dx, dy;
	dx = cx < bleft ? (bleft - cx) : 0;
	dx = cx > bright ? (cx - bright) : dx;
	dy = cy < bbottom ? (bbottom - cy) : 0;
	dy = cy > btop ? (cy - btop) : dy;
	return (dx != 0 || dy != 0) ? sqrt(float(dx * dx + dy * dy)) : 0;
}

void MFTracker::makeMatchGroups()
{
	int i, j, blobnum, tracknum;
	tracknum = trackList.size();
	blobnum = newBlobList->size();
	one2groupBestConfig = NULL;
	one2groupBestScore = NULL;
	for (j = 0; j < blobnum; j++)   //遍历前景块 对有相邻跟踪模块的前景模块生成匹配组
	{
		if (blobclose[j]>0)      //与前景块相近的个数大于0
		{
			MatchGroup* newGroup = new MatchGroup();     //创建新的匹配对象
			newGroup->blobIndex.push_back(j);
			for (i = 0; i < tracknum; i++)
			{
				if (closeMatrix[i * blobnum + j] == 1)
				{
					newGroup->objIndex.push_back(i);
				}
			}
			matchGroups.push_back(newGroup);             //将生成的匹配对象保存到matchgroups里  vector中加入对象的方法
		}

	}
	for (i = 0; i < int(matchGroups.size()) - 1; i++)       //合并具有相同跟踪模块邻域的匹配组
	{
		for (j = i + 1; j < int(matchGroups.size()); j++)
		{
			if (matchGroups[i]->compare(matchGroups[j]) == 1)
			{
				matchGroups[i]->unite(matchGroups[j]);
				delete matchGroups[j];                   //在vector中如何删除一个对象
				matchGroups.erase(matchGroups.begin() + j);
				j--;
			}
		}
	}
	int groupnum = matchGroups.size();

	if (tracknum > 0)
	{

		one2groupBestConfig = new SampleConfig[tracknum];
		one2groupBestScore = (float*)calloc(tracknum, sizeof(float));
	}

}

void MFTracker::groupMatch()
{
	int i, j, k, tracknum;
	tracknum = trackList.size();
	int groupnum = matchGroups.size();
	for (i = 0; i < groupnum; i++)      //遍历已经生成的组对象
	{
		//get group component
		MatchGroup* group = matchGroups[i];
		int tracknumInGroup = group->objIndex.size();           //组中跟踪块的个数
		int* trackIndex = new int[tracknumInGroup]; for (j = 0; j<tracknumInGroup; j++){ trackIndex[j] = group->objIndex[j]; one2oneBestMatch[trackIndex[j]] = -1; }
		int blobnumInGroup = group->blobIndex.size();      //组中前景块的个数
		int* blobIndex = new int[blobnumInGroup]; for (j = 0; j<blobnumInGroup; j++)blobIndex[j] = group->blobIndex[j];
		int* blobID = new int[blobnumInGroup]; for (j = 0; j<blobnumInGroup; j++)blobID[j] = (*newBlobList)[blobIndex[j]].ID;

		//make rect blob for each object  根据组中所有的前景块，找到生成一个全部包括的矩形范围comBlob
		int minX = ((*newBlobList)[blobIndex[0]].outRect.x);
		int maxX = ((*newBlobList)[blobIndex[0]].outRect.x + (*newBlobList)[blobIndex[0]].outRect.width);
		int minY = ((*newBlobList)[blobIndex[0]].outRect.y);
		int maxY = ((*newBlobList)[blobIndex[0]].outRect.y + (*newBlobList)[blobIndex[0]].outRect.height);
		for (j = 1; j < blobnumInGroup; j++)
		{

			minX = minX < ((*newBlobList)[blobIndex[j]].outRect.x) ? minX : ((*newBlobList)[blobIndex[j]].outRect.x);
			maxX = maxX >((*newBlobList)[blobIndex[j]].outRect.x + (*newBlobList)[blobIndex[j]].outRect.width) ? maxX : ((*newBlobList)[blobIndex[j]].outRect.x + (*newBlobList)[blobIndex[j]].outRect.width);
			minY = minY < ((*newBlobList)[blobIndex[j]].outRect.y) ? minY : ((*newBlobList)[blobIndex[j]].outRect.y);
			maxY = maxY >((*newBlobList)[blobIndex[j]].outRect.y + (*newBlobList)[blobIndex[j]].outRect.height) ? maxY : ((*newBlobList)[blobIndex[j]].outRect.y + (*newBlobList)[blobIndex[j]].outRect.height);
		}
		CvRect comBlob;
		comBlob.x = minX;
		comBlob.y = minY;
		comBlob.width = maxX - minX + 1;
		comBlob.height = maxY - minY + 1;

		if (blobnumInGroup == 1 && tracknumInGroup == 1)
			one2oneBestMatch[trackIndex[0]] = blobIndex[0];    //生成最佳匹配
		do
		{

			for (j = 0; j<tracknumInGroup; j++)
			{
				//do correlation to search best	//对每一个目标 track在一定范围comBlob内进行最优位置的搜索
				doSearch(trackList[trackIndex[j]], comBlob, &one2groupBestScore[trackIndex[j]], &one2groupBestConfig[trackIndex[j]]);
			}
			if (tracknumInGroup > 0)
			{
				//depth evaluation
				int* goodIndex = new int[tracknumInGroup];   //较好的匹配
				int* Depth = new int[tracknumInGroup];
				float* occludedW = new float[tracknumInGroup];
				float* w_sum = new float[tracknumInGroup];
				//对前景点进行标记
				labelPixel(comBlob, tracknumInGroup, trackIndex, goodIndex, Depth, occludedW, w_sum);//also need trackLabel
				for (j = 0; j < tracknumInGroup; j++)
				{
					if (goodIndex[j] == 1)
					{
						for (k = j; k < tracknumInGroup - 1; k++)
						{
							goodIndex[k] = goodIndex[k + 1];
							trackIndex[k] = trackIndex[k + 1];
						}
						tracknumInGroup--;
						j--;
					}
				}
				delete[] goodIndex;
				delete[] Depth;
				delete[] occludedW;
				delete[] w_sum;
			}
			//repeat or end
		} while (tracknumInGroup > 0);

		delete[] trackIndex;
		delete[] blobIndex;
		delete[] blobID;

	}
}

void MFTracker::labelPixel(CvRect blob, int tracknumInGroup, int* trackIndex,
	/*output:*/ int* goodIndex, int* Depth, float* occludedW, float* w_sum)//also need trackLabel
{
	int i, j, k;
	uchar* pLabel;
	int steplabel = trackLabel->widthStep;

	uchar* pImage;
	uchar* pFGmask;
	int stepI = frame->widthStep;
	int stepFG = FGmask->widthStep;

	CvMat** likeB = (CvMat**)malloc(tracknumInGroup * sizeof(CvMat*));
	CvMat** wB = (CvMat**)malloc(tracknumInGroup * sizeof(CvMat*));
	uchar** pMap = (uchar**)malloc(tracknumInGroup * sizeof(uchar*));
	int* stepM = new int[tracknumInGroup];
	float** pW = (float**)malloc(tracknumInGroup * sizeof(float*));
	int* stepW = new int[tracknumInGroup];

	uchar* pmap;
	float* pw;

	float* contest = (float*)calloc(tracknumInGroup * tracknumInGroup, sizeof(float));
	float* contestWin = (float*)calloc(tracknumInGroup * tracknumInGroup, sizeof(float));
	double PixelSum = 0;
	double PixelExplained = 0;

	int steplikeB = blob.width;
	int stepwB = blob.width;

	//prepare
	for (i = 0; i < tracknumInGroup; i++)
	{
		myObject* track = trackList[trackIndex[i]];
		pMap[i] = track->current_map->data.ptr;
		stepM[i] = track->current_map->step;
		pW[i] = track->current_weight->data.fl;
		stepW[i] = track->current_weight->step / sizeof(float);
		likeB[i] = cvCreateMat(blob.height, blob.width, CV_32SC1);
		wB[i] = cvCreateMat(blob.height, blob.width, CV_32FC1);
		cvZero(likeB[i]);
		cvZero(wB[i]);
		w_sum[i] = 0;
	}

	const int TupC = 130, TlowC = 30;

	//process like and w
	for (i = 0; i < tracknumInGroup; i++)
	{
		myObject* track = trackList[trackIndex[i]];
		SampleConfig bestC = one2groupBestConfig[trackIndex[i]];
		int xStart = max((float)bestC.c.x + track->cur_x, (float)blob.x);
		int yStart = max((float)bestC.c.y + track->cur_y, (float)blob.y);
		int xEnd = min((float)bestC.c.x + track->cur_x + track->cur_w, (float)blob.x + (float)blob.width);
		int yEnd = min((float)bestC.c.y + track->cur_y + track->cur_h, (float)blob.y + (float)blob.height);
		int* pLikeB;
		float* pWB;
		for (int y = yStart; y < yEnd; y++)
		{
			pLabel = (unsigned char*)trackLabel->imageData + y * steplabel + xStart;
			pFGmask = (unsigned char*)FGmask->imageData + y * stepFG + xStart;
			pImage = (unsigned char*)frame->imageData + y * stepI + xStart * 3;

			int rx = xStart - (bestC.c.x + track->cur_x);
			int ry = y - (bestC.c.y + track->cur_y);
			pmap = (uchar*)pMap[i] + ry * stepM[i] + rx * 3;
			pw = pW[i] + ry * stepW[i] + rx;

			pLikeB = likeB[i]->data.i + (y - blob.y) * steplikeB + (xStart - blob.x);
			pWB = wB[i]->data.fl + (y - blob.y) * stepwB + (xStart - blob.x);
			for (int x = xStart; x< xEnd; x++)
			{
				if (*pLabel == 0 && *pFGmask != 0)
				{
					*pWB = *pw;
					int difference = max(abs(int(*(pImage)) - *(pmap)),
						max(abs(int(*(pImage + 1)) - *(pmap + 1)),
						abs(int(*(pImage + 2)) - *(pmap + 2))));
					*pLikeB = 255 - difference + 1; //reserve 0 for unprocessed
				}
				pLabel++;
				pFGmask++;
				pImage += 3;
				pmap += 3;
				pw++;
				pLikeB++;
				pWB++;
			}
		}
	}

	int likeBplace, wBplace, iBuf;
	float fBuf;

	//根据like[] 做分析
	for (int y = blob.y; y < blob.y + blob.height; y++)
	{
		pLabel = (unsigned char*)trackLabel->imageData + y * steplabel + blob.x;
		pFGmask = (unsigned char*)FGmask->imageData + y * stepFG + blob.x;

		likeBplace = (y - blob.y) * steplikeB;
		wBplace = (y - blob.y) * stepwB;
		for (int x = blob.x; x < blob.x + blob.width; x++)
		{
			if (*pFGmask != 0)
			{
				PixelSum++;
				if (*pLabel == 0)
				{
					int bestK = -1;
					int bestLike = -1;
					for (k = 0; k < tracknumInGroup; k++)
					{
						iBuf = *(likeB[k]->data.i + likeBplace);
						if (iBuf != 0)//&& *(wB[k]->data.fl + wBplace) != 0)
						{
							if (iBuf > bestLike)
							{
								bestLike = iBuf;
								bestK = k;
							}
							w_sum[k] += *(wB[k]->data.fl + wBplace);
						}
					}
					if (bestK >= 0)
					{
						for (k = 0; k < tracknumInGroup; k++)
						{
							if (k == bestK || *(likeB[k]->data.i + likeBplace) == 0)
							{
								continue;
							}
							else
							{
								contest[k * tracknumInGroup + bestK] += *(wB[k]->data.fl + wBplace);
								contest[bestK * tracknumInGroup + k] += (fBuf = *(wB[bestK]->data.fl + wBplace));
								contestWin[bestK * tracknumInGroup + k] += fBuf;
							}
						}
						PixelExplained++;
					}
				}
				else
				{
					PixelExplained++;
				}
			}
			pLabel++;
			pFGmask++;
			likeBplace++;
			wBplace++;
		}
	}
	//定深度
	int* flagProcessed = new int[tracknumInGroup];
	for (i = 0; i < tracknumInGroup; i++)
	{
		flagProcessed[i] = 0;
		occludedW[i] = 0;
		Depth[i] = -1;
		goodIndex[i] = 0;
	}


	for (i = 0; i < tracknumInGroup; i++)
	{
		int* SumContest = new int[tracknumInGroup];
		int* SumContestWin = new int[tracknumInGroup];
		for (j = 0; j < tracknumInGroup; j++)
		{
			if (flagProcessed[j] == 0)
			{
				SumContest[j] = 0;
				SumContestWin[j] = 0;
				for (k = 0; k < tracknumInGroup; k++)
				{
					SumContest[j] += contest[j * tracknumInGroup + k];
					SumContestWin[j] += contestWin[j * tracknumInGroup + k];
				}
			}
		}
		int first = -1;
		double firstWinPor = -1;
		float winPor;
		for (j = 0; j<tracknumInGroup; j++)
		{
			if (flagProcessed[j] == 0)
			{
				winPor = SumContest[j] > 0 ? float(SumContestWin[j]) / SumContest[j] : 1;
				if (firstWinPor < winPor)
				{
					firstWinPor = winPor;
					first = j;
				}
			}
		}
		Depth[i] = first;
		flagProcessed[first] = 1;
		for (j = 0; j<tracknumInGroup; j++)
		{
			if (flagProcessed[j] == 0)						//new change
			{
				occludedW[j] += contest[j * tracknumInGroup + first];
			}
		}

		delete[] SumContest;
		delete[] SumContestWin;

	}//如果explainrate 高 则 无50%以上损失者 可不必迭代
	if (PixelExplained / PixelSum > 0.85)
	{
		goodIndex[Depth[0]] = 1;
		for (i = 1; i < tracknumInGroup; i++)
		{
			//if(occludedW[Depth[i]] / w_sum[Depth[i]] < 0.1)
			{
				//	goodIndex[Depth[i]] = 1;
			}
			//else
			{
				break;
			}
		}
	}
	else
	{
		goodIndex[Depth[0]] = 1;
	}


	//Label
	for (i = 0; i < tracknumInGroup; i++)
	{
		if (goodIndex[Depth[i]] == 1)
		{
			myObject* track = trackList[trackIndex[Depth[i]]];
			SampleConfig bestC = one2groupBestConfig[trackIndex[Depth[i]]];
			int xStart = max(bestC.c.x + track->cur_x, (float)blob.x);
			int yStart = max(bestC.c.y + track->cur_y, (float)blob.y);
			int xEnd = min(bestC.c.x + track->cur_x + track->cur_w, (float)blob.x + (float)blob.width);
			int yEnd = min(bestC.c.y + track->cur_y + track->cur_h, (float)blob.y + (float)blob.height);

			float* pWB;
			int* pLikeB;

			for (int y = yStart; y < yEnd; y++)
			{
				//pFGlabel = FGlabel->data.i + y * stepFGlabel + xStart;
				pLabel = (unsigned char*)trackLabel->imageData + y * steplabel + xStart;
				pFGmask = (unsigned char*)FGmask->imageData + y * stepFG + xStart;

				pWB = wB[Depth[i]]->data.fl + (y - blob.y) * stepwB + (xStart - blob.x);
				pLikeB = likeB[Depth[i]]->data.i + (y - blob.y) * steplikeB + (xStart - blob.x);
				for (int x = xStart; x< xEnd; x++)
				{
					if (*pLabel == 0 && *pFGmask != 0)
					{
						if (*pWB > 0.2 && *pLikeB > 200)
						{
							*pLabel = trackIndex[Depth[i]] + 1;
						}
					}
					else if (*pLabel == 0 && *pWB > 0.75 && *pLikeB > 240)
					{
						*pLabel = trackIndex[Depth[i]] + 1;
					}
					pLabel++;
					pFGmask++;
					pWB++;
					pLikeB++;
				}
			}
		}
		else
		{
			break;
		}
	}

	//release memory
	for (i = 0; i < tracknumInGroup; i++)
	{
		cvReleaseMat(&likeB[i]);
		cvReleaseMat(&wB[i]);
	}
	/*	delete[] likeB;
	delete[] wB;
	delete[] pMap;
	delete[] stepM;
	delete[] pW;
	delete[] stepW;
	*/
	free(likeB);
	free(wB);
	free(pMap);
	free(stepM);
	free(pW);
	free(stepW);

	delete[] flagProcessed;

	return;
}

void MFTracker::doSearch(myObject* track, CvRect comBlob, float* bestScore, SampleConfig* bestConfig)
{
	//make search range for startxy
	//-just use all foreground point for center match

	*bestScore = 0;
	float score;
	int FGplace = 0;
	int step_FGplace = FGmask->widthStep;
	uchar* pFG = (uchar*)FGmask->imageData;
	uchar* pTrackLabel;

	//确定搜索框
	int rangeX = (int)track->searchRangeX;
	int rangeY = (int)track->searchRangeY;
	int hw = track->blob.outRect.width / 2;
	int x_start = max((float)comBlob.x, track->blob.outRect.x - track->searchRangeX - hw);
	int y_start = max((float)comBlob.y, track->blob.outRect.y - track->searchRangeY - hw);
	int x_end = min((float)comBlob.x + comBlob.width, track->blob.outRect.x + track->blob.outRect.width + track->searchRangeX + hw);
	int y_end = min((float)comBlob.y + comBlob.height, track->blob.outRect.y + track->blob.outRect.height + track->searchRangeY + hw);
	//防止超出边界
	x_start = max(x_start, 0);
	y_start = max(y_start, 0);
	x_end = min(x_end, iW - 1);
	y_end = min(y_end, iH - 1);


	x_start = min(x_start, track->cx);
	y_start = min(y_start, track->cy);
	x_end = max(x_end, track->cx);
	y_end = max(y_end, track->cy);

	vector<Feature*>* pFeatures = &track->featureList;
	Feature** pF;
	int size = pFeatures->size();

	int rsw = x_end - x_start + 1;
	int rsh = y_end - y_start + 1;
	int rsx, rsy;

	CvMat *Samples = cvCreateMat(rsh, rsw, CV_32FC1);   //开辟一个和搜索范围同样大小的矩阵，存储在搜索范围内特征点和codebook元素匹配的次数
	cvSetZero(Samples);
	float* samplePoint = Samples->data.fl;

	*(samplePoint + (track->cy - y_start) * rsw + (track->cx - x_start)) = 20;     //认为矩阵中心与codebook元素匹配的次数为20，表示中心点权重很大

	for (int y = y_start; y < y_end; y++)
	{
		rsy = y - y_start;
		pF = featureMap + y * iW + x_start;
		pTrackLabel = (uchar*)trackLabel->imageData + y * step_FGplace + x_start;
		for (int x = x_start; x < x_end; x++)
		{
			rsx = x - x_start;
			if (*pF == NULL || *pTrackLabel != 0)     //若搜索范围内的点不是特征点或者像素已经被标记，则结束本轮循环
			{
				pF++;
				pTrackLabel++;
				continue;
			}
			Feature* feature = *pF;
			for (int k = 0; k < size; k++)              //对于搜索范围内每一个特征点，计算和目标物模板的codebook匹配的次数
			{
				int sx = (int)(rsx - (*pFeatures)[k]->rx);   //计算特征点位置和codebook中的特征点相对位置之差，即为根据特征点算出的目标
				if (sx < 0 || sx >= rsw) continue;
				int sy = (int)(rsy - (*pFeatures)[k]->ry);   //点位置，注意从图像坐标到Samples矩阵的坐标转换，匹配的特征点次数存储在什么位置
				if (sy < 0 || sy >= rsh) continue;
				float* pS = samplePoint + sy * rsw + sx;
				float matchScore = compare(feature, (*pFeatures)[k]);
				if (matchScore > 0.9)// && (*pFeatures)[k]->health > 25)    //若相似度大于0.9，则认为匹配上，匹配次数加1
				{
					(*pS)++;//= max(min((*pFeatures)[k]->health - 30,30),0);
				}
			}
			pF++;
			pTrackLabel++;
		}
	}
	//对Samples矩阵进行平滑处理
	int smooths = max(2.0, sqrt(track->cur_w * track->cur_h * 0.05));
	CvMat* SmoothedSamples = cvCreateMat(rsh, rsw, CV_32FC1);
	cvZero(SmoothedSamples);
	float* pS = Samples->data.fl;
	float* ss = SmoothedSamples->data.fl;
	for (int i = 0; i < rsh; i++)
	{
		for (int j = 0; j < rsw; j++)
		{
			if (pS[i * rsw + j] > 0)
			{
				for (int k = max(i - smooths, 0); k < min(i + smooths, rsh); k++)
				{
					for (int l = max(j - smooths, 0); l < min(j + smooths, rsw); l++)
					{
						ss[i * rsw + j] += pS[k * rsw + l];
					}
				}
			}
		}
	}
	double minS, maxS;
	cvMinMaxLoc(SmoothedSamples, &minS, &maxS);
	for (int i = 0; i < rsh * rsw; i++)
	{
		//排除掉一些概率较小的点
		if (*pS > 0 && *(ss) > min(maxS * 0.75, maxS - 2))
		{
			*(pS) = *(ss) < 255 ? *(ss) : 255;
		}
		else
		{
			*(pS) = 0;
		}
		pS++;
		ss++;
	}

	int stepy, stepx;
	int cy = comBlob.y + comBlob.height / 2;
	int cx = comBlob.x + comBlob.width / 2;
	for (int i = y_start; i < y_end; i++)//=stepy)
	{
		rsy = i - y_start;
		for (int j = x_start; j < x_end; j++)//=stepx)
		{
			rsx = j - x_start;
			if (samplePoint[rsy * rsw + rsx] > 0 || abs(i - track->cy) + abs(j - track->cx) == 3 || abs(i - track->cy) + abs(j - track->cx) == 6)
			{
				SampleConfig sample;
				sample.c = cvPoint(j, i);
				sample.f = 1;
				score = getLikelihood(track, sample);    //计算目标物和以采样点为中心同样大小区域的相似度
				if (score > *bestScore)
				{
					*bestScore = score;
					*bestConfig = sample;
				}

			}
		}
	}

	cvReleaseMat(&Samples);
	cvReleaseMat(&SmoothedSamples);
}


float MFTracker::getLikelihood(myObject* track, SampleConfig sample)
{
	int cx = sample.c.x; int cy = sample.c.y;
	float f = sample.f;
	//data process
	unsigned char* pImage = (unsigned char*)(frame->imageData);
	unsigned char* pFGmask = (unsigned char*)FGmask->imageData;
	unsigned char* pLabel = (unsigned char*)trackLabel->imageData;
	int widthStep3c = frame->widthStep;
	int widthStep = FGmask->widthStep;
	unsigned char* pMap = track->current_map->data.ptr;

	float* pWeight = track->current_weight->data.fl;
	int rows = track->current_map->rows;
	int cols = track->current_map->cols;
	int step3c = track->current_map->step;
	int step = track->current_weight->step / sizeof(float);

	int i, j;
	int TupL = 80, TlowL = 30;
	int TupC = 80, TlowC = 30;
	int weightPlace, mapPlace, FGPlace, imgPlace;
	double sum_w = 0, sum_difference = 0, w;
	int dL, dC, difference;
	int y_start = cy + track->cur_y;
	int x_start = cx + track->cur_x;


	//////////////////////////////////////////////////////////////////////////
#if 0
	CvMat* showW = cvCreateMat(rows, cols, CV_8UC1);
	CvMat* showMatch = cvCreateMat(rows, cols, CV_8UC1);
	uchar* pshowW = showW->data.ptr;
	uchar* pshowMatch = showMatch->data.ptr;
#endif
	//////////////////////////////////////////////////////////////////////////
	for (i = 0; i < rows; i++)
	{
		if (y_start + i < 0 || y_start + i >= frame->height)
		{
			weightPlace = i * step;
			for (j = 0; j < cols; j++)
			{
				w = *(pWeight + weightPlace);

				//*(pshowW+i*cols+j) = w * 255;
				//*(pshowMatch+i*cols+j) = 0;

				sum_difference += w * 50;
				sum_w += w;
				weightPlace++;
			}
		}
		else
		{
			weightPlace = i * step; mapPlace = i * step3c;
			FGPlace = (y_start + i) * widthStep + x_start;
			imgPlace = (y_start + i) * widthStep3c + x_start * 3;
			for (j = 0; j < cols; j++)
			{
				if (x_start + j < 0 || x_start + j >= frame->width)
				{
					w = *(pWeight + weightPlace);

					//*(pshowW+i*cols+j) = w * 255;
					//*(pshowMatch +i*cols+j) = 0;
					w = w * 0.5;
					difference = 100;
				}
				else
				{
					w = *(pWeight + weightPlace);
					//ASSERT(w >= 0);
					if (*(pFGmask + FGPlace) > 0 && *(pLabel + FGPlace) == 0)
					{
						difference = max(abs(int(*(pImage + imgPlace)) - *(pMap + mapPlace)),
							max(abs(int(*(pImage + imgPlace + 1)) - *(pMap + mapPlace + 1)),
							abs(int(*(pImage + imgPlace + 2)) - *(pMap + mapPlace + 2))));
						/*dL = (int(*(pImage+imgPlace) + *(pImage+imgPlace+1) + *(pImage+imgPlace+2))
						- *(pMap+mapPlace) - *(pMap+mapPlace+1) - *(pMap+mapPlace+2))/3;

						difference = (difference - dL);*/
						difference = min(max((difference - TlowL), 0), 50) * 2;
						//difference = min(max(difference,0),1);
					}
					else
					{
						w = w * 0.5;
						difference = 100;
						/*difference = max(abs(int(*(pImage+imgPlace)) - *(pMap+mapPlace)),
						max(abs(int(*(pImage+imgPlace+1)) - *(pMap+mapPlace+1)),
						abs(int(*(pImage+imgPlace+2)) - *(pMap+mapPlace+2))));
						difference = min(max((difference - TlowL),0),50) * 2;*/
					}
					//*(pshowW+i*cols+j) = w * 255;
					//*(pshowMatch+i*cols+j) = (1-difference) * 255;
				}
				sum_difference += w * difference;
				sum_w += w;
				weightPlace++; FGPlace++;
				mapPlace += 3; imgPlace += 3;
			}
		}
	}

	return 1 - sum_difference / sum_w / 100.0;
}

void MFTracker::limit(int x1, int y1, int x2, int y2, CvSize S)
{
	x1 = x1<0 ? 0 : x1;
	y1 = y1<0 ? 0 : y1;
	x2 = (x2 >= S.width) ? (S.width - 1) : x2;
	y2 = (y2 >= S.height) ? (S.height - 1) : y2;
}

void MFTracker::conclude()
{
	int i, j, blobnum, tracknum;
	tracknum = trackList.size();
	blobnum = newBlobList->size();
	//update
	flagNewing = 0;
	for (i = 0; i < tracknum; i++)
	{
		myObject* track = trackList[i];
		float simT = 0.3;//track->health > 10?0.5:0.3 + 0.2 * track->health / 10;
		int flagFind = one2groupBestScore[i] > simT ? 1 : 0;	//daiding
		if (track->state == obj_new && one2oneBestMatch[i] == -1)
		{
			flagFind = 0;
		}
		int ifNearBorder;
		if (track->cx < 20 || track->cx > iW - 20 || track->cy < 20 || track->cy > iH - 20)//|| track->cx * 0.87 - track->cy < - 100)
			ifNearBorder = 1;
		else
			ifNearBorder = 0;

		//calculate health
		if (trackclose[i] == 0)
		{
			track->health -= (2 + ifNearBorder);
		}
		else if (flagFind == 0)
		{
			track->health -= (1 + ifNearBorder);
		}
		else
		{
			track->health = track->health < 40 ? track->health + 1 : 40;
		}
		//state update
		switch (track->state)
		{
		case obj_new:
			flagNewing--;
			//match to disappeared ones

			if (track->health > 10)// && )
			{

				{
					if (track->ID == checkID)
						checkID = nextID;
					track->ID = nextID;
					nextID++;
					flagNewing = 0;
					track->state = obj_tracked;
				}

			}

			break;
		case obj_reappeare:
			if (track->health > 10)
			{
				track->ID = nextID;
				nextID++;
				track->state = obj_tracked;
			}
			break;
		case obj_tracked:
			if (flagFind == 0)
			{
				track->state = obj_missing;
			}
			break;
		case obj_missing:
			if (flagFind == 1)
			{
				track->state = obj_tracked;
				break;
			}
			else if (track->health <= 0)
			{
				track->state = obj_disappear;
				track->health = 0;
			}
			break;
		case obj_disappear:
		default:
			break;
		}

		string strState;
		switch (track->state)
		{
		case obj_new:
			strState = "new";
			break;
		case obj_reappeare:
			strState = "reappear";
			break;
		case obj_tracked:
			strState = "tracked";
			break;
		case obj_missing:
			strState = "missing";
			break;
		case obj_disappear:
			strState = "disappear";
		default:
			break;
		}


		CvPoint p = cvPoint(track->cx, track->cy + track->cur_y + track->cur_h);
		track->trajectory->push_back(p);              ////////////////?
		track->stateSeq->push_back(track->state);
		/////////////////////////////////////////////////////////////new
		CvPoint p1 = cvPoint(track->cur_w, track->cur_h);
		track->curwhSeq->push_back(p1);
		//model update
		if (flagFind == 1)// && iFrameCount % 3 == 0)
		{
			updateObject(track, one2groupBestConfig[i], i);   ////
		}
		else if (ifNearBorder == 0)
		{

			//updateObjectMissing(track);

			track->searchRangeX = track->searchRangeX + track->mean_v * 1.2;
			track->searchRangeY = track->searchRangeY + track->mean_v * 1.2;
		}

		if (track->state == obj_tracked && track->trajectory->size() % fps == 0)
		{
			int sumv = 0, count = 0;
			int t1 = int(track->trajectory->size()) - 1;
			int t2 = int(track->trajectory->size()) - fps;
			track->rav = abs((*track->trajectory)[t1].x - (*track->trajectory)[t2].x) + abs((*track->trajectory)[t1].y - (*track->trajectory)[t2].y);
			if (track->rav <= Tstop)
				trackList[i]->stopTime++;
			else
				trackList[i]->stopTime -= (trackList[i]->stopTime > 0);
			if (track->rav > track->maxrav)
				track->maxrav = track->rav;
		}
	}


	//recordCoupledRelation
	for (i = 0; i < (int)matchGroups.size(); i++)
	{
		for (j = 0; j < (int)matchGroups[i]->objIndex.size() - 1; j++)
		{
			for (int k = j + 1; k < (int)matchGroups[i]->objIndex.size(); k++)
			{
				addPair(trackList[matchGroups[i]->objIndex[j]]->ID, trackList[matchGroups[i]->objIndex[k]]->ID);
			}
		}
	}
	for (i = 0; i < vPair.size(); i++)
	{
		if (vPair[i].count > Tmerge*fps)
		{
			for (j = 0; j < (int)trackList.size(); j++)
			{
				if (trackList[j]->ID == vPair[i].ID1 || trackList[j]->ID == vPair[i].ID2)
				{
					if (float(vPair[i].count) / (trackList[j]->lifeTime - 10) > TmergePor)
					{
						trackList[j]->health = -1;
						break;
					}
				}
			}
		}
	}

	//kill bad track
	for (i = 0; i < (int)trackList.size(); i++)
	{
		if (trackList[i]->health <= 0)
		{
			//erase from vPair
			for (j = 0; j < vPair.size(); j++)
			{
				if (trackList[i]->ID == vPair[j].ID1 || trackList[i]->ID == vPair[j].ID2)
				{
					//	delete &(*(vPair.begin() + j));
					vPair.erase(vPair.begin() + j);
					j--;
				}
			}

			std::vector   <myObject*> ::iterator   iterbtn;
			iterbtn = trackList.begin() + i; ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			delete (myObject*)(*iterbtn);////////////////////////////////////////////////////////////////////////////////
			//allTrajectories.push_back(*(trackList[i]->trajectory));  //-------------------------把被删除的轨迹添加到轨迹统计容器中
			trackList.erase(trackList.begin() + i);
			i--;
		}
	}

	//set new and match to disappeared ones
	for (j = 0; j < blobnum; j++)
	{
		if (blobclose[j] == 0 && flagNewing == 0)
		{
			myObject* newObject = new myObject();
			initialObject(newObject, (*newBlobList)[j], featureMap);
			trackList.push_back(newObject);
			flagNewing = 5;
		}
	}

	//set the update mask value
	unStableLabel.clear();
	for (i = 0; i < (int)trackList.size(); i++)
	{
		if (trackList[i]->stopTime > 15)// ||  trackList[i]->motionFactor < 100)
		{
			unStableLabel.push_back(i + 1);
		}
	}
}

void MFTracker::updateObject(myObject* track, SampleConfig sample, int trackIndex)
{

	//statistics
	//record occluded label number

	CvPoint c = sample.c;
	int w = track->cur_w;
	int h = track->cur_h;
	uchar* pFG = (uchar*)FGmask->imageData;
	uchar* pLabel = (uchar*)trackLabel->imageData;
	float* pW = track->current_weight->data.fl;
	int stepFG = FGmask->widthStep;
	int stepLabel = trackLabel->widthStep;
	int stepW = track->current_weight->step / sizeof(float);
	float w_sum1 = 0;
	float o_sum1 = 0;
	int x_start = c.x + track->cur_x;
	int x_end = x_start + w - 1;
	int y_start = c.y + track->cur_y;
	int y_end = y_start + h - 1;

	int cx = sample.c.x; int cy = sample.c.y;
	int sw, sh, sx, sy;
	int bestMatchBlobIndex = one2oneBestMatch[trackIndex];

	if (bestMatchBlobIndex != -1)// && track->health < 30)
	{
		cx = (*newBlobList)[bestMatchBlobIndex].monRect.x + (*newBlobList)[bestMatchBlobIndex].monRect.width / 2;
		cy = (*newBlobList)[bestMatchBlobIndex].monRect.y + (*newBlobList)[bestMatchBlobIndex].monRect.height / 2;
		sw = (*newBlobList)[bestMatchBlobIndex].outRect.width; sh = (*newBlobList)[bestMatchBlobIndex].outRect.height;
		sx = (*newBlobList)[bestMatchBlobIndex].outRect.x - cx;
		sy = (*newBlobList)[bestMatchBlobIndex].outRect.y - cy;
		{
			float a = 0.05f;
			if (track->health < 10)
				a = 0.3f;
			track->cur_w = track->cur_w*(1 - a) + sw * a;			//初始时，物体大小可变化快些。
			track->cur_h = track->cur_h*(1 - a) + sh * a;
			track->cur_x = track->cur_x*(1 - a) + sx * a;
			track->cur_y = track->cur_y*(1 - a) + sy * a;
		}
	}
	else if (track->health > 30)
	{
		int x2 = track->w_map - 1;
		float leftMaxW = 0, rightMaxW = 0;
		pW = track->current_weight->data.fl;
		for (int y = 0; y < track->h_map; y++)
		{
			pW = track->current_weight->data.fl + stepW * y;
			leftMaxW = (*pW > leftMaxW) ? (*pW) : leftMaxW;
			pW += x2;
			rightMaxW = (*pW > rightMaxW) ? (*pW) : rightMaxW;
		}
		int y2 = track->h_map - 1;
		float bottomMaxW = 0, topMaxW = 0;
		pW = track->current_weight->data.fl;
		for (int x = 0; x < track->w_map; x++)
		{
			pW = track->current_weight->data.fl + x;
			bottomMaxW = (*pW > bottomMaxW) ? (*pW) : bottomMaxW;
			pW += stepW * y2;
			topMaxW = (*pW > topMaxW) ? (*pW) : topMaxW;
		}
		if (leftMaxW > 0.7){ track->cur_x--; track->cur_w++; }
		else if (leftMaxW < 0.3){ track->cur_x++; track->cur_w--; }

		if (rightMaxW > 0.7){ track->cur_w++; }
		else if (rightMaxW < 0.3){ track->cur_w--; }

		if (bottomMaxW > 0.7){ track->cur_y--; track->cur_h++; }
		else if (bottomMaxW < 0.3){ track->cur_y++; track->cur_h--; }

		if (topMaxW > 0.7){ track->cur_h++; }
		else if (topMaxW < 0.3){ track->cur_h--; }

		/*string str;
		str.Format("object %d oc : %.2f,%.2f,%.2f,%.2f\r\n", track->ID, leftMaxW, rightMaxW, bottomMaxW, topMaxW);
		strStateText += str;*/
	}

	float a = 0.1f;
	float vv = sqrt(float((track->cx - cx) * (track->cx - cx) + (track->cy - cy) * (track->cy - cy)));
	track->now_v = vv;
	int si = track->trajectory->size();
	float cx0 = (*track->trajectory)[si - 1].x;
	float cy0 = (*track->trajectory)[si - 1].y;
	float cx1 = 0;
	float cy1 = 0;
	float v1 = 0;
	if (si>5)
	{
		cx1 = (*track->trajectory)[si - 5].x;
		cy1 = (*track->trajectory)[si - 5].y;
		v1 = sqrt(float((cx0 - cx1) * (cx0 - cx1) + (cy0 - cy1) * (cy0 - cy1)));
	}
	else
	{
		v1 = 10;
	}

	string testt;

	//	AfxMessageBox(testt);
	if (track->state == obj_tracked)
	{

		for (int p = 19; p>0; p--)   //存储速度队列记录20帧？
		{
			track->arrayv[p] = track->arrayv[p - 1];

		}
		//  testt.Format("arrayv0=%f,arrayv1=%f,arrayv2=%f,arrayv3=%f,arrayv4=%f,/n arrayv5=%f,arrayv6=%f,arrayv7=%f,arrayv8=%f,arrayv9=%f,/n arrayv10=%f,arrayv11=%f,arrayv12=%f,arrayv13=%f,arrayv14=%f,/n arrayv15=%f,arrayv16=%f,arrayv17=%f,arrayv18=%f,arrayv19=%f",track->arrayv[0],track->arrayv[1],track->arrayv[2],track->arrayv[3],track->arrayv[4],track->arrayv[5],track->arrayv[6],track->arrayv[7],track->arrayv[8],track->arrayv[9],track->arrayv[10],track->arrayv[11],track->arrayv[12],track->arrayv[13],track->arrayv[14],track->arrayv[15],track->arrayv[16],track->arrayv[17],track->arrayv[18],track->arrayv[19]);
		//AfxMessageBox(testt);
		track->mean_v = (1 - a) * track->mean_v + vv * a;
		track->arrayv[0] = v1;

		float sum = 0;
		track->count_mean = 0;
		for (int r = 10; r<20; r++)
		{
			sum += track->arrayv[r];
		}
		track->pre_mean_v = sum / 10.0;
		float sum1 = 0;
		for (int z = 0; z<10; z++)
		{
			sum1 += track->arrayv[z];
		}
		track->now_mean_v = sum1 / 10.0;




	}
	track->lifeTime++;


	track->cx = cx;
	track->cy = cy;
	track->blob.outRect = cvRect(track->cx + track->cur_x, track->cy + track->cur_y, track->cur_w, track->cur_h);
	track->blob.ID = track->ID;//...
	track->blob.medianX = track->cx;
	track->blob.medianY = track->cy;

	if (bestMatchBlobIndex != -1)
	{
		track->searchRangeX = 0;
		track->searchRangeY = 0;
	}
	else
	{
		track->searchRangeX = (1 - a) * track->searchRangeX + a * track->blob.outRect.width * (1 - one2groupBestScore[trackIndex]);
		track->searchRangeY = (1 - a) * track->searchRangeY + a * track->blob.outRect.width * (1 - one2groupBestScore[trackIndex]);
	}

	if (-track->cur_x > track->map->cols / 2 || -track->cur_y > track->map->rows / 2
		|| track->cur_w + track->cur_x > track->map->cols / 2 || track->cur_h + track->cur_y > track->map->rows / 2)
	{
		int ow = track->map->cols, oh = track->map->rows;
		int nw = max(track->cur_w * 2, (float)track->map->cols * 2);
		int nh = max(track->cur_h * 2, (float)track->map->rows * 2);
		CvMat*	resizedMap = cvCreateMat(nh, nw, CV_8UC3);
		CvMat*	resizedW = cvCreateMat(nh, nw, CV_32FC1);
		CvMat*	areaCurMap = cvCreateMatHeader(oh, ow, CV_8UC3);
		CvMat*	areaCurW = cvCreateMatHeader(oh, ow, CV_32FC1);
		CvRect rect = cvRect((nw - ow) / 2, (nh - oh) / 2, ow, oh);
		cvSet(resizedMap, cvScalarAll(128));
		cvSet(resizedW, cvRealScalar(0));
		cvGetSubRect(resizedMap, areaCurMap, rect);
		cvGetSubRect(resizedW, areaCurW, rect);
		cvCopy(track->map, areaCurMap);
		cvCopy(track->weight, areaCurW);
		cvReleaseMat(&track->map);
		cvReleaseMat(&track->weight);
		cvReleaseMatHeader(&areaCurMap);
		cvReleaseMatHeader(&areaCurW);
		track->map = resizedMap;
		track->weight = resizedW;
	}

	cvReleaseMatHeader(&track->current_map);
	cvReleaseMatHeader(&track->current_weight);
	track->current_map = cvCreateMatHeader(track->cur_h, track->cur_w, CV_8UC3);
	track->current_weight = cvCreateMatHeader(track->cur_h, track->cur_w, CV_32FC1);
	CvMat* current_weight_buffer = cvCreateMat(track->cur_h, track->cur_w, CV_32FC1);
	CvRect rect = cvRect(track->map->cols / 2 + track->cur_x, track->map->rows / 2 + track->cur_y, track->cur_w, track->cur_h);
	cvGetSubRect(track->map, track->current_map, rect);
	cvGetSubRect(track->weight, track->current_weight, rect);
	cvCopy(track->current_weight, current_weight_buffer);
	cvZero(track->weight);
	cvCopy(current_weight_buffer, track->current_weight);
	cvReleaseMat(&current_weight_buffer); ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	track->w_map = track->cur_w;
	track->h_map = track->cur_h;

	//data process

	//if(bestMatchBlobIndex != -1)

	unsigned char* pImage = (uchar*)frame->imageData;
	unsigned char* pFGmask = (uchar*)FGmask->imageData;
	pLabel = (uchar*)trackLabel->imageData;
	int widthStep3c = frame->widthStep;
	int widthStep = FGmask->widthStep;
	int i, j;

	int cw = track->cur_w;
	int ch = track->cur_h;

	CvMat* imagePatch = cvCreateMat(ch, cw, CV_8UC3);
	CvMat* FGmaskPatch = cvCreateMat(ch, cw, CV_8UC1);
	CvMat* LabelPatch = cvCreateMat(ch, cw, CV_8UC1);
	unsigned char* pImagePatch = imagePatch->data.ptr;
	unsigned char* pFGmaskPatch = FGmaskPatch->data.ptr;
	unsigned char* pLabelPatch = LabelPatch->data.ptr;
	int step3c = imagePatch->step;
	int step = FGmaskPatch->step;

	int IPplace, FPplace, imgPlace, FGplace;
	x_start = cx + track->cur_x;
	y_start = cy + track->cur_y;


	step3c = imagePatch->step;
	step = FGmaskPatch->step;
	int mapstep3c = track->current_map->step;
	int wstep = track->current_weight->step / sizeof(float);
	int weightPlace, mapPlace;
	float* pWeight = track->current_weight->data.fl;
	unsigned char* pMap = track->current_map->data.ptr;
	double difference;
	int Tlow = 30;
	int Tup = 130;
	float G_a = 0.05f;
	float _G_a = 1 - G_a;
	float _a_G = 1 / _G_a;

	for (i = 0; i < ch; i++)
	{
		FPplace = i * step; IPplace = i * step3c;
		if (y_start + i  < 0 || y_start + i >= frame->height)
		{
			for (j = 0; j < cw; j++)
			{
				*(pFGmaskPatch + FPplace) = 0;
				*(pLabelPatch + FPplace) = 0;
				*(pImagePatch + IPplace) = 128;
				*(pImagePatch + IPplace + 1) = 128;
				*(pImagePatch + IPplace + 2) = 128;
			}
		}
		else
		{
			imgPlace = (y_start + i) * widthStep3c + x_start * 3;
			FGplace = (y_start + i) * widthStep + x_start;
			weightPlace = i * wstep; mapPlace = i * mapstep3c;

			for (j = 0; j < cw; j++)
			{
				if (x_start + j < 0 || x_start + j >= frame->width)
				{
					*(pFGmaskPatch + FPplace) = 0;
					*(pImagePatch + IPplace) = 128;
					*(pImagePatch + IPplace + 1) = 128;
					*(pImagePatch + IPplace + 2) = 128;
				}
				else
				{
					*(pFGmaskPatch + FPplace) = *(pFGmask + FGplace) > 0 ? 255 : 0;
					*(pLabelPatch + FPplace) = (*(pLabel + FGplace) == trackIndex + 1 || (*(pFGmask + FGplace) > 0 && *(pLabel + FGplace) == 0/* && bestMatchBlobIndex != -1)*/)) ? 255 : 0;
					*(pImagePatch + IPplace) = *(pImage + imgPlace);
					*(pImagePatch + IPplace + 1) = *(pImage + imgPlace + 1);
					*(pImagePatch + IPplace + 2) = *(pImage + imgPlace + 2);
					difference = (abs(*(pImagePatch + IPplace) - *(pMap + mapPlace))
						+ abs(*(pImagePatch + IPplace + 1) - *(pMap + mapPlace + 1))
						+ abs(*(pImagePatch + IPplace + 2) - *(pMap + mapPlace + 2))) / 3.0;
					if ((*(pWeight + weightPlace) > 0.5 || difference < 30) && *(pLabel + FGplace) == 0)
					{
						*(pLabel + FGplace) == trackIndex + 1;
					}
				}
				float& w = *(pWeight + weightPlace);
				track->sum_weight += w;
				unsigned char* pI = pImagePatch + IPplace;
				unsigned char* pM = pMap + mapPlace;
				if (*(pFGmaskPatch + FPplace) == 0 && *(pLabelPatch + FPplace) == 0)
				{
					w *= _G_a;
				}
				else if (*(pLabelPatch + FPplace) > 0)
				{
					difference = (abs(*(pI)-*(pM))
						+ abs(*(pI + 1) - *(pM + 1))
						+ abs(*(pI + 2) - *(pM + 2))) / 3.0;
					difference = (difference - Tlow) / (Tup - Tlow);
					difference = max((double)0, min(1.0, difference));
					w = (w == 0) ? 0.3 : _G_a * w + G_a * (1 - difference);
					//ASSERT(w>0);
					float a1 = w / (w + G_a);
					float b1 = 1 - a1;
					*pM = *pM * a1 + *pI * b1;
					*(pM + 1) = *(pM + 1) * a1 + *(pI + 1) * b1;
					*(pM + 2) = *(pM + 2) * a1 + *(pI + 2) * b1;
				}

				weightPlace++;
				mapPlace += 3;
				FPplace++; FGplace++;
				IPplace += 3; imgPlace += 3;
			}
		}
	}

	cvReleaseMat(&imagePatch);
	cvReleaseMat(&FGmaskPatch);
	cvReleaseMat(&LabelPatch);

	{
		int x_start, y_start, x_end, y_end;
		vector<Feature*>* pFeatures = &track->featureList;
		Feature** pF;
		int size = pFeatures->size();
		uchar* pLabel;
		int stepLabel = trackLabel->widthStep;
		int fc = 0;
		for (int i = 0; i < size; i++)
		{
			int ax = (int)min((float)7, (float)track->cur_w);
			int ay = (int)min((float)7, (float)track->cur_w);
			Feature* feature = (*pFeatures)[i];
			x_start = (int)(cx + feature->rx - ax + .5);
			y_start = (int)(cy + feature->ry - ay + .5);
			x_end = (int)(cx + feature->rx + ax + .5);
			y_end = (int)(cy + feature->ry + ay + .5);

			x_start = max(x_start, 0);
			y_start = max(y_start, 0);
			x_end = min(x_end, iW - 1);
			y_end = min(y_end, iH - 1);


			int flagMatch = 0;

			Feature** maxMatchF = NULL;
			float maxScore = (float)0.9;			//threshold
			for (int y = y_start; y <= y_end; y++)
			{
				pLabel = (uchar*)trackLabel->imageData + stepLabel * y + x_start;
				pF = featureMap + y * iW + x_start;
				for (int x = x_start; x <= x_end; x++)
				{
					if (*pLabel == trackIndex + 1 && (*pF) != NULL)
					{
						float nzc = compare(*pF, feature);
						if (nzc > maxScore)
						{
							maxScore = nzc;
							maxMatchF = pF;
						}
					}
					pF++;
					pLabel++;
				}
			}
			if (maxMatchF != NULL)
			{
				(*maxMatchF)->rx = (*maxMatchF)->x - cx;
				(*maxMatchF)->ry = (*maxMatchF)->y - cy;
				feature->update(*maxMatchF);
				if (feature->health >= 0)
				{
					//cvRectangle(&showImage_feature, cvPoint((*maxMatchF)->x-2,(*maxMatchF)->y-2),cvPoint((*maxMatchF)->x+2,(*maxMatchF)->y+2), FakeRGB[min(feature->health/10,9)]);//min(feature->health * 2,255)));
					int ex = (*maxMatchF)->x, ey = (*maxMatchF)->y;
					int ox = ex, oy = ey;
					for (int ti = int(feature->traj.size()) - 2; ti >= max(0, int(feature->traj.size()) - 30); ti--)
					{
						int nx = feature->traj[ti].x;
						int ny = feature->traj[ti].y;
						if ((nx - ex)*(nx - ex) + (ny - ey)*(ny - ey) >  900)
						{
							break;
						}
						else
						{
							//cvLine(&showImage_feature, feature->traj[ti], feature->traj[ti+1], FakeRGB[min(feature->health/10,9)]);//min(feature->health * 2,255)));
						}
					}
				}
				*maxMatchF = NULL;
				feature->health = feature->health + 2;
			}
			else
			{
				feature->health--;
				if (feature->health <= 0)
				{
					cvReleaseMat(&(*pFeatures)[i]->patch);
					delete (*pFeatures)[i];//////////////////////////////
					pFeatures->erase(pFeatures->begin() + i);
					size--;
					i--;
				}
			}
		}
		x_start = cx + (int)track->cur_x;
		y_start = cy + (int)track->cur_y;
		x_end = x_start + (int)track->cur_w - 1;
		y_end = y_start + (int)track->cur_h - 1;
		x_start = max(x_start, 0);
		y_start = max(y_start, 0);
		x_end = min(x_end, iW - 1);
		y_end = min(y_end, iH - 1);
		for (int y = y_start; y <= y_end; y++)
		{
			pLabel = (uchar*)trackLabel->imageData + stepLabel * y + x_start;
			pF = featureMap + y * iW + x_start;
			for (int x = x_start; x <= x_end; x++)
			{
				if ((*pLabel == trackIndex + 1) && *pF != NULL && ((double)rand() / (double)RAND_MAX) * (int)(pFeatures->size()) < 30)
				{
					//add feature
					Feature* fcopy = new Feature();
					*fcopy = **pF;
					fcopy->rx = (*pF)->x - cx;
					fcopy->ry = (*pF)->y - cy;
					fcopy->health = 25;
					fcopy->patch = cvCloneMat((*pF)->patch);
					pFeatures->push_back(fcopy);
					*pF = NULL;
				}
				pLabel++;
				pF++;
			}
		}
	}
}

void MFTracker::initialObject(myObject* newObject, myBlob newBlob, Feature** featureMap)
{
	newObject->blob = newBlob;
	newObject->ID = nextnewID;
	nextnewID--;
	newObject->blob.ID = newObject->ID;
	newObject->state = obj_new;
	newObject->health = 1;


	int w = newBlob.outRect.width;
	int h = newBlob.outRect.height;

	newObject->cx = newBlob.monRect.x + newBlob.monRect.width / 2;
	newObject->cy = newBlob.monRect.y + newBlob.monRect.height / 2;
	newObject->cur_w = (float)w;
	newObject->cur_h = (float)h;
	newObject->cur_x = (float)(newBlob.outRect.x - newObject->cx);
	newObject->cur_y = (float)(newBlob.outRect.y - newObject->cy);
	//new/////////////////////////////////////////////////////////////////
	newObject->curwhSeq = new vector<CvPoint>;
	newObject->trajectory = new vector<CvPoint>;
	newObject->stateSeq = new vector<objectState>;
	CvPoint p = cvPoint(newObject->cx, newObject->cy);
	CvPoint p1 = cvPoint(newObject->cur_w, newObject->cur_h);
	newObject->curwhSeq->push_back(p1);
	newObject->trajectory->push_back(p);
	newObject->stateSeq->push_back(newObject->state);
	//copy FG->w，img->map;
	newObject->w_map = w;
	newObject->h_map = h;
	newObject->map = cvCreateMat(h * 2, w * 2, CV_8UC3);
	newObject->weight = cvCreateMat(h * 2, w * 2, CV_32FC1);
	cvSet(newObject->map, cvScalarAll(128));
	cvSet(newObject->weight, cvRealScalar(0));
	CvRect area = cvRect(w + newObject->cur_x, h + newObject->cur_y, w, h);
	newObject->current_map = cvCreateMat(h, w, CV_8UC3);
	newObject->current_weight = cvCreateMat(h, w, CV_32FC1);
	cvGetSubRect(newObject->map, newObject->current_map, area);
	cvGetSubRect(newObject->weight, newObject->current_weight, area);
	CvMat* blobPatch = cvCreateMatHeader(h, w, CV_8UC3);
	CvMat* FGPatch = cvCreateMatHeader(h, w, CV_8UC1);
	cvGetSubRect(frame, blobPatch, newBlob.outRect);
	cvGetSubRect(FGmask, FGPatch, newBlob.outRect);
	cvCopy(blobPatch, newObject->current_map, FGPatch);
	cvSet(newObject->current_weight, cvRealScalar(0.3), FGPatch);
	newObject->sum_weight = 0.3 * cvCountNonZero(FGPatch);
	cvReleaseMatHeader(&blobPatch);
	cvReleaseMatHeader(&FGPatch);
	int x_start = (int)(newObject->blob.outRect.x);
	int y_start = (int)(newObject->blob.outRect.y);
	int x_end = (int)(newObject->blob.outRect.x + w);
	int y_end = (int)(newObject->blob.outRect.y + h);
	Feature** pF;
	vector<Feature*>* pFeatures = &newObject->featureList;
	for (int y = y_start; y < y_end; y++)
	{
		pF = featureMap + y * iW + x_start;
		for (int x = x_start; x < x_end; x++)
		{
			if ((*pF) != NULL)                            //新加入的features信息居然木有影响？
			{
				Feature* fcopy = new Feature();
				*fcopy = **pF;
				fcopy->rx = (*pF)->x - newObject->cx;
				fcopy->ry = (*pF)->y - newObject->cy;
				fcopy->health = 30;
				fcopy->patch = cvCloneMat((*pF)->patch);
				newObject->featureList.push_back(fcopy);
				*pF = NULL;
			}
			pF++;
		}
	}

}


vector<myObject*>* MFTracker::getTrackList()
{
	return &trackList;
}

IplImage* MFTracker::getTrackLabel()
{
	return trackLabel;
}


int MatchGroup::compare(MatchGroup* mg)      //如果两个组的前景块和跟踪块的成员全部不同则返回0否则返回1
{
	for (int i = 0; i<(int)objIndex.size(); i++)
	{
		for (int j = 0; j < (int)mg->objIndex.size(); j++)
		{
			if (objIndex[i] == mg->objIndex[j])
			{
				return 1;
			}
		}
	}
	for (int i = 0; i<(int)blobIndex.size(); i++)
	{
		for (int j = 0; j < (int)mg->blobIndex.size(); j++)
		{
			if (blobIndex[i] == mg->blobIndex[j])
			{
				return 1;
			}
		}
	}
	return 0;
}

void MatchGroup::unite(MatchGroup* mg)            //将2个匹配组合并，把mg合并到this
{
	for (int i = 0; i<(int)mg->objIndex.size(); i++)
	{
		for (int j = 0; j < (int)objIndex.size(); j++)
		{
			if (objIndex[j] == mg->objIndex[i])
			{
				mg->objIndex.erase(mg->objIndex.begin() + i);
				i--;
				break;
			}
		}
	}
	for (int i = 0; i<(int)mg->blobIndex.size(); i++)
	{
		for (int j = 0; j < (int)blobIndex.size(); j++)
		{
			if (blobIndex[j] == mg->blobIndex[i])
			{
				mg->blobIndex.erase(mg->blobIndex.begin() + i);
				i--;
				break;
			}
		}
	}
	for (int i = 0; i<(int)mg->blobIndex.size(); i++)
	{
		blobIndex.push_back(mg->blobIndex[i]);
	}
	for (int i = 0; i<(int)mg->objIndex.size(); i++)
	{
		objIndex.push_back(mg->objIndex[i]);
	}
}
vector<int>* MFTracker::getUnstableLabel()
{
	return&unStableLabel;
}

//void MFTracker::integrateAllTrajectories()
//{
//	for (int i = 0; i < (int)trackList.size(); ++i)
//	{
//		vector<CvPoint> *pTraj = trackList[i]->trajectory;
//		allTrajectories.push_back(*pTraj);
//	}
//}


