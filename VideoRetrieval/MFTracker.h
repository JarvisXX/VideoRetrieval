#ifndef MFTRACKER_H
#define MFTRACKER_H

#include "Object.h"

//停止和连接检测的参数
#define Tstop 10	  
#define Tmerge 5
#define TmergePor 0.9


//记录接近的目标对
struct closePair
{
	int ID1;
	int ID2;
	int count;
};

//跟踪过程中的一个目标位置采样点
class SampleConfig
{
public:
	SampleConfig(){ f = 1; };
	CvPoint c;
	float f;	//标记数据，暂时无用		
};

//跟踪过程中对目标和前景块的配对分组 （参考论文）
class MatchGroup
{
public:
	~MatchGroup()
	{

	};
	vector<int> blobIndex;   //组中的前景块
	vector<int> objIndex;//组中的目标
	CvRect groupBlob;//外接框
	int compare(MatchGroup* mg);
	void unite(MatchGroup* mg);
};

class MFTracker
{
public:

	MFTracker();
	~MFTracker(void);
	void initialize(int w, int h);
	vector<int> unStableLabel;
	//跟踪过程中 可以将本帧跟踪结果输出给背景建模的更新模块 用来保留停止运动的前景物体；但同时，建立unStableLabel序列，从要保留的目标区域中去除当前跟踪中的非稳定的目标。
	vector<int>* getUnstableLabel();

	//跟踪主函数
	void track(IplImage* framein, IplImage* FGmaskin, IplImage* BGin, vector<myBlob>* newBlobListin, Feature** featureMap);

	//取跟踪结果
	vector<myObject*>* getTrackList();

	//获得跟踪结果 - 像素label结果
	IplImage* getTrackLabel();
	//vector<vector<CvPoint>> allTrajectories; //轨迹聚类用，存放所有的轨迹

	//void integrateAllTrajectories();
private:
	//目标队列
	vector<myObject*>	trackList;

	//当前帧 前景模板 背景（不一定需要实现）
	IplImage* frame, *FGmask, *BG;

	//特征图 同CornerFinder中的定义 这里作指针转存用而已
	Feature** featureMap;

	//转存从前景检测模块来的blob序列
	vector<myBlob>* newBlobList;

	//图像尺寸
	int w, h;

	//计算所有目标和前景块之间的Bounding Box Distance
	void calBoundingBoxDistance();

	//分组目标和前景块
	void makeMatchGroups();

	//具体计算Bounding Box Distance
	float getBoundingBoxDistance(int cx, int cy, int bleft, int bright, int bbottom, int btop);

	//存储Bounding Box Distance 的矩阵
	float* distanceMatrix;

	//2值化的表示 前景块和目标 接近关系的矩阵
	int* closeMatrix;

	//分出来的组
	vector<MatchGroup*> matchGroups;


	//和每一个目标接近的前景块的数目
	int* trackclose;
	//和每一个前景块接近的目标的数目
	int* blobclose;

	//每一个目标单独搜索得到的最好匹配结果
	float* one2oneBestScore;
	//相应匹配最好的前景块
	int* one2oneBestMatch;

	//在组中搜索得到的最好匹配结果
	float* one2groupBestScore;
	//相应匹配最好的采样位置（目标位置）
	SampleConfig* one2groupBestConfig;

	//目标的ID控制
	int nextID, nextnewID;

	//×设置一个标志，表示有正在验证的新出现目标，此时不允许添加新目标，主要用来解决同一目标在刚出现时被检测成多个碎块。
	int flagNewing;

	//对每一组进行搜索匹配 （实现跟踪的主要过程）
	void groupMatch();

	//对每一个目标 track在一定范围comBlob内进行最优位置的搜索
	void doSearch(myObject* track, CvRect comBlob, float* bestScore, SampleConfig* bestConfig);

	//对前景点进行标记
	void labelPixel(CvRect blob, int tracknumInGroup, int* trackIndex, int* goodIndex, int* Depth, float* occludedW, float* w_sum);//also need trackLabel

	//得到对当前位置的评估
	float getLikelihood(myObject* track, SampleConfig sample);

	//对坐标位置的限定
	void limit(int x1, int y1, int x2, int y2, CvSize S);

	//在完成对各个目标的跟踪之后，对每个目标的状态和模型做更新（实现跟踪的主要过程）
	void conclude();

	//更新模型
	void updateObject(myObject* track, SampleConfig bestConfig, int trackIndex);

	//初始化新的目标
	void initialObject(myObject* newObject, myBlob newBlob, Feature** featureMap);

	//输出图像尺寸
	int iW, iH;

	//跟踪结果 像素label结果
	IplImage* trackLabel;

public:
	int testnum;
	int testbnum;
};

#endif