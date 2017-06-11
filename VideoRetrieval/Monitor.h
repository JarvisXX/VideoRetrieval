#ifndef MONITOR_H
#define MONITOR_H

#include "clude.h"
#include "Object.h"
#include "MoGFGDetector.h"
#include "CornerFinder.h"
#include "CarPedDetector.h"
#include "MFTracker.h"
#include "Rule.h"
#include "GetFeatureAll.h"
#include "GetFeatureEach.h"
#include "MeanShiftClustering.h"
#include "MeanShiftFunc.h"

class Monitor
{
public:
	Monitor();
	void Init(int ww, int hh, int stride, float factor);
	IplImage*  Process(IplImage* frame, rule*EntityRule, long lHandle);
	void DrawBlobRec(CvArr* img, myBlob* pBlob);
	//void CutBlobRec(CvArr* img, myBlob* pBlob);
	void CutBlobRec(IplImage* img, IplImage *m_pFGMask, myBlob* pBlob, int &item_id_);
	virtual ~Monitor();
	//转存 缩放 当前帧
	IplImage*               m_frame;	//current frame
public:
	int						iW, iH, iStride, iLen;
	//模块都作为 指针类成员变量存在
	MoGFGDetector*			m_pFG; /* pointer to foreground mask detector module */
	//	CvBlobDetector*			m_pBD; /* pointer to Blob detector module */
	MFTracker*				m_pTR;/* pointer to Tracker module */
	//	ObjectList				m_ObjectList;/*一个目标列表*/
	CornerFinder*			m_pCF;//角点检测器

	IplImage*               m_pFGMask;//forground mask

	IplImage*				m_pBG;		//background
	 int                     m_FGTrainFrames;	//已经训练的帧数

	float					m_f;	//缩小比例  将大图片缩小 增快速度
	vector<myBlob>*         m_BlobList;	//前景blob的list
	int                     m_FrameCount;	//经过的帧数

	int                     Crosslineflag ;             
	int                     Crosslinex1, Crosslinex2, Crossliney1, Crossliney2;
	
	int                     m_NextBlobID;	//目标被按出现顺序赋予ID，这里保存下一个可用的ID

	////////////////////////////////////////////////////////////////////////////////////////////
	////  trajectory clustering	- Joseph YU                        							////
	////////////////////////////////////////////////////////////////////////////////////////////
	vector<int>				myObjectID;
	vector<int>				itemIDlist;
	vector<int>				itemIDlist_detect;
	vector<bool>			ObjectMatched;
	vector<vector<CvPoint>> allTrajectories;	//all trajectories are in this vector

	vector<int>				completedObjPos;
	vector<vector<CvPoint>> completeTrajectories;
	////////////////////////////////////////////////////////////////////////////////////////////
	//// trajectory clustering	- Joseph YU													////
	////////////////////////////////////////////////////////////////////////////////////////////

	// list<WarningString>     m_listwarn;
	//	vector<Warning>			m_warningList;
	float SuddenAccelerateValue;

	/////////////////////////
	////////////////////////

	CCarPedDetector* carpedDec;

	// 检测到的人和车
//ector<DetectObject> detectedobject;

	//级联异常
	string basedir_global;
	bool readFlag = false;
	vector<vector<double>> centers_v_read;//读入的聚类中心
	vector<vector<double>> centers_traj_read;//读入的聚类中心
	vector<vector<CvPoint>> completeTrajectories_detect;
	int detectCount = 0;//已检测的轨迹数量
	vector<int> isNormalAll_v;
	vector<int> isNormalAll_traj;
	vector<int> isNormalAll_plot;
	vector<int> isNormalAll_res;//检测结果

	

public:

	//Joseph
	template <typename T>
	int posInVector(const vector<T> &vec, const T elem);
	//Joseph

	// 判断是否是突然加速
	void IsSuddenAccelerate(float velocity, std::vector<myObject*> &trackList, rule *pRule, int tracklistID, IplImage *frame, float sudden_accelerate_value);
	// 判断是否徘徊
	void IsWander(float vps, float cur_width, float cur_height, std::vector<myObject*> &trackList, rule *pRule, int tracklistID, IplImage *frame);


	void IsRemove(vector<myObject*> TrackList, int iH, int iW, IplImage*frame, rule* pRule);

	// 入侵检测函数
	void InvasionDetection(IplImage* frame, std::vector<myObject*> &trackList, rule *pRule);
	bool PtInRectangle(double X[], double Y[], double x, double y);
	int NumOfPts(rule *p_invarule);
	void CalculateK(double TopP_X[20], double TopP_Y[20], int NumP, double *k);
	bool IsInArea(double TopP_X[20], double TopP_Y[20], int NumP, double k[20], double X, double Y);

	// 过线检测函数
	void DrawLine(IplImage *img, float xa, float xb, float ya, float yb, int lineid, int type);
	void DrawBlobRec(IplImage* img, myBlob* pBlob, int Warn);
	linedirection CrossDirection(float x, float y, float x1, float y1, float x2, float y2);
	void CrossLineDetection(IplImage *frame, std::vector<myObject*> &trackList, rule *pRule, int x1, int x2, int y1, int y2);

	// 路径检测函数
	void PathDetection(IplImage *frame, IplImage *m_pFGMask, std::vector<myObject*> &trackList, rule *pRule);

	// 滞留检测函数
	void StayDetection(IplImage* frame, std::vector<myObject*> &trackList, rule *pRule);

	// 徘徊检测
	void WanderDetection(IplImage* frame, std::vector<myObject*> &trackList, rule *pRule);

	// 突然出现检测函数
	void SuddenAppearDetection(IplImage *frame, std::vector<myObject*> &trackList, rule *pRule);
	Points Crosslinelocation(IplImage *frame);
	void MoveOutDetection(IplImage* frame, rule* pRule);
	void AddNewWarning(string strWarning, string HappenTime, int warntype);
	string GetRightTime(void);
	string GetTheWholeWarns(void);
	// 0-入侵检测 1-徘徊检测 2-滞留检测 3-移走检测 4-遗留检测 5-突然出现检测 6-突然加速检测 7-双向拌线检测 8-从内到外拌线检测 9-从外到内拌线检测
	int CurrentWarnInfo[10];
	void ClearCurrenWarnInfor(void);
	int GetTheCurrentWarnArray(int warninforarray[10]);
	void GetOutPutInfor(char* pOutAlarm, int* pInOutAlarmMemSize, int* pOutAlarmCount);
	bool moveout_OPEN;
	string GetTheShowWarns(void);
	// 当前处理的视频标记
	int ilHandle;
	void DrawBlobRecByClaa(CvArr* img, myBlob* pBlob, char*TypeofWarn);
	bool IsPointOnLine(float x, float y, float x1, float y1, float x2, float y2);
	int TESTPathCount;
};

#endif