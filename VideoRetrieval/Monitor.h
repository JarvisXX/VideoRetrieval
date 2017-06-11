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
	//ת�� ���� ��ǰ֡
	IplImage*               m_frame;	//current frame
public:
	int						iW, iH, iStride, iLen;
	//ģ�鶼��Ϊ ָ�����Ա��������
	MoGFGDetector*			m_pFG; /* pointer to foreground mask detector module */
	//	CvBlobDetector*			m_pBD; /* pointer to Blob detector module */
	MFTracker*				m_pTR;/* pointer to Tracker module */
	//	ObjectList				m_ObjectList;/*һ��Ŀ���б�*/
	CornerFinder*			m_pCF;//�ǵ�����

	IplImage*               m_pFGMask;//forground mask

	IplImage*				m_pBG;		//background
	 int                     m_FGTrainFrames;	//�Ѿ�ѵ����֡��

	float					m_f;	//��С����  ����ͼƬ��С �����ٶ�
	vector<myBlob>*         m_BlobList;	//ǰ��blob��list
	int                     m_FrameCount;	//������֡��

	int                     Crosslineflag ;             
	int                     Crosslinex1, Crosslinex2, Crossliney1, Crossliney2;
	
	int                     m_NextBlobID;	//Ŀ�걻������˳����ID�����ﱣ����һ�����õ�ID

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

	// ��⵽���˺ͳ�
//ector<DetectObject> detectedobject;

	//�����쳣
	string basedir_global;
	bool readFlag = false;
	vector<vector<double>> centers_v_read;//����ľ�������
	vector<vector<double>> centers_traj_read;//����ľ�������
	vector<vector<CvPoint>> completeTrajectories_detect;
	int detectCount = 0;//�Ѽ��Ĺ켣����
	vector<int> isNormalAll_v;
	vector<int> isNormalAll_traj;
	vector<int> isNormalAll_plot;
	vector<int> isNormalAll_res;//�����

	

public:

	//Joseph
	template <typename T>
	int posInVector(const vector<T> &vec, const T elem);
	//Joseph

	// �ж��Ƿ���ͻȻ����
	void IsSuddenAccelerate(float velocity, std::vector<myObject*> &trackList, rule *pRule, int tracklistID, IplImage *frame, float sudden_accelerate_value);
	// �ж��Ƿ��ǻ�
	void IsWander(float vps, float cur_width, float cur_height, std::vector<myObject*> &trackList, rule *pRule, int tracklistID, IplImage *frame);


	void IsRemove(vector<myObject*> TrackList, int iH, int iW, IplImage*frame, rule* pRule);

	// ���ּ�⺯��
	void InvasionDetection(IplImage* frame, std::vector<myObject*> &trackList, rule *pRule);
	bool PtInRectangle(double X[], double Y[], double x, double y);
	int NumOfPts(rule *p_invarule);
	void CalculateK(double TopP_X[20], double TopP_Y[20], int NumP, double *k);
	bool IsInArea(double TopP_X[20], double TopP_Y[20], int NumP, double k[20], double X, double Y);

	// ���߼�⺯��
	void DrawLine(IplImage *img, float xa, float xb, float ya, float yb, int lineid, int type);
	void DrawBlobRec(IplImage* img, myBlob* pBlob, int Warn);
	linedirection CrossDirection(float x, float y, float x1, float y1, float x2, float y2);
	void CrossLineDetection(IplImage *frame, std::vector<myObject*> &trackList, rule *pRule, int x1, int x2, int y1, int y2);

	// ·����⺯��
	void PathDetection(IplImage *frame, IplImage *m_pFGMask, std::vector<myObject*> &trackList, rule *pRule);

	// ������⺯��
	void StayDetection(IplImage* frame, std::vector<myObject*> &trackList, rule *pRule);

	// �ǻ����
	void WanderDetection(IplImage* frame, std::vector<myObject*> &trackList, rule *pRule);

	// ͻȻ���ּ�⺯��
	void SuddenAppearDetection(IplImage *frame, std::vector<myObject*> &trackList, rule *pRule);
	Points Crosslinelocation(IplImage *frame);
	void MoveOutDetection(IplImage* frame, rule* pRule);
	void AddNewWarning(string strWarning, string HappenTime, int warntype);
	string GetRightTime(void);
	string GetTheWholeWarns(void);
	// 0-���ּ�� 1-�ǻ���� 2-������� 3-���߼�� 4-������� 5-ͻȻ���ּ�� 6-ͻȻ���ټ�� 7-˫����߼�� 8-���ڵ�����߼�� 9-���⵽�ڰ��߼��
	int CurrentWarnInfo[10];
	void ClearCurrenWarnInfor(void);
	int GetTheCurrentWarnArray(int warninforarray[10]);
	void GetOutPutInfor(char* pOutAlarm, int* pInOutAlarmMemSize, int* pOutAlarmCount);
	bool moveout_OPEN;
	string GetTheShowWarns(void);
	// ��ǰ�������Ƶ���
	int ilHandle;
	void DrawBlobRecByClaa(CvArr* img, myBlob* pBlob, char*TypeofWarn);
	bool IsPointOnLine(float x, float y, float x1, float y1, float x2, float y2);
	int TESTPathCount;
};

#endif