#ifndef MFTRACKER_H
#define MFTRACKER_H

#include "Object.h"

//ֹͣ�����Ӽ��Ĳ���
#define Tstop 10	  
#define Tmerge 5
#define TmergePor 0.9


//��¼�ӽ���Ŀ���
struct closePair
{
	int ID1;
	int ID2;
	int count;
};

//���ٹ����е�һ��Ŀ��λ�ò�����
class SampleConfig
{
public:
	SampleConfig(){ f = 1; };
	CvPoint c;
	float f;	//������ݣ���ʱ����		
};

//���ٹ����ж�Ŀ���ǰ�������Է��� ���ο����ģ�
class MatchGroup
{
public:
	~MatchGroup()
	{

	};
	vector<int> blobIndex;   //���е�ǰ����
	vector<int> objIndex;//���е�Ŀ��
	CvRect groupBlob;//��ӿ�
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
	//���ٹ����� ���Խ���֡���ٽ�������������ģ�ĸ���ģ�� ��������ֹͣ�˶���ǰ�����壻��ͬʱ������unStableLabel���У���Ҫ������Ŀ��������ȥ����ǰ�����еķ��ȶ���Ŀ�ꡣ
	vector<int>* getUnstableLabel();

	//����������
	void track(IplImage* framein, IplImage* FGmaskin, IplImage* BGin, vector<myBlob>* newBlobListin, Feature** featureMap);

	//ȡ���ٽ��
	vector<myObject*>* getTrackList();

	//��ø��ٽ�� - ����label���
	IplImage* getTrackLabel();
	//vector<vector<CvPoint>> allTrajectories; //�켣�����ã�������еĹ켣

	//void integrateAllTrajectories();
private:
	//Ŀ�����
	vector<myObject*>	trackList;

	//��ǰ֡ ǰ��ģ�� ��������һ����Ҫʵ�֣�
	IplImage* frame, *FGmask, *BG;

	//����ͼ ͬCornerFinder�еĶ��� ������ָ��ת���ö���
	Feature** featureMap;

	//ת���ǰ�����ģ������blob����
	vector<myBlob>* newBlobList;

	//ͼ��ߴ�
	int w, h;

	//��������Ŀ���ǰ����֮���Bounding Box Distance
	void calBoundingBoxDistance();

	//����Ŀ���ǰ����
	void makeMatchGroups();

	//�������Bounding Box Distance
	float getBoundingBoxDistance(int cx, int cy, int bleft, int bright, int bbottom, int btop);

	//�洢Bounding Box Distance �ľ���
	float* distanceMatrix;

	//2ֵ���ı�ʾ ǰ�����Ŀ�� �ӽ���ϵ�ľ���
	int* closeMatrix;

	//�ֳ�������
	vector<MatchGroup*> matchGroups;


	//��ÿһ��Ŀ��ӽ���ǰ�������Ŀ
	int* trackclose;
	//��ÿһ��ǰ����ӽ���Ŀ�����Ŀ
	int* blobclose;

	//ÿһ��Ŀ�굥�������õ������ƥ����
	float* one2oneBestScore;
	//��Ӧƥ����õ�ǰ����
	int* one2oneBestMatch;

	//�����������õ������ƥ����
	float* one2groupBestScore;
	//��Ӧƥ����õĲ���λ�ã�Ŀ��λ�ã�
	SampleConfig* one2groupBestConfig;

	//Ŀ���ID����
	int nextID, nextnewID;

	//������һ����־����ʾ��������֤���³���Ŀ�꣬��ʱ�����������Ŀ�꣬��Ҫ�������ͬһĿ���ڸճ���ʱ�����ɶ����顣
	int flagNewing;

	//��ÿһ���������ƥ�� ��ʵ�ָ��ٵ���Ҫ���̣�
	void groupMatch();

	//��ÿһ��Ŀ�� track��һ����ΧcomBlob�ڽ�������λ�õ�����
	void doSearch(myObject* track, CvRect comBlob, float* bestScore, SampleConfig* bestConfig);

	//��ǰ������б��
	void labelPixel(CvRect blob, int tracknumInGroup, int* trackIndex, int* goodIndex, int* Depth, float* occludedW, float* w_sum);//also need trackLabel

	//�õ��Ե�ǰλ�õ�����
	float getLikelihood(myObject* track, SampleConfig sample);

	//������λ�õ��޶�
	void limit(int x1, int y1, int x2, int y2, CvSize S);

	//����ɶԸ���Ŀ��ĸ���֮�󣬶�ÿ��Ŀ���״̬��ģ�������£�ʵ�ָ��ٵ���Ҫ���̣�
	void conclude();

	//����ģ��
	void updateObject(myObject* track, SampleConfig bestConfig, int trackIndex);

	//��ʼ���µ�Ŀ��
	void initialObject(myObject* newObject, myBlob newBlob, Feature** featureMap);

	//���ͼ��ߴ�
	int iW, iH;

	//���ٽ�� ����label���
	IplImage* trackLabel;

public:
	int testnum;
	int testbnum;
};

#endif