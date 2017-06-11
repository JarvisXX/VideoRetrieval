#ifndef OBJECT_H
#define OBJECT_H

#include"clude.h"
#include"Rule.h"

//��Ŀ��ģ�͵Ķ���  //Ŀ������״̬
enum objectState{ obj_invalid = -1, obj_new, obj_reappeare, obj_tracked, obj_missing, obj_disappear, obj_gone };
enum linedirection{ init_direction, Left, Right, Other };


//feture�ĳߴ�
#define a_feature 4
#define l_feature a_feature*2+1

#define alpha_fx 0.5

class Feature
{
public:
	Feature()
	{
		patch = NULL;
	};
	~Feature()
	{
		if (patch != NULL)
		{
			cvReleaseMat(&patch);
		}
		traj.clear();
	};
	//���ҵ���ǰ֡��ƥ��������� ��ģ���м�¼���������λ�� ��һ���ĸ��¡�
	void update(Feature* f)
	{
		x = f->x;
		y = f->y;
		traj.push_back(cvPoint(x, y));
		rx = float(rx * alpha_fx + f->rx * alpha_fx);
		ry = float(ry * alpha_fx + f->ry * alpha_fx);
	}
	//ͼ������
	float x;
	float y;
	//���Ŀ����������
	float rx;
	float ry;
	//ƽ���Ҷ�
	float mean;
	//ƽ��RGB����
	CvScalar meanRGB;
	//����
	float sumXX;
	//��¼����Χ��СΪl_feature
	CvMat* patch;
	//������    ά�������ȼ�¼��������ʾ��һ�����Ƿ��㹻�Ƚ�������Ŀ������з������ã���ǳ����ȶ���Ӧ�ö���
	int health;
	//��¼�Ĺ켣 �����¼������켣 ֻΪ��ʾ����ã�ʵ�ʹ���������Ҫ
	vector<CvPoint> traj;
};

//���־����ṹ��


struct myBlob
{
	CvRect outRect;
	CvRect monRect;
	int medianX, medianY;
	int ID;
};
//Ŀ��ģ�� ��Ȼ����ǳ���Ҫ ��blob tracking�Ŀ���� ģ�ͷǳ���Ҫ
class myObject
{
public:
	~myObject();
	//������ӿ�����þ�����Ŀ����ĵĽṹ�� 
	//������ӿ� ����ֻ�ڳ��ν�������ʱ���ô� 
	//��ӿ�֮���������Ŀ��İ�Χ�򣨵�ʵ���Ѳ�����ӿ�
	myBlob blob;
	float cur_x, cur_y, cur_w, cur_h;   //�߽������centroid��λ��
	int ID;                          //ΨһID
	int item_id_;
	int cx, cy;                       //����
	float mean_v;                    //ƽ�����ٶ�
	float pre_mean_v;
	float now_mean_v;
	float arrayv[20];
	float now_v;
	int count_mean;
	enum catagory catagories;
	myObject(){    //��ʼ������ ��Ϊ�ۼ�ֵ�ͱ��ֵ
		for (int i = 0; i < 5; ++i) { linePre[i] = init_direction; }
		for (int i = 0; i < 5; ++i) { lineCur[i] = init_direction; }
		for (int i = 0; i < 5; ++i) { Warn[i] = 0; }
		for (int i = 0; i<20; i++)arrayv[i] = 10;
		lifeTime = 1;
		state = obj_new;
		health = 0;
		mean_v = 0;
		searchRangeX = 0; searchRangeY = 0;
		areaID = -1; flagInvasion = false;
		count_mean = 0;
		pre_mean_v = 100;
		now_mean_v = 0;
		now_v = 0;
		rav = -1;
		stopTime = 0;
	
	

		Invasion_OPEN = false;
		Stay_OPEN = false;
		Appearea_OPEN = false;
		Wander_OPEN = false;
		Leave_OPEN = false;
		Acclerate_OPEN = false;
		trajectory = NULL;
		stateSeq = NULL;
		curwhSeq = NULL;
		catagories=pedestrian;

	};
	int					health;	       //�����Ⱦ����Ƿ��ȶ� �Ƿ���
	int					lifeTime;      //����ʱ��
	float				searchRangeX, searchRangeY;   //������Χ


	objectState			state;

	vector<CvPoint>*	trajectory;
	vector<objectState>*	stateSeq;
	vector<CvPoint>*	curwhSeq;   //�������
	float				rav, maxrav;    //ÿ���˶��ٶ�
	int					stopTime;      //Ŀ���Ѿ�ֹͣ�������������жϵĹؼ�����  
	int warned;
	//������ؿ��ر���
	bool Invasion_OPEN;
	bool Stay_OPEN;
	bool Appearea_OPEN;
	bool Wander_OPEN;
	bool Leave_OPEN;
	bool Acclerate_OPEN;

	//���ּ����ر���
	int areaID;
	bool flagInvasion;


	//���������ر���
	int stayareaID;
	bool flagStay;


	//ͻȻ���ּ����ر���
	int appearareaID;
	bool flagAppear;


	//���߼����ر���
	int Warn[5];
	linedirection linePre[5];
	linedirection lineCur[5];

	//�ǻ������ر���
	int wanderareaID;
	bool flagWander;

	//��Щ����������¼apperance model������
	int w_map, h_map;
	CvMat* current_map, *current_weight;    //RGBģ�� ��ǰ������
	CvMat* map, *weight;
	float sum_weight, weight_observed;
	//feature 
	vector<Feature*> featureList;

	void deObject(void);
};

class ObjectList
{
public:
	ObjectList();
	virtual ~ObjectList();
	vector<myObject>	Objvec;
};

class Trajectory
{
public:
	Trajectory()
	{
		trj = NULL;
		trjState = NULL;
	};
	~Trajectory()
	{
		if (trj != NULL)
		{
			trj->clear();
			delete trj;
		}
		if (trjState != NULL)
		{
			trjState->clear();
			delete trjState;
		}
	}
	vector<CvPoint>* trj;
	vector<objectState>* trjState;
	int	ID;
};

class PathTraj
{
public:
	~PathTraj()
	{
		if (trj != NULL)
		{
			trj->clear();
			delete trj;
		}

	};
	PathTraj()
	{
		trj = NULL;
	};
	vector<CvPoint>* trj;
	int ID;
	int lastTime;
};
struct WarningInvasion
{
	int ID; //Ŀ��ID
	long startTime;   //����澯����ʱ��
	long endTime;     //�뿪�澯����ʱ��
	bool invasionFlag;//����澯�����ʶ
	bool invasion;    //�Ƿ������澯����
};
#endif