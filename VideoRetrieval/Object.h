#ifndef OBJECT_H
#define OBJECT_H

#include"clude.h"
#include"Rule.h"

//对目标模型的定义  //目标所处状态
enum objectState{ obj_invalid = -1, obj_new, obj_reappeare, obj_tracked, obj_missing, obj_disappear, obj_gone };
enum linedirection{ init_direction, Left, Right, Other };


//feture的尺存
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
	//当找到当前帧的匹配特征点后 对模型中记录的特征点的位置 做一定的更新。
	void update(Feature* f)
	{
		x = f->x;
		y = f->y;
		traj.push_back(cvPoint(x, y));
		rx = float(rx * alpha_fx + f->rx * alpha_fx);
		ry = float(ry * alpha_fx + f->ry * alpha_fx);
	}
	//图像坐标
	float x;
	float y;
	//相对目标质心坐标
	float rx;
	float ry;
	//平均灰度
	float mean;
	//平均RGB向量
	CvScalar meanRGB;
	//方差
	float sumXX;
	//记录下周围大小为l_feature
	CvMat* patch;
	//健康度    维护健康度记录，用来表示这一特征是否足够稳健进而在目标跟踪中发挥作用，或非常不稳定而应该丢掉
	int health;
	//记录的轨迹 这里记录特征点轨迹 只为显示输出用，实际工作并不需要
	vector<CvPoint> traj;
};

//入侵警报结构体


struct myBlob
{
	CvRect outRect;
	CvRect monRect;
	int medianX, medianY;
	int ID;
};
//目标模型 当然这个非常重要 在blob tracking的框架下 模型非常重要
class myObject
{
public:
	~myObject();
	//包含外接框和利用矩算出的框、质心的结构体 
	//除了外接框 其他只在初次建立物体时有用处 
	//外接框之后用来输出目标的包围框（但实质已不是外接框）
	myBlob blob;
	float cur_x, cur_y, cur_w, cur_h;   //边界相对于centroid的位置
	int ID;                          //唯一ID
	int item_id_;
	int cx, cy;                       //中心
	float mean_v;                    //平均线速度
	float pre_mean_v;
	float now_mean_v;
	float arrayv[20];
	float now_v;
	int count_mean;
	enum catagory catagories;
	myObject(){    //初始化参数 多为累计值和标记值
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
	int					health;	       //健康度决定是否稳定 是否丢弃
	int					lifeTime;      //存在时间
	float				searchRangeX, searchRangeY;   //搜索范围


	objectState			state;

	vector<CvPoint>*	trajectory;
	vector<objectState>*	stateSeq;
	vector<CvPoint>*	curwhSeq;   //长宽队列
	float				rav, maxrav;    //每秒运动速度
	int					stopTime;      //目标已经停止的秒数，用于判断的关键参数  
	int warned;
	//报警相关开关变量
	bool Invasion_OPEN;
	bool Stay_OPEN;
	bool Appearea_OPEN;
	bool Wander_OPEN;
	bool Leave_OPEN;
	bool Acclerate_OPEN;

	//入侵检测相关变量
	int areaID;
	bool flagInvasion;


	//滞留检测相关变量
	int stayareaID;
	bool flagStay;


	//突然出现检测相关变量
	int appearareaID;
	bool flagAppear;


	//过线检测相关变量
	int Warn[5];
	linedirection linePre[5];
	linedirection lineCur[5];

	//徘徊检测相关变量
	int wanderareaID;
	bool flagWander;

	//这些变量用来记录apperance model的数据
	int w_map, h_map;
	CvMat* current_map, *current_weight;    //RGB模型 ，前景概率
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
	int ID; //目标ID
	long startTime;   //进入告警区域时间
	long endTime;     //离开告警区域时间
	bool invasionFlag;//进入告警区域标识
	bool invasion;    //是否进入过告警区域
};
#endif