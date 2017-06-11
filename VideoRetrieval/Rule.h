#ifndef rule_H
#define rule_H

typedef struct
{

	int Rule_foregroundXsize;                 // 前景区域大小X
	int Rule_foregroundYsize;				   // 前景区域大小Y
	int areashape[5];						// 区域形状标记  0:正方形；1：多边形

	double PointX[5][20];	            //记录选区横坐标
	double PointY[5][20];              // 记录选区纵坐标，保存区域顶点顺时针的位置

	int invasiontime;                  // 入侵到区域中的容忍时间

	int invasion_flag;                // 标识变量，1代表设置了入侵检测，0代表没有
	int linecross_flag;               //1代表绊线检测
	int wander_flag;                  //1代表徘徊
	int remove_flag;                  //1代表移走
	int suddenappear_flag;            //1代表突然出现
	int suddenAccelerate_flag;        //1代表突然加速    
	int route_flag;                   //1代表路径检测
	int stay_flag;                    //1代表滞留检测
	int dropbag_flag;                 //1代表遗留检测

	//double LinePointX[5][2];	          //记录绊线检测直线的横坐标，最多支持5条线
	//double LinePointY[5][2];             //记录绊线检测直线的纵坐标，最多支持5条线

	double LineStartX[5];
	double LineStartY[5];
	double LineEndX[5];
	double LineEndY[5];
	int LineDirection[5];                //记录绊线的方向，1为从1区到2区，2为从2区到1区

	int  NumOfArea;            //设置的区域数目
	int  NumOfLine;            //设置的绊线数目
	int  NumPointArea[5];      //每个区域内的顶点数


}rule_son;



typedef struct
{
	rule_son Rule_invasion;                 //入侵
	rule_son Rule_linecross;                //绊线
	rule_son Rule_wander;                   //徘徊
	rule_son Rule_remove;                   //移走
	rule_son Rule_suddenappear;             //突然出现
	rule_son Rule_suddenAccelerate;         //突然加速
	rule_son Rule_route;	                  //路径
	rule_son Rule_stay;                     //滞留
	rule_son Rule_dropbag;                  //遗留（丢包）

}rule;                                        // 第一路规则结构体


typedef struct
{
	int points[10];
}Points;

enum catagory{vehicle,pedestrian};

#endif