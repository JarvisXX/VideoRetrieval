//函数功能：用于对单条轨迹进行特征抽取，转化为维数一定的特征，不同特征(如速度，位置等)为串联形式，对应头文件为GetFeatureEach.h
//参数说明 by Keren Fu
//Traj: 单条轨迹
//w：长度为5的double型数组，对应各个特征的权值
//	w[0] for 平均速度幅值
//	w[1] for 平均速度向量
//	w[2] for 平均位置
//	w[3] for 起点终点向量
//	w[4] for 轨迹描述点
//Feat: 提取到单条轨迹的特征向量
#include "clude.h"
#include "GetFeatureEach.h"

void GetFeatureEach(vector<CvPoint> &Traj, double *w, vector<double> &Feat)
{
	int length = Traj.size();
	double sum_x = 0;
	double sum_y = 0;
	/////////////平均速度
	double w0 = w[0];
	for (int i = 1; i<length; i++)
	{
		sum_x = sum_x + sqrt((Traj[i].x - Traj[i - 1].x)*(Traj[i].x - Traj[i - 1].x) + (Traj[i].y - Traj[i - 1].y)*(Traj[i].y - Traj[i - 1].y));//速度幅值
	}
	sum_x = sum_x / (length - 1);
	Feat.push_back(sum_x*w0);//将特征指针加入堆栈
	/////////////平均速度向量
	sum_x = 0; sum_y = 0;
	double w1 = w[1];
	for (int i = 1; i<length; i++)
	{
		sum_x = sum_x + (Traj[i].x - Traj[i - 1].x);//x速度
		sum_y = sum_y + (Traj[i].y - Traj[i - 1].y);//y速度
	}
	sum_x = sum_x / (length - 1);
	sum_y = sum_y / (length - 1);
	Feat.push_back(sum_x*w1);//将特征指针加入堆栈
	Feat.push_back(sum_y*w1);//将特征指针加入堆栈
	//////////////平均位置
	sum_x = 0; sum_y = 0;
	double w2 = w[2];
	for (int i = 0; i <length; i++)
	{
		sum_x = sum_x + Traj[i].x;//x位置
		sum_y = sum_y + Traj[i].y;//y位置
	}
	sum_x = sum_x / length;
	sum_y = sum_y / length;
	Feat.push_back(sum_x*w2);//将特征指针加入堆栈
	Feat.push_back(sum_y*w2);//将特征指针加入堆栈
	///////////////起点终点向量
	double w3 = w[3];
	Feat.push_back((Traj[length - 1].x - Traj[0].x)*w3);//将特征指针加入堆栈
	Feat.push_back((Traj[length - 1].y - Traj[0].y)*w3);//将特征指针加入堆栈
	///////////////轨迹描述点
	double w4 = w[4];
	double num_seg = 5;//将轨迹分为几段
	double step = (length - 1) / num_seg;
	for (int i = 0; i <= num_seg; i++)//坐标点数为段数+1
	{
		double up = ceil(i*step);     //向上取整   
		double down = floor(i*step);   //向下取整 
		if (up != down)
		{
			Feat.push_back((Traj[up].x + double(Traj[down].x)) / 2 * w4);//插值坐标点x坐标
			Feat.push_back((Traj[up].y + double(Traj[down].y)) / 2 * w4);//插值坐标点y坐标
		}
		else
		{
			Feat.push_back(Traj[i*step].x*w4);//坐标点x坐标
			Feat.push_back(Traj[i*step].y*w4);//坐标点y坐标
		}
	}
}

/*
*分别得到一个轨迹的速度特征和轨迹特征
*   Traj: 单条轨迹
*   w：长度为5的double型数组，对应各个特征的权值
*   ////以下的为速度特征权重
*	w[0] for 平均速度幅值
*	w[1] for 平均速度向量
*   ////以下的为轨迹特征的权重
*	w[2] for 平均位置
*	w[3] for 起点终点向量
*	w[4] for 轨迹描述点
*   FeatV: 提取到单条轨迹的速度特征向量
*   FeatTraj：提取到的单条轨迹的轨迹特征向量
*/
void getFeatureEachRespective(vector<CvPoint> &Traj, double *w, vector<double> &FeatV, vector<double> &FeatTraj)
{

	int length = Traj.size();
	double sum_x = 0;
	double sum_y = 0;
	/////////////平均速度
	double w0 = w[0];
	for (int i = 1; i<length; i++)
	{
		sum_x = sum_x + sqrt((Traj[i].x - Traj[i - 1].x)*(Traj[i].x - Traj[i - 1].x) + (Traj[i].y - Traj[i - 1].y)*(Traj[i].y - Traj[i - 1].y));//速度幅值
	}
	sum_x = sum_x / (length - 1);
	FeatV.push_back(sum_x*w0);//将特征指针加入堆栈
	/////////////平均速度向量
	sum_x = 0; sum_y = 0;
	double w1 = w[1];
	for (int i = 1; i<length; i++)
	{
		sum_x = sum_x + (Traj[i].x - Traj[i - 1].x);//x速度
		sum_y = sum_y + (Traj[i].y - Traj[i - 1].y);//y速度
	}
	sum_x = sum_x / (length - 1);
	sum_y = sum_y / (length - 1);
	FeatV.push_back(sum_x*w1);//将特征指针加入堆栈
	FeatV.push_back(sum_y*w1);//将特征指针加入堆栈
	//////////////平均位置
	sum_x = 0; sum_y = 0;
	double w2 = w[2];
	for (int i = 0; i <length; i++)
	{
		sum_x = sum_x + Traj[i].x;//x位置
		sum_y = sum_y + Traj[i].y;//y位置
	}
	sum_x = sum_x / length;
	sum_y = sum_y / length;
	FeatTraj.push_back(sum_x*w2);//将特征指针加入堆栈
	FeatTraj.push_back(sum_y*w2);//将特征指针加入堆栈
	///////////////起点终点向量
	double w3 = w[3];
	FeatTraj.push_back((Traj[length - 1].x - Traj[0].x)*w3);//将特征指针加入堆栈
	FeatTraj.push_back((Traj[length - 1].y - Traj[0].y)*w3);//将特征指针加入堆栈
	///////////////轨迹描述点
	double w4 = w[4];
	double num_seg = 5;//将轨迹分为几段
	double step = (length - 1) / num_seg;
	for (int i = 0; i <= num_seg; i++)//坐标点数为段数+1
	{
		double up = ceil(i*step);     //向上取整   
		double down = floor(i*step);   //向下取整 
		if (up != down)
		{
			FeatTraj.push_back((Traj[up].x + double(Traj[down].x)) / 2 * w4);//插值坐标点x坐标
			FeatTraj.push_back((Traj[up].y + double(Traj[down].y)) / 2 * w4);//插值坐标点y坐标
		}
		else
		{
			FeatTraj.push_back(Traj[i*step].x*w4);//坐标点x坐标
			FeatTraj.push_back(Traj[i*step].y*w4);//坐标点y坐标
		}
	}

}