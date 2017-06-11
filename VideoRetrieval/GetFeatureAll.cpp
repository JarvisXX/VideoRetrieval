//函数功能：用于对多条轨迹进行特征抽取，转化为维数相同的特征，对应头文件为GetFeatureAll.h
//参数说明 by Keren Fu
//complete_trajectory: 完整用于聚类的轨迹
//w：长度为5的double型数组，对应各个特征的权值
//	w[0] for 平均速度幅值
//	w[1] for 平均速度向量
//	w[2] for 平均位置
//	w[3] for 起点终点向量
//	w[4] for 轨迹描述点
//FeatAll: 提取到特征向量，每条轨迹对应一个vector<double>
#include "clude.h"
#include "GetFeatureAll.h"
#include "GetFeatureEach.h"

void GetFeatureALL(vector<vector<CvPoint>> &complete_trajectory, double *w, vector<vector<double>> &FeatAll)
{
	int length = complete_trajectory.size();//轨迹条数
	for (int i = 0; i <length; i++)
	{
		vector<double> Feat;
		GetFeatureEach(complete_trajectory[i], w, Feat);
		FeatAll.push_back(Feat);//将得到的vector特征存入featureall
	}
}

/*
用于对多条轨迹进行特征抽取，分别抽取速度特征和轨迹特征。
*/
void GetFeatureAllRespective(vector<vector<CvPoint>> &complete_trajectory, double *w, vector<vector<double>> &FeatAllV, vector<vector<double>> &FeatAllTraj)
{
	int length = complete_trajectory.size();//轨迹条数
	for (int i = 0; i <length; i++)
	{
		vector<double> FeatV, FeatTraj;
		getFeatureEachRespective(complete_trajectory[i], w, FeatV, FeatTraj);
		FeatAllV.push_back(FeatV);//将得到的vector特征分别存入对应的featureall
		FeatAllTraj.push_back(FeatTraj);
	}
}