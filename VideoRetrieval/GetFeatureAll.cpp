//�������ܣ����ڶԶ����켣����������ȡ��ת��Ϊά����ͬ����������Ӧͷ�ļ�ΪGetFeatureAll.h
//����˵�� by Keren Fu
//complete_trajectory: �������ھ���Ĺ켣
//w������Ϊ5��double�����飬��Ӧ����������Ȩֵ
//	w[0] for ƽ���ٶȷ�ֵ
//	w[1] for ƽ���ٶ�����
//	w[2] for ƽ��λ��
//	w[3] for ����յ�����
//	w[4] for �켣������
//FeatAll: ��ȡ������������ÿ���켣��Ӧһ��vector<double>
#include "clude.h"
#include "GetFeatureAll.h"
#include "GetFeatureEach.h"

void GetFeatureALL(vector<vector<CvPoint>> &complete_trajectory, double *w, vector<vector<double>> &FeatAll)
{
	int length = complete_trajectory.size();//�켣����
	for (int i = 0; i <length; i++)
	{
		vector<double> Feat;
		GetFeatureEach(complete_trajectory[i], w, Feat);
		FeatAll.push_back(Feat);//���õ���vector��������featureall
	}
}

/*
���ڶԶ����켣����������ȡ���ֱ��ȡ�ٶ������͹켣������
*/
void GetFeatureAllRespective(vector<vector<CvPoint>> &complete_trajectory, double *w, vector<vector<double>> &FeatAllV, vector<vector<double>> &FeatAllTraj)
{
	int length = complete_trajectory.size();//�켣����
	for (int i = 0; i <length; i++)
	{
		vector<double> FeatV, FeatTraj;
		getFeatureEachRespective(complete_trajectory[i], w, FeatV, FeatTraj);
		FeatAllV.push_back(FeatV);//���õ���vector�����ֱ�����Ӧ��featureall
		FeatAllTraj.push_back(FeatTraj);
	}
}