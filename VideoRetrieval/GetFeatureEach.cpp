//�������ܣ����ڶԵ����켣����������ȡ��ת��Ϊά��һ������������ͬ����(���ٶȣ�λ�õ�)Ϊ������ʽ����Ӧͷ�ļ�ΪGetFeatureEach.h
//����˵�� by Keren Fu
//Traj: �����켣
//w������Ϊ5��double�����飬��Ӧ����������Ȩֵ
//	w[0] for ƽ���ٶȷ�ֵ
//	w[1] for ƽ���ٶ�����
//	w[2] for ƽ��λ��
//	w[3] for ����յ�����
//	w[4] for �켣������
//Feat: ��ȡ�������켣����������
#include "clude.h"
#include "GetFeatureEach.h"

void GetFeatureEach(vector<CvPoint> &Traj, double *w, vector<double> &Feat)
{
	int length = Traj.size();
	double sum_x = 0;
	double sum_y = 0;
	/////////////ƽ���ٶ�
	double w0 = w[0];
	for (int i = 1; i<length; i++)
	{
		sum_x = sum_x + sqrt((Traj[i].x - Traj[i - 1].x)*(Traj[i].x - Traj[i - 1].x) + (Traj[i].y - Traj[i - 1].y)*(Traj[i].y - Traj[i - 1].y));//�ٶȷ�ֵ
	}
	sum_x = sum_x / (length - 1);
	Feat.push_back(sum_x*w0);//������ָ������ջ
	/////////////ƽ���ٶ�����
	sum_x = 0; sum_y = 0;
	double w1 = w[1];
	for (int i = 1; i<length; i++)
	{
		sum_x = sum_x + (Traj[i].x - Traj[i - 1].x);//x�ٶ�
		sum_y = sum_y + (Traj[i].y - Traj[i - 1].y);//y�ٶ�
	}
	sum_x = sum_x / (length - 1);
	sum_y = sum_y / (length - 1);
	Feat.push_back(sum_x*w1);//������ָ������ջ
	Feat.push_back(sum_y*w1);//������ָ������ջ
	//////////////ƽ��λ��
	sum_x = 0; sum_y = 0;
	double w2 = w[2];
	for (int i = 0; i <length; i++)
	{
		sum_x = sum_x + Traj[i].x;//xλ��
		sum_y = sum_y + Traj[i].y;//yλ��
	}
	sum_x = sum_x / length;
	sum_y = sum_y / length;
	Feat.push_back(sum_x*w2);//������ָ������ջ
	Feat.push_back(sum_y*w2);//������ָ������ջ
	///////////////����յ�����
	double w3 = w[3];
	Feat.push_back((Traj[length - 1].x - Traj[0].x)*w3);//������ָ������ջ
	Feat.push_back((Traj[length - 1].y - Traj[0].y)*w3);//������ָ������ջ
	///////////////�켣������
	double w4 = w[4];
	double num_seg = 5;//���켣��Ϊ����
	double step = (length - 1) / num_seg;
	for (int i = 0; i <= num_seg; i++)//�������Ϊ����+1
	{
		double up = ceil(i*step);     //����ȡ��   
		double down = floor(i*step);   //����ȡ�� 
		if (up != down)
		{
			Feat.push_back((Traj[up].x + double(Traj[down].x)) / 2 * w4);//��ֵ�����x����
			Feat.push_back((Traj[up].y + double(Traj[down].y)) / 2 * w4);//��ֵ�����y����
		}
		else
		{
			Feat.push_back(Traj[i*step].x*w4);//�����x����
			Feat.push_back(Traj[i*step].y*w4);//�����y����
		}
	}
}

/*
*�ֱ�õ�һ���켣���ٶ������͹켣����
*   Traj: �����켣
*   w������Ϊ5��double�����飬��Ӧ����������Ȩֵ
*   ////���µ�Ϊ�ٶ�����Ȩ��
*	w[0] for ƽ���ٶȷ�ֵ
*	w[1] for ƽ���ٶ�����
*   ////���µ�Ϊ�켣������Ȩ��
*	w[2] for ƽ��λ��
*	w[3] for ����յ�����
*	w[4] for �켣������
*   FeatV: ��ȡ�������켣���ٶ���������
*   FeatTraj����ȡ���ĵ����켣�Ĺ켣��������
*/
void getFeatureEachRespective(vector<CvPoint> &Traj, double *w, vector<double> &FeatV, vector<double> &FeatTraj)
{

	int length = Traj.size();
	double sum_x = 0;
	double sum_y = 0;
	/////////////ƽ���ٶ�
	double w0 = w[0];
	for (int i = 1; i<length; i++)
	{
		sum_x = sum_x + sqrt((Traj[i].x - Traj[i - 1].x)*(Traj[i].x - Traj[i - 1].x) + (Traj[i].y - Traj[i - 1].y)*(Traj[i].y - Traj[i - 1].y));//�ٶȷ�ֵ
	}
	sum_x = sum_x / (length - 1);
	FeatV.push_back(sum_x*w0);//������ָ������ջ
	/////////////ƽ���ٶ�����
	sum_x = 0; sum_y = 0;
	double w1 = w[1];
	for (int i = 1; i<length; i++)
	{
		sum_x = sum_x + (Traj[i].x - Traj[i - 1].x);//x�ٶ�
		sum_y = sum_y + (Traj[i].y - Traj[i - 1].y);//y�ٶ�
	}
	sum_x = sum_x / (length - 1);
	sum_y = sum_y / (length - 1);
	FeatV.push_back(sum_x*w1);//������ָ������ջ
	FeatV.push_back(sum_y*w1);//������ָ������ջ
	//////////////ƽ��λ��
	sum_x = 0; sum_y = 0;
	double w2 = w[2];
	for (int i = 0; i <length; i++)
	{
		sum_x = sum_x + Traj[i].x;//xλ��
		sum_y = sum_y + Traj[i].y;//yλ��
	}
	sum_x = sum_x / length;
	sum_y = sum_y / length;
	FeatTraj.push_back(sum_x*w2);//������ָ������ջ
	FeatTraj.push_back(sum_y*w2);//������ָ������ջ
	///////////////����յ�����
	double w3 = w[3];
	FeatTraj.push_back((Traj[length - 1].x - Traj[0].x)*w3);//������ָ������ջ
	FeatTraj.push_back((Traj[length - 1].y - Traj[0].y)*w3);//������ָ������ջ
	///////////////�켣������
	double w4 = w[4];
	double num_seg = 5;//���켣��Ϊ����
	double step = (length - 1) / num_seg;
	for (int i = 0; i <= num_seg; i++)//�������Ϊ����+1
	{
		double up = ceil(i*step);     //����ȡ��   
		double down = floor(i*step);   //����ȡ�� 
		if (up != down)
		{
			FeatTraj.push_back((Traj[up].x + double(Traj[down].x)) / 2 * w4);//��ֵ�����x����
			FeatTraj.push_back((Traj[up].y + double(Traj[down].y)) / 2 * w4);//��ֵ�����y����
		}
		else
		{
			FeatTraj.push_back(Traj[i*step].x*w4);//�����x����
			FeatTraj.push_back(Traj[i*step].y*w4);//�����y����
		}
	}

}