#include "clude.h"
#include "MeanShiftClustering.h"
#include "MeanShiftFunc.h"
/*
**pointsΪ�����㼯�ϡ�
**widthΪ����
**thresh Ϊ���ƾ��������Ĳ���
**centersΪ���ļ��ϡ�
**labelsΪ������꼯�ϣ�����Ԫ�ظ�����������ĸ���һ�£���λ��һһ��Ӧ��
*/
void meanshift_cluster(std::vector< std::vector<double> > &points, double width, double thresh, std::vector< std::vector<double> > &centers, std::vector<int> &labels)
{
	const int num_of_dimension = points[0].size();//�������ά��
	const double clustering_threshold = thresh;
	double sigma = width;//����

	MeanShiftClustering mean_shift_cluster(points, num_of_dimension, sigma);
	centers = mean_shift_cluster.Clustering(labels, clustering_threshold);
}

//����Ƿ��쳣��һ�μ��ʱ��featureAll������в�ֹһ���켣������
bool detect_func(vector<vector<double>> &centers, vector<vector<double>> &featureAll, double threshhold, vector<int> &isNormal)
{
	bool detectedAbnormal = false;
	int size_cenetrs = centers.size();
	int size_featureAll = featureAll.size();
	for (int i = 0; i < size_featureAll; ++i)
	{
		int j = 0;
		for (j = 0; j < size_cenetrs; ++j)
		{
			int distance = EuclideanDistance(featureAll[i], centers[j]);
			if (distance < threshhold)
			{
				isNormal.push_back(1);//1������
				break;//�����ڲ�ѭ����ִ�к�������
			}
		}
		if (j >= size_cenetrs)
		{
			isNormal.push_back(0);//���һ���켣�����о��ඥ��ľ��붼����threshhold����Ϊ�쳣
			detectedAbnormal = true;
		}


	}
	return detectedAbnormal;
}

//return 1:������return 0:�쳣
int detect_func_one_traj(vector<vector<double>> &centers, vector<double> &oneFeature, double thresh)
{
	int centers_size = centers.size();
	int i = 0;
	for (i = 0; i < centers_size; ++i)
	{
		int distance = EuclideanDistance(oneFeature, centers[i]);
		if (distance < thresh)
		{
			return 1;
		}
	}

	if (i >= centers_size)//���һ���켣�����о��ඥ��ľ��붼����threshhold����Ϊ�쳣
		return 0;
	else
		return 1;
}

//ŷʽ����
double EuclideanDistance(std::vector<double> p1, std::vector<double> p2)
{
	double distance = 0;
	int dim = p1.size();
	for (int i = 0; i < dim; i++){
		distance += pow(p1[i] - p2[i], 2.0);
	}
	return sqrt(distance);
}
