#include "clude.h"
#include "MeanShiftClustering.h"
#include "MeanShiftFunc.h"
/*
**points为样本点集合。
**width为带宽
**thresh 为控制聚类收敛的参数
**centers为中心集合。
**labels为样本类标集合，它的元素个数与样本点的个数一致，且位置一一对应。
*/
void meanshift_cluster(std::vector< std::vector<double> > &points, double width, double thresh, std::vector< std::vector<double> > &centers, std::vector<int> &labels)
{
	const int num_of_dimension = points[0].size();//样本点的维数
	const double clustering_threshold = thresh;
	double sigma = width;//带宽

	MeanShiftClustering mean_shift_cluster(points, num_of_dimension, sigma);
	centers = mean_shift_cluster.Clustering(labels, clustering_threshold);
}

//检测是否异常，一次检测时，featureAll里可能有不止一条轨迹的特征
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
				isNormal.push_back(1);//1：正常
				break;//跳出内层循环，执行后面的语句
			}
		}
		if (j >= size_cenetrs)
		{
			isNormal.push_back(0);//如果一条轨迹和所有聚类顶点的距离都大于threshhold，则为异常
			detectedAbnormal = true;
		}


	}
	return detectedAbnormal;
}

//return 1:正常；return 0:异常
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

	if (i >= centers_size)//如果一条轨迹和所有聚类顶点的距离都大于threshhold，则为异常
		return 0;
	else
		return 1;
}

//欧式距离
double EuclideanDistance(std::vector<double> p1, std::vector<double> p2)
{
	double distance = 0;
	int dim = p1.size();
	for (int i = 0; i < dim; i++){
		distance += pow(p1[i] - p2[i], 2.0);
	}
	return sqrt(distance);
}
