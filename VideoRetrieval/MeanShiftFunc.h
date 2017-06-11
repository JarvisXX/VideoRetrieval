#pragma once
/*
**points为样本点集合。
**width为带宽
**thresh 为控制聚类收敛的参数
**centers为中心集合。
**labels为样本类标集合，它的元素个数与样本点的个数一致，且位置一一对应。
*/
void meanshift_cluster(std::vector< std::vector<double> > &points, double width, double thresh, std::vector< std::vector<double> > &centers, std::vector<int> &labels);

/*
*检测是否有异常
*/
bool detect_func(vector<vector<double>> &centers, vector<vector<double>> &featureALL, double threshhold, vector<int> &isNormal);//0：正常，1：异常

//return 1:正常；return 0:异常
int detect_func_one_traj(vector<vector<double>> &centers, vector<double> &oneFeature, double thresh);

double EuclideanDistance(std::vector<double> p1, std::vector<double> p2);