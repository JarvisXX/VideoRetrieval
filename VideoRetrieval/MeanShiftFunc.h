#pragma once
/*
**pointsΪ�����㼯�ϡ�
**widthΪ����
**thresh Ϊ���ƾ��������Ĳ���
**centersΪ���ļ��ϡ�
**labelsΪ������꼯�ϣ�����Ԫ�ظ�����������ĸ���һ�£���λ��һһ��Ӧ��
*/
void meanshift_cluster(std::vector< std::vector<double> > &points, double width, double thresh, std::vector< std::vector<double> > &centers, std::vector<int> &labels);

/*
*����Ƿ����쳣
*/
bool detect_func(vector<vector<double>> &centers, vector<vector<double>> &featureALL, double threshhold, vector<int> &isNormal);//0��������1���쳣

//return 1:������return 0:�쳣
int detect_func_one_traj(vector<vector<double>> &centers, vector<double> &oneFeature, double thresh);

double EuclideanDistance(std::vector<double> p1, std::vector<double> p2);