#include "clude.h"
#include "MeanShiftClustering.h"

MeanShiftClustering::MeanShiftClustering(std::vector< std::vector<double> >&points,
	int dim,
	double sigma)
	: points_(points), dim_(dim), sigma_(sigma)
{
	num_points_ = points_.size();
}

MeanShiftClustering::~MeanShiftClustering()
{
}

double MeanShiftClustering::EuclideanDistance(std::vector<double> p1,
	std::vector<double> p2)
{
	double distance = 0;
	for (int i = 0; i < dim_; i++){
		distance += pow(p1[i] - p2[i], 2.0);
	}
	return sqrt(distance);
}

std::vector<double> MeanShiftClustering::MeanShiftProcedure(std::vector<double> initX,
	double threshold)
{
	std::vector<double> node(initX);
	std::vector<double> last_node(dim_, 100);

	while (true){
		std::vector<double> sum_gx(dim_, 0.0);
		double sum_g = 0;
		for (int i = 0; i < num_points_; i++){
			double dist = EuclideanDistance(node, points_[i]);
			dist = sigma_ * dist * dist;
			double g_i = exp(-dist);
			sum_g += g_i;
			for (int j = 0; j < dim_; j++){
				sum_gx[j] += g_i * points_[i][j];
			}
		}
		for (int i = 0; i < dim_; i++){
			node[i] = sum_gx[i] / sum_g;
		}
		if (EuclideanDistance(node, last_node) < threshold){
			break;
		}
		else{
			last_node = node;
		}
	}
	return node;
}

std::vector<std::vector<double>> MeanShiftClustering::Clustering(std::vector<int> &indices, double threshold)
{
	if (indices.size() != points_.size()){
		indices.resize(points_.size());
	}
	std::vector< std::vector<double> > clusters;//cluserts没有初始化，一开始里边没有值，后续的调用clusters.size()会不会出错
	for (int i = 0; i < num_points_; i++){
		std::vector<double> node = MeanShiftProcedure(points_[i], 3e-5);
		bool is_new_cluster(true);
		for (int j = 0; j < (int)clusters.size(); j++){
			if (EuclideanDistance(clusters[j], node) < threshold){
				indices[i] = j;
				is_new_cluster = false;
				break;
			}
		}
		if (is_new_cluster){
			indices[i] = clusters.size();
			clusters.push_back(node);
		}
	}
	return clusters;
}
