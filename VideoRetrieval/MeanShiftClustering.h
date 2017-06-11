#ifndef MEAN_SHIFT_CLUSTERING
#define MEAN_SHIFT_CLUSTERING

#include <iostream>
#include <vector>
#include <cmath>
class MeanShiftClustering
{
public:
	MeanShiftClustering(std::vector< std::vector<double> >&points,
		int dim,
		double sigma = 0.1);
	~MeanShiftClustering();
	std::vector<std::vector<double>> Clustering(std::vector<int> &indices, double threshold);
private:
	std::vector<double> MeanShiftProcedure(std::vector<double> initX,
		double threshold = 3e-5);
	double EuclideanDistance(std::vector<double> p1,
		std::vector<double> p2);
	std::vector< std::vector<double> > points_;
	int dim_;
	double sigma_;
	double num_points_;
};


#endif // MEAN_SHIFT_CLUSTERING
