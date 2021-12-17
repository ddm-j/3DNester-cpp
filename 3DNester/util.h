#pragma once
#include <Eigen/Dense>

namespace util 
{
	Eigen::Matrix<double, 3, 16> split_node(Eigen::Matrix<double, 3, 2> node);

	int msb(int key);

	void log_bitset(int i, int size);
}