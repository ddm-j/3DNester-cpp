#pragma once
#include <Eigen/Dense>
#include <vector>

namespace util 
{
	constexpr double pi = 3.14159265358979323846;

	Eigen::Matrix<double, 3, 16> split_node(Eigen::Matrix<double, 3, 2> node);

	int msb(int key);

	void log_bitset(int i, int size);

	int get_depth(int key);

	double rand_double(double min, double max);

	int rand_select(std::vector<double> weights);

	Eigen::Vector3d rand_unit_vector();

	int rand_int(int n);

	Eigen::Matrix4d create_rotation_matrix(double angle, int axis, bool degrees);

	bool sphere_collision(Eigen::Vector4d u, Eigen::Vector4d v, double distance);

	bool outside_envelope(Eigen::Vector4d point, Eigen::Vector3d envelope, double distance, bool partial = 1);
}