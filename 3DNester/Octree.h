#pragma once
#include <iostream>
#include "Common.h"

#define LOG(x) std::cout << x << std::endl

class Octree
{
public:

	// Class Variable Declarations
	double mesh_sa; // Surface area of mesh
	Eigen::Vector4d bboxCenter; // Center point of the down-sampled (poisson) pointcloud bounding box
	Eigen::Matrix<double, 4, 2> bboxMinMax; // Min & Max Vectors of the bounding box
	Eigen::MatrixXd samplePoints; // Matrix 3xN_samples (unknown size) that will store the poisson sampled mesh points
	Eigen::MatrixXd octreeCenters; // Matrix 3xN_nodes (unknown size) that will store the octree node center points
	double rootSize; // Scalar maximum dimension of the bounding box (size of the root node)


	Octree(const char* file_path, double min_voxel);

	void update_bbox(Eigen::MatrixXd& pcd);

	void build_tree(Eigen::MatrixXd& points, Eigen::Matrix<double, 3, 16>& nodes, int maxDepth, int& depth);

private:

	void calc_surface_area(MyMesh& m)
	{
		// Calculate mesh surface area: WHERE THE FUCK IS VCG DOCS?
		double area = 0;
		for (int i = 0; i < m.FN(); i++) {
			area += m.face[i].Q();
		}
		this->mesh_sa = area;
	}

	void get_mesh_points(MyMesh& m, Eigen::MatrixXd& matrix)
	{
		// Pull the mesh points out of their vector container and pack them into an Eigen Matrix

		// Allocate our matrix
		matrix.resize(4, m.VN());

		for (int i = 0; i < m.vert.size(); i++)
		{
			/* LOG the vertex coords
			LOG(m.vert[i].P()[0]);
			LOG(m.vert[i].P()[1]);
			LOG(m.vert[i].P()[2]);
			*/

			matrix(0, i) = m.vert[i].P()[0];
			matrix(1, i) = m.vert[i].P()[1];
			matrix(2, i) = m.vert[i].P()[2];
			matrix(3, i) = 1.0;

		}

		LOG("Matrix packing complete.");
	}

	Eigen::MatrixXd check_points(Eigen::MatrixXd& points, Eigen::Matrix<double, 3, 2> node)
	{
		// Function to check which points are inside our node

		Eigen::Vector3d min = node.col(0);
		Eigen::Vector3d max = node.col(1);

		// Preprocess
		Eigen::Vector3d b1 = min;
		Eigen::Vector3d b2{ max(0), min(1), min(2) };
		Eigen::Vector3d b3{ max(0), max(1), min(2) };
		Eigen::Vector3d b4{ min(0), max(1), min(2) };
		Eigen::Vector3d t1{ min(0), min(1), max(2) };
		Eigen::Vector3d t2{ max(0), min(1), max(2) };
		Eigen::Vector3d t3{ max(0), max(1), max(2) };
		Eigen::Vector3d t4{ min(0), max(1), max(2) };
		Eigen::Vector3d center = (b1 + t3) / 2.0;

		// First Direction
		Eigen::Vector3d d1 = t1 - b1;
		double s1 = d1.norm();
		d1 /= s1;

		// Second Direction
		Eigen::Vector3d d2 = b2 - b1;
		double s2 = d2.norm();
		d2 /= s2;

		// Third Directionn
		Eigen::Vector3d d3 = b4 - b1;
		double s3 = d3.norm();
		d3 /= s3;

		// Work Some Magic
		Eigen::MatrixXd dir_vec = points.block(0, 0, 3, points.cols()).colwise() - center;

		Eigen::Matrix<bool, 1, Eigen::Dynamic> res1 = ((2 * (d1.asDiagonal() * dir_vec).colwise().sum().cwiseAbs()).array() > s1);
		Eigen::Matrix<bool, 1, Eigen::Dynamic> res2 = ((2 * (d2.asDiagonal() * dir_vec).colwise().sum().cwiseAbs()).array() > s2);
		Eigen::Matrix<bool, 1, Eigen::Dynamic> res3 = ((2 * (d3.asDiagonal() * dir_vec).colwise().sum().cwiseAbs()).array() > s3);

		Eigen::Matrix<bool, 1, Eigen::Dynamic> mask = res1.array() * res2.array() * res3.array();


		LOG(points(Eigen::all,mask));

		std::cin.get();


		return dir_vec;
	}

};

