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


	Octree(const char* file_path, double min_voxel);

	void update_bbox(Eigen::MatrixXd& pcd);

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

};

