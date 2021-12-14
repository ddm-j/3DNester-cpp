#pragma once
#include <iostream>
#include "Common.h"

#define LOG(x) std::cout << x << std::endl

struct Node {

	Eigen::Vector4d center;
	int key;
	int8_t childMask;
	bool isLeaf;

	Node(Eigen::Vector3d center, int parentKey, int childNum, bool aisLeaf)
	{
		// Set the Node Centerpoint (Affine ready format 4x1)
		this->center(Eigen::seq(0, 2)) = center;
		this->center(3) = 1.0;
		LOG(this->center);

		// Calculate the location code (key) for this node
		this->key = (parentKey << 3) + childNum;

		// Initialize childMask
		this->childMask = 0;

		// Set the leaf status
		this->isLeaf = aisLeaf;
	}

	void add_child()
	{
		
		(this->childMask << 1) + 1;

	}
};

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

	// Public Methods
	Octree(const char* file_path, double min_voxel);

	void update_bbox(Eigen::MatrixXd& pcd);

	void build_tree(Node *parent, Eigen::MatrixXd points, Eigen::Matrix<double, 3, 16>& nodes, int maxDepth, int& depth);

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

	Eigen::VectorXi check_points(Eigen::MatrixXd points, Eigen::Matrix<double, 3, 2> node)
	{
		// Which Points are inside node
		bool c1, c2, c3;
		Eigen::VectorXi indices(points.cols());
		Eigen::MatrixXd p;
		int cnt = 0;

		// Loop through points
		for (int i = 0; i < points.cols(); i++)
		{
			p = points.block(0, i, 3, 1);
			c1 = (node(0, 0) <= p(0, 0)) && (p(0, 0) <= node(0, 1));
			c2 = (node(1, 0) <= p(1, 0)) && (p(1, 0) <= node(1, 1));
			c3 = (node(2, 0) <= p(2, 0)) && (p(2, 0) <= node(2, 1));

			if (c1 && c2 && c3)
			{
				indices(cnt) = i;
				cnt++;
			}
		}

		
		if (cnt == 0)
		{
			// Return unintialized vector (no assignments made) for checking outside of the function
			return indices;
		}

		return indices(Eigen::seq(0,cnt));
	}

};

