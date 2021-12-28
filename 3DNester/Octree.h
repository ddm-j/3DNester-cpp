#pragma once
#include <iostream>
#include <climits>
#include <bitset>
#include "util.h"
#include "Common.h"

#define LOG(x) std::cout << x << std::endl

struct Node;

class Octree
{
public:

	// Class Variable Declarations
	double mesh_sa, mesh_v; // Surface area & volume of mesh
	Eigen::Vector4d bboxCenter; // Center point of the down-sampled (poisson) pointcloud bounding box
	Eigen::Matrix<double, 4, 2> bboxMinMax; // Min & Max Vectors of the bounding box
	Eigen::MatrixXd samplePoints; // Matrix 3xN_samples (unknown size) that will store the poisson sampled mesh points
	Eigen::MatrixXd octreeCenters; // Matrix 3xN_nodes (unknown size) that will store the octree node center points
	double rootSize; // Scalar maximum dimension of the bounding box (size of the root node)
	double rootRadius; // Scalar radius of the root node. Used in collision detection
	double leafRadius; // Scalar radius of the leaf nodes. Used in collision detection
	int maxDepth; // Maximum tree depth, calculated during construction
	Node* rootNode; // Pointer to the root node of the octree
	std::unordered_map<int, Node *> treeMap; // Hash map containing the nodes in the tree
	int leafNodeCount = 0;

	// Public Methods
	Octree();

	Octree(const char* file_path, double min_voxel);

	void update_bbox(Eigen::MatrixXd& pcd, double voxSize=0);

	void build_tree(Node* parent, Eigen::MatrixXd points, Eigen::Matrix<double, 3, 16> nodes, int depth);

	void traverse_tree(Node * pNode);

	void part_collision_test(Eigen::Matrix4d u, Eigen::Matrix4d v, int pU, int pV, double interval, int& cnt);

	void envelope_collision_test(Eigen::Matrix4d u, int pU, Eigen::Vector3d envelope, double interval, int& cnt);

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

	void calc_volume(MyMesh& m)
	{
		// Calculates volume of the mesh
		vcg::tri::Inertia<MyMesh> Ib(m);

		this->mesh_v = Ib.Mass();
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

	std::vector<int> check_points(Eigen::MatrixXd points, Eigen::Matrix<double, 3, 2> node)
	{
		// Which Points are inside node
		bool c1, c2, c3;
		std::vector<int> indices;
		Eigen::MatrixXd p;

		// Loop through points
		for (int i = 0; i < points.cols(); i++)
		{
			p = points.block(0, i, 3, 1);
			c1 = (node(0, 0) <= p(0, 0)) && (p(0, 0) <= node(0, 1));
			c2 = (node(1, 0) <= p(1, 0)) && (p(1, 0) <= node(1, 1));
			c3 = (node(2, 0) <= p(2, 0)) && (p(2, 0) <= node(2, 1));

			if (c1 && c2 && c3)
			{
				indices.push_back(i);
			}
		}

		return indices;
	}

};

struct Node {

	Eigen::Vector4d center;
	double radius;
	int key;
	int8_t childMask;
	bool isLeaf;

	Node(Octree* tree, Eigen::VectorXd center, int parentKey, int childNum, bool aisLeaf)
	{
		// Node constructor

		if (center.rows() == 3)
		{
			// We've been passed a 3D vector. Convert to a 4D for affine readiness
			this->center(Eigen::seq(0, 2)) = center;
			this->center(3) = 1.0;
		}
		else if (center.rows() == 4)
		{
			// We've been passed an affine ready vector
			this->center = center;
		}
		else {
			// We've been passed a bad vector (not the right size)
			assert(0);
		}

		// Calculate the location code (key) for this node
		this->key = (parentKey << 3) + childNum;

		// Calculate the radius of this node
		int depth = util::get_depth(this->key);
		int d = tree->rootSize / (pow(2, depth));
		this->radius = 3 * sqrt(d) / 2;

		// Add this node to the tree hashmap
		tree->treeMap[this->key] = this;
		//bitset<sizeof(this->key)* CHAR_BIT> x(this->key);

		// Initialize childMask
		this->childMask = 0;

		// Set the leaf status
		this->isLeaf = aisLeaf;
	}

	void advance_child_mask(int i)
	{
		this->childMask |= 1ull << i;
	}
};

