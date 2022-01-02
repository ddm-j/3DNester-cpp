#include <iostream>
#include <bitset>
#include <math.h>
#include "Octree.h"
#include "Common.h"
#include "util.h"

#define LOG(x) std::cout << x << std::endl

Octree::Octree()
{
	// Empty default constructor
}

Octree::Octree(const char* file_path, double vox_size)
{
	// Octree Constructor Function
	LOG("Constructing Octree");

	// Variable Declarations
	double max_voxel_sa, point_density;
	int min_points;
	Eigen::Matrix4d bbox_trans;
	bbox_trans.setIdentity();


	// Load the mesh with VCG
	MyMesh m;
	if (vcg::tri::io::Importer<MyMesh>::Open(m, file_path) != 0)
	{
		printf("Error reading file  %s\n", file_path);
		exit(0);
	}
	
	vcg::tri::RequirePerFaceNormal(m);
	vcg::tri::RequirePerFaceQuality(m);
	vcg::tri::UpdateNormal<MyMesh>::NormalizePerFaceByArea(m);
	vcg::tri::UpdateQuality<MyMesh>::FaceArea(m);

	printf("Input mesh  vn:%i fn:%i\n", m.VN(), m.FN());
	printf("Mesh has %i vert and %i faces\n", m.VN(), m.FN());

	// Define Pointcloud Parameters
	max_voxel_sa = pow(vox_size, 2) / sqrt(2.0);
	point_density = 15.0 / max_voxel_sa;

	this->calc_surface_area(m);
	this->calc_volume(m);
	min_points = round(this->mesh_sa * point_density);

	// Perform Poisson Sampling over the Mesh
	MyMesh MontecarloSurfaceMesh;
	MyMesh PoissonSurfaceMesh;

	vector<vcg::Point3f> sampleVec;
	vcg::tri::TrivialSampler<MyMesh> mps(sampleVec);
	vcg::tri::SurfaceSampling<MyMesh, vcg::tri::TrivialSampler<MyMesh> >::PoissonDiskParam pp;

	vcg::tri::SurfaceSampling<MyMesh, vcg::tri::TrivialSampler<MyMesh> >::Montecarlo(m, mps, 2*min_points);
	vcg::tri::BuildMeshFromCoordVector(MontecarloSurfaceMesh, sampleVec);

	sampleVec.clear();
	auto radius = vcg::tri::ComputePoissonDiskRadius<MyMesh>(m, min_points);
	vcg::tri::SurfaceSampling<MyMesh, vcg::tri::TrivialSampler<MyMesh> >::PoissonDiskPruning(mps, MontecarloSurfaceMesh, radius, pp);
	vcg::tri::BuildMeshFromCoordVector(PoissonSurfaceMesh, sampleVec);

	printf("Computed a poisson disk distribution of %i vertices radius is %6.3f\n", PoissonSurfaceMesh.VN(), radius);

	//vcg::tri::io::ExporterOFF<MyMesh>::Save(PoissonSurfaceMesh, "test.off");

	// Get "affine ready" column vector matrix of points
	this->get_mesh_points(PoissonSurfaceMesh, this->samplePoints);

	// Translate the center of our bounding box to the origin of the CS
	this->update_bbox(this->samplePoints, vox_size);
	bbox_trans.block(0, 3, 3, 1) = -this->bboxCenter.head(3);
	this->samplePoints = bbox_trans * this->samplePoints;
	this->update_bbox(this->samplePoints, vox_size);

	// Build the tree
	Eigen::Matrix<double, 3, 16> nodes = util::split_node(bboxMinMax.block(0, 0, 3, 2));
	int depth = 0;
	this->rootNode = new Node(this, (bboxMinMax.col(0)+bboxMinMax.col(1))/2, 0, 1, false);
	this->build_tree(this->rootNode, this->samplePoints, nodes, depth);

}

void Octree::update_bbox(Eigen::MatrixXd& pcd, double voxSize)
{
	// This function updates the class bounding box attributes using a supplied pointcloud

	Eigen::Vector4d min = pcd.rowwise().minCoeff();
	Eigen::Vector4d max = pcd.rowwise().maxCoeff();

	// Set the Centerpoint
	this->bboxCenter = (max + min)/2.0;

	// Set outMinMax
	this->bboxMinMax.block(0, 0, 4, 1) = min;
	this->bboxMinMax.block(0, 1, 4, 1) = max;

	// Set the bounding cube size (root node)
	this->rootSize = (max - min).maxCoeff();

	if (voxSize != 0)
	{
		this->maxDepth = round(log(this->rootSize / voxSize) / log(2.0));
	}

	// Calculate root/lead radius'
	this->rootRadius = 3 * sqrt(this->rootSize) / 2;
	this->leafRadius = 3 * sqrt(this->rootSize / pow(2, this->maxDepth+1)) / 2;

	// Update bounding box volume
	this->bboxVol = pow(this->rootSize, 3);

	printf("Updating root radius to %f\n", this->rootRadius);
	printf("Updating leaf radius to %f\n", this->leafRadius);
}

void Octree::build_tree(Node *parent, Eigen::MatrixXd points, Eigen::Matrix<double, 3, 16> nodes, int depth)
{
	// Recursively build the octree node structure

	std::vector<int> indices;

	// If we are calling this function, we are advancing to the next depth level in the octree
	depth++;

	// Loop into the nodes
	for (int i = 0; i < 8; i ++)
	{
		// Get the indices of points that are inside the node
		indices = this->check_points(points, nodes.block(0, i*2, 3, 2));

		if ((indices.size() != 0) && (depth <= this->maxDepth))
		{
			// In this case, we need to split this node and advance the octree depth down this branch
			Node* pNode = new Node(this, (nodes.col(i*2) + nodes.col(i*2 + 1)) / 2, parent->key, i, false);

			// Split the nodes and start recursive tree build
			this->build_tree(pNode, points(Eigen::all, indices), util::split_node(nodes.block(0, i*2, 3, 2)), depth);

			// Advance the childMask of the parent node
			parent->advance_child_mask(i);

		}
		else if (indices.size() != 0)
		{
			// In this case, we are at maximum depth and this is a leaf node
			Node* pNode = new Node(this, (nodes.col(i*2) + nodes.col(i*2 + 1)) / 2, parent->key, i, true);

			// Increment the leaf node counter
			this->leafNodeCount++;

			// Advance the childmask
			parent->advance_child_mask(i);
		}
	}
}

void Octree::traverse_tree(Node * pNode)
{
	// Traverses the linear hashed octree
	for (int i = 0; i < 8; i++)
	{
		// See if ith child exists (use childMask)
		if (pNode->childMask & (1 << i))
		{
			// Calculate the "key" of this child
			int key = (pNode->key << 3) + i;

			// Retrieve the node pointer of this child
			Node* childNode = this->treeMap.at(key);

			// Get the depth of this node
			int depth = util::get_depth(childNode->key);
			
			if (childNode->isLeaf)
			{
				printf("Visiting leaf node at depth %i with center point:\n", depth);
				LOG(childNode->center);
			}
			else
			{
				printf("Visiting node at depth %i with center point:\n", depth);
				LOG(childNode->center);
			}

			// Recursively visit nodes
			this->traverse_tree(childNode);
		}
	}
}

void Octree::part_collision_test(Eigen::Matrix4d u, Eigen::Matrix4d v, int pU, int pV, double interval, int& cnt)
{
	// Test the number of leaf collisions between two instances of octree (defined by different affine positions U & V)

	// Loop through the children of U & V, testing collisions between them
	int8_t maskU = this->treeMap.at(pU)->childMask;
	int8_t maskV = this->treeMap.at(pV)->childMask;
	int childKeyU, childKeyV;
	double dist;

	for (int i = 0; i < 8; i++)
	{
		for (int j = i; j < 8; j++)
		{
			if (maskU & (1 << i) && (maskV & (1 << j)))
			{
				// Test for a collision at this level
				childKeyU = (pU << 3) + i;
				childKeyV = (pV << 3) + j;

				dist = 2 * this->treeMap.at(childKeyU)->radius + interval;
				if (util::sphere_collision(u*this->treeMap.at(childKeyU)->center, v*this->treeMap.at(childKeyV)->center, dist))
				{
					// Potential collision detected
					if (this->treeMap.at(childKeyU)->isLeaf)
					{
						// We are at max depth. Increment the counter
						cnt++;
					}
					else
					{
						// We are not at max depth of the tree. Recursively go deeper
						Octree::part_collision_test(u, v, childKeyU, childKeyV, interval, cnt);
					}
				}

			}
		}
	}
}

void Octree::envelope_collision_test(Eigen::Matrix4d u, int pU, Eigen::Vector3d envelope, double interval, int& cnt)
{
	// Calculate the number of leafs that are not within the envelope bounds
	// Similar to Octree::part_collision_test() but with no nested loops & and a different collision test.

	// Loop through the children of U, testing envelope collisions
	int8_t maskU = this->treeMap.at(pU)->childMask;
	int childKeyU;
	double dist;

	for (int j = 0; j < 8; j++)
	{
		if (maskU & (1 << j))
		{
			// Test for a collision at this level
			childKeyU = (pU << 3) + j;

			dist = this->treeMap.at(childKeyU)->radius + interval;
		
			if (util::outside_envelope(u * this->treeMap.at(childKeyU)->center, envelope, dist))
			{
				// Potential collision detected
				if (this->treeMap.at(childKeyU)->isLeaf)
				{
					// We are at max depth. Increment the counter
					cnt++;
				}
				else
				{
					// We are not at max depth of the tree. Recursively go deeper
					Octree::envelope_collision_test(u, childKeyU, envelope, interval, cnt);
				}
			}

		}
	}
}
