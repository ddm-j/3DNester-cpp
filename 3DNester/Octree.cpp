#include <iostream>
#include <math.h>
#include "Octree.h"
#include "Common.h"
#include "util.h"
//#include <open3d/Open3D.h>

#define LOG(x) std::cout << x << std::endl

Octree::Octree(const char* file_path, double vox_size)
{
	// Octree Constructor Function

	// Variable Declarations
	double max_voxel_sa, point_density;
	int min_points;
	Eigen::Matrix4d bbox_trans;
	bbox_trans << 
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;


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
	this->update_bbox(this->samplePoints);
	bbox_trans.block(0, 3, 3, 1) = -this->bboxCenter.head(3);
	this->samplePoints = bbox_trans * this->samplePoints;

	// Build the tree
	Eigen::Matrix<double, 3, 16> nodes = util::split_node(bboxMinMax.block(0, 0, 3, 2));
	int depth = 0;
	int max_depth = round(log(this->rootSize / vox_size) / log(2.0));
	this->build_tree(this->samplePoints, nodes, max_depth, depth);

}

void Octree::update_bbox(Eigen::MatrixXd& pcd)
{
	// This function updates the class bounding box attributes using a supplied pointcloud

	Eigen::Vector4d min = pcd.rowwise().minCoeff();
	Eigen::Vector4d max = pcd.rowwise().maxCoeff();

	// Set the Centerpoint
	this->bboxCenter = min + 0.5 * (max - min);

	// Set outMinMax
	this->bboxMinMax.block(0, 0, 4, 1) = min;
	this->bboxMinMax.block(0, 1, 4, 1) = max;

	// Set the bounding cube size (root node)
	this->rootSize = (max - min).maxCoeff();
}

void Octree::build_tree(Eigen::MatrixXd& points, Eigen::Matrix<double, 3, 16>& nodes, int maxDepth, int &depth)
{
	// Recursively build the octree node structure

	// If we are calling this function, we are advancing to the next depth level
	depth++;

	// Loop into the nodes
	for (int i = 0; i < 16; i += 2)
	{
		// Checkpoints
		this->check_points(points, nodes.block(0, i, 3, 2));
	}


}
