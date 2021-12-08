#include <iostream>
#include <math.h>
#include "Octree.h"
#include "Common.h"
//#include <open3d/Open3D.h>


Octree::Octree(const char* file_path, double vox_size)
{
/*

	using namespace open3d;

	// Define PCD Parameters
	this->max_voxel_sa = pow(vox_size, 2) / sqrt(2.0);
	this->point_density = 15.0 / this->max_voxel_sa;

	// Mesh
	//auto mesh = geometry::TriangleMesh::CreateSphere(10);
	auto mesh = io::CreateMeshFromFile(file_path);
	this->mesh_sa = mesh->GetSurfaceArea();
	this->min_points = round(this->mesh_sa * this->point_density);

	// Sample mesh with point-cloud
	auto pcd = mesh->SamplePointsPoissonDisk(this->min_points);

	//visualization::DrawGeometries({ pcd });
*/
	MyMesh m;
	vcg::tri::io::Importer<MyMesh>::Open(m, file_path);



}