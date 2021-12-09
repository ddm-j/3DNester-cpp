#include <iostream>
#include <math.h>
#include "Octree.h"
#include "Common.h"
//#include <open3d/Open3D.h>

#define LOG(x) std::cout << x << std::endl

Octree::Octree(const char* file_path, double vox_size)
{
	// Octree Constructor Function
	double max_voxel_sa, point_density;
	int min_points;

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

	printf("Computed a feature aware poisson disk distribution of %i vertices radius is %6.3f\n", PoissonSurfaceMesh.VN(), radius);

	//vcg::tri::io::ExporterOFF<MyMesh>::Save(PoissonSurfaceMesh, "test.off");
	
	
	

	
}
