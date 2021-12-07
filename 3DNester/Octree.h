#pragma once
#include <iostream>

class Octree
{
public:

	double max_voxel_sa; // Maximum surface area estimate of a minimum voxel cross section
	double point_density; // Density of points for random point sampling on mesh
	double mesh_sa; // Surface area of mesh
	int min_points; // Number of points for random sampling

	Octree(std::string file_path, double min_voxel);

};

