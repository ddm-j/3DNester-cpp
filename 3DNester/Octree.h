#pragma once
#include <iostream>
#include "Common.h"

class Octree
{
public:

	double mesh_sa; // Surface area of mesh

	Octree(const char* file_path, double min_voxel);

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

};

