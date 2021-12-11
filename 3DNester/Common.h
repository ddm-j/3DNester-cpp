#pragma once
#include <opencv2/opencv.hpp>
#include "vcg/complex/complex.h"
#include "wrap/io_trimesh/import.h"
#include<wrap/io_trimesh/export_off.h>
#include "vcg/simplex/vertex/base.h"
#include "vcg/simplex/vertex/component.h"
#include "vcg/simplex/face/base.h"
#include "vcg/simplex/face/component.h"
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <Eigen/Dense>


class MyVertex; class MyEdge; class MyFace;
struct MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex>   ::AsVertexType,
	vcg::Use<MyEdge>     ::AsEdgeType,
	vcg::Use<MyFace>     ::AsFaceType> {};
class MyVertex : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  > {};
class MyFace : public vcg::Face<   MyUsedTypes, vcg::face::FFAdj, vcg::face::Normal3f, vcg::face::Qualityd, vcg::face::VertexRef, vcg::face::BitFlags > {};
class MyEdge : public vcg::Edge<   MyUsedTypes> {};
class MyMesh : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace>, std::vector<MyEdge>  > {};