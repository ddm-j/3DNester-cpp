#pragma once
#include "Common.h"
#include "Octree.h"

struct Object;

class Scene
{

public:

	// Class Variable Declarations
	double partInterval, envelopeInterval; // Target part & envelope spacings
	int nParts = 0, nPairs = 0, idCount = 0; // Counters
	std::vector<Object*> objects; // Vector of pointers to objects in the scene
	Eigen::MatrixXi partCollisions = Eigen::MatrixXi(5000, 5000); // Array holding part to part collision data
	std::vector<int> envelopeCollisions; // Vector holding part to envelope collision data
	Octree referencePart; // Octree reference part
	Eigen::Vector3d envelope;

	// Public Class Methods
	Scene(Octree referencePart, Eigen::Vector3d envelope, double partInterval, double envelopeInterval);

	void add_part(Eigen::Vector3d location, bool random);

	void remove_part(int index);

	void part_collisions(int index);

	void envelope_collisions(int index);

	int total_collisions(std::vector<int> indices);

	int sum_collisions();


private:

	void remove_collision_pairs(int index)
	{
		// Removes the collision history data for a part

		// Shift the diagonal matrix
		for (int i = index; i < this->nParts - 1; i++)
		{
			for (int j = i; j < this->nParts - 1; j++)
			{
				this->partCollisions(i, j) = this->partCollisions(i + 1, j + 1);
			}
		}

		// Shift the top row
		for (int j = index; j < this->nParts - 1; j++)
		{
			this->partCollisions(0, j) = this->partCollisions(0, j + 1);
		}

		// Set column n-1 to zero
		for (int i = 0; i < this->nParts; i++)
		{
			this->partCollisions(i, this->nParts - 1) = 0;
		}
	}
};

struct Object
{
	Eigen::Matrix4d affine;
	Eigen::Matrix4d eye;

	Object()
	{
		affine.setIdentity();
	}

	Eigen::Vector4d get_center()
	{
		return affine.col(3);
	}
	
	void transform(Eigen::Matrix4d matrix)
	{
		// General object transform (translation, rotation, scale, shear, etc)
		this->affine = this->affine*matrix;
	}
	
	void translate(Eigen::Vector4d vector)
	{
		// Object translation method (affine w/ 4d vector)
		this->eye.setIdentity();
		eye.col(3) = vector;

		this->transform(eye);
	}

	void rotate(double angle, int axis, bool degrees = 1.0)
	{
		// Object rotation method (affine w/ rotation matrix)
		this->transform(util::rotation_matrix(angle, axis, degrees));
	}
	
};

