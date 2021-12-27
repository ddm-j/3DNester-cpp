#include "util.h"
#include <bitset>
#include <math.h>
#include <iostream>
#include <random>

namespace util {

	Eigen::Matrix<double, 3, 16> split_node(Eigen::Matrix<double, 3, 2> node)
	{
		// Takes a node definition (minimum vector, maximum vector) and splits it into 8 min/max pairs for child nodes

		//Declarations
		Eigen::Matrix<double, 3, 16> nodes;
		Eigen::Matrix3d minmedmax;
		minmedmax.col(0) = node.col(0);
		minmedmax.col(2) = node.col(1);
		minmedmax.col(1) = (node.col(0) + node.col(1)) / 2;
		int ids[3][16] = {
			{ 0, 1, 0, 1, 1, 2, 1, 2, 0, 1, 0, 1, 1, 2, 1, 2 },
			{ 0, 1, 1, 2, 1, 2, 0, 1, 0, 1, 1, 2, 1, 2, 0, 1 },
			{ 0, 1, 0, 1, 0, 1, 0, 1, 1, 2, 1, 2, 1, 2, 1, 2 }
		};

		// Work
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 16; j++)
			{
				nodes(i, j) = minmedmax(i, ids[i][j]);
			}
		}

		return nodes;

	}

	int msb(int key)
	{
		// Get the most significant bit of our integer
		int cnt = 0;
		while (key > 1) { cnt++; key >>= 1; }
		return cnt;
	}

	void log_bitset(int i, int size)
	{
		if (size == 8)
		{
			std::bitset<8> x(i);
			std::cout << x << std::endl;
		}
		else if (size == sizeof(i)*CHAR_BIT)
		{
			std::bitset<sizeof(i)* CHAR_BIT> x(i);
			std::cout << x << std::endl;
		}
		else {
			assert("Option not defined.");
		}
	}

	int get_depth(int key)
	{
		// Calculate the octree node depth given the location code (key)
		// Unsign the key

		// Add a sentinel bit to the number
		//key |= 1ull << util::msb(key);

		// Begin depth loop
		for (int d = 0; key; d++)
		{
			if (key == 1) return d;
			key >>= 3;


			if (d > 21) assert(0); // bad key
		}
		assert(0); // bad key
	}

	double rand_double(double min, double max)
	{
		std::random_device rd;
		std::default_random_engine eng(rd());
		std::uniform_real_distribution<double> distr(min, max);

		return distr(eng);
	}

	Eigen::Matrix4d create_rotation_matrix(double angle, int axis, bool degrees)
	{
		// Takes an angle, axis and outputs an affine rotation matrix. 
		// Bettern than a general rotation matrix (x, y, z) rotations. Avoids gimbal lock
		Eigen::Matrix4d rotationMatrix;

		// Convert degrees to radians
		angle = degrees ? angle * pi / 180 : angle;

		// Get the Affine Transformation matrices
		switch (axis)
		{
		case 0:
			// Rotation about the x-axis
			rotationMatrix << 1, 0,		      0,		  0,
							  0, cos(angle), -sin(angle), 0,
							  0, sin(angle),  cos(angle), 0,
							  0, 0,			  0,		  1;
			break;

		case 1:
			// Rotation about the y-axis
			rotationMatrix << cos(angle), 0, -sin(angle), 0,
							  0,	      1,  0,		  0,
							  sin(angle), 0,  cos(angle), 0,
							  0,		  0,  0,		  1;
			break;

		case 2:
			// Rotation about the z-axis
			rotationMatrix << cos(angle), -sin(angle), 0, 0,
							  sin(angle),  cos(angle), 0, 0,
							  0,		   0,		   1, 0,
							  0,		   0,		   0, 1;
			break;

		defualt:
			// Incorrect axis specified.
			printf("Axis options 0, 1, 2 (x, y, z) are acceptable. Incorrect axis specified.\n");
			assert(0);
		}

		return rotationMatrix;
	}

	bool sphere_collision(Eigen::Vector4d u, Eigen::Vector4d v, double distance)
	{
		// Test the collision between two spheres

		return (u - v).norm() <= distance;
	}

	bool outside_envelope(Eigen::Vector4d point, Eigen::Vector3d envelope, double distance, bool partial)
	{
		// Calculate if point is outside of build envelope
		bool outX, outY, outZ;
		int mod = partial ? 1 : -1;

		// Conditions for partial/complete outlier
		outX = (point(0) + mod * -1 * distance < 0) || (point(0) + mod * distance > envelope(0));
		outY = (point(1) + mod * -1 * distance < 0) || (point(1) + mod * distance > envelope(1));
		outZ = (point(2) + mod * -1 * distance < 0) || (point(2) + mod * distance > envelope(2));

		if (outX)
		{
			return true;
		}
		else if (outY)
		{
			return true;
		}
		else if (outZ)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}