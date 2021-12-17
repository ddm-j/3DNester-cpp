#include "util.h"
#include <bitset>
#include <iostream>

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

}