#pragma once
#include "Common.h"
#include "Octree.h"
#include <numeric>

struct Object;

class Scene
{

public:

	// Class Variable Declarations
	double partInterval, envelopeInterval; // Target part & envelope spacings
	int nParts = 0, nPairs = 0, idCount = 0; // Counters
	std::vector<std::shared_ptr<Object>> objects, previousObjects; // Vector of pointers to objects in the scene
	Eigen::MatrixXi partCollisions = Eigen::MatrixXi(5000, 5000), previousPartCollisions = Eigen::MatrixXi(5000, 5000); // Array holding part to part collision data
	std::vector<int> envelopeCollisions, previousEnvelopeCollisions; // Vector holding part to envelope collision data
	Octree referencePart; // Octree reference part
	Eigen::Vector3d envelope; // Vector containing the max coordinate of the build envelope (x, y, z dimensions)
	double sceneVolume;
	double density; // Scene packing density
	int collisions; // Total collisions in the scene

	// Optimization Declarations
	double minTrans = 1.0, maxTrans = 50.0;
	int transMoves = 5, totalMoves = transMoves + 4, optPreSteps = 1000;
	Eigen::VectorXd optPreObjectives = Eigen::VectorXd(1000);
	double step = (maxTrans - minTrans) / (transMoves - 1);
	double minProb = 0.02, remProb = 1 - minProb * (transMoves + 2);
	std::vector<double> quality, deltaObj, probability;
	std::vector<int> tempCount, prevTempCount;
	std::vector<double> objectiveChangePerMove; // Tracks the change in objective function over accepted steps per move in move set
	std::vector<double> objectiveAtTemp; // Tracks the change in objective function at a given temperature
	double cD = 0; // Objective function coefficients (Collisions, Density)
	int cC = 0;

	// Public Class Methods
	Scene(Octree referencePart, Eigen::Vector3d envelope, double partInterval, double envelopeInterval);

	int add_part(Eigen::Vector3d location, bool random);

	void remove_part(int index, bool delay);

	void translate_part(int index, Eigen::Vector4d vector);

	void rotate_part(int index, int axis, double angle);

	void swap_parts(int index1, int index2);

	void part_collisions(int index);

	void envelope_collisions(int index);

	int total_collisions(std::vector<int> indices);

	int sum_collisions();

	double packing_density(bool complete);

	std::vector<int> update_state();

	double objective_function(std::vector<int> moves);

	void recover_state(std::vector<int> moves);

	void optimize();

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

	void backup_state()
	{
		// Backup the current object state
		this->previousEnvelopeCollisions = this->envelopeCollisions;
		this->previousPartCollisions = this->partCollisions;
		this->previousObjects = this->objects;
	}

	void reset_state()
	{
		// Resets the state to default (Zero objects, no collision data)

		// Collision pairing matrices
		this->partCollisions.fill({});
		this->previousPartCollisions.fill({});

		// Envelope collision vectors
		std::fill(this->envelopeCollisions.begin(), this->envelopeCollisions.end(), 0);
		std::fill(this->previousEnvelopeCollisions.begin(), this->previousEnvelopeCollisions.end(), 0);

		// Object structs in the scene
		this->objects.clear();
		this->previousObjects.clear();

		// Reset part counter
		this->nParts = 0;
	}

	void fill_random(int n = 0)
	{
		// Fills the scene with random parts based on a naive packing method
		if (n == 0)
		{
			// Calculate a naive packing estimate
			double pack = round(this->sceneVolume / this->referencePart.bboxVol);
			n = (int)round(pack);
		}
		Eigen::Vector3d dummy;
		for (int i = 0; i < n; i++)
		{
			LOG(i);
			int ind = this->add_part(dummy, 1);
			// Calculate collisions in the scene
			this->part_collisions(ind);
			this->envelope_collisions(ind);
		}
	}

	void accept_state(int moveIndex, double fNew, double fOld)
	{
		// Updates the optimization statistics for the scene at a new, accepted scene state

		// Update delta of the objective function
		this->objectiveChangePerMove[moveIndex] += abs(fNew - fOld);

		// Update move quality
		this->quality[moveIndex] = (1.0 / this->prevTempCount[moveIndex]) * this->objectiveChangePerMove[moveIndex];

		// Update move selection probabilities
		double qualitySum = 0;
		for (auto& n : this->quality)
		{
			qualitySum += n;
		}

		for (int i = 0; i < this->quality.size(); i++)
		{
			this->probability[i] = this->minProb + this->remProb * (this->quality[i] / qualitySum);
		}
		
	}

	void delete_object(int index)
	{
		// Delete an Object struct given an index in the objects vector
		this->objects.erase(this->objects.begin() + index);
	}

	double cooling(double T, std::vector<double> v)
	{
		// Returns a new temperature for the next simulated annealing iteration

		// Get the standard deviation of the objective function at this temperature
		double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
		double m = sum / v.size();

		double accum = 0.0;
		std::for_each(std::begin(v), std::end(v), [&](const double d) {
			accum += (d - m) * (d - m);
			});

		double stdev = sqrt(accum / (v.size() - 1));

		// Calculate the new temperature
		return T * exp(-(0.7 * T / stdev));
	}
};

struct Object
{
	Eigen::Matrix4d affine;
	Eigen::Matrix4d previousState;
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
		this->backup();
		
		// Update (transform) the state
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
		this->transform(util::create_rotation_matrix(angle, axis, degrees));
	}

	void backup()
	{
		// Backup the current object state
		this->previousState = this->affine;
	}

	void undo()
	{
		// Recover the previous state of this object
		this->affine = this->previousState;
	}


	
};

