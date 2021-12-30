#include "Scene.h"
#include "util.h"
#include <iostream>

#define LOG(x) std::cout << x << std::endl

Scene::Scene(Octree referencePart, Eigen::Vector3d envelope, double partInterval, double envelopeInterval)
{
	// Parameterized constructor for the scene class
	LOG("Constructing scene.");

	// Initialize the class variables from arguments
	this->partInterval = partInterval;
	this->envelopeInterval = envelopeInterval;
	this->referencePart = referencePart;
	this->envelope = envelope;

	// Initialize optimiation variables
	for (int i = 0; i < this->totalMoves - 1; i++)
	{
		this->quality.push_back(0.0);
		this->deltaObj.push_back(0.0);
		this->probability.push_back(this->minProb + this->remProb / this->totalMoves);
		this->probability.push_back(0);
	}

	// Initialize other variables
	this->partCollisions.fill({}); // Should fill the collision array with zeros
	this->optPreObjectives.fill({});
	this->sceneVolume = envelope.prod();
}

int Scene::add_part(Eigen::Vector3d location, bool random)
{
	// Add a object to the scene
	this->nParts++;
	this->idCount++;
	Object* pObj = new Object();

	// Add new object to pointer vector & envelop 
	this->envelopeCollisions.push_back(0);
	this->objects.push_back(pObj);

	if (random)
	{
		// Initialize the part to a random location within the build volume
		pObj->affine(0, 3) = util::rand_double(0.0, this->envelope(0));
		pObj->affine(1, 3) = util::rand_double(0.0, this->envelope(1));
		pObj->affine(2, 3) = util::rand_double(0.0, this->envelope(2));
	}
	else
	{
		// Initialize the part using the location vector provided
		pObj->affine.block(0, 3, 3, 1) = location;
	}

	return this->objects.size() - 1;
}

void Scene::remove_part(int index)
{
	// Removes a part from the scene

	// Remove collision data
	this->remove_collision_pairs(index);
	this->envelopeCollisions.erase(this->envelopeCollisions.begin() + index);

	// Locate the part pointer and delete the object
	delete this->objects[index];
	this->objects.erase(this->objects.begin()+index);

	// Decrement our part counter
	this->nParts--;
}

void Scene::translate_part(int index, Eigen::Vector4d vector)
{
	// Translate a part (given by index) by a relative vector
	this->objects[index]->translate(vector);
}

void Scene::rotate_part(int index, int axis, double angle)
{
	// Rotate a part (given by index) on axis (0 - x, 1 - y, 2 - z) by an angle (default is degrees)
	this->objects[index]->rotate(angle, axis);
}

void Scene::swap_parts(int index1, int index2)
{
	// Swaps the locations of two parts (without modifying their rotations)
	Eigen::Vector4d temp;

	// This is the only part modification method that does not create backups in its constituents.
	this->objects[index1]->backup();
	this->objects[index2]->backup();

	// Swap the object centerpoints
	temp = this->objects[index1]->get_center();
	this->objects[index1]->affine.col(3) = this->objects[index2]->get_center();
	this->objects[index2]->affine.col(3) = temp;
}

void Scene::part_collisions(int index)
{
	// Calculates part collisions between part at given index in the instance object vector and all other objects.
	double broad_distance = 2 * this->referencePart.rootRadius + this->partInterval;
	int collisions = 0;
	int r, c;

	if (this->nParts > 1)
	// Only perform collision calculations if the number of parts in the scene is 2 or greater.
	{
		// BROAD PHASE ALGORITHM
		for (int i = 0; i < this->nParts - 1; i++)
		{
			if (i != index)
			// Do not test part collision against itself
			{
				// Test part center 2 center collision
				if (util::sphere_collision(this->objects[index]->get_center(), this->objects[i]->get_center(), broad_distance))
				{
					// NARROW PHASE ALGORITHM
					collisions = 0;
					this->referencePart.part_collision_test(this->objects[index]->affine, this->objects[i]->affine, 1, 1, this->partInterval, collisions);

					// Store collision data
					r = index < i ? index : i;
					c = index < i ? i : index;
					this->partCollisions(r, c) = collisions;
				}
			}
		}
	}
}

void Scene::envelope_collisions(int index)
{
	// Calculates the number of leaf nodes outside of the build envelope
	double broad_distance = this->referencePart.rootRadius + this->partInterval;
	int collisions = 0;

	if (this->nParts > 0)
	// Don't run this check if there are zero parts
	{
		// BROAD PHASE ALGORITHM
		if (util::outside_envelope(this->objects[index]->get_center(), this->envelope, broad_distance, 0))
		{
			// Complete outlier detected
			this->envelopeCollisions[index] = this->referencePart.leafNodeCount;
		}
		else if (util::outside_envelope(this->objects[index]->get_center(), this->envelope, broad_distance))
		{
			// Partial outlier detected: NARROW PHASE ALGORITHM
			this->referencePart.envelope_collision_test(
				this->objects[index]->affine, 1, this->envelope, this->envelopeInterval, collisions
			);
			this->envelopeCollisions[index] = collisions;
		}
		else
		{
			// No collisions
			this->envelopeCollisions[index] = 0;
		}
	}

}

int Scene::total_collisions(std::vector<int> indices)
{
	// Calculates the collisions for a set of indices. Then sums all of the collisions in the entire scene.

	// Loop through the part indices
	for (int i = 0; i < indices.size(); i++)
	{
		// Calculate the collisions for these parts
		this->part_collisions(indices[i]);
		this->envelope_collisions(indices[i]);
	}

	return this->sum_collisions();
}

int Scene::sum_collisions()
{
	// Sums all of the collisions in the scene
	int total = 0;

	// Loop
	for (int i = 0; i < this->nParts; i++)
	{
		
		// Sum the part 2 part collisions
		for (int j = i; j < this->nParts; j++)
		{
			total += this->partCollisions(i, j);
		}

		// Sum the part 2 envelope collisions
		total += this->envelopeCollisions[i];
	}

	return total;
}

double Scene::packing_density()
{
	// Calculates the packing density of the scene

	// Get number of parts that are COMPLETELY inside the scene
	int cnt = 0;
	for (int i = 0; i < this->envelopeCollisions.size(); i++)
	{
		if (this->envelopeCollisions[i] == 0)
		{
			cnt++;
		}
	}

	return cnt * this->referencePart.mesh_v / this->sceneVolume;

}

std::vector<int> Scene::update_state()
{
	// Select a move and perform a state update. Returns vector {moveIndex, partIndex, [partIndex2]}
	int moveSelection = util::rand_select(this->probability);
	int partSelection, partSelection2;
	std::vector<int> selections;
	selections.push_back(moveSelection);

	// Select random part(s) to be moved
	partSelection = util::rand_int(this->nParts - 1);
	partSelection2 = util::rand_int(this->nParts - 1);
	while (partSelection == partSelection2)
	// Protect from selecting the same part (in case of swap move)
	{
		partSelection2 = util::rand_int(this->nParts - 1);
	}

	// Select a move to update the state
	if (moveSelection < this->transMoves)
	// Translation Move Selected
	{
		// Generate random unit vector and scale it accordingly
		double mag = this->minTrans + moveSelection * this->step;
		Eigen::Vector4d trans{1, 1, 1, 1};
		trans.segment(0, 3) = mag * util::rand_unit_vector();

		// Translate the selected part
		this->translate_part(partSelection, trans);
		selections.push_back(partSelection);
	}
	else if (moveSelection == this->transMoves)
	// Rotation Move Selected
	{
		this->rotate_part(partSelection, util::rand_int(2), 90 * (util::rand_int(2) + 1));
		selections.push_back(partSelection);
	}
	else if (moveSelection == this->transMoves + 1)
	// Swap Move Selected
	{
		this->swap_parts(partSelection, partSelection2);
		selections.push_back(partSelection);
		selections.push_back(partSelection2);
	}
	else if (moveSelection == this->transMoves + 2)
	// Add Part Move Selected
	{
		Eigen::Vector3d dummy;
		int ind;
		ind = this->add_part(dummy, 1.0);
		selections.push_back(ind);
	}
	else
	// Remove Part Move Selected
	{
		this->remove_part(partSelection);
	}

	return selections;
}

double Scene::objective_function(std::vector<int> moves)
{
	// Calculates the current value of the objective function for the scene
	double collisions, density;

	density = 1.0 / this->packing_density();
	this->cD = max(this->cD, density);

	if (moves[0] == this->totalMoves - 1)
	// Part has been removed from the scene - collisions do not need re-calculation
	{
		collisions = this->sum_collisions();
		this->cC = max(this->cC, collisions);
	}
	else
	// Collisions need to be calculated
	{
		collisions = this->total_collisions(std::vector<int>(moves.begin() + 1, moves.end()));
		this->cC = max(this->cC, collisions);
	}

	return this->cC * collisions + this->cD * density;
}

void Scene::optimize()
{
	// Optimize the scene (Simulated Annealing)

	// Pre-process the optimizer (calculate initial annealing temperature)
	// Add a naive packing guess of how many parts can fit in the build
	int preParts = round(this->sceneVolume / this->referencePart.bboxVol);
	Eigen::Vector3d dummy;
	for (int i = 0; i < preParts; i++)
	{
		int ind = this->add_part(dummy, 1.0);
	}
	printf("%i parts generated for preprocessing.\n", this->nParts);

	// Random walk through the solution space
	std::vector<int> moves;
	for (int i = 0; i < this->optPreSteps; i++)
	{
		moves = this->update_state();
		this->optPreObjectives(i) = this->objective_function(moves);
	}
	// Calculate the initial temperature for annealing
	double std_dev = sqrt((this->optPreObjectives.array() - this->optPreObjectives.mean()).square().sum() / (this->optPreObjectives.size() - 1));
	double Ti = -3 * std_dev / log(0.85);
	LOG("Initial Temperature for annealing calculated as:");
	LOG(Ti);

}


