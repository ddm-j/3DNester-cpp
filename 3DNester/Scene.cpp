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

	// Initialize other variables
	this->partCollisions.fill({}); // Should fill the collision array with zeros
	this->sceneVolume = envelope.prod();
}

void Scene::add_part(Eigen::Vector3d location, bool random)
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


