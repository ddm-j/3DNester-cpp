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
	this->collisionArray.fill({}); // Should fill the collision array with zeros
}

void Scene::add_part(Eigen::Vector3d location, bool random)
{
	// Add a object to the scene
	this->nParts++;
	this->idCount++;
	Object* pObj = new Object();

	// Add new object to pointer vector
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

	// Locate the part pointer and delete the object
	delete this->objects[index];
	this->objects.erase(this->objects.begin()+index);

	// Decrement our part counter
	this->nParts--;
}

void Scene::part_collisions(int index)
{
	// Calculates part collisions between part at given index in object vector and all other objects.
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
					this->referencePart.collision_test(this->objects[index]->affine, this->objects[i]->affine, 1, 1, this->partInterval, collisions);

					// Store collision data
					r = index < i ? index : i;
					c = index < i ? i : index;
					this->collisionArray(r, c) = collisions;
				}
			}
		}
	}
}


