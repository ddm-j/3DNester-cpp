// 3DNester.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <chrono>
#include "Octree.h"
#include "Scene.h"

#define LOG(x) std::cout << x << std::endl

int main()
{
    LOG("Firing up the 3D Nester");

    // Create a part
    Octree tree("GenerativeBracket.stl", 10.0);

    // Create a scene
    Eigen::Vector3d envelope = { 380, 284, 380 };
    Scene scene(tree, envelope, 2.5, 2.5);

 

    // Test collision pairing matrix removal
    Eigen::Vector3d n;
    int cnt = 0;
    int max = 500;
    LOG("Starting collision removal loop");
    for (int i = 0; i < max; i++)
    {
        // Add a part
        scene.add_part(n, 1.0);

        // Add phony collision data
        for (int j = i; j < max; j++)
        {
            //scene.collisionArray(i, j) = cnt;
            cnt++;
        }
    }

    LOG("Testing collisions");
    auto start = chrono::high_resolution_clock::now();
    scene.part_collisions(1);
    auto stop = chrono::high_resolution_clock::now();
    auto diff = chrono::duration_cast<chrono::milliseconds>(stop - start);
    LOG(diff.count());
    
    /*
    for (int i = 0; i < max; i++)
    {
        scene.part_collisions(i);
    }
    */

    //LOG(scene.collisionArray);

    /*

    // Before removal
    LOG(scene.collisionArray);
    
    scene.remove_part(5);
    
    // After removal
    LOG(scene.collisionArray);

    scene.remove_part(3);

    LOG(scene.collisionArray);

    LOG(scene.nParts);

    */


    /*

    // Run through the parts in the scene and log out their local CS origin (location)
    LOG("Logging scene part locations.");
    for (int i = 0; i < scene.objects.size(); i++)
    {
        LOG(scene.objects[i]->get_center());
    }

    */

}
