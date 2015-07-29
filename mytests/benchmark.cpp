#include <d-collide/world.h>
#include <d-collide/timing.h>
#include <d-collide/shapes/mesh/triangle.h>
#include <d-collide/shapes/mesh/vertex.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/bvhnode.h>
#include <d-collide/narrowphase/narrowphase.h>
#include <d-collide/collisioninfo.h>
#include <d-collide/proxy.h>
#include <d-collide/worldcollisions.h>
#include <d-collide/timing.h>

#include <modelloader/loader3ds.h>

#include <iostream>
#include <vector>
#include <stdlib.h>

#define NUMBER_OF_RUNS 2000

dcollide::Mesh* createTetraeder() {
        //Mesh-Tetraeder
/*
        3 \
       / \ \
      /  \  \
     /  --\---2
    0----  \ /
       -----1
*/
    using namespace dcollide;
    Vertex* v0 = new Vertex(Vector3(0, 0, 0));
    Vertex* v1 = new Vertex(Vector3(75, 0, 0));
    Vertex* v2 = new Vertex(Vector3(50, 50, 0));
    Vertex* v3 = new Vertex(Vector3(50, 50, 75));

    Triangle* t0 = new Triangle(v0, v2, v1);
    Triangle* t1 = new Triangle(v0, v1, v3);
    Triangle* t2 = new Triangle(v1, v2, v3);
    Triangle* t3 = new Triangle(v2, v0, v3);

    std::vector<Vertex*> vertices;
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);

    std::vector<Triangle*> triangles;
    triangles.push_back(t0);
    triangles.push_back(t1);
    triangles.push_back(t2);
    triangles.push_back(t3);

    return new Mesh(vertices, triangles);
}

int main() {
    std::cout << "starting benchmark" << std::endl;
    using namespace dcollide;
    //Benchmark:
    // create some boxes, spheres and meshes, randomly (but reproducable)
    // placed within a room of 1000x1000x1000 units)
    // move them around and get all collisions 
    
    WorldParameters parameters;

    std::list<DeformableAlgorithmType> algorithm;
    //algorithm.push_back(DEFORMABLE_TYPE_BV_HIERARCHY);
    algorithm.push_back(DEFORMABLE_TYPE_SPATIAL_HASH);
    //algorithm.push_back(DEFORMABLE_TYPE_SURFACE_HIERARCHY);

    parameters.setWorldDimension(Vector3(2000.0f,2000.0f,2000.0f));
    parameters.setDeformableAlgorithms(algorithm);
    parameters.setWorkerThreadCount(4);

    World world(parameters);

    //TODO add some proxy hierarchy elements

    std::vector<Proxy*> allElements;
    std::vector<long int> measuredTimings;
    
    Timing timer;
    ModelLoader::Loader3ds loader;

    //initialize pseudo-random-number generator with reproducable seed
    int seed = 12345;
    srand(seed);
    //usage: 
    //int randomValue = rand() % 100 + 1; //Generates value between 1 and 100
    
    //stop timer for now
    timer.stop();
    
    std::cout << "generating objects" << std::endl;
    //generate 200 objects of each type
    for (int i = 0; i<50; i++) {
        //Box 10x5x15
        Proxy* boxProxy = world.createProxy(new dcollide::Box(50, 25, 75));
        world.addProxy(boxProxy);
        allElements.push_back(boxProxy);

        //Sphere radius 8
        Proxy* sphereProxy = world.createProxy(new dcollide::Sphere(40));
        world.addProxy(sphereProxy);
        allElements.push_back(sphereProxy);

        //Mesh (tetraeder)
        Proxy* meshProxy = world.createProxy(createTetraeder());
        world.addProxy(meshProxy);
        allElements.push_back(meshProxy);
    }

    for (int i = 0; i<5; i++) {
        //Helicopter Model
        Mesh* helicopterMesh = loader.loadFromFileToOneMesh("puma.3ds");
        if (helicopterMesh) {
            Proxy* helicopterProxy = world.createProxy(helicopterMesh);
            world.addProxy(helicopterProxy);
            allElements.push_back(helicopterProxy);
        } else {
            std::cerr << "Couldn't load helicopter model!" << std::endl;
        }
    }

    for (int i = 0; i<1; i++) {
        //Helicopter Model
        Mesh* helicopterMesh = loader.loadFromFileToOneMesh("puma.3ds");
        if (helicopterMesh) {
            Proxy* helicopterProxy = world.createProxy(helicopterMesh, (dcollide::ProxyTypes)(dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_CLOSEDHULL | dcollide::PROXYTYPE_SELFCOLLIDABLE));
            world.addProxy(helicopterProxy);
            allElements.push_back(helicopterProxy);
        } else {
            std::cerr << "Couldn't load helicopter model!" << std::endl;
        }
    }
    
    std::cout << "placing objects in the world" << std::endl;
    //random initial placement and orientation of the proxies
    for (std::vector<Proxy*>::iterator proxyIter = allElements.begin();
                                proxyIter != allElements.end(); proxyIter++) {
        Proxy* element = *proxyIter;

        Matrix rotation;
        //random rotation around x,y,and z
        rotation.rotate(rand() % 360, 1, 0, 0);
        rotation.rotate(rand() % 360, 0, 1, 0);
        rotation.rotate(rand() % 360, 0, 0, 1);
        element->setRotation(rotation);
        
        element->translate(rand() % 1000, rand() % 1000, rand() % 1000);
    }

    world.prepareSimulation();

    std::cout   << "stepping: moving all objects and calculating collisions"
                << std::endl;
    
    //perform some steps: move/rotate all objects and get all collisions
    int totalSteps = 100;
    int maxRotation = 10; //max rotation per axis
    int maxTranslation = 10; //max translation per axis
    for (int step = 0; step < totalSteps; step++) {
        for (std::vector<Proxy*>::iterator proxyIter = allElements.begin();
                                proxyIter != allElements.end(); proxyIter++) {
            Proxy* element = *proxyIter;
    
            Matrix rotation = element->getRotation();
            //random rotation around x,y,and z
            rotation.rotate(rand() % maxRotation - maxRotation/2, 1, 0, 0);
            rotation.rotate(rand() % maxRotation - maxRotation/2, 0, 1, 0);
            rotation.rotate(rand() % maxRotation - maxRotation/2, 0, 0, 1);
            element->setRotation(rotation);
            
            element->translate( rand() % maxTranslation - maxTranslation/2,
                                rand() % maxTranslation - maxTranslation/2,
                                rand() % maxTranslation - maxTranslation/2);
        }

        //get new collisions and stop time
        timer.restart();
        world.calculateAllCollisions();
        timer.stop();
        measuredTimings.push_back(timer.elapsedTime());
        
        std::cout << "#" << step << std::endl;
    }

    std::cout << "calculating average frame time" << std::endl;
    
    int size = measuredTimings.size();
    double average = measuredTimings[0];
    for (int i = 1; i < size; i++) {
        average += measuredTimings[i];
        average /= 2;
    }
    
    std::cout << "finished benchmark" << std::endl;
    std::cout << " - an average frame took: " << average << "us" << std::endl;
    return 0;
}
