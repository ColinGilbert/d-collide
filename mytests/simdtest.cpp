#include <d-collide/world.h>
#include <d-collide/worldcollisions.h>
#include <d-collide/math/vector.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/collisioninfo.h>
#include <d-collide/debuglog.h>
#include <d-collide/debugstream.h>
#include <d-collide/timing.h>
#include <d-collide/proxy.h>
#include <d-collide/detectordeform/detectordeformmanager.h>
#include <d-collide/detectordeform/spatialhash/spatialhashalgorithm.h>

#include <modelloader/loaderply.h>
#include <modelloader/loaderdummy.h>

#include <iostream>

using namespace dcollide;

#define NUMBER_OF_RUNS 500000



Mesh* createMeshifiedBox(real w, real h, real d) {
    Box box(w, h, d);
    const Mesh* m = box.getMesh();
    return m->cloneMesh();
}

Mesh* createMeshifiedSphere(real r) {
    Sphere sphere(r);
    const Mesh* m = sphere.getMesh();
    return m->cloneMesh();
}

int main() {

    WorldParameters parameters;

    std::list<DeformableAlgorithmType> algorithm;
    //algorithm.push_back(DEFORMABLE_TYPE_BV_HIERARCHY);
    algorithm.push_back(DEFORMABLE_TYPE_SPATIAL_HASH);
    //algorithm.push_back(DEFORMABLE_TYPE_SURFACE_HIERARCHY);

    parameters.setWorldDimension(Vector3(1000.0, 1000.0, 1000.0));
    parameters.setDeformableAlgorithms(algorithm);

    World world(parameters);

    
    Proxy* boxStackBottomLeft;
    Proxy* boxStackBottomMid;
    Proxy* boxStackBottomRight;
    Proxy* boxStackMidLeft;
    Proxy* boxStackMidRight;
    Proxy* boxStackTop;
    
    Proxy* midBox;
    Proxy* leftSphere;
    Proxy* rightSphere;
    
    Proxy* groundBox;
    Proxy* cylinderFirst;
    Proxy* cylinderSecond;
    Proxy* cylinderThird;
    
    Proxy* bunny1;
    Proxy* bunny2;
    
    
    /* Stack of six boxes */
    boxStackBottomLeft  = world.createProxy(new Box(10,10,10), PROXYTYPE_RIGID);
    boxStackBottomMid   = world.createProxy(new Box(10,10,10), PROXYTYPE_RIGID);
    boxStackBottomRight = world.createProxy(new Box(10,10,10), PROXYTYPE_RIGID);

    boxStackMidLeft     = world.createProxy(new Box(10,10,10), PROXYTYPE_RIGID);
    boxStackMidRight    = world.createProxy(new Box(10,10,10), PROXYTYPE_RIGID);

    boxStackTop         = world.createProxy(new Box(10,10,10), PROXYTYPE_RIGID);


    world.addProxy(boxStackBottomLeft);
    world.addProxy(boxStackBottomMid);
    world.addProxy(boxStackBottomRight);

    world.addProxy(boxStackMidLeft);
    world.addProxy(boxStackMidRight);

    world.addProxy(boxStackTop);


    boxStackBottomLeft->setPosition(-100, 0, -5);
    boxStackBottomMid->setPosition(-90, 0, -5);
    boxStackBottomRight->setPosition(-80, 0, -5);

    boxStackMidLeft->setPosition(-95, 10, -5);
    boxStackMidRight->setPosition(-85, 10, -5);

    boxStackTop->setPosition(-90, 20, -5);


    /* Rigid Mesh collision sphere/box/sphere */
    midBox      = world.createProxy(createMeshifiedBox(2, 100, 100), PROXYTYPE_RIGID);
    leftSphere  = world.createProxy(createMeshifiedSphere(50), PROXYTYPE_RIGID);
    rightSphere = world.createProxy(createMeshifiedSphere(50), PROXYTYPE_RIGID);

    world.addProxy(midBox);
    world.addProxy(leftSphere);
    world.addProxy(rightSphere);

    midBox->setPosition(200, 0, 50);
    leftSphere->setPosition(151, 0, 0);
    rightSphere->setPosition(251, 0, 0);


    /* Rigid ground plate with three cylinders */
    groundBox      = world.createProxy(new Box(50, 5, 100), PROXYTYPE_RIGID);
    cylinderFirst  = world.createProxy(new Cylinder(3, 25), PROXYTYPE_RIGID);
    cylinderSecond = world.createProxy(new Cylinder(3, 25), PROXYTYPE_RIGID);
    cylinderThird  = world.createProxy(new Cylinder(3, 25), PROXYTYPE_RIGID);
    
    world.addProxy(groundBox);
    world.addProxy(cylinderFirst);
    world.addProxy(cylinderSecond);
    world.addProxy(cylinderThird);

    cylinderFirst->setRotation(90, 0, 1, 0);
    cylinderSecond->setRotation(90, 0, 1, 0);
    cylinderThird->setRotation(90, 0, 1, 0);
    
    groundBox->setPosition(-25, 0, 50);
    cylinderFirst->setPosition(-20, 7.5, 10);
    cylinderSecond->setPosition(-15, 7.5, -5);
    cylinderThird->setPosition(-10, 7.5, -20);


    world.prepareSimulation();

    Timing t;
    for (int i=0; i < NUMBER_OF_RUNS; i++) {
        try {
            world.calculateAllCollisions();
        } catch (Exception e) {
            e.printBacktrace();
        }
    }
    t.stop();

    std::cout << "An average collision test took "
              << t.elapsedTime()/NUMBER_OF_RUNS
              << "us (measured over "
              << NUMBER_OF_RUNS
              << " runs)."
              << std::endl;

    return 0;
}

/*
 * vim: et sw=4 ts=4
 */
