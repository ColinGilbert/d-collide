#include "world.h"
#include "timing.h"
#include "shapes/mesh/triangle.h"
#include "shapes/mesh/vertex.h"
#include "shapes/shapes.h"
#include "bvhnode.h"
#include "narrowphase/narrowphase.h"
#include "collisioninfo.h"
#include "proxy.h"

#include <iostream>
#include <vector>

#define NUMBER_OF_RUNS 2000
using namespace dcollide;

Mesh* createMeshifiedBox(const Vector3& boxDimensions, const Vector3& offset = Vector3()) {
    std::vector<Vertex*> vertices(8);
    std::vector<Triangle*> triangles(6 * 2);
    vertices[0] = new Vertex(offset + Vector3(0.0f, 0.0f, 0.0f));
    vertices[1] = new Vertex(offset + Vector3(0.0f, 0.0f, boxDimensions.getZ()));
    vertices[2] = new Vertex(offset + Vector3(0.0f, boxDimensions.getY(), 0.0f));
    vertices[3] = new Vertex(offset + Vector3(0.0f, boxDimensions.getY(), boxDimensions.getZ()));
    vertices[4] = new Vertex(offset + Vector3(boxDimensions.getX(), 0.0f, 0.0f));
    vertices[5] = new Vertex(offset + Vector3(boxDimensions.getX(), 0.0f, boxDimensions.getZ()));
    vertices[6] = new Vertex(offset + Vector3(boxDimensions.getX(), boxDimensions.getY(), 0.0f));
    vertices[7] = new Vertex(offset + Vector3(boxDimensions.getX(), boxDimensions.getY(), boxDimensions.getZ()));
    triangles[0] = new Triangle(vertices[2], vertices[1], vertices[0]);
    triangles[1] = new Triangle(vertices[1], vertices[2], vertices[3]);
    triangles[2] = new Triangle(vertices[4], vertices[5], vertices[6]);
    triangles[3] = new Triangle(vertices[5], vertices[7], vertices[6]);
    triangles[4] = new Triangle(vertices[0], vertices[1], vertices[4]);
    triangles[5] = new Triangle(vertices[1], vertices[5], vertices[4]);
    triangles[6] = new Triangle(vertices[2], vertices[6], vertices[3]);
    triangles[7] = new Triangle(vertices[3], vertices[6], vertices[7]);
    triangles[8] = new Triangle(vertices[0], vertices[4], vertices[2]);
    triangles[9] = new Triangle(vertices[2], vertices[4], vertices[6]);
    triangles[10] = new Triangle(vertices[1], vertices[3], vertices[5]);
    triangles[11] = new Triangle(vertices[5], vertices[3], vertices[7]);

    Mesh* mesh = new Mesh(vertices, triangles);
    return mesh;
}

/*!
 *\brief test intersection of two tetraeder meshes in the narrowphase
 * can be tested at different detail levels/strategies
 *
 * no broad/or middlephase used here, just the narrowphase code is tested
 * the proxies do not move wile the run, so the worldpositions stay valid
 * for the whole testrun
 */
void narrowPhaseTetraederTest(NarrowPhaseStrategy strategy) {
    World world(Vector3(1000, 1000, 1000));
    world.setUseCollisionCaching(false);
    //----------Mesh-Mesh performance check-----------//
    //create two Meshes with 4 Triangles each, let them collide and check
    //NP-speed for levels 1 and 2

    //---------Mesh1----------//
    std::vector<Triangle*> mesh1Triangles;
    std::vector<Vertex*> mesh1Vertices;

    Vertex* v1a0 = new Vertex(Vector3(0, 0, 0));
    Vertex* v1a1 = new Vertex(Vector3(1, 0, 0));
    Vertex* v1a2 = new Vertex(Vector3(0, 1, 0));

    mesh1Vertices.push_back(v1a0);
    mesh1Vertices.push_back(v1a1);
    mesh1Vertices.push_back(v1a2);

    mesh1Triangles.push_back(new Triangle(v1a0, v1a1, v1a2));

    Vertex* v1b0 = new Vertex(Vector3((real)(0.1), 0, (real)(0.2)));
    Vertex* v1b1 = new Vertex(Vector3((real)(1.1), 0, (real)(0.2)));
    Vertex* v1b2 = new Vertex(Vector3((real)(0.1), 1, (real)(0.2)));

    mesh1Vertices.push_back(v1b0);
    mesh1Vertices.push_back(v1b1);
    mesh1Vertices.push_back(v1b2);

    mesh1Triangles.push_back(new Triangle(v1b0, v1b1, v1b2));

    Vertex* v1c0 = new Vertex(Vector3((real)(0.2), (real)(0.2), (real)(0.4)));
    Vertex* v1c1 = new Vertex(Vector3((real)(1.2), (real)(0.1), (real)(0.4)));
    Vertex* v1c2 = new Vertex(Vector3((real)(0.2), (real)(1.1), (real)(0.4)));

    mesh1Vertices.push_back(v1c0);
    mesh1Vertices.push_back(v1c1);
    mesh1Vertices.push_back(v1c2);

    mesh1Triangles.push_back(new Triangle(v1c0, v1c1, v1c2));

    Vertex* v1d0 = new Vertex(Vector3((real)(0.5), (real)(0.1), (real)(0.7)));
    Vertex* v1d1 = new Vertex(Vector3((real)(0.8), (real)(0.4), (real)(0.4)));
    Vertex* v1d2 = new Vertex(Vector3((real)(0.4), 3, (real)(0.9)));

    mesh1Vertices.push_back(v1d0);
    mesh1Vertices.push_back(v1d1);
    mesh1Vertices.push_back(v1d2);

    mesh1Triangles.push_back(new Triangle(v1d0, v1d1, v1d2));

    //--------Mesh2-----------//
    std::vector<Triangle*> mesh2Triangles;
    std::vector<Vertex*> mesh2Vertices;

    Vertex* v2a0 = new Vertex(Vector3(1, (real)(0.7), 0));
    Vertex* v2a1 = new Vertex(Vector3(0, (real)(0.2), 1));
    Vertex* v2a2 = new Vertex(Vector3(0, (real)(1.2), -1));

    mesh2Vertices.push_back(v2a0);
    mesh2Vertices.push_back(v2a1);
    mesh2Vertices.push_back(v2a2);


    mesh2Triangles.push_back(new Triangle(v2a0, v2a1, v2a2));

    Vertex* v2b0 = new Vertex(Vector3((real)(1.1), (real)(0.5), (real)(0.2)));
    Vertex* v2b1 = new Vertex(Vector3((real)(0.3), (real)(0.3), (real)(1.2)));
    Vertex* v2b2 = new Vertex(Vector3((real)(0.2), (real)(1.1), (real)(-1.2)));

    mesh2Vertices.push_back(v2b0);
    mesh2Vertices.push_back(v2b1);
    mesh2Vertices.push_back(v2b2);

    mesh2Triangles.push_back(new Triangle(v2b0, v2b1, v2b2));

    Vertex* v2c0 = new Vertex(Vector3((real)(1.3), (real)(0.6), (real)(0.4)));
    Vertex* v2c1 = new Vertex(Vector3((real)(0.2), (real)(0.4), (real)(1.4)));
    Vertex* v2c2 = new Vertex(Vector3((real)(0.4), (real)(1.5), (real)(-1.5)));

    mesh2Vertices.push_back(v2c0);
    mesh2Vertices.push_back(v2c1);
    mesh2Vertices.push_back(v2c2);

    mesh2Triangles.push_back(new Triangle(v2c0, v2c1, v2c2));

    Vertex* v2d0 = new Vertex(Vector3((real)(0.8), (real)(0.8), (real)(-0.2)));
    Vertex* v2d1 = new Vertex(Vector3((real)(0.5), (real)(0.5), (real)(1.5)));
    Vertex* v2d2 = new Vertex(Vector3((real)(-0.2), (real)(1.8), (real)(-1.5)));

    mesh2Vertices.push_back(v2d0);
    mesh2Vertices.push_back(v2d1);
    mesh2Vertices.push_back(v2d2);

    mesh2Triangles.push_back(new Triangle(v2d0, v2d1, v2d2));

    //-------create proxies and prepare world------//
    Proxy* simpleProxy1 = world.createProxy(new Mesh(mesh1Vertices, mesh1Triangles));
    Proxy* simpleProxy2 = world.createProxy(new Mesh(mesh2Vertices, mesh2Triangles));


    world.addProxy(simpleProxy1);
    world.addProxy(simpleProxy2);

    //we want to test narrowphase only, so we just skip everything else
    //by creating a middlephasecollision manually
    BoundingVolumeCollision coll;
    coll.node1 = simpleProxy1->getBvHierarchyNode()->getChildren().front();
    coll.node2 = simpleProxy2->getBvHierarchyNode()->getChildren().front();

    NarrowPhaseShapeStrategies strategies;
    strategies.mStrategyMeshMesh = strategy;
    NarrowPhase np(strategies);

    //--------------------//

    Timing t;
    for (int i=0; i<NUMBER_OF_RUNS; i++) {
        np.getIntersection(coll);
    }
    t.stop();

    std::cout << "tetraeder-tetraeder lvl"<< strategy <<": average time from "<<NUMBER_OF_RUNS<<" runs: "<< t.elapsedTime()/NUMBER_OF_RUNS << "us." << std::endl;
}

/*!
 *\brief test intersection of two boxes in the narrowphase
 * tests box-box intersection at lvl2
 * as a benchmark to be beaten, this test also calculates the avg. time of two
 * colliding meshes representing boxes with exactly the same size/orientation
 *
 * no broad/or middlephase used here, just the narrowphase code is tested
 * the proxies do not move wile the run, so the worldpositions stay valid
 * for the whole testrun
 */
void narrowPhaseBoxBoxTest() {
    World world(Vector3(1000, 1000, 1000));
    world.setUseCollisionCaching(false);
    //create two rotated boxes that collide and call NarrowPhase several times

    Proxy* box1proxy = world.createProxy(new Box(Vector3(10, 10, 10)));
    Matrix rot;
    rot.rotate(20, 1, 0, 0);
    rot.rotate(20, 0, 1, 0);
    rot.rotate(20, 0, 0, 1);
    box1proxy->rotate(rot);
    world.addProxy(box1proxy);

    Proxy* box2proxy = world.createProxy(new Box(Vector3(10, 10, 10)));
    box2proxy->translate(5, 2, 1);
    world.addProxy(box2proxy);

    //we want to test narrowphase only, so we just skip everything else
    //by creating a middlephasecollision manually
    BoundingVolumeCollision coll;
    coll.node1 = box1proxy->getBvHierarchyNode()->getChildren().front();
    coll.node2 = box2proxy->getBvHierarchyNode()->getChildren().front();

    NarrowPhaseShapeStrategies strategies;
    strategies.mStrategyBoxBox = NP_STRATEGY_FAST_CONSIDER_SHAPE;
    strategies.mStrategyMeshMesh = NP_STRATEGY_FAST_CONSIDER_SHAPE;

    NarrowPhase np(strategies);

    Timing t;
    for (int i=0; i<NUMBER_OF_RUNS; i++) {
        np.getIntersection(coll);
    }
    t.stop();

    std::cout << "box-box lvl2 average time: from "<<NUMBER_OF_RUNS<<" runs: "<< t.elapsedTime()/NUMBER_OF_RUNS << "us." << std::endl;


    //--------------------//

    //now the same thing again, just with meshes representing the boxes
    //first, move the original box-proxies out of the way
    box1proxy->translate(500, 0, 0);
    box2proxy->translate(-500, 0, 0);

    Proxy* meshBox1proxy = world.createProxy(createMeshifiedBox(Vector3(10, 10, 10)));
    rot.loadIdentity();
    rot.rotate(20, 1, 0, 0);
    rot.rotate(20, 0, 1, 0);
    rot.rotate(20, 0, 0, 1);
    meshBox1proxy->rotate(rot);
    world.addProxy(meshBox1proxy);

    Proxy* meshBox2proxy = world.createProxy(createMeshifiedBox(Vector3(10, 10, 10)));
    meshBox2proxy->translate(5, 2, 1);
    world.addProxy(meshBox2proxy);

    //we want to test narrowphase only, so we just skip everything else
    //by creating a middlephasecollision manually
    coll.node1 = meshBox1proxy->getBvHierarchyNode()->getChildren().front();
    coll.node2 = meshBox2proxy->getBvHierarchyNode()->getChildren().front();

    strategies.mStrategyMeshMesh = NP_STRATEGY_FAST_CONSIDER_SHAPE;

    //--------------------//

    t.restart();
    for (int i=0; i<NUMBER_OF_RUNS; i++) {
        np.getIntersection(coll);
    }
    t.stop();

    std::cout << "box-box (Mesh version lvl2) average time: from "<<NUMBER_OF_RUNS<<" runs: "<< t.elapsedTime()/NUMBER_OF_RUNS << "us." << std::endl;

}

/*!
 *\brief test intersection of a box and a sphere in the narrowphase
 *
 * tests sphere-box intersection at lvl2
 *
 * no broad/or middlephase used here, just the narrowphase code is tested
 * the proxies do not move wile the run, so the worldpositions stay valid
 * for the whole testrun
 */
void narrowPhaseSphereBoxTest() {
    World world(Vector3(1000, 1000, 1000));
    world.setUseCollisionCaching(false);
    //create two rotated boxes that collide and call NarrowPhase several times

    Proxy* box1proxy = world.createProxy(new Box(Vector3(10, 10, 10)));
    Matrix rot;
    rot.rotate(20, 1, 0, 0);
    rot.rotate(20, 0, 1, 0);
    rot.rotate(20, 0, 0, 1);
    box1proxy->rotate(rot);
    world.addProxy(box1proxy);

    Proxy* sphere = world.createProxy(new Sphere(15));
    sphere->translate(5, 2, 1);
    world.addProxy(sphere);

    //we want to test narrowphase only, so we just skip everything else
    //by creating a middlephasecollision manually
    BoundingVolumeCollision coll;
    coll.node1 = box1proxy->getBvHierarchyNode()->getChildren().front();
    coll.node2 = sphere->getBvHierarchyNode()->getChildren().front();

    NarrowPhaseShapeStrategies strategies;
    //strategies.mStrategyBoxBox = NP_STRATEGY_FAST_CONSIDER_SHAPE;
    //strategies.mStrategyMeshMesh = NP_STRATEGY_FAST_CONSIDER_SHAPE;

    NarrowPhase np(strategies);

    Timing t;
    for (int i=0; i<NUMBER_OF_RUNS; i++) {
        np.getIntersection(coll);
    }
    t.stop();

    std::cout << "sphere-box lvl2 average time: from "<<NUMBER_OF_RUNS<<" runs: "<< t.elapsedTime()/NUMBER_OF_RUNS << "us." << std::endl;
}

void vectorIteratorTest() {
    World world;

    //Vector iterating / size precaching test
    std::vector<Proxy*> vec;
    for (int i = 0; i<10000; ++i) {
        vec.push_back(world.createProxy());
    }

    std::cout << "Vector vec contains " << vec.size() << " elements" <<std::endl;
    //running through the vector index-based without size precaching
    Timing t;

    for (int run = 0; run < NUMBER_OF_RUNS; ++run) {
        for (unsigned int i=0; i < vec.size(); ++i) {
            Proxy* p = vec[i];
        }
    }
    t.stop();
    std::cout << "for (unsigned int i=0; i < vec.size(); ++i) {Proxy* p = vec[i];} : "<< t.elapsedTime() / NUMBER_OF_RUNS << "us." << std::endl;

    //running through the vector index-based with size precaching (pre-increment)
    t.restart();

    for (int run = 0; run < NUMBER_OF_RUNS; ++run) {
        unsigned int vecSize = vec.size();
        for (unsigned int i=0; i < vecSize; ++i) {
            Proxy* p = vec[i];
        }
    }
    t.stop();
    std::cout << "unsigned int vecSize = vec.size(); for (unsigned int i=0; i < vecSize; ++i) {Proxy* p = vec[i];} : "<< t.elapsedTime() / NUMBER_OF_RUNS << "us." << std::endl;


    //running through the vector index-based with size precaching (post-increment)
    t.restart();

    for (int run = 0; run < NUMBER_OF_RUNS; ++run) {
        unsigned int vecSize = vec.size();
        for (unsigned int i=0; i < vecSize; i++) {
            Proxy* p = vec[i];
        }
    }
    t.stop();
    std::cout << "unsigned int vecSize = vec.size(); for (unsigned int i=0; i < vecSize; i++) {Proxy* p = vec[i];} : "<< t.elapsedTime() / NUMBER_OF_RUNS << "us." << std::endl;



    //running through the vector with iterator (pre-increment)
    t.restart();

    for (int run = 0; run < NUMBER_OF_RUNS; ++run) {
        for (std::vector<Proxy*>::iterator iter = vec.begin(); iter != vec.end(); ++iter) {
            Proxy* p = *iter;
        }
    }
    t.stop();
    std::cout << "for (std::vector<Proxy*>::iterator iter = vec.begin(); iter != vec.end(); ++iter) {Proxy* p = *iter;} : "<< t.elapsedTime() / NUMBER_OF_RUNS << "us." << std::endl;

    //running through the vector with iterator (post-increment)
    t.restart();

    for (int run = 0; run < NUMBER_OF_RUNS; ++run) {
        for (std::vector<Proxy*>::iterator iter = vec.begin(); iter != vec.end(); iter++) {
            Proxy* p = *iter;
        }
    }
    t.stop();
    std::cout << "for (std::vector<Proxy*>::iterator iter = vec.begin(); iter != vec.end(); iter++) {Proxy* p = *iter;} : "<< t.elapsedTime() / NUMBER_OF_RUNS << "us." << std::endl;


}

void incrementOperatorTest() {
    //increment-operator for ints
    //pre-increment
    Timing t;
    for (int i = 0; i< 1000000; ++i) {}
    t.stop();
    std::cout << "pre-increment: for (int i = 0; i< 1000000; ++i) {} : "<< t.elapsedTime() << "us." << std::endl;

    //post-increment
    t.restart();
    for (int j = 0; j< 1000000; j++) {}
    t.stop();
    std::cout << "postincrement: for (int i = 0; i< 1000000; i++) {} : "<< t.elapsedTime() << "us." << std::endl;
}


int main() {
    narrowPhaseTetraederTest(NP_STRATEGY_FASTEST_SLOPPY);
    narrowPhaseTetraederTest(NP_STRATEGY_FAST_CONSIDER_SHAPE);
    narrowPhaseBoxBoxTest();
    narrowPhaseSphereBoxTest();

    vectorIteratorTest();
    incrementOperatorTest();

    //GJ: these are my results from rev 1195 (28.6.07)
    //tetraeder-tetraeder lvl1: average time from 2000 runs: 5us.
    //tetraeder-tetraeder lvl2: average time from 2000 runs: 176us.
    //box-box lvl2 average time: from 2000 runs: 436us.
    //box-box (Mesh version lvl2) average time: from 2000 runs: 400us.
    //at this revision, the box-box test is not activated yet, so the times
    //should be nearly identical.
    //In the box-box test, the meshes are generated while running, the first
    //iteration so the first case should actually be somewhat slower

    return 0;
}
