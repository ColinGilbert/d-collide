/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-users@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,          *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz            *
 *                                                                             *
 *  All rights reserved.                                                       *
 *                                                                             *
 *  Redistribution and use in source and binary forms, with or without         *
 *  modification, are permitted provided that the following conditions are met:*
 *   - Redistributions of source code must retain the above copyright          *
 *     notice, this list of conditions and the following disclaimer.           *
 *   - Redistributions in binary form must reproduce the above copyright       *
 *     notice, this list of conditions and the following disclaimer in the     *
 *     documentation and/or other materials provided with the distribution.    *
 *   - Neither the name of the PG510 nor the names of its contributors may be  *
 *     used to endorse or promote products derived from this software without  *
 *     specific prior written permission.                                      *
 *                                                                             *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR      *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER *
 *  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,   *
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,        *
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR         *
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF     *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING       *
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS         *
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE                *
 *******************************************************************************/
//include directives
#include "narrowphasetest.h"

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <math.h>
#include <list>

#include "world.h"
#include "worldcollisions.h"
#include "broadphase/broadphasecollisions.h"
#include "shapes/shapes.h"
#include "narrowphase/narrowphase.h"
#include "narrowphase/narrowphasestrategy.h"
#include "math/matrix.h"
#include "math/vector.h"
#include "real.h"
#include "proxy.h"

#include "dcollide-defines.h"
#include "narrowphase/triangleintersector.h"

#include <iomanip>

//-------------------------------------
//-------using directives--------------
using namespace std;
using namespace dcollide;

/*
add the class test to a global list of tests
if this line is forgotten - nothing will happen
the test methods will never get called

IMPORTANT:
This registration MUST happen in the .cpp file
If it is done in the header, which is also possible,
the registration gets run every time the header is
included. The Unit Test may be registered several times.
*/

CPPUNIT_TEST_SUITE_REGISTRATION (NarrowPhaseTest);

const float epsilon = 0.0001f;
namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
     void NarrowPhaseTest::setUp(void) {
        //Constructor for Unit Tests
        //not needed here
        //use this to allocate resources as memory, files etc.
     }

    /*!
     *  \brief resource deallocation for unit test
     */
     void NarrowPhaseTest::tearDown(void) {
        //Destructor for Unit Tests
        //not needed here
        //use this to free the resources allocated in setUp()
     }
     void NarrowPhaseTest::sphereSphereTest() {
        World world(Vector3(1000, 1000, 1000));
        world.setNarrowPhaseStrategySphereSphere(NP_STRATEGY_SLOWEST_EXTENSIVE);
        Proxy* sphere1 = world.createProxy(new Sphere(1));
        Proxy* sphere2 = world.createProxy(new Sphere(1));

        world.addProxy(sphere1);
        world.addProxy(sphere2);

        world.prepareSimulation();

        //Convenience matrices for fast resets
        Matrix origin;
        Matrix o500;
        o500.translate(5,0,0);

        /*
        BoundingVolumeCollision mpc;
        mpc.node1 = sphere1->getBvHierarchyNode();
        mpc.node2 = sphere2->getBvHierarchyNode();

        NarrowPhaseShapeStrategies strategies;
        strategies.mStrategySphereSphere = NP_STRATEGY_SLOWEST_EXTENSIVE;
        NarrowPhase np(strategies);
        */
        //--- Test 1: no collision , 2 fixed spheres ---//
        //Two spheres of radius 1, sphere 1 at (1.9, 1.9, 1.9) so that the BVs
        //overlap
        // sphere2 is placed fixed at the origin


        sphere1->translate((real)(1.9), (real)(1.9), (real)(1.9));

        std::list<CollisionInfo> result =
        world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT_EQUAL(true, result.empty());
        //examine result

        //--- Test 2: basic collision---//

        //Two spheres of radius 1, sphere 1 was at (5,0,0) moving to (1.5, 0, 0)
        // sphere2 is placed fixed at the origin
        sphere1->setTransformation(o500);
        world.startNextStep();

        sphere1->translate(Vector3(-3.5f, 0, 0));

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT_EQUAL((size_t) 1, result.size());
        //examine result
        CollisionInfo coll = *result.begin();

        CPPUNIT_ASSERT_EQUAL(Vector3(1, 0, 0), coll.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(0.5f, coll.penetrationDepth);
        CPPUNIT_ASSERT_EQUAL(Vector3(1, 0, 0), coll.normal);

        //--- Test 3: no collision (stops before coll)---//

        //Two spheres of radius 1, sphere 1 at (5,0,0) moving to (4, 0, 0)
        // sphere2 is placed fixed at the origin
        //first reset the sphere1
        sphere1->setTransformation(o500);
        world.startNextStep();

        sphere1->translate(-1, 0, 0);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT(result.empty());

        //--- Test 4: no collision (parallel movement)---//
        //reset to a coordinates with colliding BVs, but no sphere-collision
        sphere2->setTransformation(origin);
        sphere1->setTransformation(origin);
        sphere1->translate(2, 2, 0);
        world.startNextStep();

        //move the spheres up in y-direction
        sphere1->translate(0, 5, 0);
        sphere2->translate(0, 5, 0);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT(result.empty());

        //--- Test 5: one collision, touching at t=1---//
        //sphere1 starts at 5,0,0 and moves to 3,0,0
        //sphere2 moves to 1,0,0 so that the collision is at 2,0,0

        //reset
        sphere2->setTransformation(origin);
        sphere1->setTransformation(o500);
        world.startNextStep();

        sphere1->translate(-2, 0, 0);
        sphere2->translate(1, 0, 0);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        //examine result
        coll = *result.begin();

        CPPUNIT_ASSERT_EQUAL(Vector3(2, 0, 0), coll.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(1, 0, 0), coll.normal);

        CPPUNIT_ASSERT_EQUAL(0.0f, coll.penetrationDepth);

        //--- Test 7: all axis test---//

        //sphere1 going from 2,2,2 to 0,0,0
        //sphere2 going from 0,0,0 to 2,2,2

        sphere1->setTransformation(origin);
        sphere2->setTransformation(origin);
        sphere1->translate(2, 2, 2);

        world.startNextStep();

        sphere1->translate(-2, -2, -2);
        sphere2->translate(2, 2, 2);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        //this is a collision we cannot find without doing backtracking
        CPPUNIT_ASSERT(result.size()==1);
        //examine result
        coll = *result.begin();

        Vector3 expNormal(1, 1, 1);
        expNormal.normalize();
        real expDepth = Vector3(1, 1, 1).length()+1;
        CPPUNIT_ASSERT_EQUAL(Vector3(1, 1, 1), coll.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(expNormal, coll.normal);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(expDepth, coll.penetrationDepth, epsilon);


        //------- Test 8: Collision in the previous state, no collision now---//
        sphere1->setTransformation(origin);
        sphere2->setTransformation(origin);

        world.startNextStep();

        sphere1->translate(2, 2, 2);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT(result.empty());

        //--- Test 9: Collision in the previous state and the current state---//
        sphere1->setTransformation(origin);
        sphere2->setTransformation(origin);

        world.startNextStep();

        sphere1->translate(0.5, 0, 0);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT_EQUAL((size_t) 1, result.size());
        //examine result
        coll = *result.begin();

        //GJ: Note that the simple algorithm is used here
        CPPUNIT_ASSERT_EQUAL(Vector3(0.25, 0, 0), coll.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(1, 0, 0), coll.normal);
        CPPUNIT_ASSERT_EQUAL(1.5f, coll.penetrationDepth);

        //---------Test 10:: touching while passing by---------------//
        //sphere1 is fixed, sphere2 moves from (2, -2, 0) to (2, 2, 0)
        //to touch sphere1 at (1, 0, 0)
        sphere1->setTransformation(origin);
        sphere2->setTransformation(origin);
        sphere2->translate(Vector3(2, -2, 0));
        world.startNextStep();

        //std::cout << "Touching while passing by test" <<std::endl;
        sphere2->translate(Vector3(0, 2, 0));

        result = world.calculateAllCollisions(World::COLLISIONFLAG_NONE).getNarrowPhaseCollisions();

        //another case whithout collision at current state
        CPPUNIT_ASSERT_EQUAL((size_t) 1, result.size());
        //examine result
        coll = *result.begin();

        CPPUNIT_ASSERT_EQUAL(Vector3(1,0,0), coll.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(1,0,0), coll.normal);
        CPPUNIT_ASSERT_EQUAL(0.0f, coll.penetrationDepth);

        //---------Test 11:: collision while passing by---------------//
        //sphere1 is fixed, sphere2 moves from (1, -5, 0) to (1, 2, 0)
        //to get a full collision with sphere 1 on the way
        sphere1->setTransformation(origin);
        sphere2->setTransformation(origin);
        sphere2->translate(Vector3(1, -5, 0));
        world.startNextStep();


        sphere2->translate(Vector3(0, 7, 0));

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        //another case whithout collision at current state
        CPPUNIT_ASSERT_EQUAL((size_t) 1, result.size());
        //examine result
        coll = *result.begin();
        Vector3 expectedCollPoint (0.5, (real)(- sqrt(3.0)/2.0), 0);
        CPPUNIT_ASSERT_EQUAL(expectedCollPoint, coll.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(expectedCollPoint, coll.normal);
        CPPUNIT_ASSERT(coll.penetrationDepth>2.5);

        //---------Test 12a:: starting at a collision and not moving--------//
        //This is kind of difficult. We cannot be sure where the collision was
        //GJ: i propose following interpretation:
        //just return the center of the overlap region as collision point
        sphere1->setTransformation(origin);
        sphere1->translate(Vector3(1, 0, 0));
        sphere2->setTransformation(origin);
        world.startNextStep();

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT(result.size()==1);
        //examine result
        coll = *result.begin();

        CPPUNIT_ASSERT_EQUAL(Vector3(0.5, 0, 0), coll.collisionPoint);
        //we don't care about the direction of the normal
        CPPUNIT_ASSERT( coll.normal == Vector3(1, 0, 0) ||
                        coll.normal == Vector3(-1, 0, 0));
        CPPUNIT_ASSERT_EQUAL(1.0f, coll.penetrationDepth);


        //---------Test 12b:: starting at a collision moving parallel--------//
        sphere1->setTransformation(origin);

        sphere2->setTransformation(origin);
        sphere2->translate(Vector3(1, 0, 0));
        world.startNextStep();

        sphere1->translate(Vector3(0, 1, 0));
        sphere2->translate(Vector3(0, 1, 0));

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT(result.size()==1);
        //examine result
        coll = *result.begin();

        CPPUNIT_ASSERT_EQUAL(Vector3(0.5, 1, 0), coll.collisionPoint);
        //we don't care about the direction of the normal
        CPPUNIT_ASSERT( coll.normal == Vector3(1, 0, 0) ||
                        coll.normal == Vector3(-1, 0, 0));
        CPPUNIT_ASSERT_EQUAL(1.0f, coll.penetrationDepth);


        //---------Test 12c:: starting and staying at one point--------//

        sphere1->setTransformation(origin);
        sphere2->setTransformation(origin);
        world.startNextStep();

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();


        CPPUNIT_ASSERT(result.size()==1);
        //examine result
        coll = *result.begin();

        //collisionpoint should be centerpoint of both spheres
        //collision normal is some random normalized vector
        CPPUNIT_ASSERT_EQUAL(Vector3(0,0,0), coll.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(1.0f, coll.normal.length());
        CPPUNIT_ASSERT_EQUAL(2.0f, coll.penetrationDepth);
     }

    void NarrowPhaseTest::sphereMeshTest() {
        World world(Vector3(1000, 1000, 1000));

        // The Mesh will have only one triangle,
        // the sphere will have a radius of 2 and will be moved around
        // to test area collisions, edge collisions and vertex collisions

        std::stringstream ss;       // <- error message

        std::vector<Vertex*> vertices;
        std::vector<Triangle*> triangles;
        vertices.push_back(new Vertex(Vector3( 0.0,  0.0,  0.0)));

        vertices.push_back(new Vertex(Vector3( 5.0, 0.0,  0.0)));
        vertices.push_back(new Vertex(Vector3( 0.0,  5.0, 0.0)));
        Triangle* t = new Triangle(vertices[0], vertices[1], vertices[2]);
        triangles.push_back(t);

        Mesh* myMesh = new Mesh(vertices, triangles);
        Sphere* mySphere = new Sphere(2.0);

        Proxy* meshProxy = world.createProxy(myMesh);
        Proxy* sphereProxy = world.createProxy(mySphere);

        world.addProxy(meshProxy);
        world.addProxy(sphereProxy);

        world.prepareSimulation();

        //First: area collision from "above"
        sphereProxy->setPosition(1.0, 1.0, 1.0);

        std::list<CollisionInfo> result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CollisionInfo collision = result.front();
        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        CPPUNIT_ASSERT_EQUAL(Vector3(1.0, 1.0, 0.0), collision.collisionPoint);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, collision.penetrationDepth, epsilon);
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 1.0), collision.normal);

        //second test: area collision from "below"
        sphereProxy->setPosition(2.0, 1.0, -1.5);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        collision = result.front();
        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        CPPUNIT_ASSERT_EQUAL(Vector3(2.0, 1.0, 0.0), collision.collisionPoint);
        //the penetration normal always point "out"
        //so the sphere gets pushed back outside the triangle
        CPPUNIT_ASSERT_DOUBLES_EQUAL(3.5, collision.penetrationDepth, epsilon);
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 1.0), collision.normal);

        //third test: edge collisions
        //with edge01
        Vector3 spherePosition (2.5, -1.5, 0.5);
        sphereProxy->setPosition(spherePosition);

        result = world.calculateAllCollisions(World::COLLISIONFLAG_NONE).getNarrowPhaseCollisions();

        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        collision = result.front();
        CPPUNIT_ASSERT_EQUAL(Vector3(2.5, 0.0, 0.0), collision.collisionPoint);
        Vector3 connection (0, -1.5, 0.5); //sphere to collision connection
        CPPUNIT_ASSERT_DOUBLES_EQUAL(2 - connection.length(), collision.penetrationDepth, epsilon);
        connection.normalize();
        CPPUNIT_ASSERT_EQUAL(connection, collision.normal);

        //with edge02 (slightly below)
        sphereProxy->setPosition(Vector3(-1, 3, -0.5));

        result = world.calculateAllCollisions(World::COLLISIONFLAG_NONE).getNarrowPhaseCollisions();

        collision = result.front();
        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 3.0, 0.0), collision.collisionPoint);
        connection = Vector3(-1.0, 0.0, -0.5); //sphere to collision connection
        CPPUNIT_ASSERT_DOUBLES_EQUAL(2 - connection.length(), collision.penetrationDepth, epsilon);
        connection.normalize();
        CPPUNIT_ASSERT_EQUAL(connection, collision.normal);

        //with edge12 (same plane)
        sphereProxy->setPosition(Vector3(3.0, 3.0, 0.0));

        result = world.calculateAllCollisions(World::COLLISIONFLAG_NONE).getNarrowPhaseCollisions();

        collision = result.front();
        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        CPPUNIT_ASSERT_EQUAL(Vector3(2.5, 2.5, 0.0), collision.collisionPoint);
        connection = Vector3(0.5, 0.5, 0.0); //sphere to collision connection
        CPPUNIT_ASSERT_DOUBLES_EQUAL(2 - connection.length(), collision.penetrationDepth, epsilon);
        connection.normalize();
        CPPUNIT_ASSERT_EQUAL(connection, collision.normal);


        //fourth test: collision with vertices
        //with (0,0,0)
        spherePosition.set(-1,-1, 1);
        sphereProxy->setPosition(spherePosition);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        collision = result.front();
        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 0.0), collision.collisionPoint);
        connection.set(-1.0, -1.0, 1); //sphere to collision connection
        CPPUNIT_ASSERT_DOUBLES_EQUAL(2 - connection.length(), collision.penetrationDepth, epsilon);
        connection.normalize();
        CPPUNIT_ASSERT_EQUAL(connection, collision.normal);

        //with vertex 5,0,0
        spherePosition.set(6,0, 1);
        sphereProxy->setPosition(spherePosition);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        collision = result.front();
        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        CPPUNIT_ASSERT_EQUAL(Vector3(5.0, 0.0, 0.0), collision.collisionPoint);
        connection.set(1.0, 0.0, 1.0); //collpoint to spherecenter connection
        CPPUNIT_ASSERT_DOUBLES_EQUAL(2 - connection.length(), collision.penetrationDepth, epsilon);
        connection.normalize();
        CPPUNIT_ASSERT_EQUAL(connection, collision.normal);

        //with vertex 0,5,0
        spherePosition.set(1, 6, 1);
        sphereProxy->setPosition(spherePosition);

        result = world.calculateAllCollisions().getNarrowPhaseCollisions();

        collision = result.front();
        CPPUNIT_ASSERT_EQUAL((size_t)1, result.size());
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 5.0, 0.0), collision.collisionPoint);
        connection.set(1.0, 1.0, 1.0); //collpoint to spherecenter connection
        CPPUNIT_ASSERT_DOUBLES_EQUAL(2 - connection.length(), collision.penetrationDepth, epsilon);
        connection.normalize();
        CPPUNIT_ASSERT_EQUAL(connection, collision.normal);
    }


    void NarrowPhaseTest::boxBoxTest() {        
#define DUMP_VERTICES(box) { \
    cout << #box << " = [" << endl; \
    Box* shape = (Box*) box->getShape(); \
    const Vector3* verts = shape->getVertices(box->getWorldTransformation()); \
    for (int i=0; i<8; i++) { \
        cout << "  " << fixed << verts[i] << "," << endl; \
    } \
    cout << "]" << endl << endl; \
}
        
#define DUMP_RESULTS(results) { \
    int i = 0; \
    for (it = results.begin(); it != results.end(); ++it) { \
        CollisionInfo info = *it; \
        cout << "CP" << i << " = " << info.collisionPoint << ", depth = " \
            << info.penetrationDepth << endl; \
        i++; \
    } \
}
        World world(Vector3(1000, 1000, 1000));

        Proxy* box1;
        Proxy* box2;
        Proxy* box3;
        
        const Matrix identity;

        list<CollisionInfo> results;
        list<CollisionInfo>::iterator it;


        /* --- Test 1: no collision , 2 fixed boxes ---
         *
         * Two boxes, each one 1x1x1 each, first one at (5, 0, 0) the second
         * one is placed fixed at the origin. No collision.
         */

        box1 = world.createProxy(new Box(Vector3(1, 1, 1)));
        box2 = world.createProxy(new Box(Vector3(1, 1, 1)));

        world.addProxy(box1);
        world.addProxy(box2);

        world.prepareSimulation();

        box1->translate(5, 0, 0);

        results = world.calculateAllCollisions().getNarrowPhaseCollisions();
        CPPUNIT_ASSERT(results.empty());


        /* --- Test 2: basic collision ---
         *
         * Two 1x1x1-boxes, first one, moving from (5, 0, 0) to (0.5, 0.5, 0.0)
         * into the other one, which is fixed at the origin. Should give us 4
         * collision points.
         */
        world.startNextStep();

        box1->setTransformation(identity);
        box2->setTransformation(identity);
        box1->translate(5, 0, 0);

        results = world.calculateAllCollisions().getNarrowPhaseCollisions();
        CPPUNIT_ASSERT(results.empty());

        world.startNextStep();

        // move to (0.5, 0.5, 0.0)
        box1->translate(-4.5, 0.5, 0.0);

        results = world.calculateAllCollisions().getNarrowPhaseCollisions();
        CPPUNIT_ASSERT_EQUAL((size_t) 4, results.size());
        
        // Collision points should be (0.5, 0.5, 0), (0.5, 1.5, 0),
        // (0.5, 0.5, 1) and (0.5, 1.5, 1), with a penetration depth of 0.5.
        {
            int a = 0; // (0.5, 0.5, 0)
            int b = 0; // (0.5, 1.5, 0)
            int c = 0; // (0.5, 0.5, 1)
            int d = 0; // (0.5, 1.5, 1)

            for (it = results.begin(); it != results.end(); ++it) {
                CollisionInfo info = *it;
                CPPUNIT_ASSERT_DOUBLES_EQUAL(info.penetrationDepth, (real)0.5, 0.1e-5);

                // epsilon is 0.001 by default (see Vector3 class)
                if (info.collisionPoint.isEqual(Vector3(0.5, 0.5, 0))) {
                    a++;
                }

                if (info.collisionPoint.isEqual(Vector3(0.5, 1.5, 0))) {
                    b++;
                }

                if (info.collisionPoint.isEqual(Vector3(0.5, 0.5, 1))) {
                    c++;
                }

                if (info.collisionPoint.isEqual(Vector3(0.5, 1.5, 1))) {
                    d++;
                }
            }

            // Assert each collision point is returned exactly once.
            CPPUNIT_ASSERT_EQUAL(1, a);
            CPPUNIT_ASSERT_EQUAL(1, b);
            CPPUNIT_ASSERT_EQUAL(1, c);
            CPPUNIT_ASSERT_EQUAL(1, d);
        }
        
        
        /* --- Test 3: a more complicated setup ---
         *
         * Starting with a collision, then box2 moves 0.5 upwards (y-axis),
         * still colliding with box1.
         */

        world.startNextStep();

        box1->setTransformation(identity);
        box2->setTransformation(identity);
        
        box2->translate(1.4, 0.0, 0.5);
        box2->rotate(45.0, 1, 0, 0);
        box2->rotate(45.0, 0, 0, 1);
        
        results = world.calculateAllCollisions().getNarrowPhaseCollisions();
        CPPUNIT_ASSERT_EQUAL((size_t) 2, results.size());

        // expected: (1.400000, 0.000000, 0.500000)
        //           and (0.692893, 0.500000, 1.000000)
        {
            int a = 0; // (1.400000, 0.000000, 0.500000)
            int b = 0; // (0.692893, 0.500000, 1.000000)
            
            for (it = results.begin(); it != results.end(); ++it) {
                CollisionInfo info = *it;
                CPPUNIT_ASSERT_DOUBLES_EQUAL(info.penetrationDepth, (real)0.177308, 0.1e-5);
                
                if (info.collisionPoint.isEqual(Vector3(1.4, 0, 0.5))) {
                    a++;
                }
                
                if (info.collisionPoint.isEqual(Vector3(0.692893, 0.50, 1.0))) {
                    b++;
                }
            }

            // Assert each collision point is returned exactly once.
            CPPUNIT_ASSERT_EQUAL(1, a);
            CPPUNIT_ASSERT_EQUAL(1, b);
        }
        
        world.startNextStep();
        box2->translate(0.0, 0.0, -0.5);
        
        results = world.calculateAllCollisions().getNarrowPhaseCollisions();        
        CPPUNIT_ASSERT_EQUAL((size_t) 2, results.size());

        // expected: (0.692893, 0.853553, 0.646447)
        //           and (0.692893, 0.146447, 1.353553)
        {
            int a = 0; // (0.692893, 0.853553, 0.646447)
            int b = 0; // (0.692893, 0.146447, 1.353553)
            
            for (it = results.begin(); it != results.end(); ++it) {
                CollisionInfo info = *it;
                CPPUNIT_ASSERT_DOUBLES_EQUAL(info.penetrationDepth, (real)0.307107, 0.1e-5);
                
                if (info.collisionPoint.isEqual(Vector3(0.692893, 0.853553, 0.646447))) {
                    a++;
                }
                
                if (info.collisionPoint.isEqual(Vector3(0.692893, 0.146447, 1.353553))) {
                    b++;
                }
            }
            
            // Assert each collision point is returned exactly once.
            CPPUNIT_ASSERT_EQUAL(1, a);
            CPPUNIT_ASSERT_EQUAL(1, b);
        }
        
        
        /* --- Test 4: Box on box ---
         *
         * Put a 1x1x1 box on top of a 2x1x1-box, should yield four collision
         * points in the 'y=1'-plane.
         */

        world.startNextStep();
        world.removeProxy(box2);

        box3 = world.createProxy(new Box(Vector3(2, 1, 1)));

        box1->setTransformation(identity);
        box3->setTransformation(identity);
        world.addProxy(box3);

        box1->translate(0.0, (real) 1.0, 0.0);

        results = world.calculateAllCollisions().getNarrowPhaseCollisions();
        CPPUNIT_ASSERT_EQUAL(results.size(), (size_t) 4);

        for (it = results.begin(); it != results.end(); ++it) {
            CollisionInfo info = *it;
            Vector3 p = info.collisionPoint;
            CPPUNIT_ASSERT_DOUBLES_EQUAL((real) 1.0, p.getY(), 0.1e-5);
            CPPUNIT_ASSERT_DOUBLES_EQUAL((real) 0.0, info.penetrationDepth, 0.1e-5);
        }
        
        
#undef DUMP_RESULTS
#undef DUMP_VERTICES
    }

    void NarrowPhaseTest::boxSphereTest() {
        //---------Setup----------------//
        World world(Vector3(2000, 2000, 2000));
        Proxy* sphere = world.createProxy(new Sphere(2));
        Proxy* box = world.createProxy(new Box(10, 10, 10));
        //put box into a container to simplify translations with rotated box
        Proxy* boxContainer = world.createProxy();
        boxContainer->addChild(box);

        world.addProxy(sphere);
        world.addProxy(boxContainer);
        world.prepareSimulation();

        //Basic setup: Sphere of radius 2 around (0,0,0)
        //             Cube of size 10 moving around

        //--------Test 1: No Collision, but overlapping BVs-------//
        boxContainer->translate(0, 2, 2);

        std::list<CollisionInfo> npc1 = world.calculateAllCollisions().getNarrowPhaseCollisions();

        CPPUNIT_ASSERT_EQUAL(true, npc1.empty());

        //--------Test 2: unrotated full collision with box-face ABCD-----//
        boxContainer->translate(-1, -3, -1);
        WorldCollisions coll2 = world.calculateAllCollisions();
        std::list<CollisionInfo> npc2 = coll2.getNarrowPhaseCollisions();


        //we should now have a collision
        CPPUNIT_ASSERT_EQUAL((size_t)1, npc2.size());
        CollisionInfo ci2 = *(npc2.begin());
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 1), ci2.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 1), ci2.normal);
        CPPUNIT_ASSERT_EQUAL((real) 1.0, ci2.penetrationDepth);
        CPPUNIT_ASSERT_EQUAL(box, ci2.penetratingProxy);
        CPPUNIT_ASSERT_EQUAL(sphere, ci2.penetratedProxy);

        //--------Test 3: rotated full collision with box-edge AB-----//
        //move box in y-dir so that the edge AB holds the closest point
        boxContainer->translate(0, 1, 0);
        box->rotate(45, 1, 0, 0);
        WorldCollisions coll3 = world.calculateAllCollisions();
        std::list<CollisionInfo> npc3 = coll3.getNarrowPhaseCollisions();
        //This ensures that broadphase and rigid middlephase are working correctly

        CPPUNIT_ASSERT_EQUAL((size_t)1,
                (size_t)coll3.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL((size_t)1, (size_t)coll3.getRigidBoundingVolumeCollisions().size());

        //Collision should be the same
        CPPUNIT_ASSERT_EQUAL((size_t)1, npc3.size());
        CollisionInfo ci3 = *(npc3.begin());
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 1), ci3.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 1), ci3.normal);
        CPPUNIT_ASSERT_EQUAL((real) 1.0, ci3.penetrationDepth);
        CPPUNIT_ASSERT_EQUAL(box, ci3.penetratingProxy);
        CPPUNIT_ASSERT_EQUAL(sphere, ci3.penetratedProxy);

        //--------Test 4: rotated full collision with vertex A-------//
        //move box in y-dir so that the vertex A is the closest point
        boxContainer->translate(1, 0, 0);
        box->rotate(45, 0, 0, 1); //i am not sure if this is correct- vertexA
        //should now be the single lowest point from the box
        WorldCollisions coll4 = world.calculateAllCollisions();
        std::list<CollisionInfo> npc4 = coll4.getNarrowPhaseCollisions();

        //Collision should be the same
        CPPUNIT_ASSERT_EQUAL((size_t)1, npc4.size());
        CollisionInfo ci4 = *(npc4.begin());
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 1), ci4.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 1), ci4.normal);
        CPPUNIT_ASSERT_EQUAL((real) 1.0, ci4.penetrationDepth);
        CPPUNIT_ASSERT_EQUAL(box, ci4.penetratingProxy);
        CPPUNIT_ASSERT_EQUAL(sphere, ci4.penetratedProxy);

        //--------Test 5: rotated touching-------//
        //move box up 1 unit in z-dir so that vertexA touches the sphere
        boxContainer->translate(0, 0, 1);
        //should now be the single lowest point from the box
        WorldCollisions coll5 = world.calculateAllCollisions();
        std::list<CollisionInfo> npc5 = coll5.getNarrowPhaseCollisions();

        //Collision should be the same
        CPPUNIT_ASSERT_EQUAL((size_t)1, npc5.size());
        CollisionInfo ci5 = *(npc5.begin());
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 2), ci5.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 1), ci5.normal);
        CPPUNIT_ASSERT_EQUAL((real) 0.0, ci5.penetrationDepth);
        CPPUNIT_ASSERT_EQUAL(box, ci5.penetratingProxy);
        CPPUNIT_ASSERT_EQUAL(sphere, ci5.penetratedProxy);

        //--------Test 6: box hits center of sphere-------//
        boxContainer->translate(0, 0, -2);
        //should now be the single lowest point from the box
        WorldCollisions coll6 = world.calculateAllCollisions();
        std::list<CollisionInfo> npc6 = coll6.getNarrowPhaseCollisions();

        //Collision should be the same
        CPPUNIT_ASSERT_EQUAL((size_t)1, npc6.size());
        CollisionInfo ci6 = *(npc6.begin());
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 0), ci6.collisionPoint);
        //We should have an arbitrary normal vector of lenght 1
        CPPUNIT_ASSERT_DOUBLES_EQUAL(1 , ci6.normal.length(), 0.01);
        CPPUNIT_ASSERT_EQUAL((real) 2.0, ci6.penetrationDepth);
        CPPUNIT_ASSERT_EQUAL(box, ci6.penetratingProxy);
        CPPUNIT_ASSERT_EQUAL(sphere, ci6.penetratedProxy);

        //-------Test 7: unrotated collision with vertexE---//
        //reset the box-orientation, move it down so that vertexG is at (0,0,0)
        //now we will move the sphere around the faces, edges and vertices
        Matrix m;
        box->setTransformation(m); //unrotated
        boxContainer->setTransformation(m);
        boxContainer->translate(-10, -10, -10);
        world.calculateAllCollisions(); //make sure the box has MOVEFLAG_UNMOVED

        sphere->translate(1, 1, 1);
        WorldCollisions coll7 = world.calculateAllCollisions();
        std::list<CollisionInfo> npc7 = coll7.getNarrowPhaseCollisions();

        CPPUNIT_ASSERT_EQUAL((size_t)1, npc7.size());
        CollisionInfo ci7 = *(npc7.begin());
        //we have moved the sphere - it should be the penetrating proxy
        CPPUNIT_ASSERT_EQUAL(sphere, ci7.penetratingProxy);
        CPPUNIT_ASSERT_EQUAL(box, ci7.penetratedProxy);
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 0), ci7.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(1, 1, 1).normalize(), ci7.normal);
        //high tolerance, we are calculating with babylonian length
        CPPUNIT_ASSERT_DOUBLES_EQUAL((real) 2.0 - Vector3(1,1,1).length(), ci7.penetrationDepth, 0.1);

        //---------Test 8: Collision with Face CDHG-----------//
        sphere->translate(-3, 0, -3);
        WorldCollisions coll8 = world.calculateAllCollisions();
        std::list<CollisionInfo> npc8 = coll8.getNarrowPhaseCollisions();

        CPPUNIT_ASSERT_EQUAL((size_t)1, npc8.size());
        CollisionInfo ci8 = *(npc8.begin());
        CPPUNIT_ASSERT_EQUAL(sphere, ci8.penetratingProxy);
        CPPUNIT_ASSERT_EQUAL(box, ci8.penetratedProxy);
        CPPUNIT_ASSERT_EQUAL(Vector3(-2, 0, -2), ci8.collisionPoint);
        CPPUNIT_ASSERT_EQUAL(Vector3(0, 1, 0), ci8.normal);
        //high tolerance, we are calculating with babylonian length
        CPPUNIT_ASSERT_EQUAL((real) 1.0, ci8.penetrationDepth);

    }

}
/*
 * vim: et sw=4 ts=4
 */
