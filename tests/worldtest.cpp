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

#include "worldtest.h"

#include <proxy.h>
#include <world.h>
#include <shapes/shapes.h>
#include <worldcollisions.h>
#include <broadphase/broadphasecollisions.h>
#include <narrowphase/narrowphase.h>
#include <bvhnode.h>

#include <math.h>
#include <algorithm>

using namespace std;

CPPUNIT_TEST_SUITE_REGISTRATION ( WorldTest );

const float epsilon = 0.0001f;

// AB: Aabb::adjustToCylinder/Wedge() are not yet implemented.
#define USE_CYLINDER_WEDGE_CONE 0


namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
    void WorldTest::setUp(void) {
    }

    /*!
     *  \brief resource deallocation for unit test
     */
    void WorldTest::tearDown(void) {
    }

    void WorldTest::simulationTestUncached() {
        World* world = new World(Vector3(200000, 200000, 200000));
        world->setUseCollisionCaching(false);
        Vector3 boxDimensions(10, 10, 10);
        Proxy* box1 = world->createProxy(new Box(boxDimensions));
        //Proxy* childBox = world->createProxy(new Box(boxDimensions));
        //childBox->translate(5, 5, 5);
        //box->addChild(childBox);

        world->addProxy(box1);

        Proxy* box2 = world->createProxy(new Box(boxDimensions));
        box2->translate(0, -20, 0);
        world->addProxy(box2);

        Proxy* box3 = world->createProxy(new Box(boxDimensions)); //always colliding with box2
        box3->translate(5, -25, 0);
        world->addProxy(box3);

        
        Proxy* moveBox = world->createProxy(new Box(Vector3(10, 50, 10)));
        moveBox->translate(20, -25, 0); //starting in no-collision state
        world->addProxy(moveBox);

        //WorldCollisions coll;
        
        world->prepareSimulation();
//std::cout << "-----------Starting simulation test-----------"<<std::endl;
//std::cout << "doing pre-move collisionCheck"<<std::endl;
        //test and ensure 1 BPC and rigid C
        WorldCollisions coll0 = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(1,
                (int)coll0.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(1, (int)coll0.getRigidBoundingVolumeCollisions().size());

        //Step 1 -> move moveBox to collidy with box3
//std::cout <<std::endl<< "doing step 1 - move in to collide with box3"<<std::endl;
        moveBox->translate(-6, 0, 0);

        //ensure 2 BPC/RC
        WorldCollisions coll1  = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(2,
                (int)coll1.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(2, (int)coll1.getRigidBoundingVolumeCollisions().size());

        //Step 2-> move to collide with all boxes
//std::cout <<std::endl<<"doing step 2 - move further to collide with all"<<std::endl;
        moveBox->translate(-5, 0, 0);
        //ensure 4 collisions
        WorldCollisions coll2  = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(4,
                (int)coll2.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(4, (int)coll2.getRigidBoundingVolumeCollisions().size());

        //Step 3-> move out again
//std::cout << std::endl<<"doing step 3 - moving out"<<std::endl;
        moveBox->translate(11, 0, 0);
        //ensure 1 collisions
        WorldCollisions coll3  = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(1,
                (int)coll3.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(1, (int)coll3.getRigidBoundingVolumeCollisions().size());

        //Step 4-> move in again
//std::cout << std::endl<<"doing step 4 - moving back in"<<std::endl;
        moveBox->translate(-11, 0, 0);
        //ensure 4 collisions
        WorldCollisions coll4  = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(4,
                (int)coll4.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(4, (int)coll4.getRigidBoundingVolumeCollisions().size());

        delete world;
    }

    void WorldTest::simulationTestCached() {
        World* world = new World(Vector3(200000, 200000, 200000));
        world->setUseCollisionCaching(true);
        Vector3 boxDimensions(10, 10, 10);
        Proxy* box1 = world->createProxy(new Box(boxDimensions));
        //Proxy* childBox = world->createProxy(new Box(boxDimensions));
        //childBox->translate(5, 5, 5);
        //box->addChild(childBox);

        world->addProxy(box1);

        Proxy* box2 = world->createProxy(new Box(boxDimensions));
        box2->translate(0, -20, 0);
        world->addProxy(box2);

        Proxy* box3 = world->createProxy(new Box(boxDimensions)); //always colliding with box2
        box3->translate(5, -25, 0);
        world->addProxy(box3);

        
        Proxy* moveBox = world->createProxy(new Box(Vector3(10, 50, 10)));
        moveBox->translate(20, -25, 0); //starting in no-collision state
        world->addProxy(moveBox);

        //WorldCollisions coll;
        
        world->prepareSimulation();
//std::cout << "-----------Starting simulation test-----------"<<std::endl;
//std::cout << "doing pre-move collisionCheck"<<std::endl;
        //test and ensure 1 BPC and rigid C
        WorldCollisions coll0 = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(1,
                (int)coll0.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(1, (int)coll0.getRigidBoundingVolumeCollisions().size());

        //Step 1 -> move moveBox to collidy with box3
//std::cout <<std::endl<< "doing step 1 - move in to collide with box3"<<std::endl;
        moveBox->translate(-6, 0, 0);

        //ensure 2 BPC/RC
        WorldCollisions coll1  = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(2,
                (int)coll1.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(2, (int)coll1.getRigidBoundingVolumeCollisions().size());

        //Step 2-> move to collide with all boxes
//std::cout <<std::endl<<"doing step 2 - move further to collide with all"<<std::endl;
        moveBox->translate(-5, 0, 0);
        //ensure 4 collisions
        WorldCollisions coll2  = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(4,
                (int)coll2.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(4, (int)coll2.getRigidBoundingVolumeCollisions().size());

        //Step 3-> move out again
//std::cout << std::endl<<"doing step 3 - moving out"<<std::endl;
        moveBox->translate(11, 0, 0);
        //ensure 1 collisions
        WorldCollisions coll3  = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(1,
                (int)coll3.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(1, (int)coll3.getRigidBoundingVolumeCollisions().size());

        //Step 4-> move in again
//std::cout << std::endl<<"doing step 4 - moving back in"<<std::endl;
        moveBox->translate(-11, 0, 0);
        //ensure 4 collisions
        WorldCollisions coll4  = world->calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL(4,
                (int)coll4.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT_EQUAL(4, (int)coll4.getRigidBoundingVolumeCollisions().size());

        delete world;
    }


    static Mesh* createSimpleMesh() {
        // simple mesh with 2 triangles
        std::vector<Vertex*> vertices;
        std::vector<int> indices;
        vertices.push_back(new Vertex(0.0, 0.0, 0.0));
        vertices.push_back(new Vertex(100.0, 0.0, 0.0));
        vertices.push_back(new Vertex(100.0, 100.0, 0.0));
        vertices.push_back(new Vertex(0.0, 100.0, 0.0));
        indices.push_back(0);
        indices.push_back(1);
        indices.push_back(2);
        indices.push_back(1);
        indices.push_back(2);
        indices.push_back(3);
        Mesh* mesh = new Mesh(vertices, indices);
        return mesh;
    }
    void WorldTest::testRigidAndDeformableProxies() {
        // create and add both, rigid and deformable proxies to the world and
        // make sure collision detection knows about all of them
        //
        // they are all colliding with each other, so we get n^2 pairs.
        // simply checking the broadphase results is sufficient, since we only
        // want to know if the World object knows about all proxies, not whether
        // the collision detection algorithms work correctly (that's the job of
        // other tests)

        std::list<Proxy*> proxies;
        World world(Vector3(1024.0, 1024.0, 1024.0));

        // simple rigid proxies
        proxies.push_back(world.createProxy(new Box(100.0, 100.0, 100.0)));
        proxies.push_back(world.createProxy(new Sphere(100.0)));
#if USE_CYLINDER_WEDGE_CONE
        proxies.push_back(world.createProxy(new Cone(100.0, 100.0)));
        proxies.push_back(world.createProxy(new Wedge(100.0, 100.0, 100.0)));
        proxies.push_back(world.createProxy(new Cylinder(100.0, 100.0)));
#endif

        // simple rigid && fixed proxies
        proxies.push_back(world.createProxy(new Box(100.0, 100.0, 100.0), PROXYTYPE_FIXED));
        proxies.push_back(world.createProxy(new Sphere(100.0), PROXYTYPE_FIXED));
#if USE_CYLINDER_WEDGE_CONE
        proxies.push_back(world.createProxy(new Cone(100.0, 100.0), PROXYTYPE_FIXED));
        proxies.push_back(world.createProxy(new Wedge(100.0, 100.0, 100.0), PROXYTYPE_FIXED));
        proxies.push_back(world.createProxy(new Cylinder(100.0, 100.0), PROXYTYPE_FIXED));
#endif

        // simple rigid && self collidable proxies
        // this is actually nonsense, as there is a single shape in the proxy
        // only, i.e. no self collision possible.
        // however for a test whether this proxy type still is collided with
        // other objects properly this is sufficient.
        proxies.push_back(world.createProxy(new Box(100.0, 100.0, 100.0), (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE)));
        proxies.push_back(world.createProxy(new Sphere(100.0), (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE)));
#if USE_CYLINDER_WEDGE_CONE
        proxies.push_back(world.createProxy(new Cone(100.0, 100.0), (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE)));
        proxies.push_back(world.createProxy(new Wedge(100.0, 100.0, 100.0), (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE)));
        proxies.push_back(world.createProxy(new Cylinder(100.0, 100.0), (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE)));
#endif

        // meshes, rigid and deformable.
        proxies.push_back(world.createProxy(createSimpleMesh(), PROXYTYPE_RIGID));
        proxies.push_back(world.createProxy(createSimpleMesh(), (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE)));
        proxies.push_back(world.createProxy(createSimpleMesh(), PROXYTYPE_FIXED));

        proxies.push_back(world.createProxy(createSimpleMesh(), PROXYTYPE_DEFORMABLE));
        proxies.push_back(world.createProxy(createSimpleMesh(), (ProxyTypes) (PROXYTYPE_DEFORMABLE | PROXYTYPE_SELFCOLLIDABLE)));

        for (std::list<Proxy*>::iterator it = proxies.begin(); it != proxies.end(); ++it) {
            world.addProxy(*it);
        }

        world.prepareSimulation();

        WorldCollisions collisions = world.calculateAllCollisions(World::COLLISIONFLAG_SKIP_MIDDLE_PHASE);
        const BroadPhaseCollisions* broad = collisions.getBroadPhaseCollisions();
        for (std::list<Proxy*>::iterator it1 = proxies.begin(); it1 != proxies.end(); ++it1) {
            std::list<Proxy*>::iterator it2 = it1;
            ++it2;
            for (; it2 != proxies.end(); ++it2) {
                if (((*it1)->getProxyType() & PROXYTYPE_FIXED) &&
                    ((*it2)->getProxyType() & PROXYTYPE_FIXED)) {
                    // 2 fixed proxies dont collide with each other
                    // (at least they dont have to - World is allowed to return
                    // them anyway)
                    continue;
                }
                CollisionPair pair;
                CPPUNIT_ASSERT((*it1)->getBvHierarchyNode() != 0);
                CPPUNIT_ASSERT((*it1)->getBvHierarchyNode()->getBoundingVolume() != 0);
                CPPUNIT_ASSERT((*it2)->getBvHierarchyNode() != 0);
                CPPUNIT_ASSERT((*it2)->getBvHierarchyNode()->getBoundingVolume() != 0);
                pair.bvol1 = (*it1)->getBvHierarchyNode()->getBoundingVolume();
                pair.bvol2 = (*it2)->getBvHierarchyNode()->getBoundingVolume();

                CPPUNIT_ASSERT(std::count(broad->getResults().begin(), broad->getResults().end(), pair));
            }
        }
    }
}
/*
 * vim: et sw=4 ts=4
 */
