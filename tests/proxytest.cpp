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

#include "proxytest.h"
#include "math/matrix.h"
#include <proxy.h>
#include <world.h>
#include <shapes/shapes.h>
#include <debugstream.h>

#include <math.h>

using namespace std;
using namespace dcollide;

CPPUNIT_TEST_SUITE_REGISTRATION ( ProxyTest );

const float epsilon = 0.0001f;


namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
    void ProxyTest::setUp(void) {
    }

    /*!
     *  \brief resource deallocation for unit test
     */
    void ProxyTest::tearDown(void) {
    }

    void ProxyTest::testProxyCreation() {
        World world(Vector3(2000, 2000, 2000));
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.getTopLevelProxies().size());

        testProxyCreationInternal(&world);

        world.prepareSimulation();

        // repeat the same tests again, but this time _after_
        // prepareSimulation() was called
        testProxyCreationInternal(&world);
    }

    void ProxyTest::testProxyCreationInternal(World* world) {
        size_t initialProxies = world->getTopLevelProxies().size();

        // create a simple proxy
        Box* box1 = new Box(Vector3(10.0, 20.0, 30.0));
        Proxy* proxy1 = world->createProxy(box1);
        world->addProxy(proxy1);
        CPPUNIT_ASSERT_EQUAL(initialProxies + 1, world->getTopLevelProxies().size());

        // another simple proxy
        Box* box2 = new Box(Vector3(10.0, 20.0, 30.0));
        Proxy* proxy2 = world->createProxy(box2);
        CPPUNIT_ASSERT_EQUAL(initialProxies + 1, world->getTopLevelProxies().size());
        world->addProxy(proxy2);
        CPPUNIT_ASSERT_EQUAL(initialProxies + 2, world->getTopLevelProxies().size());

        // create an empty proxy and add it to the world before it has any
        // children
        Proxy* parentOfProxy3 = world->createProxy();

        // just tests if transformations before adding the proxy to the world work.
        // -> no cppunit tests here, just check if it crashes
        // TODO: cppunit tests for correct results (in particular once added to
        // the world)
        parentOfProxy3->translate(10.0, 0.0, 0.0);
        parentOfProxy3->rotate(10.0, 1.0, 0.0, 0.0);
        parentOfProxy3->setTransformation(Matrix());

        CPPUNIT_ASSERT_EQUAL(initialProxies + 2, world->getTopLevelProxies().size());
        world->addProxy(parentOfProxy3);
        CPPUNIT_ASSERT_EQUAL(initialProxies + 3, world->getTopLevelProxies().size());

        // add a child to the proxy
        Box* box3 = new Box(Vector3(10.0, 20.0, 30.0));
        Proxy* proxy3 = world->createProxy(box3);
        CPPUNIT_ASSERT_EQUAL((size_t)0, parentOfProxy3->getChildProxies().size());
        parentOfProxy3->addChild(proxy3);
        CPPUNIT_ASSERT_EQUAL((size_t)1, parentOfProxy3->getChildProxies().size());
        CPPUNIT_ASSERT_EQUAL(initialProxies + 3, world->getTopLevelProxies().size());

    }

    void ProxyTest::testProxyRemoving() {
        // TODO

        // in particular TODO:
        // * remove proxy from world
        // * remove proxy from proxy
        // * maybe also test if collision detection still works
    }

    void ProxyTest::testMeshInitAdjacency() {
        //construct a simple mesh manually, and test adjacencies afterwards
        //Mesh-Tetraeder
/*
        3 \
       / \ \
      /  \  \
     /  --\---2
    0----  \ /
       -----1
*/

        Vertex* v0 = new Vertex(Vector3(0, 0, 0));
        Vertex* v1 = new Vertex(Vector3(10, 0, 0));
        Vertex* v2 = new Vertex(Vector3(10, 10, 0));
        Vertex* v3 = new Vertex(Vector3(10, 10, 20));

        Triangle* t0 = new Triangle(v0, v1, v2);
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

        Mesh mesh(vertices, triangles);

        CPPUNIT_ASSERT_EQUAL((unsigned int)4, mesh.getTriangleCount());
        CPPUNIT_ASSERT_EQUAL((unsigned int)4, mesh.getVertexCount());

        //now test adjacency lists of all vertices
        //first run: triangles
        //Test Vertex 0
        CPPUNIT_ASSERT_EQUAL((size_t)3, v0->getAdjacentTriangles().size());
        std::list<Triangle*>::const_iterator tIter = v0->getAdjacentTriangles().begin();
        CPPUNIT_ASSERT(*tIter == t0);
        tIter++;
        CPPUNIT_ASSERT(*tIter == t1);
        tIter++;
        CPPUNIT_ASSERT(*tIter == t3);

        //Test Vertex 1
        CPPUNIT_ASSERT_EQUAL((size_t)3, v1->getAdjacentTriangles().size());
        tIter = v1->getAdjacentTriangles().begin();
        CPPUNIT_ASSERT(*tIter == t0);
        tIter++;
        CPPUNIT_ASSERT(*tIter == t1);
        tIter++;
        CPPUNIT_ASSERT(*tIter == t2);

        //Test Vertex 2
        CPPUNIT_ASSERT_EQUAL((size_t)3, v2->getAdjacentTriangles().size());
        tIter = v2->getAdjacentTriangles().begin();
        CPPUNIT_ASSERT(*tIter == t0);
        tIter++;
        CPPUNIT_ASSERT(*tIter == t2);
        tIter++;
        CPPUNIT_ASSERT(*tIter == t3);


        //Test Vertex 2
        CPPUNIT_ASSERT_EQUAL((size_t)3, v3->getAdjacentTriangles().size());
        tIter = v3->getAdjacentTriangles().begin();
        CPPUNIT_ASSERT(*tIter == t1);
        tIter++;
        CPPUNIT_ASSERT(*tIter == t2);
        tIter++;
        CPPUNIT_ASSERT(*tIter == t3);

        //Test vertex neighborship
        CPPUNIT_ASSERT_EQUAL((size_t)3, v0->getAdjacentVertices().size());
        std::list<Vertex*>::const_iterator vIter = v0->getAdjacentVertices().begin();
        CPPUNIT_ASSERT(*vIter == v1);
        vIter++;
        CPPUNIT_ASSERT(*vIter == v2);
        vIter++;
        CPPUNIT_ASSERT(*vIter == v3);

        CPPUNIT_ASSERT_EQUAL((size_t)3, v1->getAdjacentVertices().size());
        vIter = v1->getAdjacentVertices().begin();
        CPPUNIT_ASSERT(*vIter == v2);
        vIter++;
        CPPUNIT_ASSERT(*vIter == v0);
        vIter++;
        CPPUNIT_ASSERT(*vIter == v3);

        CPPUNIT_ASSERT_EQUAL((size_t)3, v2->getAdjacentVertices().size());
        vIter = v2->getAdjacentVertices().begin();
        CPPUNIT_ASSERT(*vIter == v0);
        vIter++;
        CPPUNIT_ASSERT(*vIter == v1);
        vIter++;
        CPPUNIT_ASSERT(*vIter == v3);

        CPPUNIT_ASSERT_EQUAL((size_t)3, v3->getAdjacentVertices().size());
        vIter = v3->getAdjacentVertices().begin();
        CPPUNIT_ASSERT(*vIter == v0);
        vIter++;
        CPPUNIT_ASSERT(*vIter == v1);
        vIter++;
        CPPUNIT_ASSERT(*vIter == v2);


    }

    // Testing changing the Type of the Proxy
    void ProxyTest::testProxySetType() {
        World world(Vector3(2000, 2000, 2000));

        // Creating simple Proxy, should be of Type PROXYTYPE_RIGID (1)
        Box* box1 = new Box(Vector3(10.0, 20.0, 30.0));
        Proxy* proxy1 = world.createProxy(box1);
        world.addProxy(proxy1);
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.getTopLevelProxies().size());
        // Proxy should have flags:
        // * rigid (should be the default)
        // * convex (since we don't create a Mesh)
        // * closedhull (same reason as for convex)
        // * no other flags
        CPPUNIT_ASSERT_EQUAL((int)(PROXYTYPE_RIGID | PROXYTYPE_CONVEX | PROXYTYPE_CLOSEDHULL),
                proxy1->getProxyType());

        // Now changing to PROXYTYPE_DEFORMABLE:
        proxy1->setProxyType(PROXYTYPE_DEFORMABLE);
        // Checking Type, proxy still should be in the world:
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.getTopLevelProxies().size());
        CPPUNIT_ASSERT_EQUAL((int)PROXYTYPE_DEFORMABLE, proxy1->getProxyType());

        // Creating Parent-Proxy:
        Proxy* parentProxy = world.createProxy();
        world.addProxy(parentProxy);

        // Creating Child:
        Box* box2 = new Box(Vector3(10.0, 20.0, 40.0));
        Proxy* proxy2 = world.createProxy(box2);
        parentProxy->addChild(proxy2);
        CPPUNIT_ASSERT_EQUAL((size_t)2, world.getTopLevelProxies().size());
        // Trying to change type of child, should not work:
        CPPUNIT_ASSERT_EQUAL((int)(PROXYTYPE_RIGID | PROXYTYPE_CONVEX | PROXYTYPE_CLOSEDHULL),
                proxy2->getProxyType());
        proxy2->setProxyType((int)PROXYTYPE_DEFORMABLE);
        CPPUNIT_ASSERT_EQUAL((int)(PROXYTYPE_RIGID | PROXYTYPE_CONVEX | PROXYTYPE_CLOSEDHULL),
                proxy2->getProxyType());
        // Trying to change type of parent, should work:
        CPPUNIT_ASSERT_EQUAL((int)PROXYTYPE_RIGID, parentProxy->getProxyType());
        parentProxy->setProxyType(PROXYTYPE_DEFORMABLE);
        CPPUNIT_ASSERT_EQUAL((int)PROXYTYPE_DEFORMABLE, parentProxy->getProxyType());
        CPPUNIT_ASSERT_EQUAL((int)PROXYTYPE_DEFORMABLE, proxy2->getProxyType());

        // Checking if we still can remove Proxies from the world:
        world.removeProxy(proxy1);
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.getTopLevelProxies().size());
        world.removeProxy(parentProxy);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.getTopLevelProxies().size());
        delete proxy1;
        delete parentProxy;
    }

    void ProxyTest::testProxyHierarchy() {
        World world(Vector3(2000, 2000, 2000));

        // simple proxy creation, adding, removing
        Proxy* topProxy = world.createProxy();
        CPPUNIT_ASSERT_EQUAL(false, topProxy->isInHierarchy());
        world.addProxy(topProxy);
        CPPUNIT_ASSERT_EQUAL(true, topProxy->isInHierarchy());
        world.removeProxy(topProxy);
        CPPUNIT_ASSERT_EQUAL(false, topProxy->isInHierarchy());

        // a proxy is "in hierarchy" if and only if its toplevel proxy is in the
        // World.
        Proxy* child1 = world.createProxy();
        Proxy* child2 = world.createProxy();
        Proxy* grandChild = world.createProxy();
        topProxy->addChild(child1);
        CPPUNIT_ASSERT_EQUAL(false, topProxy->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, child1->isInHierarchy());
        topProxy->addChild(child2);
        child1->addChild(grandChild);
        CPPUNIT_ASSERT_EQUAL(false, topProxy->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, child1->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, child2->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, grandChild->isInHierarchy());

        // adding the whole proxy hierarchy to the world hierarchy
        world.addProxy(topProxy);
        CPPUNIT_ASSERT_EQUAL(true, topProxy->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(true, child1->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(true, child2->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(true, grandChild->isInHierarchy());

        // removing the whole proxy hierarchy from the world hierarchy
        world.removeProxy(topProxy);
        CPPUNIT_ASSERT_EQUAL(false, topProxy->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, child1->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, child2->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, grandChild->isInHierarchy());

        // removing part of the proxy hierarchy from the world-hierarchy
        world.addProxy(topProxy);
        topProxy->removeChild(child1);
        CPPUNIT_ASSERT_EQUAL(true, topProxy->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, child1->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(true, child2->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(false, grandChild->isInHierarchy());

        // adding a proxy hierarchy to an existing world-hierarchy
        topProxy->addChild(child1);
        CPPUNIT_ASSERT_EQUAL(true, topProxy->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(true, child1->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(true, child2->isInHierarchy());
        CPPUNIT_ASSERT_EQUAL(true, grandChild->isInHierarchy());
    }
}
/*
 * vim: et sw=4 ts=4
 */
