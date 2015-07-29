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

#include "middlephaserigidtest.h"

#include <proxy.h>
#include <world.h>
#include <shapes/shapes.h>
#include <detectorrigid/bvhtraverse.h>
#include <broadphase/broadphasecollisions.h>
#include <narrowphase/boundingvolumecollision.h>
#include <worldcollisions.h>
#include <bvhnode.h>
#include <debugstream.h>

#include <modelloader/loaderdummy.h>

#include <math.h>

CPPUNIT_TEST_SUITE_REGISTRATION(dcollide::MiddlePhaseRigidTest);

const float epsilon = 0.0001f;


static dcollide::Mesh* createMeshifiedBox(const dcollide::Vector3& boxDimensions) {
    std::vector<dcollide::Vertex*> vertices(8);
    std::vector<dcollide::Triangle*> triangles(6 * 2);
    vertices[0] = new dcollide::Vertex(dcollide::Vector3(0.0f, 0.0f, 0.0f));
    vertices[1] = new dcollide::Vertex(dcollide::Vector3(0.0f, 0.0f, boxDimensions.getZ()));
    vertices[2] = new dcollide::Vertex(dcollide::Vector3(0.0f, boxDimensions.getY(), 0.0f));
    vertices[3] = new dcollide::Vertex(dcollide::Vector3(0.0f, boxDimensions.getY(), boxDimensions.getZ()));
    vertices[4] = new dcollide::Vertex(dcollide::Vector3(boxDimensions.getX(), 0.0f, 0.0f));
    vertices[5] = new dcollide::Vertex(dcollide::Vector3(boxDimensions.getX(), 0.0f, boxDimensions.getZ()));
    vertices[6] = new dcollide::Vertex(dcollide::Vector3(boxDimensions.getX(), boxDimensions.getY(), 0.0f));
    vertices[7] = new dcollide::Vertex(dcollide::Vector3(boxDimensions.getX(), boxDimensions.getY(), boxDimensions.getZ()));
    triangles[0] = new dcollide::Triangle(vertices[2], vertices[1], vertices[0]);
    triangles[1] = new dcollide::Triangle(vertices[1], vertices[2], vertices[3]);
    triangles[2] = new dcollide::Triangle(vertices[4], vertices[5], vertices[6]);
    triangles[3] = new dcollide::Triangle(vertices[5], vertices[7], vertices[6]);
    triangles[4] = new dcollide::Triangle(vertices[0], vertices[1], vertices[4]);
    triangles[5] = new dcollide::Triangle(vertices[1], vertices[5], vertices[4]);
    triangles[6] = new dcollide::Triangle(vertices[2], vertices[6], vertices[3]);
    triangles[7] = new dcollide::Triangle(vertices[3], vertices[6], vertices[7]);
    triangles[8] = new dcollide::Triangle(vertices[0], vertices[4], vertices[2]);
    triangles[9] = new dcollide::Triangle(vertices[2], vertices[4], vertices[6]);
    triangles[10] = new dcollide::Triangle(vertices[1], vertices[3], vertices[5]);
    triangles[11] = new dcollide::Triangle(vertices[5], vertices[3], vertices[7]);
    dcollide::Mesh* mesh = new dcollide::Mesh(vertices, triangles);
    return mesh;
}


namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
    void MiddlePhaseRigidTest::setUp(void) {
        try {
            mWorld = new World(Vector3(2000, 2000, 2000));
        } catch (Exception e) {
            mWorld = 0;
            throw (e);
        }

        mSimpleBoxDimensions = Vector3(10.0f, 20.0f, 30.0f);
        mSimpleBox1 = mWorld->createProxy(new Box(mSimpleBoxDimensions));
        mSimpleBox2 = mWorld->createProxy(new Box(mSimpleBoxDimensions));
        mSimpleMeshifiedBox1 = mWorld->createProxy(createMeshifiedBox(mSimpleBoxDimensions));
        mSimpleMeshifiedBox2 = mWorld->createProxy(createMeshifiedBox(mSimpleBoxDimensions));

        CPPUNIT_ASSERT_EQUAL(false, mSimpleBox1->getBvHierarchyNode()->isLeaf());
        CPPUNIT_ASSERT_EQUAL(false, mSimpleBox2->getBvHierarchyNode()->isLeaf());
        CPPUNIT_ASSERT_EQUAL(true, mSimpleBox1->getBvHierarchyNode()->getChildren().front()->isLeaf());
        CPPUNIT_ASSERT_EQUAL(true, mSimpleBox2->getBvHierarchyNode()->getChildren().front()->isLeaf());

        mWorld->addProxy(mSimpleBox1);
        mWorld->addProxy(mSimpleBox2);
        mWorld->addProxy(mSimpleMeshifiedBox1);
        mWorld->addProxy(mSimpleMeshifiedBox2);
    }

    /*!
     *  \brief resource deallocation for unit test
     */
    void MiddlePhaseRigidTest::tearDown(void) {
        delete mWorld;
    }

    void MiddlePhaseRigidTest::testSimpleBoxBoxCollisions() {
        BvhTraverse rigid;

        CollisionPair input;
        std::list<BoundingVolumeCollision> collisions;
        BoundingVolumeCollision collision;

        mSimpleBox1->setTransformation(Matrix());
        mSimpleBox2->setTransformation(Matrix());

        input.bvol1 = mSimpleBox1->getBvHierarchyNode()->getBoundingVolume();
        input.bvol2 = mSimpleBox2->getBvHierarchyNode()->getBoundingVolume();

        collisions = rigid.getBoundingVolumeCollisions(input);
        CPPUNIT_ASSERT_EQUAL(1, (int)collisions.size());
        collision = collisions.front();

        // collision nodes must always be leafs
        CPPUNIT_ASSERT_EQUAL(true, collision.node1->isLeaf());
        CPPUNIT_ASSERT_EQUAL(true, collision.node2->isLeaf());

        // one node is the Shape node of proxy1, the other the shape node of
        // proxy2.
        // note: the shape node is (for simple non-mesh shapes) the (only) child
        // of the proxy
        if (collision.node1 != mSimpleBox1->getBvHierarchyNode()->getChildren().front()) {
            const BvhNode* tmp = collision.node1;
            collision.node1 = collision.node2;
            collision.node2 = tmp;
        }
        CPPUNIT_ASSERT_EQUAL(const_cast<const BvhNode*>(mSimpleBox1->getBvHierarchyNode()->getChildren().front()), collision.node1);
        CPPUNIT_ASSERT_EQUAL(const_cast<const BvhNode*>(mSimpleBox2->getBvHierarchyNode()->getChildren().front()), collision.node2);

        // must not collide anymore
        mSimpleBox2->translate(Vector3(20.0f, 0.0f, 0.0f));
        input.bvol1 = mSimpleBox1->getBvHierarchyNode()->getBoundingVolume();
        input.bvol2 = mSimpleBox2->getBvHierarchyNode()->getBoundingVolume();
        CPPUNIT_ASSERT_EQUAL(0, (int)rigid.getBoundingVolumeCollisions(input).size());
    }

    // AB: this is a rather complicated test, so we test exactly one collision
    // only!
    void MiddlePhaseRigidTest::testSimpleBoxMeshCollision() {
        BvhTraverse rigid;

        CollisionPair input;
        std::list<BoundingVolumeCollision> collisions;

        mSimpleBox1->setTransformation(Matrix());
        mSimpleBox2->setTransformation(Matrix());
        mSimpleMeshifiedBox1->setTransformation(Matrix());
        mSimpleMeshifiedBox2->setTransformation(Matrix());

        input.bvol1 = mSimpleBox1->getBvHierarchyNode()->getBoundingVolume();
        input.bvol2 = mSimpleMeshifiedBox1->getBvHierarchyNode()->getBoundingVolume();
        collisions = rigid.getBoundingVolumeCollisions(input);
        CPPUNIT_ASSERT(collisions.size() >= 1);
        for (std::list<BoundingVolumeCollision>::iterator it = collisions.begin(); it != collisions.end(); ++it) {
            BoundingVolumeCollision c = *it;
            CPPUNIT_ASSERT_EQUAL(true, c.node1->isLeaf());
            CPPUNIT_ASSERT_EQUAL(true, c.node2->isLeaf());
            CPPUNIT_ASSERT(c.node1 != c.node2);

            Proxy* p1 = c.node1->getProxy();
            Proxy* p2 = c.node2->getProxy();
            const BvhNode* n1 = c.node1;
            const BvhNode* n2 = c.node2;
            while (n1->getParent()) {
                n1 = n1->getParent();
                p1 = n1->getProxy();
            }
            while (n2->getParent()) {
                n2 = n2->getParent();
                p2 = n2->getProxy();
            }

            // toplevel BvhNodes must always belong to a Proxy
            // -> colliding nodes must always belong to different Proxies
            //
            // (since we are not testing self collisions here)
            CPPUNIT_ASSERT(p1 != 0);
            CPPUNIT_ASSERT(p2 != 0);
            CPPUNIT_ASSERT(p1 != p2);
        }

        // each collision pair must appear in the results at most once.
        for (std::list<BoundingVolumeCollision>::iterator it1 = collisions.begin(); it1 != collisions.end(); ++it1) {
            BoundingVolumeCollision c1 = *it1;
            std::list<BoundingVolumeCollision>::iterator it2 = it1;
            ++it2;
            for (; it2 != collisions.end(); ++it2) {
                BoundingVolumeCollision c2 = *it2;
                if (c1.node1 == c2.node1 || c1.node1 == c2.node2) {
                    if (c1.node2 == c2.node1 || c1.node2 == c2.node2) {
                        CPPUNIT_FAIL("Collision pair more than once in results list");
                    }
                }
            }
        }

        // both Boxes cover each other completely, so every leaf of Proxy 1
        // should collide with at least one leaf from Proxy 2 and the other way
        // around.
        // -> Proxy 1 is a Box shape, so it has exactly one leaf. Since we have
        //    correct collisions at this point
        // -> all we need to check for, is that every leaf of Proxy 2 appears at
        //    least once in the results
        std::list<const BvhNode*> leafsOfMesh;
        std::list<const BvhNode*> nodeList;
        nodeList.push_back(mSimpleMeshifiedBox1->getBvHierarchyNode());
        while (!nodeList.empty()) {
            const BvhNode* n = nodeList.front();
            nodeList.pop_front();

            if (n->isLeaf()) {
                leafsOfMesh.push_back(n);
            } else {
                const std::list<BvhNode*>& children = n->getChildren();
                for (std::list<BvhNode*>::const_iterator it = children.begin(); it != children.end(); ++it) {
                    nodeList.push_back(*it);
                }
            }
        }
        for (std::list<const BvhNode*>::iterator it = leafsOfMesh.begin(); it != leafsOfMesh.end(); ++it) {
            bool found = false;
            for (std::list<BoundingVolumeCollision>::iterator collIt = collisions.begin(); collIt != collisions.end() && !found; ++collIt) {
                if ((*collIt).node1 == *it || (*collIt).node2 == *it) {
                    found = true;
                }
            }
            if (!found) {
                CPPUNIT_FAIL("Not all BvhNode leafs of the mesh (in the box-mesh test) appear in the results!");
            }
        }
    }

    void MiddlePhaseRigidTest::testSimpleSelfCollisions() {
        World world(Vector3(5000, 5000, 5000));
        world.prepareSimulation();
        const Vector3 boxDimensions(10.0, 20.0, 30.0);

        Box* boxNoSelfCollision1 = new Box(boxDimensions);
        Box* boxNoSelfCollision2 = new Box(boxDimensions);
        Proxy* proxyNoSelfCollision = world.createProxy(PROXYTYPE_RIGID);
        Proxy* boxNoSelfCollision1Proxy = world.createProxy(boxNoSelfCollision1);
        Proxy* boxNoSelfCollision2Proxy = world.createProxy(boxNoSelfCollision2);
        proxyNoSelfCollision->addChild(boxNoSelfCollision1Proxy);
        proxyNoSelfCollision->addChild(boxNoSelfCollision2Proxy);
        world.addProxy(proxyNoSelfCollision);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // we won't use this anymore, delete it
        world.removeProxy(proxyNoSelfCollision);
        delete proxyNoSelfCollision;
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());


        Proxy* proxy = world.createProxy((ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        Box* box1 = new Box(boxDimensions);
        Box* box2 = new Box(boxDimensions);
        Proxy* box1Proxy = world.createProxy(box1, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        Proxy* box2Proxy = world.createProxy(box2, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        proxy->addChild(box1Proxy);
        proxy->addChild(box2Proxy);
        world.addProxy(proxy);

        {
            WorldCollisions collisions = world.calculateAllCollisions();
            CPPUNIT_ASSERT_EQUAL((size_t)1, collisions.getRigidBoundingVolumeCollisions().size());
            const BvhNode* node1 = collisions.getRigidBoundingVolumeCollisions().front().node1;
            const BvhNode* node2 = collisions.getRigidBoundingVolumeCollisions().front().node2;

            CPPUNIT_ASSERT(node1 != 0);
            CPPUNIT_ASSERT(node2 != 0);
            CPPUNIT_ASSERT_EQUAL(true, node1->isLeaf());
            CPPUNIT_ASSERT_EQUAL(true, node2->isLeaf());
            if (node1->getShape() != box1) {
                const BvhNode* tmp = node1;
                node1 = node2;
                node2 = tmp;
            }
            CPPUNIT_ASSERT_EQUAL((Shape*)box1, node1->getShape());
            CPPUNIT_ASSERT_EQUAL((Shape*)box2, node2->getShape());
        }

        // make we dont still have collisions after moving one child away from
        // the other
        box1Proxy->translate((real)(boxDimensions.getX() + 5.0), 0.0, 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // make sure our collision comes back if we move both proxies on each
        // other again
        box2Proxy->translate((real)(boxDimensions.getX() + 5.0), 0.0, 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // moving away again, with negative translations this time. just a
        // sanity check.
        box2Proxy->translate((real)(-boxDimensions.getX() - 5.0), 0.0, 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        world.removeProxy(proxy);
        delete proxy;
        
        // another one, this time with a simple hierarchy
        Proxy* topLevelProxy = world.createProxy((ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        Proxy* boxContainer = world.createProxy((ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        box1 = new Box(boxDimensions);
        box2 = new Box(boxDimensions);
        box1Proxy = world.createProxy(box1, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        box2Proxy = world.createProxy(box2, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        boxContainer->addChild(box1Proxy);
        boxContainer->addChild(box2Proxy);
        topLevelProxy->addChild(boxContainer);
        world.addProxy(topLevelProxy);
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        box2Proxy->translate((real)(boxDimensions.getX() + 5.0), 0.0, 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());
    }

    void MiddlePhaseRigidTest::testSelfCollisions() {
        World world(Vector3(2000, 2000, 2000));
        world.prepareSimulation();
        const Vector3 boxDimensions(10.0, 20.0, 30.0);

        Proxy* proxy = world.createProxy((ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));

        Box* box1 = new Box(boxDimensions);
        Box* box2 = new Box(boxDimensions);
        Proxy* box1Proxy = world.createProxy(box1, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        Proxy* box2Proxy = world.createProxy(box2, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        box1Proxy->translate(0.0, (real)(-boxDimensions.getY() - 5.0), 0.0);
        box2Proxy->translate((real)(boxDimensions.getX() + 5.0), (real)(-boxDimensions.getY() - 5.0), 0.0);
        proxy->addChild(box1Proxy);
        proxy->addChild(box2Proxy);

        Proxy* dummyProxy = world.createProxy((ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        Proxy* boxesContainer = world.createProxy((ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        {
            for (int i = 0; i < 10; i++) {
                Box* b1 = new Box(boxDimensions);
                Box* b2 = new Box(boxDimensions);
                Proxy* p1 = world.createProxy(b1, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
                p1->translate((real)(boxDimensions.getX() * i + 5.0 * i), 0.0, 0.0);

                Proxy* p2 = world.createProxy(b2, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
                p2->translate(0.0, (real)(boxDimensions.getY() * (i + 1) + 5.0 * (i + 1)), 0.0);

                boxesContainer->addChild(p1);
                boxesContainer->addChild(p2);
            }
            dummyProxy->addChild(boxesContainer);
        }
        proxy->addChild(dummyProxy);


        // adding a node with a higher depth in the tree
        Proxy* highDepth = world.createProxy((ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
        {
            Proxy* parent = highDepth;
            for (int i = 0; i < 20; i++) {
                Proxy* child = world.createProxy((ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
                parent->addChild(child);
                parent = child;
            }
            Box* box3 = new Box(boxDimensions);
            Proxy* box3Proxy = world.createProxy(box3, (ProxyTypes) (PROXYTYPE_RIGID | PROXYTYPE_SELFCOLLIDABLE));
            parent->addChild(box3Proxy);

            highDepth->translate((real)(-boxDimensions.getX() - 5.0), 0.0, 0.0);
        }
        proxy->addChild(highDepth);

        world.addProxy(proxy);

        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // AB: the hierarchy is supposed to look like this:
        //    Y
        //    Y
        //    Y
        //    Y
        //    Y
        //    Y
        //    Y
        //    Y
        //    Y
        //    Y
        // C  X  X  X  X  X  X  X  X  X  X
        //    A  B
        // where X,Y are the boxes of dummyProxy/boxesContainer, C is the
        // highDepth node and A is box1, B is box2

        // test "highDepth", i.e. 'C' against all 'X' nodes
        for (int i = 0; i < 10; i++) {
            highDepth->translate((real)(boxDimensions.getX() + 5.0), 0.0, 0.0);
            CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());
            const BvhNode* node = world.calculateAllCollisions().getRigidBoundingVolumeCollisions().front().node1;
            Proxy* p = 0;
            while (node) {
                if (node->getParent()) {
                    p = node->getProxy();
                }
                node = node->getParent();
            }
            if (p != highDepth) {
                node = world.calculateAllCollisions().getRigidBoundingVolumeCollisions().front().node2;
                while (node) {
                    if (node->getParent()) {
                        p = node->getProxy();
                    }
                    node = node->getParent();
                }
                if (p != highDepth) {
                    CPPUNIT_FAIL("Collision, but not with highDepth");
                }
            }
        }
        highDepth->translate((real)(boxDimensions.getX() + 5.0), 0.0, 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // move node to (0,0,0)
        highDepth->setTransformation(Matrix());
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // test "highDepth", i.e. 'C' against all 'Y' nodes
        for (int i = 0; i < 10; i++) {
            highDepth->translate(0.0, (real)(boxDimensions.getY() + 5.0), 0.0);
            CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());
            const BvhNode* node = world.calculateAllCollisions().getRigidBoundingVolumeCollisions().front().node1;
            Proxy* p = 0;
            while (node) {
                if (node->getParent()) {
                    p = node->getProxy();
                }
                node = node->getParent();
            }
            if (p != highDepth) {
                node = world.calculateAllCollisions().getRigidBoundingVolumeCollisions().front().node2;
                while (node) {
                    if (node->getParent()) {
                        p = node->getProxy();
                    }
                    node = node->getParent();
                }
                if (p != highDepth) {
                    CPPUNIT_FAIL("Collision, but not with highDepth");
                }
            }
        }

        highDepth->translate(0.0, (real)(boxDimensions.getY() + 5.0), 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // test 'A' against 'B'
        box1Proxy->translate((real)(boxDimensions.getX() + 5.0), 0.0, 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // test 'A' against one 'X' node
        box1Proxy->setTransformation(Matrix());
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // test 3 collisions: 'A' with 'X', 'B' with 'X' and 'A' with 'B'
        box2Proxy->setTransformation(Matrix());
        {
            WorldCollisions collisions = world.calculateAllCollisions();
            const std::list<BoundingVolumeCollision>& list = collisions.getRigidBoundingVolumeCollisions();
            CPPUNIT_ASSERT_EQUAL((size_t)3, list.size());

            bool AB = false;
            bool AX = false;
            bool BX = false;
            for (std::list<BoundingVolumeCollision>::const_iterator it = list.begin(); it != list.end(); ++it) {
                const BvhNode* n1 = (*it).node1;
                const BvhNode* n2 = (*it).node2;
                CPPUNIT_ASSERT(n1 != 0);
                CPPUNIT_ASSERT(n2 != 0);
                const Shape* s1 = n1->getShape();
                const Shape* s2 = n2->getShape();
                CPPUNIT_ASSERT(s1 != 0);
                CPPUNIT_ASSERT(s2 != 0);
                if (s1 == box1 && s2 == box2 || s1 == box2 && s2 == box1) {
                    AB = true;
                } else if (s1 == box1 || s2 == box1) {
                    AX = true;
                } else if (s1 == box2 || s2 == box2) {
                    BX = true;
                }
            }
            CPPUNIT_ASSERT_EQUAL(true, AB);
            CPPUNIT_ASSERT_EQUAL(true, AX);
            CPPUNIT_ASSERT_EQUAL(true, BX);
        }

        // rotations scare me... but we have to test them:
        box1Proxy->translate((real)(-boxDimensions.getX() - 1.0), 0.0, 0.0);
        box2Proxy->translate((real)((-boxDimensions.getX() - 1.0) * 2), 0.0, 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());
        box2Proxy->rotate(-45.0, 0.0, 0.0, 1.0);
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());

        // test with highDepth instead of box2, too
        box2Proxy->setTransformation(Matrix());
        box2Proxy->translate(0.0, (real)(-boxDimensions.getY() - 5.0), 0.0);
        highDepth->setTransformation(Matrix());
        highDepth->translate((real)(-boxDimensions.getX() * 2.0 - 2.0), 0.0, 0.0);
        CPPUNIT_ASSERT_EQUAL((size_t)0, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());
        highDepth->rotate(-45.0, 0.0, 0.0, 1.0);
        CPPUNIT_ASSERT_EQUAL((size_t)1, world.calculateAllCollisions().getRigidBoundingVolumeCollisions().size());
    }


    // AB: this test primarily tests that the MeshSplitter and MeshPart work 
    // properly, since large meshes are split up on construction.
    // -> e.g. we can test that this does not introduce memory leaks (use
    //    valgrind on cppunit tests)
    void MiddlePhaseRigidTest::testLargeMeshes() {
        World world(Vector3(1000, 1000, 1000));

        ModelLoader::LoaderDummy loader;
        real width = 100.0;
        real height = 100.0;
        Mesh* mesh1 = loader.createRectangleSurface(width, height, 4.0, 4.0);
        Mesh* mesh2 = loader.createRectangleSurface(width, height, 4.0, 4.0);

        Proxy* proxy1 = world.createProxy(mesh1);
        Proxy* proxy2 = world.createProxy(mesh2);

        proxy1->translate((dcollide::real)(-width / 2.0), (dcollide::real)(-height / 2.0), 0.0);
        proxy2->translate((dcollide::real)(-width / 2.0), 0.0, (dcollide::real)(-height / 2.0));
        proxy2->rotate(90, 1.0, 0.0, 0.0);

        world.addProxy(proxy1);
        world.addProxy(proxy2);

        world.prepareSimulation();

        WorldCollisions collisions = world.calculateAllCollisions();
        CPPUNIT_ASSERT_EQUAL((size_t)1, collisions.getBroadPhaseCollisions()->getResults().size());
        CPPUNIT_ASSERT(collisions.getRigidBoundingVolumeCollisions().size() >= 1);
    }
}
/*
 * vim: et sw=4 ts=4
 */
