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

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "obbtest.h"

#include <d-collide/debugstream.h>
#include <d-collide/math/vector.h>
#include <d-collide/math/matrix.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/boundingvolumes/obb.h>
#include <d-collide/world.h>
#include <d-collide/proxy.h>

using namespace std;
using namespace dcollide;

CPPUNIT_TEST_SUITE_REGISTRATION(ObbTest);

namespace dcollide {
    ObbTest::ObbTest() {
        mWorld = 0;
    }

    ObbTest::~ObbTest() {
        delete mWorld;
    }

    void ObbTest::setUp(void) {
        WorldParameters parameters;
        parameters.setRigidBoundingVolumeType(BV_TYPE_OBB);
        mWorld = new World(parameters);
    }

    void ObbTest::tearDown(void) {
        delete mWorld;
        mWorld = 0;
    }

    void ObbTest::testConstructors() {
        Obb emptyObb;
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 0.0), emptyObb.getCenter());
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 0.0), emptyObb.getDimension());
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 0.0), emptyObb.getReferencePoint());
        CPPUNIT_ASSERT_EQUAL(Matrix(), emptyObb.getState());
        CPPUNIT_ASSERT_EQUAL(Matrix(), emptyObb.getRotation());
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 0.0), emptyObb.getSurroundingAabbExtents());
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 0.0), emptyObb.getSurroundingAabbMin());
        CPPUNIT_ASSERT_EQUAL(Vector3(0.0, 0.0, 0.0), emptyObb.getSurroundingAabbMax());

        Vector3 dimension(100, 100, 100);
        Vector3 reference(10, 10, 10);
        Matrix rotation;
        rotation.rotate(45, 1, 0, 0);
        Vector3 center(5, 5, 5);
        Obb obb(center, dimension, rotation, reference);
        CPPUNIT_ASSERT_EQUAL(center, obb.getCenter()); // FIXME: this test fails!!
        CPPUNIT_ASSERT_EQUAL(dimension, obb.getDimension());
        CPPUNIT_ASSERT_EQUAL(reference, obb.getReferencePoint());
//        CPPUNIT_ASSERT_EQUAL(, obb.getState()); // TODO
        CPPUNIT_ASSERT_EQUAL(rotation, obb.getRotation());
//        CPPUNIT_ASSERT_EQUAL(, obb.getSurroundingAabbExtents());
//        CPPUNIT_ASSERT_EQUAL(, obb.getSurroundingAabbMin());
//        CPPUNIT_ASSERT_EQUAL(, obb.getSurroundingAabbMax());

        // testing operator=()
        Obb obb2;
        obb2 = obb;
        checkIfObbsAreEqual(obb, obb2, __LINE__);
    }

    void ObbTest::testAdjustTo() {
        Box* box = new Box(100, 50, 20);
        Proxy* p = mWorld->createProxy(box);
        mWorld->addProxy(p);

        Obb obb;

#define CHECK_ADJUST_TO_BOX \
            { \
                Matrix m = p->getTransformation(); \
                obb.adjustToBox(&m, box); \
                checkIfVerticesAreEqual(obb, box->getVertices(), __LINE__); \
            }

        // initial adjustTo()
        CHECK_ADJUST_TO_BOX;

        // simple translations
        p->setTransformation(Matrix());
        p->translate(10, 2, 66);
        CHECK_ADJUST_TO_BOX;

        p->setTransformation(Matrix());
        p->translate(-10, -2, -66);
        CHECK_ADJUST_TO_BOX;

        p->setTransformation(Matrix());
        p->translate(10, 2, -66);
        CHECK_ADJUST_TO_BOX;

        // simple rotations
        p->setTransformation(Matrix());
        p->rotate(45, 1, 0, 0);
        CHECK_ADJUST_TO_BOX;

        p->setTransformation(Matrix());
        p->rotate(45, 0, 1, 0);
        CHECK_ADJUST_TO_BOX;

        p->setTransformation(Matrix());
        p->rotate(45, 0, 0, 1);
        CHECK_ADJUST_TO_BOX;

        // multiple rotations
        p->setTransformation(Matrix());
        p->rotate(45, 1, 0, 0);
        p->rotate(45, 0, 1, 0);
        CHECK_ADJUST_TO_BOX;

        // translations and rotations
        p->setTransformation(Matrix());
        p->translate(10, 2, 66);
        p->rotate(45, 1, 0, 0);
        CHECK_ADJUST_TO_BOX;

        p->setTransformation(Matrix());
        p->rotate(45, 1, 0, 0);
        p->translate(10, 2, 66);
        CHECK_ADJUST_TO_BOX;

        p->setTransformation(Matrix());
        p->translate(10, 2, 66);
        p->rotate(45, 1, 0, 0);
        p->translate(10, 2, 66);
        CHECK_ADJUST_TO_BOX;

        p->setTransformation(Matrix());
        p->rotate(45, 1, 0, 0);
        p->translate(10, 2, 66);
        p->rotate(45, 1, 0, 0);
        CHECK_ADJUST_TO_BOX;

        p->setTransformation(Matrix());
        p->rotate(45, 1, 0, 0);
        p->translate(10, 2, 66);
        p->rotate(45, 1, 0, 0);
        p->translate(-10, -2, 66);
        CHECK_ADJUST_TO_BOX;

    }

    void ObbTest::testHierarchyWithTranslateRotate() {
        // create several simple hierarchies that should have all the same OBB values
        Box* referenceBox = new Box(100, 50, 20);
        Proxy* referenceProxy = mWorld->createProxy(referenceBox);
        referenceProxy->translate(11, 22, 33);
        referenceProxy->rotate(180, 1, 0, 0);
        mWorld->addProxy(referenceProxy);

        // this OBB will be used as a reference OBB
        Obb referenceObb;
        referenceObb.adjustToShape(referenceBox);

        {
            // translate and rotate the shapeNode AFTER adding to container
            // and world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            mWorld->addProxy(container);
            shapeNode->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate and rotate the shapeNode BEFORE adding to container
            // and world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            shapeNode->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);
            container->addChild(shapeNode);
            mWorld->addProxy(container);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate and rotate the shapeNode AFTER adding to container
            // but BEFORE adding to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            shapeNode->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);
            mWorld->addProxy(container);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate and rotate the shapeNode AFTER adding to world
            // but BEFORE adding to container
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            mWorld->addProxy(container);
            shapeNode->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);
            container->addChild(shapeNode);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate and rotate the container AFTER adding shapeNode to container
            // and container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            mWorld->addProxy(container);
            container->translate(11, 22, 33);
            container->rotate(180, 1, 0, 0);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate and rotate the container BEFORE adding shapeNode to container
            // and container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->translate(11, 22, 33);
            container->rotate(180, 1, 0, 0);
            container->addChild(shapeNode);
            mWorld->addProxy(container);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate and rotate the container AFTER adding shapeNode to container
            // and BEFORE adding container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            container->translate(11, 22, 33);
            container->rotate(180, 1, 0, 0);
            mWorld->addProxy(container);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate and rotate the container BEFORE adding shapeNode to container
            // and AFTER adding container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            mWorld->addProxy(container);
            container->translate(11, 22, 33);
            container->rotate(180, 1, 0, 0);
            container->addChild(shapeNode);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate container, rotate shapeNode
            // AFTER adding shapeNode to container and container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            mWorld->addProxy(container);
            container->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate container, rotate shapeNode
            // BEFORE adding shapeNode to container and container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);
            container->addChild(shapeNode);
            mWorld->addProxy(container);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate container, rotate shapeNode
            // BEFORE adding shapeNode to container and AFTER adding container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            mWorld->addProxy(container);
            container->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);
            container->addChild(shapeNode);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // translate container, rotate shapeNode
            // AFTER adding shapeNode to container and BEFORE adding container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            container->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);
            mWorld->addProxy(container);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // FIRST rotate, THEN translate (translation must be adjusted)
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            mWorld->addProxy(container);
            shapeNode->rotate(180, 1, 0, 0);
            shapeNode->translate(11, -22, -33);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // rotate the CONTAINER (== first rotate), translate the
            // SHAPENODE (== then translate)
            // AFTER adding shapeNode to container and container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            mWorld->addProxy(container);
            container->rotate(180, 1, 0, 0);
            shapeNode->translate(11, -22, -33);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
        {
            // rotate the CONTAINER (== first rotate), translate the
            // SHAPENODE (== then translate)
            // BEFORE adding shapeNode to container and container to world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->rotate(180, 1, 0, 0);
            shapeNode->translate(11, -22, -33);
            container->addChild(shapeNode);
            mWorld->addProxy(container);

            checkIfObbsAreEqual(referenceObb, container->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode(), __LINE__);
            checkIfObbsAreEqual(referenceObb, shapeNode->getBvHierarchyNode()->getChildren().front(), __LINE__);
        }
    }

    void ObbTest::testHierarchyWithTwoShapesAndTranslateRotate() {
        Box* referenceBox1 = new Box(100, 50, 20);
        Box* referenceBox2 = new Box(10, 115, 200);
        Proxy* referenceProxy1 = mWorld->createProxy(referenceBox1);
        Proxy* referenceProxy2 = mWorld->createProxy(referenceBox2);
        referenceProxy1->translate(11, 22, 33);
        referenceProxy1->rotate(180, 1, 0, 0);
        referenceProxy2->translate(11, 22, 33);
        referenceProxy2->rotate(180, 1, 0, 0);
        mWorld->addProxy(referenceProxy1);
        mWorld->addProxy(referenceProxy2);

        {
            // translate and rotate the shapeNode AFTER adding to container
            // and world
            Box* box = new Box(100, 50, 20);
            Proxy* shapeNode = mWorld->createProxy(box);
            Proxy* container = mWorld->createProxy();
            container->addChild(shapeNode);
            mWorld->addProxy(container);
            shapeNode->translate(11, 22, 33);
            shapeNode->rotate(180, 1, 0, 0);

            checkIfObbContains(container->getBvHierarchyNode(), referenceBox1, __LINE__);
            checkIfObbContains(container->getBvHierarchyNode(), referenceBox2, __LINE__);
            checkIfObbContains(shapeNode->getBvHierarchyNode(), referenceBox1, __LINE__);
            checkIfObbContains(shapeNode->getBvHierarchyNode(), referenceBox2, __LINE__);
            checkIfObbContains(shapeNode->getBvHierarchyNode()->getChildren().front(), referenceBox1, __LINE__);
            checkIfObbContains(shapeNode->getBvHierarchyNode()->getChildren().front(), referenceBox2, __LINE__);
        }
    }

    void ObbTest::checkIfObbsAreEqual(const Obb& obb1, const BvhNode* bvhNode, int line) {
        const BoundingVolume* bv = bvhNode->getBoundingVolume();
        CPPUNIT_ASSERT(bv != 0);
        CPPUNIT_ASSERT(bv->getVolumeType() == BV_TYPE_OBB);
        const Obb* obb2 = static_cast<const Obb*>(bv);
        checkIfObbsAreEqual(obb1, *obb2, line);
    }

    void ObbTest::checkIfObbsAreEqual(const Obb& obb1, const Obb& obb2, int line) {
        std::stringstream str;
        str << "checkIfObbsAreEqual() failed called by line " << line;
        CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), obb1.getCenter(), obb2.getCenter());
        CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), obb1.getDimension(), obb2.getDimension());
        CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), obb1.getReferencePoint(), obb2.getReferencePoint());
        CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), obb1.getState(), obb2.getState());
        CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), obb1.getRotation(), obb2.getRotation());
        CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), obb1.getSurroundingAabbExtents(), obb2.getSurroundingAabbExtents());
        CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), obb1.getSurroundingAabbMin(), obb2.getSurroundingAabbMin());
        CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), obb1.getSurroundingAabbMax(), obb2.getSurroundingAabbMax());
    }

    void ObbTest::checkIfVerticesAreEqual(const Obb& obb, const Vector3* vertices, int line) {
        Vector3* obbVertices = obb.getVertices();
        for (int i = 0; i < 8; i++) {
            std::stringstream str;
            str << "checkIfVerticesAreEqual() failed called by line " << line
                << " with vertex " << i;
            CPPUNIT_ASSERT_EQUAL_MESSAGE(str.str(), vertices[i], obbVertices[i]);
        }
        delete[] obbVertices;
    }

    void ObbTest::checkIfObbContains(const BvhNode* bvhNode, Box* box, int line) {
        const Obb* obb = static_cast<const Obb*>(bvhNode->getBoundingVolume());
        const Vector3* vertices = box->getVertices();

        Plane planes[6];
        /* stolen from Box::getPlanes() */
        Vector3* vs = obb->getVertices();
        Vector3 norm;
        planes[0] = Plane((vs[2]-vs[0])*(vs[4]-vs[0]),vs[0],true);
        norm = (vs[5]-vs[1])*(vs[3]-vs[1]);
        planes[1] = Plane(norm,vs[1],true);
        norm = (vs[4]-vs[0])*(vs[1]-vs[0]);
        planes[2] = Plane(norm,vs[0],true);
        planes[3] = Plane((vs[3]-vs[2])*(vs[6]-vs[2]),vs[2],true);
        planes[4] = Plane((vs[1]-vs[0])*(vs[2]-vs[0]),vs[0],true);
        norm = (vs[6]-vs[4])*(vs[5]-vs[4]);
        planes[5] = Plane(norm,vs[4],true);
        /* stolen from Box::getPlanes() end */

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 8; j++) {
                std::stringstream str;
                str << "checkIfObbContains() failed called by line " << line
                    << " with plane " << i << " and with vertex " << j << " - distance: " << planes[i].calculateDistance(vertices[j]);
                CPPUNIT_ASSERT_MESSAGE(str.str(), planes[i].calculateDistance(vertices[j]) > -0.0001);
            }
        }


    }

}
/*
 * vim: et sw=4 ts=4
 */
