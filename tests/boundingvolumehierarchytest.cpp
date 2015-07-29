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

#include "boundingvolumehierarchytest.h"

#include <proxy.h>
#include <world.h>
#include <shapes/shapes.h>
#include "dcollide-defines.h"

#include <math.h>

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

CPPUNIT_TEST_SUITE_REGISTRATION (BoundingVolumeHierarchyTest);

const float epsilon = 0.0001f;


static dcollide::Mesh* createMeshifiedBox(const dcollide::Vector3& boxDimensions, const dcollide::Vector3& offset = dcollide::Vector3()) {
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


namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
    void BoundingVolumeHierarchyTest::setUp(void) {
    }

    /*!
     *  \brief resource deallocation for unit test
     */
    void BoundingVolumeHierarchyTest::tearDown(void) {
    }

    // test correction creation of a BVH for a single Proxy (i.e. without
    // children)
    void BoundingVolumeHierarchyTest::singleProxyBvhConstructionTest() {
        World world(Vector3(2000, 2000, 2000));

        Proxy* proxy = 0;
        const Vector3 boxDimensions(10.0f, 20.0f, 30.0f);
        proxy = world.createProxy(new Box(boxDimensions));
        world.addProxy(proxy);
        CPPUNIT_ASSERT(proxy->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(proxy->getBvHierarchyNode()->getBoundingVolume() != 0);

        Vector3 min = proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        Vector3 max = proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, min.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, min.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, min.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getX(), max.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getY(), max.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getZ(), max.getZ(), epsilon);

        // The same with a Mesh.
        // We simply emulate a box with the same dimensions here.
        proxy = world.createProxy(createMeshifiedBox(boxDimensions));
        world.addProxy(proxy);

        min = proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        max = proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, min.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, min.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, min.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getX(), max.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getY(), max.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getZ(), max.getZ(), epsilon);

        // A special case: a mesh where the point (0,0,0) is not inside the
        // model and should not be inside the bounding volume.
        Vector3 offset(10.0f, 10.0f, 10.0f);
        proxy = world.createProxy(createMeshifiedBox(boxDimensions, offset));
        world.addProxy(proxy);
        min = proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        max = proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        CPPUNIT_ASSERT_DOUBLES_EQUAL(offset.getX(), min.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(offset.getY(), min.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(offset.getZ(), min.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(offset.getX() + boxDimensions.getX(), max.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(offset.getY() + boxDimensions.getY(), max.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(offset.getZ() + boxDimensions.getZ(), max.getZ(), epsilon);
    }

    /*!
     * Construct a simply Proxy hierarchy with a single proxy only and test if
     * moving that proxy around works.
     */
    void BoundingVolumeHierarchyTest::singleProxyBvhTranslationTest() {
        World world(Vector3(2000, 2000, 2000));

        Proxy* proxyBox = 0;
        Proxy* proxyMesh = 0; // We simply emulate a box with the same dimensions here.
        const Vector3 boxDimensions(10.0f, 20.0f, 30.0f);
        proxyBox = world.createProxy(new Box(boxDimensions));
        proxyMesh = world.createProxy(createMeshifiedBox(boxDimensions));


        //GJ: some more tests to check that the mesh hierarchy is correct
        CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().size() ==1);
        CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()->getShape() != 0);

        // meshsplitting testcode is adjusted to MAX_BVHNODE_LEAF_TRIANGLES==4
        if (MAX_BVHNODE_LEAF_TRIANGLES == 4) {
            CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()->getChildren().size() == 2);
            //each of these nodes should be a Meshpart node with 6 Triangles
            //Each of these meshpart nodes should have: a shape pointer pointing to a meshpart, a boundingvolume
            MeshPart* part = (MeshPart*) proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().front()//first split, should have 6 Triangles
                                                            ->getShape();
            CPPUNIT_ASSERT(part->getTriangleIndices().size()==6);
    
            CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().front()//first split, should have 6 Triangles
                                                            ->getProxy() == 0);
            CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().front()//first split, should have 6 Triangles
                                                            ->getBoundingVolume() != 0);
        
            //each of the nodes with 6 triangles should have 2 children, with 2 or 4 triangles
            CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()
                                                        ->getChildren().front()
                                                        ->getChildren().size() == 2);
            part = (MeshPart*) proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().front()//first split, should have 6 Triangles
                                                            ->getChildren().front()//next level split
                                                            ->getShape();
            CPPUNIT_ASSERT(part->getTriangleIndices().size()==4);
    
            part = (MeshPart*) proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().front()//first split, should have 6 Triangles
                                                            ->getChildren().back()//next level split
                                                            ->getShape();
            CPPUNIT_ASSERT(part->getTriangleIndices().size()==2);
    
            part = (MeshPart*) proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().back()//first split, should have 6 Triangles
                                                            ->getChildren().front()//next level split
                                                            ->getShape();
            CPPUNIT_ASSERT(part->getTriangleIndices().size()==2);
    
            part = (MeshPart*) proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().back()//first split, should have 6 Triangles
                                                            ->getChildren().back()//next level split
                                                            ->getShape();
            CPPUNIT_ASSERT(part->getTriangleIndices().size()==4);
    
            //the 2/4 nodes should be leaves.
            CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().front()//first split, should have 6 Triangles
                                                            ->getChildren().front()//next level split
                                                            ->isLeaf());
    
            CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().front()//first split, should have 6 Triangles
                                                            ->getChildren().back()//next level split
                                                            ->isLeaf());
            CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().back()//first split, should have 6 Triangles
                                                            ->getChildren().front()//next level split
                                                            ->isLeaf());
    
            CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getChildren().front()//Mesh-Node
                                                            ->getChildren().back()//first split, should have 6 Triangles
                                                            ->getChildren().back()//next level split
                                                            ->isLeaf());
        }

        //----------- Test 1: transform BEFORE adding to the world ----------//
        Vector3 moveBy = Vector3(10.0f, 10.0f, 10.0f);
        proxyBox->translate(moveBy);
        proxyMesh->translate(moveBy);


        CPPUNIT_ASSERT(proxyBox->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(proxyBox->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getBoundingVolume() != 0);


        //front node should have 2 children with 
        // make sure translations _before_ adding the Proxy to the world are
        // working properly
        Vector3 minBox = proxyBox->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        Vector3 maxBox = proxyBox->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        Vector3 minMesh = proxyMesh->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        Vector3 maxMesh = proxyMesh->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getX(), minBox.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getY(), minBox.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getZ(), minBox.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getX() + moveBy.getX(), maxBox.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getY() + moveBy.getY(), maxBox.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getZ() + moveBy.getZ(), maxBox.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getX(), minMesh.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getY(), minMesh.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getZ(), minMesh.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getX() + moveBy.getX(), maxMesh.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getY() + moveBy.getY(), maxMesh.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getZ() + moveBy.getZ(), maxMesh.getZ(), epsilon);

        //----------- Test 2: transform AFTER adding to the world ----------//
        world.addProxy(proxyBox);
        world.addProxy(proxyMesh);

        moveBy = Vector3(10.0f, 10.0f, 10.0f);
        proxyBox->translate(moveBy);
        proxyMesh->translate(moveBy);
        // make sure translations _after_ adding the Proxy to the world are
        // working properly
        minBox = proxyBox->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        maxBox = proxyBox->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        minMesh = proxyMesh->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        maxMesh = proxyMesh->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getX() * 2, minBox.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getY() * 2, minBox.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getZ() * 2, minBox.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getX() + moveBy.getX() * 2, maxBox.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getY() + moveBy.getY() * 2, maxBox.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getZ() + moveBy.getZ() * 2, maxBox.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getX() * 2, minMesh.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getY() * 2, minMesh.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(moveBy.getZ() * 2, minMesh.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getX() + moveBy.getX() * 2, maxMesh.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getY() + moveBy.getY() * 2, maxMesh.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getZ() + moveBy.getZ() * 2, maxMesh.getZ(), epsilon);
    }

    /*!
     * Construct a simply Proxy hierarchy with a single proxy only and test if
     * applying arbitrary matrices (using \ref Proxy::setTransformation) works.
     */
    void BoundingVolumeHierarchyTest::singleProxyBvhTransformationTest() {
        World world(Vector3(2000, 2000, 2000));

        Proxy* proxyBox = 0;
        Proxy* proxyMesh = 0; // We simply emulate a box with the same dimensions here.
        const Vector3 boxDimensions(10.0f, 20.0f, 30.0f);
        proxyBox = world.createProxy(new Box(boxDimensions));
        proxyMesh = world.createProxy(createMeshifiedBox(boxDimensions));
        world.addProxy(proxyBox);
        world.addProxy(proxyMesh);

        Matrix m;
        m.translate(10.0f, 30.0f, 70.0f);
        proxyBox->setTransformation(m);
        proxyMesh->setTransformation(m);

        CPPUNIT_ASSERT(proxyBox->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(proxyBox->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(proxyMesh->getBvHierarchyNode()->getBoundingVolume() != 0);

        Vector3 minBox = proxyBox->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        Vector3 maxBox = proxyBox->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        Vector3 minMesh = proxyMesh->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();
        Vector3 maxMesh = proxyMesh->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax();
        CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0f, minBox.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, minBox.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(70.0f, minBox.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getX() + 10.0f, maxBox.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getY() + 30.0f, maxBox.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getZ() + 70.0f, maxBox.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0f, minMesh.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, minMesh.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(70.0f, minMesh.getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getX() + 10.0f, maxMesh.getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getY() + 30.0f, maxMesh.getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(boxDimensions.getZ() + 70.0f, maxMesh.getZ(), epsilon);

        // TODO: at least also test rotations

    }

    void BoundingVolumeHierarchyTest::hierarchyConstructionTest() {
        World world(Vector3(1000, 1000, 1000));

        Proxy* proxy = world.createProxy();
        Proxy* boxContainer = world.createProxy();
        proxy->addChild(boxContainer);

        Proxy* box1 = world.createProxy(new Box(Vector3(10.0f, 20.0f, 30.0f)));
        Proxy* box2 = world.createProxy(new Box(Vector3(10.0f, 20.0f, 30.0f)));
        box2->translate(10.0f, 0.0f, 0.0f);
        Proxy* box3 = world.createProxy(new Box(Vector3(10.0f, 20.0f, 30.0f)));
        box3->translate(10.0f, 20.0f, 30.0f);

        boxContainer->addChild(box1);
        boxContainer->addChild(box2);
        boxContainer->addChild(box3);

        world.addProxy(proxy);

        // ensure that BvhNode objects have been created for all proxies
        CPPUNIT_ASSERT(proxy->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(boxContainer->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(box1->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(box2->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(box3->getBvHierarchyNode() != 0);

        // ensure that every BvhNode has a bounding volume
        CPPUNIT_ASSERT(proxy->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(boxContainer->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(box1->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(box2->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(box3->getBvHierarchyNode()->getBoundingVolume() != 0);

        //check the correct structure of the bvh
        CPPUNIT_ASSERT(proxy->getBvHierarchyNode()->getParent() == 0);
        CPPUNIT_ASSERT(boxContainer->getBvHierarchyNode()->getParent() == proxy->getBvHierarchyNode());
        CPPUNIT_ASSERT(box1->getBvHierarchyNode()->getParent() == boxContainer->getBvHierarchyNode());
        CPPUNIT_ASSERT(box2->getBvHierarchyNode()->getParent() == boxContainer->getBvHierarchyNode());
        CPPUNIT_ASSERT(box3->getBvHierarchyNode()->getParent() == boxContainer->getBvHierarchyNode());
        //each of the box-proxies should have a child node with the shape
        CPPUNIT_ASSERT(box1->getBvHierarchyNode()->getShape() == 0);
        CPPUNIT_ASSERT(box1->getBvHierarchyNode()->getChildren().size() == 1);
        CPPUNIT_ASSERT(box1->getBvHierarchyNode()->getChildren().front()->getParent() == box1->getBvHierarchyNode());
        CPPUNIT_ASSERT(box1->getBvHierarchyNode()->getChildren().front()->getShape() != 0);

        // ensure correct size of each BV
        // (actually we ensure a correct minimum size only. we don't care if
        // larger BVs are created - the BV tests should catch such problems. we
        // can't do that here, since we don't know which type of BV is in use)
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(40.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);

        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(40.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);
        // AB: since proxy has exactly one child, its BV should be the same as
        // the one of its child (==boxContainer).
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(40.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);


        // test if adding a box _after_ adding the proxy to the World works
        // properly
        Proxy* box4 = world.createProxy(new Box(Vector3(10.0f, 20.0f, 30.0f)));
        box4->translate(20.0f, 0.0f, 0.0f);
        //still need to add the box to the container
        boxContainer->addChild(box4);
        CPPUNIT_ASSERT(boxContainer->getBvHierarchyNode()->getChildren().size()==4);

        CPPUNIT_ASSERT(proxy->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(boxContainer->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(box1->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(box2->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(box3->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(box4->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(proxy->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(boxContainer->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(box1->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(box2->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(box3->getBvHierarchyNode()->getBoundingVolume() != 0);
        CPPUNIT_ASSERT(box4->getBvHierarchyNode()->getBoundingVolume() != 0);
        // make sure other children have not changed
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, box1->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, box2->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(40.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0f, box3->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);

        // test new child
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box4->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box4->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, box4->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, box4->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(20.0f, box4->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, box4->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);

        // test if boxContainer and proxy (which already were added to World) got updated properly when new child
        // was added
        // note: proxy has exactly 1 child (boxContainer) and thus it's BV
        // should match the one of its child
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(40.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0f, boxContainer->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin().getZ(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(30.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getX(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(40.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getY(), epsilon);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0f, proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMax().getZ(), epsilon);

    }

    void BoundingVolumeHierarchyTest::hierarchyStructureTest() {
        //--create a hierarchy with intermediate nodes--
        World world;

        Proxy* top = world.createProxy();
        //ensure top-level node is set up correct
        CPPUNIT_ASSERT(top->getBvHierarchyNode() != 0);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getParent() == 0);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().empty());

        //create and add children MAX_CHILD_COUNT children 
        //(no intermediate nodes should be generated so far)
        Proxy* lv1Child1 = world.createProxy();
        top->addChild(lv1Child1);
        Proxy* lv1Child2 = world.createProxy();
        top->addChild(lv1Child2);
        Proxy* lv1Child3 = world.createProxy();
        top->addChild(lv1Child3);
        Proxy* lv1Child4 = world.createProxy();
        top->addChild(lv1Child4);
        Proxy* lv1Child5 = world.createProxy();
        top->addChild(lv1Child5);

        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getParent() == 0);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().size()==5);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()
                            == lv1Child1->getBvHierarchyNode());
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().back() == 
                            lv1Child5->getBvHierarchyNode());
        CPPUNIT_ASSERT(lv1Child1->getBvHierarchyNode()->getParent() == top->getBvHierarchyNode());
        CPPUNIT_ASSERT(lv1Child1->getBvHierarchyNode()->getChildren().empty());
        CPPUNIT_ASSERT(lv1Child1->getBvHierarchyNode()->getProxy() == lv1Child1);
        CPPUNIT_ASSERT(lv1Child5->getBvHierarchyNode()->getParent() == top->getBvHierarchyNode());
        CPPUNIT_ASSERT(lv1Child5->getBvHierarchyNode()->getChildren().empty());
        
        //create and add another child, intermediate nodes should be created
        Proxy* lv1Child6 = world.createProxy();
        top->addChild(lv1Child6);
        
        //now, top should have only 2 nodes, which have no proxy assigned
        //each intermediate node should carry 3 of the proxy-nodes
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().size()==2);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()
                                                ->getChildren().size()==3);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().back()
                                                ->getChildren().size()==3);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()
                                                ->getProxy() == NULL);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()->getParent() 
                                                == top->getBvHierarchyNode());
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()
                                                ->getChildren().front()
                                                == lv1Child1->getBvHierarchyNode());
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()
                                                ->getChildren().back()
                                                == lv1Child3->getBvHierarchyNode());
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().back()
                                                ->getChildren().front()
                                                == lv1Child4->getBvHierarchyNode());
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().back()
                                                ->getChildren().back()
                                                == lv1Child6->getBvHierarchyNode());
        //add a child to lv1child1
        Proxy* lv2Child1 = world.createProxy();
        lv1Child1->addChild(lv2Child1);

        CPPUNIT_ASSERT(lv1Child1->getBvHierarchyNode()->getChildren().size()==1);
        CPPUNIT_ASSERT(lv1Child1->getBvHierarchyNode()->getChildren().front()
                            == lv2Child1->getBvHierarchyNode());
        //other nodes should remain unchanged
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().size()==2);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()
                                                ->getChildren().size()==3);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().back()
                                                ->getChildren().size()==3);

        //-----delete inner proxies---------
        delete lv1Child1;
        //now, lvl1child and its lv2child should be removed.
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().size()==2);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()
                                                ->getChildren().size()==2);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().back()
                                                ->getChildren().size()==3);
        CPPUNIT_ASSERT(top->getBvHierarchyNode()->getChildren().front()
                                                ->getChildren().front()
                                                == lv1Child2->getBvHierarchyNode());
        delete top;
        
    }

    // TODO another test which:
    // * creates a proxy hierarchy
    // * applies transformations on it (translate and maybe a simple rotation)
    // * _then_ adds to the world (_after_ the transformations were made)
    // * once added to the world, checks if the BV is still calculated correctly
}
/*
 * vim: et sw=4 ts=4
 */
