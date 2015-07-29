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

#include "deformablebvhnodetest.h"

#include "shapes/mesh/vertex.h"
#include "detectordeform/surfacehierarchy/deformablebvhnode.h"
#include "world.h"
#include "shapes/mesh.h"
#include <list>
#include "debugstream.h"

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

CPPUNIT_TEST_SUITE_REGISTRATION (DeformableBvhNodeTest);


namespace dcollide {

/*!
 *  \brief resource allocation for unit test
 */
    void DeformableBvhNodeTest::setUp(void) {
        //Constructor for Unit Tests

        try {
            mWorld = new World();
        } catch (Exception e) {
            mWorld = 0;
            throw (e);
        }

        std::vector<Triangle*> meshTriangles;
        std::vector<Vertex*> meshVertecis;


        triangles1.push_back(new Triangle(new Vertex(0.0,0.0,0.0),
                                         new Vertex(0.0,1.0,0.0),
                                         new Vertex(1.0,0.0,0.0)));

        triangles1.push_back(new Triangle(new Vertex(1.0,0.0,0.0),
                                         new Vertex(0.0,1.0,0.0),
                                         new Vertex(1.0,1.0,0.0)));

        triangles1.push_back(new Triangle(new Vertex(1.0,1.0,0.0),
                                         new Vertex(0.0,2.0,0.0),
                                         new Vertex(0.0,1.0,0.0)));

        for (std::list<Triangle*>::iterator iter = triangles1.begin(); iter != triangles1.end(); iter++) {
            meshTriangles.push_back(*iter);
            dcollide::array<Vertex*,3> v = (*iter)->getVertices();
                meshVertecis.push_back(v[0]);
                meshVertecis.push_back(v[1]);
                meshVertecis.push_back(v[2]);
        }

        center1 = triangles1.back()->getVertices()[2];
        //creating a node with a list of enclosing triangles and one center Vertex
        node1 = new DeformableBvhNode(mWorld, triangles1, center1);

        //Second node
        triangles2.push_back(new Triangle(new Vertex(0.0,5.0,0.0),
                                          new Vertex(1.0,5.0,0.0),
                                          new Vertex(0.0,6.0,0.0)));

        triangles2.push_back(new Triangle(new Vertex(0.0,6.0,0.0),
                                          new Vertex(1.0,5.0,0.0),
                                          new Vertex(1.0,6.0,0.0)));

        triangles2.push_back(new Triangle(new Vertex(1.0,6.0,0.0),
                                          new Vertex(0.0,6.0,0.0),
                                          new Vertex(0.0,7.0,0.0)));


        center2 = triangles2.front()->getVertices()[0];
        node2 = new DeformableBvhNode(mWorld, triangles2, center2);

        for (std::list<Triangle*>::iterator iter = triangles2.begin(); iter != triangles2.end(); iter++) {
            meshTriangles.push_back(*iter);
            dcollide::array<Vertex*,3> v = (*iter)->getVertices();
            meshVertecis.push_back(v[0]);
            meshVertecis.push_back(v[1]);
            meshVertecis.push_back(v[2]);
        }

        //Third node
        triangles3.push_back(new Triangle(new Vertex(3.0,3.0,0.0),
                                          new Vertex(3.0,3.0,0.0),
                                          new Vertex(4.0,3.0,0.0)));

        triangles3.push_back(new Triangle(new Vertex(3.0,3.0,0.0),
                                          new Vertex(3.0,4.0,0.0),
                                          new Vertex(4.0,3.0,0.0)));

        triangles3.push_back(new Triangle(new Vertex(3.0,4.0,0.0),
                                          new Vertex(3.0,5.0,0.0),
                                          new Vertex(4.0,4.0,0.0)));

        center3 = triangles3.front()->getVertices()[0];
        node3 = new DeformableBvhNode(mWorld, triangles3, center3);

        for (std::list<Triangle*>::iterator iter = triangles3.begin(); iter != triangles3.end(); iter++) {
            meshTriangles.push_back(*iter);
            dcollide::array<Vertex*,3> v = (*iter)->getVertices();
            meshVertecis.push_back(v[0]);
            meshVertecis.push_back(v[1]);
            meshVertecis.push_back(v[2]);
        }
        
        //Fourth node
        trianglesCenterNode.push_back(new Triangle(new Vertex(0.0,3.0,0.0),
                                          new Vertex(0.0,4.0,0.0),
                                          new Vertex(1.0,3.0,0.0)));

        trianglesCenterNode.push_back(new Triangle(new Vertex(0.0,2.0,0.0),
                                          new Vertex(0.0,3.0,0.0),
                                          new Vertex(1.0,2.0,0.0)));

        trianglesCenterNode.push_back(new Triangle(new Vertex(0.0,3.0,0.0),
                                          new Vertex(1.0,3.0,0.0),
                                          new Vertex(1.0,2.0,0.0)));

        center_centerNode = trianglesCenterNode.front()->getVertices()[0];
        //std::cout << center_centerNode->getPosition().getX() << " " << center_centerNode->getPosition().getY() << " " << center_centerNode->getPosition().getZ() << std::endl;
                
        for (std::list<Triangle*>::iterator iter = trianglesCenterNode.begin(); iter != trianglesCenterNode.end(); iter++) {
            meshTriangles.push_back(*iter);
            dcollide::array<Vertex*,3> v = (*iter)->getVertices();
            meshVertecis.push_back(v[0]);
            meshVertecis.push_back(v[1]);
            meshVertecis.push_back(v[2]);
        }
        
        mMesh = new Mesh(meshVertecis, meshTriangles);
        Proxy* p = mWorld->createProxy(mMesh, PROXYTYPE_DEFORMABLE);
        mWorld->addProxy(p);
        mWorld->prepareSimulation();
        centerNode = new DeformableBvhNode(mWorld, trianglesCenterNode, center_centerNode);
    }

    /*!
     *  \brief resource deallocation for unit test
     */
    void DeformableBvhNodeTest::tearDown(void) {
        delete mWorld;

        // TODO: delete the other objects created by setUp()!
    }

    void DeformableBvhNodeTest::bvhNodeCreationWithTriangleListTest() {

        real r = ((BoundingSphere*)((node1)->getBoundingVolume()))->getRadius();
        //std::cout << r << std::endl;
        CPPUNIT_ASSERT ((r >= 1.414) && (r <= 1.415));

        r = ((BoundingSphere*)((node2)->getBoundingVolume()))->getRadius();
        //std::cout << r << std::endl;
        CPPUNIT_ASSERT (r == 2);

        r = ((BoundingSphere*)((node3)->getBoundingVolume()))->getRadius();
        CPPUNIT_ASSERT (r == 2);


        //std::cout << "ende CreationWithTriangleListTest" << std::endl;
        //std::cout << std::endl;
    }

    void DeformableBvhNodeTest::bvhNodeCreationWithNodesAndCenterVertexTest() {
        std::list<DeformableBvhNode*> bvhNodes;
        bvhNodes.push_back(node1);
        bvhNodes.push_back(node2);
        bvhNodes.push_back(node3);

        node4 = new DeformableBvhNode(mWorld, bvhNodes, centerNode);

        real r = ((BoundingSphere*)((node4)->getBoundingVolume()))->getRadius();

        //distance between centerVertex and the center of the last bvhSphere
        // + the radius of the last bvhSphere = 5
        CPPUNIT_ASSERT (r == 5);

        //std::cout << "ende test creationWithNodesAndCenterVertex" << std::endl;
        //std::cout << std::endl;
    }

    void DeformableBvhNodeTest::bvhNodeCreationWithNodeListTest() {
        std::list<DeformableBvhNode*> bvhNodes;
        bvhNodes.push_back(node1);
        bvhNodes.push_back(node2);
        bvhNodes.push_back(node3);

        node4 = new DeformableBvhNode(mWorld, bvhNodes);

        real r = ((BoundingSphere*)((node4)->getBoundingVolume()))->getRadius();
/*        Vector3 v = ((BoundingSphere*)((node4)->getBoundingVolume()))->getCenterVertex()->getPosition();

        warning() << r ;
        warning() << v.getX() << " " << v.getY() << " " << v.getZ();
        
        CPPUNIT_ASSERT (r < 4.37 && r > 4.36);
        CPPUNIT_ASSERT (v.getX() < 0.66 && v.getX() > 0.65);
        CPPUNIT_ASSERT (v.getY() < 3.23 && v.getY() > 3.22);
        CPPUNIT_ASSERT (v.getZ() == 0.0);
*/
        //std::cout << std::endl;
        //std::cout << "Radius " << r << std::endl;
        //std::cout << "Center " << v.getX() << " " << v.getY() << " " << v.getZ() << std::endl;

        //std::cout << "ende Test CreationWithNodeList" << std::endl;
        //std::cout << std::endl;
    }

    void DeformableBvhNodeTest::addOneTriangleTest() {
        //adds one Triangle to the node and recalculate the Sphere
        Triangle* triangle = new Triangle(new Vertex(0.0,2.0,0.0),
                                          new Vertex(0.0,3.0,0.0),
                                          new Vertex(1.0,2.0,0.0));
        node1->addTriangle(triangle);
        real r = ((BoundingSphere*)((node1)->getBoundingVolume()))->getRadius();

        CPPUNIT_ASSERT(r == 2);

        Triangle* triangle2 = new Triangle(new Vertex(2.0,1.0,0.0),
                                           new Vertex(3.0,1.0,0.0),
                                           new Vertex(2.0,2.0,0.0));
        node1->addTriangle(triangle2);
        r = ((BoundingSphere*)((node1)->getBoundingVolume()))->getRadius();

        CPPUNIT_ASSERT(r == 3);

        //std::cout << "ende Test addOneTriangle" << std::endl;
        //std::cout << std::endl;
    }
}
/*
 * vim: et sw=4 ts=4
 */

