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
#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <math.h>

#include "real.h"
#include "math/vector.h"
#include "boundingvolumes/aabb.h"
#include "aabbtest.h"
#include "shapes/shapes.h"
#include "world.h"
#include "proxy.h"


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

CPPUNIT_TEST_SUITE_REGISTRATION (AabbTest);

const float epsilon = 0.001f;

static std::string box2String(const Aabb& box) {
    std::stringstream ss;
    ss << "("   << box.getMin().getX() << ", "
                << box.getMin().getY() << ", "
                << box.getMin().getZ() << ")";
    ss << ", ";
    ss << "("   << box.getMax().getX() << ", " 
                << box.getMax().getY() << ", " 
                << box.getMax().getZ() << ")";
    ss << ")";
    return ss.str();
}

namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
     void AabbTest::setUp(void) {
        //Constructor for Unit Tests
        //not needed here
        //use this to allocate resources as memory, files etc.
     }

    /*!
     *  \brief resource deallocation for unit test
     */
     void AabbTest::tearDown(void) {
        //Destructor for Unit Tests
        //not needed here
        //use this to free the resources allocated in setUp()
     }

     /*!
      * Test if we can use our 3D Aabbs as 2D Aabbs (i.e. use minZ=maxZ=0.0f)
      */
     void AabbTest::collisionTest2D() {
         Vector3 min;
         Vector3 max;
         min = Vector3(5.0f, 5.0f, 0.0f);
         max = Vector3(6.0f, 6.0f, 0.0f);
         Aabb box(min, max);

         //Check if identical boxes do collide
         CPPUNIT_ASSERT(box.collidesWith(box));

         min = Vector3(6.1f,  5.0f, 0.0f);
         max = Vector3(17.0f, 6.0f, 0.0f);
         Aabb boxNoCollisionRight(min, max);
         CPPUNIT_ASSERT(!box.collidesWith(boxNoCollisionRight));

         min = Vector3(-10.0f, 0.0f, 0.0f);
         max = Vector3(4.9f,  60.0f, 0.0f);
         Aabb boxNoCollisionLeft(min, max);
         CPPUNIT_ASSERT(!box.collidesWith(boxNoCollisionLeft));

         min = Vector3(0.0f, -100.0f, 0.0f);
         max = Vector3(10.0f,   4.9f, 0.0f);
         Aabb boxNoCollisionBelow(min, max);
         CPPUNIT_ASSERT(!box.collidesWith(boxNoCollisionBelow));

         min = Vector3(0.0f,   6.1f, 0.0f);
         max = Vector3(10.0f, 15.0f, 0.0f);
         Aabb boxNoCollisionAbove(min, max);
         CPPUNIT_ASSERT(!box.collidesWith(boxNoCollisionAbove));


         min = Vector3(5.3f, 5.3f, 0.0f);
         max = Vector3(5.6f, 5.6f, 0.0f);
         Aabb boxCollisionCompletelyInside(min, max);
         CPPUNIT_ASSERT(box.collidesWith(boxCollisionCompletelyInside));

         // collisions
         min = Vector3(-15.0f, -5.0f, 0.0f);
         max = Vector3(5.1f, 5.1f, 0.0f);
         CPPUNIT_ASSERT(box.collidesWith(Aabb(min, max)));

         min = Vector3(5.9f,  5.9f, 0.0f);
         max = Vector3(15.0f, 15.0f, 0.0f);
         CPPUNIT_ASSERT(box.collidesWith(Aabb(min, max)));
     }

     void AabbTest::collisionTest3D() {
         Vector3 min;
         Vector3 max;
         min = Vector3(5.0f, -6.0f, -1.0f);
         max = Vector3(6.0f, -5.0f,  1.0f);
         Aabb box(min, max);

         //Check if identical boxes do collide
         CPPUNIT_ASSERT(box.collidesWith(box));

         double diffX = fabs(min.getX() - max.getX());
         double diffY = fabs(min.getY() - max.getY());
         double diffZ = fabs(min.getZ() - max.getZ());
         
         // offsets with that the boxes are meant NOT to collide
         std::list<Vector3> offsetsNoCollision;
         offsetsNoCollision.push_back(Vector3((real)(diffX + 0.1f), 0.0f, 0.0f));
         offsetsNoCollision.push_back(Vector3((real)(-(diffX + 0.1f)), 0.0f, 0.0f));
         offsetsNoCollision.push_back(Vector3(0.0f, (real)((diffY + 0.1f)), 0.0f));
         offsetsNoCollision.push_back(Vector3(0.0f, (real)(-(diffY + 0.1f)), 0.0f));
         offsetsNoCollision.push_back(Vector3(0.0f, 0.0f, (real)(diffZ + 0.1f)));
         offsetsNoCollision.push_back(Vector3(0.0f, 0.0f, (real)(-(diffZ + 0.1f))));

         std::list<Vector3> offsetsCollision;
         offsetsCollision.push_back(Vector3(0.0f, 0.0f, 0.0f)); // will check box against itself
         offsetsCollision.push_back(Vector3((real)(diffX - 0.1f), 0.0f, 0.0f));
         offsetsCollision.push_back(Vector3((real)(-(diffX - 0.1f)), 0.0f, 0.0f));
         offsetsCollision.push_back(Vector3(0.0f, (real)((diffY - 0.1f)), 0.0f));
         offsetsCollision.push_back(Vector3(0.0f, (real)(-(diffY - 0.1f)), 0.0f));
         offsetsCollision.push_back(Vector3(0.0f, 0.0f, (real)(diffZ - 0.1f)));
         offsetsCollision.push_back(Vector3(0.0f, 0.0f, (real)(-(diffZ - 0.1f))));

         std::list<Vector3>::iterator it;
         for (it = offsetsNoCollision.begin(); it != offsetsNoCollision.end(); ++it) {
             std::string str;
             Vector3 min = box.getMin() + *it;
             Vector3 max = box.getMax() + *it;

             // hopefully useful error message
             std::stringstream ss;
             ss << "box1: " << box2String(box) << " ";
             ss << "box2: " << box2String(Aabb(min, max));

             CPPUNIT_ASSERT_MESSAGE(ss.str(), !box.collidesWith(Aabb(min, max)));
         }

         for (it = offsetsCollision.begin(); it != offsetsCollision.end(); ++it) {
             std::string str;
             Vector3 min = box.getMin() + *it;
             Vector3 max = box.getMax() + *it;

             // hopefully useful error message
             std::stringstream ss;
             ss << "box1: " << box2String(box) << " ";
             ss << "box2: " << box2String(Aabb(min, max));

             CPPUNIT_ASSERT_MESSAGE(ss.str(), box.collidesWith(Aabb(min, max)));
         }
     }

     
     void AabbTest::adjustToTest() {
         World world;

        /* The following Macro gives a reasonable error message to the user in
         * the case the test fails.
         * It assumes that the error streams is named "ss" and that always
         * "testAabb" and "wishedSolution" are tested against each other.
         * It also assumes that the heading is named "header"
         * So if you modify their names you better be sure to also modify 
         * the macro!
         * Parameters are the ShapeType & a Shape-Variable you want to test
         */
#define TEST(ShapeType, Shape) \
         ss << header \
            << ShapeType << " failed!" << endl; \
         ss << ShapeType<< " : " << *Shape \
         << " was computed to: " << endl; \
         ss << "    " << *testAabb << " expected result: " << endl; \
         ss << "    " << wishedSolution << endl; \
         CPPUNIT_ASSERT_MESSAGE(ss.str(), wishedSolution == *testAabb); \
         ss.str("");

         std::stringstream ss;      // <- error messages
         Aabb* testAabb = new Aabb();
         Aabb wishedSolution = Aabb(Vector3(-10.0, -10.0, -10.0), 
                                    Vector3(10.0, 10.0, 10.0));
         Sphere* mySphere = new Sphere(10.0);
         Box*   myBox = new Box(Vector3(10.0, 20.0, 30.0));
         Proxy* sphereProxy = world.createProxy(mySphere);
         Proxy* boxProxy = world.createProxy(myBox);
         
         Matrix myMatrix;
         Matrix transMatrix;
         Matrix rotateMatrix;
         
         // header: This one stands above an error message if a test fails
         std::string header = "error: computation of surrounding Aabb of a ";
         
         /* Points and Triangles for my own hardcoded Mesh
          *       +--#   The + points construct a simple triangle with all
          *      / \ |   vertexes have z-coordinate 0.
          *     /   \|   # is the only vertex which has a z-coord. different
          *    +-----+   to 0.
          *              Overall the mesh is a simple pyramid
          * Beware! The ASCII Picture above is not exactly what this pyramid
          *         looks like in reality
          */
         std::vector<Vertex*> vertices(4);
         std::vector<Triangle*> triangles(4);
         /* Ownership for all those items belongs to the vector, who deletes
          * all those objects when the vector-destructor is called
          */
         vertices[0] = new Vertex(Vector3( 0.0,  0.0,  0.0));
         vertices[1] = new Vertex(Vector3(10.0,  0.0,  0.0));
         vertices[2] = new Vertex(Vector3( 5.0, 10.0,  0.0));
         vertices[3] = new Vertex(Vector3( 5.0,  0.0, 10.0));
         triangles[0] = new Triangle(vertices[0], vertices[1], vertices[2]);
         triangles[1] = new Triangle(vertices[0], vertices[1], vertices[3]);
         triangles[2] = new Triangle(vertices[0], vertices[2], vertices[3]);
         triangles[3] = new Triangle(vertices[1], vertices[2], vertices[3]);
         Mesh* myMesh = new Mesh(vertices, triangles);
         myMatrix.loadIdentity();
         Proxy* meshProxy = world.createProxy(myMesh);

         testAabb->adjustToShape(mySphere);
         TEST("sphere", mySphere);
         
         wishedSolution = Aabb(Vector3(0.0, 0.0, 0.0), Vector3(10.0, 20.0, 30.0));
         testAabb->adjustToShape(myBox);
         TEST("box", myBox);
         
         wishedSolution = Aabb(Vector3(0.0, 0.0, 0.0), Vector3(10.0, 10.0, 10.0));
         testAabb->adjustToShape(myMesh);
         TEST("mesh", myMesh);
         
         /**********************************************************************
          * create a translation matrix which moves the object                 *
          * (x:+10,y+20,z:+30) spaces away                                     *
          * Translation shouldn't alter the bounding volumes!                  *
          **********************************************************************/
         
         real transMatrixValues[16] = {1.0, 0.0, 0.0, 10.0,
                                       0.0, 1.0, 0.0, 20.0,
                                       0.0, 0.0, 1.0, 30.0,
                                       0.0, 0.0, 0.0,  1.0};
         transMatrix.loadMatrix(transMatrixValues);
         boxProxy->setTransformation(transMatrix);
         sphereProxy->setTransformation(transMatrix);
         meshProxy->setTransformation(transMatrix);
         header = "error: computation of surrounding Aabb of a translated ";
         
         wishedSolution = Aabb(Vector3(-10.0, -10.0, -10.0),
                                Vector3(10.0, 10.0, 10.0));
         testAabb->adjustToShape(mySphere);
         TEST("sphere", mySphere);
         
         wishedSolution = Aabb(Vector3(0.0, 0.0, 0.0), Vector3(10.0, 20.0, 30.0));
         testAabb->adjustToShape(myBox);
         TEST("box", myBox);
         
         wishedSolution = Aabb(Vector3(0.0, 0.0, 0.0), Vector3(10.0, 10.0, 10.0));
         testAabb->adjustToShape(myMesh);
         TEST("mesh", myMesh);
         
         /**********************************************************************
          * create a rotation matrix which rotates the object around           *
          * the x-axis by 90 degree                                            *
          **********************************************************************/
         real rotateMatrixValues[16] = {1.0, 0.0,  0.0, 0.0,
                                        0.0, 0.0, -1.0, 0.0,
                                        0.0, 1.0,  0.0, 0.0,
                                        0.0, 0.0,  0.0, 1.0};
         rotateMatrix.loadMatrix(rotateMatrixValues);
         boxProxy->setTransformation(rotateMatrix);
         sphereProxy->setTransformation(rotateMatrix);
         meshProxy->setTransformation(rotateMatrix);
         header = "error: computation of surrounding Aabb of a x-rotated ";
    
         wishedSolution = Aabb(Vector3(-10.0, -10.0, -10.0), 
                                Vector3(10.0, 10.0, 10.0));
         testAabb->adjustToShape(mySphere);
         TEST("sphere", mySphere);

         wishedSolution = Aabb(Vector3(0.0, 0.0, -20.0), Vector3(10.0, 30.0, 0.0));
         testAabb->adjustToShape(myBox);
         TEST("box", myBox);
    
         wishedSolution = Aabb(Vector3(0.0, 0.0, -10.0), Vector3(10.0, 10.0, 0.0));
         testAabb->adjustToShape(myMesh);
         TEST("mesh", myMesh);
         
         
         /**********************************************************************
          * rotate it around the y-axis as well! (also 90 degrees)             *
          **********************************************************************/
         rotateMatrixValues[0] = rotateMatrixValues[1] = 0.0;
         rotateMatrixValues[2] = 1.0;
         rotateMatrixValues[3] = rotateMatrixValues[4] = 0.0;
         rotateMatrixValues[5] = 1.0;
         rotateMatrixValues[6] = rotateMatrixValues[7] = 0.0;
         rotateMatrixValues[8] = -1.0;
         rotateMatrixValues[9] = rotateMatrixValues[10] = rotateMatrixValues[11]= 0.0;
         rotateMatrixValues[12] = rotateMatrixValues[13] = rotateMatrixValues[14] = 0.0;
         rotateMatrixValues[15] = 1.0;
         rotateMatrix.loadMatrix(rotateMatrixValues);
         boxProxy->setTransformation(rotateMatrix);
         sphereProxy->setTransformation(rotateMatrix);
         meshProxy->setTransformation(rotateMatrix);
         header = "error: computation of surrounding Aabb of a y-rotated ";
    
         wishedSolution = Aabb(Vector3(-10.0, -10.0, -10.0), Vector3(10.0, 10.0, 10.0));
         testAabb->adjustToShape(mySphere);
         TEST("sphere", mySphere);

         wishedSolution = Aabb(Vector3(-30.0, 0.0, 0.0), Vector3(0.0, 20.0, 10.0));
         testAabb->adjustToShape(myBox);
         TEST("box", myBox);
    
         wishedSolution = Aabb(Vector3(-10.0, 0.0, 0.0), Vector3(0.0, 10.0, 10.0));
         testAabb->adjustToShape(myMesh);
         TEST("mesh", myMesh);
         
         /**********************************************************************
         * rotate it around the z-axis as well! (also 90 degrees)              *
         ***********************************************************************/
         rotateMatrixValues[0] =  0.0;
         rotateMatrixValues[1] = -1.0;
         rotateMatrixValues[2] =  rotateMatrixValues[3] =  0.0;
         rotateMatrixValues[4] =  1.0;
         rotateMatrixValues[5] =  rotateMatrixValues[6] = rotateMatrixValues[7] = 0.0;
         rotateMatrixValues[8] =  rotateMatrixValues[9] = 0.0;
         rotateMatrixValues[10] = 1.0;
         rotateMatrixValues[11]=  rotateMatrixValues[12] = 
                 rotateMatrixValues[13] = rotateMatrixValues[14] = 0.0;
         rotateMatrixValues[15] = 1.0;
         rotateMatrix.loadMatrix(rotateMatrixValues);
         boxProxy->setTransformation(rotateMatrix);
         sphereProxy->setTransformation(rotateMatrix);
         meshProxy->setTransformation(rotateMatrix);
         header = "error: computation of surrounding Aabb of a z-rotated ";
    
         wishedSolution = Aabb(Vector3(-10.0, -10.0, -10.0), Vector3(10.0, 10.0, 10.0));
         testAabb->adjustToShape(mySphere);
         TEST("sphere", mySphere);

         wishedSolution = Aabb(Vector3(0.0, -10.0, 0.0), Vector3(20.0, 0.0, 30.0));
         testAabb->adjustToShape(myBox);
         TEST("box", myBox);
    
         wishedSolution = Aabb(Vector3(0.0, -10.0, 0.0), Vector3(10.0, 0.0, 10.0));
         testAabb->adjustToShape(myMesh);
         TEST("mesh", myMesh);
         
         /**********************************************************************
         * rotate it around the x-axis with a radiant of 0.23                  *
         * our wished Solution was computed by octavia                         *
         ***********************************************************************/
         rotateMatrixValues[0] = 1.0;
         rotateMatrixValues[1] =
             rotateMatrixValues[2] =
             rotateMatrixValues[3] =
             rotateMatrixValues[4] = 0.0;
         rotateMatrixValues[5]  = (real) 0.97367;
         rotateMatrixValues[6]  = (real) 0.22798;
         rotateMatrixValues[7]  = rotateMatrixValues[8] = 0.0;
         rotateMatrixValues[9]  = (real) -0.22798;
         rotateMatrixValues[10] = (real) 0.97367;
         rotateMatrixValues[11] =
             rotateMatrixValues[12] =
             rotateMatrixValues[13] =
             rotateMatrixValues[14] = 0.0;
         rotateMatrixValues[15] = 1.0;
         rotateMatrix.loadMatrix(rotateMatrixValues);
         boxProxy->setTransformation(rotateMatrix);
         sphereProxy->setTransformation(rotateMatrix);
         meshProxy->setTransformation(rotateMatrix);
         header = "error: computation of surrounding Aabb of a (0.23 radiant) x-rotated ";
         
         wishedSolution = Aabb(Vector3(-10.0, -10.0, -10.0),
                                Vector3(10.0, 10.0, 10.0));
         testAabb->adjustToShape(mySphere);
         TEST("sphere", mySphere);
         
         testAabb->adjustToShape(myBox);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0,      testAabb->getMin().getX(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(-6.83933, testAabb->getMin().getY(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0,      testAabb->getMin().getZ(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0,     testAabb->getMax().getX(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(19.4733,  testAabb->getMax().getY(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(33.76954, testAabb->getMax().getZ(), epsilon);
         
         testAabb->adjustToShape(myMesh);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0,      testAabb->getMin().getX(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.2798,  testAabb->getMin().getY(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0,      testAabb->getMin().getZ(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0,     testAabb->getMax().getX(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(9.7367,   testAabb->getMax().getY(), epsilon);
         CPPUNIT_ASSERT_DOUBLES_EQUAL(9.7367,   testAabb->getMax().getZ(), epsilon);
         
         delete testAabb;
         delete boxProxy;
         delete sphereProxy;
         delete meshProxy;

     }
#undef TEST
     
}
/*
 * vim: et sw=4 ts=4
 */
