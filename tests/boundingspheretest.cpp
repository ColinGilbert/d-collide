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

#include "math.h"
#include "boundingspheretest.h"
#include "shapes/mesh/vertex.h"
#include "shapes/mesh/triangle.h"

#include "boundingvolumes/boundingsphere.h"

//-------------------------------------
//-------using directives--------------
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

CPPUNIT_TEST_SUITE_REGISTRATION (BoundingSphereTest);

namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
     void BoundingSphereTest::setUp(void) {
         // nothing to do
     }

    /*!
     *  \brief resource deallocation for unit test
     */
     void BoundingSphereTest::tearDown(void) {
         // nothing to do
     }


     /*!
      * \brief Test if BoundingSphere::mergeWith works properly
      */
     void BoundingSphereTest::mergeTest() {

         //first the creation of a dummyWorld and one dummyMesh,
         //without the algorithm does not work correct
         try {
             mWorld = new World();
         } catch (Exception e) {
             mWorld = 0;
             throw (e);
         }
         
         std::vector<Triangle*> meshTriangles;
         std::vector<Vertex*> meshVertecis;
         
         Vertex* v1 = new Vertex(0,0,0);
         Vertex* v2 = new Vertex(0,0,0);
         Vertex* v3 = new Vertex(10,0,0);
         Vertex* v4 = new Vertex(1,0,0);
         Vertex* v5 = new Vertex(4,0,0);
         
         meshTriangles.push_back(new Triangle(v1,v2,v3));
         meshTriangles.push_back(new Triangle(v3,v4,v5));
         meshVertecis.push_back(v1);
         meshVertecis.push_back(v2);
         meshVertecis.push_back(v3);
         meshVertecis.push_back(v4);
         meshVertecis.push_back(v5);
         
         mMesh = new Mesh(meshVertecis, meshTriangles);
         Proxy* p = mWorld->createProxy(mMesh, PROXYTYPE_DEFORMABLE);
         mWorld->addProxy(p);
         mWorld->prepareSimulation();
         
         const float epsilon = 0.00001f;

         BoundingSphere sphere_null  = BoundingSphere();
         BoundingSphere sphere_one   = BoundingSphere(v1, 1);
         BoundingSphere sphere_two   = BoundingSphere(v2, 1);
         BoundingSphere sphere_three = BoundingSphere(v3, 3.5);
         BoundingSphere sphere_four  = BoundingSphere(v4, 1);
         BoundingSphere sphere_five  = BoundingSphere(v5, 5);
         sphere_one.mergeWith(&sphere_two);

         CPPUNIT_ASSERT_EQUAL(Vector3(0,0,0), sphere_one.getCenterVertex()->getPosition());
         CPPUNIT_ASSERT_DOUBLES_EQUAL(1, sphere_one.getRadius(), epsilon);


         sphere_one.mergeWith(&sphere_three);
         //sphere_three.mergeWith(&sphere_one);

         
         CPPUNIT_ASSERT_EQUAL(Vector3(6.25,0,0), sphere_one.getCenterVector());
         //CPPUNIT_ASSERT_EQUAL(Vector3(6.25,0,0), sphere_one.getCenterVertex()->getPosition());
         CPPUNIT_ASSERT_DOUBLES_EQUAL(7.25, sphere_one.getRadius(), epsilon);


         sphere_one.mergeWith(&sphere_two);

         CPPUNIT_ASSERT_EQUAL(Vector3(6.25,0,0), sphere_one.getCenterVector());
         CPPUNIT_ASSERT_DOUBLES_EQUAL(7.25, sphere_one.getRadius(), epsilon);


         sphere_two.mergeWith(&sphere_four);

         CPPUNIT_ASSERT_EQUAL(Vector3(0.5,0,0), sphere_two.getCenterVector());
         CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, sphere_two.getRadius(), epsilon);


         sphere_five.mergeWith(&sphere_two);

         CPPUNIT_ASSERT_EQUAL(Vector3(4,0,0), sphere_five.getCenterVector());
         CPPUNIT_ASSERT_DOUBLES_EQUAL(5, sphere_five.getRadius(), epsilon);


         sphere_one   = BoundingSphere(Vector3(-3,  3, 0), 5);
         sphere_two   = BoundingSphere(Vector3( 3,  3, 0), 5);
         sphere_three = BoundingSphere(Vector3(-3, -3, 0), 5);
         sphere_four  = BoundingSphere(Vector3( 3, -3, 0), 5);

         std::list<BoundingSphere*> sphere_list = std::list<BoundingSphere*>();

         sphere_list.push_back(&sphere_one);
         sphere_list.push_back(&sphere_two);
         sphere_list.push_back(&sphere_three);
         sphere_list.push_back(&sphere_four);

         sphere_null.mergeWith(sphere_list);

         //our current algorithm only calculates an approximated new boundingSphere from 3 or more
         //given boundingSpheres
         //maybe we can activate the tests after we have implemented
         //an exact way to calculate new boundingSpheres
         //CPPUNIT_ASSERT_DOUBLES_EQUAL(sqrt(18.0f)+5, sphere_null.getRadius(), epsilon);
         //CPPUNIT_ASSERT_EQUAL(Vector3(0, 0, 0), sphere_null.getCenter());
     }

     /*!
      * \brief Test if BoundingSphere::reset works properly
      */
     void BoundingSphereTest::resetTest() {

     }
}
/*
 * vim: et sw=4 ts=4
 */
