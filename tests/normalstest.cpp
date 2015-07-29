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
#include <math.h>
#include "normalstest.h"

#include "math/vector.h"

#include "shapes/mesh/vertex.h"
#include "shapes/mesh/triangle.h"


using namespace dcollide;


CPPUNIT_TEST_SUITE_REGISTRATION (NormalsTest);

namespace dcollide {

    /*!
     *  \brief resource allocation for unit test
     */
     void NormalsTest::setUp(void) {
        //Constructor for Unit Tests
        //   *not needed here*
     }

    /*!
     *  \brief resource deallocation for unit test
     */
     void NormalsTest::tearDown(void) {
        //Destructor for Unit Tests
        //   *not needed here*
     }


    /*!
     * \brief Tests the generation of normal vectors for an triangle object
     */
    void NormalsTest::testNormalVectorGeneration() {
        
        Vector3 n1, n2, n3;
        std::vector<Vector3*> normals;
        
        
        Vertex v1(1,0,0);
        Vertex v2(0,1,0);
        Vertex v3(0,2,0);
        
        Triangle t1(&v1, &v2, &v3);
        
        normals = t1.getNormals();
        n1 = *normals[0];
        n2 = *normals[1];
        n3 = *normals[2];
        
        CPPUNIT_ASSERT(n1 == Vector3(0,0,-1));
        CPPUNIT_ASSERT(n2 == Vector3(0,0,-1));
        CPPUNIT_ASSERT(n3 == Vector3(0,0,-1));
    }
    
    /*!
     * \brief Tests the generation of an normal cone out of the normal vectors
     *        of a triangle
     */
    void NormalsTest::testNormalConeGeneration() {

        Vector3 n1, n2, n3;
        std::vector<Vector3*> normals;
        
        
        Vertex v1(1,0,0);
        Vertex v2(0,1,0);
        Vertex v3(0,2,0);
        
        Triangle t1(&v1, &v2, &v3);

        CPPUNIT_ASSERT(*t1.getNormalCone()->getAxis() == Vector3(0,0,-1));
    }
    
    /*!
     * \brief Tests all possible operations of an normal cone object
     */
    void NormalsTest::testNormalConeOperations() {
        
        const float epsilon = 0.00001f;
        
        NormalCone nc1(Vector3(0,1,0), (real)0.000000);
        NormalCone nc2(Vector3(1,0,0), (real)0.087266);
        NormalCone nc3(Vector3(1,2,4), (real)0.413818);
        NormalCone nce1, nce2, nce3;
        
        
        // Test of NormalCone.operator=(const NormalCone& nc)
        nce1 = nc1;
        nce2 = nc1;
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), nc1.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == *nc1.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nc1.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nc1.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), 0.000000, epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == Vector3(0.0, 1.0, 0.0));
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), 0.000000, epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == Vector3(0.0, 1.0, 0.0));
        
        
        nce1 = nc2;
        nce2 = nc2;
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), nc2.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == *nc2.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nc2.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nc2.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), 0.087266, epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == Vector3(1.0, 0.0, 0.0));
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), 0.087266, epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == Vector3(1.0, 0.0, 0.0));
        
        
        nce1 = nc3;
        nce2 = nc3;
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), nc3.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == *nc3.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nc3.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nc3.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), 0.413818, epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == Vector3((real)0.218218,
                                                  (real)0.436436,
                                                  (real)0.872872));
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), 0.413818, epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == Vector3((real)0.218218,
                                                  (real)0.436436,
                                                  (real)0.872872));
        
        
        
        // Test of NormalCone.operator+(const NormalCone& nc)
        nce1 = nc1 + nc2;
        nce2 = nc2 + nc1;
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), nce2.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == *nce2.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), 0.829031, epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == Vector3((real)0.707107,
                                                  (real)0.707107,
                                                  (real)0.000000));
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), 0.829031, epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == Vector3((real)0.707107,
                                                  (real)0.707107,
                                                  (real)0.000000));
        
        
        nce1 = nc1 + nc3;
        nce2 = nc3 + nc1;

        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), nce2.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == *nce2.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), 0.766490, epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == Vector3((real)0.128745331,
                                                  (real)0.847477949,
                                                  (real)0.514981324));
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), 0.766490, epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == Vector3((real)0.128745331,
                                                  (real)0.847477949,
                                                  (real)0.514981324));


        nce1 = nc2 + nc3;
        nce2 = nc3 + nc2;
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), nce2.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == *nce2.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce1.getAngle(), 0.925946, epsilon);
        CPPUNIT_ASSERT(*nce1.getAxis() == Vector3((real)0.780454,
                                                  (real)0.279604,
                                                  (real)0.559207));
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), 0.925946, epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == Vector3((real)0.780454,
                                                  (real)0.279604,
                                                  (real)0.559207));
        
        
        
        // Test of NormalCone.operator+=(const NormalCone& nc)
        nce1  = nc1 + nc2;
        
        nce2  = nc1;
        nce2 += nc2;
        
        nce3  = nc2;
        nce3 += nc1;
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nce3.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nce3.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nce1.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nce1.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce3.getAngle(), nce1.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce3.getAxis() == *nce1.getAxis());
        
        
        nce1  = nc1 + nc3;
        
        nce2  = nc1;
        nce2 += nc3;
        
        nce3  = nc3;
        nce3 += nc1;
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nce3.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nce3.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nce1.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nce1.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce3.getAngle(), nce1.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce3.getAxis() == *nce1.getAxis());
        
        
        nce1  = nc2 + nc3;
        
        nce2  = nc2;
        nce2 += nc3;
        
        nce3  = nc3;
        nce3 += nc2;
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nce3.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nce3.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce2.getAngle(), nce1.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce2.getAxis() == *nce1.getAxis());
        
        CPPUNIT_ASSERT_DOUBLES_EQUAL(nce3.getAngle(), nce1.getAngle(), epsilon);
        CPPUNIT_ASSERT(*nce3.getAxis() == *nce1.getAxis());
    }
}

/*
 * vim: et sw=4 ts=4
 */
