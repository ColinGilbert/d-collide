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

#include "kdoptest.h"
#include <math/vector.h>
#include <boundingvolumes/kdop.h>
#include <shapes/sphere.h>
#include <shapes/box.h>
#include <shapes/mesh.h>
#include <world.h>
#include <proxy.h>
#include <debugstream.h>

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

CPPUNIT_TEST_SUITE_REGISTRATION (KDOPTest);

#define DEBUG_KDOP_STRINGS 0
#if DEBUG_KDOP_STRINGS
static std::string kdop2String(const Kdop& kdop) {
    std::stringstream ss;
    int k = kdop.getK();
    ss << "(" << k << "; (";
    for (int i = 0; i < k; i++) {
        ss << kdop.getDistanceOfPlaneToOrigin(i);
        if (i < k - 1) {
            ss << "; ";
        }
    }
    ss << "))";
    return ss.str();
}

static std::string kdop2DetailedString(const Kdop& kdop) {
    std::stringstream ss;
    int k = kdop.getK();
    ss << "(k=" << k << "; (" << std::endl;
    for (int i = 0; i < k; i++) {
        Vector3 n = kdop.getPlaneNormal(i);
        ss << "  plane " << i << ": distance: " << kdop.getDistanceOfPlaneToOrigin(i);
        ss << " ; normal: ";
        ss << "(" << n[0] << "; " << n[1] << "; " << n[2] << ")";
        ss << std::endl;
    }
    ss << "))";
    return ss.str();
}
#endif

namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
     void KDOPTest::setUp(void) {
        //Constructor for Unit Tests
        //not needed here
        //use this to allocate resources as memory, files etc.
     }

    /*!
     *  \brief resource deallocation for unit test
     */
     void KDOPTest::tearDown(void) {
        //Destructor for Unit Tests
        //not needed here
        //use this to free the resources allocated in setUp()
     }

     void KDOPTest::testConstructor() {
        Kdop kdopDefaultK;
        CPPUNIT_ASSERT(kdopDefaultK.getK() == 14);

        Kdop kdop6(6);
        Kdop kdop14(14);
        Kdop kdop18(18);
        Kdop kdop26(26);
        CPPUNIT_ASSERT(kdop6.getK() == 6);
        CPPUNIT_ASSERT(kdop14.getK() == 14);
        CPPUNIT_ASSERT(kdop18.getK() == 18);
        CPPUNIT_ASSERT(kdop26.getK() == 26);

        // all other values for k should be invalid and default to a valid
        // k.
        // (testing values of k > 100 will be pointless imho)
        for (int i = 0; i < 100; i++) {
            if (i == 6 || i == 14 || i == 18 || i == 26) {
                continue;
            }
            Kdop kdopInvalid(i);

            // atm the default fallback is k == 14
            CPPUNIT_ASSERT(kdopInvalid.getK() == 14);
        }
     }

     void KDOPTest::testPlaneNormals() {
         Kdop kdop6(6);
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 0.0f, 0.0f), kdop6.getPlaneNormal(0));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, -1.0f, 0.0f), kdop6.getPlaneNormal(1));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 0.0f, -1.0f), kdop6.getPlaneNormal(2));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 0.0f, 0.0f), kdop6.getPlaneNormal(3));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 1.0f, 0.0f), kdop6.getPlaneNormal(4));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 0.0f, 1.0f), kdop6.getPlaneNormal(5));

         Kdop kdop14(14);
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 0.0f, 0.0f), kdop14.getPlaneNormal(0));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, -1.0f, 0.0f), kdop14.getPlaneNormal(1));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 0.0f, -1.0f), kdop14.getPlaneNormal(2));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, -1.0f, -1.0f), kdop14.getPlaneNormal(3));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 1.0f, -1.0f), kdop14.getPlaneNormal(4));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, -1.0f, 1.0f), kdop14.getPlaneNormal(5));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 1.0f, 1.0f), kdop14.getPlaneNormal(6));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 0.0f, 0.0f), kdop14.getPlaneNormal(7));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 1.0f, 0.0f), kdop14.getPlaneNormal(8));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 0.0f, 1.0f), kdop14.getPlaneNormal(9));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 1.0f, 1.0f), kdop14.getPlaneNormal(10));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, -1.0f, 1.0f), kdop14.getPlaneNormal(11));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 1.0f, -1.0f), kdop14.getPlaneNormal(12));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, -1.0f, -1.0f), kdop14.getPlaneNormal(13));

         Kdop kdop18(18);
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 0.0f, 0.0f), kdop18.getPlaneNormal(0));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, -1.0f, 0.0f), kdop18.getPlaneNormal(1));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 0.0f, -1.0f), kdop18.getPlaneNormal(2));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, -1.0f, 0.0f), kdop18.getPlaneNormal(3));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 0.0f, -1.0f), kdop18.getPlaneNormal(4));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, -1.0f, -1.0f), kdop18.getPlaneNormal(5));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 1.0f, 0.0f), kdop18.getPlaneNormal(6));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 0.0f, 1.0f), kdop18.getPlaneNormal(7));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, -1.0f, 1.0f), kdop18.getPlaneNormal(8));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 0.0f, 0.0f), kdop18.getPlaneNormal(9));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 1.0f, 0.0f), kdop18.getPlaneNormal(10));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 0.0f, 1.0f), kdop18.getPlaneNormal(11));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 1.0f, 0.0f), kdop18.getPlaneNormal(12));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 0.0f, 1.0f), kdop18.getPlaneNormal(13));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 1.0f, 1.0f), kdop18.getPlaneNormal(14));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, -1.0f, 0.0f), kdop18.getPlaneNormal(15));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 0.0f, -1.0f), kdop18.getPlaneNormal(16));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 1.0f, -1.0f), kdop18.getPlaneNormal(17));


         Kdop kdop26(26);
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 0.0f, 0.0f), kdop26.getPlaneNormal(0));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, -1.0f, 0.0f), kdop26.getPlaneNormal(1));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 0.0f, -1.0f), kdop26.getPlaneNormal(2));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, -1.0f, -1.0f), kdop26.getPlaneNormal(3));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 1.0f, -1.0f), kdop26.getPlaneNormal(4));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, -1.0f, 1.0f), kdop26.getPlaneNormal(5));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 1.0f, 1.0f), kdop26.getPlaneNormal(6));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, -1.0f, 0.0f), kdop26.getPlaneNormal(7));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 0.0f, -1.0f), kdop26.getPlaneNormal(8));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, -1.0f, -1.0f), kdop26.getPlaneNormal(9));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 1.0f, 0.0f), kdop26.getPlaneNormal(10));
         CPPUNIT_ASSERT_EQUAL(Vector3(-1.0f, 0.0f, 1.0f), kdop26.getPlaneNormal(11));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, -1.0f, 1.0f), kdop26.getPlaneNormal(12));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 0.0f, 0.0f), kdop26.getPlaneNormal(13));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 1.0f, 0.0f), kdop26.getPlaneNormal(14));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 0.0f, 1.0f), kdop26.getPlaneNormal(15));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 1.0f, 1.0f), kdop26.getPlaneNormal(16));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, -1.0f, 1.0f), kdop26.getPlaneNormal(17));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 1.0f, -1.0f), kdop26.getPlaneNormal(18));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, -1.0f, -1.0f), kdop26.getPlaneNormal(19));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 1.0f, 0.0f), kdop26.getPlaneNormal(20));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 0.0f, 1.0f), kdop26.getPlaneNormal(21));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 1.0f, 1.0f), kdop26.getPlaneNormal(22));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, -1.0f, 0.0f), kdop26.getPlaneNormal(23));
         CPPUNIT_ASSERT_EQUAL(Vector3(1.0f, 0.0f, -1.0f), kdop26.getPlaneNormal(24));
         CPPUNIT_ASSERT_EQUAL(Vector3(0.0f, 1.0f, -1.0f), kdop26.getPlaneNormal(25));
     }

     void KDOPTest::testResetK6() {
         Kdop kdop1(6);
         Kdop kdop2(6);

         // a simple box with min=(5,-9-1) and max=(6,-7,1)
         // -> the resulting k-DOP should be equivalent to an Aabb
         Vector3 min = Vector3(5.0f, -9.0f, -1.0f);
         Vector3 max = Vector3(6.0f, -7.0f, 1.0f);
         std::list<Vector3> box;
         box.push_back(min);
         box.push_back(max);

         kdop1.reset(box);

         // more points of the box (but the same box!)
         box.clear();
         box.push_back(min);
         box.push_back(Vector3(5.0f, -9.0f, 1.0f));
         box.push_back(Vector3(5.0f, -7.0f, -1.0f));
         box.push_back(Vector3(5.0f, -7.0f, 1.0f));
         box.push_back(Vector3(6.0f, -9.0f, -1.0f));
         box.push_back(Vector3(6.0f, -9.0f, 1.0f));
         box.push_back(Vector3(6.0f, -7.0f, -1.0f));
         box.push_back(max);

         kdop2.reset(box);

         // atm the plane distances in the k-DOPs are sorted:
         // minX, minY, minZ, maxX, maxY, maxZ
         // note: if this sorting ever changes, this test will fail, but it
         // is NOT a bug!
         // -> this test then needs to be adapted
         std::vector<float> expectedDistance(6);

         // AB: note: the values returned by getDistanceOfPlaneToOrigin() are
         // w.r.t. getPlaneNormal(), i.e. they are not actually min or max
         // (depending on the implementation) of that normal, whereas we
         // calculate the AABB always w.r.t. only one of the two normals. thus
         // we have to change the sign for k/2 values.
         expectedDistance[0] = min[0];
         expectedDistance[1] = min[1];
         expectedDistance[2] = min[2];
         expectedDistance[3] = (real)(max[0] * -1.0);
         expectedDistance[4] = (real)(max[1] * -1.0);
         expectedDistance[5] = (real)(max[2] * -1.0);

         CPPUNIT_ASSERT((int)expectedDistance.size() == kdop1.getK());
         for (int i = 0; i < kdop1.getK(); i++) {
             const float epsilon = 0.0001f;
             CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedDistance[i], kdop1.getDistanceOfPlaneToOrigin(i), epsilon);
             CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedDistance[i], kdop2.getDistanceOfPlaneToOrigin(i), epsilon);
         }



         // k-DOPs with k!=6 may look totally different - however 6
         // planes (the Aabb planes) should be the same as for k==6.
         box.clear();
         box.push_back(min);
         box.push_back(max);
         Kdop kdop14(14);
         Kdop kdop18(18);
         Kdop kdop26(26);
         kdop14.reset(box);
         kdop18.reset(box);
         kdop26.reset(box);
         for (int i = 0; i < 6; i++) {
             float d14;
             float d18;
             float d26;
             if (i < 3) {
                 d14 = kdop14.getDistanceOfPlaneToOrigin(i);
                 d18 = kdop14.getDistanceOfPlaneToOrigin(i);
                 d26 = kdop14.getDistanceOfPlaneToOrigin(i);
             } else {
                 d14 = kdop14.getDistanceOfPlaneToOrigin(14 / 2 + i - 3);
                 d18 = kdop18.getDistanceOfPlaneToOrigin(18 / 2 + i - 3);
                 d26 = kdop26.getDistanceOfPlaneToOrigin(26 / 2 + i - 3);
             }
             const float epsilon = 0.0001f;
             CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedDistance[i], d14, epsilon);
             CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedDistance[i], d18, epsilon);
             CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedDistance[i], d26, epsilon);
         }
     }

     void KDOPTest::testResetK14() {
         Kdop kdop(14);

         // a simple box with min=(5,-9-1) and max=(6,-7,1)
         // -> the resulting k-DOP should be equivalent to an Aabb
         std::list<Vector3> box;
         box.push_back(Vector3(5.0f, -9.0f, -1.0f));
         box.push_back(Vector3(5.0f, -9.0f, 1.0f));
         box.push_back(Vector3(5.0f, -7.0f, -1.0f));
         box.push_back(Vector3(5.0f, -7.0f, 1.0f));
         box.push_back(Vector3(6.0f, -9.0f, -1.0f));
         box.push_back(Vector3(6.0f, -9.0f, 1.0f));
         box.push_back(Vector3(6.0f, -7.0f, -1.0f));
         box.push_back(Vector3(6.0f, -7.0f, 1.0f));

         kdop.reset(box);


         std::vector<float> expectedDistance(14);

         // AB: note: the values returned by getDistanceOfPlaneToOrigin() are
         // w.r.t. getPlaneNormal(), i.e. they are not actually min or max
         // (depending on the implementation) of that normal, whereas we
         // calculate the AABB always w.r.t. only one of the two normals. thus
         // we have to change the sign for k/2 values.

         expectedDistance[0] = 5.0f; // neg. x-axis
         expectedDistance[1] = -9.0f; // neg. y-axis
         expectedDistance[2] = -1.0f; // neg. z-axis
         expectedDistance[3] = -5.0f; // normal=(-1,-1,-1)
         expectedDistance[4] = 11.0f; // normal=(-1,1,-1)
         expectedDistance[5] = -5.0f; // normal=(-1,-1,1)
         expectedDistance[6] = 11.0f; // normal=(-1,1,1)
         expectedDistance[14 / 2 + 0] = -6.0f; // x-axis
         expectedDistance[14 / 2 + 1] = 7.0f; //y-axis
         expectedDistance[14 / 2 + 2] = -1.0f; // z-axis
         expectedDistance[14 / 2 + 3] = 0.0f; // normal=(1,1,1)
         expectedDistance[14 / 2 + 4] = -16.0f; // normal=(1,-1,1)
         expectedDistance[14 / 2 + 5] = 0.0f; // normal=(1,1,-1)
         expectedDistance[14 / 2 + 6] = -16.0f; // normal=(1,-1,-1)

         for (int i = 0; i < kdop.getK(); i++) {
             const float epsilon = 0.0001f;
             CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedDistance[i], kdop.getDistanceOfPlaneToOrigin(i), epsilon);
         }
     }

     void KDOPTest::testResetK18() {
         Kdop kdop(18);

         // a simple box with min=(5,-9-1) and max=(6,-7,1)
         // -> the resulting k-DOP should be equivalent to an Aabb
         std::list<Vector3> box;
         box.push_back(Vector3(5.0f, -9.0f, -1.0f));
         box.push_back(Vector3(5.0f, -9.0f, 1.0f));
         box.push_back(Vector3(5.0f, -7.0f, -1.0f));
         box.push_back(Vector3(5.0f, -7.0f, 1.0f));
         box.push_back(Vector3(6.0f, -9.0f, -1.0f));
         box.push_back(Vector3(6.0f, -9.0f, 1.0f));
         box.push_back(Vector3(6.0f, -7.0f, -1.0f));
         box.push_back(Vector3(6.0f, -7.0f, 1.0f));

         kdop.reset(box);

         // AB: about the expected values: they can be calculated in different
         //     ways.
         //     (1) assume the code is correct and use the values calculated by
         //         the k-DOP implementation. obviously this way sucks, as we
         //         can only find _new_ errors, we have to assume the initial
         //         implementation is correct.
         //     (2) use a piece of paper and a pen and draw the box (in 2D) in a
         //         coordinate system and "project" it on the normals (4 points
         //         on every normal). use the min/max values on that normal.
         //         -> essentially a few geometry calculations here
         //     (3) what we essentially need is the distance of every point from
         //         the plane (and then use the min/max values).
         //         -> the distance of a point p from a plane is:
         //            distance(p, plane) = dotProduct(n, plane.normal) +
         //                                 plane.distanceFromOrigin
         //            (both, normal and distanceFromOrigin define the normal)
         //            with distanceFromOrigin=0 always for our use here.
         //
         //            so if you know the correct planes/plane normals, then
         //            it's pretty easy to calculate the values (that's why we
         //            have a testPlaneNormals() test)
         //     I used (2) for some points (to ensure the formulas actually are
         //     correct) and (3) for the rest of the values below.
         //     (1) was used for "believability" checks only.
         std::vector<float> expectedDistance(18);

         // AB: note: the values returned by getDistanceOfPlaneToOrigin() are
         // w.r.t. getPlaneNormal(), i.e. they are not actually min or max
         // (depending on the implementation) of that normal, whereas we
         // calculate the AABB always w.r.t. only one of the two normals. thus
         // we have to change the sign for k/2 values.

         expectedDistance[0] = 5.0f; // neg. x-axis
         expectedDistance[1] = -9.0f; // neg. y-axis
         expectedDistance[2] = -1.0f; // neg. z-axis
         expectedDistance[3] = -4.0f; // normal=(-1,-1,0)
         expectedDistance[4] = 4.0f; // normal=(-1,0,-1)
         expectedDistance[5] = -10.0f; // normal=(0,-1,-1)
         expectedDistance[6] = 12.0f; // normal=(-1,1,0)
         expectedDistance[7] = 4.0f; // normal=(-1,0,1)
         expectedDistance[8] = -10.0f; // normal=(0,-1,1)
         expectedDistance[18 / 2 + 0] = -6.0f; // max on x-axis
         expectedDistance[18 / 2 + 1] = 7.0f; // max on y-axis
         expectedDistance[18 / 2 + 2] = -1.0f; // max on z-axis
         expectedDistance[18 / 2 + 3] = 1.0f; // max on normal=(1,1,0)
         expectedDistance[18 / 2 + 4] = -7.0f; // max on normal=(1,0,1)
         expectedDistance[18 / 2 + 5] = 6.0f; // max on normal=(0,1,1)
         expectedDistance[18 / 2 + 6] = -15.0f; // max on normal=(1,-1,0)
         expectedDistance[18 / 2 + 7] = -7.0f; // max on normal=(1,0,-1)
         expectedDistance[18 / 2 + 8] = 6.0f; // max on normal=(0,1,-1)

         for (int i = 0; i < kdop.getK(); i++) {
             const float epsilon = 0.0001f;
             CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedDistance[i], kdop.getDistanceOfPlaneToOrigin(i), epsilon);
         }

         // for debugging purposes
#if DEBUG_KDOP_STRINGS
         std::cout << "k-DOP: " << kdop2String(kdop) << std::endl;
         std::cout << "k-DOP: " << kdop2DetailedString(kdop) << std::endl;
#endif
     }

     void KDOPTest::testResetK26() {
         Kdop kdop(26);

         // a simple box with min=(5,-9-1) and max=(6,-7,1)
         // -> the resulting k-DOP should be equivalent to an Aabb
         std::list<Vector3> box;
         box.push_back(Vector3(5.0f, -9.0f, -1.0f));
         box.push_back(Vector3(5.0f, -9.0f, 1.0f));
         box.push_back(Vector3(5.0f, -7.0f, -1.0f));
         box.push_back(Vector3(5.0f, -7.0f, 1.0f));
         box.push_back(Vector3(6.0f, -9.0f, -1.0f));
         box.push_back(Vector3(6.0f, -9.0f, 1.0f));
         box.push_back(Vector3(6.0f, -7.0f, -1.0f));
         box.push_back(Vector3(6.0f, -7.0f, 1.0f));

         kdop.reset(box);

         std::vector<float> expectedDistance(26);

         // AB: note: the values returned by getDistanceOfPlaneToOrigin() are
         // w.r.t. getPlaneNormal(), i.e. they are not actually min or max
         // (depending on the implementation) of that normal, whereas we
         // calculate the AABB always w.r.t. only one of the two normals. thus
         // we have to change the sign for k/2 values.

         expectedDistance[0] = 5.0f;    // neg. x-axis
         expectedDistance[1] = -9.0f;   // neg. y-axis
         expectedDistance[2] = -1.0f;   // neg. z-axis
         expectedDistance[3] = -5.0f;   // normal=(-1,-1,-1)
         expectedDistance[4] = 11.0f;   // normal=(-1,1,-1)
         expectedDistance[5] = -5.0f;   // normal=(-1,-1,1)
         expectedDistance[6] = 11.0f;   // normal=(-1,1,1)
         expectedDistance[7] = -4.0f;   // normal=(-1,-1,0)
         expectedDistance[8] = 4.0f;    // normal=(-1,0,-1)
         expectedDistance[9] = -10.0f;  // normal=(0,-1,-1)
         expectedDistance[10] = 12.0f;  // normal=(-1,1,0)
         expectedDistance[11] = 4.0f;   // normal=(-1,0,1)
         expectedDistance[12] = -10.0f; // normal=(0,-1,1)
         expectedDistance[26 / 2 + 0] = -6.0f;   // x-axis
         expectedDistance[26 / 2 + 1] = 7.0f;  // max on y-axis
         expectedDistance[26 / 2 + 2] = -1.0f;   // max on z-axis
         expectedDistance[26 / 2 + 3] = 0.0f;   // max on normal=(1,1,1)
         expectedDistance[26 / 2 + 4] = -16.0f;  // max on normal=(1,-1,1)
         expectedDistance[26 / 2 + 5] = 0.0f;   // max on normal=(1,1,-1)
         expectedDistance[26 / 2 + 6] = -16.0f;  // max on normal=(1,-1,-1)
         expectedDistance[26 / 2 + 7] = 1.0f;  // max on normal=(1,1,0)
         expectedDistance[26 / 2 + 8] = -7.0f;   // max on normal=(1,0,1)
         expectedDistance[26 / 2 + 9] = 6.0f;  // max on normal=(0,1,1)
         expectedDistance[26 / 2 + 10] = -15.0f; // max on normal=(1,-1,0)
         expectedDistance[26 / 2 + 11] = -7.0f;  // max on normal=(1,0,-1)
         expectedDistance[26 / 2 + 12] = 6.0f; // max on normal=(0,1,-1)

         for (int i = 0; i < kdop.getK(); i++) {
             const float epsilon = 0.0001f;
             CPPUNIT_ASSERT_DOUBLES_EQUAL(expectedDistance[i], kdop.getDistanceOfPlaneToOrigin(i), epsilon);
         }
     }

     void KDOPTest::testCollideK6() {
         Kdop kdop1(6);
         Kdop kdop2(6);

         std::list<Vector3> box1;
         box1.push_back(Vector3(5.0f, -9.0f, -1.0f));
         box1.push_back(Vector3(6.0f, -7.0f, 1.0f));
         kdop1.reset(box1);

         // test if equal k-DOPs collide
         std::list<Vector3> box2;
         box2.push_back(Vector3(5.0f, -9.0f, -1.0f));
         box2.push_back(Vector3(6.0f, -7.0f, 1.0f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         box2.clear();
         box2.push_back(Vector3(0.0f, -9.0f, -1.0f));
         box2.push_back(Vector3(6.0f, -7.0f, 1.0f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision in left-lower-back corner
         box2.clear();
         box2.push_back(Vector3(4.9f, -19.0f, -10.0f));
         box2.push_back(Vector3(5.1f, -8.9f, -0.9f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision in left-lower-front corner
         box2.clear();
         box2.push_back(Vector3(4.9f, -19.0f, 0.9f));
         box2.push_back(Vector3(5.1f, -8.9f, 2.0f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision in left-upper-back corner
         box2.clear();
         box2.push_back(Vector3(4.9f, 19.0f, -10.0f));
         box2.push_back(Vector3(5.1f, -7.1f, -0.9f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision in left-upper-front corner
         // -> in fact this also tests that the "epsilon" for float comparisons
         //    is not too large
         box2.clear();
         box2.push_back(Vector3(4.9f, 19.0f, 0.999f));
         box2.push_back(Vector3(5.1f, -7.1f, 1.001f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision in right-lower-back corner
         box2.clear();
         box2.push_back(Vector3(5.9f, -19.0f, -10.0f));
         box2.push_back(Vector3(6.1f, -8.9f, -0.9f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision in right-lower-front corner
         box2.clear();
         box2.push_back(Vector3(5.9f, -19.0f, 0.9f));
         box2.push_back(Vector3(6.1f, -8.9f, 10.0f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision in right-upper-back corner
         box2.clear();
         box2.push_back(Vector3(5.9f, -5.0f, -10.0f));
         box2.push_back(Vector3(6.1f, -7.1f, -0.9f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision in right-upper-front corner
         box2.clear();
         box2.push_back(Vector3(5.9f, -5.0f, 0.9f));
         box2.push_back(Vector3(6.1f, -7.1f, 100.0f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // collision inside a k-DOP
         box2.clear();
         box2.push_back(Vector3(5.4f, -8.5f, -0.5f));
         box2.push_back(Vector3(5.6f, -7.5f, 0.5f));
         kdop2.reset(box2);
         CPPUNIT_ASSERT(kdop1.collidesWith(kdop2));

         // NO collision (left)
         box2.clear();
         box2.push_back(Vector3(0.0f, -10.0f, -10.0f));
         box2.push_back(Vector3(4.9f, 0.0f, 10.0f));
         kdop1.reset(box2);
         CPPUNIT_ASSERT(!kdop1.collidesWith(kdop2));

         // NO collision (right)
         box2.clear();
         box2.push_back(Vector3(6.1f, -10.0f, -10.0f));
         box2.push_back(Vector3(100.0f, 0.0f, 10.0f));
         kdop1.reset(box2);
         CPPUNIT_ASSERT(!kdop1.collidesWith(kdop2));

         // NO collision (above)
         box2.clear();
         box2.push_back(Vector3(-100.0f, -6.9f, -10.0f));
         box2.push_back(Vector3(100.0f, 0.0f, 10.0f));
         kdop1.reset(box2);
         CPPUNIT_ASSERT(!kdop1.collidesWith(kdop2));

         // NO collision (above2)
         box2.clear();
         box2.push_back(Vector3(-100.0f, 6.0f, -10.0f));
         box2.push_back(Vector3(100.0f, 10.0f, 10.0f));
         kdop1.reset(box2);
         CPPUNIT_ASSERT(!kdop1.collidesWith(kdop2));

         // NO collision (below)
         box2.clear();
         box2.push_back(Vector3(-100.0f, -9.1f, -10.0f));
         box2.push_back(Vector3(100.0f, -100.0f, 10.0f));
         kdop1.reset(box2);
         CPPUNIT_ASSERT(!kdop1.collidesWith(kdop2));

         // NO collision (infront)
         box2.clear();
         box2.push_back(Vector3(-100.0f, -10.0f, 1.1f));
         box2.push_back(Vector3(100.0f, 0.0f, 10.0f));
         kdop1.reset(box2);
         CPPUNIT_ASSERT(!kdop1.collidesWith(kdop2));

         // NO collision (behind)
         box2.clear();
         box2.push_back(Vector3(-100.0f, -10.0f, -1.1f));
         box2.push_back(Vector3(100.0f, 0.0f, -10.0f));
         kdop1.reset(box2);
         CPPUNIT_ASSERT(!kdop1.collidesWith(kdop2));

     }

     // TODO: maybe also testCollideK14(), testCollideK18() and testCollideK26()
     //
     // the collision tests for these are essentially the same as for
     // 6-DOPs, so i don't think testing them separately is important.
     // it is much more important to actually test the correct construktions of
     // the k-DOPs with k!=6, which we already do.


     void KDOPTest::testResetClears() {
         std::list<Vector3> box1;
         box1.push_back(Vector3(5.0f, -9.0f, -1.0f));
         box1.push_back(Vector3(5.0f, -9.0f, 1.0f));
         box1.push_back(Vector3(5.0f, -7.0f, -1.0f));
         box1.push_back(Vector3(5.0f, -7.0f, 1.0f));
         box1.push_back(Vector3(6.0f, -9.0f, -1.0f));
         box1.push_back(Vector3(6.0f, -9.0f, 1.0f));
         box1.push_back(Vector3(6.0f, -7.0f, -1.0f));
         box1.push_back(Vector3(6.0f, -7.0f, 1.0f));

         std::list<Vector3> box2;
         box2.push_back(Vector3(-5.0f, -9.0f, -1.0f));
         box2.push_back(Vector3(-5.0f, -9.0f, 1.0f));
         box2.push_back(Vector3(-5.0f, -7.0f, -1.0f));
         box2.push_back(Vector3(-5.0f, -7.0f, 1.0f));
         box2.push_back(Vector3(-6.0f, -9.0f, -1.0f));
         box2.push_back(Vector3(-6.0f, -9.0f, 1.0f));
         box2.push_back(Vector3(-6.0f, -7.0f, -1.0f));
         box2.push_back(Vector3(-6.0f, -7.0f, 1.0f));

         Kdop kdop6(6);
         Kdop kdop14(14);
         Kdop kdop18(18);
         Kdop kdop26(26);
         kdop6.reset(box1);
         kdop14.reset(box1);
         kdop18.reset(box1);
         kdop26.reset(box1);
         kdop6.reset(box2);
         kdop14.reset(box2);
         kdop18.reset(box2);
         kdop26.reset(box2);

         // AB: these boxes clearly don't collide with each other.
         //     so if resetting clears the k-DOP properly, they should not
         //     collide anymore with a k-DOP that contains the first box only.

         Kdop referenceKdop6(6);
         Kdop referenceKdop14(14);
         Kdop referenceKdop18(18);
         Kdop referenceKdop26(26);
         referenceKdop6.reset(box1);
         referenceKdop14.reset(box1);
         referenceKdop18.reset(box1);
         referenceKdop26.reset(box1);

         CPPUNIT_ASSERT(!kdop6.collidesWith(referenceKdop6));
         CPPUNIT_ASSERT(!referenceKdop6.collidesWith(kdop6));
         CPPUNIT_ASSERT(!kdop14.collidesWith(referenceKdop14));
         CPPUNIT_ASSERT(!referenceKdop14.collidesWith(kdop14));
         CPPUNIT_ASSERT(!kdop18.collidesWith(referenceKdop18));
         CPPUNIT_ASSERT(!referenceKdop18.collidesWith(kdop18));
         CPPUNIT_ASSERT(!kdop26.collidesWith(referenceKdop26));
         CPPUNIT_ASSERT(!referenceKdop26.collidesWith(kdop26));
     }

    void KDOPTest::testSphere() {
        const real epsilon = 0.001f;
        World world;
        real radius = 20.0;
        Sphere* sphereShape = new Sphere(radius);
        Proxy* sphere = world.createProxy(sphereShape);


        // any k-DOP for a sphere must be
        // * >= the k-DOP for the largest box completely inside the sphere
        // * <= the k-DOP for the smallest box enclosing the sphere
        // * roughly equal to sphere.getMesh()
        const real innerBoxW = (real)(radius / sqrt(3.0));
        Proxy* innerBox = world.createProxy(new Box((real)(innerBoxW * 2.0), (real)(innerBoxW * 2.0), (real)(innerBoxW * 2.0)));
        Proxy* outerBox = world.createProxy(new Box((real)(radius * 2.0), (real)(radius * 2.0), (real)(radius * 2.0)));
        sphereShape->setAverageEdgeLength((real)(radius / 3.0));
        Proxy* sphereMesh = world.createProxy(sphereShape->getMesh()->cloneMesh());
        innerBox->translate(-innerBoxW, -innerBoxW, -innerBoxW);
        outerBox->translate(-radius, -radius, -radius);

        // use a non-trivial sphere (i.e. a transformed sphere)
        sphere->translate(15.0, 5.0, 10.0);
        sphereMesh->translate(15.0, 5.0, 10.0);
        innerBox->translate(15.0, 5.0, 10.0);
        outerBox->translate(15.0, 5.0, 10.0);
        // rotating a sphere should not have any effect on the outcome
        sphere->rotate(24, 1.0, 5.0, 4.0);

        std::list<int> valuesOfK;
        valuesOfK.push_back(6);
        valuesOfK.push_back(14);
        valuesOfK.push_back(18);
        valuesOfK.push_back(26);
        for (std::list<int>::const_iterator it = valuesOfK.begin(); it != valuesOfK.end(); ++it) {
            int k = *it;
            Kdop kdopSphere(k);
            Kdop kdopInnerBox(k);
            Kdop kdopOuterBox(k);
            Kdop kdopSphereMesh(k);

            kdopSphere.adjustToShape(sphere->getShape());
            kdopInnerBox.adjustToShape(innerBox->getShape());
            kdopOuterBox.adjustToShape(outerBox->getShape());
            kdopSphereMesh.adjustToShape(sphereMesh->getShape());

            // for the 6 AABB planes sphere, outerbox and mesh should be equal.
            for (int i = 0; i < 3; i++) {
                CPPUNIT_ASSERT_DOUBLES_EQUAL(kdopOuterBox.getDistanceOfPlaneToOrigin(i), kdopSphere.getDistanceOfPlaneToOrigin(i), epsilon);
                CPPUNIT_ASSERT_DOUBLES_EQUAL(kdopSphereMesh.getDistanceOfPlaneToOrigin(i), kdopSphere.getDistanceOfPlaneToOrigin(i) ,epsilon);
            }
            for (int i = k / 2; i < k / 2 + 3; i++) {
                CPPUNIT_ASSERT_DOUBLES_EQUAL(kdopOuterBox.getDistanceOfPlaneToOrigin(i), kdopSphere.getDistanceOfPlaneToOrigin(i), epsilon);
                CPPUNIT_ASSERT_DOUBLES_EQUAL(kdopSphereMesh.getDistanceOfPlaneToOrigin(i), kdopSphere.getDistanceOfPlaneToOrigin(i) ,epsilon);
            }

            for (int i = 0; i < k; i++) {
//                std::cout << "k: " << k << " i=" << i << " sphere: " << kdopSphere.getDistanceOfPlaneToOrigin(i) << std::endl;
//                std::cout << "k: " << k << " i=" << i << ": mesh: " << kdopSphereMesh.getDistanceOfPlaneToOrigin(i) << std::endl;
//                std::cout << "k: " << k << " i=" << i << ": inner: " << kdopInnerBox.getDistanceOfPlaneToOrigin(i) << std::endl;
//                std::cout << "k: " << k << " i=" << i << ": outer: " << kdopOuterBox.getDistanceOfPlaneToOrigin(i) << std::endl;

                real dSphere = kdopSphere.getDistanceOfPlaneToOrigin(i);
                real dInnerBox = kdopInnerBox.getDistanceOfPlaneToOrigin(i);
                real dOuterBox = kdopOuterBox.getDistanceOfPlaneToOrigin(i);
                real dMesh = kdopSphereMesh.getDistanceOfPlaneToOrigin(i);

                // AB: sphere and mesh are roughly equal, but only roughly, so
                // we need a larger epsilon
                // note: use sphere->setAverageEdgeLength(x) with a smaller x
                // before retrieving the mesh to
                // get a more precise mesh (however running time is larger then)
                const real epsilonSphereMesh = 0.2f;
                CPPUNIT_ASSERT_DOUBLES_EQUAL(dMesh, dSphere, epsilonSphereMesh);

                CPPUNIT_ASSERT(dOuterBox - epsilon <= dSphere);
                CPPUNIT_ASSERT(dInnerBox + epsilon >= dSphere);
            }

        }

        delete sphere;
        delete innerBox;
        delete outerBox;
        delete sphereMesh;
    }
}
/*
 * vim: et sw=4 ts=4
 */
