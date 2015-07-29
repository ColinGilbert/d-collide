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

#include "settest.h"
#include "collisionpair.h"
#include "boundingvolumes/aabb.h"

using namespace std;
using namespace dcollide;

CPPUNIT_TEST_SUITE_REGISTRATION(SetTest);

namespace dcollide {
    void SetTest::setUp(void) {
    }

    void SetTest::tearDown(void) {
    }

    // Testing real objects:
    // ====================================================
    void SetTest::testSetObjects() {

        Set<CollisionPair> set(5);
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, set.size());
        CPPUNIT_ASSERT_EQUAL(true, set.empty());
        //CPPUNIT_ASSERT(set.getElement() == 0);

        set.clear();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, set.size());
        CPPUNIT_ASSERT_EQUAL(true, set.empty());
        //CPPUNIT_ASSERT(set.getElement() == 0);

        Aabb* bv1 = new Aabb(Vector3(0,0,0),Vector3(2,2,2));
        Aabb* bv2 = new Aabb(Vector3(0,0,0),Vector3(1,1,1));
        Aabb* bv3 = new Aabb(Vector3(1,2,3),Vector3(1,1,1));
        CollisionPair cp1;
        CollisionPair cp2;
        cp1.bvol1 = bv1;
        cp1.bvol2 = bv2;
        // cp with bv's swapped (== should be true in this case!)
        cp2.bvol1 = bv2;
        cp2.bvol2 = bv1;

        // testing the == operator for cp's:
        CPPUNIT_ASSERT(cp1==cp1);
        CPPUNIT_ASSERT(cp1==cp2);
        CPPUNIT_ASSERT(cp2==cp1);

        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp1));
        CPPUNIT_ASSERT_EQUAL((unsigned int)1, set.size());
        CPPUNIT_ASSERT_EQUAL(false, set.insert(cp2));
        CPPUNIT_ASSERT_EQUAL((unsigned int)1, set.size());
        CPPUNIT_ASSERT_EQUAL(false, set.empty());

        // checking pop_back():
        cp2.bvol1 = bv2;
        cp2.bvol2 = bv3;
        CPPUNIT_ASSERT_EQUAL(false, (cp1==cp2));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp2));
        CPPUNIT_ASSERT_EQUAL((unsigned int)2, set.size());
        CPPUNIT_ASSERT_EQUAL(cp2,set.pop_back());
        CPPUNIT_ASSERT_EQUAL((unsigned int)1, set.size());
        CPPUNIT_ASSERT_EQUAL(cp1,set.pop_back());
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, set.size());
        CPPUNIT_ASSERT_EQUAL(true, set.empty());

        // checking getElement():
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp1));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp2));
        CPPUNIT_ASSERT_EQUAL(cp1,set.getElement());
        CPPUNIT_ASSERT_EQUAL((unsigned int)1,set.getPosition());
        CPPUNIT_ASSERT_EQUAL(cp2,set.getElement());
        CPPUNIT_ASSERT_EQUAL((unsigned int)2,set.getPosition());
        CPPUNIT_ASSERT_EQUAL((unsigned int)2, set.size());
        set.resetPosition();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0,set.getPosition());

        // Checking increaseDataContainer()
        CPPUNIT_ASSERT_EQUAL((int)5,set.getMaxNumber());
        CPPUNIT_ASSERT_EQUAL((int)5,set.getNumberOfHashValues());
        Aabb* bv4 = new Aabb(Vector3(0,1,0),Vector3(2,2,2));
        Aabb* bv5 = new Aabb(Vector3(0,1,0),Vector3(1,1,4));
        Aabb* bv6 = new Aabb(Vector3(1,1,3),Vector3(1,5,1));
        Aabb* bv7 = new Aabb(Vector3(1,1,3),Vector3(7,1,1));
        CollisionPair cp3;
        CollisionPair cp4;
        CollisionPair cp5;
        CollisionPair cp6;
        cp3.bvol1 = bv4;
        cp3.bvol2 = bv4;
        cp4.bvol1 = bv5;
        cp4.bvol2 = bv5;
        cp5.bvol1 = bv6;
        cp5.bvol2 = bv6;
        cp6.bvol1 = bv7;
        cp6.bvol2 = bv7;
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp3));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp4));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp5));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp6));
        CPPUNIT_ASSERT_EQUAL((int)10,set.getMaxNumber());
        CPPUNIT_ASSERT_EQUAL((int)5,set.getNumberOfHashValues());

        // Checking find():
        CollisionPair cp7;
        cp7.bvol1 = bv1;
        cp7.bvol2 = bv6;
        CPPUNIT_ASSERT_EQUAL(0, set.find(cp1));
        CPPUNIT_ASSERT_EQUAL(1, set.find(cp2));
        CPPUNIT_ASSERT_EQUAL(2, set.find(cp3));
        CPPUNIT_ASSERT_EQUAL(3, set.find(cp4));
        CPPUNIT_ASSERT_EQUAL(4, set.find(cp5));
        CPPUNIT_ASSERT_EQUAL(5, set.find(cp6));
        CPPUNIT_ASSERT_EQUAL(-1, set.find(cp7));

        // Checking getEndPosition:
        CPPUNIT_ASSERT_EQUAL(5, set.getEndPosition());

        // Checking contains():
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp1));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp2));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp3));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp4));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp5));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp6));
        CPPUNIT_ASSERT_EQUAL(false,set.contains(cp7));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp7));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp7));

        // checking iteration with getElement(i):
        CollisionPair* cps = new CollisionPair[7];
        cps[0] = cp1;
        cps[1] = cp2;
        cps[2] = cp3;
        cps[3] = cp4;
        cps[4] = cp5;
        cps[5] = cp6;
        cps[6] = cp7;
        for (int i = 0; i<=set.getEndPosition();++i) {
            CPPUNIT_ASSERT_EQUAL(cps[i],set.getElement(i));
        }

        // cleanup:
        delete bv1;
        delete bv2;
        delete bv3;
        delete bv4;
        delete bv5;
        delete bv6;
        delete bv7;
        delete[] cps;

    }

    // Testing Pointers:
    // ====================================================
    void SetTest::testSetPointers() {

        Set<CollisionPair*> set(5);
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, set.size());
        CPPUNIT_ASSERT_EQUAL(true, set.empty());
        //CPPUNIT_ASSERT(set.getElement() == 0);

        set.clear();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, set.size());
        CPPUNIT_ASSERT_EQUAL(true, set.empty());
        //CPPUNIT_ASSERT(set.getElement() == 0);

        Aabb* bv1 = new Aabb(Vector3(0,0,0),Vector3(2,2,2));
        Aabb* bv2 = new Aabb(Vector3(0,0,0),Vector3(1,1,1));
        Aabb* bv3 = new Aabb(Vector3(1,2,3),Vector3(1,1,1));
        CollisionPair* cp1 = new CollisionPair();
        CollisionPair* cp2 = new CollisionPair();
        cp1->bvol1 = bv1;
        cp1->bvol2 = bv2;
        cp2->bvol1 = bv2;
        cp2->bvol2 = bv1;

        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp1));
        CPPUNIT_ASSERT_EQUAL((unsigned int)1, set.size());
        CPPUNIT_ASSERT_EQUAL(false, set.insert(cp1));
        CPPUNIT_ASSERT_EQUAL((unsigned int)1, set.size());
        CPPUNIT_ASSERT_EQUAL(false, set.empty());

        // checking pop_back():
        cp2->bvol1 = bv2;
        cp2->bvol2 = bv3;
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp2));
        CPPUNIT_ASSERT_EQUAL((unsigned int)2, set.size());
        CPPUNIT_ASSERT_EQUAL(cp2,set.pop_back());
        CPPUNIT_ASSERT_EQUAL((unsigned int)1, set.size());
        CPPUNIT_ASSERT_EQUAL(cp1,set.pop_back());
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, set.size());
        CPPUNIT_ASSERT_EQUAL(true, set.empty());

        // checking getElement():
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp1));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp2));
        CPPUNIT_ASSERT_EQUAL(cp1,set.getElement());
        CPPUNIT_ASSERT_EQUAL((unsigned int)1,set.getPosition());
        CPPUNIT_ASSERT_EQUAL(cp2,set.getElement());
        CPPUNIT_ASSERT_EQUAL((unsigned int)2,set.getPosition());
        CPPUNIT_ASSERT_EQUAL((unsigned int)2, set.size());
        set.resetPosition();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0,set.getPosition());

        // Checking increaseDataContainer()
        CPPUNIT_ASSERT_EQUAL((int)5,set.getMaxNumber());
        CPPUNIT_ASSERT_EQUAL((int)5,set.getNumberOfHashValues());
        Aabb* bv4 = new Aabb(Vector3(0,1,0),Vector3(2,2,2));
        Aabb* bv5 = new Aabb(Vector3(0,1,0),Vector3(1,1,4));
        Aabb* bv6 = new Aabb(Vector3(1,1,3),Vector3(1,5,1));
        Aabb* bv7 = new Aabb(Vector3(1,1,3),Vector3(7,1,1));
        CollisionPair* cp3 = new CollisionPair();
        CollisionPair* cp4 = new CollisionPair();
        CollisionPair* cp5 = new CollisionPair();
        CollisionPair* cp6 = new CollisionPair();
        cp3->bvol1 = bv4;
        cp3->bvol2 = bv4;
        cp4->bvol1 = bv5;
        cp4->bvol2 = bv5;
        cp5->bvol1 = bv6;
        cp5->bvol2 = bv6;
        cp6->bvol1 = bv7;
        cp6->bvol2 = bv7;
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp3));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp4));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp5));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp6));
        CPPUNIT_ASSERT_EQUAL((int)10,set.getMaxNumber());
        CPPUNIT_ASSERT_EQUAL((int)5,set.getNumberOfHashValues());

        // Checking find():
        CollisionPair* cp7 = new CollisionPair();
        cp7->bvol1 = bv1;
        cp7->bvol2 = bv6;
        CPPUNIT_ASSERT_EQUAL(0, set.find(cp1));
        CPPUNIT_ASSERT_EQUAL(1, set.find(cp2));
        CPPUNIT_ASSERT_EQUAL(2, set.find(cp3));
        CPPUNIT_ASSERT_EQUAL(3, set.find(cp4));
        CPPUNIT_ASSERT_EQUAL(4, set.find(cp5));
        CPPUNIT_ASSERT_EQUAL(5, set.find(cp6));
        CPPUNIT_ASSERT_EQUAL(-1, set.find(cp7));

        // Checking getEndPosition:
        CPPUNIT_ASSERT_EQUAL(5, set.getEndPosition());

        // Checking contains():
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp1));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp2));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp3));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp4));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp5));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp6));
        CPPUNIT_ASSERT_EQUAL(false,set.contains(cp7));
        CPPUNIT_ASSERT_EQUAL(true, set.insert(cp7));
        CPPUNIT_ASSERT_EQUAL(true,set.contains(cp7));

        // cleanup:
        delete cp1;
        delete cp2;
        delete cp3;
        delete cp4;
        delete cp5;
        delete cp6;
        delete cp7;
        delete bv1;
        delete bv2;
        delete bv3;
        delete bv4;
        delete bv5;
        delete bv6;
        delete bv7;
    }
}
/*
 * vim: et sw=4 ts=4
 */
