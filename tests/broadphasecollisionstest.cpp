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

#include "broadphasecollisionstest.h"

#include "shapes/box.h"
#include "world.h"
#include "worldparameters.h"
#include "math/vector.h"
#include "real.h"
#include "broadphase/broadphase.h"
#include "broadphase/broadphasecollisions.h"
#include "worldcollisions.h"
#include "collisioninfo.h"
#include "math/vector.h"
#include "timing.h"
#include "proxy.h"
#include "boundingvolumes/boundingvolume.h"
#include "bvhnode.h"
#include "collisionpair.h"
#include <debugstream.h>

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <math.h>
#include <iostream>

//-------using directives--------------
using namespace std;
using namespace dcollide;

// AB: no need to register this suite, since this class is only a base class.
//     derived classes should implement themselves.
//CPPUNIT_TEST_SUITE_REGISTRATION (BroadPhaseCollisionsTest);

namespace dcollide {
    BroadPhaseCollisionsTest::BroadPhaseCollisionsTest() {
        mBroadPhaseType = BROADPHASE_TYPE_HIERARCHICALGRID;
    }

    /*!
     *  \brief resource allocation for unit test
     */
    void BroadPhaseCollisionsTest::setUp(void) {
        // AB: default value. this should not be used at all, since derived
        // classes should implement their own setUp().
        mBroadPhaseType = BROADPHASE_TYPE_HIERARCHICALGRID;

        std::cerr << dc_funcinfo << "should not be called! -> derived classes should implement their own setUp() function!" << std::endl;
    }

    /*!
     *  \brief resource deallocation for unit test
     */
    void BroadPhaseCollisionsTest::tearDown(void) {
        //Destructor for Unit Tests not needed here
        //use this to free the resources allocated in setUp()
    }

    void BroadPhaseCollisionsTest::testGrow() {
        WorldParameters parameters;
        parameters.setBroadPhaseType(mBroadPhaseType);
        parameters.setWorldMinMax(Vector3(-4, 1, -4), Vector3(4, 4, 4));
        World world(parameters);
        world.prepareSimulation();

        //test if the world increases its size properly
        //First box has a x-position that is too large and outside the world
        Proxy* box1 = world.createProxy(new Box(1, 1, 1));
        box1->translate(Vector3(10, 0, 0));
        world.addProxy(box1);

        //World should have grown in X-direction to contain the box
        CPPUNIT_ASSERT_EQUAL(Vector3(16, 4, 4), world.getWorldMax());

        // FIXME: this test fails until the world growth quickfix is worked out
        // world should not have grown in the direction of the minimum boundary
        //CPPUNIT_ASSERT_EQUAL(Vector3(-4, 1, -4), world.getWorldMin());

        // second box is used to test proper growth of a world with positive
        // minimum boundary
        Proxy* box2 = world.createProxy(new Box(1, 1, 1));
        box2->translate(Vector3(0, -3, 0));
        world.addProxy(box2);

        // world should not have grown in the direction of the maximum boundary
        CPPUNIT_ASSERT_EQUAL(Vector3(16, 4, 4), world.getWorldMax());

        //World should have grown in y-direction to contain the box
        CPPUNIT_ASSERT_EQUAL(Vector3(-4, -4, -4), world.getWorldMin());
    }

    void BroadPhaseCollisionsTest::testMain() {
        WorldParameters parameters;
        parameters.setBroadPhaseType(mBroadPhaseType);
        parameters.setWorldDimension(dcollide::Vector3(10000.0f,10000.0f,10500.0f));

        int numberOfProxiesAdded = 0;

        /*
         * ---------------------------------------------------------------------------------------
         * Here we generate two objects we definitely know, to test if the calculation is correct.
         * ---------------------------------------------------------------------------------------
         * Testing two boxes with positive coordinates:
         *
         *      Box_1                   Box_2
         *                   (2,3,2)                 (3,4,3)
         *            7------8                7------8
         *           /|     /|               /|     /|
         *          3-+---4/ |              3-+---4/ |
         *          | 6---|--5              | 6---|--5
         *          |/    | /               |/    | /
         *          1-----2                 1-----2
         *   (0,0,0)                  (1,1,1)
         *
         * Therefore the Volume of Interest must range from (1,1,1) to (2,3,2)
         */

        // create a new world to test the application
        dcollide::World* w1 = new dcollide::World(parameters);
        w1->prepareSimulation();
        dcollide::BroadPhase* testBroadphase = w1->getBroadPhase();

            // Adding first proxy
            dcollide::Proxy* testProxy1 = w1->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)3, (dcollide::real)2)));
            dcollide::Matrix testMatrix1;
            testMatrix1.translate(dcollide::Vector3((dcollide::real)0, (dcollide::real)0, (dcollide::real)0));
            testProxy1->setTransformation(testMatrix1);
            w1->addProxy(testProxy1);
            ++numberOfProxiesAdded;

            // Getting the boundingvolume to test with
            const BvhNode* testBvn1 = testProxy1->getBvHierarchyNode();
            const dcollide::BoundingVolume* testBv1;
            testBv1 = testBvn1->getBoundingVolume();

            dcollide::Vector3 firstVecTestProxy1 = testBv1->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestProxy1 = testBv1->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestProxy1 << std::endl;
            // std::cout << "Max" << secondVecTestProxy1 << std::endl;

            // Adding second proxy
            dcollide::Proxy* testProxy2 = w1->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)3, (dcollide::real)2)));
            dcollide::Matrix testMatrix2;
            testMatrix2.translate(dcollide::Vector3((dcollide::real)1, (dcollide::real)1, (dcollide::real)1));
            testProxy2->setTransformation(testMatrix2);
            w1->addProxy(testProxy2);
            ++numberOfProxiesAdded;

            // Again, getting the boundingvolume
            const BvhNode* testBvn2 = testProxy2->getBvHierarchyNode();
            const dcollide::BoundingVolume* testBv2;
            testBv2 = testBvn2->getBoundingVolume();

            dcollide::Vector3 firstVecTestProxy2 = testBv2->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestProxy2 = testBv2->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestProxy2 << std::endl;
            // std::cout << "Max" << secondVecTestProxy2 << std::endl;

            // test if the bounding boxes collide
            CPPUNIT_ASSERT (testBv1->collidesWith(*testBv2));
            CPPUNIT_ASSERT (testBv2->collidesWith(*testBv1));

             // Getting the collsions
            dcollide::WorldCollisions wc =  w1->calculateAllCollisions();
            const dcollide::BroadPhaseCollisions* bvc = wc.getBroadPhaseCollisions();

            // Inserting the testpair
            dcollide::CollisionPair colPair = CollisionPair();
            colPair.bvol1 = testBv1;
            colPair.bvol2 = testBv2;

            std::list<dcollide::CollisionPair> testList;
            testList.push_back(colPair);

            // Getting the collisionpairs of the broadphase
            const std::list<CollisionPair>& broadPhaseCollisionPairs =
                    bvc->getResults();
            int number = broadPhaseCollisionPairs.size();
            // std::cout << "Number of collisionpairs: " << number << std::endl;

            dcollide::CollisionPair pairBrPh = *broadPhaseCollisionPairs.begin();
            dcollide::CollisionPair pairTestList = *testList.begin();

            const dcollide::BoundingVolume* bvol1BrPh = pairBrPh.bvol1;
            const dcollide::BoundingVolume* bvol2BrPh = pairBrPh.bvol2;
            const dcollide::BoundingVolume* bvol1TestSet = pairTestList.bvol1;
            const dcollide::BoundingVolume* bvol2TestSet = pairTestList.bvol2;
            
            if (   !((*bvol1BrPh == *bvol1TestSet) && (*bvol2BrPh == *bvol2TestSet))
                && !((*bvol2BrPh == *bvol1TestSet) && (*bvol1BrPh == *bvol2TestSet))) {
                
                std::stringstream ss;
                
                ss  << "The returned bounding volume pair ("
                    << *bvol1BrPh
                    << ", "
                    << *bvol2BrPh
                    << ") isn't the expected one!"
                    << std::endl
                    << "We'd expected the pair ("
                    << *bvol1TestSet
                    << ", "
                    << *bvol2TestSet
                    << ").";
                
                CPPUNIT_FAIL(ss.str());
            }
            
            dcollide::Proxy* proxy1BrPh = bvol1BrPh->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy2BrPh = bvol2BrPh->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy1TestSet = bvol1TestSet->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy2TestSet = bvol2TestSet->getHierarchyNode()->getProxy();

            CPPUNIT_ASSERT_EQUAL (proxy1BrPh, proxy1TestSet);
            CPPUNIT_ASSERT_EQUAL (proxy2BrPh, proxy2TestSet);

        delete w1;

         /*
         * -----------------------------------------------------------------------------------------------
         * Here we generate another two objects we definitely know, to test if the calculation is correct.
         * -----------------------------------------------------------------------------------------------
         * Testing with a box having negative coordinates:
         *
         *      Box_1                   Box_2
         *                   (1,2,1)                 (0,1,0)
         *            7------8                7------8
         *           /|     /|               /|     /|
         *          3-+---4/ |              3-+---4/ |
         *          | 6---|--5              | 6---|--5
         *          |/    | /               |/    | /
         *          1-----2                 1-----2
         *  (-1,-1,-1)              (-2,-2,-2)
         *
         * Therefore the Volume of Interest must range from (-1,-1,-1) to (0,1,0)
         */

        // create a new world to test the application
        dcollide::World* w2 = new dcollide::World(parameters);
        dcollide::BroadPhase* testBroadphase2 = w2->getBroadPhase();

            dcollide::Proxy* testTwoProxy1 = w2->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)3, (dcollide::real)2)));
            dcollide::Matrix testTwoMatrix1;
            testTwoMatrix1.translate(dcollide::Vector3((dcollide::real)-2, (dcollide::real)-2, (dcollide::real)-2));
            testTwoProxy1->setTransformation(testTwoMatrix1);
            w2->addProxy(testTwoProxy1);
            ++numberOfProxiesAdded;

            const BvhNode* testTwoBvn1 = testTwoProxy1->getBvHierarchyNode();
            const dcollide::BoundingVolume* testTwoBv1;
            testTwoBv1 = testTwoBvn1->getBoundingVolume();

            dcollide::Vector3 firstVecTestTwoProxy1 = testTwoBv1->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestTwoProxy1 = testTwoBv1->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestTwoProxy1 << std::endl;
            // std::cout << "Max" << secondVecTestTwoProxy1 << std::endl;

            dcollide::Proxy* testTwoProxy2 = w2->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)3, (dcollide::real)2)));
            dcollide::Matrix testTwoMatrix2;
            testTwoMatrix2.translate(dcollide::Vector3((dcollide::real)-1, (dcollide::real)-1, (dcollide::real)-1));
            testTwoProxy2->setTransformation(testTwoMatrix2);
            w2->addProxy(testTwoProxy2);
            ++numberOfProxiesAdded;

            const BvhNode* testTwoBvn2 = testTwoProxy2->getBvHierarchyNode();
            const dcollide::BoundingVolume* testTwoBv2;
            testTwoBv2 = testTwoBvn2->getBoundingVolume();

            dcollide::Vector3 firstVecTestTwoProxy2 = testTwoBv2->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestTwoProxy2 = testTwoBv2->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestTwoProxy2 << std::endl;
            // std::cout << "Max" << secondVecTestTwoProxy2 << std::endl;

            CPPUNIT_ASSERT (testTwoBv1->collidesWith(*testTwoBv2));
            CPPUNIT_ASSERT (testTwoBv2->collidesWith(*testTwoBv1));

            // Getting the collsions
            w2->prepareSimulation();
            dcollide::WorldCollisions wc2 =  w2->calculateAllCollisions();
            const dcollide::BroadPhaseCollisions* bvc2 = wc2.getBroadPhaseCollisions();

            // Inserting the testpair
            dcollide::CollisionPair colPair2 = CollisionPair();
            colPair2.bvol1 = testTwoBv1;
            colPair2.bvol2 = testTwoBv2;

            std::list<dcollide::CollisionPair> testList2;
            testList2.push_back(colPair2);

            // Getting the collisionpairs of the broadphase
            const std::list<CollisionPair>& broadPhaseCollisionPairs2 = bvc2->getResults();
            int nr = broadPhaseCollisionPairs2.size();
            // std::cout << "Number of collisionpairs: " << nr << std::endl;

            dcollide::CollisionPair pairBrPh2 = *broadPhaseCollisionPairs2.begin();
            dcollide::CollisionPair pairTestList2 = *testList2.begin();

            const dcollide::BoundingVolume* bvol1BrPh2 = pairBrPh2.bvol1;
            const dcollide::BoundingVolume* bvol2BrPh2 = pairBrPh2.bvol2;
            const dcollide::BoundingVolume* bvol1TestSet2 = pairTestList2.bvol1;
            const dcollide::BoundingVolume* bvol2TestSet2 = pairTestList2.bvol2;

            CPPUNIT_ASSERT_EQUAL (bvol1BrPh2, bvol1TestSet2);
            CPPUNIT_ASSERT_EQUAL (bvol2BrPh2, bvol2TestSet2);

            dcollide::Proxy* proxy1BrPh2 = bvol1BrPh2->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy2BrPh2 = bvol2BrPh2->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy1TestSet2 = bvol1TestSet2->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy2TestSet2 = bvol2TestSet2->getHierarchyNode()->getProxy();

            CPPUNIT_ASSERT_EQUAL (proxy1BrPh2, proxy1TestSet2);
            CPPUNIT_ASSERT_EQUAL (proxy2BrPh2, proxy2TestSet2);

        delete w2;

         /*
         * -----------------------------------------------------------------------------------------------
         * Here we generate another two objects we definitely know, to test if the calculation is correct.
         * -----------------------------------------------------------------------------------------------
         * Testing a box with itself:
         *
         *      Box_1                   Box_2
         *                   (3,3,4)                 (3,3,4)
         *            7------8                7------8
         *           /|     /|               /|     /|
         *          3-+---4/ |              3-+---4/ |
         *          | 6---|--5              | 6---|--5
         *          |/    | /               |/    | /
         *          1-----2                 1-----2
         *  (1,2,3)                 (1,2,3)
         *
         * Therefore the Volume of Interest must range from (1,2,3) to (3,3,4)
         */

        // create a new world to test the application
        dcollide::World* w3 = new dcollide::World(parameters);
        dcollide::BroadPhase* testBroadphase3 = w3->getBroadPhase();

            dcollide::Proxy* testThreeProxy1 = w3->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)1, (dcollide::real)1)));
            dcollide::Matrix testThreeMatrix1;
            testThreeMatrix1.translate(dcollide::Vector3((dcollide::real)1, (dcollide::real)2, (dcollide::real)3));
            testThreeProxy1->setTransformation(testThreeMatrix1);
            w3->addProxy(testThreeProxy1);
            ++numberOfProxiesAdded;

            const BvhNode* testThreeBvn1 = testThreeProxy1->getBvHierarchyNode();
            const dcollide::BoundingVolume* testThreeBv1;
            testThreeBv1 = testThreeBvn1->getBoundingVolume();

            dcollide::Vector3 firstVecTestThreeProxy1 = testThreeBv1->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestThreeProxy1 = testThreeBv1->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestThreeProxy1 << std::endl;
            // std::cout << "Max" << secondVecTestThreeProxy1 << std::endl;

            dcollide::Proxy* testThreeProxy2 = w3->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)1, (dcollide::real)1)));
            dcollide::Matrix testThreeMatrix2;
            testThreeMatrix2.translate(dcollide::Vector3((dcollide::real)1, (dcollide::real)2, (dcollide::real)3));
            testThreeProxy2->setTransformation(testThreeMatrix2);
            w3->addProxy(testThreeProxy2);
            ++numberOfProxiesAdded;

            const BvhNode* testThreeBvn2 = testThreeProxy2->getBvHierarchyNode();
            const dcollide::BoundingVolume* testThreeBv2;
            testThreeBv2 = testThreeBvn2->getBoundingVolume();

            dcollide::Vector3 firstVecTestThreeProxy2 = testThreeBv2->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestThreeProxy2 = testThreeBv2->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestThreeProxy2 << std::endl;
            // std::cout << "Max" << secondVecTestThreeProxy2 << std::endl;

            CPPUNIT_ASSERT (testThreeBv1->collidesWith(*testThreeBv2));
            CPPUNIT_ASSERT (testThreeBv2->collidesWith(*testThreeBv1));

            // Getting the collsions
            w3->prepareSimulation();
            dcollide::WorldCollisions wc3 =  w3->calculateAllCollisions();
            const dcollide::BroadPhaseCollisions* bvc3 = wc3.getBroadPhaseCollisions();

            // Inserting the testpair
            dcollide::CollisionPair colPair3 = CollisionPair();
            colPair3.bvol1 = testThreeBv1;
            colPair3.bvol2 = testThreeBv2;

            std::list<dcollide::CollisionPair> testList3;
            testList3.push_back(colPair3);

            // Getting the collisionpairs of the broadphase
            const std::list<CollisionPair>& broadPhaseCollisionPairs3 = bvc3->getResults();
            int nr_3 = broadPhaseCollisionPairs3.size();
            // std::cout << "Number of collisionpairs: " << nr_3 << std::endl;

            dcollide::CollisionPair pairBrPh3 = *broadPhaseCollisionPairs3.begin();
            dcollide::CollisionPair pairTestList3 = *testList3.begin();

            const dcollide::BoundingVolume* bvol1BrPh3 = pairBrPh3.bvol1;
            const dcollide::BoundingVolume* bvol2BrPh3 = pairBrPh3.bvol2;
            const dcollide::BoundingVolume* bvol1TestSet3 = pairTestList3.bvol1;
            const dcollide::BoundingVolume* bvol2TestSet3 = pairTestList3.bvol2;

            CPPUNIT_ASSERT_EQUAL (bvol1BrPh3, bvol1TestSet3);
            CPPUNIT_ASSERT_EQUAL (bvol2BrPh3, bvol2TestSet3);

            dcollide::Proxy* proxy1BrPh3 = bvol1BrPh3->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy2BrPh3 = bvol2BrPh3->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy1TestSet3 = bvol1TestSet3->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy2TestSet3 = bvol2TestSet3->getHierarchyNode()->getProxy();

            CPPUNIT_ASSERT_EQUAL (proxy1BrPh3, proxy1TestSet3);
            CPPUNIT_ASSERT_EQUAL (proxy2BrPh3, proxy2TestSet3);

        delete w3;

         /*
         * -----------------------------------------------------------------------------------------------
         * Here we generate another two objects we definitely know, to test if the calculation is correct.
         * -----------------------------------------------------------------------------------------------
         * Two boxes which cannot collide:
         *
         *      Box_1                   Box_2
         *                   (3,3,3)                 (12,22,32)
         *            7------8                7------8
         *           /|     /|               /|     /|
         *          3-+---4/ |              3-+---4/ |
         *          | 6---|--5              | 6---|--5
         *          |/    | /               |/    | /
         *          1-----2                 1-----2
         *  (1,1,1)                 (10,20,30)
         *
         * Therefore the Volume of Interest does not exist!
         */

        // create a new world to test the application
        dcollide::World* w4 = new dcollide::World(parameters);
        dcollide::BroadPhase* testBroadphase4 = w4->getBroadPhase();

            dcollide::Proxy* testFourProxy1 = w4->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)2, (dcollide::real)2)));
            dcollide::Matrix testFourMatrix1;
            testFourMatrix1.translate(dcollide::Vector3((dcollide::real)1, (dcollide::real)1, (dcollide::real)1));
            testFourProxy1->setTransformation(testFourMatrix1);
            w4->addProxy(testFourProxy1);
            ++numberOfProxiesAdded;

            const BvhNode* testFourBvn1 = testFourProxy1->getBvHierarchyNode();
            const dcollide::BoundingVolume* testFourBv1;
            testFourBv1 = testFourBvn1->getBoundingVolume();

            dcollide::Vector3 firstVecTestFourProxy1 = testFourBv1->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestFourProxy1 = testFourBv1->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestFourProxy1 << std::endl;
            // std::cout << "Max" << secondVecTestFourProxy1 << std::endl;

            dcollide::Proxy* testFourProxy2 = w4->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)2, (dcollide::real)2)));
            dcollide::Matrix testFourMatrix2;
            testFourMatrix2.translate(dcollide::Vector3((dcollide::real)10, (dcollide::real)20, (dcollide::real)30));
            testFourProxy2->setTransformation(testFourMatrix2);
            w4->addProxy(testFourProxy2);
            ++numberOfProxiesAdded;

            const BvhNode* testFourBvn2 = testFourProxy2->getBvHierarchyNode();
            const dcollide::BoundingVolume* testFourBv2;
            testFourBv2 = testFourBvn2->getBoundingVolume();

            dcollide::Vector3 firstVecTestFourProxy2 = testFourBv2->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestFourProxy2 = testFourBv2->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestFourProxy2 << std::endl;
            // std::cout << "Max" << secondVecTestFourProxy2 << std::endl;

            bool collisionFourFirst = testFourBv1->collidesWith(*testFourBv2);
            // std::cout << collisionFourFirst << std::endl;
            CPPUNIT_ASSERT_EQUAL (false, testFourBv1->collidesWith(*testFourBv2));
            bool collisionFourSecond = testFourBv2->collidesWith(*testFourBv1);
            // std::cout << collisionFourSecond << std::endl;
            CPPUNIT_ASSERT_EQUAL (false, testFourBv2->collidesWith(*testFourBv1));

            if (collisionFourFirst == true && collisionFourSecond == true){

                // Getting the collsions
                w4->prepareSimulation();
                dcollide::WorldCollisions wc4 =  w4->calculateAllCollisions();
                const dcollide::BroadPhaseCollisions* bvc4 = wc4.getBroadPhaseCollisions();

                // Inserting the testpair
                dcollide::CollisionPair colPair4 = CollisionPair();
                colPair4.bvol1 = testFourBv1;
                colPair4.bvol2 = testFourBv2;

                std::list<dcollide::CollisionPair> testList4;
                testList4.push_back(colPair4);

                // Getting the collisionpairs of the broadphase
                const std::list<CollisionPair>& broadPhaseCollisionPairs4 = bvc4->getResults();
                int nr_4 = broadPhaseCollisionPairs4.size();
                // std::cout << "Number of collisionpairs: " << nr_4 << std::endl;

                dcollide::CollisionPair pairBrPh4 = *broadPhaseCollisionPairs4.begin();
                dcollide::CollisionPair pairTestList4 = *testList4.begin();

                const dcollide::BoundingVolume* bvol1BrPh4 = pairBrPh4.bvol1;
                const dcollide::BoundingVolume* bvol2BrPh4 = pairBrPh4.bvol2;
                const dcollide::BoundingVolume* bvol1TestSet4 = pairTestList4.bvol1;
                const dcollide::BoundingVolume* bvol2TestSet4 = pairTestList4.bvol2;

                CPPUNIT_ASSERT_EQUAL (bvol1BrPh4, bvol1TestSet4);
                CPPUNIT_ASSERT_EQUAL (bvol2BrPh4, bvol2TestSet4);

                dcollide::Proxy* proxy1BrPh4 = bvol1BrPh4->getHierarchyNode()->getProxy();
                dcollide::Proxy* proxy2BrPh4 = bvol2BrPh4->getHierarchyNode()->getProxy();
                dcollide::Proxy* proxy1TestSet4 = bvol1TestSet4->getHierarchyNode()->getProxy();
                dcollide::Proxy* proxy2TestSet4 = bvol2TestSet4->getHierarchyNode()->getProxy();

                CPPUNIT_ASSERT_EQUAL (proxy1BrPh4, proxy1TestSet4);
                CPPUNIT_ASSERT_EQUAL (proxy2BrPh4, proxy2TestSet4);

            }

        delete w4;

         /*
         * -----------------------------------------------------------------------------------------------
         * Here we generate another two objects we definitely know, to test if the calculation is correct.
         * -----------------------------------------------------------------------------------------------
         * Two boxes with one equal face:
         *
         *      Box_1                   Box_2
         *                   (3,3,3)                 (5,3,3)
         *            7------8                7------8
         *           /|     /|               /|     /|
         *          3-+---4/ |              3-+---4/ |
         *          | 6---|--5              | 6---|--5
         *          |/    | /               |/    | /
         *          1-----2                 1-----2
         *  (0,0,0)                 (3,1,1)
         *
         *
         *      Box_1_2
         *               (3,3,3)    (5,3,3)
         *           10-----11-----12
         *           /|     /|     /|
         *          4-+---5/-+---6/ |
         *          | 7---|--8---|--9
         *          |/    | /    | /
         *          1-----2------3
         *  (0,0,0)    (3,1,1)
         *
         *
         * Therefore the Volume of Interest actualy is no Volume!
         * But the correct values are (3,1,1) and (3,3,3)
         */

        // create a new world to test the application
        dcollide::World* w5 = new dcollide::World(parameters);
        dcollide::BroadPhase* testBroadphase5 = w5->getBroadPhase();

            dcollide::Proxy* testFiveProxy1 = w5->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)2, (dcollide::real)2)));
            dcollide::Matrix testFiveMatrix1;
            testFiveMatrix1.translate(dcollide::Vector3((dcollide::real)1, (dcollide::real)1, (dcollide::real)1));
            testFiveProxy1->setTransformation(testFiveMatrix1);
            w5->addProxy(testFiveProxy1);
            ++numberOfProxiesAdded;

            const BvhNode* testFiveBvn1 = testFiveProxy1->getBvHierarchyNode();
            const dcollide::BoundingVolume* testFiveBv1;
            testFiveBv1 = testFiveBvn1->getBoundingVolume();

            dcollide::Vector3 firstVecTestFiveProxy1 = testFiveBv1->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestFiveProxy1 = testFiveBv1->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestFiveProxy1 << std::endl;
            // std::cout << "Max" << secondVecTestFiveProxy1 << std::endl;

            dcollide::Proxy* testFiveProxy2 = w5->createProxy(new dcollide::Box(
                dcollide::Vector3((dcollide::real)2, (dcollide::real)2, (dcollide::real)2)));
            dcollide::Matrix testFiveMatrix2;
            testFiveMatrix2.translate(dcollide::Vector3((dcollide::real)3, (dcollide::real)1, (dcollide::real)1));
            testFiveProxy2->setTransformation(testFiveMatrix2);
            w5->addProxy(testFiveProxy2);
            ++numberOfProxiesAdded;

            const BvhNode* testFiveBvn2 = testFiveProxy2->getBvHierarchyNode();
            const dcollide::BoundingVolume* testFiveBv2;
            testFiveBv2 = testFiveBvn2->getBoundingVolume();

            dcollide::Vector3 firstVecTestFiveProxy2 = testFiveBv2->getSurroundingAabbMin();
            dcollide::Vector3 secondVecTestFiveProxy2 = testFiveBv2->getSurroundingAabbMax();
            // std::cout << "Min" << firstVecTestFiveProxy2 << std::endl;
            // std::cout << "Max" << secondVecTestFiveProxy2 << std::endl;

            CPPUNIT_ASSERT (testFiveBv1->collidesWith(*testFiveBv2));
            CPPUNIT_ASSERT (testFiveBv2->collidesWith(*testFiveBv1));

            // Getting the collsions
            w5->prepareSimulation();
            dcollide::WorldCollisions wc5 =  w5->calculateAllCollisions();
            const dcollide::BroadPhaseCollisions* bvc5 = wc5.getBroadPhaseCollisions();

            // Inserting the testpair
            dcollide::CollisionPair colPair5 = CollisionPair();
            colPair5.bvol1 = testFiveBv1;
            colPair5.bvol2 = testFiveBv2;

            std::list<dcollide::CollisionPair> testList5;
            testList5.push_back(colPair5);

            const std::list<CollisionPair>& broadPhaseCollisionPairs5 = bvc5->getResults();
            int nr_5 = broadPhaseCollisionPairs5.size();
            // std::cout << "Number of collisionpairs: " << nr_5 << std::endl;

            dcollide::CollisionPair pairBrPh5 = *broadPhaseCollisionPairs5.begin();
            dcollide::CollisionPair pairTestList5 = *testList5.begin();

            const dcollide::BoundingVolume* bvol1BrPh5 = pairBrPh5.bvol1;
            const dcollide::BoundingVolume* bvol2BrPh5 = pairBrPh5.bvol2;
            const dcollide::BoundingVolume* bvol1TestSet5 = pairTestList5.bvol1;
            const dcollide::BoundingVolume* bvol2TestSet5 = pairTestList5.bvol2;

            CPPUNIT_ASSERT_EQUAL (bvol1BrPh5, bvol1TestSet5);
            CPPUNIT_ASSERT_EQUAL (bvol2BrPh5, bvol2TestSet5);

            dcollide::Proxy* proxy1BrPh5 = bvol1BrPh5->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy2BrPh5 = bvol2BrPh5->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy1TestSet5 = bvol1TestSet5->getHierarchyNode()->getProxy();
            dcollide::Proxy* proxy2TestSet5 = bvol2TestSet5->getHierarchyNode()->getProxy();

            CPPUNIT_ASSERT_EQUAL (proxy1BrPh5, proxy1TestSet5);
            CPPUNIT_ASSERT_EQUAL (proxy2BrPh5, proxy2TestSet5);

        delete w5;
    }

}
/*
 * vim: et sw=4 ts=4
 */
