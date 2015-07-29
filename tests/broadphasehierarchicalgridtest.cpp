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

#include "broadphasehierarchicalgridtest.h"

#include <world.h>
#include <worldparameters.h>
#include <broadphase/broadphasehierarchicalgrid.h>
#include <shapes/shapes.h>
#include <proxy.h>

#include <cppunit/extensions/HelperMacros.h>

#include <math.h>
#include <iostream>

using namespace std;
using namespace dcollide;

CPPUNIT_TEST_SUITE_REGISTRATION(BroadPhaseHierarchicalGridTest);

namespace dcollide {
    void BroadPhaseHierarchicalGridTest::setUp(void) {
        mBroadPhaseType = BROADPHASE_TYPE_HIERARCHICALGRID;
    }

    void BroadPhaseHierarchicalGridTest::tearDown(void) {
    }

    void BroadPhaseHierarchicalGridTest::testCreation() {
        WorldParameters parameters;
        parameters.setBroadPhaseType(mBroadPhaseType);
        World world(parameters);
        world.prepareSimulation();

        // test if setBroadPhaseType(mBroadPhaseType) actually created a
        // hierarchical grid broadphase
        BroadPhaseHierarchicalGrid* hierarchicalGrid =
                dynamic_cast<BroadPhaseHierarchicalGrid*>
                (world.getBroadPhase());
        CPPUNIT_ASSERT(hierarchicalGrid != 0);
    }

    // check whether the grid remains correct after adding/removing/moving
    // a single proxy
    void BroadPhaseHierarchicalGridTest::testTreeCorrectnessOneProxy() {
        const real worldSize = 1000.0;
        const real boxSize = 10.0;
        WorldParameters parameters;
        parameters.setBroadPhaseType(mBroadPhaseType);
        parameters.setWorldMinMax(Vector3(-worldSize, -worldSize, -worldSize), Vector3(worldSize, worldSize, worldSize));
        World world(parameters);

        BroadPhaseHierarchicalGrid* broadPhase =
                dynamic_cast<BroadPhaseHierarchicalGrid*>
                (world.getBroadPhase());
        CPPUNIT_ASSERT(broadPhase != 0);

        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        Proxy* firstBox = world.createProxy(new Box(boxSize, boxSize, boxSize));

        // adding the proxy should keep the tree valid.
        // note: adding does not have to have any effect at all before
        // prepareSimulation(), so we can check the tree only once
        // prepareSimulation() was called.
        world.addProxy(firstBox);
        world.prepareSimulation();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        world.removeProxy(firstBox);
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // test adding after prepareSimulation() was called
        world.addProxy(firstBox);
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // (very) small movements
        firstBox->translate(10, 0, 0);
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // back to origin
        firstBox->setTransformation(Matrix());
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // movement near to the border of the world
        firstBox->translate((real)(worldSize - boxSize - 5.0), (real)(worldSize - boxSize - 5.0), (real)(worldSize - boxSize - 5.0));

        // movement near to the _other_ border of the world
        // -> the proxy should definitely go to a different grid now
        firstBox->translate((real)(-(worldSize - boxSize - 5.0)), (real)(-(worldSize - boxSize - 5.0)), (real)(-(worldSize - boxSize - 5.0)));
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // a lot of movements
        const real stepSize = 50.0;
        for (real position = (real)(-(worldSize - boxSize - 5.0)); position < (real)((worldSize - boxSize - 5.0)); position += stepSize) {
            firstBox->translate(stepSize, stepSize, stepSize);
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // a lot of movements again, each followed by a call to update()
        for (real position = (real)(worldSize - boxSize - 5.0); position > (real)(-(worldSize - boxSize - 5.0)); position -= stepSize) {
            firstBox->translate(-stepSize, -stepSize, -stepSize);
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
            broadPhase->update();
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }

        world.removeProxy(firstBox);
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // update without a proxy in the tree
        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // re-add the proxy at a position != origin
        firstBox->setTransformation(Matrix());
        firstBox->translate((real)(-(worldSize - boxSize - 5.0)), (real)(-(worldSize - boxSize - 5.0)), (real)(-(worldSize - boxSize - 5.0)));
        world.addProxy(firstBox);
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // moving (a little) beyond the worldsize
        firstBox->translate((real)(-(boxSize + 20.0)), (real)(-(boxSize + 20.0)), (real)(-(boxSize + 20.0)));
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // moving (a lot) beyond the worldsize
        firstBox->translate((real)(-worldSize * 20.0), (real)(-worldSize * 20.0), (real)(-worldSize * 20.0));
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // moving (a lot) beyond the worldsize
        firstBox->translate((real)(worldSize * 2000.0), (real)(worldSize * 2000.0), (real)(worldSize * 2000.0));
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        world.removeProxy(firstBox);
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        delete firstBox;
    }


    void BroadPhaseHierarchicalGridTest::testTreeCorrectnessManyProxies() {
        const real worldSize = 1000.0;
        const real boxSizeSmall = 10.0;
        const real boxSizeMedium = 40.0;
        const real boxSizeLarge = 150.0;
        WorldParameters parameters;
        parameters.setBroadPhaseType(mBroadPhaseType);
        parameters.setWorldMinMax(Vector3(-worldSize, -worldSize, -worldSize), Vector3(worldSize, worldSize, worldSize));
        World world(parameters);

        BroadPhaseHierarchicalGrid* broadPhase =
                dynamic_cast<BroadPhaseHierarchicalGrid*>
                (world.getBroadPhase());
        CPPUNIT_ASSERT(broadPhase != 0);

        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        std::list<Proxy*> proxies;
        std::list<Vector3> speeds;
        // proxies of different sizes that always overlap
        for (int i = 0; i < 20; i++) {
            Proxy* p1 = world.createProxy(new Box(boxSizeSmall, boxSizeSmall, boxSizeSmall));
            Proxy* p2 = world.createProxy(new Box(boxSizeMedium, boxSizeMedium, boxSizeMedium));
            Proxy* p3 = world.createProxy(new Box(boxSizeLarge, boxSizeLarge, boxSizeLarge));
            p1->translate((real)(i * boxSizeLarge + 10.0), 0.0, 100.0);
            p2->translate((real)(i * boxSizeLarge + 10.0), 0.0, 100.0);
            p3->translate((real)(i * boxSizeLarge + 10.0), 0.0, 100.0);
            world.addProxy(p1);
            world.addProxy(p2);
            world.addProxy(p3);
            proxies.push_back(p1);
            proxies.push_back(p2);
            proxies.push_back(p3);

            // they should always overlap -> same speed
            const Vector3 speed = Vector3(10.0, 0.0, 0.0);
            speeds.push_back(speed);
            speeds.push_back(speed);
            speeds.push_back(speed);
        }

        // proxies of different sizes with different speeds
        for (int i = 0; i < 20; i++) {
            Proxy* p1 = world.createProxy(new Box(boxSizeSmall, boxSizeSmall, boxSizeSmall));
            Proxy* p2 = world.createProxy(new Box(boxSizeMedium, boxSizeMedium, boxSizeMedium));
            Proxy* p3 = world.createProxy(new Box(boxSizeLarge, boxSizeLarge, boxSizeLarge));
            world.addProxy(p1);
            world.addProxy(p2);
            world.addProxy(p3);
            proxies.push_back(p1);
            proxies.push_back(p2);
            proxies.push_back(p3);

            Vector3 speed1((real)(50.0 * i), 5.0, 0.0);
            Vector3 speed2((real)(20.0 * i), 0.0, 0.0);
            Vector3 speed3((real)(10.0 * i), 0.0, 0.0);
            speeds.push_back(speed1);
            speeds.push_back(speed2);
            speeds.push_back(speed3);
        }


        world.prepareSimulation();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // move without updates
        for (int i = 0; i < 20; i++) {
            moveProxies(proxies, speeds);
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // move, followed by update
        for (int i = 0; i < 20; i++) {
            moveProxies(proxies, speeds);
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
            broadPhase->update();
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }

        // back to origin
        for (std::list<Proxy*>::const_iterator it = proxies.begin(); it != proxies.end(); ++it) {
            (*it)->setTransformation(Matrix());
        }
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // increase speed
        for (std::list<Vector3>::iterator it = speeds.begin(); it != speeds.end(); ++it) {
            *it = (*it) * 20.0;
        }

        // move again - note: this should certainly go far off the initial
        // worldSize, so world growth is included here
        for (int i = 0; i < 100; i++) {
            moveProxies(proxies, speeds);
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // back to origin
        for (std::list<Proxy*>::const_iterator it = proxies.begin(); it != proxies.end(); ++it) {
            (*it)->setTransformation(Matrix());
        }
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // make speed negative, i.e. move into the opposite direction
        for (std::list<Vector3>::iterator it = speeds.begin(); it != speeds.end(); ++it) {
            *it = (*it) * -1.0;
        }

        // move again
        for (int i = 0; i < 100; i++) {
            moveProxies(proxies, speeds);
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }
        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // remove some proxies
        int i = 0;
        while (i < 20 && !proxies.empty()) {
            i++;
            world.removeProxy(proxies.front());
            delete proxies.front();
            proxies.pop_front();
            speeds.pop_front();
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }

        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        // move again
        for (int i = 0; i < 100; i++) {
            moveProxies(proxies, speeds);
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());

        // remove all proxies
        while (!proxies.empty()) {
            world.removeProxy(proxies.front());
            delete proxies.front();
            proxies.pop_front();
            speeds.pop_front();
            CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
        }

        broadPhase->update();
        CPPUNIT_ASSERT_EQUAL(true, broadPhase->checkTreeConsistency());
    }

    void BroadPhaseHierarchicalGridTest::moveProxies(const std::list<Proxy*>&
            proxies, const std::list<Vector3>& speeds) {
        CPPUNIT_ASSERT(proxies.size() == speeds.size());
        std::list<Proxy*>::const_iterator proxyIt = proxies.begin();
        std::list<Vector3>::const_iterator speedIt = speeds.begin();
        while (proxyIt != proxies.end()) {
            (*proxyIt)->translate(*speedIt);

            ++proxyIt;
            ++speedIt;
        }
    }

}
/*
 * vim: et sw=4 ts=4
 */
