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

#include "undotest.h"

#include <world.h>
#include <proxy.h>
#include <shapes/box.h>
#include <debugstream.h>
#include <math/matrix.h>

using namespace std;
using namespace dcollide;

CPPUNIT_TEST_SUITE_REGISTRATION(UndoTest);

namespace dcollide {
    void UndoTest::setUp() {
    }

    void UndoTest::tearDown() {
    }

    void UndoTest::testUndo() {
        // we maintain 2 World objects, a "undo" World and a "reference" world
        //
        // we will do some changes on both worlds and then do some changes on
        // the "undo" world only. after calling undo, both worlds should be
        // synced again.
        //
        // whenever we start tests, both worlds should be synced.
        World undoWorld;
        World referenceWorld;

        // add simple proxies and never move them
        // this tests in particular that a undoStep() call does not change
        // things that shouldnt change
        undoWorld.addProxy(undoWorld.createProxy(new Box(10, 10, 10)));
        referenceWorld.addProxy(referenceWorld.createProxy(new Box(10, 10, 10)));

        // add simple proxies, move them before adding, never move them
        // afterwards
        // this tests in particular that a undoStep() call does not change
        // things that shouldnt change
        Proxy* staticUndoProxy = undoWorld.createProxy(new Box(10, 10, 10));
        Proxy* staticRefereceProxy = referenceWorld.createProxy(new Box(10, 10, 10));
        staticUndoProxy->translate(-10, 10, 5);
        staticRefereceProxy->translate(-10, 10, 5);
        undoWorld.addProxy(staticUndoProxy);
        referenceWorld.addProxy(staticRefereceProxy);

        // add simple proxies, but move them before adding
        Proxy* proxyUndoWorld = undoWorld.createProxy(new Box(10, 10, 10));
        Proxy* proxyReferenceWorld = referenceWorld.createProxy(new Box(10, 10, 10));
        proxyUndoWorld->translate(-10, 10, 5);
        proxyReferenceWorld->translate(-10, 10, 5);
        undoWorld.addProxy(proxyUndoWorld);
        referenceWorld.addProxy(proxyReferenceWorld);

        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));
        undoWorld.startNextStep();
        referenceWorld.startNextStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo simple translations
        proxyUndoWorld->translate(100, 100, 100);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo multiple translations
        proxyUndoWorld->translate(100, 100, 100);
        proxyUndoWorld->translate(100, 300, 200);
        proxyUndoWorld->translate(100, 400, 500);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo simple rotations
        proxyUndoWorld->rotate(45, 1, 0, 0);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo multiple simple rotations
        proxyUndoWorld->rotate(45, 1, 0, 0);
        proxyUndoWorld->rotate(15, 1, 0, 0);
        proxyUndoWorld->rotate(45, 0, 1, 0);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo translations and rotations
        proxyUndoWorld->translate(100, 100, 100);
        proxyUndoWorld->rotate(45, 1, 0, 0);
        proxyUndoWorld->translate(100, 1, 100);
        proxyUndoWorld->rotate(15, 1, 0, 0);
        proxyUndoWorld->translate(100, 20, 100);
        proxyUndoWorld->rotate(45, 0, 1, 0);
        proxyUndoWorld->translate(100, 100, 400);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo setTransformation()
        Matrix m;
        m.translate(100, 0, 0);
        proxyUndoWorld->setTransformation(m);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo multiple setTransformation()
        m = Matrix();
        m.translate(100, 0, 0);
        proxyUndoWorld->setTransformation(m);
        m.translate(0, 50, 0);
        proxyUndoWorld->setTransformation(m);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo setProxyType()
        proxyUndoWorld->setProxyType(PROXYTYPE_DEFORMABLE);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // undo multiple setProxyType()
        proxyUndoWorld->setProxyType(PROXYTYPE_DEFORMABLE);
        proxyUndoWorld->setProxyType(PROXYTYPE_FIXED);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));


        ////////////////////////////////////////////////////////////////
        // build up a simple hierarchy with static + non-static children
        ////////////////////////////////////////////////////////////////
        Proxy* staticChildUndoWorld = undoWorld.createProxy(new Box(10, 10, 10));
        Proxy* staticChildReferenceWorld = referenceWorld.createProxy(new Box(10, 10, 10));
        staticChildUndoWorld->translate(40, 10, 10);
        staticChildReferenceWorld->translate(40, 10, 10);
        proxyUndoWorld->addChild(staticChildUndoWorld);
        proxyReferenceWorld->addChild(staticChildReferenceWorld);

        Proxy* childUndoWorld = undoWorld.createProxy(new Box(10, 10, 10));
        Proxy* childReferenceWorld = referenceWorld.createProxy(new Box(10, 10, 10));
        childUndoWorld->translate(0, 0, 10);
        childReferenceWorld->translate(0, 0, 10);
        childUndoWorld->rotate(100, 0, 0, 1);
        childReferenceWorld->rotate(100, 0, 0, 1);
        proxyUndoWorld->addChild(childUndoWorld);
        proxyReferenceWorld->addChild(childReferenceWorld);

        Proxy* staticGrandChildUndoWorld = undoWorld.createProxy(new Box(10, 10, 10));
        Proxy* staticGrandChildReferenceWorld = referenceWorld.createProxy(new Box(10, 10, 10));
        staticGrandChildUndoWorld->translate(20, 10, 10);
        staticGrandChildReferenceWorld->translate(20, 10, 10);
        childUndoWorld->addChild(staticGrandChildUndoWorld);
        childReferenceWorld->addChild(staticGrandChildReferenceWorld);
        ////////////////////////////////////
        // build up a simple hierarchy (end)
        ////////////////////////////////////

        // sanity check
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // adding child proxies should not have any influence on undo, i.e. undo
        // should be a noop here.
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        Proxy* dummyUndoProxy = undoWorld.createProxy(new Box(10, 10, 10));
        Proxy* dummyReferenceProxy = referenceWorld.createProxy(new Box(10, 10, 10));
        undoWorld.addProxy(dummyUndoProxy);
        referenceWorld.addProxy(dummyReferenceProxy);

        // another check that the addProxy() did not change anything
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        //////////////////////////////////////
        // add grand-children to the hierarchy
        //////////////////////////////////////
        Proxy* grandChildUndoWorld = undoWorld.createProxy(new Box(10, 10, 10));
        Proxy* grandChildReferenceWorld = referenceWorld.createProxy(new Box(10, 10, 10));
        grandChildUndoWorld->translate(0, 0, 100);
        grandChildReferenceWorld->translate(0, 0, 100);
        grandChildUndoWorld->rotate(100, 0, 0, 1);
        grandChildReferenceWorld->rotate(100, 0, 0, 1);
        childUndoWorld->addChild(grandChildUndoWorld);
        childReferenceWorld->addChild(grandChildReferenceWorld);
        ////////////////////////////////////////////
        // add grand-children to the hierarchy (end)
        ////////////////////////////////////////////

        // check that grand-children adding did not change anything
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // check for undo on children
        childUndoWorld->translate(60, 0, 0);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // check for undo on grand-children
        grandChildUndoWorld->translate(60, 0, 0);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // check for undo on the whole hierarchy
        proxyUndoWorld->translate(5, 5, 5);
        childUndoWorld->rotate(54, 1, 0, 0);
        m = Matrix();
        m.translate(10, 10, 10);
        m.rotate(44, 1, 0, 0);
        grandChildUndoWorld->setTransformation(m);
        grandChildUndoWorld->translate(60, 0, 0);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));


        // check that removeProxy() is NOT undone
        undoWorld.removeProxy(dummyUndoProxy);
        referenceWorld.removeProxy(dummyReferenceProxy);
        delete dummyUndoProxy;
        delete dummyReferenceProxy;
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));


        proxyUndoWorld->translate(10, 10, 10);
        proxyReferenceWorld->translate(10, 10, 10);
        // undo should cause the worlds NOT to be equal
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(false, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        proxyUndoWorld->translate(10, 10, 10);
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // check that that startNextStep() actually works as intended, i.e. we undo
        // the things _after_ the startNextStep(), not the ones before
        undoWorld.startNextStep();
        // startNextStep() should not undo anything (except the moveflags)
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // startNextStep() should clear all possible "undo" values, i.e. this
        // should be a noop
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // things _after_ a startNextStep() should be undone again
        proxyUndoWorld->translate(10, 10, 10);
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // check that removing of grand-children does not change anything
        childUndoWorld->removeChild(grandChildUndoWorld);
        childReferenceWorld->removeChild(grandChildReferenceWorld);
        delete grandChildUndoWorld;
        delete grandChildReferenceWorld;
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // check that calls to startNextStep() after removeChild() still works
        undoWorld.startNextStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));
        undoWorld.undoStep();
        CPPUNIT_ASSERT_EQUAL(true, checkWorldsAreEqual(&undoWorld, &referenceWorld));

        // TODO: check that deformations are un-done?
        //       open question: do we want to support un-doing of deformations?
    }

    bool UndoTest::checkWorldsAreEqual(World* undoWorld, World* referenceWorld) {
        if (referenceWorld->getTopLevelProxies().size() != undoWorld->getTopLevelProxies().size()) {
//            debug() << dc_funcinfo << "different toplevel proxy count";
            return false;
        }

        const std::list<Proxy*>& undoProxies = undoWorld->getTopLevelProxies();
        const std::list<Proxy*>& referenceProxies = referenceWorld->getTopLevelProxies();
        std::list<Proxy*>::const_iterator proxyItUndo = undoProxies.begin();
        std::list<Proxy*>::const_iterator proxyItReference = referenceProxies.begin();
        while (proxyItReference != referenceProxies.end()) {
            if (!checkProxiesAreEqual(*proxyItUndo, *proxyItReference)) {
                return false;
            }

            ++proxyItReference;
            ++proxyItUndo;
        }

        return true;
    }

    bool UndoTest::checkProxiesAreEqual(Proxy* undoProxy, Proxy* referenceProxy) {
        if (undoProxy->getProxyType() != referenceProxy->getProxyType()) {
//            debug() << dc_funcinfo << "proxytype differs: is=" << undoProxy->getProxyType() << " should=" << referenceProxy->getProxyType();
            return false;
        }
        if (!undoProxy->getTransformation().isEqual(referenceProxy->getTransformation())) {
//            debug() << dc_funcinfo << "matrix differs: is=" << undoProxy->getTransformation() << " should=" << referenceProxy->getTransformation();
            return false;
        }

        // AB: we do NOT check the Proxy::getMoveFlags() for being equal!
        //     -> if world A uses startNextStep() and world B does not, then the
        //        proxies will usually have different moveflags, since
        //        startNextStep() resets them to "umoved".

        const std::list<Proxy*>& undoChildren = undoProxy->getChildProxies();
        const std::list<Proxy*>& referenceChildren = referenceProxy->getChildProxies();
        std::list<Proxy*>::const_iterator proxyItUndo = undoChildren.begin();
        std::list<Proxy*>::const_iterator proxyItReference = referenceChildren.begin();
        while (proxyItReference != referenceChildren.end()) {
            if (!checkProxiesAreEqual(*proxyItUndo, *proxyItReference)) {
                return false;
            }

            ++proxyItReference;
            ++proxyItUndo;
        }
        return true;
    }

}
/*
 * vim: et sw=4 ts=4
 */
