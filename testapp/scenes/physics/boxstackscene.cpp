/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,         *
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

#include "boxstackscene.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"
#include "debug.h"

#include "Ogre.h"

#include <world.h>
#include <shapes/shapes.h>
#include <worldcollisions.h>
#include <collisioninfo.h>
#include <proxyfactory.h>

#include <algorithm>

/*!
 * Construct a new d-collide based scene object.
 *
 * The scene itself is not yet created, only the necessary data
 * structures. Call \ref initializeScene to actually create a scene.
 *
 * \param root A pointer to the ogre root object. Ownership is NOT
 * taken, the pointer will not be deleted by this class.
 */
BoxstackScene::BoxstackScene(Ogre::Root* root)
    : PhysicsSceneBase(root) {

    mPhysicsSimStepsize = 0.01;
    }

    BoxstackScene::~BoxstackScene() {

    }


    dcollide::Vector3 BoxstackScene::initialWorldDimension() const {
        return dcollide::Vector3(2048.0, 2048.0, 2048.0);
    }


/*!
     * Setup the actual scene, i.e. add objects to the \ref dcollide::World
     * object.
     *
     * \return TRUE on success, otherwise FALSE.
 */
    bool BoxstackScene::initializeScene() {
        dWorldSetGravity(mOdeWorld, 0,0,-10);

        // small cube on big cube, only two vertices overlap
        baseA = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_FIXED, true, false);

        baseA->translate(dcollide::Vector3(-530, -30, 0));
        addTopLevelObject(baseA);

        topA = new MyObjectNode(  getCollisionWorld(),
                                                new dcollide::Box(30, 30, 30),
                                                dcollide::PROXYTYPE_RIGID);
        topA->createPhysicsBody(mOdeWorld,  5);
        topA->translate(dcollide::Vector3(-480, -20, 70));
        addTopLevelObject(topA);

        // two cubes of equal size, 45 deg. rotated
        baseB = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_FIXED, true, false);

        baseB->translate(dcollide::Vector3(-400, -30, 0));
        addTopLevelObject(baseB);

        topB = new MyObjectNode(  getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_RIGID);
        topB->createPhysicsBody(mOdeWorld,  5);
        topB->translate(dcollide::Vector3(-370, -30, 70));
        topB->rotate(45, 0,0,1);
        addTopLevelObject(topB);

        // two cubes of equal size, edge-aligned
        baseC = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_FIXED, true, false);

        baseC->translate(dcollide::Vector3(-300, 0, 0));
        addTopLevelObject(baseC);

        topC = new MyObjectNode(  getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_RIGID);
        topC->createPhysicsBody(mOdeWorld,  5);
        topC->translate(dcollide::Vector3(-300, 0, 65));
        addTopLevelObject(topC);


        // two cubes of equal size, top box somewhat translated
        baseD = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_FIXED, true, false);

        baseD->translate(dcollide::Vector3(-200, 0, 0));
        addTopLevelObject(baseD);

        topD = new MyObjectNode(  getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_RIGID);
        topD->createPhysicsBody(mOdeWorld,  5);
        topD->translate(dcollide::Vector3(-180, 10, 70));
        addTopLevelObject(topD);

        // box on cube, box translated so the boxe's center of gravity is NOT supported by the base cube
        baseE = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_FIXED, true, false);

        baseE->translate(dcollide::Vector3(0, 0, 0));
        addTopLevelObject(baseE);

        topE = new MyObjectNode(  getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_RIGID);
        topE->createPhysicsBody(mOdeWorld,  5);
        topE->translate(dcollide::Vector3(50, 50, 72));
        topE->rotate(30,0,0,1);
        addTopLevelObject(topE);


        // box falling on base and hitting the base with an vertex
        baseF = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(60, 60, 60),
                                                dcollide::PROXYTYPE_FIXED, true, false);

        baseF->translate(dcollide::Vector3(200, -30, 0));
        addTopLevelObject(baseF);

        topF = new MyObjectNode(  getCollisionWorld(),
                                                new dcollide::Box(60, 100, 60),
                                                dcollide::PROXYTYPE_RIGID);
        topF->createPhysicsBody(mOdeWorld,  5);
        topF->translate(dcollide::Vector3(200, -30, 110));
        topF->rotate(30, 1,1,1);
        addTopLevelObject(topF);

        return true;
    }

    // Restart-function (r-key)
    void BoxstackScene::restart() {
        // remove all objects from the scene
        removeObject(baseA);
        removeObject(topA);

        removeObject(baseB);
        removeObject(topB);

        removeObject(baseC);
        removeObject(topC);

        removeObject(baseD);
        removeObject(topD);

        removeObject(baseE);
        removeObject(topE);

        removeObject(baseF);
        removeObject(topF);
        
        // delete all collision results
        deleteCollisions();
        
        // initialize the scene again
        initializeScene();
    
    }

    void BoxstackScene::startNextSceneFrame() {
        //TODO simulate friction: calculate velocities and apply damping forces

        //General velocity-based damping force for all balls
        //dReal damping = 0.1;

        //const dReal* velocity = dBodyGetLinearVel(mBall2->getPhysicsBody());
        //dBodyAddForce(mBall2->getPhysicsBody(), -damping * velocity[0], -damping * velocity[1], -damping * velocity[2]);
    }

    std::string BoxstackScene::getSceneDescription() const {
        return "Box stacking physics testscene.\nUntextured Boxes are fixed.";
    }
/*
    * vim: et sw=4 ts=4
 */
