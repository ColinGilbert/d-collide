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

#include "rampscene.h"
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
RampScene::RampScene(Ogre::Root* root)
    : PhysicsSceneBase(root) {
    mGround=0;
    mRamp=0;
    mBall=0;
    mBox=0;
    
    mPhysicsSimStepsize = 0.03;
    mLinearDampingFactor = 0.001;
    mAngularDampingFactor = 0.005;
    }

    RampScene::~RampScene() {

    }


    dcollide::Vector3 RampScene::initialWorldDimension() const {
        return dcollide::Vector3(2048.0, 2048.0, 2048.0);
    }


/*!
     * Setup the actual scene, i.e. add objects to the \ref dcollide::World
     * object.
     *
     * \return TRUE on success, otherwise FALSE.
 */
    bool RampScene::initializeScene() {
        dWorldSetGravity(mOdeWorld, 0,0,-10);
        mCollisionWorld->setNarrowPhaseStrategySphereSphere(dcollide::NP_STRATEGY_SLOWEST_EXTENSIVE);
        //Scene setup description (side view):
        //                                      Box, supposed to fall onto the
        //                                      ramp and slide/tumble down
        //                                      ___
        //                                      |__|
        //
        //           ball, gets initial           .:: ramp (wedge)
        //          torque to roll right       .:::::
        //               o  --->            .::::::::
        //   z=0 -----------------------------------
        //       |  base box,                       |
        //
        
        //Put the "static environment" proxies into a parent to keep them from
        //colliding
        MyObjectNode* environment = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);
        
        //Base Box: large Box with top surface at z=0;
        
        mGround = new MyObjectNode( getCollisionWorld(),
                new dcollide::Box(1000,1000, 50),
                                  dcollide::PROXYTYPE_FIXED);
        mGround->translate(dcollide::Vector3(-500, -500, -50));
        environment->addChild(mGround);
    
        //Ramp: 
        /*
        mRamp = new MyObjectNode( getCollisionWorld(),
                new dcollide::Wedge(200, 300, 100),
                                  dcollide::PROXYTYPE_FIXED, true, false);
        //the ramp inclines in y-direction
        mRamp->translate(dcollide::Vector3(-100, 50, 0));
        */
        mRamp = new MyObjectNode( getCollisionWorld(),
                        new dcollide::Box(200, 400, 100),
                                      dcollide::PROXYTYPE_FIXED, true, false);
        //the ramp inclines in y-direction
        mRamp->translate(dcollide::Vector3(-100, 50, -100));
        mRamp->rotate(15, 1, 0,0);

        
        environment->addChild(mRamp);

        //ramp and ground are static environment without phyisics bodies
        addTopLevelObject(environment);

        mBall = new MyObjectNode(getCollisionWorld(),
                                 new dcollide::Sphere(5),
                                 dcollide::PROXYTYPE_RIGID);
        mBall->createPhysicsBody(mOdeWorld,  4);
        //place the ball on the ground
        mBall->translate(0, 0, 5);
        addTopLevelObject(mBall);
        mDampedBodyNodes.push_back(mBall);

        //create and place another ball which should roll the ramp down
        mUpperBall = new MyObjectNode(getCollisionWorld(),
                                      new dcollide::Sphere(10),
                                      dcollide::PROXYTYPE_RIGID);
        mUpperBall->createPhysicsBody(mOdeWorld,  5);
        mUpperBall->translate(50, 300, 97);
        addTopLevelObject(mUpperBall);
        mDampedBodyNodes.push_back(mUpperBall);

        mBox = new MyObjectNode( getCollisionWorld(),
                                 new dcollide::Box(30, 20, 50),
                                 dcollide::PROXYTYPE_RIGID);
        mBox->createPhysicsBody(mOdeWorld,  10);
        addTopLevelObject(mBox);

        //Place the box above the ramp
        mBox->translate(-25, 300, 95);

        //TODO why does the box not slide down the ramp?
        
        //initial forces
        //Force that rotates the ball so that it should roll towards the ramp
        //dBodyAddTorque(mBall->getPhysicsBody(), 11000, 0, 0);
        dBodyAddForceAtRelPos(mBall->getPhysicsBody(), 0, 7000, 0, 0, 5, 0);

        return true;
    }

    // Restart-function (r-key)
    void RampScene::restart() {
        // reset the positions of the two balls
        mBall->setPosition(0, 0, 5);

        mUpperBall->setPosition(50, 300, 97);

        //remove the box
        removeObject(mBox);
        
        //create another box with the same specifications
        mBox = new MyObjectNode( getCollisionWorld(),
                                 new dcollide::Box(30, 20, 50),
                                 dcollide::PROXYTYPE_RIGID);
        mBox->createPhysicsBody(mOdeWorld,  10);
        addTopLevelObject(mBox);

        //Place the box above the ramp
        mBox->translate(-25, 300, 95);
         
        dBodyAddForceAtRelPos(mBall->getPhysicsBody(), 0, 7000, 0, 0, 5, 0);
     
        // delete all collision results
        deleteCollisions();
    
    }

    void RampScene::startNextSceneFrame() {

    }

    std::string RampScene::getSceneDescription() const {
        return "Sliding and rolling on a ramp.\nUses ODE physics.";
    }
/*
    * vim: et sw=4 ts=4
 */
