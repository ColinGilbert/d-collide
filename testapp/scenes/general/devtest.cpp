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

#include "devtest.h"
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
DevTestScene::DevTestScene(Ogre::Root* root) : PhysicsSceneBase(root) {
    mSphere = new dcollide::Sphere(5);
    mPhysicsSimStepsize = 0.03;
    mLinearDampingFactor = 0.005;
    mAngularDampingFactor = 0.005;
}

DevTestScene::~DevTestScene() {
    delete mSphere;
}


//-------------------------- Scene configuration -------------------------------

dcollide::Vector3 DevTestScene::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

void DevTestScene::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {

}

std::string DevTestScene::getSceneDescription() const {
    return "Developer Scene (Local Testing)";
}


//----------------------- Scene objects/initialisation -------------------------

/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 **/
bool DevTestScene::initializeScene() {
    dWorldSetGravity(mOdeWorld, 0,0,-10);
    //dWorldSetContactSurfaceLayer(mOdeWorld, 2.0);

mCollisionWorld->setNarrowPhaseStrategySphereSphere(dcollide::
NP_STRATEGY_SLOWEST_EXTENSIVE);



    //the table will not receive a physics body and is considered to be
    //"static environment" which is not affected by gravity.

    MyObjectNode* baseplate = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(2000, 2000,
20),
                                                dcollide::PROXYTYPE_FIXED);
    baseplate->translate(dcollide::Vector3(-1000, -1000, -20));
    addTopLevelObject(baseplate);
/*
    mBunny = new MyObjectNode( getCollisionWorld(),
                                    new dcollide::Box(10,10,10),
                                    dcollide::PROXYTYPE_RIGID, true, true);
//*/
#ifdef PHYSIC_BUNNY
    mBunny = createPlyModel("bunny_lower_res2.ply",
            (dcollide::ProxyTypes) dcollide::PROXYTYPE_RIGID,
             800.0f);
    mBunny->createPhysicsBody(mOdeWorld, mOdeCollisionSpace, 5);
    addTopLevelObject(mBunny);
    mDampedBodyNodes.push_back(mBunny);
#else
    mBunny = createPlyModel("bunny_lower_res2.ply",
            (dcollide::ProxyTypes) dcollide::PROXYTYPE_FIXED,
             800.0f);
    addTopLevelObject(mBunny);
#endif

    restart();
    return true;
}

void DevTestScene::action() {
    double radius = 2 + (rand() % 5);
#ifdef MESH_BALL
    MyObjectNode* ball = new MyObjectNode(getCollisionWorld(),
                            mSphere->getMesh()->cloneMesh(),
                            dcollide::PROXYTYPE_RIGID);
#else
    MyObjectNode* ball = new MyObjectNode(getCollisionWorld(),
                                          new dcollide::Sphere(radius),
                                                  dcollide::PROXYTYPE_RIGID);
#endif
    ball->createPhysicsBody(mOdeWorld, 1);
    const Ogre::Vector3 camDirection = mOgreCamera->getDirection();
    const Ogre::Vector3 camPos = mOgreCamera->getPosition();

    Ogre::Vector3 ballPosition = camPos + camDirection *2* radius
            - mOgreCamera->getUp() * radius;
    ball->setPosition(ballPosition[0], ballPosition[1], ballPosition[2]);

    addTopLevelObject(ball);
    mBalls.push_back(ball);
    mDampedBodyNodes.push_back(ball);

    Ogre::Vector3 force =  camDirection * 3000;
    dBodyAddForce(ball->getPhysicsBody(), force[0], force[1], force[2]);
}


//---------------- Object manipulations / status information -------------------

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void DevTestScene::restart() {
    mDampedBodyNodes.clear();
    mBunny->setPosition(100, 0, -27); //[-25, -28]
    mBunny->setRotation(dcollide::Matrix());
    mBunny->rotate(90.0, 90.0, 0.0, 0.0);

#ifdef PHYSIC_BUNNY
    dBodySetForce(mBunny->getPhysicsBody(), 0,0,0);
    dBodySetAngularVel(mBunny->getPhysicsBody(), 0,0,0);
    dBodySetLinearVel(mBunny->getPhysicsBody(), 0,0,0);

    mDampedBodyNodes.push_back(mBunny);
#endif

    //delete all balls
    for (std::list<MyObjectNode*>::iterator iter = mBalls.begin();
         iter != mBalls.end(); ++iter){
             MyObjectNode* ball = *iter;
             removeObject(ball);
         }

         mBalls.clear();


         deleteCollisions();
}

void DevTestScene::startNextSceneFrame() {
}

/*
 * vim: et sw=4 ts=4
 */
