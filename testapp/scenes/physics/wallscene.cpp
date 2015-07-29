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

#include "wallscene.h"
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
WallScene::WallScene(Ogre::Root* root)
    : PhysicsSceneBase(root) {
    mPhysicsSimStepsize = 0.03;
    mLinearDampingFactor = 0.005;
    mAngularDampingFactor = 0.005;
}

WallScene::~WallScene() {

}


dcollide::Vector3 WallScene::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}


/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 */
bool WallScene::initializeScene() {
    dWorldSetGravity(mOdeWorld, 0,0,-10);
    //dWorldSetContactSurfaceLayer(mOdeWorld, 2.0);
    mCollisionWorld->setNarrowPhaseStrategySphereSphere(dcollide::NP_STRATEGY_SLOWEST_EXTENSIVE);
            

    
    // Top view:
    // z=200
    //  ^
    //  |                   
    //  |               X   
    //  |               X    
    //  |               X    
    //  0------------------->
    //origin            x=300
    
    //The table-proxy will contain 5 children
    //one for the baseplate, and 4 cushions
    
    //the table will not receive a physics body and is considered to be
    //"static environment" which is not affected by gravity.

    MyObjectNode* baseplate = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(2000, 2000, 20),
                                                dcollide::PROXYTYPE_FIXED);
    baseplate->translate(dcollide::Vector3(-1000, -1000, -20));
    addTopLevelObject(baseplate);

    //create boxes for wall1 and wall2

    for (int i = 0; i< 8+7+6+5+4+3+2+1; i++){
        MyObjectNode* box = new MyObjectNode( getCollisionWorld(),
                                                        new dcollide::Box(10,10, 10),
                                                        dcollide::PROXYTYPE_RIGID, true, true);
        box->createPhysicsBody(mOdeWorld,  1);
        addTopLevelObject(box);
        mWall1.push_back(box);
        mDampedBodyNodes.push_back(box);
            
        MyObjectNode* box2 = new MyObjectNode( getCollisionWorld(),
                                                        new dcollide::Box(10,10, 10),
                                                        dcollide::PROXYTYPE_RIGID, true, true);
        box2->createPhysicsBody(mOdeWorld,  5);
        addTopLevelObject(box2);
        mWall2.push_back(box2);
        mDampedBodyNodes.push_back(box2);
    }
    restart();
    return true;
}

/*!
 * \brief wallscene special action: generate and fire a ball in camera direction
 */
void WallScene::action() {
    double radius = 2 + (rand() % 5);
    MyObjectNode* ball = new MyObjectNode(getCollisionWorld(),
                           new dcollide::Sphere(radius), dcollide::PROXYTYPE_RIGID);
    ball->createPhysicsBody(mOdeWorld,  1);
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

void WallScene::restart() {
    mDampedBodyNodes.clear();
    //reset boxes
    //Wall of Boxes
    //x-value is constant
    dcollide::real wallDistance = 100;
    int baseboxes = 8;
    int boxIndex = 0;
    for (int row = 0; row<baseboxes;row++){
        for (int i=0; i<baseboxes-row; i++){
            MyObjectNode* box = mWall1[boxIndex];
            box->setPosition(wallDistance, -50 + (row * 15)/2 + i * 13, row*10);
            box->setRotation(dcollide::Matrix());
            
            dBodySetForce(box->getPhysicsBody(), 0,0,0);
            dBodySetAngularVel(box->getPhysicsBody(), 0,0,0);
            dBodySetLinearVel(box->getPhysicsBody(), 0,0,0);
            
            boxIndex++;
            mDampedBodyNodes.push_back(box);
        }
    }
    
    wallDistance = 250;
    boxIndex = 0;
    for (int row = 0; row<baseboxes;row++){
        for (int i=0; i<baseboxes-row; i++){
            MyObjectNode* box = mWall2[boxIndex];
            box->setPosition(wallDistance, -50 + (row * 15)/2 + i * 13, row*10);
            box->setRotation(dcollide::Matrix());
            
            dBodySetForce(box->getPhysicsBody(), 0,0,0);
            dBodySetAngularVel(box->getPhysicsBody(), 0,0,0);
            dBodySetLinearVel(box->getPhysicsBody(), 0,0,0);
            
            boxIndex++;
            mDampedBodyNodes.push_back(box);
        }
    }
    
    //delete all balls
    for (std::list<MyObjectNode*>::iterator iter = mBalls.begin();
    iter != mBalls.end(); ++iter){
        MyObjectNode* ball = *iter;
        removeObject(ball);
    }
    
    mBalls.clear();

    
    deleteCollisions();
}
void WallScene::startNextSceneFrame() {
}

std::string WallScene::getSceneDescription() const {
    return "a Wall of boxes is hit by some cannonballs.\nPush space to launch a cannonball in camera direction";
}
/*
 * vim: et sw=4 ts=4
 */
