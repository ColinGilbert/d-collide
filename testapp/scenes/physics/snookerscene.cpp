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

#include "snookerscene.h"
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
SnookerScene::SnookerScene(Ogre::Root* root)
    : PhysicsSceneBase(root) {
    mTable=0;
    mBall1=0;
    mBall2=0;
    
    
    mPhysicsSimStepsize = 0.03;
    mLinearDampingFactor = 0.001;
    mAngularDampingFactor = 0.005;
}

SnookerScene::~SnookerScene() {

}


dcollide::Vector3 SnookerScene::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}


/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 */
bool SnookerScene::initializeScene() {
    dWorldSetGravity(mOdeWorld, 0,0,-10);
    mCollisionWorld->setNarrowPhaseStrategySphereSphere(dcollide::NP_STRATEGY_SLOWEST_EXTENSIVE);
            
    //Table: Box, top at z=0, height = 20
    //cusions: boxes
    //balls: spheres
    
    // Top view:
    // y=200
    //  3-------------------2
    //  |                   |
    //  |                   |
    //  |                   |
    //  |                   |
    //  0-------------------1
    //origin               x=350
    
    //The table-proxy will contain 5 children
    //one for the baseplate, and 4 cushions
    
    //the table will not receive a physics body and is considered to be
    //"static environment" which is not affected by gravity.
   mTable = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);
    
    MyObjectNode* baseplate = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(350,200, 20),
                                                dcollide::PROXYTYPE_FIXED);
    baseplate->translate(dcollide::Vector3(0, 0, -20));

    
    MyObjectNode* cushion01 = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(350, 10, 10),
                                                dcollide::PROXYTYPE_FIXED, true, false);
    cushion01->translate(dcollide::Vector3(0, -10, 0));
    
    MyObjectNode* cushion23 = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(350, 10, 10),
                                                dcollide::PROXYTYPE_FIXED, true, false);
    cushion23->translate(dcollide::Vector3(0, 200, 0));
    
    MyObjectNode* cushion12 = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(10, 200, 10),
                                                dcollide::PROXYTYPE_FIXED, true, false);
    cushion12->translate(dcollide::Vector3(350, 0, 0));
    
    MyObjectNode* cushion30 = new MyObjectNode( getCollisionWorld(),
                                                new dcollide::Box(10, 200, 10),
                                                dcollide::PROXYTYPE_FIXED, true, false);
    cushion30->translate(dcollide::Vector3(-10, 0, 0));
    
    
    mTable->addChild(baseplate);
    mTable->addChild(cushion01);
    mTable->addChild(cushion12);
    mTable->addChild(cushion23);
    mTable->addChild(cushion30);
    

    addTopLevelObject(mTable);


    //spheres
    mBall1 = new MyObjectNode(getCollisionWorld(),
                           new dcollide::Sphere(5), dcollide::PROXYTYPE_RIGID);
    mBall1->createPhysicsBody(mOdeWorld,  4);
    mBall1->translate(100.0, 50.0, 40.0);
    addTopLevelObject(mBall1);
    
    mBall2 = new MyObjectNode(getCollisionWorld(),
                              new dcollide::Sphere(10), dcollide::PROXYTYPE_RIGID);
    mBall2->createPhysicsBody(mOdeWorld,  20);
    mBall2->translate(200.0, 100.0, 60.0);
    
    addTopLevelObject(mBall2);

    //another ball, falling on top of ball 2
    ball3 = new MyObjectNode(getCollisionWorld(),
                              new dcollide::Sphere(10), dcollide::PROXYTYPE_RIGID);
    ball3->createPhysicsBody(mOdeWorld,  20);
    ball3->translate(205.0, 105.0, 200.0);
    
    addTopLevelObject(ball3);

    //A box wich should hit a cushion
    fallingBox = new MyObjectNode(getCollisionWorld(),
                            new dcollide::Box(50, 20, 30), dcollide::PROXYTYPE_RIGID);


    fallingBox->createPhysicsBody(mOdeWorld,  20);
    fallingBox->translate(305, 100, 150);
    //rotate 45 deg on x-axis.
    fallingBox->rotate(45, 1, 0 , 0);
    
    addTopLevelObject(fallingBox);
    
    return true;
}

// Restart-function (r-key)
void SnookerScene::restart() {
    // reset the positions of the three balls
    removeObject(mBall1);
    removeObject(mBall2);
    removeObject(ball3);

    /*mBall1->setPosition(100.0, 50.0, 40.0);
    mBall2->setPosition(200.0, 100.0, 60.0);
    ball3->setPosition(205.0, 105.0, 200.0);*/
    
    //remove the falling box
    removeObject(fallingBox);

    //create the balls again
    mBall1 = new MyObjectNode(getCollisionWorld(),
                           new dcollide::Sphere(5), dcollide::PROXYTYPE_RIGID);
    mBall1->createPhysicsBody(mOdeWorld,  4);
    mBall1->translate(100.0, 50.0, 40.0);
    addTopLevelObject(mBall1);
    
    mBall2 = new MyObjectNode(getCollisionWorld(),
                              new dcollide::Sphere(10), dcollide::PROXYTYPE_RIGID);
    mBall2->createPhysicsBody(mOdeWorld,  20);
    mBall2->translate(200.0, 100.0, 60.0);
    
    addTopLevelObject(mBall2);

    ball3 = new MyObjectNode(getCollisionWorld(),
                              new dcollide::Sphere(10), dcollide::PROXYTYPE_RIGID);
    ball3->createPhysicsBody(mOdeWorld,  20);
    ball3->translate(205.0, 105.0, 200.0);
    
    addTopLevelObject(ball3);
    
    //create another falling box with the same specifications
    fallingBox = new MyObjectNode(getCollisionWorld(),
                            new dcollide::Box(50, 20, 30), dcollide::PROXYTYPE_RIGID);


    fallingBox->createPhysicsBody(mOdeWorld,  20);
    fallingBox->translate(305, 100, 150);
    //rotate 45 deg on x-axis.
    fallingBox->rotate(45, 1, 0 , 0);
    
    addTopLevelObject(fallingBox);

    // delete all collision results
    deleteCollisions();

}

void SnookerScene::startNextSceneFrame() {
}

std::string SnookerScene::getSceneDescription() const {
    return "Some spheres on a table. Physics included";
}
/*
 * vim: et sw=4 ts=4
 */
