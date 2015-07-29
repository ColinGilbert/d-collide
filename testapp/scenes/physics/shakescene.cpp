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

#include "shakescene.h"
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
ShakeScene::ShakeScene(Ogre::Root* root)  : PhysicsSceneBase(root) {
    mPhysicsSimStepsize = 0.03;
    mLinearDampingFactor = 0.005;
    mAngularDampingFactor = 0.005;
}

ShakeScene::~ShakeScene() {

}


dcollide::Vector3 ShakeScene::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}


/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 */
bool ShakeScene::initializeScene() {
    dWorldSetGravity(mOdeWorld, 0,0,-10);

    MyObjectNode* baseplate = new MyObjectNode( getCollisionWorld(),
            new dcollide::Box(500,500, 20),
                              dcollide::PROXYTYPE_FIXED);
    baseplate->translate(dcollide::Vector3(-250, -250, -20));
    addTopLevelObject(baseplate);

    /*     z 
     *     ^  -10    10 y
     *  40 |   +-----+ /
     *     |   |     |/
     *     |   |     |
     *  20 |   +-----+
     *  20 | +---+ +---+
     *     | |   | |   |
     *   0 | +---+ +---+
     *     +-----------------> x
     *      -21 -1,0,1 21
     */
    
    MyObjectNode* underBox1 =  new MyObjectNode(
        getCollisionWorld(),new dcollide::Box(20,20,20), dcollide::PROXYTYPE_RIGID, true, false);
    underBox1->setPosition(-29,0,0);
    underBox1->createPhysicsBody(mOdeWorld,4);
    addTopLevelObject(underBox1);
    
    MyObjectNode* underBox2 =  new MyObjectNode(
            getCollisionWorld(),new dcollide::Box(20,20,20), dcollide::PROXYTYPE_RIGID, true, false);
    underBox2->setPosition(9,0,0);
    underBox2->createPhysicsBody(mOdeWorld,4);
    addTopLevelObject(underBox2);
    
    MyObjectNode* TopBox =  new MyObjectNode(
            getCollisionWorld(),new dcollide::Box(20,20,20), dcollide::PROXYTYPE_RIGID, true, false);
    TopBox->setPosition(-10,0,20);
    TopBox->createPhysicsBody(mOdeWorld,50);
    addTopLevelObject(TopBox);

    
    // create my own wall
    /*     z 
     *     ^                +---+
     *     |                |   |
     *     |                +---+
     *     |             +---+ +---+
     *     |             |   | |   | 
     *     |             +---+ +---+
     *     |          +---+ +---+ +---+
     *     |          |   | |   | |   |
     *     |          +---+ +---+ +---+
     +     |       +---+ +---+ +---+ +---+
     *     |       |   | |   | |   | |   |
     *     |       +---+ +---+ +---+ +---+
     *     |    +---+ +---+ +---+ +---+ +---+
     *     |    |   | |   | |   | |   | |   |
     *     |    +---+ +---+ +---+ +---+ +---+
     *     | +---+ +---+ +---+ +---+ +---+ +---+
     *     | |   | |   | |   | |   | |   | |   |
     *     | +---+ +---+ +---+ +---+ +---+ +---+
     *     +---------------------------------------> x
     *  -63,-43,-42,-22,-21,-1,0,1 21,22 42,43 63
     *  2nd level: -31,-11,-10,10,11,31,32,52
                              
     */
    int baseBoxes = 10;
    int boxWidth = 20;
    int boxDistance = 15;
    for (int row = 0; row<baseBoxes;row++){
        for (int i=0; i<baseBoxes-row; i++){
            MyObjectNode* box = new MyObjectNode( getCollisionWorld(),
                    new dcollide::Box(boxWidth,boxWidth,boxWidth),dcollide::PROXYTYPE_RIGID, true, false);
            float nr = (baseBoxes-row)/2.0;
            box->setPosition(-(nr*boxWidth+(nr-0.5)*boxDistance) + i*boxWidth+i*boxDistance, 100 , row*boxWidth );
            box->createPhysicsBody(mOdeWorld,  2);
            addTopLevelObject(box);
        }
    }
    
    // Gregors Wall
    dcollide::real wallDistance = -100;
    int baseboxes = 8;
    for (int row = 0; row<baseboxes;row++){
        for (int i=0; i<baseboxes-row; i++){
            MyObjectNode* box = new MyObjectNode( getCollisionWorld(),
                    new dcollide::Box(10,10, 10),
                                      dcollide::PROXYTYPE_RIGID, true, false);
            box->setPosition(-50 + (row * 15)/2 + i * 13, wallDistance , row*10);
            box->createPhysicsBody(mOdeWorld,  4);
            addTopLevelObject(box);
        }
    }


    return true;
}

void ShakeScene::startNextSceneFrame() {
}

std::string ShakeScene::getSceneDescription() const {
    return "One physikbody on top of two other. Why is it shaking?";
}
/*
 * vim: et sw=4 ts=4
 */
