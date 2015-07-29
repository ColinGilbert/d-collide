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

#include "dominoday.h"
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
DominoDay::DominoDay(Ogre::Root* root)
    : PhysicsSceneBase(root) {
    mGround=0;
    mRamp=0;
 
    mPhysicsSimStepsize = 0.04;
    mLinearDampingFactor = 0.001;
    mAngularDampingFactor = 0.005;
    }

    DominoDay::~DominoDay() {

    }


    dcollide::Vector3 DominoDay::initialWorldDimension() const {
        return dcollide::Vector3(2048.0, 2048.0, 2048.0);
    }


/*!
     * Setup the actual scene, i.e. add objects to the \ref dcollide::World
     * object.
     *
     * \return TRUE on success, otherwise FALSE.
 */
    bool DominoDay::initializeScene() {
        dWorldSetGravity(mOdeWorld, 0,0,-10);

        MyObjectNode* environment = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);

        //Base Box: large Box with top surface at z=0;
        mGround = new MyObjectNode( getCollisionWorld(),
                new dcollide::Box(1200,1200, 50),
                                  dcollide::PROXYTYPE_FIXED);
        mGround->translate(dcollide::Vector3(-500, -600, -50));

        environment->addChild(mGround); 
        //Ramp: 
        mRamp = new MyObjectNode( getCollisionWorld(),
                        new dcollide::Box(100, 300, 100),
                                      dcollide::PROXYTYPE_FIXED,true, false);
        //the ramp inclines in y-direction
        mRamp->translate(dcollide::Vector3(-400, 200, -100));
        mRamp->rotate(40, 1, 0,0);

        environment->addChild(mRamp);

        //ramp and ground are static environment without phyisics bodies
        addTopLevelObject(environment);

        
        //create and place a ball which should roll the ramp down
        mBall = new MyObjectNode(getCollisionWorld(),
                                      new dcollide::Sphere(25),
                                      dcollide::PROXYTYPE_RIGID);
        
        mBall->createPhysicsBody(mOdeWorld,  15);
        mBall->translate(-350, 340, 185);
        addTopLevelObject(mBall);
       
        setupDominoBricks();
        
        return true;
    }

    void DominoDay::restart() {
        mBall->setPosition( -350, 340,185 );
        dBodySetForce( mBall->getPhysicsBody(), 0, 0, 0);

        for(std::list<MyObjectNode*>::iterator i = mBricks.begin(); i != mBricks.end(); ++i ) {
            removeObject(*i);
        }

        mBricks.clear();
        setupDominoBricks();

        deleteCollisions(); 
    }
    
    void DominoDay::setupDominoBricks() {
        dcollide::Matrix brickOrientation;

        brickOrientation.loadIdentity();
        brickOrientation.translate(-330, 150, 0);
        
        //Now positive y points downwards
        brickOrientation.rotate(180,0,0,1);

        for( int i = 0; i < 10; i++ ) {
            addBrick(brickOrientation);
            brickOrientation.translate(0,50,0);
        }

        //left curve
        for( int i = 0; i < 10; i++ ) {
            addBrick(brickOrientation);
           
            brickOrientation.translate(7,30,0);
            brickOrientation.rotate((float)180/(float)10,0,0,1);
        }

        for( int i = 0; i < 15; i++ ) {
            addBrick(brickOrientation);
            brickOrientation.translate(0,50,0);
        }

        //right curve
        for( int i = 0; i < 10; i++ ) {
            addBrick(brickOrientation);
                   
            brickOrientation.translate(7,30,0);
            brickOrientation.rotate(-(float)90/(float)10,0,0,1);
        }
        
        for( int i = 0; i < 5; i++ ) {
                    addBrick(brickOrientation);
                    brickOrientation.translate(0,50,0);
        }
       
        
        //right curve
        for( int i = 0; i < 5; i++ ) {
            addBrick(brickOrientation);
            brickOrientation.translate(7,30,0);
            brickOrientation.rotate(-(float)90/(float)5,0,0,1);
        }
        
        for( int i = 0; i < 2; i++ ) {
          addBrick(brickOrientation);
          brickOrientation.translate(0,50,0);
        }
        //save branch location - so we may start here later again
        dcollide::Matrix branchpoint;
        
        //increase brick count per row
        for( int i = 1; i < 12; i++ ) {
            branchpoint = brickOrientation;
            drawDominoLine(brickOrientation, i);
            
            //restore branch location
            brickOrientation = branchpoint;
            brickOrientation.translate(-20, 50, 0);
        }

        //decrease brick count per row
        for( int i = 11; i > 4; i-- ) {
            branchpoint = brickOrientation;
            drawDominoLine(brickOrientation, i);
            
            //restore branch location
            brickOrientation = branchpoint;
            brickOrientation.translate( 20, 50, 0);
        }
    }

    void DominoDay::drawDominoLine(dcollide::Matrix orientation, int length) {
        for( int i = 0; i < length; i++) {
            addBrick(orientation);
            orientation.translate(50,0,0);
        }
    }
    /*!
     * \brief Adds Brick to scene with orientation given by parameter
     */
    void DominoDay::addBrick(dcollide::Matrix orientation) {
        MyObjectNode* newBrick = new MyObjectNode(getCollisionWorld(), 
                                                      new dcollide::Box(30,5,60),
                                                      dcollide::PROXYTYPE_RIGID);
        newBrick->createPhysicsBody(mOdeWorld,  2);

        //Actually this is rotation + translation - =)
        newBrick->rotate(orientation);

        addTopLevelObject(newBrick);

        //Store all allocated bricks, so we may deallocate them later
        mBricks.push_back(newBrick);
    }

    void DominoDay::startNextSceneFrame() {

    }

    std::string DominoDay::getSceneDescription() const {
        return "Domino - you know it :) \n Lots of rigid interaction";
    }
/*
    * vim: et sw=4 ts=4
 */
