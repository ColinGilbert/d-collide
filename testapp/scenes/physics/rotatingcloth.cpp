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

#include "rotatingcloth.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"
#include "mydeformableobjectnode.h"

#include <modelloader/loaderdummy.h>

#include <d-collide/debug.h>
#include <d-collide/math/vector.h>
#include <d-collide/proxy.h>
#include <d-collide/world.h>
#include <d-collide/shapes/mesh/vertex.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/shapes/mesh/vertex.h>

#include <d-collide/datatypes/multimapelement.h>

#include <d-collide/debugstream.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <set>

RotatingCloth::RotatingCloth(Ogre::Root* root) : PhysicsSceneBase(root) {
    mGround=0;
    mRamp=0;
    mBall=0;
    mCloth=0;
    mPhysicsSimStepsize = (float) 0.03;
    mLinearDampingFactor = (dReal) 0.001;
    mAngularDampingFactor = (dReal) 0.001;
}

RotatingCloth::~RotatingCloth() {
}

dcollide::Vector3 RotatingCloth::initialWorldDimension() const {
    return dcollide::Vector3 (2048.0, 2048.0, 2048.0);
}

void RotatingCloth::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_BV_HIERARCHY);
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_SPATIAL_HASH);
    algorithm.push_back(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);
    parameters.setDeformableAlgorithms(algorithm);
}

bool RotatingCloth::initializeScene() {
    dWorldSetGravity(getOdeWorld(), 0, 0, -10);
    mCollisionWorld->setNarrowPhaseStrategySphereSphere(dcollide::NP_STRATEGY_SLOWEST_EXTENSIVE);

    // Ramp, ground and rolling sphere:
    // ------------------------
    // Put the "static environment" proxies into a parent to keep them from
    // colliding
    MyObjectNode* environment = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);

    // Base Box: large Box with top surface at z=0;
    mGround = new MyObjectNode( getCollisionWorld(),
                                new dcollide::Box(1000,1000, 50),
                                dcollide::PROXYTYPE_FIXED,
                                true, false);
    mGround->translate(-500, -500, -50);
    environment->addChild(mGround);
/*
    // Ramp:
    mRamp = new MyObjectNode( getCollisionWorld(),
                    new dcollide::Box(200, 400, 100),
                                  dcollide::PROXYTYPE_FIXED, true, false);
    // the ramp inclines in y-direction
    mRamp->translate(dcollide::Vector3(-100, 100, -100));
    mRamp->rotate(35, 1, 0,0);

    environment->addChild(mRamp);
*/
    // ramp and ground are static environment without phyisics bodies
    addTopLevelObject(environment);

    // create and place another ball which should roll the ramp down
    mBall = new MyObjectNode(getCollisionWorld(),
                                  new dcollide::Sphere(10),
                                  dcollide::PROXYTYPE_RIGID);
    mBall->createPhysicsBody(mOdeWorld,  155);
    mBall->translate(50, -325, 10);
    addTopLevelObject(mBall);
    mDampedBodyNodes.push_back(mBall);

    //--- The cloth:
    //----------------------------------------------------
    ModelLoader::LoaderDummy loader;
    dcollide::Mesh* clothMesh
        = loader.createRectangleSurface((DCOLLIDE_CBS2_GRIDSIZE-1) * DCOLLIDE_CBS2_STEPSIZE,
                                             (DCOLLIDE_CBS2_GRIDSIZE-1) * DCOLLIDE_CBS2_STEPSIZE,
                                             DCOLLIDE_CBS2_STEPSIZE,
                                             DCOLLIDE_CBS2_STEPSIZE);



    mCloth = new MyDeformableObjectNode(getCollisionWorld(),
                                        clothMesh,
                                        dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_SELFCOLLIDABLE,
                                        false);
    mCloth->createPhysicsBody(mOdeWorld,
                        DCOLLIDE_CBS2_GRIDSIZE * DCOLLIDE_CBS2_GRIDSIZE * 0.5);

    addTopLevelObject(mCloth);
    mCloth->translate(0,-350,50);
    
    
/*
 * emulation by grid of rigid spheres
    //generate a grid of spheres, which will be connected with their neighbors
    //by ball joints
    for (int x=0; x < DCOLLIDE_CBS2_GRIDSIZE; x++) {
        for (int y=0; y < DCOLLIDE_CBS2_GRIDSIZE; y++) {
            MyObjectNode* object = new MyObjectNode(getCollisionWorld(), new dcollide::Sphere(1));

            object->createPhysicsBody(mOdeWorld,  (dReal) 0.01);
            addTopLevelObject(object);

            object->setPosition((dcollide::real) -x * DCOLLIDE_CBS2_STEPSIZE + 30,
                                (dcollide::real)  y * DCOLLIDE_CBS2_STEPSIZE +
                                60,
                                (dcollide::real) 250);

            mRigidCloth[x][y] = object;
        }
    }

    std::set<dcollide::MultiMapElement<MyObjectNode, dJointID> > finnished;
    std::set<dcollide::MultiMapElement<MyObjectNode, dJointID> >::iterator finnished_pos;

    mJoints = dJointGroupCreate(0);

    for (int x=0; x < DCOLLIDE_CBS2_GRIDSIZE; x++) {
        for (int y=0; y < DCOLLIDE_CBS2_GRIDSIZE; y++) {

            MyObjectNode* v1 = mRigidCloth[x][y];

            //connect with all neighbors trough joints

            for (int connectX = x-1; connectX <= x+1; connectX++) {
                for (int connectY = y-1; connectY <= y+1; connectY++) {

                    if (   connectX >= 0 && connectX < DCOLLIDE_CBS2_GRIDSIZE
                        && connectY >= 0 && connectY < DCOLLIDE_CBS2_GRIDSIZE) {

                        MyObjectNode* v2 = mRigidCloth[connectX][connectY];

                        finnished_pos = finnished.find(dcollide::MultiMapElement<MyObjectNode, dJointID>(v1, v2));
                        if (finnished_pos == finnished.end() && v1 != v2) {

                            dBodyID body1 = v1->getPhysicsBody();
                            dBodyID body2 = v2->getPhysicsBody();

                            dcollide::Vector3 center  = (v1->getTranslation() + v2->getTranslation()) / 2;

                            dJointID joint = dJointCreateBall(getOdeWorld(), mJoints);
                            //dJointID joint = dJointCreateFixed(getOdeWorld(), mJoints);

                            dJointAttach(joint, body1, body2);
                            dJointSetBallAnchor(joint, center.getX(), center.getY(), center.getZ());
                            //dJointSetFixed(joint);

                            // remember already done pairs of adjecent objects
                            finnished.insert(dcollide::MultiMapElement<MyObjectNode, dJointID>(v1, v2, joint));
                        }
                    }//end if indexes valid
                }
            }//end for connectx
        }
    }
*/
    return true;
}

void RotatingCloth::restart() {

    //Reset position and physic attributes
    mBall->setPosition(-20, 180, 97);
    dBodySetForce( mBall->getPhysicsBody(), 0, 0, 0);
    dBodySetLinearVel(mBall->getPhysicsBody(), 0, 0, 0);
    dBodySetAngularVel(mBall->getPhysicsBody(), 0, 0, 0);

    //Delete the cloth object
    removeObject(mCloth);
    ModelLoader::LoaderDummy loader;
        dcollide::Mesh* clothMesh
            = loader.createRectangleSurface((DCOLLIDE_CBS2_GRIDSIZE-1) * DCOLLIDE_CBS2_STEPSIZE,
                                                 (DCOLLIDE_CBS2_GRIDSIZE-1) * DCOLLIDE_CBS2_STEPSIZE,
                                                 DCOLLIDE_CBS2_STEPSIZE,
                                                 DCOLLIDE_CBS2_STEPSIZE);


    //Recreate it at initial position
    mCloth = new MyDeformableObjectNode(getCollisionWorld(),
                                            clothMesh,
                                            dcollide::PROXYTYPE_DEFORMABLE,
                                            false);

    mCloth->createPhysicsBody(mOdeWorld,
                            DCOLLIDE_CBS2_GRIDSIZE * DCOLLIDE_CBS2_GRIDSIZE * 0.5);

    addTopLevelObject(mCloth);
    mCloth->translate(-75,75,250);
    deleteCollisions();
}

void RotatingCloth::startNextSceneFrame() {
    //mBall->rotate(5.0f, 0, 1, 0);
}

void RotatingCloth::action() {
    //std::cout << "Rotate the BALL!!!" << std::endl;
    //mBall->rotate(90.0f, 0, 1, 0);
    
    dBodySetAngularVel(mBall->getPhysicsBody(), 0,0,5);
    
    //dBodyAddRelForceAtPos(mBall->getPhysicsBody(), 0, 1, 0, 5, 0, 5);

/*
    double radius = 1 + (rand () % 4); //5+-3
    double posX = -90;
    double posY = 30 + (rand () % 40); 
    double posZ = 220 + (rand() % 60);
    //(-90, 20, 220);//-90, [20-70], 220

    MyObjectNode* marble = new MyObjectNode(getCollisionWorld(),
                             new dcollide::Sphere(radius),
                             dcollide::PROXYTYPE_RIGID);

    marble->createPhysicsBody(mOdeWorld,  4);

    addTopLevelObject(marble);
    marble->setPosition(posX, posY, posZ);
    mDampedBodyNodes.push_back(marble);
    mMarbles.push_back(marble);
*/
}

std::string RotatingCloth::getSceneDescription() const {
    return "simulation of a deformable cloth falling onto a rolling sphere.";
}
