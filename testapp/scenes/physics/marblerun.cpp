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

#include "marblerun.h"
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
MarblerunScene::MarblerunScene(Ogre::Root* root)
    : PhysicsSceneBase(root) {

    mPhysicsSimStepsize = 0.03;
    mLinearDampingFactor = 0.001;
    mAngularDampingFactor = 0.005;
    }

    MarblerunScene::~MarblerunScene() {

    }


    dcollide::Vector3 MarblerunScene::initialWorldDimension() const {
        return dcollide::Vector3(2048.0, 2048.0, 2048.0);
    }


/*!
     * Setup the actual scene, i.e. add objects to the \ref dcollide::World
     * object.
     *
     * \return TRUE on success, otherwise FALSE.
 */
    bool MarblerunScene::initializeScene() {
        dWorldSetGravity(mOdeWorld, 0,0,-10);


        //Put the "static environment" proxies into a parent to keep them from
        //colliding
        MyObjectNode* environment = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);


        //first Ramp: 
        /*
        mRamp = new MyObjectNode( getCollisionWorld(),
                new dcollide::Wedge(200, 300, 100),
                                  dcollide::PROXYTYPE_FIXED, true, false);
        //the ramp inclines in y-direction
        mRamp->translate(dcollide::Vector3(-100, 50, 0));
        */
        MyObjectNode* ramp1 = new MyObjectNode( getCollisionWorld(),
                                        new dcollide::Box(120, 50, 5),
                                        dcollide::PROXYTYPE_FIXED, true, false);
        //the ramp declines in positive x-direction
        ramp1->translate(dcollide::Vector3(-100, 25, 200));
        ramp1->rotate(20, 0, 1,0);

        environment->addChild(ramp1);

        MyObjectNode* ramp2 = new MyObjectNode( getCollisionWorld(),
                                        new dcollide::Box(130, 50, 5),
                                        dcollide::PROXYTYPE_FIXED, true, false);
        //the ramp declines in positive x-direction
        ramp2->translate(dcollide::Vector3(0, 25, 130));
        ramp2->rotate(-15, 0, 1,0);
        environment->addChild(ramp2);

        //funnel mesh
        std::vector<dcollide::Triangle*> triangles;
        std::vector<dcollide::Vertex*> vertices;

        int ringsections = 6;
        double upperRadius = 100;
        double middleRadius= 15;
        double lowerRadius = 2;

        const double angleIncrement = (2.0 * M_PI / ringsections);

        for (int i = 0; i < ringsections; i++){
            vertices.push_back(new dcollide::Vertex(cos(i * angleIncrement) * upperRadius, sin(i*angleIncrement) * upperRadius, 50));
            vertices.push_back(new dcollide::Vertex(cos(i * angleIncrement) * middleRadius, sin(i*angleIncrement) * middleRadius, 0));
            vertices.push_back(new dcollide::Vertex(cos(i * angleIncrement) * lowerRadius, sin(i*angleIncrement) * lowerRadius, -40));
        }

        //triangles
        for (int i = 0; i < ringsections; i++){
            //inner triangles (top to middle)
            triangles.push_back(new dcollide::Triangle(vertices[(3*i+3)%(ringsections*3)], vertices[3*i+1], vertices[3*i]));
            triangles.push_back(new dcollide::Triangle(vertices[3*i+1], vertices[(3*i+3)%(ringsections*3)], vertices[(3*i+4)%(ringsections*3)]));

            //inner triangles (middle to bottom)
            triangles.push_back(new dcollide::Triangle(vertices[(3*i+4)%(ringsections*3)], vertices[3*i+2], vertices[3*i+1]));
            triangles.push_back(new dcollide::Triangle(vertices[3*i+2], vertices[(3*i+4)%(ringsections*3)], vertices[(3*i+5)%(ringsections*3)]));
        }
        MyObjectNode* funnel = new MyObjectNode( getCollisionWorld(),
                                        new dcollide::Mesh(vertices, triangles),
                                        dcollide::PROXYTYPE_FIXED,
                                        false);
        funnel->translate(-50, 0, 50);
        environment->addChild(funnel);

        //cylinder

        addTopLevelObject(environment);

        //first marble

        mMarble1 = new MyObjectNode(getCollisionWorld(),
                                 new dcollide::Sphere(5),
                                 //new dcollide::Box(5,5,5),
                                 dcollide::PROXYTYPE_RIGID);

        mMarble1->createPhysicsBody(mOdeWorld,  4);

        addTopLevelObject(mMarble1);
        mDampedBodyNodes.push_back(mMarble1);
//*
        //cylinder supposed to roll down the lanes
        mCylinder = new MyObjectNode(getCollisionWorld(),
                                 new dcollide::Cylinder(2,10, 1),
                                 dcollide::PROXYTYPE_RIGID);

        mCylinder->createPhysicsBody(mOdeWorld,  4);
        mCylinder->rotate(90,1,0,0);

        addTopLevelObject(mCylinder);
        mDampedBodyNodes.push_back(mCylinder);
// */
/*
        //cone supposed to "roll" down the lanes
        mCone = new MyObjectNode(getCollisionWorld(),
                                 new dcollide::Cone(2,10, 1),
                                 dcollide::PROXYTYPE_RIGID);

        mCone->createPhysicsBody(mOdeWorld,  4);
        mCone->rotate(90,1,0,0);

        addTopLevelObject(mCone);
        mDampedBodyNodes.push_back(mCone);
// */
         restart();
        return true;
    }

    void MarblerunScene::restart() {
        //set inital positions
        mCylinder->setPosition(-90, 50, 220);
        mCylinder->rotate(90,1,0,0);
   //     mCone->setPosition(-90, 50, 220);
   //     mCone->rotate(90,1,0,0);
        mMarble1->setPosition(-90, 70, 220);

        //nullify body velocity, accelleration and forces
        dBodySetForce(mCylinder->getPhysicsBody(), 0,0,0);
     //   dBodySetForce(mCone->getPhysicsBody(), 0,0,0);
        dBodySetForce(mMarble1->getPhysicsBody(), 0,0,0);

        dBodySetAngularVel(mCylinder->getPhysicsBody(), 0,0,0);
     //   dBodySetAngularVel(mCone->getPhysicsBody(), 0,0,0);
        dBodySetAngularVel(mMarble1->getPhysicsBody(), 0,0,0);

        dBodySetLinearVel(mCylinder->getPhysicsBody(), 0,0,0);
     //   dBodySetLinearVel(mCone->getPhysicsBody(), 0,0,0);
        dBodySetLinearVel(mMarble1->getPhysicsBody(), 0,0,0);

        //delete all marbles from mMarbles list
        for (std::list<MyObjectNode*>::iterator iter = mMarbles.begin();
        iter != mMarbles.end(); ++iter){
            MyObjectNode* marble = *iter;
            removeObject(marble);
        }

        mMarbles.clear();

        //remove extra marbles from damping list.
        mDampedBodyNodes.clear();
        mDampedBodyNodes.push_back(mMarble1);


        deleteCollisions();
    }
    /*!
     * \brief Scene action: insert a new marble at the top of the run
     * 
     * the marble will have a random radius and starting position
     */
    void MarblerunScene::action() {
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
    }
    void MarblerunScene::startNextSceneFrame() {

    }

    std::string MarblerunScene::getSceneDescription() const {
        return "Some spheres rolling down a marble run.\nUses ODE physics.";
    }
/*
    * vim: et sw=4 ts=4
 */
