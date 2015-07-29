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

#include "collisionresponse.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"
#include "debug.h"

#include "Ogre.h"

#include <world.h>
#include <shapes/shapes.h>
#include <shapes/mesh/meshfactory.h>
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
CollisionResponse::CollisionResponse(Ogre::Root* root)
            : PhysicsSceneBase(root) {
        // Set mRandomSeed to 0 for no variation
        // to a constant value for reproducable variation
        // or to time(NULL) for true pseudo-randomness
        mRandomSeed = 123;
        mPhysicsSimStepsize = 0.01;
    }

    CollisionResponse::~CollisionResponse() {

    }

    dcollide::Vector3 CollisionResponse::initialWorldDimension() const {
        return dcollide::Vector3(2048.0, 2048.0, 2048.0);
    }


/*!
     * Setup the actual scene, i.e. add objects to the \ref dcollide::World
     * object.
     *
     * \return TRUE on success, otherwise FALSE.
 */
    bool CollisionResponse::initializeScene() {
        using namespace dcollide;

        srand(mRandomSeed);

        dWorldSetGravity(mOdeWorld, 0,0,-10);

        Vector3 startPositionMoving(75, -75, -100);
        Vector3 startPositionStatic(75, -75, -150);
        Vector3 rowDistance(0, 50, 0);
        Vector3 columnDistance(-50, 0, 0);

        MeshFactory meshFactory;
        
        Vector3 temporaryStartPosition;
        
        // create the grid of static objects
        for (int i = 0; i < 4; ++i) {
            temporaryStartPosition = startPositionStatic + (rowDistance * i);

            temporaryObjectNode = new MyObjectNode(
                        getCollisionWorld(),
                        new Wedge(10, 20, 20),
                        dcollide::PROXYTYPE_FIXED, true, false);           

            temporaryObjectNode->translate(temporaryStartPosition);
            if (mRandomSeed != 0) {
                Matrix rot;
                rot.rotate(rand(), 1, 0, 0);
                rot.rotate(rand(), 0, 1, 0);
                rot.rotate(rand(), 0, 0, 1);
                temporaryObjectNode->rotate(rot);
            }
            const_cast<ModelLoader::TextureInformation&> (temporaryObjectNode
                    ->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(temporaryObjectNode);

            mStaticObjects.push_back(temporaryObjectNode);

            temporaryStartPosition += columnDistance;

            temporaryObjectNode = new MyObjectNode(
                        getCollisionWorld(),
                        meshFactory.createTetraeder(Vector3(0, 0, 0),
                            Vector3(15,0, 0),Vector3(10, 10, 0),
                            Vector3(10, 10, 15)),
                        dcollide::PROXYTYPE_FIXED, true, false);

            temporaryObjectNode->translate(temporaryStartPosition);
            if (mRandomSeed != 0) {
                Matrix rot;
                rot.rotate(rand(), 1, 0, 0);
                rot.rotate(rand(), 0, 1, 0);
                rot.rotate(rand(), 0, 0, 1);
                temporaryObjectNode->rotate(rot);
            }
            const_cast<ModelLoader::TextureInformation&> (temporaryObjectNode
                    ->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(temporaryObjectNode);
            
            mStaticObjects.push_back(temporaryObjectNode);          
           
            temporaryStartPosition += columnDistance;

            temporaryObjectNode = new MyObjectNode(
                        getCollisionWorld(),
                        new Box(Vector3(10.0, 10.0, 10.0)),
                        dcollide::PROXYTYPE_FIXED, true, false);

            temporaryObjectNode->translate(temporaryStartPosition);
            if (mRandomSeed != 0) {
                Matrix rot;
                rot.rotate(rand(), 1, 0, 0);
                rot.rotate(rand(), 0, 1, 0);
                rot.rotate(rand(), 0, 0, 1);
                temporaryObjectNode->rotate(rot);
            }
            const_cast<ModelLoader::TextureInformation&> (temporaryObjectNode
                    ->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(temporaryObjectNode);

            mStaticObjects.push_back(temporaryObjectNode);

            temporaryStartPosition += columnDistance;

            temporaryObjectNode = new MyObjectNode(
                        getCollisionWorld(),
                        new Sphere(10),
                        dcollide::PROXYTYPE_FIXED, true, false);
            temporaryObjectNode->translate(temporaryStartPosition);
            if (mRandomSeed != 0) {
                Matrix rot;
                rot.rotate(rand(), 1, 0, 0);
                rot.rotate(rand(), 0, 1, 0);
                rot.rotate(rand(), 0, 0, 1);
                temporaryObjectNode->rotate(rot);
            }
            const_cast<ModelLoader::TextureInformation&> (temporaryObjectNode
                    ->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(temporaryObjectNode);

           mStaticObjects.push_back(temporaryObjectNode);  
            
        }

        // create the grid of physically moving objects
        for (int i = 0; i < 4; ++i) {
            temporaryStartPosition = startPositionMoving + (columnDistance * i);

            temporaryObjectNode = new MyObjectNode(
                        getCollisionWorld(),
                        new Wedge(10, 20, 20),
                        dcollide::PROXYTYPE_RIGID);         

            temporaryObjectNode->createPhysicsBody(mOdeWorld,  5);
            temporaryObjectNode->translate(temporaryStartPosition);
            if (mRandomSeed != 0) {
                Matrix rot;
                rot.rotate(rand(), 1, 0, 0);
                rot.rotate(rand(), 0, 1, 0);
                rot.rotate(rand(), 0, 0, 1);
                temporaryObjectNode->rotate(rot);
                Vector3 trans =
                    Vector3(rand()%2 - 1, rand()%2 - 1, rand()%2 - 1);
                temporaryObjectNode->translate(trans);
            }
            const_cast<ModelLoader::TextureInformation&> (temporaryObjectNode
                    ->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(temporaryObjectNode);
           
            mMovingObjects.push_back(temporaryObjectNode); 

            temporaryStartPosition += rowDistance;

            temporaryObjectNode = new MyObjectNode(
                        getCollisionWorld(),
                        meshFactory.createTetraeder(Vector3(0, 0, 0),
                            Vector3(15,0, 0),Vector3(10, 10, 0),
                            Vector3(10, 10, 15)),
                        dcollide::PROXYTYPE_RIGID);

            temporaryObjectNode->createPhysicsBody(mOdeWorld,  5);
            temporaryObjectNode->translate(temporaryStartPosition);
            if (mRandomSeed != 0) {
                Matrix rot;
                rot.rotate(rand(), 1, 0, 0);
                rot.rotate(rand(), 0, 1, 0);
                rot.rotate(rand(), 0, 0, 1);
                temporaryObjectNode->rotate(rot);
                Vector3 trans =
                        Vector3(rand()%2 - 1, rand()%2 - 1, rand()%2 - 1);
                temporaryObjectNode->translate(trans);
            }
            const_cast<ModelLoader::TextureInformation&> (temporaryObjectNode
                    ->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(temporaryObjectNode);
            
            mMovingObjects.push_back(temporaryObjectNode);            
            
            temporaryStartPosition += rowDistance;

            temporaryObjectNode = new MyObjectNode(
                        getCollisionWorld(),
                        new Box(Vector3(10.0, 10.0, 10.0)),
                        dcollide::PROXYTYPE_RIGID);

            temporaryObjectNode->createPhysicsBody(mOdeWorld,  5);
            temporaryObjectNode->translate(temporaryStartPosition);
            if (mRandomSeed != 0) {
                Matrix rot;
                rot.rotate(rand(), 1, 0, 0);
                rot.rotate(rand(), 0, 1, 0);
                rot.rotate(rand(), 0, 0, 1);
                temporaryObjectNode->rotate(rot);
                Vector3 trans =
                        Vector3(rand()%2 - 1, rand()%2 - 1, rand()%2 - 1);
                temporaryObjectNode->translate(trans);
            }
            const_cast<ModelLoader::TextureInformation&> (temporaryObjectNode
                    ->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(temporaryObjectNode);
            
            mMovingObjects.push_back(temporaryObjectNode);            
           
            temporaryStartPosition += rowDistance;

            temporaryObjectNode = new MyObjectNode(
                        getCollisionWorld(),
                        new Sphere(10),
                        dcollide::PROXYTYPE_RIGID);

            temporaryObjectNode->createPhysicsBody(mOdeWorld,  5);
            temporaryObjectNode->translate(temporaryStartPosition);
            if (mRandomSeed != 0) {
                Matrix rot;
                rot.rotate(rand(), 1, 0, 0);
                rot.rotate(rand(), 0, 1, 0);
                rot.rotate(rand(), 0, 0, 1);
                temporaryObjectNode->rotate(rot);
                Vector3 trans =
                        Vector3(rand()%2 - 1, rand()%2 - 1, rand()%2 - 1);
                temporaryObjectNode->translate(trans);
            }
            const_cast<ModelLoader::TextureInformation&> (temporaryObjectNode
                    ->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(temporaryObjectNode);
           
            mMovingObjects.push_back(temporaryObjectNode);  

        }
        
        return true;
    }
    
    // Restart-function (r-key)
    void CollisionResponse::restart() {        

        // delete all static objects
        for(std::list<MyObjectNode*>::iterator j = mStaticObjects.begin(); j != mStaticObjects.end(); j++ ) {
            removeObject(*j);            
        }

        // to clean up the list
        mStaticObjects.clear(); 

        // delete all moving objects        
        for(std::list<MyObjectNode*>::iterator i = mMovingObjects.begin(); i != mMovingObjects.end(); i++ ) {
            removeObject(*i);
        }
        
        // to clean up the list
        mMovingObjects.clear();
        
        // delete all collision results
        deleteCollisions();
        
        // the actual restart
        initializeScene();

    }

    void CollisionResponse::startNextSceneFrame() {
        //TODO simulate friction: calculate velocities and apply damping forces

        //General velocity-based damping force for all balls
        //dReal damping = 0.1;

        //const dReal* velocity = dBodyGetLinearVel(mBall2->getPhysicsBody());
        //dBodyAddForce(mBall2->getPhysicsBody(), -damping * velocity[0], -damping * velocity[1], -damping * velocity[2]);
    }

    std::string CollisionResponse::getSceneDescription() const {
        return "A 'matrix' of static, columnwise different shapes and its \n 'transpose' of physical renderred objects";
    }
/*
    * vim: et sw=4 ts=4
 */
