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

#include "penetrationdepthscene.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"

#include "Ogre.h"
#include "axisobject.h"

#include <d-collide/debug.h>
#include <d-collide/debugstream.h>
#include <d-collide/world.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/shapes/mesh/meshfactory.h>
#include <d-collide/worldcollisions.h>
#include <d-collide/collisioninfo.h>
#include <d-collide/proxy.h>
#include <d-collide/boundingvolumes/boundingvolume.h>
#include <d-collide/boundingvolumes/obb.h>

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
    PenetrationDepthScene::PenetrationDepthScene(Ogre::Root* root)
            : SceneBase(root) {
    }

    PenetrationDepthScene::~PenetrationDepthScene() {
    }

    dcollide::Vector3 PenetrationDepthScene::initialWorldDimension() const {
        return dcollide::Vector3(2048.0, 2048.0, 2048.0);
    }

    void PenetrationDepthScene::addObject( dcollide::Shape* shape,
            const dcollide::Vector3& startPosition,
            bool isMoving, dcollide::Vector3 speed) {
        using namespace dcollide;

        //calculate some random-looking, but reproducable rotation
        Matrix rot;
        rot.rotate(startPosition.getX() / 10, 1, 0, 0);
        rot.rotate(startPosition.getY(), 0, 1, 0);
        rot.rotate(startPosition.getZ(), 0, 0, 1);

        MyObjectNode* shapeNode;
        if (isMoving) {
            shapeNode = new MyObjectNode(getCollisionWorld(),
                    shape, PROXYTYPE_RIGID, true, false);
            shapeNode->rotate(rot);
            const_cast<ModelLoader::TextureInformation&>
                    (shapeNode->getTextureInformation()).setTransparent(0.5);
            MyObjectNode* container = new MyObjectNode(getCollisionWorld(),
                    (dcollide::Shape*)0, PROXYTYPE_RIGID, true, false);
            mMovingObjects.push_back(container);
            mObjectSpeeds.insert(std::make_pair(container, speed));
            container->addChild(shapeNode);
            container->translate(startPosition);
            addTopLevelObject(container);
        /*    dcollide::debug() << "m-M-POS: " << container->getWorldPosition();
            dcollide::debug() << "m-B-CENTER: " <<
                static_cast<const Obb*>(container->getProxy()->getToplevelProxy()->getBvHierarchyNode()->getBoundingVolume())->getCenter();
            dcollide::debug() << "m-B-POS: " <<
                static_cast<const
                Obb*>(container->getProxy()->getToplevelProxy()->getBvHierarchyNode()->getBoundingVolume())->getState().getPosition();
            dcollide::debug() << "--";*/

        } else {
            shapeNode = new MyObjectNode(getCollisionWorld(),
                    shape, PROXYTYPE_FIXED, true, false);
            shapeNode->translate(startPosition);
            shapeNode->rotate(rot);
            const_cast<ModelLoader::TextureInformation&>
                    (shapeNode->getTextureInformation()).setTransparent(0.5);
            addTopLevelObject(shapeNode);
            /*dcollide::debug() << "f-M-POS: " << shapeNode->getWorldPosition();
            dcollide::debug() << "f-B-CENTER: " <<
                static_cast<const Obb*>(shapeNode->getProxy()->getToplevelProxy()->getBvHierarchyNode()->getBoundingVolume())->getCenter();
            dcollide::debug() << "f-B-POS: " <<
                static_cast<const
                Obb*>(shapeNode->getProxy()->getToplevelProxy()->getBvHierarchyNode()->getBoundingVolume())->getState().getPosition();
            dcollide::debug() << "--";*/
        }
    }

    /*!
     * Setup the actual scene, i.e. add objects to the \ref dcollide::World
     * object.
     *
     * \return TRUE on success, otherwise FALSE.
     **/
    bool PenetrationDepthScene::initializeScene() {
        //Display Axis
        Ogre::String objectname = "Axis";
        AxisObject axisObject;
        //hint: last parameter is the size/scale of the axis object
        Ogre::ManualObject* manualObject = axisObject.createAxis(mSceneManager,
                                                                objectname, 10);
        mSceneManager->getRootSceneNode()->createChildSceneNode()
                                        ->attachObject(manualObject);

        using namespace dcollide;
        //initial setup:
        // a column of moving objects
        // a grid of static objects
        // the column will move through the grid causing all types of collisions

        Vector3 startPositionMoving(-150, -70, 0);
        Vector3 startPositionStatic(80, -70, 0);

        // The Factory to generate meshes of special shapes:
        MeshFactory meshFactory;

        // direction and speed for the moving objects
        Vector3 moveSpeed(0.2, 0, 0);

        //FIXME: add a addStaticObjects function and remove this vector
        Vector3 staticSpeed(0, 0, 0);

        Vector3 rowDistance(0, 40, 0);
        Vector3 columnDistance(-40, 0, 0);

        // create the column of moving objects

        addObject(  new Wedge(10, 20, 20),
                    startPositionMoving,
                    true,
                    moveSpeed);
//*
        startPositionMoving += rowDistance;

        addObject(  meshFactory.createTetraeder(Vector3(0, 0, 0),
                        Vector3(15,0, 0),Vector3(10, 10,0),
                        Vector3(10,10,15)),
                    startPositionMoving,
                    true,
                    moveSpeed);

        startPositionMoving += rowDistance;

        addObject(  new Box(Vector3(10.0, 10.0, 10.0)),
                    startPositionMoving,
                    true,
                    moveSpeed);

        startPositionMoving += rowDistance;

        addObject(  new Sphere(10),
                    startPositionMoving,
                    true,
                    moveSpeed);

        startPositionMoving += rowDistance;

        addObject(  new Cylinder(2,10,1),
                    startPositionMoving,
                    true,
                    moveSpeed);

        startPositionMoving += rowDistance;

        addObject(  new Cone(2,10,1),
                    startPositionMoving,
                    true,
                    moveSpeed);

        startPositionMoving += rowDistance;
        //TODO other meshes (more complex/concave/etc)
//*/
        // create the grid of static objects
        Vector3 startPositionStatic2;

        for (int i = 0; i < 6; ++i) {
            startPositionStatic2 = startPositionStatic + (rowDistance * i);

            addObject(  new Wedge(10, 20, 20),
                        startPositionStatic2,
                        false,
                        staticSpeed);

            startPositionStatic2 += columnDistance;

            addObject(  meshFactory.createTetraeder(Vector3(0, 0, 0),
                        Vector3(15,0, 0),Vector3(10, 10,0),
                        Vector3(10,10,15)),
                        startPositionStatic2,
                        false,
                        staticSpeed);

            startPositionStatic2 += columnDistance;

            addObject(  new Box(Vector3(10.0, 10.0, 10.0)),
                        startPositionStatic2,
                        false,
                        staticSpeed);

            startPositionStatic2 += columnDistance;

            addObject(  new Sphere(10),
                        startPositionStatic2,
                        false,
                        staticSpeed);

            startPositionStatic2 += columnDistance;

            addObject(  new Cylinder(2,10,1),
                        startPositionStatic2,
                        false,
                        staticSpeed);

            startPositionStatic2 += columnDistance;

           // addObject(  new Cone(2,10,1),
           //             startPositionStatic2,
           //             false,
           //             staticSpeed);
          }

        std::cout << dc_funcinfo << "done" << std::endl;
        return true;
    }

    /*!
     * Start a new frame of the scene, i.e. moves all movable object by a bit.
     *
     * This is NOT necessarily equal to a graphic frame! In fact the scene
     * frames should depend on time only, not on CPU or GPU power as the graphic
     * frames do.
     */
    void PenetrationDepthScene::startNextSceneFrame() {
        //Objects will move until there is a narrowphase collision
        //alternatively, revert the direction at positions with X ~= +-100
        for (std::list<MyObjectNode*>::iterator it
                = mMovingObjects.begin(); it != mMovingObjects.end(); ++it) {
            MyObjectNode* object = *it;
            dcollide::Vector3 speed = mObjectSpeeds[object];
            object->translate(speed);
          /*  dcollide::debug() << "m-M-POS: " << object->getWorldPosition();
            dcollide::debug() << "m-B-CENTER: " <<
                static_cast<const dcollide::Obb*>(object->getProxy()->getToplevelProxy()->getBvHierarchyNode()->getBoundingVolume())->getCenter();
            dcollide::debug() << "m-B-POS: " <<
                static_cast<const dcollide::Obb*>(object->getProxy()->getToplevelProxy()->getBvHierarchyNode()->getBoundingVolume())->getState().getPosition();
            dcollide::debug() << "--";*/

        }

        //now check if they have moved "too far" and stop them
        for (std::list<MyObjectNode*>::iterator it = mMovingObjects.begin();
                it != mMovingObjects.end(); ++it) {
            MyObjectNode* object = *it;
            dcollide::Vector3 speed = mObjectSpeeds[object];
            if ((speed.getX() > 0 &&
                    object->getWorldPosition().getX() > 100.0) ||
                    (speed.getX() < 0 &&
                    object->getWorldPosition().getX() < -100.0)) {
                //it = mMovingObjects.erase(it);
                //revert speed
                speed = speed *-1;
                mObjectSpeeds[object] = speed;
            }
        }
    }

    std::string PenetrationDepthScene::getSceneDescription() const {
        return "A row of shapes moves through columns of the same shapes to visualize \nall combinations of shape-shape collisions with varying penetration depths";
    }


    void PenetrationDepthScene::applyCollisions(
            const dcollide::WorldCollisions& collisions) {
        SceneBase::applyCollisions(collisions);
    }

/*
 * vim: et sw=4 ts=4
 */
