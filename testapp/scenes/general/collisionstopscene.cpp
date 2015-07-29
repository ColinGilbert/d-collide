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

#include "collisionstopscene.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"

#include "Ogre.h"
#include "axisobject.h"

#include <d-collide/debug.h>
#include <d-collide/world.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/worldcollisions.h>
#include <d-collide/collisioninfo.h>
#include <d-collide/proxy.h>

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
CollisionStopScene::CollisionStopScene(Ogre::Root* root)
        : SceneBase(root) {
}

CollisionStopScene::~CollisionStopScene() {
}

dcollide::Mesh* createTetraeder() {
        //construct a simple mesh manually, and test adjacencies afterwards
        //Mesh-Tetraeder
/*
        3 \
       / \ \
      /  \  \
     /  --\---2
    0----  \ /
       -----1
*/
    using namespace dcollide;
    Vertex* v0 = new Vertex(Vector3(0, 0, 0));
    Vertex* v1 = new Vertex(Vector3(15, 0, 0));
    Vertex* v2 = new Vertex(Vector3(10, 10, 0));
    Vertex* v3 = new Vertex(Vector3(10, 10, 15));

    Triangle* t0 = new Triangle(v0, v2, v1);
    Triangle* t1 = new Triangle(v0, v1, v3);
    Triangle* t2 = new Triangle(v1, v2, v3);
    Triangle* t3 = new Triangle(v2, v0, v3);

    std::vector<Vertex*> vertices;
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);

    std::vector<Triangle*> triangles;
    triangles.push_back(t0);
    triangles.push_back(t1);
    triangles.push_back(t2);
    triangles.push_back(t3);

    return new Mesh(vertices, triangles);
}

dcollide::Vector3 CollisionStopScene::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

void CollisionStopScene::addObject( dcollide::Shape* shape,
                                    dcollide::Vector3& startPosition,
dcollide::Vector3 speed) {
    using namespace dcollide;

    MyObjectNode* shapeNode = new MyObjectNode(getCollisionWorld(), shape, PROXYTYPE_RIGID, true, false);

    //use some random-looking, but reproducable rotation
    Matrix rot;
    rot.rotate(startPosition.getX() / 10, 1, 0, 0);
    rot.rotate(startPosition.getY(), 0, 1, 0);
    rot.rotate(startPosition.getZ(), 0, 0, 1);
    shapeNode->rotate(rot);


    MyObjectNode* container = new MyObjectNode(getCollisionWorld(), (dcollide::Shape*)0, PROXYTYPE_RIGID, true, false);
    container->addChild(shapeNode);
    addTopLevelObject(container);
    container->translate(startPosition);


    mMovingObjects.push_back(container);
    mObjectSpeeds.insert(std::make_pair(container, speed));
}

/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 **/
bool CollisionStopScene::initializeScene() {
    //Display Axis
    Ogre::String objectname = "Axis";
    AxisObject axisObject;
    //hint: last parameter is the size/scale of the axis object
    Ogre::ManualObject* manualObject = axisObject.createAxis(mSceneManager,
                                                                objectname, 10);
    mSceneManager->getRootSceneNode()->createChildSceneNode()
                                        ->attachObject(manualObject);

    using namespace dcollide;
    //initial setup: different objects pairs at x = +- 100, stacked at y-axis
    //the pairs will move along the x-axis and collide somewhere around x==0

    Vector3 startPositionLeft(-100, -70, 0);
    Vector3 startPositionRight(100, -70, 0);

    Vector3 directionLeft(1, 0, 0); //direction for the left object
    Vector3 directionRight(-1, 0, 0); //direction for the left object

    Vector3 rowDistance(0, 40, 0);

    //First row: two rotated boxes
    addObject(  new Box(Vector3(10.0, 10.0, 10.0)),
                startPositionLeft,
                directionLeft * 0.2);


    addObject(  new Box(Vector3(10.0, 10.0, 10.0)),
                startPositionRight,
                directionRight * 0.2);


    startPositionLeft += rowDistance;
    startPositionRight += rowDistance;

    addObject(  createTetraeder(),
                startPositionLeft,
                directionLeft * 0.2);

    addObject(  createTetraeder(),
                startPositionRight,
                directionRight * 0.2);

    startPositionLeft += rowDistance;
    startPositionRight += rowDistance;


    //4th row: Box with Mesh
    addObject(  new Box(Vector3(8, 10, 4)),
                startPositionLeft,
                directionLeft * 0.1);

    addObject(  createTetraeder(),
                startPositionRight,
                directionRight * 0.3);

    startPositionLeft += rowDistance;
    startPositionRight += rowDistance;

    //5th row: Box with Sphere
    addObject(  new Box(Vector3(8, 10, 4)),
                startPositionLeft,
                directionLeft * 0.1);

    addObject(  new Sphere(10),
                startPositionRight,
                directionRight * 0.1);

    startPositionLeft += rowDistance;
    startPositionRight += rowDistance;


    //6th row: Sphere-Mesh
    addObject(  createTetraeder(),
                startPositionLeft,
                directionLeft * 0.2);

    addObject(  new Sphere(10),
                startPositionRight,
                directionRight * 0.1);

    startPositionLeft += rowDistance;
    startPositionRight += rowDistance;

    //7th row: Sphere-Mesh
    addObject(  new Sphere(18),
                startPositionLeft,
                directionLeft * 0.2);

    addObject(  new Sphere(10),
                startPositionRight,
                directionRight * 0.2);

    startPositionLeft += rowDistance;
    startPositionRight += rowDistance;
    
    //8th row: Wedges
    addObject(  new Wedge(10, 20, 20),
                startPositionLeft,
                directionLeft * 0.12);

    addObject(  new Wedge(30, 10, 20),
                startPositionRight,
                directionRight * 0.2);

    startPositionLeft += rowDistance;
    startPositionRight += rowDistance;
    //TODO other meshes (more complex/concave/etc)

    std::cout << dc_funcinfo << "done" << std::endl;
    return true;
}

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void CollisionStopScene::startNextSceneFrame() {
    //Objects will move until there is a narrowphase collision
    //alternatively, revert the direction at positions with X ~= +-100
    for (std::list<MyObjectNode*>::iterator it
                = mMovingObjects.begin(); it != mMovingObjects.end(); ++it) {
        MyObjectNode* object = *it;
        dcollide::Vector3 speed = mObjectSpeeds[object];
        object->translate(speed);

    }

    //now check if they have moved "too far" and stop them
    for (std::list<MyObjectNode*>::iterator it
                = mMovingObjects.begin(); it != mMovingObjects.end(); ++it) {
        MyObjectNode* object = *it;
        dcollide::Vector3 speed = mObjectSpeeds[object];
        if (   (speed.getX() > 0 && object->getWorldPosition().getX() >  100.0)
            || (speed.getX() < 0 && object->getWorldPosition().getX() < -100.0)) {
            //it = mMovingObjects.erase(it);
            //revert speed
            speed = speed *-1;
            mObjectSpeeds[object] = speed;
        }
    }
}

std::string CollisionStopScene::getSceneDescription() const {
    return "In this scene, objects will move until they collide and then stop.";
}


void CollisionStopScene::applyCollisions(const dcollide::WorldCollisions& collisions) {
    SceneBase::applyCollisions(collisions);
    //stop objects
    for (std::list<dcollide::CollisionInfo>::const_iterator iter = collisions.getNarrowPhaseCollisions().begin();
        iter != collisions.getNarrowPhaseCollisions().end(); ++iter) {
        //stop the proxies that participate in this collision
        dcollide::Proxy* p1 = (*iter).penetratingProxy->getParent();
        dcollide::Proxy* p2 = (*iter).penetratedProxy->getParent();
        for (std::list<MyObjectNode*>::iterator it
                    = mMovingObjects.begin(); it != mMovingObjects.end(); ++it) {
            MyObjectNode* object = *it;
            if ( object->getProxy() == p1 || object->getProxy() == p2) {
                it = mMovingObjects.erase(it);
                if (it == mMovingObjects.end()) {
                    break;
                }
            }
        }

    }
}
/*
 * vim: et sw=4 ts=4
 */
