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

#include "specificationscene.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"

#include <d-collide/debug.h>
#include <d-collide/world.h>
#include <d-collide/shapes/shapes.h>
#include <modelloader/loaderdummy.h>

/*!
 * Construct a new d-collide based scene object.
 *
 * The scene itself is not yet created, only the necessary data
 * structures. Call \ref initializeScene to actually create a scene.
 *
 * \param root A pointer to the ogre root object. Ownership is NOT
 * taken, the pointer will not be deleted by this class.
 */
SpecificationScene::SpecificationScene(Ogre::Root* root)
    : SceneBase(root) {
    }

SpecificationScene::~SpecificationScene() {
}

dcollide::Vector3 SpecificationScene::initialWorldDimension() const {
    return dcollide::Vector3(4096.0, 4096.0, 4096.0);
}

void SpecificationScene::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    parameters.addPrimaryDeformableAlgorithm(dcollide::DEFORMABLE_TYPE_BV_HIERARCHY);
    //parameters.addPrimaryDeformableAlgorithm(dcollide::DEFORMABLE_TYPE_SPATIAL_HASH);
    //parameters.addPrimaryDeformableAlgorithm(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);
}

/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 **/
bool SpecificationScene::initializeScene() {

    ModelLoader::LoaderDummy loader;

    dcollide::Mesh* clothMesh = loader.createRectangleSurface(100,100, 10, 10);
    mCloth = new MyObjectNode(getCollisionWorld(), clothMesh, dcollide::PROXYTYPE_DEFORMABLE, false);
    mCloth->translate(0,0,-1000);
    mCloth->setRepeatedMovementSpeed(3.0f);
    addTopLevelObject(mCloth);

    for (int i = 0; i <= 7; ++i) {

        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),loader.createRectangleSurface(100,100,10,10),
                                                dcollide::PROXYTYPE_DEFORMABLE, false);
        object->translate(-400+i*110,0,-1200);
        addTopLevelObject(object);
    }

    for (int i = 0; i <= 8; ++i) {

        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),loader.createRectangleSurface(100,100,10,10),
                                                dcollide::PROXYTYPE_DEFORMABLE, false);
        object->translate(-400+i*110,200,-1300);
        addTopLevelObject(object);
}

    for (int i = 0; i <= 10; ++i) {

        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),loader.createRectangleSurface(100,100,10,10),
                                            dcollide::PROXYTYPE_RIGID, false);
        object->rotate(90,0,0,1);
        object->translate(-400+i*110,200,-1500);
        addTopLevelObject(object);
    }

    for (int i = 0; i <= 10; ++i) {

        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),loader.createRectangleSurface(100,100,10,10),
                dcollide::PROXYTYPE_RIGID, false);
        object->rotate(45,0,0,1);
        object->translate(-400+i*110,-200,-1700);
        addTopLevelObject(object);
    }

    MyObjectNode* helicopter = create3dsModelWithOneMesh("puma.3ds",dcollide::PROXYTYPE_DEFORMABLE);
    //MyObjectNode* helicopter = create3dsModelWithOneMesh("puma.3ds");
    if (helicopter) {
        helicopter->rotate(90,0,0,1);
        helicopter->rotate(90,0,1,0);
        helicopter->translate(1000,-600,720);
        addTopLevelObject(helicopter);
    }

    //fixed objects
    for (int i = 0; i <= 20; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),new dcollide::Box(40,40,40));

        addTopLevelObject(object);
        object->translate(-500+i*70,0,-1600);
    }

    for (int i = 0; i <= 20; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),new dcollide::Sphere(20,20));

        addTopLevelObject(object);
        object->translate(-500+i*60,0,-1000);
    }

    for (int i = 0; i <= 40; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),new dcollide::Wedge(20,20,20));

        addTopLevelObject(object);
        object->translate(-500+i*40,300,-1600);
    }

    for (int i = 0; i <= 20; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),new dcollide::Wedge(20,20,20));

        addTopLevelObject(object);
        object->translate(-500+i*40,-300,-1600);
    }

    for (int i = 0; i <= 20; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),new dcollide::Wedge(20,20,20));

        addTopLevelObject(object);
        object->translate(-500+i*40,-200,-1600);
    }

    //movable objects
    mMovingHelicopter = create3dsModelWithOneMesh("puma.3ds",dcollide::PROXYTYPE_DEFORMABLE);
    if (mMovingHelicopter) {
        mMovingHelicopter->rotate(90,0,0,1);
        mMovingHelicopter ->rotate(90,0,1,0);
        mMovingHelicopter->translate(1000,0,700);
        mMovingHelicopter->setRepeatedMovementSpeed(2.0f);
        addTopLevelObject(mMovingHelicopter);
    }

    for (int i = 0; i <= 20; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(),new dcollide::Box(40,40,40));

        addTopLevelObject(object);
        object->translate(-520+i*70,-520+i*70,-1600);
        //object->rotate(30.0f,1,1,1);
        object->setRepeatedMovementSpeed(2.0f);
        mMovingBoxesZaxis.push_back(object);
    }

    for (int i = 0; i <= 20; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(), new dcollide::Box(40,40,40));

        addTopLevelObject(object);
        object->translate(-520+i*70,0,-1900+i*70);
        object->setRepeatedMovementSpeed(1.5f);
        mMovingBoxesXaxis.push_back(object);
    }

    for (int i = 0; i <= 40; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(), new dcollide::Box(20,20,20));

        addTopLevelObject(object);
        object->translate(-400+i*30,0,-1500+i*25);
        object->setRepeatedMovementSpeed(2.5f);
        mMovingBoxesYaxis.push_back(object);
    }

    for (int i = 0; i <= 20; ++i) {
        MyObjectNode* object = new MyObjectNode(getCollisionWorld(), new dcollide::Sphere(20,20));

        addTopLevelObject(object);
        object->translate(-520+i*60,-300+i*40,-1000);
        object->setRepeatedMovementSpeed(3.0f);
        mMovingSpheresZaxis.push_back(object);
    }

    float min = -350.0;
    float max = 360.0;
    //float max = 350.0;
    float step = 80.0;
    float size = 20.0;
    for (float z = min; z <= max; z += step) {
        for (float y = min; y <= max; y += step) {
            for (float x = min; x <= max; x += step) {
                dcollide::Box* box = new dcollide::Box(size, size, size);
                MyObjectNode* object = new MyObjectNode(getCollisionWorld(), box);
                addTopLevelObject(object);

                object->translate(x, y, z-1600);
                float speed = 0.1f * (rand() % 100);
                if ((rand() % 2) == 1) {
                    speed *= -1;
                }
                object->setRepeatedMovementSpeed(speed);
                mMovableObjects.push_back(object);
            }
        }
    }

    return true;
    }

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void SpecificationScene::startNextSceneFrame() {

    std::list<MyObjectNode*>::iterator it;
    for (it = mMovingBoxesZaxis.begin(); it != mMovingBoxesZaxis.end(); ++it) {

        float speed = (*it)->getRepeatedMovementSpeed();
        (*it)->translate(0, speed, 0);

        const dcollide::Vector3& t = (*it)->getTranslation();
        if (fabsf(t.getY()) >= 950.0f) {
            (*it)->setRepeatedMovementSpeed(speed * -1.0f);
            }
    }

    for (it = mMovingBoxesXaxis.begin(); it != mMovingBoxesXaxis.end(); ++it) {

        float speed = (*it)->getRepeatedMovementSpeed();
        (*it)->translate(0, 0, speed);

        const dcollide::Vector3& t = (*it)->getTranslation();
        if (fabsf(t.getZ()) <= 400.0f ||
            fabsf(t.getZ()) >= 2400.0f) {
            (*it)->setRepeatedMovementSpeed(speed * -1.0f);
        }
    }

    for (it = mMovingBoxesYaxis.begin(); it != mMovingBoxesYaxis.end(); ++it) {

        float speed = (*it)->getRepeatedMovementSpeed();
        (*it)->translate(speed,0,0);

        const dcollide::Vector3& t = (*it)->getTranslation();
        if (fabsf(t.getX()) >= 801.0f) {
            (*it)->setRepeatedMovementSpeed(speed * -1.0f);
        }
    }

    for (it = mMovingSpheresZaxis.begin(); it != mMovingSpheresZaxis.end(); ++it) {

        float speed = (*it)->getRepeatedMovementSpeed();
        (*it)->translate(0,speed,0);

        const dcollide::Vector3& t = (*it)->getTranslation();
        if (fabsf(t.getY()) >= 550.0f) {
            (*it)->setRepeatedMovementSpeed(speed * -1.0f);
        }
    }

    for (it = mMovableObjects.begin(); it != mMovableObjects.end(); ++it) {
        MyObjectNode* object = *it;

        float speed = (*it)->getRepeatedMovementSpeed();
        float x = speed * 1.0f;
        float y = speed * 2.0f;
        float z = speed * 3.0f;
        object->translate(x, y, z);

        const dcollide::Vector3& t = object->getTranslation();
        if (fabsf(t.getX()) >= 750.0f ||
            fabsf(t.getY()) >= 750.0f ||
            fabsf(t.getZ()) >= 2450.0f ||
            fabsf(t.getZ()) <= 400.0f) {
            object->setRepeatedMovementSpeed(speed * -1.0f);
            }
    }

    mMovingHelicopter->translate(0,mMovingHelicopter->getRepeatedMovementSpeed(),0);
    if (fabsf((mMovingHelicopter->getTranslation()).getX() ) >= 250.0f) {
        mMovingHelicopter->setRepeatedMovementSpeed(mMovingHelicopter->getRepeatedMovementSpeed() * -1.0f);
    }

    mCloth->translate(mCloth->getRepeatedMovementSpeed(),0,0);
    if (fabsf((mCloth->getTranslation()).getX() ) >= 300.0f) {
       mCloth->setRepeatedMovementSpeed(mCloth->getRepeatedMovementSpeed() * -1.0f);
    }
}

std::string SpecificationScene::getSceneDescription() const {
    return "A scene with exact attributes from our specifications";
}

/*
 * vim: et sw=4 ts=4
 */
