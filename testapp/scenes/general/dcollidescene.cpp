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

#include "dcollidescene.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"

#include <d-collide/debug.h>
#include <d-collide/world.h>
#include <d-collide/shapes/shapes.h>

/*!
 * Construct a new d-collide based scene object.
 *
 * The scene itself is not yet created, only the necessary data
 * structures. Call \ref initializeScene to actually create a scene.
 *
 * \param root A pointer to the ogre root object. Ownership is NOT
 * taken, the pointer will not be deleted by this class.
 */
DCollideScene::DCollideScene(Ogre::Root* root)
        : SceneBase(root) {
}

DCollideScene::~DCollideScene() {
}

dcollide::Vector3 DCollideScene::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 **/
bool DCollideScene::initializeScene() {
    createManyRigidBoxes();

    MyObjectNode* model = create3dsModel("puma.3ds");
    if (model) {
        addTopLevelObject(model);

        model->translate(dcollide::Vector3(-500.0, 0.0, -100.0));
        model->setRepeatedMovement(dcollide::Vector3(-500.0f, 0.0f, -100.0f), dcollide::Vector3(500.0f, 0.0f, -100.0f), 2.0f);
        mMovableObjects.push_back(model);
    }


     //actually RIGID at the moment
    MyObjectNode* deformableModel = create3dsModelWithOneMesh("puma.3ds", dcollide::PROXYTYPE_RIGID);
    if (deformableModel) {
        addTopLevelObject(deformableModel);

        deformableModel->translate(dcollide::Vector3(0.0, 500.0, -100.0));
    }

#if 0
    MyObjectNode* bigModelRigid = createPlyModel("bunny_lower_res2.ply", dcollide::PROXYTYPE_RIGID, 750.0f);
    if (bigModelRigid) {
        addTopLevelObject(bigModelRigid);
        bigModelRigid->translate(dcollide::Vector3(0.0, -500.0, 0.0));
        bigModelRigid->setRepeatedMovement(dcollide::Vector3(0.0f, -500.0f, 0.0f), dcollide::Vector3(0.0f, 500.0f, 0.0f), 2.0f);
        mMovableObjects.push_back(bigModelRigid);
    }
    MyObjectNode* bigModelDeformable = createPlyModel("bunny_lower_res4.ply", dcollide::PROXYTYPE_DEFORMABLE, 750.0f);
    if (bigModelDeformable) {
        addTopLevelObject(bigModelDeformable);
        bigModelDeformable->translate(dcollide::Vector3(500.0, -500.0, 0.0));
    }
#endif

    std::cout << dc_funcinfo << "done" << std::endl;
    return true;
}

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void DCollideScene::startNextSceneFrame() {
    std::list<MyObjectNode*>::iterator it;
    for (it = mMovableObjects.begin(); it != mMovableObjects.end(); ++it) {
        MyObjectNode* object = *it;
        const dcollide::Vector3& start = object->getRepeatedMovementStart();
        const dcollide::Vector3& end = object->getRepeatedMovementEnd();
        float speed = object->getRepeatedMovementSpeed();

        dcollide::Vector3 currentPosition = object->getWorldPosition();

        if (speed == 0.0f) {
            continue;
        }
        dcollide::Vector3 goal;
        if (speed > 0.0f) {
            // moving towards end
            goal = end;

        } else {
            // moving towards start
            goal = start;
        }


        dcollide::Vector3 direction = goal - currentPosition;
        if (direction.length() > 1.0f) {
            direction.normalize();
        } else {
            // turn around in the next frame
            object->setRepeatedMovementSpeed(speed * -1.0f);

            // in this frame move by exactly the remaining distance to the goal
            speed = 1.0f;
        }

        object->translate(direction * fabsf(speed));
    }
}

std::string DCollideScene::getSceneDescription() const {
    return "Default d-collide scene to test various aspects of the library.";
}

/*
 * vim: et sw=4 ts=4
 */
