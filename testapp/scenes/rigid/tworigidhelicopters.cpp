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

#include "tworigidhelicopters.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"

#include <d-collide/debug.h>
#include <d-collide/world.h>
#include <d-collide/shapes/shapes.h>

TwoRigidHelicopters::TwoRigidHelicopters(Ogre::Root* root)
        : SceneBase(root) {
}

TwoRigidHelicopters::~TwoRigidHelicopters() {
}

dcollide::Vector3 TwoRigidHelicopters::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 **/
bool TwoRigidHelicopters::initializeScene() {
    MyObjectNode* helicopter1 = create3dsModel("puma.3ds");
    MyObjectNode* helicopter2 = create3dsModel("puma.3ds");
    if (helicopter1) {
        addTopLevelObject(helicopter1);

        helicopter1->translate(dcollide::Vector3(-500.0, 0.0, -100.0));
        helicopter1->setRepeatedMovement(dcollide::Vector3(-500.0f, 0.0f, -100.0f), dcollide::Vector3(500.0f, 0.0f, -100.0f), 2.0f);
        mMovableObjects.push_back(helicopter1);
    }
    if (helicopter2) {
        addTopLevelObject(helicopter2);

        helicopter2->translate(dcollide::Vector3(-400.0, 0.0, -100.0));
        helicopter2->setRepeatedMovement(dcollide::Vector3(-500.0f, 0.0f, -100.0f), dcollide::Vector3(500.0f, 0.0f, -100.0f), 2.0f);
        mMovableObjects.push_back(helicopter2);
    }

    return true;
}

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void TwoRigidHelicopters::startNextSceneFrame() {
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

std::string TwoRigidHelicopters::getSceneDescription() const {
    return "Two (moving) rigid helicopters that collide with each other all the time.";
}

/*
 * vim: et sw=4 ts=4
 */
