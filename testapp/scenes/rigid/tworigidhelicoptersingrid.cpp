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

#include "tworigidhelicoptersingrid.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"

#include <d-collide/debug.h>
#include <d-collide/world.h>
#include <d-collide/shapes/shapes.h>

static const dcollide::real helicopterDistanceX = 200.0;
static const dcollide::real pathLength = 1000.0;
static const dcollide::real circleRadius = 300.0;

TwoRigidHelicoptersInGrid::TwoRigidHelicoptersInGrid(Ogre::Root* root)
        : SceneBase(root) {
    mHelicopter1 = 0;
    mHelicopter2 = 0;
    mRotate = false;
}

TwoRigidHelicoptersInGrid::~TwoRigidHelicoptersInGrid() {
}

dcollide::Vector3 TwoRigidHelicoptersInGrid::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 **/
bool TwoRigidHelicoptersInGrid::initializeScene() {
    createManyRigidBoxes();

    mHelicopter1 = create3dsModel("puma.3ds");
    mHelicopter2 = create3dsModel("puma.3ds");

    if (mHelicopter1) {
        addTopLevelObject(mHelicopter1);
        mMovableObjects.push_back(mHelicopter1);
    }
    if (mHelicopter2) {
        addTopLevelObject(mHelicopter2);
        mMovableObjects.push_back(mHelicopter2);
    }

    restart();

    std::cout << dc_funcinfo << "done" << std::endl;

    return true;
}

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void TwoRigidHelicoptersInGrid::startNextSceneFrame() {
    if (!mRotate) {
        // dont rotate, translate only
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
    } else {
        // rotate
        const dcollide::real degree = 1.0;
        const dcollide::real degreeRadians = degree * M_PI / 180.0;

        dcollide::real radiusHelicopter1 = circleRadius;
        dcollide::real radiusHelicopter2 = circleRadius + helicopterDistanceX;
        dcollide::real speedHelicopter1 = sqrt(radiusHelicopter1 * radiusHelicopter1 * (2.0 - 2.0 * cos(degreeRadians)));
        dcollide::real speedHelicopter2 = sqrt(radiusHelicopter2 * radiusHelicopter2 * (2.0 - 2.0 * cos(degreeRadians)));

        mHelicopter1->rotate(degree, 0.0, 0.0, 1.0);
        mHelicopter2->rotate(degree, 0.0, 0.0, 1.0);

        mHelicopter1->translate(0.0, speedHelicopter1, 0.0);
        mHelicopter2->translate(0.0, speedHelicopter2, 0.0);
    }
}

std::string TwoRigidHelicoptersInGrid::getSceneDescription() const {
    return "Two (moving) rigid helicopters that collide with the grid, and with each other.";
}

void TwoRigidHelicoptersInGrid::restart() {
    const dcollide::Vector3 start(-pathLength / 2.0, 0.0, -100.0);
    const dcollide::Vector3 end(pathLength / 2.0 - helicopterDistanceX, 0.0, -100.0);
    const dcollide::Vector3 helicopterDistance(helicopterDistanceX, 0.0, 0.0);

    if (mHelicopter1) {
        mHelicopter1->resetTransformation();

        if (!mRotate) {
            mHelicopter1->translate(start);
        } else {
            mHelicopter1->translate(end);
        }
        mHelicopter1->setRepeatedMovement(start, end, 2.0f);
    }
    if (mHelicopter2) {
        mHelicopter2->resetTransformation();

        if (!mRotate) {
            mHelicopter2->translate(start + helicopterDistance);
        } else {
            mHelicopter2->translate(end + helicopterDistance);
        }
        mHelicopter2->setRepeatedMovement(start + helicopterDistance, end + helicopterDistance, 2.0f);
    }
}

void TwoRigidHelicoptersInGrid::action() {
    mRotate = !mRotate;
    restart();
}

/*
 * vim: et sw=4 ts=4
 */
