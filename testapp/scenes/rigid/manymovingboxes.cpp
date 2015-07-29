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

#include "manymovingboxes.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"

#include <d-collide/debug.h>
#include <d-collide/world.h>
#include <d-collide/shapes/shapes.h>

ManyMovingBoxes::ManyMovingBoxes(Ogre::Root* root)
        : SceneBase(root) {
    mMoveRight = true;
    mHasMoved = 0.0f;
}

ManyMovingBoxes::~ManyMovingBoxes() {
}

dcollide::Vector3 ManyMovingBoxes::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 **/
bool ManyMovingBoxes::initializeScene() {
    createManyMovingRigidBoxes(0.0f);
    createManyMovingRigidBoxes(7.0f, 2.0f);
    createManyMovingRigidBoxes(15.0f, 5.0f);

    std::cout << dc_funcinfo << "done" << std::endl;

    return true;
}

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void ManyMovingBoxes::startNextSceneFrame() {
    const float step = 1.0f;
    float xStep = step;
    if (!mMoveRight) {
        xStep = -step;
    }
    std::list<MyObjectNode*>::iterator it;
    for (it = mMovableObjects.begin(); it != mMovableObjects.end(); ++it) {
        MyObjectNode* object = *it;

        object->translate(-xStep, 0.0f, 0.0f);
    }

    mHasMoved += step;
    if (mHasMoved > 500.0f) {
        mMoveRight = !mMoveRight;
        mHasMoved = 0.0f;
    }

}

void ManyMovingBoxes::createManyMovingRigidBoxes(float xOffset, float yOffset) {
    const float minX = -500.0f;
    const float minY = -500.0f;
    const float maxX = -minX;
    const float maxY = -minY;
    for (float x = minX; x <= maxX; x += 40.0f) {
        for (float y = minY; y <= maxY; y += 50.0f) {
            MyObjectNode* box =
                new MyObjectNode(
                    getCollisionWorld(),
                    //new dcollide::Wedge(10.0, 20.0, 5.0));
                    //new dcollide::Cylinder(10.0, 25.0));
                    new dcollide::Box(dcollide::Vector3(10.0, 10.0, 10.0)));
            dcollide::Vector3 translation = dcollide::Vector3(x + xOffset, y + yOffset, 0.0f);
            dcollide::Matrix rot;
            rot.rotate(xOffset, 1,1,1);
            rot.rotate(yOffset, 1,0,0);
            box->rotate(rot);

            MyObjectNode* container = new MyObjectNode(getCollisionWorld(), (dcollide::Shape*)0);
            container->addChild(box);

            addTopLevelObject(container);
            container->translate(translation);

            mMovableObjects.push_back(container);
        }
    }
}

std::string ManyMovingBoxes::getSceneDescription() const {
    return "Many moving boxes that collide with each other.";
}

/*
 * vim: et sw=4 ts=4
 */
