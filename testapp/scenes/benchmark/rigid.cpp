/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *                                                                             *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,          *
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

#include "rigid.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"
#include "mydeformableobjectnode.h"

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


RigidBenchmark::RigidBenchmark(Ogre::Root* root) : PhysicsSceneBase(root) {
    mPhysicsSimStepsize = (float) 0.03;
    mLinearDampingFactor = (dReal) 0.001;
    mAngularDampingFactor = (dReal) 0.005;
}

RigidBenchmark::~RigidBenchmark() {

}

dcollide::Vector3 RigidBenchmark::initialWorldDimension() const {
    return dcollide::Vector3 (2048.0, 2048.0, 2048.0);
}

void RigidBenchmark::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;

    parameters.setDeformableAlgorithms(algorithm);
    parameters.setWorkerThreadCount(4);
}

bool RigidBenchmark::initializeScene() {

    dWorldSetGravity(getOdeWorld(), 0, 0, -10.5);


    // === Environement (Box -> Ground) ==========

    MyObjectNode* environment = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);

    mGround = new MyObjectNode (getCollisionWorld(),
                                new dcollide::Box(1000.0, 1000.0, 10.0),
                                dcollide::PROXYTYPE_FIXED,
                                true, false);

    mGround->translate(-500, -500, -50);
    environment->addChild(mGround);

    addTopLevelObject(environment);


    // === Helicopter ==========

    MyObjectNode* mHelicopter = create3dsModelWithOneMesh("puma.3ds", (dcollide::PROXYTYPE_RIGID | dcollide::PROXYTYPE_CLOSEDHULL));
    if (mHelicopter) {
        mHelicopter->createPhysicsBody(mOdeWorld, 500);
        mHelicopter->setRotation(90, 0,0,-1);
        mHelicopter->setPosition(-400, 0, 200);

        addTopLevelObject(mHelicopter);

        dBodySetLinearVel (mHelicopter->getPhysicsBody(), 300, 0, 5);
    }


    // === Boxes ==========

    dcollide::real height = 10;
    for (dcollide::real angle = 0; angle < 180; angle += 10) {

        addBoxRing(angle, height);
        height += 25;
    }

    return true;
}

void RigidBenchmark::addBoxRing(dcollide::real startAngle, dcollide::real height) {

    dcollide::real radius = 150;

    dcollide::real boxWidth  = 40;
    dcollide::real boxDepth  = 25;
    dcollide::real boxHeight = 40;

    dcollide::real ringCenterX = 200;
    dcollide::real ringCenterY = 0;

    dcollide::real step = acos(1-((boxWidth*boxWidth)/(2*radius*radius))) * 180 / M_PI;

    for (dcollide::real angle = startAngle;
         angle < (360+startAngle);
         angle += step) {

        dcollide::real x = (cos(angle * M_PI / 180)*radius) + ringCenterX;
        dcollide::real y = (sin(angle * M_PI / 180)*radius) + ringCenterY;

        dcollide::real rotation = -1 * (180 - angle - (acos(boxWidth/(2*radius)) * 180 / M_PI));


        MyObjectNode* box = new MyObjectNode(getCollisionWorld(),
                                             new dcollide::Box(boxWidth,
                                                               boxHeight,
                                                               boxDepth));

        box->createPhysicsBody(mOdeWorld, 5);
        box->setRotation(rotation, 0, 0, 1);
        box->setPosition(x, y, height);

        mBoxes.push_back(box);
        addTopLevelObject(box);
    }
}

void RigidBenchmark::startNextSceneFrame() {

}

std::string RigidBenchmark::getSceneDescription() const {
    return "Benchmark of rigid object collisions with physic simulation";
}
