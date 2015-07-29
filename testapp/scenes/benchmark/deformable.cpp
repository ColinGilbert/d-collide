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

#include "deformable.h"
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


void DeformableBenchmark::addSphere(dcollide::real x, dcollide::real y, dcollide::real z) {

    dcollide::Mesh* meshSphere = dcollide::Sphere(12, 7.5).getMesh()->cloneMesh();

    MyDeformableObjectNode* sphere = new MyDeformableObjectNode(getCollisionWorld(),
                                                                meshSphere,
                                                                dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_CLOSEDHULL | dcollide::PROXYTYPE_CONVEX,
                                                                false, false);

    sphere->createPhysicsBody(mOdeWorld, 1/6 * 3.14 * 1728);
    sphere->translate( x, y, z);

    mSpheres.push_back(sphere);
    addTopLevelObject(sphere);
}

DeformableBenchmark::DeformableBenchmark(Ogre::Root* root) : PhysicsSceneBase(root) {
    mPhysicsSimStepsize = (float) 0.03;
    mLinearDampingFactor = (dReal) 0.001;
    mAngularDampingFactor = (dReal) 0.005;
}

DeformableBenchmark::~DeformableBenchmark() {

}

dcollide::Vector3 DeformableBenchmark::initialWorldDimension() const {
    return dcollide::Vector3 (2048.0, 2048.0, 2048.0);
}

void DeformableBenchmark::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_BV_HIERARCHY);
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_SPATIAL_HASH);
    algorithm.push_back(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);

    parameters.setDeformableAlgorithms(algorithm);
    parameters.setWorkerThreadCount(4);
}

bool DeformableBenchmark::initializeScene() {

    dWorldSetGravity(getOdeWorld(), 0, 0, -20.5);


    // === Some Spheres ========

    addSphere(0,0,80);
    addSphere(0,0,160);

    addSphere(50,0,80);
    addSphere(50,0,160);

    addSphere(100,0,80);
    addSphere(100,0,160);


    //addSphere(0,20,80);
    //addSphere(0,20,160);

    //addSphere(50,50,80);
    //addSphere(50,50,160);

    //addSphere(100,50,80);
    //addSphere(100,50,160);


    // === Environement (Box -> Ground) ==========

    MyObjectNode* environment = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);

    mGround = new MyObjectNode (getCollisionWorld(),
                                new dcollide::Box(200.0, 150.0, 10.0),
                                dcollide::PROXYTYPE_FIXED,
                                true, false);

    mGround->translate(-50, -50, -50);
    environment->addChild(mGround);

    addTopLevelObject(environment);

    return true;
}

void DeformableBenchmark::startNextSceneFrame() {

}

std::string DeformableBenchmark::getSceneDescription() const {
    return "Benchmark of deformable object collisions with physic simulation";
}
