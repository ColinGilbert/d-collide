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

#include "clothbox.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"
#include "mydeformableobjectnode.h"

#include <modelloader/loaderdummy.h>

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

ClothBox::ClothBox(Ogre::Root* root) : PhysicsSceneBase(root) {
    mPhysicsSimStepsize = (float) 0.03;
    mLinearDampingFactor = (dReal) 0.001;
    mAngularDampingFactor = (dReal) 0.005;
}

ClothBox::~ClothBox() {

}

dcollide::Vector3 ClothBox::initialWorldDimension() const {
    return dcollide::Vector3 (2048.0, 2048.0, 2048.0);
}

void ClothBox::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_BV_HIERARCHY);
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_SPATIAL_HASH);
    algorithm.push_back(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);
    parameters.setDeformableAlgorithms(algorithm);

}

bool ClothBox::initializeScene() {
    ModelLoader::LoaderDummy firstLoader;
    dWorldSetGravity(getOdeWorld(), 0, 0, -20.5);

    // === First Sphere (lower one) ========

    dcollide::Mesh* meshSphere1 = dcollide::Sphere(12).getMesh()->cloneMesh();

    mSphere1 = new MyDeformableObjectNode(getCollisionWorld(),
                                          meshSphere1,
                                          dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_CLOSEDHULL | dcollide::PROXYTYPE_CONVEX,
                                          false);

    mSphere1->createPhysicsBody(mOdeWorld, 1/6 * 3.14 * 1728);

    mSphere1->translate( 0, 0, 80);

    addTopLevelObject(mSphere1);


    // === Second Sphere (upper one) ========

    dcollide::Mesh* meshSphere2 = dcollide::Sphere(12).getMesh()->cloneMesh();

    mSphere2 = new MyDeformableObjectNode(getCollisionWorld(),
                                          meshSphere2,
                                          dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_CLOSEDHULL | dcollide::PROXYTYPE_CONVEX,
                                          false);

    mSphere2->createPhysicsBody(mOdeWorld, 1/6 * 3.14 * 1728);

    mSphere2->translate( 0, 0, 320);

    addTopLevelObject(mSphere2);


    MyObjectNode* environment = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);
    // === Box -> Ground ==========

    mGround = new MyObjectNode (getCollisionWorld(),
                                new dcollide::Box(200.0, 150.0, 10.0),
                                dcollide::PROXYTYPE_FIXED,
                                true);

    mGround->translate(-50, -50, -50);
    environment->addChild(mGround);



    // === First Box (left one) ==========

    mBox1 = new MyObjectNode (getCollisionWorld(),
                             new dcollide::Box(10.0, 100.0, 20.0),
                             dcollide::PROXYTYPE_FIXED,
                             true);

    mBox1->translate(90, -50, -50);

    environment->addChild(mBox1);


    // === Sphere (between boxes) ==========

    mSphere = new MyObjectNode (getCollisionWorld(),
                                new dcollide::Sphere(15.0),
                                dcollide::PROXYTYPE_FIXED,
                                true);
    mSphere->translate(115, -30, -35);

    environment->addChild(mSphere);


    // === Second Box (right one) ==========

    mBox2 = new MyObjectNode (getCollisionWorld(),
                             new dcollide::Box(10.0, 100.0, 20.0),
                             dcollide::PROXYTYPE_FIXED,
                             true);

    mBox2->translate(130, -50, -50);

    environment->addChild(mBox2);


    addTopLevelObject(environment);

    // === Cloth ==========

    dcollide::Mesh* meshCloth = firstLoader.createRectangleSurface(100.0f, 50.0f, 5.0f, 5.0f);

    mCloth = new MyDeformableObjectNode(getCollisionWorld(),
                                        meshCloth,
                                        dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_SELFCOLLIDABLE,
                                        true);

    mCloth->createPhysicsBody(mOdeWorld, 100);
    //FIXME this causes a weird effect if activated. See ticket #368
    //mCloth->rotate(180,1,0,0);
    mCloth->translate(85, -50, 40, false);

    addTopLevelObject(mCloth);

    return true;
}

void ClothBox::startNextSceneFrame() {

}

std::string ClothBox::getSceneDescription() const {
    return "simulation of a deformable cloth falling onto a box.\nCurrently: a grid of rigid particles/spheres connected by joints";
}
