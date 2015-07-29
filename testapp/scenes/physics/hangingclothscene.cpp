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

#include "hangingclothscene.h"
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

HangingClothScene::HangingClothScene(Ogre::Root* root) : PhysicsSceneBase(root) {
    mPhysicsSimStepsize = (float) 0.03;
    mLinearDampingFactor = (dReal) 0.001;
    mAngularDampingFactor = (dReal) 0.005;

    mSphere = 0;
    mCloth = 0;
}

HangingClothScene::~HangingClothScene() {
    dJointGroupDestroy(mRestrictionJoints);
}

dcollide::Vector3 HangingClothScene::initialWorldDimension() const {
    return dcollide::Vector3 (2048.0, 2048.0, 2048.0);
}

void HangingClothScene::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;
    algorithm.push_back(dcollide::DEFORMABLE_TYPE_BV_HIERARCHY);
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_SPATIAL_HASH);
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);
    parameters.setDeformableAlgorithms(algorithm);

}

bool HangingClothScene::initializeScene() {
    ModelLoader::LoaderDummy firstLoader;
    dWorldSetGravity(getOdeWorld(), 0, 0, -7.5);
    //--- deformable cloth-box demonstration -----------------------------------
/*
    //single triangle as mesh
    using namespace dcollide;
    std::vector<Vertex*> vertices;
    std::vector<Triangle*> triangles;
    Vertex* v1 = new Vertex(0,0,0);
    Vertex* v2 = new Vertex(20,0,0);
    Vertex* v3 = new Vertex(0,20,0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    triangles.push_back(new Triangle(v1, v2, v3));
    dcollide::Mesh* clothMesh = new Mesh (vertices, triangles);
*/
    // Cloth
    int gridSize = 10;
    dcollide::Mesh* clothMesh 
        = firstLoader.createRectangleSurface(100,100,gridSize,gridSize);
    
    mCloth = new MyDeformableObjectNode(getCollisionWorld(),
                                        clothMesh,
                                        dcollide::PROXYTYPE_DEFORMABLE,
                                        false);

    mCloth->createPhysicsBody(mOdeWorld,  10);

    addTopLevelObject(mCloth);


    //Jointgroup for restriction joints
    mRestrictionJoints = dJointGroupCreate(0);

    //Restrict vertex 0,0 to (0,0,0)
    dBodyID body1 = mCloth->getPhysicsBody(0);
    dcollide::Vector3 position = clothMesh->getVertices()[0]->getWorldPosition();
    
    dJointID joint1 = dJointCreateBall(mOdeWorld, mRestrictionJoints);
    dJointAttach(joint1, body1, 0);
    dJointSetBallAnchor(joint1, position.getX(), position.getY(), position.getZ());

    //Restrict vertex 10,0 to its position
    dBodyID body2 = mCloth->getPhysicsBody(10);
    position.set(clothMesh->getVertices()[10]->getWorldPosition());
    
    dJointID joint2 = dJointCreateBall(mOdeWorld, mRestrictionJoints);
    dJointAttach(joint2, body2, 0);
    dJointSetBallAnchor(joint2, position.getX(), position.getY(), position.getZ());

    //Restrict vertex 0,10 to its position
    dBodyID body3 = mCloth->getPhysicsBody(110);
    position.set(clothMesh->getVertices()[110]->getWorldPosition());
    
    dJointID joint3 = dJointCreateBall(mOdeWorld, mRestrictionJoints);
    dJointAttach(joint3, body3, 0);
    dJointSetBallAnchor(joint3, position.getX(), position.getY(), position.getZ());
    
    //Sphere falling onto the cloth
    
    mSphere = new MyObjectNode (getCollisionWorld(),
            //new dcollide::Box(10.0, 10.0, 10.0),
            new dcollide::Sphere(5.0, 2.0),
                              dcollide::PROXYTYPE_RIGID, true);
    mSphere->createPhysicsBody(mOdeWorld,  1);
    addTopLevelObject(mSphere);
    mSphere->translate(20,20, 10);
    const_cast<ModelLoader::TextureInformation&> (mSphere->getTextureInformation()).setTransparent(0.1);
/*
    MyObjectNode* origin = new MyObjectNode (getCollisionWorld(),
                                             new dcollide::Sphere(2.0));
    addTopLevelObject(origin);
    //origin->setPosition(offset2[0],offset2[1],offset2[2]);
*/
    return true;
}

void HangingClothScene::startNextSceneFrame() {
    //dcollide::debug() << "............. step .............";
}

std::string HangingClothScene::getSceneDescription() const {
    return "simulation of a deformable cloth fixed at some points.";
}
