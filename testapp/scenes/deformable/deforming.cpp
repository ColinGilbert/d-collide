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

#include "deforming.h"
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

#include <d-collide/detectordeform/spatialhash/spatialhash.h>
#include <d-collide/detectordeform/detectordeformmanager.h>
#include <d-collide/worldparameters.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

Deforming::Deforming(Ogre::Root* root) : SceneBase(root) {
}

Deforming::~Deforming() {
}

dcollide::Vector3 Deforming::initialWorldDimension() const {
    return dcollide::Vector3 (2048.0, 2048.0, 2048.0);
}

void Deforming::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;
    algorithm.push_back(dcollide::DEFORMABLE_TYPE_SPATIAL_HASH);
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);
    parameters.setDeformableAlgorithms(algorithm);
}

bool Deforming::initializeScene() {
    mStep = 0;

    //SelfColliding
    ModelLoader::LoaderDummy secondLoader;
    dcollide::Mesh* selfCollidingCloth = secondLoader.createRectangleSurface(60.0, 60.0, 7.0, 11.0);
    //dcollide::Mesh* selfCollidingCloth = secondLoader.createRectangleSurface(60.0, 5.0, 6.0, 1.0);
    mSelfCollidingCloth = new MyDeformableObjectNode(getCollisionWorld(),
                                                     selfCollidingCloth,
                                                     (dcollide::ProxyTypes) (dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_SELFCOLLIDABLE),
                                                     false);

    addTopLevelObject(mSelfCollidingCloth);
    mSelfCollidingCloth->translate(dcollide::Vector3(5.0, -75.0, -250.0));
    mSelfCollidingCloth->rotate(75.0, 1.0, 0.0, 0.0);

    unsigned int selfColClothNrOfVert = selfCollidingCloth->getVertexCount();
    std::cout << selfColClothNrOfVert << std::endl;
    for (unsigned int i=0; i<selfColClothNrOfVert; i++) {
        dcollide::real x = (dcollide::real) 0.0;
        dcollide::real y = (dcollide::real) 0.0;
        dcollide::real z = (dcollide::real) 0.0;

        dcollide::Vector3 vec(x, y, z);
        mSelfCollidingClothDeformVector.push_back(vec);
    }

    // Cloth
    ModelLoader::LoaderDummy firstLoader;
    dcollide::Mesh* clothMesh = firstLoader.createRectangleSurface(100.0, 100.0, 1.0, 1.0);
    mCloth = new MyDeformableObjectNode(getCollisionWorld(),
                                        clothMesh,
                                        dcollide::PROXYTYPE_DEFORMABLE,
                                        false);

    addTopLevelObject(mCloth);
    mCloth->translate(dcollide::Vector3(-125.0, 0.0, -50.0));
    mCloth->rotate(30.0, 1.0, 0.0, 0.0);

    mAmplitude = 5.0;
    mPeriod = (dcollide::real) 0.2;

    mCircleAngle = 0.0;

        for (unsigned int c=0; c<101; c++){

            for (int i=0; i<101; i++) {

                dcollide::real x = (dcollide::real) 0.0;
                dcollide::real y = (dcollide::real) 0.0;
                dcollide::real z = (dcollide::real) (mAmplitude*(sin(mCircleAngle)));

                dcollide::Vector3 vec(x, y, z);
                mClothDeformVector.push_back(vec);
            }
            mCircleAngle = (mCircleAngle + mPeriod); //%360
        }

    mCircleAngle = 0.0;

    // Box
    dcollide::Box box(40.0, 40.0, 40.0);
    dcollide::Mesh* mesh = box.getMesh()->cloneMesh();

    mDeformingBox = new MyDeformableObjectNode (getCollisionWorld(),
                                                mesh,
                                                dcollide::PROXYTYPE_DEFORMABLE);

    addTopLevelObject(mDeformingBox);

    mDeformingBox->translate(dcollide::Vector3(25.0, 0.0, -50.0));

    unsigned int numberOfVertices = mesh->getVertexCount();

    for (unsigned int i=0; i<numberOfVertices; i++){
        dcollide::real x = (dcollide::real) (sin((dcollide::real)(rand() % 360)));
        dcollide::real y = (dcollide::real) (sin((dcollide::real)(rand() % 360)));
        dcollide::real z = (dcollide::real) (sin((dcollide::real)(rand() % 360)));

        dcollide::Vector3 vec(x, y, z);
        mDeformVector.push_back(vec);
    }

    mDeformingBox->deform(mDeformVector);
    mCloth->deform(mClothDeformVector);
    mSelfCollidingCloth->deform(mSelfCollidingClothDeformVector);

    return true;
}

void Deforming::startNextSceneFrame() {
    mStep+=1;

    mRadius = (dcollide::real) -0.2;
    mAngle = (dcollide::real) 25.0;

    //SelfColliding
    unsigned int selfColClothNrOfVert = mSelfCollidingCloth->getProxy()->getShape()->getMesh()->getVertexCount();
    unsigned int middlePointIndex = selfColClothNrOfVert/2;

    std::vector<dcollide::Vertex*> selfCollidingVertexList = mSelfCollidingCloth->getProxy()->getShape()->getMesh()->getVertices();
    std::vector<dcollide::Vertex*>::iterator iter_SelfVertex = selfCollidingVertexList.begin();

    dcollide::real xCoord = selfCollidingVertexList[middlePointIndex]->getPosition().getX();
    dcollide::real zCoord = selfCollidingVertexList[middlePointIndex]->getPosition().getZ();

    //dcollide::real xCoord = 0.0;
    //dcollide::real zCoord = 0.0;

        if(mStep <= 100) {
            for (unsigned int i=0; i<selfColClothNrOfVert; i++) {
                std::vector<dcollide::Vertex*> vertices = mSelfCollidingCloth->getProxy()->getShape()->getMesh()->getVertices();

                dcollide::real x = (dcollide::real) ((-1.0)*(xCoord + (mRadius+0.3) * -2 * cos((mAngle-25.0))) + (xCoord + mRadius * -2 * cos(mAngle)));
                dcollide::real y = (dcollide::real) 0.0;
                dcollide::real z = (dcollide::real) ((-1.0)*(zCoord + (mRadius+0.3) * -2 * sin((mAngle-25.0))) + (zCoord + mRadius * -2 * sin(mAngle)));

                dcollide::Vector3 vec(x, y, z);
                mSelfCollidingClothDeformVector[i] = vec;

                mAngle += 25.0;
                ++iter_SelfVertex;
            }
        }

    // Cloth
    //unsigned int clothNumberOfVertices = mCloth->getProxy()->getShape()->getMesh()->getVertexCount();
    //unsigned int clothNumberofTriangles = mCloth->getProxy()->getShape()->getMesh()->getTriangleCount();
    std::vector<dcollide::Vertex*> vertexList = mCloth->getProxy()->getShape()->getMesh()->getVertices();
    std::vector<dcollide::Vertex*>::iterator iter_vertex = vertexList.begin();

    mClothDeformVector.clear();

    for (unsigned int c=0; c<101; c++){
        for (int i=0 ; i<101; i++) {
            dcollide::real x = (dcollide::real) 0.0;
            dcollide::real y = (dcollide::real) 0.0;
            dcollide::real z = (dcollide::real) ( (-1.0) * (mAmplitude*sin((c * mPeriod)+mCircleAngle)) + (mAmplitude*sin((c * mPeriod)+mCircleAngle + mPeriod)));
            //dcollide::real z = (dcollide::real) (5*(sin(mCircleAngle)));

            dcollide::Vector3 vec(x, y, z);
            //mClothDeformVector[c*101+i] = vec;
            mClothDeformVector.push_back(vec);

            ++iter_vertex;
        }
    }

    mCircleAngle = (mCircleAngle + mPeriod);   //%360;

    // Box
    dcollide::Mesh* mesh = mDeformingBox->getProxy()->getShape()->getMesh()->cloneMesh();
    unsigned int numberOfVertices = mesh->getVertexCount();

    for (unsigned int i=0; i<numberOfVertices; i++){
        dcollide::real x = (dcollide::real) (sin((dcollide::real)(rand() % 360)));
        dcollide::real y = (dcollide::real) (sin((dcollide::real)(rand() % 360)));
        dcollide::real z = (dcollide::real) (sin((dcollide::real)(rand() % 360)));

        dcollide::Vector3 vec(x, y, z);
        mDeformVector[i] = vec;
    }

    mCloth->deform(mClothDeformVector);
    mDeformingBox->deform(mDeformVector);

    if(mStep <= 100){
        mSelfCollidingCloth->deform(mSelfCollidingClothDeformVector);
    }
}

std::string Deforming::getSceneDescription() const {
    return "A simple scene with deforming objects";
}

