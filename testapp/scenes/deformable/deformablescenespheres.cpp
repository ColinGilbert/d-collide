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

#include "deformablescenespheres.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"
#include "mydeformableobjectnode.h"

#include <d-collide/debug.h>
#include <d-collide/worldparameters.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/debugstream.h>

#include <iostream>

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
DeformableSceneSpheres::DeformableSceneSpheres(Ogre::Root* root)
        : SceneBase(root) {
}

DeformableSceneSpheres::~DeformableSceneSpheres() {
}

dcollide::Vector3 DeformableSceneSpheres::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

void DeformableSceneSpheres::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;
    algorithm.push_back(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);
    parameters.setDeformableAlgorithms(algorithm);
}

/*!
 * Setup the actual scene, i.e. add objects to the \ref dcollide::World
 * object.
 *
 * \return TRUE on success, otherwise FALSE.
 **/
bool DeformableSceneSpheres::initializeScene() {

    ModelLoader::LoaderDummy loader;

    float clothSize = 200.0f;
    dcollide::Mesh* staticClothMesh = loader.createRectangleSurface(clothSize, clothSize, 5.0f, 5.0f); //20,20
    if (!staticClothMesh) {
        dcollide::error() << "Could not create mesh (static)";
        return false;
    }

    dcollide::Mesh* deformingClothMesh = loader.createRectangleSurface(clothSize, clothSize, 5.0f, 5.0f); //20,20
    if (!deformingClothMesh) {
        dcollide::error() << "Could not create mesh (deforming)";
        return false;
    }

    mStaticCloth = new MyObjectNode(getCollisionWorld(), staticClothMesh, dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_SELFCOLLIDABLE, false);
    mDeformingCloth = new MyDeformableObjectNode(getCollisionWorld(), deformingClothMesh, dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_SELFCOLLIDABLE, false);

    if (mStaticCloth) {
        addTopLevelObject(mStaticCloth);
        mStaticCloth->translate(dcollide::Vector3(25.0, -100.0, -300.0));
    }

    if (mDeformingCloth) {
        addTopLevelObject(mDeformingCloth);
        mDeformingCloth->translate(dcollide::Vector3(-225.0, -100.0, -300.0));
        mDeformingCloth->rotate(-60.0, 1.0, 0.0, 0.0);

        //BoundingVolumes are still misplaced
    }

    mAmplitude = 5.0;
    mPeriod = (dcollide::real) 0.2;

    mCircleAngle = 0.0;

    for(unsigned int c=0; c<41; c++){

        for (int i = 0 ; i < 41; i++) {

            dcollide::real x = (dcollide::real) 0.0;
            dcollide::real y = (dcollide::real) 0.0;
            dcollide::real z = (dcollide::real) (mAmplitude*(sin(mCircleAngle)));

            dcollide::Vector3 vec(x, y, z);
            mClothDeformVector.push_back(vec);

        }
        mCircleAngle = (mCircleAngle + mPeriod);   //%360 ???
    }

    mDeformingCloth->deform(mClothDeformVector);

    mCircleAngle = 0.0;

    std::cout << dc_funcinfo << "done" << std::endl;
    return true;
}

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void DeformableSceneSpheres::startNextSceneFrame() {

    //mClothDeformVector.clear();

    for(unsigned int c=0; c<41; c++){

        for (int i = 0 ; i < 41; i++) {

            //dcollide::real x = (dcollide::real) 0.0;
            //dcollide::real y = (dcollide::real) 0.0;
            //dcollide::real z = (dcollide::real) ( (-1.0) * (mAmplitude*sin((c * mPeriod)+mCircleAngle)) + (mAmplitude*sin((c * mPeriod)+mCircleAngle + mPeriod)));

            mClothDeformVector[c*41+i].setZ((dcollide::real) ( (-1.0) * (mAmplitude*sin((c * mPeriod)+mCircleAngle)) + (mAmplitude*sin((c * mPeriod)+mCircleAngle + mPeriod))));

            //dcollide::Vector3 vec(x, y, z);
            //mClothDeformVector.push_back(vec);

        }
    }

    mDeformingCloth->deform(mClothDeformVector);

    mCircleAngle += mPeriod;   //%360 ???
    if (mCircleAngle >= 360) {
        mCircleAngle = mPeriod;
    }
}

std::string DeformableSceneSpheres::getSceneDescription() const {
    return "this scene shows the SurfaceHierarchy algorithm\nworking on a flat and regular triangle-grid.";
}

/*
 * vim: et sw=4 ts=4
 */
