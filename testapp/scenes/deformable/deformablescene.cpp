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

#include "deformablescene.h"
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
DeformableScene::DeformableScene(Ogre::Root* root)
        : SceneBase(root) {
}

DeformableScene::~DeformableScene() {
}

dcollide::Vector3 DeformableScene::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

void DeformableScene::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
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
bool DeformableScene::initializeScene() {


    //bunny.ply , bunny_lower_res2.ply , horse.ply , dragon_vrip_res4.ply , dragon_vrip_res3.ply , dragon_stand/dragonStandRight_0.ply
    MyObjectNode* bigModelDeformable = createPlyModel("bunny_lower_res2.ply",
            (dcollide::ProxyTypes) (dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_SELFCOLLIDABLE),
             800.0f);

    bigModelDeformable->translate(dcollide::Vector3(0.0, -80.0, -120.0));
    bigModelDeformable->rotate(15.0, 1.0, 0.0, 0.0);

    if (bigModelDeformable) {
        addTopLevelObject(bigModelDeformable);
        //bigModelDeformable->rotate();
    }


    std::cout << dc_funcinfo << "done" << std::endl;
    return true;
}

/*!
 * Start a new frame of the scene, i.e. moves all movable object by a bit.
 *
 * This is NOT necessarily equal to a graphic frame! In fact the scene frames
 * should depend on time only, not on CPU or GPU power as the graphic frames do.
 */
void DeformableScene::startNextSceneFrame() {



}

std::string DeformableScene::getSceneDescription() const {
    return "This scene is a performance test for the SurfaceHierarchy algorithm.\nThe bunny consists of ~16.000 triangles";
}

/*
 * vim: et sw=4 ts=4
 */
