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

#include "ogrematerials.h"

#include <d-collide/debugstream.h>

#include <modelloader/textureinformation.h>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreString.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreBlendMode.h>


OgreMaterials::OgreMaterials() {
}

OgreMaterials::~OgreMaterials() {
}

/*!
 * Initialize the materials
 *
 * \return TRUE on success, FALSE otherwise (this object should not be used
 * then).
 */
bool OgreMaterials::init() {
    static bool initialized = false;
    if (initialized) {
        return true;
    }
    initialized = true;

    Ogre::MaterialManager& materialManager = Ogre::MaterialManager::getSingleton();

    // AB: we don't use the resourceGroup anyway, so it should not matter which
    // one we pick.
    const Ogre::String resourceGroup = "DCollideMaterials";
    Ogre::MaterialPtr boundingVolumeMaterial = materialManager.create(getBoundingVolumeMaterial(), resourceGroup);
    Ogre::MaterialPtr boundingVolumeMaterialCollisionBroad = materialManager.create(getBoundingVolumeMaterial(true, false), resourceGroup);
    Ogre::MaterialPtr boundingVolumeMaterialCollisionMiddle = materialManager.create(getBoundingVolumeMaterial(false, true), resourceGroup);
    Ogre::MaterialPtr hierarchicalGridMaterial =
        materialManager.create(getHierarchicalGridMaterial(), resourceGroup);

    // AB: some of these properties can be changed by using the corresponding
    //     Ogre::Material methods. However these are just shortcuts to the ones in
    //     Ogre::Pass and so for convenience (all properties at one place) we
    //     always use the version in Ogre::Pass.

    // no light => colour values are used (with light colour is usually ignored
    // completely!)
    // AB: atm we want light, so we don't use these lines
    //boundingVolumeMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);

    // if light is enabled: "colour tracking" means that the "colour" value at a
    // vertex is used as "colour" of a light. use this for diffuse and ambient
    // light and you get essentially a correctly coloured object.
    // AB: atm we use one "global" colour for all objects of a certain type
    //     (proxies or BVs), so no tracking is required.
    //boundingVolumeMaterial->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE | Ogre::TVC_AMBIENT);

    boundingVolumeMaterial->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    boundingVolumeMaterial->getTechnique(0)->setDepthCheckEnabled(1);
    boundingVolumeMaterial->getTechnique(0)->setDepthFunction(Ogre::CMPF_LESS_EQUAL);
    boundingVolumeMaterial->getTechnique(0)->setSelfIllumination(0.1, 0.3, 0.1);
    boundingVolumeMaterial->getTechnique(0)->setShininess(0.3);
    boundingVolumeMaterial->getTechnique(0)->setSpecular(0.2, 0.5, 0.2, 0.2);
    boundingVolumeMaterial->getTechnique(0)->setShadingMode(Ogre::SO_FLAT);
    boundingVolumeMaterial->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_SOLID);
    boundingVolumeMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(0);
    boundingVolumeMaterial->getTechnique(0)->getPass(0)->setAmbient(0.0, 0.7, 0.0);
    boundingVolumeMaterial->getTechnique(0)->getPass(0)->setDiffuse(0.0, 0.4, 0.0, 0.2);

    //boundingVolumeMaterial->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    //boundingVolumeMaterial->getTechnique(0)->getPass(0)->setAmbient(0.0, 1.0, 0.0);
    //boundingVolumeMaterial->getTechnique(0)->getPass(0)->setDiffuse(0.0, 1.0, 0.0, 1.0);

    boundingVolumeMaterialCollisionBroad->getTechnique(0)->getPass(0)->setAmbient(0.0, 0.0, 1.0);
    boundingVolumeMaterialCollisionBroad->getTechnique(0)->getPass(0)->setDiffuse(0.0, 0.0, 1.0, 1.0);
    boundingVolumeMaterialCollisionMiddle->getTechnique(0)->getPass(0)->setAmbient(0.0, 1.0, 1.0);
    boundingVolumeMaterialCollisionMiddle->getTechnique(0)->getPass(0)->setDiffuse(0.0, 1.0, 1.0, 1.0);
    hierarchicalGridMaterial->getTechnique(0)->getPass(0)->
            setAmbient(1.0, 0.7, 0.0);
    hierarchicalGridMaterial->getTechnique(0)->getPass(0)->
            setDiffuse(1.0, 0.7, 0.0, 1.0);

    boundingVolumeMaterialCollisionBroad->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    boundingVolumeMaterialCollisionMiddle->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    hierarchicalGridMaterial->getTechnique(0)->getPass(0)->
            setPolygonMode(Ogre::PM_WIREFRAME);


    Ogre::MaterialPtr collisionPointsMaterial = materialManager.create(getNarrowPhaseCollisionPointsMaterial(), resourceGroup);
    collisionPointsMaterial->getTechnique(0)->getPass(0)->setAmbient(1.0, 1.0, 0.0);
    collisionPointsMaterial->getTechnique(0)->getPass(0)->setDiffuse(1.0, 1.0, 0.0, 1.0);
    collisionPointsMaterial->getTechnique(0)->getPass(0)->setPointSize(5.0);

    Ogre::MaterialPtr normalsMaterial = materialManager.create(getNarrowPhaseNormalsMaterial(), resourceGroup);
    normalsMaterial->getTechnique(0)->getPass(0)->setAmbient(0.0, 1.0, 0.0);
    normalsMaterial->getTechnique(0)->getPass(0)->setDiffuse(1.0, 1.0, 0.0, 1.0);


    return true;
}

std::string OgreMaterials::getBoundingVolumeMaterial(bool collisionBroad, bool collisionMiddle) {
    if (collisionBroad && collisionMiddle) {
        std::cerr << dc_funcinfo << "invalid parameters" << std::endl;
        return std::string();
    }
    if (!collisionBroad && !collisionMiddle) {
        return "boundingvolume_material";
    } else if (collisionBroad) {
        return "boundingvolume_material_collision_broad";
    } else if (collisionMiddle) {
        return "boundingvolume_material_collision_middle";
    }

    return getBoundingVolumeMaterial(false, false); // fallback
}

std::string OgreMaterials::getNarrowPhaseCollisionPointsMaterial() {
    return "narrowphase_collisionpoints_material";
}

std::string OgreMaterials::getNarrowPhaseNormalsMaterial() {
    return "narrowphase_normals_material";
}

std::string OgreMaterials::getHierarchicalGridMaterial() {
    return "hierarchicalGrid_material";
}

/*!
 * Create a new \ref Ogre::Material for use in a \ref MyObjectNode which
 * represents a \ref dcollide::Shape.
 *
 * The material is (curently anyway) valid for that single shape only and is
 * meant to be destroyed on destruction of the corresponding \ref MyObjectNode.
 */
Ogre::MaterialPtr OgreMaterials::createShapeMaterial(const Ogre::String& name, const ModelLoader::TextureInformation& textureInformation, bool useCulling, bool useTextures) {
    Ogre::MaterialManager& materialManager = Ogre::MaterialManager::getSingleton();
    const Ogre::String resourceGroup = "DCollideMaterials";
    Ogre::MaterialPtr material = materialManager.create(name, resourceGroup);

    Ogre::Pass* pass = material->getTechnique(0)->getPass(0);

    pass->setAmbient(1.0, 0.0, 0.0);

    if (!useCulling) {
        pass->setCullingMode(Ogre::CULL_NONE);
    }

    if (useTextures) {
        if (textureInformation.isTextured()) {
            pass->setAmbient(1.0, 1.0, 1.0);
            pass->setDiffuse(1.0, 1.0, 1.0, 1.0);
            pass->createTextureUnitState(textureInformation.getTextureFileName());
        }
    }

    if (textureInformation.isTransparent()) {
        material->getTechnique(0)->
            setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    }

    pass->setDiffuse(getShapeDiffuseColor(textureInformation));

    return material;
}

Ogre::ColourValue OgreMaterials::getShapeDiffuseColor(const ModelLoader::TextureInformation& materialProperties, bool isPenetrating, bool isPenetrated) {
    float alpha = 1.0;
    if (materialProperties.isTransparent()) {
        alpha = materialProperties.getTransparency();
    }
    if (!isPenetrating && !isPenetrated) {
        if (materialProperties.isTextured()) {
            return Ogre::ColourValue(1.0, 1.0, 1.0, alpha);
        }
    }
    if (isPenetrating && isPenetrated) {
        return Ogre::ColourValue(1.0, 0.0, 1.0, alpha);
    }
    if (isPenetrating) {
        return Ogre::ColourValue(4.0, 4.5, 0.0, alpha);
    }
    if (isPenetrated) {
        return Ogre::ColourValue(0.5, 1.0, 4.0, alpha);
    }
    return Ogre::ColourValue(1.0, 0.0, 0.0, alpha);
}

/*!
 * Destroy (see \ref Ogre::MaterialManager::remove) a material created by \ref
 * createShapeMaterial.
 *
 * If we ever want to re-use materials in multiple \ref MyObjectNode objects,
 * this method should delete the material only if all \ref MyObjectNode objects
 * using it have been deleted.
 */
void OgreMaterials::destroyShapeMaterial(const Ogre::String& name) {
    Ogre::MaterialManager& materialManager = Ogre::MaterialManager::getSingleton();
    materialManager.remove(name);
}

/*
 * vim: et sw=4 ts=4
 */
