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

#ifndef DCOLLIDE_OGREMATERIALS_H
#define DCOLLIDE_OGREMATERIALS_H

#include "dcollide-config_testapp.h" // fixes some compiler warnings caused by ogre
#include <d-collide/math/vector.h>
#include <OgreString.h>

class MyObjectNode;

namespace Ogre {
    class SceneManager;
    class SceneNode;
    class ManualObject;
    class MaterialPtr;
    class ColourValue;
}

namespace ModelLoader {
    class TextureInformation;
};

/*!
 * \brief Manages Ogre materials
 */
// AB: this class may be partially obsolete: proxy materials are NOT handled
// here anymore!
class OgreMaterials {
    public:
        OgreMaterials();
        virtual ~OgreMaterials();

        static bool init();

        static std::string getBoundingVolumeMaterial(bool collisionBroad = false, bool collisionMiddle = false);
        static std::string getNarrowPhaseCollisionPointsMaterial();
        static std::string getNarrowPhaseNormalsMaterial();
        static std::string getHierarchicalGridMaterial();

        static Ogre::MaterialPtr createShapeMaterial(const Ogre::String& name, const ModelLoader::TextureInformation& textureInformation, bool useCulling, bool useTextures);
        static Ogre::ColourValue getShapeDiffuseColor(const ModelLoader::TextureInformation& materialProperties, bool isPenetrating = false, bool isPenetrated = false);
        static void destroyShapeMaterial(const Ogre::String& name);

    protected:

    private:
};

#endif
/*
 * vim: et sw=4 ts=4
 */
