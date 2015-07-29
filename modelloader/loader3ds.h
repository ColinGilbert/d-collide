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

#ifndef MODELLOADER_LOADER3DS_H
#define MODELLOADER_LOADER3DS_H

#include "math/vector.h"
#include "math/matrix.h"
#include "dcollide-global.h"
#include "modelloader/dcollide-modelloader-config.h"
#include "modelloader/textureinformation.h"

#include <map>

namespace dcollide {
    class Proxy;
    class Shape;
    class Mesh;
    class World;
}

namespace ModelLoader {
    class Loader3dsInternal;

    /*!
     * Class that loads .3ds files into d-collide data structures
     *
     * Use \ref loadFromFile to load a complete \ref Proxy hierarchy which
     * matches the .3ds hierarchy. Use \ref getTransformationNodes to apply the
     * necessary transformations (translations and rotations) to the hierarchy
     * afterwards.
     *
     * Use \ref loadFromFileToOneMesh if you are not interested in the actual
     * hierarchy, but prefer a single big mesh with all vertices of the model in
     * it. This is in particular useful for deformable objects or for testing of
     * mesh-splitting algorithms (see \ref MeshSplitter).
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class Loader3ds {
        public:
            /*!
             * Transformations that should be applied to the \ref Proxy
             * hierarchy returned by \ref loadFromFile, objects of this struct
             * are returned by \ref getTransformationNodes after calling \ref
             * loadFromFile.
             *
             * Note that even though .3ds files contain translations, rotations
             * and scale values, only translations and rotations are provided by
             * this model loader: the reason for this is that OGRE does not
             * handle scale values the same way as .3ds does (in particular OGRE
             * does not apply scale values to child nodes or to translations).
             * Therefore all scale values are calculated away from the model and
             * included directly into the vertices of the \ref Shape objects.
             */
            struct TransformationNode {
                TransformationNode() {
                }
                dcollide::Vector3 translation;
                dcollide::Matrix rotation;
            };

        public:
            Loader3ds();
            virtual ~Loader3ds();

            dcollide::Proxy* loadFromFile(dcollide::World* world, const char* fileName, dcollide::ProxyTypes proxyType = dcollide::PROXYTYPE_RIGID);
            dcollide::Mesh* loadFromFileToOneMesh(const char* fileName, TextureInformation* textureInformation = 0);

            inline const std::map<dcollide::Proxy*, TransformationNode>& getTransformationNodes() const;
            inline const std::map<dcollide::Proxy*, TextureInformation>& getTextureInformation() const;

        private:
            friend class Loader3dsInternal;
            std::map<dcollide::Proxy*, TransformationNode> mProxy2Transformation;
            std::map<dcollide::Proxy*, TextureInformation> mProxy2TextureInformation;
#ifdef HAVE_LIB3DS
            Loader3dsInternal* mLoader;
#endif // HAVE_LIB3DS
    };


    /*!
     * \return The transformation nodes (see \ref TransformationNode) that
     * contain the transformations that are to-be applied to the proxies created
     * by \ref loadFromFile. Every \ref Proxy object created by \ref
     * loadFromFile is contained in the map exactly once.
     */
    inline const std::map<dcollide::Proxy*, Loader3ds::TransformationNode>& Loader3ds::getTransformationNodes() const {
        return mProxy2Transformation;
    }

    inline const std::map<dcollide::Proxy*, TextureInformation>& Loader3ds::getTextureInformation() const {
        return mProxy2TextureInformation;
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
