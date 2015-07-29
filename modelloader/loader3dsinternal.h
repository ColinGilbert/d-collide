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

#ifndef MODELLOADER_LOADER3DSINTERNAL_H
#define MODELLOADER_LOADER3DSINTERNAL_H

#include "math/vector.h"
#include "math/matrix.h"
#include "dcollide-global.h"
#include "modelloader/dcollide-modelloader-config.h"
#include "loader3ds.h"

#include <lib3ds/types.h>

#include <map>

namespace dcollide {
    class Proxy;
    class Shape;
    class Mesh;
    class World;
}

namespace ModelLoader {
    /*!
     * \brief Helper class to \ref Loader3ds
     *
     * This class merely exists to hide implementation detail of the .3ds loader
     * from the user, in particular to avoid the requirements of including
     * lib3ds headers when including the \ref Loader3ds header.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class Loader3dsInternal {
        public:
            Loader3dsInternal(Loader3ds* data);
            virtual ~Loader3dsInternal();

            dcollide::Proxy* loadFromFile(dcollide::World* world, const char* fileName, dcollide::ProxyTypes proxyType = dcollide::PROXYTYPE_RIGID);
            dcollide::Mesh* loadFromFileToOneMesh(const char* fileName, TextureInformation* textureInformation = 0);

        protected:
            bool loadNode(dcollide::World* world, dcollide::Proxy* parent, Lib3dsNode* node, Lib3dsMatrix* parentTranslateRotateMatrix);
            dcollide::Shape* createShape(Lib3dsNode* node, Lib3dsMatrix* translateRotateMatrix);
            TextureInformation loadTextureInformation(Lib3dsNode* node);

            bool checkUniqueMeshMaterial() const;

        private:
            Loader3ds* mData;
            Lib3dsFile* mFile;
    };
}

#endif
/*
 * vim: et sw=4 ts=4
 */
