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

#include "loader3ds.h"

#ifdef HAVE_LIB3DS
#include "loader3dsinternal.h"
#else
#warning lib3ds was not available. all all tests that require .3ds files will NOT function properly.
#endif
#include "modelloader/dcollide-modelloader-config.h"

#include <debug.h>

namespace ModelLoader {
    Loader3ds::Loader3ds() {
#ifdef HAVE_LIB3DS
        mLoader = new Loader3dsInternal(this);
#endif
    }

    Loader3ds::~Loader3ds() {
#ifdef HAVE_LIB3DS
        delete mLoader;
#endif
    }

    /*!
     * Like \ref loadFromFile, but loads the whole model into a single mesh.
     *
     * This may be useful to test the mesh-splitting algorithms of rigid meshes, but
     * it is probably most useful for loading deformable objects.
     *
     * Note that when using this method, \ref getTransformationNodes does \em
     * not provide any information. All transformations are included in the
     * returned mesh.
     *
     * \return A new mesh containing the loaded object or NULL if loading
     * failed for some reason. The caller is responsible for deleting the returned
     * object.
     *
     * \param fileName An (absolute) filename to the .3ds file
     * \param textureInformation (Dummy) texture information are placed into
     * this object, if it is non-NULL. Note that (unless the model in
     * \p fileName actually has a single mesh only) these texture information
     * will usually look "weird" and "wrong" on the model.
     */
    dcollide::Mesh* Loader3ds::loadFromFileToOneMesh(const char* fileName, TextureInformation* textureInformation) {
        mProxy2Transformation.clear();
#ifdef HAVE_LIB3DS
        return mLoader->loadFromFileToOneMesh(fileName, textureInformation);
#else
        return 0;
#endif
    }

    /*!
     * Load a .3ds model from file. This method keeps the hierarchy in the .3ds file
     * intact, by creating a \ref dcollide::Proxy hierarchy. If you want to load the
     * model into a single \ref dcollide::Mesh, use loadFromFileInOneMesh instead.
     *
     * The returned hierarchy is \em not transformed correctly - in fact all
     * nodes have no translation and no rotation applied at all. The caller is
     * responsible to apply the required transformations (as obtained by \ref
     * getTransformationNodes) manually. The reason for this is, that this
     * method assumes the caller also wants to apply these transformations to
     * non d-collide data structures (in particular e.g. OGRE data structures)
     * and therefore applying the transformations here would make things more
     * difficult.
     *
     * \return A new Proxy containing the loaded object or NULL if loading
     * failed for some reason. The caller is responsible for deleting the returned
     * object.
     *
     * \param fileName An (absolute) filename to the .3ds file
     * \param proxyType Used as proxyType parameter to the toplevel \ref Proxy
     */
    dcollide::Proxy* Loader3ds::loadFromFile(dcollide::World* world, const char* fileName, dcollide::ProxyTypes proxyType) {
        mProxy2Transformation.clear();
#ifdef HAVE_LIB3DS
        return mLoader->loadFromFile(world, fileName, proxyType);
#else
        return 0;
#endif
    }

}

/*
 * vim: et sw=4 ts=4
 */
