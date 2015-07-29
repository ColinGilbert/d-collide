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

#include "modelloader3ds.h"
#include "myobjectnode.h"

#include <modelloader/loader3ds.h>

#include <d-collide/debug.h>
#include <d-collide/world.h>
#include <d-collide/proxy.h>
#include <d-collide/shapes/mesh.h>
#include <d-collide/shapes/mesh/vertex.h>

#include <iostream>

ModelLoader3ds::ModelLoader3ds() {
    mLoader = new ModelLoader::Loader3ds();
}

ModelLoader3ds::~ModelLoader3ds() {
    delete mLoader;
}

/*!
 * Load a .3ds model from file. This method keeps the hierarchy in the .3ds file
 * intact, by creating a \ref dcollide::Proxy hierarchy. If you want to load the
 * model into a single \ref dcollide::Mesh, use loadFromFileInOneMesh instead.
 *
 * \return A new MyObjectNode containing the loaded object or NULL if loading
 * failed for some reason. The caller is responsible for deleting the returned
 * object.
 *
 * \param fileName An (absolute) filename to the .3ds file
 * \param proxyType Used as proxyType parameter to the toplevel \ref MyObjectNode.
 */
MyObjectNode* ModelLoader3ds::loadFromFile(dcollide::World* world, const char* fileName, int proxyType) {
    if (!fileName) {
        // TODO: exception?
        return 0;
    }

    if (proxyType < 0) {
        proxyType = dcollide::PROXYTYPE_RIGID;
    }
    dcollide::Proxy* topProxy = mLoader->loadFromFile(world, fileName, (dcollide::ProxyTypes)proxyType);

    if (!topProxy) {
        return 0;
    }

    MyObjectNode* topObject = new MyObjectNode(topProxy);
    if (!loadNode(topObject)) {
        std::cerr << dc_funcinfo << "loading " << fileName << " failed" << std::endl;
        delete topObject;
        return false;
    }
    return topObject;
}

bool ModelLoader3ds::loadNode(MyObjectNode* node) {
    if (!node) {
        return false;
    }
    if (mLoader->getTransformationNodes().find(node->getProxy()) == mLoader->getTransformationNodes().end()) {
        std::cerr << dc_funcinfo << "internal error: proxy not in map" << std::endl;
        return false;
    }
    ModelLoader::Loader3ds::TransformationNode transformation = (*mLoader->getTransformationNodes().find(node->getProxy())).second;
    node->resetTransformation();
    node->translate(transformation.translation);
    node->rotate(transformation.rotation);

    const std::map<dcollide::Proxy*, ModelLoader::TextureInformation>& textureInformation = mLoader->getTextureInformation();
    if (textureInformation.find(node->getProxy()) != textureInformation.end()) {
        node->setTextureInformation((*textureInformation.find(node->getProxy())).second);
    }

    for (std::list<dcollide::Proxy*>::const_iterator it = node->getProxy()->getChildProxies().begin(); it != node->getProxy()->getChildProxies().end(); ++it) {
        MyObjectNode* child = new MyObjectNode(*it);
        loadNode(child);
        node->addChild(child);
    }
    return true;
}

/*!
 * See \ref ModelLoader::Loader3ds::loadFromFileToOneMesh
 */
MyObjectNode* ModelLoader3ds::loadFromFileInOneMesh(dcollide::World* world, const char* fileName, int proxyType) {
    ModelLoader::TextureInformation textureInformation;
    dcollide::Mesh* mesh = mLoader->loadFromFileToOneMesh(fileName, &textureInformation);
    MyObjectNode* topObject = new MyObjectNode(world, mesh, proxyType);
    topObject->setTextureInformation(textureInformation);
    return topObject;
}

/*
 * vim: et sw=4 ts=4
 */
