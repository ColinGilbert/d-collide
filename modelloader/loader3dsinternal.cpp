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

#include "loader3dsinternal.h"

#include <world.h>
#include <proxy.h>
#include <shapes/mesh.h>
#include <shapes/mesh/vertex.h>
#include <exceptions/exception.h>
#include <debugstream.h>

#include <lib3ds/file.h>
#include <lib3ds/node.h>
#include <lib3ds/mesh.h>
#include <lib3ds/matrix.h>
#include <lib3ds/material.h>

#include <iostream>
#include <cstring>


namespace ModelLoader {
    Loader3dsInternal::Loader3dsInternal(Loader3ds* data) {
        mData = data;
    }

    Loader3dsInternal::~Loader3dsInternal() {
        if (mFile) {
            lib3ds_file_free(mFile);
            mFile = 0;
        }
    }

    /*!
     * See \ref Loader3ds::loadFromFileToOneMesh
     */
    dcollide::Mesh* Loader3dsInternal::loadFromFileToOneMesh(const char* fileName, TextureInformation* textureInformation) {
        // AB:
        // we cannot load texture information here, because we are merging all
        // meshes into a single one.
        // -> adjusting the texels may still be possible, however if the model
        //    uses > 1 texture, we can't change that.

        mData->mProxy2Transformation.clear();
        mData->mProxy2TextureInformation.clear();
        if (!fileName) {
            throw dcollide::NullPointerException("fileName");
        }

        mFile = lib3ds_file_load(fileName);
        if (!mFile) {
            std::cout << dc_funcinfo << "unable to load " << fileName << std::endl;
            return 0;
        }

        // AB: some files don't store nodes and just want exactly one node per mesh.
        //     atm we don't support that.
        if (!mFile->nodes) {
            std::cout << dc_funcinfo << "File " << fileName << " does not contain any nodes. mesh-only files not supported currently." << std::endl;
            lib3ds_file_free(mFile);
            mFile = 0;
            return 0;
        }

        // 3ds stores several frames, we want the first
        lib3ds_file_eval(mFile, 0);

        std::list<Lib3dsNode*> nodes;
        Lib3dsNode* node = mFile->nodes;
        while (node) {
            nodes.push_back(node);
            node = node->next;
        }

        bool texelLoadError = false;
        std::vector<dcollide::Vector3> texels;
        std::vector<dcollide::Vertex*> vertices;
        std::vector<dcollide::Triangle*> triangles;
        std::vector<int> indices;
        while (!nodes.empty()) {
            Lib3dsNode* node = nodes.front();
            nodes.pop_front();

            for (Lib3dsNode* n = node->childs; n; n = n->next) {
                nodes.push_back(n);
            }

            // node->type can be object, light, camera, ...
            // -> only object nodes are relevant to us
            if (node->type != LIB3DS_OBJECT_NODE) {
                continue;
            }

            if (strcmp(node->name, "$$$DUMMY") == 0) {
                // AB: nodes with this name are only transformation nodes, i.e. they
                // don't have a mesh, only a matrix.
                continue;
            }

            unsigned int pointOffset = vertices.size();

            Lib3dsMesh* mesh = lib3ds_file_mesh_by_name(mFile, node->name);
            Lib3dsMatrix origMeshMatrix;
            lib3ds_matrix_copy(origMeshMatrix, mesh->matrix);
            lib3ds_matrix_inv(origMeshMatrix); // 3ds stores inverted mesh matrix
            dcollide::Matrix meshMatrix(&origMeshMatrix[0][0]);
            dcollide::Matrix nodeMatrix(&node->matrix[0][0]);

            Lib3dsObjectData* data = &node->data.object;
            nodeMatrix.translate(dcollide::Vector3(-data->pivot[0], -data->pivot[1], -data->pivot[2]));

            dcollide::Matrix matrix = nodeMatrix;
            matrix.multiply(&meshMatrix);

            for (unsigned int i = 0; i < mesh->points; i++) {
                Lib3dsPoint* p = &mesh->pointL[i];
                dcollide::Vector3 pos;
                matrix.transform(&pos, dcollide::Vector3(
                            p->pos[0],
                            p->pos[1],
                            p->pos[2]
                            ));
                vertices.push_back(new dcollide::Vertex(pos));
            }

            for (unsigned int i = 0; i < mesh->faces; i++) {
                Lib3dsFace* f = &mesh->faceL[i];
                indices.push_back(f->points[0] + pointOffset);
                indices.push_back(f->points[1] + pointOffset);
                indices.push_back(f->points[2] + pointOffset);
            }

            if (textureInformation) {
                TextureInformation t = loadTextureInformation(node);
                const std::vector<dcollide::Vector3>& nodeTexels = t.getTexels();
                if (mesh->texelL && nodeTexels.size() != mesh->texels) {
                    dcollide::warning() << "texturing problem in " << fileName;
                    dcollide::warning() << "invalid texel count loaded: have=" << nodeTexels.size() << " expected=" << mesh->texels;
                    dcollide::warning() << "adding dummy texels...";
                    for (unsigned int i = 0; i < mesh->texels; i++) {
                        texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));
                    }
//                    texelLoadError = true;
                } else {
                    for (unsigned int i = 0; i < nodeTexels.size(); i++) {
                        texels.push_back(nodeTexels[i]);
                    }
                }
            }
        }

        if (textureInformation) {
            if (!texelLoadError) {
                textureInformation->setTextured(true);
                textureInformation->setTextureFileName("deformable.tga");
                textureInformation->setTexels(texels);
            } else {
                textureInformation->setTextured(false);
            }
        }

        lib3ds_file_free(mFile);
        mFile = 0;

        dcollide::Mesh* mesh = new dcollide::Mesh(vertices, indices);
        return mesh;
    }

    /*!
     * See \ref Loader3ds::loadFromFile
     */
    dcollide::Proxy* Loader3dsInternal::loadFromFile(dcollide::World* world, const char* fileName, dcollide::ProxyTypes proxyType) {
        mData->mProxy2Transformation.clear();
        mData->mProxy2TextureInformation.clear();
        if (!fileName) {
            // TODO: exception?
            return 0;
        }

        mFile = lib3ds_file_load(fileName);
        if (!mFile) {
            std::cout << dc_funcinfo << "unable to load " << fileName << std::endl;
            return 0;
        }

        // AB: some files don't store nodes and just want exactly one node per mesh.
        //     atm we don't support that.
        if (!mFile->nodes) {
            std::cout << dc_funcinfo << "File " << fileName << " does not contain any nodes. mesh-only files not supported currently." << std::endl;
            lib3ds_file_free(mFile);
            mFile = 0;
            return 0;
        }

        // 3ds stores several frames, we want the first
        lib3ds_file_eval(mFile, 0);

        if (!checkUniqueMeshMaterial()) {
            std::cerr << dc_funcinfo << "the file " << fileName << " contains at least one mesh with different materials. this is not supported by this modelloader, only the first material will be used!" << std::endl;
        }

        dcollide::Proxy* topObject = world->createProxy(proxyType);
        mData->mProxy2Transformation.insert(std::make_pair(topObject, Loader3ds::TransformationNode()));

        Lib3dsMatrix identityMatrix;
        lib3ds_matrix_identity(identityMatrix);
        Lib3dsNode* node = mFile->nodes;
        while (node) {
            // node->type can be object, light, camera, ...
            // -> only object nodes are relevant to us
            if (node->type == LIB3DS_OBJECT_NODE) {
                if (!loadNode(world, topObject, node, &identityMatrix)) {
                    std::cerr << dc_funcinfo << "Loading node from " << fileName << " failed" << std::endl;
                    // TODO: is MyObjectNode::mProxy being deleted?
                    delete topObject;
                    lib3ds_file_free(mFile);
                    mFile = 0;
                    return 0;
                }
            }

            node = node->next;
        }

        lib3ds_file_free(mFile);
        mFile = 0;
        return topObject;
    }

    /*!
     * \internal
     *
     * The objects created as children of \p parent will \em not be rotated or
     * translated correctly. Instead the required transformations are stored and
     * provided to the user, see \ref Loader3ds::getTransformationNodes
     */
    bool Loader3dsInternal::loadNode(dcollide::World* world, dcollide::Proxy* parent, Lib3dsNode* node, Lib3dsMatrix* parentTranslateRotateMatrix) {
        if (!parent || !node) {
            return false;
        }
        if (node->type != LIB3DS_OBJECT_NODE) {
            return false;
        }
        Lib3dsObjectData* data = &node->data.object;

        Lib3dsMatrix translateRotateMatrix;
        lib3ds_matrix_copy(translateRotateMatrix, *parentTranslateRotateMatrix);
        lib3ds_matrix_translate(translateRotateMatrix, data->pos);
        lib3ds_matrix_rotate(translateRotateMatrix, data->rot);

        dcollide::Shape* shape = createShape(node, &translateRotateMatrix);
        dcollide::Proxy* object = world->createProxy(shape);

        mData->mProxy2TextureInformation.insert(std::make_pair(object, loadTextureInformation(node)));

        dcollide::Vector3 scale(data->scl[0], data->scl[1], data->scl[2]);
        dcollide::Vector3 translation(data->pos[0], data->pos[1], data->pos[2]);

        Lib3dsMatrix lib3dsRotationMatrix;
        lib3ds_matrix_identity(lib3dsRotationMatrix);
        lib3ds_matrix_rotate(lib3dsRotationMatrix, data->rot);
        dcollide::Matrix rotation(&lib3dsRotationMatrix[0][0]);

        Loader3ds::TransformationNode transformation;
        transformation.translation = translation;
        transformation.rotation = rotation;

        mData->mProxy2Transformation.insert(std::make_pair(object, transformation));

        for (Lib3dsNode* n = node->childs; n; n = n->next) {
            if (!loadNode(world, object, n, &translateRotateMatrix)) {
                std::cerr << "Failed loading node " << n->name << std::endl;
                // TODO: delete object->getProxy() ?
                delete object;
                return false;
            }
        }

        parent->addChild(object);

        return true;
    }

    /*!
     * \internal
     *
     * \return A new mesh object containing the mesh in \p node, or NULL if no such
     * mesh can be found.
     */
    dcollide::Shape* Loader3dsInternal::createShape(Lib3dsNode* node, Lib3dsMatrix* translateRotateMatrix) {
        if (!node) {
            return 0;
        }
        if (node->type != LIB3DS_OBJECT_NODE) {
            return 0;
        }
        if (strcmp(node->name, "$$$DUMMY") == 0) {
            // AB: nodes with this name are only transformation nodes, i.e. they
            // don't have a mesh, only a matrix.
            return 0;
        }
        Lib3dsMesh* mesh = lib3ds_file_mesh_by_name(mFile, node->name);
        if (!mesh) {
            std::cout << "Cannot find mesh " << node->name << " in 3ds file" << std::endl;
            return 0;
        }
        if (mesh->faces < 1) {
            std::cerr << "No faces in mesh " << node->name << std::endl;
            return 0;
        }
        if (mesh->points < 3) {
            std::cerr << "Less than 3 points in mesh " << node->name << std::endl;
            return 0;
        }

        Lib3dsMatrix origMeshMatrix;
        lib3ds_matrix_copy(origMeshMatrix, mesh->matrix);
        lib3ds_matrix_inv(origMeshMatrix); // 3ds stores inverted mesh matrix
        dcollide::Matrix meshMatrix(&origMeshMatrix[0][0]);


        // AB: Ogre does not apply "scale" values to child nodes, lib3ds does.
        //     so to display the model correctly we remove the scale completely
        //     (we integrate it directly into the vertices).
        //     to do this:
        //       let M:=the lib3ds matrix the point is normally transformed by
        //              (i.e. including the parent-node matrix)
        //       let M':=the matrix d-collide will use (i.e.
        //               translation+rotation only, no scale)
        //     then for every vertex v the correct transformed position p is:
        //       Mv=p
        //     we search for a vertex v' so that:
        //       M'v'=p
        //     since we have M and v (and thus p) and M' we can do this like
        //     this:
        //       v'=(M'^-1)p
        //     which equals:
        //       v'=(M'^-1)Mv

        dcollide::Matrix nodeMatrix(&node->matrix[0][0]); // M
        Lib3dsMatrix invertedTranslateRotate3ds;
        lib3ds_matrix_copy(invertedTranslateRotate3ds, *translateRotateMatrix);
        lib3ds_matrix_inv(invertedTranslateRotate3ds);
        dcollide::Matrix invertedTranslateRotate(&invertedTranslateRotate3ds[0][0]); // M'^-1
        dcollide::Matrix removeScaleMatrix(invertedTranslateRotate);
        removeScaleMatrix.multiply(&nodeMatrix);

        Lib3dsObjectData* data = &node->data.object;
        std::vector<dcollide::Vertex*> vertices(mesh->points);
        for (unsigned int i = 0; i < mesh->points; i++) {
            Lib3dsPoint* p = &mesh->pointL[i];
            dcollide::Vector3 meshPos;
            meshMatrix.transform(&meshPos, dcollide::Vector3(
                        p->pos[0] - data->pivot[0],
                        p->pos[1] - data->pivot[1],
                        p->pos[2] - data->pivot[2]));

            dcollide::Vector3 pos;
            removeScaleMatrix.transform(&pos, meshPos);

            vertices[i] = new dcollide::Vertex(pos);
        }

        std::vector<int> indices(mesh->faces * 3);
        for (unsigned int i = 0; i < mesh->faces; i++) {
            Lib3dsFace* f = &mesh->faceL[i];
            indices[i * 3 + 0] = f->points[0];
            indices[i * 3 + 1] = f->points[1];
            indices[i * 3 + 2] = f->points[2];
        }

        dcollide::Mesh* newMesh = new dcollide::Mesh(vertices, indices);

        return newMesh;
    }

    TextureInformation Loader3dsInternal::loadTextureInformation(Lib3dsNode* node) {
        if (!node) {
            return TextureInformation();
        }
        if (node->type != LIB3DS_OBJECT_NODE) {
            return TextureInformation();
        }
        if (strcmp(node->name, "$$$DUMMY") == 0) {
            // AB: nodes with this name are only transformation nodes, i.e. they
            // don't have a mesh, only a matrix.
            return TextureInformation();
        }
        Lib3dsMesh* mesh = lib3ds_file_mesh_by_name(mFile, node->name);
        if (!mesh) {
            std::cout << "Cannot find mesh " << node->name << " in 3ds file" << std::endl;
            return TextureInformation();
        }
        if (mesh->faces < 1) {
            std::cerr << "No faces in mesh " << node->name << std::endl;
            return TextureInformation();
        }

        if (mesh->texelL == 0) {
            // mesh not textured.
            return TextureInformation();
        }

        if (mesh->texels != mesh->points) {
            std::cerr << "ERROR: mesh->texels != mesh->points, with non-NULL mesh->texelL" << std::endl;
            return TextureInformation();
        }

        if (mesh->faceL[0].material[0] == 0) {
            return TextureInformation();
        }

        // AB: at this point we assume that the mesh uses a single material
        // only. see checkUniqueMeshMaterial()
        Lib3dsMaterial* mat = lib3ds_file_material_by_name(mFile, mesh->faceL[0].material);

        TextureInformation textureInformation;
        textureInformation.setTextured(true);
        std::vector<dcollide::Vector3> texels;
        texels.reserve(mesh->texels);
        dcollide::Matrix textureMatrix;
        Lib3dsTextureMap* t = &mat->texture1_map;
        if (t->scale[0] && t->scale[1]) {
            // AB: see boson's bobmfloader/loaders/loader-3ds.cpp for details
            //     (remark: yes, I have permission to relicense to BSD!)
            textureMatrix.translate((dcollide::real)((1.0 - t->scale[0]) / 2.0),
                                    (dcollide::real)((1.0 - t->scale[1]) / 2.0),
                                    (dcollide::real) 0.0);
            textureMatrix.scale(t->scale[0], t->scale[1], 1.0);
        }
        if (t->rotation != 0.0) {
            textureMatrix.rotate(-t->rotation, 0.0, 0.0, 1.0);
        }
        textureMatrix.translate(-t->offset[0], -t->offset[1], 0.0);
        textureMatrix.translate(mesh->map_data.pos[0], mesh->map_data.pos[1], mesh->map_data.pos[2]);
        float scale = mesh->map_data.scale;
        if (scale != 0.0) {
            textureMatrix.scale(scale, scale, 1.0);
        }

        dcollide::Vector3 tmp1;
        dcollide::Vector3 tmp2;
        for (unsigned int i = 0; i < mesh->texels; i++) {
            tmp1.set((dcollide::real)(mesh->texelL[i])[0], (dcollide::real)(mesh->texelL[i])[1], (dcollide::real)0.0);
            textureMatrix.transform(&tmp2, tmp1);
            texels.push_back(tmp2);
        }
        textureInformation.setTexels(texels);

        textureInformation.setTextureFileName(mat->texture1_map.name);

        return textureInformation;
    }

    /*!
     * Check whether all meshes in this file use a single material, i.e. all
     * faces use the same material.
     *
     * \return TRUE if all meshes use a single material, FALSE if at least one
     * mesh uses several materials
     */
    bool Loader3dsInternal::checkUniqueMeshMaterial() const {
        for (Lib3dsMesh* mesh = mFile->meshes; mesh; mesh = mesh->next) {
            if (mesh->faces == 0) {
                continue;
            }
            for (unsigned int i = 1; i < mesh->faces; i++) {
                if (strncmp(mesh->faceL[0].material, mesh->faceL[i].material, 64) != 0) {
                    return false;
                }
            }
        }
        return true;
    }
}

/*
 * vim: et sw=4 ts=4
 */
