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

#include "loaderply.h"

#include <proxy.h>
#include <shapes/mesh.h>
#include <shapes/mesh/vertex.h>
#include <shapes/mesh/triangle.h>
#include <debugstream.h>

#include "rply/rply.h"

#include <iostream>


static int vertexCallback(p_ply_argument arg) {
    std::vector<double>* vertexComponents = 0;
    void* pointer = 0;
    ply_get_argument_user_data(arg, &pointer, NULL);
    vertexComponents = static_cast<std::vector<double>*>(pointer);

    if (!vertexComponents) {
        std::cerr << dc_funcinfo << "internal error" << std::endl;
        return 0; // abort loading
    }

    double v = ply_get_argument_value(arg);// * 1000.0;
    vertexComponents->push_back(v);

    return 1;
}


static int faceCallback(p_ply_argument arg) {
    std::vector<int>* faceComponents = 0;
    void* pointer = 0;
    ply_get_argument_user_data(arg, &pointer, NULL);
    faceComponents = static_cast<std::vector<int>*>(pointer);

    if (!faceComponents) {
        std::cerr << dc_funcinfo << "internal error" << std::endl;
        return 0; // abort loading
    }

    long valueIndex = 0;
    long length;
    ply_get_argument_property(arg, NULL, &length, &valueIndex);
    switch (valueIndex) {
        case 0:
        case 1:
        case 2:
        {
            int vertexIndex = (int)ply_get_argument_value(arg);
            faceComponents->push_back(vertexIndex);
            break;
        }
        case -1:
        {
            int count = (int)ply_get_argument_value(arg);
            if (count != 3) {
                std::cerr << dc_funcinfo << "unexpected list count " << count << std::endl;
                return 0; // abort loading
            }
            break;
        }
        default:
            std::cerr << dc_funcinfo << "unexpected valueIndex: " << valueIndex << std::endl;
            return 0; // abort loading
    }

    return 1;
}


namespace ModelLoader {
    LoaderPly::LoaderPly() {
    }

    LoaderPly::~LoaderPly() {
    }

    dcollide::Mesh* LoaderPly::loadFromFileToOneMesh(const char* fileName, double scale) {
        if (!fileName) {
            // TODO: exception?
            return 0;
        }

        p_ply ply = ply_open(fileName, NULL);
        if (!ply) {
            std::cerr << dc_funcinfo << "unable to load " << fileName << std::endl;
            return 0;
        }
        if (!ply_read_header(ply)) {
            std::cerr << dc_funcinfo << "unable to load " << fileName << ": reading header failed" << std::endl;
            return 0;
        }

        long vertexCount = 0;
        std::vector<double> vertexComponents;
        vertexCount = ply_set_read_cb(ply, "vertex", "x", vertexCallback, &vertexComponents, 0);
        ply_set_read_cb(ply, "vertex", "y", vertexCallback, &vertexComponents, 0);
        ply_set_read_cb(ply, "vertex", "z", vertexCallback, &vertexComponents, 0);
        vertexComponents.reserve(vertexCount * 3);


        long triangleCount = 0;
        std::vector<int> faceComponents;
        triangleCount = ply_set_read_cb(ply, "face", "vertex_indices", faceCallback, &faceComponents, 0);
        faceComponents.reserve(triangleCount * 3);

        if (!ply_read(ply)) {
            std::cerr << dc_funcinfo << "loading " << fileName << " failed" << std::endl;
            ply_close(ply);
            return 0;
        }
        ply_close(ply);

        if (vertexComponents.size() != vertexComponents.capacity()) {
            std::cerr << dc_funcinfo
                        << "not all vertex components loaded. Expected: "
                        << vertexComponents.capacity()
                        << " have: "
                        << vertexComponents.size()
                        << std::endl;
            return 0;
        }



        if (faceComponents.size() != faceComponents.capacity()) {
            std::cerr << dc_funcinfo
                        << "not all face components loaded. Expected: "
                        << faceComponents.capacity()
                        << " have: "
                        << faceComponents.size()
                        << std::endl;
            return 0;
        }

        std::vector<dcollide::Vertex*> vertices;
        std::vector<dcollide::Triangle*> triangles;

        vertices.reserve(vertexCount);
        triangles.reserve(triangleCount);

        for (int i = 0; i < vertexCount; i++) {
            double x = vertexComponents.at(i * 3 + 0);
            double y = vertexComponents.at(i * 3 + 1);
            double z = vertexComponents.at(i * 3 + 2);

            x *= scale;
            y *= scale;
            z *= scale;

            dcollide::Vertex* v = new dcollide::Vertex((dcollide::real)x, (dcollide::real)y, (dcollide::real)z);
            vertices.push_back(v);
        }



        for (int i = 0; i < triangleCount; i++) {
            int index1 = faceComponents.at(i * 3 + 0);
            int index2 = faceComponents.at(i * 3 + 1);
            int index3 = faceComponents.at(i * 3 + 2);

            dcollide::Vertex* v1 = vertices.at(index1);
            dcollide::Vertex* v2 = vertices.at(index2);
            dcollide::Vertex* v3 = vertices.at(index3);


            dcollide::Triangle* t = new dcollide::Triangle(v1, v2, v3);
            triangles.push_back(t);
        }
        if (vertices.empty() || triangles.empty()) {
            dcollide::debug() << "could not load from file " << fileName << ": verticesCount=" << vertices.size() << " trianglesCount=" << triangles.size();
            for (std::vector<dcollide::Vertex*>::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {
                delete *it;
            }
            for (std::vector<dcollide::Triangle*>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
                delete *it;
            }
            return 0;
        }
        dcollide::Mesh* mesh = new dcollide::Mesh(vertices, triangles);

        return mesh;
    }
}

/*
 * vim: et sw=4 ts=4
 */
