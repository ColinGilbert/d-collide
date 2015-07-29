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

#include "loaderdummy.h"

#include <shapes/mesh.h>
#include <shapes/mesh/vertex.h>
#include <shapes/mesh/triangle.h>
#include <debug.h>
#include <debugstream.h>

namespace ModelLoader {
    LoaderDummy::LoaderDummy() {
    }

    LoaderDummy::~LoaderDummy() {
    }

    /*!
     * Create a rectangular surface (i.e. 2D) made of triangles.
     *
     * \param width The width of the returned mesh
     * \param height The height of the returned mesh
     * \param edgeWidth The distance (in x direction) between two consecutive
     *                  triangles
     * \param edgeHeight The distance (in y direction) between two consecutive
     *                   triangles
     */
    dcollide::Mesh* LoaderDummy::createRectangleSurface(dcollide::real width,
            dcollide::real height,
            dcollide::real edgeWidth,
            dcollide::real edgeHeight) const {
        if (edgeWidth <= 0.0 || edgeHeight <= 0.0) {
            std::cout << dc_funcinfo << "invalid edge parameter" << std::endl;
            return 0;
        }
        if (width <= 0.0 || height <= 0.0) {
            std::cout << dc_funcinfo << "invalid dimension parameter" << std::endl;
            return 0;
        }

        // AB: we simply put a raster on the (width,height) rect.
        //     -> ceilf(width/edgeWidth) is the number of cells per row
        //     -> ceilf(height/edgeHeight) is the number of cells per column

        int horizontalSteps = (int)ceil(width / edgeWidth);
        int verticalSteps = (int)ceil(height / edgeHeight);
        if (horizontalSteps * verticalSteps <= 0) {
            std::cerr << dc_funcinfo << "ERROR: horizontalSteps=" << horizontalSteps << " verticalSteps=" << verticalSteps << std::endl;
            return 0;
        }
        std::vector<dcollide::Vertex*> vertices;
        std::vector<dcollide::Triangle*> triangles;
        vertices.reserve((horizontalSteps + 1) * (verticalSteps + 1));
        triangles.reserve((horizontalSteps) * (verticalSteps) * 2);

        const dcollide::real z = 0.0;
        for (int _x = 0; _x < horizontalSteps + 1; _x++) {
            float x = _x * edgeWidth;
            if (x > width) {
                x = width;
            }
            for (int _y = 0; _y < verticalSteps + 1; _y++) {
                float y = _y * edgeHeight;
                if (y > height) {
                    y = height;
                }
                dcollide::Vertex* vertex = new dcollide::Vertex(x, y, z);
                vertices.push_back(vertex);
            }
        }
        if ((int)vertices.size() != (horizontalSteps + 1) * (verticalSteps + 1)) {
            std::cerr << dc_funcinfo << "internal error: unexpected vertex count: " << vertices.size() << ", expected " << (horizontalSteps+1)*(verticalSteps+1) << std::endl;
            for (unsigned int i = 0; i < vertices.size(); i++) {
                delete vertices[i];
            }
            return 0;
        }
        for (int x = 0; x < (horizontalSteps + 1 - 1); x++) {
            for (int y = 0; y < (verticalSteps + 1 - 1); y++) {
                int x1 = x;
                int x2 = x + 1;
                int y1 = y;
                int y2 = y + 1;
                dcollide::Vertex* v1 = vertices[x1 * (verticalSteps + 1) + y1];
                dcollide::Vertex* v2 = vertices[x2 * (verticalSteps + 1) + y1];
                dcollide::Vertex* v3 = vertices[x2 * (verticalSteps + 1) + y2];
                dcollide::Vertex* v4 = vertices[x1 * (verticalSteps + 1) + y2];
                dcollide::Triangle* t1 = new dcollide::Triangle(v1, v2, v3);
                dcollide::Triangle* t2 = new dcollide::Triangle(v1, v3, v4);
                triangles.push_back(t1);
                triangles.push_back(t2);
            }
        }

        if ((int)triangles.size() != (horizontalSteps * verticalSteps * 2)) {
            std::cerr << dc_funcinfo << "ERROR: unexpected triangle count: " << triangles.size() << ", expected " << (horizontalSteps*verticalSteps) << std::endl;
            for (unsigned int i = 0; i < triangles.size(); i++) {
                delete triangles[i];
            }
            for (unsigned int i = 0; i < vertices.size(); i++) {
                delete vertices[i];
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
