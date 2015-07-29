/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-users@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,          *
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

#ifndef DCOLLIDE_MESHFACTORY_H
#define DCOLLIDE_MESHFACTORY_H

#include "math/vector.h"
#include "real.h"
#include "datatypes/multimapelement.h"

#include <set>
#include <vector>

namespace dcollide {
    class Mesh;
    class Sphere;
    class Wedge;
    class Box;
    class Cone;
    class Cylinder;
    class Vertex;
    class Triangle;

    /*!
     * \brief Factory to create several Mesh-constructs
     */
    class MeshFactory {

        public:
            MeshFactory();
            ~MeshFactory();

            Mesh* createTetraeder(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3);
            Mesh* createSphere(real radius, real averageEdgeLength = 0);
            Mesh* createSphere(const Sphere& sphere);
            Mesh* createBox(Vector3 dimension);
            Mesh* createBox(const Box& box);
            Mesh* createWedge(const real* dimension);
            Mesh* createWedge(const Wedge& wedge);
            Mesh* createCone(real radius, real height,real averageEdgeLength = 0);
            Mesh* createCone(const Cone& cone);
            Mesh* createCylinder(real radius, real height,real averageEdgeLength = 0);
            Mesh* createCylinder(const Cylinder& cylinder);

        private:
            void subdivide(std::vector<Vertex*>& vertices,
                 std::vector<Triangle*>& triangles,
                 std::set<Triangle*>& redundantTriangles,
                 std::set<MultiMapElement<Vertex, Vertex*> >& combinations,
                 Triangle* triangle,real radius,real averageEdgeLength = 0);
    };

/*
    Mesh* MeshFactory::createSphere(const Sphere& sphere) {
        return createSphere(sphere.getRadius(),sphere.getAverageEdgeLength());
    }
    Mesh* MeshFactory::createBox(const Box& box) {
        return createBox(box.getDimension());
    }
    Mesh* MeshFactory::createWedge(const Wedge& wedge) {
        return createWedge(wedge.getDimension());
    }
    Mesh* MeshFactory::createCone(const Cone& cone) {
        return createCone(cone.getRadius(),cone.getHeight(),cone.getAverageEdgeLength());
    }
*/
}

#endif // DCOLLIDE_MESHFACTORY_H
/*
 * vim: et sw=4 ts=4
 */
