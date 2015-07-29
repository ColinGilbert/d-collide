/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,          *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz            *
 *                                                                             *
 *  All rights reserved.                                                       *
 *                                                                             *
 *  Redistribution and use in source and binary forms, with or without         *
 *  modification, are permitted provided that the following conditions are met:*
 *   - Redistributions of source code must retain the above copyright          *
       notice, this list of conditions and the following disclaimer.           *
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



#ifndef DCOLLIDE_NARROWPHASE_H
#define DCOLLIDE_NARROWPHASE_H

#include "narrowphasestrategy.h"
#include "boundingvolumecollision.h"

#include "real.h"

#include <list>
#include <vector>


namespace dcollide {

    //Forward declarations
    class Shape;
    class Sphere;
    class Box;
    class Cylinder;
    class Mesh;
    class Matrix;
    class BvhNode;
    class MeshPart;
    class Triangle;
    class Vertex;
    class BoundingVolume;
    struct CollisionInfo;
    class Proxy;
    class Vector3;
    struct PotentialCollidingSets;



    /*!
     * \brief Class to calculate detailed intersections between shapes
     */
    class NarrowPhase {
        public:
            NarrowPhase(const NarrowPhaseShapeStrategies& strategies);
            ~NarrowPhase();

            std::list<CollisionInfo>
                    getIntersection(const BoundingVolumeCollision& collision) const;

            std::list<CollisionInfo> getTriangleSetIntersection(PotentialCollidingSets& sets) const;

        private:
            /*!
             * \brief accuracy level definitions
             */
            NarrowPhaseStrategy mStrategySphereSphere;
            NarrowPhaseStrategy mStrategyMeshMesh;
            NarrowPhaseStrategy mStrategyBoxBox;
            NarrowPhaseStrategy mStrategyBoxSphere;
            NarrowPhaseStrategy mStrategySphereMesh;
            // TODO: add other combinations

            std::list<CollisionInfo> testVerticesAgainstPlanes(
                    const std::vector<Triangle*>& vertexTriangles, Proxy* vertexProxy,
                    const std::vector<Triangle*>& planeTriangles, Proxy* planeProxy) const;
            
            std::list<CollisionInfo> getSphereMeshIntersection(
                    const Sphere* sphere,
                    const std::vector<Triangle*> triangles,
                    Proxy* meshProxy) const;

            std::list<CollisionInfo> getBoxSphereIntersection(
                    const Matrix& prevStateBox, const Matrix& currentStateBox,
                    const Box* box,
                    const Matrix& prevStateSphere, const Matrix& currentStateSphere,
                    const Sphere* sphere) const;


            std::list<CollisionInfo> getMeshMeshIntersection(
                    const Mesh* mesh1, const Mesh* mesh2) const;

            std::list<CollisionInfo> getBvOverlapCollision(
                                const BoundingVolume* vol1,
                                const BoundingVolume* vol2,
                                Proxy* proxy1, Proxy* proxy2) const;

            std::list<CollisionInfo> getBoxBoxIntersection(
                    const Matrix& prevState1, const Matrix& currentState1,
                    Box* box1, const Matrix& prevState2,
                    const Matrix& currentState2, Box* box2) const;

    };

}

#endif // DCOLLIDE_NARROWPHASE_H
/*
 * vim: et sw=4 ts=4
 */
