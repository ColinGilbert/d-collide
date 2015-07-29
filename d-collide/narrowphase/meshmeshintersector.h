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


#ifndef DCOLLIDE_MESHMESHINTERSECTOR_H
#define DCOLLIDE_MESHMESHINTERSECTOR_H

#include <list>

#include "real.h"
#include "../shapes/mesh.h"
//#include "detectordeform/trianglepair.h" // TODO: remove if obsolete
#include "worldcollisions.h"
#include "dcollide-defines.h"
#include "narrowphase/narrowphasestrategy.h"

// less collision points than this threshold may lead to a bad regression plane
#define COLLISION_POINTS_THRESHOLD 5

// TODO: remove the debug output controlled by this define
//#define MESHMESHPLANEDEBUG

namespace dcollide {
    class Matrix;
    class Mesh;
    struct CollisionInfo;
    class Vector3;
    class Plane;

    class MeshMeshIntersector {
        public:
            MeshMeshIntersector(NarrowPhaseStrategy strategy =
                (NarrowPhaseStrategy) NARROWPHASE_DEFAULT_STRATEGY_MESH_MESH);
            ~MeshMeshIntersector();
            std::list<CollisionInfo> getIntersection(const Mesh* mesh1,
                    const Mesh* mesh2, bool swapped = false) const;
            std::list<CollisionInfo> getIntersection(
                    std::list<BoundingVolumeCollision>& bvCollisions) const;
            std::list<CollisionInfo> getIntersection(
                    std::vector<Triangle*>& triangles1, Proxy* proxy1,
                    std::vector<Triangle*>& triangles2, Proxy* proxy2) const;
            void unifyCollisionInfo(std::list<CollisionInfo>& result) const;

            void thinOutCollisionInfo(std::list<CollisionInfo>& result) const;
        private:
            NarrowPhaseStrategy mStrategy;

            void calculateTriangleIntersections(
                    const std::vector<Triangle*>& triangles1,
                    const std::vector<Triangle*>& triangles2,
                    std::list<CollisionInfo>& result,
                    bool fullCheck = true) const;
            void calculateBvIntersections(
                    BoundingVolumeCollision& bvCollision,
                    std::list<CollisionInfo>& result) const;
            bool isTooFewOrTooClose(std::list<CollisionInfo>& pointList,
                    real minDistance = 0.01f) const;
            Plane calculateRegressionPlane(
                std::list<CollisionInfo>& result, unsigned long* count = 0,
                Vector3* average = 0, real* averageXX = 0, real* averageXY = 0,
                real* averageYY = 0, real* averageYZ = 0, real* averageXZ = 0)
                const;
            bool assignHalfSpaces(Plane& dividingPlane,
                            const Vector3& center1, const Vector3& center2,
                            bool forceDividing = false) const;
            real calculateHighestDepth(const Mesh* mesh, Plane& plane,
                bool inFront, std::list<CollisionInfo>& result) const;
            real calculateDepthFast(const Mesh* mesh, Plane& plane,
                bool inFront, CollisionInfo& startingPoint) const;
            void calculatePenetratingVertices(
                const std::set<Vertex*>& vertices,
                Plane& plane, bool inFront, std::list<CollisionInfo>& result,
                Proxy* penetratingProxy, Proxy* penetratedProxy) const;

            std::set<Vertex*> collectVertices(std::vector<Triangle*> triangles,
                        Vector3& centerPoint) const;
    };
}

#endif // DCOLLIDE_MESHMESHINTERSECTOR_H
/*
 * vim: et sw=4 ts=4
 */
