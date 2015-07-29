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

// author: Marc Schulz <shade@nightmareshadow.de>

#ifndef DCOLLIDE_SURFACE_HIERARCHY_JOB_H
#define DCOLLIDE_SURFACE_HIERARCHY_JOB_H

#include "shapes/shape.h"

#include "detectordeform/detectordeformjob.h"

#include "detectordeform/trianglepair.h"

#include <list>
#include <vector>
#include <set>

namespace dcollide {

    class World;
    class Proxy;
    class Triangle;
    class Vertex;
    class DeformableBvhNode;
    class SurfaceHierarchyProxyData;

    class SurfaceHierarchyJob : public  DetectorDeformJob {
        public:
            //SurfaceHierarchyJob(SurfaceHierarchyProxyData* proxyData1, SurfaceHierarchyProxyData* proxyData2);
            SurfaceHierarchyJob(Proxy* proxy1, Proxy* proxy2, int algoIndex);
            ~SurfaceHierarchyJob();

            virtual void run();

        protected:

        private:

            std::list<TrianglePair> mTrianglePairs;

            Proxy* mProxy1;
            Proxy* mProxy2;

            const int mAlgoIndex;

            SurfaceHierarchyProxyData* mProxyData1;
            SurfaceHierarchyProxyData* mProxyData2;

            //std::vector<Triangle*> mTriangles_1;
            //std::vector<Triangle*> mTriangles_2;

            void checkCollisionType(Proxy* proxy1, Proxy* proxy2);
            void calculateMinimalCollisions(const BvhNode* p1_work, const BvhNode* p2_work, Proxy* prox1, Proxy* prox2);
            void calculateMinimalCollisionsWithRigid(const BvhNode* p1_work, const BvhNode* p2_work, Proxy* p1, Proxy* p2);
            void calculateMinimalCollisionsWithShape(const BvhNode* p1_work, const BvhNode* shapeNode, Proxy* p1, Proxy* p2);
            inline bool intersectSpheres(const BvhNode* sphere1, const BvhNode* sphere2);
            inline bool intersectSphereTriangle(const BvhNode* sphere, Triangle* triangle);
            inline bool intersectBoundingVolumeTriangle(const BvhNode* node, Triangle* triangle);
    };

}
/*
 * vim: et sw=4 ts=4
 */
#endif // DCOLLIDE_SURFACE_HIERARCHY_JOB_H