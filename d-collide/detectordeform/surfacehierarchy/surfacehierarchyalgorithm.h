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


#ifndef DCOLLIDE_SURFACE_HIERARCHY_ALGORITHM_H
#define DCOLLIDE_SURFACE_HIERARCHY_ALGORITHM_H

#include "detectordeform/detectordeformalgorithm.h"
#include "surfacehierarchy.h"
#include "shapes/shape.h"
#include "bvhnode.h"



namespace dcollide {

    class World;
    class Proxy;
    class SurfaceHierarchyProxyData;

    /*!
     * \brief Implementation of the surface hierarchy algorithm
     *
     * The surface hierarchy algorithm can operate on a connected mesh, as long
     * as the deformations are limited to a certain degree.
     *
     * The surface hierarchy builds and maintains a \ref BoundingSphere
     * hierarchy, which is created by the \ref SurfaceHierarchyCreator. See the
     * documentation of that class for details.
     *
     * Once the hierarchy has been built, deformations can be handled by simply
     * updating the center point of all spheres, since the deformations are
     * limited (therefore the spheres can be chosen large enough to cover the
     * maximal deformation, so the radius can remain constant).
     *
     * Collision detection is just a matter of simple BoundingVolume Hierarchy
     * Tree testing.
     */
    // TODO: document how much the deformations must be limited
    class SurfaceHierarchyAlgorithm : public DetectorDeformAlgorithm {
        public:
            SurfaceHierarchyAlgorithm(World* world, Pipeline* pipeline);
            ~SurfaceHierarchyAlgorithm();

            virtual bool supportsSelfCollisions() const;
            virtual bool supportsPairCollisions() const;
            virtual bool getListenForProxyChanges() const;

            virtual void prepareSimulation();

            virtual void createCollisionJobFor(const CollisionPair& pair);
            virtual void notifyAllCollisionPairsAreAdded();

            virtual DetectorDeformProxyData* createProxyData(Proxy* proxy);

        private:
            World* mWorld;

    };

}
/*
 * vim: et sw=4 ts=4
 */
#endif
