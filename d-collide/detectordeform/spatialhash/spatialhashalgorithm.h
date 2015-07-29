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


#ifndef DCOLLIDE_SPATIAL_HASH_ALGORITHM_H
#define DCOLLIDE_SPATIAL_HASH_ALGORITHM_H

#include "detectordeform/detectordeformalgorithm.h"
#include "real.h"
#include <set>
#include <list>

namespace dcollide {

    class World;
    class Proxy;
    class SpatialHashJobCollection;
    class SpatialGrid;
    
    class Mesh;
    /*!
     * \brief The spatial hash algorithm detects collisions and selfcollisions of freely deformable proxies.
     * This algorithm is basically an implementation of the method decribed in "Optimized Spatial 
     * Hashing for Collision Detection of Deformable Objects". You may find this paper on B. Heidelbergers
     * personal homepage: \link http://www.beosil.com/publications.html
     *
     * This method uses a hashing algorithm to speed up the search for intersecting triangles in and between
     * objects. The algorithm is split into two phases, as described in the paper. The first phase, in the
     * following referenced as "phase one" is the update phase of the algorithm. All vertices which have
     * changed their position are classified against the dictionary. As phase one is rather performant
     * and to allow for generic deformations, we classify every vertice of all participating proxies, 
     * every frame against the hash. So every vertex may be transformed per frame, without any constraints.
     *
     * In phase two, the actual collision detection takes places. As described in the paper we don´t do
     * edge-edge collision detection. As a consequence the collision of rather large triangles may be
     * missed. Edge-edge collisions are rare especially when we consider the dense meshes used for
     * deformable objects.
     *
     * The division in phase one and two was done for performance reasons. The first implementation of this
     * algorithm classified triangles only. This turned out to be inefficent, because the bounding boxes
     * of adjacent triangles overlap a lot. A better solution is to only add the vertices and afterwards determine the
     * set of triangles starting at this vertice.
     * Multithreading:
     *
     * \li Phase 1, the update phase is not multithreaded.
     * \li Phase 2, the collision detection phase is multithreaded, the user may control the number of
     * triangles per job, via the define SPATIALHASH_TRIANGLES_PER_THREAD.
     * Class Overview:
     *
     * The \ref dcollide::SpatialHashAlgorithm implements DetectorDeform interface, and is therefore
     * responsible for job creation and interfacing with the \ref dcollide::pipeline
     * The  \ref dcollide::SpatialGrid class represents a three dimensional uniform subdivision of space
     * using a datastructure of type \ref dcollide::SpatialHash.
     * 
     */
    class SpatialHashAlgorithm : public DetectorDeformAlgorithm {
        public:
            SpatialHashAlgorithm(Pipeline* pipeline);
            ~SpatialHashAlgorithm();

            virtual bool supportsSelfCollisions() const;
            virtual bool supportsPairCollisions() const;
            virtual bool getListenForProxyChanges() const;

            virtual void prepareSimulation();

            virtual void createCollisionJobFor(const CollisionPair& pair);
            virtual void notifyAllCollisionPairsAreAdded();
            virtual void resetAlgorithm();

            virtual PipelineThreadJobCollection* getJobCollection();


            void createJobsForPhaseTwo();

        protected:
            std::list<const Mesh*> getMeshesFromProxy(const Proxy* p);
            real calculateAverageSideLength(const std::list<const Mesh*>& meshes) const;
        private:
            real mAverageSideLength;
            SpatialGrid* mSpatialGrid;
            SpatialHashJobCollection* mJobCollection;
            std::set<Proxy*> mProxies;
            
            std::list<const Mesh*> mMeshes;
    };

}
/*
 * vim: et sw=4 ts=4
 */
#endif
