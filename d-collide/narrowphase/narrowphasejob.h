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


#ifndef DCOLLIDE_NARROWPHASEJOB_H
#define DCOLLIDE_NARROWPHASEJOB_H

#include "pipeline.h"
#include "datatypes/list.h"
#include "narrowphase/narrowphasestrategy.h"
#include "detectordeform/trianglepair.h"

#include "detectordeform/potentialcollidingsets.h"
#include <list>

namespace dcollide {
    struct BoundingVolumeCollision;
    class World;
    class ThreadPool;

    /*!
     * \brief calls NarrowPhase for some given BoundingVolumeCollisions
     *
     */
    class NarrowPhaseJob  : public PipelineThreadJob {
        public:
            explicit NarrowPhaseJob(unsigned int jobPoolIndex,
                                    const NarrowPhaseShapeStrategies& strategies);
            ~NarrowPhaseJob();

            void assignInput(std::list<PotentialCollidingSets>& middlePhaseCollision);
            void addInput(const BoundingVolumeCollision& middlePhaseCollision);
            void addInput(const TrianglePair& middlePhaseCollision);

            virtual void processResults(Pipeline* pipeline);
            virtual void run();

            std::list<CollisionInfo>& getNarrowPhaseCollisions();

        private:
        	void createCollisionInfo(const Triangle* tri1, const Proxy* proxy1,
        	                         const Triangle* tri2, const Proxy* proxy2,
        	                         std::list<CollisionInfo>& results);

            std::list<PotentialCollidingSets> mSetList;
            /*!
             * \brief NarrowPhaseJob elements to be processed when run is called
             *
             */
             List<BoundingVolumeCollision> mBoundingVolumeCollisions;

            /*!
             * \brief TrianglePair elements to be processed when run is called
             *
             */
             List<TrianglePair> mTrianglePairs;
            
            /*!
             * \brief list to store the results
             *
             */
            std::list<CollisionInfo> mNarrowPhaseCollisions;

            NarrowPhaseShapeStrategies mNarrowPhaseShapeStrategies;
            
    };
}

#endif // DCOLLIDE_NARROWPHASEJOB_H
/*
 * vim: et sw=4 ts=4
 */
