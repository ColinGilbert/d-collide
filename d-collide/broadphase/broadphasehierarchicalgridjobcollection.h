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


#ifndef DCOLLIDE_BROADPHASEHIERARCHICALGRIDJOBCOLLECTION_H
#define DCOLLIDE_BROADPHASEHIERARCHICALGRIDJOBCOLLECTION_H

#include "../pipeline.h"
#include "datatypes/list.h"
#include <vector>

namespace dcollide {
    class BroadPhaseHierarchicalGrid;
    class HierarchicalGrid;
    class BroadPhaseCollisions;
    class BroadPhaseHierarchicalGridJob2;

    /*!
     * \brief Collection of all broadphase collision detection jobs
     *
     * This class groups all broadphase jobs that do collision detection, i.e.
     * once all jobs in this collection are completed, the broadphase is
     * completed. Note however, that the \ref BroadPhase class (or this class)
     * may create additional (e.g. postprocessing) jobs for the broadphase.
     * These jobs do not however influence the results of the broadphase in any
     * way and are merely "maintenance" jobs (see e.g. \ref
     * BroadPhase::addUpdateJob)
     *
     * You can use \ref addStartOnceCompleted here to add jobs that depend on the
     * broadphase being completed.
     */
    class BroadPhaseHierarchicalGridJobCollection :
            public PipelineThreadJobCollection {
        public:
            explicit BroadPhaseHierarchicalGridJobCollection(
                    Pipeline* pipeline);
            virtual ~BroadPhaseHierarchicalGridJobCollection();

            void setBroadPhase(BroadPhaseHierarchicalGrid* b);
            void addBroadPhaseJobs(const std::vector<HierarchicalGrid*>& startNodes, unsigned int jobPoolIndex);
            void addBroadPhaseJobs2(unsigned int jobPoolIndex);
            void addBroadPhaseJobsProxyOnly(unsigned int jobPoolIndex, Proxy*
                    proxy);

        protected:
            virtual void allJobsCompleted();

        private:
            BroadPhaseHierarchicalGrid* mBroadPhase;

            bool mBroadPhaseJob2Added;
            List<BroadPhaseHierarchicalGridJob2*>
                mHierarchicalGridJob2AllocationPool;
    };
}



#endif
/*
 * vim: et sw=4 ts=4
 */
