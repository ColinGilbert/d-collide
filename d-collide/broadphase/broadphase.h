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

#ifndef DCOLLIDE_BROADPHASE_H
#define DCOLLIDE_BROADPHASE_H

#include "thread/thread.h"
#include "real.h"
#include "dcollide-global.h"

namespace dcollide {

    class World;
    class BroadPhaseCollisions;
    struct VolumeOfInterest;
    class HierarchicalGrid;
    class BoundingVolume;
    class Proxy;
    class CollisionPair;
    class Pipeline;
    class PipelineThreadJobCollection;
    class ThreadPool;

     /*!
     * \brief representation of broadphase
     *
     * \author Maximilian Hegele <maximilian.hegele@cs.uni-dortmund.de>
     */
    class BroadPhase {
        public:
            virtual ~BroadPhase();

            virtual void init();
            virtual void notifyWorldSizeChanged();
            virtual void notifyProxyChanged(Proxy* toplevelProxy);
            virtual void addProxy(Proxy* proxy);
            virtual void removeProxy(Proxy* proxy);
            virtual void createBroadPhaseJobs(Proxy* proxy = 0) = 0;

            inline bool getInitCalled() const;
//TODO remove!!!!!!!!!!*******!
            /*static VolumeOfInterest calculateVolumeOfInterest(
                    const BoundingVolume *bv1,
                    const BoundingVolume *bv2);
*/
            inline World* getWorld() const;
            Pipeline* getPipeline() const;
            BroadPhaseCollisions* getBroadPhaseCollisions() const;
            inline unsigned int getJobPoolIndex() const;

            virtual PipelineThreadJobCollection* getJobCollection();

            inline BroadPhaseType getBroadPhaseType() const;

        protected:
            // Use BroadphaseFactory to intanciate this class
            BroadPhase(World* world, BroadPhaseType type);

        private:
            /*!
            * \brief Pointer to the great wide World
            */
            World* mWorld;

            /*!
            * \brief true after init() was called:
            */
            bool mInitCalled;

            /*!
             * \brief The ThreadPool where we put the the jobs into
             * OWNERSHIP NOTICE: world is owner!
             */
            ThreadPool* mWorkerPool;

            /*!
             * \brief The job pool index (see \ref ThreadPool::addJobPool) for
             * broadphase jobs
             */
            unsigned int mJobPoolIndex;

            /*!
            * \brief saves BP-Collisions
            */
            BroadPhaseCollisions* mBPCollisions;

            // it is assumed that the pointer (returned by \ref getJobCollection
            // only) remains valid for the lifetime of the broadphase!
            PipelineThreadJobCollection* mJobCollection;

            /*!
            * \brief The type of the broadphase
            */
            BroadPhaseType mType;
    };

    inline bool BroadPhase::getInitCalled() const {
        return mInitCalled;
    }

    inline World* BroadPhase::getWorld() const {
        return mWorld;
    }

    inline unsigned int BroadPhase::getJobPoolIndex() const {
        return mJobPoolIndex;
    }

    inline BroadPhaseType BroadPhase::getBroadPhaseType() const {
        return mType;
    }
}

#endif // DCOLLIDE_BROADPHASE_H
/*
 * vim: et sw=4 ts=4
 */
