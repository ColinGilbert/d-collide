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

#ifndef DCOLLIDE_BROADPHASEHIERARCHICALGRID_H
#define DCOLLIDE_BROADPHASEHIERARCHICALGRID_H

#define PREFERRED_DEPTH 2
#define DEFAULT_PROBABILITY 0.5f

#include "broadphase.h"

#include "math/vector.h"
#include "datatypes/set.h"
#include "real.h"

namespace dcollide {

    class World;
    class BroadPhaseCollisions;
    struct VolumeOfInterest;
    class HierarchicalGrid;
    class BoundingVolume;
    class Proxy;
    class CollisionPair;
    class BroadPhaseHierarchicalGridJobCollection;

     /*!
     * \brief representation of broadphase
     *
     * \author Maximilian Hegele <maximilian.hegele@cs.uni-dortmund.de>
     */
    class BroadPhaseHierarchicalGrid : public BroadPhase {
        public:
            virtual ~BroadPhaseHierarchicalGrid();

            /*!
            * \brief Initializes the broadphase
            */
            virtual void init();
            virtual void notifyWorldSizeChanged();
            virtual void notifyProxyChanged(Proxy* toplevelProxy);

            void createGrid();

            /*!
            * \brief Calculates the maximal GridDepth heuristically
            */
            int calculateMaxGridDepth(real upperProbabilityBound =
                DEFAULT_PROBABILITY);

            /*!
            * \brief Calculates the maximal number of members in non leaf grids
            * heuristically
            */
            int calculateMaxGridMembers();

            /*!
            * \brief Updates the broadphase
            */
            void update();
            void addUpdateJob();
            void recreateGrid();

            void updateProxyAllocation(Proxy* proxy);
            void updateProxyAllocation2(Proxy* proxy);

            /*!
            * \brief add Aabb to grid
            */
            virtual void addProxy(Proxy* proxy);
            /*!
            * \brief remove Aabb from grid
            */
            virtual void removeProxy(Proxy* proxy);

            virtual void createBroadPhaseJobs(Proxy* proxy = 0);

            /*!
            * \brief Set the grid-depth
            * This sets the depth of the grid to something ^2
            * Maximum value allowed (to be discussed): 256
            */
            inline void setMaxGridDepth(int maxGridDepth);
            inline int getMaxGridDepth() const;
            inline int getMaxGridMembers() const;

            inline void setUpdateIntervall(int updateIntervall);

            inline void setUpperProbabilityBound(real upperBound);

            inline const HierarchicalGrid* getHierarchicalGrid() const;

            virtual PipelineThreadJobCollection* getJobCollection();

            bool checkTreeConsistency() const;
            void writeTreeToDebug();

        private:
            explicit BroadPhaseHierarchicalGrid(World* world);

            friend class BroadPhaseFactory;

        private:
            /*!
            * \brief Depth of the Grid
            * This is in class Broadphase, we only need this value once,
            * saving it in HierarchicalGrid would result in duplicates!
            */
            int mMaxGridDepth;

            /*!
            * \brief Upper bound for the number of grid members
            * This bound may be violated in grids of maximal depth
            */
            unsigned int mMaxGridMembers;

            /*!
            * \brief Average size of the Bounding Volumes
            */
            Vector3 mAverageBoundingVolumeSize;

            /*!
            * \brief UpperProbabilityBound for the algo calculateMaxGridSize
            */
            real mUpperProbabilityBound;

            /*!
            * \brief The Hierarchical Grid itself
            */
            HierarchicalGrid* mGrid;

            /*!
             * \brief reusable set of Pointers to Proxies for
             * calls to \ref HierarchicalGrid::getChildrenMembers
             */
            Set<Proxy*> mProxySet;

            BroadPhaseHierarchicalGridJobCollection* mJobCollection;

            /*!
            * \brief counter for calls of \ref createBroadPhaseJobs
            */
            int mGetCollisionsCalls;

            /*!
            * \brief Intervall of Update calls
            * if 0, no updates are performed!
            */
            int mUpdateIntervall;

        private:
            inline int calculateDepthFromSize(int worldDimension,
                    int smallestGridSize, int divisor) const;

            bool addProxyToGrid(Proxy* proxy);

            void traverseGridForUpdate(HierarchicalGrid* grid, int
                newMaxGridDepth, unsigned int newMaxGridMembers);

            void removeProxyForUpdate(Proxy* proxy);
            void addProxyForUpdate(Proxy* proxy, HierarchicalGrid* start);

            void increaseGrid(const Vector3& proxyPosition);
            void increaseWorldSize(const Vector3& proxyPosition);


    };

    void BroadPhaseHierarchicalGrid::setMaxGridDepth(int maxGridDepth) {
        mMaxGridDepth = maxGridDepth;
    }
    void BroadPhaseHierarchicalGrid::setUpdateIntervall(int updateIntervall) {
        mUpdateIntervall = updateIntervall;
    }
    int BroadPhaseHierarchicalGrid::getMaxGridDepth() const {
        return mMaxGridDepth;
    }
    int BroadPhaseHierarchicalGrid::getMaxGridMembers() const {
        return mMaxGridMembers;
    }
    void BroadPhaseHierarchicalGrid::setUpperProbabilityBound(real upperBound) {
        mUpperProbabilityBound = upperBound;
    }

    const HierarchicalGrid* BroadPhaseHierarchicalGrid::getHierarchicalGrid()
            const {
        return mGrid;
    }
}

#endif // DCOLLIDE_BROADPHASE_H
/*
 * vim: et sw=4 ts=4
 */
