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

#include "broadphase/broadphasecollisions.h"

#include <list>


namespace dcollide {


    /*! 
     *  \brief constructor
     */
    BroadPhaseCollisions::BroadPhaseCollisions() {
    }

    /*! 
     *  \brief destructor
     */
    BroadPhaseCollisions::~BroadPhaseCollisions() {
    }


    /*!
     *  \brief resets all containers
     */
    void BroadPhaseCollisions::reset() {
        mBroadPhaseCollisionsSet.clear();
        mResults.clear();
        //mBroadPhaseCollisions.clear();
    }


    /*!
     * \brief add both bounding volumes and the volume of interest
     *
     * also fills \ref BroadPhaseJob results data structure \p jobResults.
     *
     * this method is thread safe
     *
     * \return TRUE if pair was added, FALSE if it was discarded (duplicate).
     */
    bool BroadPhaseCollisions::addCollisionPair(const BoundingVolume *bv1,
            const BoundingVolume *bv2,
            bool hasDeformableProxy,
            List<CollisionPair*>* jobResults) {
        return addCollisionPairPreventDuplicates(bv1, bv2, hasDeformableProxy, jobResults);
    }

    bool BroadPhaseCollisions::addCollisionPairPreventDuplicates(const BoundingVolume *bv1,
            const BoundingVolume *bv2,
            bool hasDeformableProxy,
            List<CollisionPair*>* jobResults) {

        CollisionPair colPair = makePair(bv1, bv2, hasDeformableProxy);

        MutexLocker lock(&mMutex);

        // Own Set:
        bool success = false;
        success = mBroadPhaseCollisionsSet.insert(colPair);

        if (success) {
            // If we inserted the value, add it to the jobResults list, too (if we have one)
            if (jobResults) {
                addCollisionPairToJobResultsOnly(bv1, bv2, hasDeformableProxy, jobResults);
            }
            mResults.push_back(colPair);
            #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "\t" << "NEU: INSERTED!" << std::endl;
            #endif
        } else {
            #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "\t" << "NEU: SKIPPED" << std::endl;
            #endif
        }

        return success;
    }

    bool BroadPhaseCollisions::addCollisionPairNoCheckDuplicatesNotThreadSafe(const CollisionPair& pair) {
        mResults.push_back(pair);
        return true;
    }

    void BroadPhaseCollisions::addCollisionPairToJobResultsOnly(const BoundingVolume *bv1,
            const BoundingVolume *bv2,
            bool hasDeformableProxy,
            List<CollisionPair*>* jobResults) {
        jobResults->push_back(new CollisionPair(makePair(bv1, bv2, hasDeformableProxy)));
    }

    CollisionPair BroadPhaseCollisions::makePair(const BoundingVolume* bv1,
            const BoundingVolume* bv2,
            bool hasDeformableProxy) {
        CollisionPair colPair;
        colPair.bvol1 = bv1;
        colPair.bvol2 = bv2;

        return colPair;
    }

}
/*
 * vim: et sw=4 ts=4
 */
