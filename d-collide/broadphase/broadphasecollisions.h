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


#ifndef DCOLLIDE_BROADPHASECOLLISIONS_H
#define DCOLLIDE_BROADPHASECOLLISIONS_H

#include "thread/thread.h"
#include "datatypes/set.h"

#include <set>
#include <iostream>
#include <list>


namespace dcollide {
    class CollisionPair;

    /*! 
     * \brief holds all collision pairs
     * CollisionPairs are saved as std::set
     * 
     */
    class BroadPhaseCollisions {
        private:
            
            /*! 
             * \brief container for unique collision pairs
             */
            Set<CollisionPair> mBroadPhaseCollisionsSet;

            std::list<CollisionPair> mResults;

            Mutex mMutex;

        public:

            // Constructor(s) and Destructor
             BroadPhaseCollisions();
             ~BroadPhaseCollisions();

            bool addCollisionPair(const BoundingVolume *bv1,
                    const BoundingVolume *bv2,
                    bool hasDeformableProxy,
                    List<CollisionPair*>* jobResults = NULL);

            bool addCollisionPairNoCheckDuplicatesNotThreadSafe(const CollisionPair& pair);

            bool addCollisionPairPreventDuplicates(const BoundingVolume *bv1,
                    const BoundingVolume* bv2,
                    bool hasDeformableProxy,
                    List<CollisionPair*>* jobResults = NULL);

            void addCollisionPairToJobResultsOnly(const BoundingVolume *bv1,
                    const BoundingVolume* bv2,
                    bool hasDeformableProxy,
                    List<CollisionPair*>* jobResults);

            void reset();

            inline const std::list<CollisionPair>& getResults() const;

        protected:
            static CollisionPair makePair(const BoundingVolume* bv1,
                    const BoundingVolume* bv2,
                    bool hasDeformableProxy);

    };

    const std::list<CollisionPair>& BroadPhaseCollisions::getResults() const {
        return mResults;
    }
}

/*
 * vim: et sw=4 ts=4
 */

#endif // DCOLLIDE_BROADPHASECOLLISIONS_H
