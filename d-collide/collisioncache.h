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


#ifndef DCOLLIDE_COLLISIONCACHE_H
#define DCOLLIDE_COLLISIONCACHE_H

#include "narrowphase/narrowphase.h"
#include "collisioninfo.h"
#include "detectordeform/trianglepair.h"

#include <map>
#include <list>

namespace dcollide {
    struct CollisionInfo;
    class Proxy;
    struct CollisionCacheElement;
    struct TrianglePair;
    class Pipeline;

    class CollisionCache {
        public:
            typedef std::multimap<Proxy*, CollisionCacheElement*> MyMultiMap;

        public:
            CollisionCache(Pipeline* pipeline);
            ~CollisionCache();

            void invalidate(Proxy* proxy);
            bool applyCacheIfAvailable(Proxy* p1, Proxy* p2);

            void addToCacheElement(const std::list<BoundingVolumeCollision>& boundingVolumeCollisions);
            void addToCacheElement(const std::list<TrianglePair>& collisions);
            void addToCacheElement(const std::list<CollisionInfo>& narrowPhaseCollisions);

            void clear();

        protected:
            CollisionCacheElement* retrieveOrAddElement(Proxy* p1, Proxy* p2);
            CollisionCacheElement* retrieveElement(Proxy* proxy1, Proxy* proxy2) const;

        private:
            Pipeline* mPipeline;
            MyMultiMap mMap;
    };

    /*!
     * \brief Internal helper struct for \ref CollisionCache
     *
     * This class stores the data associated with a collision pai. See \ref
     * CollisionCache::addToCacheElement.
     */
    struct CollisionCacheElement {
        inline CollisionCacheElement();
        inline CollisionCacheElement(Proxy* p1, Proxy* p2);

        void clear();

        Proxy* mProxy1;
        Proxy* mProxy2;

        // TODO: dcollide::List
        //   --> along with an allocation pool for CollisionCacheElement objects
        std::list<BoundingVolumeCollision> mBoundingVolumeCollisions;
        std::list<CollisionInfo> mNarrowPhaseCollisions;
        std::list<TrianglePair> mTrianglePairCollisions;

        // AB: std::multimap iterators remain valid on all insert and erase
        // operations (except of course on erasing the elements the iterators
        // point to).
        // -> we can use that toe speed invalidate() up significantly
        CollisionCache::MyMultiMap::iterator mCachePosition1; // for Proxy1
        CollisionCache::MyMultiMap::iterator mCachePosition2; // for Proxy2
    };

    inline CollisionCacheElement::CollisionCacheElement() :
            mProxy1(0), mProxy2(0) {
    }
    inline CollisionCacheElement::CollisionCacheElement(Proxy* p1, Proxy* p2) :
            mProxy1(p1), mProxy2(p2) {
    }
}

#endif // DCOLLIDE_COLLISIONCACHE_H
/*
 * vim: et sw=4 ts=4
 */
