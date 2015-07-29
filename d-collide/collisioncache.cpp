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

#include "collisioncache.h"

#include "worldcollisions.h"
#include "proxy.h"
#include "bvhnode.h"
#include "pipeline.h"
#include "shapes/shape.h"
#include "debugstream.h"

namespace dcollide {
    void CollisionCacheElement::clear() {
        mBoundingVolumeCollisions.clear();
        mNarrowPhaseCollisions.clear();
        mTrianglePairCollisions.clear();
    }


    CollisionCache::CollisionCache(Pipeline* pipeline) {
        mPipeline = pipeline;
    }

    CollisionCache::~CollisionCache() {
        clear();
    }

    /*!
     * Apply the cache to the \ref Pipeline, i.e. make sure that the \ref
     * WorldCollisions object of the current collision run will contain the
     * information stored in the cache for the (toplevel-)proxy pair (p1,p2).
     *
     * Note that \p p1 and \p p2 must be toplevel proxies, see \ref
     * Proxy::getToplevelProxy
     *
     * This method uses \ref Pipeline::processCollisionCacheElement to perform
     * its actual task.
     */
    bool CollisionCache::applyCacheIfAvailable(Proxy* p1, Proxy* p2) {
        CollisionCacheElement* element = retrieveElement(p1, p2);
        if (!element) {
            return false;
        }

        mPipeline->processCollisionCacheElement(element);

        return true;
    }

    /*!
     * Returns the element for the pair (p1, p2), either by retrieving an
     * existing element (see \ref retrieveElement) or by creating a new one, if
     * no element for that pair exists yet.
     *
     * \return Either the value of \ref retrieveElement or a new element for
     * (p1,p2), if \ref retrieveElement returns NULL.
     */
    CollisionCacheElement* CollisionCache::retrieveOrAddElement(Proxy* p1, Proxy* p2) {
        if (!p1) {
            throw NullPointerException("p1");
        }
        if (!p2) {
            throw NullPointerException("p2");
        }

        CollisionCacheElement* element = retrieveElement(p1, p2);
        if (element) {
            return element;
        }

        element = new CollisionCacheElement(p1, p2);

        element->mCachePosition1 = mMap.insert(std::make_pair(p1, element));

        //insert the second element only for non-selfcollisions
        if (p1 != p2) {
            element->mCachePosition2 = mMap.insert(std::make_pair(p2, element));
        } else {
            element->mCachePosition2 = element->mCachePosition1;
        }

        return element;
    }

    /*!
     * Add \p collisions to the cache.
     *
     * Every entry of the \p collisions list is added to the \ref
     * CollisionCacheElement corresponding to the (toplevel-)proxy pair that the
     * list item belongs to.
     *
     * It is recommended to have \p collisions sorted by collision pairs (i.e.
     * if possible consecutive list items should belong to the same toplevel
     * proxy pair). This speeds up the caching process, however it is not
     * required - if the list is not sorted, a map lookup for the correct \ref
     * CollisionCacheElement object is made.
     */
    void CollisionCache::addToCacheElement(const std::list<BoundingVolumeCollision>& collisions) {
        Proxy* proxy1 = 0;
        Proxy* proxy2 = 0;
        CollisionCacheElement* element = 0;
        for (std::list<BoundingVolumeCollision>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
            // TODO: store the toplevel proxy in BoundingVolumeCollision
            Proxy* p1 = (*it).node1->getShape()->getProxy()->getToplevelProxy();
            Proxy* p2 = (*it).node2->getShape()->getProxy()->getToplevelProxy();

            if (!p1) {
                throw NullPointerException("p1");
            }

            // AB: usually we have multiple consecutive entries in the list with the same
            // proxies. we can simply re-use the pointer, without another map
            // lookup
            if (p1 != proxy1 || p2 != proxy2) {
                proxy1 = p1;
                proxy2 = p2;
                element = retrieveOrAddElement(proxy1, proxy2);
            }
            element->mBoundingVolumeCollisions.push_back(*it);
        }
    }

    /*!
     * \overload
     */
    void CollisionCache::addToCacheElement(const std::list<CollisionInfo>& collisions) {
        Proxy* proxy1 = 0;
        Proxy* proxy2 = 0;
        CollisionCacheElement* element = 0;
        for (std::list<CollisionInfo>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
            // TODO: maybe store the toplevel proxy in CollisionInfo?
            // -> it may be a lot faster to call getToplevelProxy() once for
            //    many CollisionInfo objects, instead of doing it here for every
            //    object
            Proxy* p1 = (*it).penetratingProxy->getToplevelProxy();
            Proxy* p2 = (*it).penetratedProxy->getToplevelProxy();

            // AB: usually we have multiple consecutive entries in the list with the same
            // proxies. we can simply re-use the pointer, without another map
            // lookup
            if (p1 != proxy1 || p2 != proxy2) {
                proxy1 = p1;
                proxy2 = p2;
                element = retrieveOrAddElement(proxy1, proxy2);
            }
            element->mNarrowPhaseCollisions.push_back(*it);
        }
    }

    /*!
     * \overload
     */
    void CollisionCache::addToCacheElement(const std::list<TrianglePair>& collisions) {
        Proxy* proxy1 = 0;
        Proxy* proxy2 = 0;
        CollisionCacheElement* element = 0;
        for (std::list<TrianglePair>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
            Proxy* p1 = (*it).toplevelProxy1;
            Proxy* p2 = (*it).toplevelProxy2;

            // AB: usually we have multiple consecutive entries in the list with the same
            // proxies. we can simply re-use the pointer, without another map
            // lookup
            if (p1 != proxy1 || p2 != proxy2) {
                proxy1 = p1;
                proxy2 = p2;
                element = retrieveOrAddElement(proxy1, proxy2);
            }
            element->mTrianglePairCollisions.push_back(*it);
        }
    }

    /*!
     * Invalidate all \ref CollisionCacheElement objects that \p proxy_ is part
     * of, i.e. make sure that no information regarding \p proxy_ is cached
     * anymore.
     *
     * This should be called whenever \p proxy_ moves or otherwise changes in a
     * way that may change the collisions with other proxies.
     */
    void CollisionCache::invalidate(Proxy* proxy_) {
        Proxy* proxy = proxy_->getToplevelProxy();

        MyMultiMap::iterator iter;

        iter = mMap.find(proxy);
        while (iter != mMap.end()) {
            CollisionCacheElement* element = (*iter).second;

            mMap.erase(element->mCachePosition1);
            if (element->mProxy1 != element->mProxy2) {
                mMap.erase(element->mCachePosition2);
            }

            delete element;

            iter = mMap.find(proxy);
        }//end while
    }

    /*!
     * \return The \ref CollisionCacheElement containing the cache information
     * for the (toplevel-)proxy pair (proxy1,proxy2). Note that \p proxy1 and \p
     * proxy2 must be toplevel proxies, see \ref Proxy::getToplevelProxy
     */
    CollisionCacheElement* CollisionCache::retrieveElement(Proxy* proxy1, Proxy* proxy2) const {
        //find and return the element where both proxies act as a key
        MyMultiMap::const_iterator iter;
        MyMultiMap::const_iterator lastElement;

        iter = mMap.find(proxy1);
        if (iter == mMap.end()) {
            return 0; // no elements associated with key, so return immediately
        }

        // get an iterator to the element that is one past the last element associated with key
        lastElement = mMap.upper_bound(proxy1);

        for ( ; iter != lastElement; ++iter) {
            CollisionCacheElement* element = (*iter).second;

            // sanity check
            if (element->mProxy1 != proxy1 && element->mProxy2 != proxy1) {
                error() << "oops: proxy1 not in element";
                continue;
            }

            //see if this cache-element holds the second proxy as well
            if (proxy1 != proxy2) {
                if (element->mProxy1 == proxy2 || element->mProxy2 == proxy2) {
                    return element;
                }
            } else {
                if (element->mProxy1 == element->mProxy2) {
                    return element;
                }
            }
        }
//        debug() << "retrieveElement(): found the invalidated proxy, but not the needed pair.";

        return 0;
    }

    /*!
     * Clear the cache.
     */
    void CollisionCache::clear() {
        while (!mMap.empty()) {
            MyMultiMap::iterator begin = mMap.begin();
            CollisionCacheElement* element = (*begin).second;
            mMap.erase(element->mCachePosition1);
            if (element->mProxy1 != element->mProxy2) {
                mMap.erase(element->mCachePosition2);
            }
            delete element;
        }
    }

}
/*
 * vim: et sw=4 ts=4
 */
