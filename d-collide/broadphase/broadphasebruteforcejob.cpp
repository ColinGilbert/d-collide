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


#include "broadphasebruteforcejob.h"

#include "broadphasebruteforce.h"
#include "broadphasecollisions.h"
#include "thread/threadpool.h"
#include "boundingvolumes/boundingvolume.h"
#include "bvhnode.h"
#include "world.h"
#include "dcollide-config.h"
#include "proxy.h"
#include "debug.h"

namespace dcollide {

    /*!
     * \brief c'tor for broadphasejob: performs update
     */
    BroadPhaseBruteForceJob::BroadPhaseBruteForceJob(unsigned int intendedJobPoolIndex,
            BroadPhaseBruteForce* bp, Proxy* proxy)
            : BroadPhaseJob(intendedJobPoolIndex, bp) {
        mBP = bp;
        mProxy= proxy;
    }

    /*!
     * \brief d'tor for broadphasejob
     */
    BroadPhaseBruteForceJob::~BroadPhaseBruteForceJob() {
    }

    /*!
     * \brief this method is called by threadpool!
     */
    void BroadPhaseBruteForceJob::run() {
        // Should we check all Proxies or only one?
        if (!mProxy) {
            resolveChecklist(mBroadPhaseCollisions);
        } else {
            resolveChecklistProxyOnly(mBroadPhaseCollisions,mProxy);
        }
    }

    /*!
     * \brief Checks for Collisions via brute force method O=n^2
     * this method is thread safe
     * \param This pointer is the container in which the possible collisions are
     *        saved
     * \author Maximilian Hegele <maximilian.hegele@cs.uni-dortmund.de>
     */
    void BroadPhaseBruteForceJob::resolveChecklist(BroadPhaseCollisions*
        BPCollisions) {

        // Get Pointer to allTopLevelProxies
        const std::list<Proxy*>& allTopLevelProxies =
            mBP->getWorld()->getTopLevelProxies();


        // Only If more than 1 Proxy in the world:
        if (allTopLevelProxies.size() >= 2) {
            #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "\tChecking for possible Collisions"
                << std::endl;
            #endif

            //  Go through all Proxies:
            for (std::list<Proxy*>::const_iterator proxy1 =
                allTopLevelProxies.begin();
                proxy1 != allTopLevelProxies.end();
                ++proxy1) {

                // Check if proxy should be ignored:
                if ((*proxy1)->getProxyType() & PROXYTYPE_IGNORE) {
                    continue;
                }

                // Get BV of proxy1:
                const BoundingVolume* bv1 = (*proxy1)->getBvHierarchyNode()->getBoundingVolume();

                // Here we use the iterator proxy1, we want to start in
                // proxy2 not earlier then proxy1 is at the moment
                std::list<Proxy*>::const_iterator proxy2 = proxy1;
                proxy2++;
                for (;proxy2 != allTopLevelProxies.end();proxy2++) {

                    // Checking for NullPointer:
                    if (!(*proxy2)) {
                        std::cerr << dc_funcinfo << "WARNING: Proxy is Null-Pointer!" << std::endl;
                        break;
                    }
                    if (!(*proxy2)->getBvHierarchyNode()) {
                        std::cerr << dc_funcinfo << "WARNING: No BVHierarchyNode!" << std::endl;
                        break;
                    }

                    // Check if proxy should be ignored:
                    if ((*proxy1)->getProxyType() & PROXYTYPE_IGNORE) {
                        continue;
                    }

                    // don't calculate collisions for two fixed proxies
                    if ((*proxy1)->getProxyType() & PROXYTYPE_FIXED &&
                            (*proxy2)->getProxyType() & PROXYTYPE_FIXED) {
                        continue;
                    }

                    // Get BV of proxy2:
                    const BoundingVolume* bv2 = (*proxy2)->getBvHierarchyNode()->getBoundingVolume();

                    // if bv's are intersecting, put them in container:
                    if (bv1->collidesWith(*bv2)) {

                        #ifdef DCOLLIDE_BROADPHASE_DEBUG
                        std::cout << std::endl
                            << bv1->getSurroundingAabbMin() << "-"
                            << bv1->getSurroundingAabbMax()
                            << " collides with "
                            << bv2->getSurroundingAabbMin() << "-"
                            << bv2->getSurroundingAabbMax()
                            << std::endl;
                        #endif

                        bool hasDeformableProxy = false;
                        if ((*proxy1)->getProxyType() & PROXYTYPE_DEFORMABLE ||
                                (*proxy2)->getProxyType() & PROXYTYPE_DEFORMABLE) {
                            hasDeformableProxy = true;
                        }
                        BPCollisions->addCollisionPair(
                                bv1, bv2, hasDeformableProxy, &mResults);
                    } // END if (bv1->collidesWith(*bv2))

                } // END for (; proxy2 != allMembers->end();++proxy2

            } // END for proxy1

        } // END if (allTopLevelProxies.size()->size() >= 2)

    }

    /*!
     * \brief Checks for Collisions for one proxy via brute force method O=n
     * this method is thread safe
     * \param BPCollisions This pointer is the container in which the possible
     *  collisions are saved
     * \param proxy the proxy to check
     * \author Maximilian Hegele <maximilian.hegele@cs.uni-dortmund.de>
     */
    void BroadPhaseBruteForceJob::resolveChecklistProxyOnly(
            BroadPhaseCollisions* BPCollisions,Proxy* proxy) {

        // Checking for NullPointer:
        if (!proxy) {
            std::cerr << dc_funcinfo << "WARNING: Proxy is Null-Pointer!" << std::endl;
            return;
        }
        // Check if proxy should be ignored:
        if (proxy->getProxyType() & PROXYTYPE_IGNORE) {
            std::cerr << dc_funcinfo << "WARNING: Proxy is marked as IGNORE!" << std::endl;
            return;
        }

        // Get Pointer to allTopLevelProxies
        const std::list<Proxy*>& allTopLevelProxies =
            mBP->getWorld()->getTopLevelProxies();

        // Get BV of the one proxy:
        const BoundingVolume* bv = proxy->getBvHierarchyNode()->getBoundingVolume();

        //  Go through all other Proxies:
        for (std::list<Proxy*>::const_iterator proxy1 =
            allTopLevelProxies.begin();
            proxy1 != allTopLevelProxies.end();
            ++proxy1) {

            // skip proxy if it is the same as the other one:
            // Check if proxy should be ignored:
            if ((*proxy1) == proxy) {
                continue;
            }

            // Get BV of proxy1:
            const BoundingVolume* bv1 = (*proxy1)->getBvHierarchyNode()->getBoundingVolume();

            // Checking for NullPointer:
            if (!(*proxy1)) {
                std::cerr << dc_funcinfo << "WARNING: Proxy is Null-Pointer!" << std::endl;
                continue;
            }
            if (!(*proxy1)->getBvHierarchyNode()) {
                std::cerr << dc_funcinfo << "WARNING: No BVHierarchyNode!" << std::endl;
                continue;
            }

            // Check if proxy should be ignored:
            if ((*proxy1)->getProxyType() & PROXYTYPE_IGNORE) {
                continue;
            }

            // don't calculate collisions for two fixed proxies
            if (proxy->getProxyType() & PROXYTYPE_FIXED &&
                    (*proxy1)->getProxyType() & PROXYTYPE_FIXED) {
                continue;
            }

            // if bv's are intersecting, put them in container:
            if (bv1->collidesWith(*bv)) {

                #ifdef DCOLLIDE_BROADPHASE_DEBUG
                std::cout << std::endl
                    << bv1->getSurroundingAabbMin() << "-"
                    << bv1->getSurroundingAabbMax()
                    << " collides with "
                    << bv->getSurroundingAabbMin() << "-"
                    << bv->getSurroundingAabbMax()
                    << std::endl;
                #endif

                bool hasDeformableProxy = false;
                if (proxy->getProxyType() & PROXYTYPE_DEFORMABLE ||
                        (*proxy1)->getProxyType() & PROXYTYPE_DEFORMABLE) {
                    hasDeformableProxy = true;
                }
                BPCollisions->addCollisionPair(
                        bv1, bv, hasDeformableProxy, &mResults);
            } // END if (bv1->collidesWith(*bv))

        } // END for proxy1
    }
}
/*
 * vim: et sw=4 ts=4
 */
