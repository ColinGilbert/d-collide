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

#include "broadphasehierarchicalgridjob2.h"

#include "broadphasehierarchicalgrid.h"
#include "broadphasehierarchicalgridproxydata.h"
#include "hierarchicalgrid.h"
#include "broadphasecollisions.h"
#include "bvhnode.h"
#include "boundingvolumes/boundingvolume.h"
#include "proxy.h"
#include "dcollide-config.h"
#include "debug.h"
#include "debugstream.h"

namespace dcollide {

    BroadPhaseHierarchicalGridJob2::BroadPhaseHierarchicalGridJob2(
            unsigned int intendedJobPoolIndex, BroadPhaseHierarchicalGrid* bp)
            : BroadPhaseJob(intendedJobPoolIndex, bp) {
        mSingleProxyOnly = false;

    }

    BroadPhaseHierarchicalGridJob2::~BroadPhaseHierarchicalGridJob2() {
    }

    void BroadPhaseHierarchicalGridJob2::clear() {
        mProxies.clear();
        while (!getJobResultsReference().empty()) {
            delete getJobResultsReference().front();
            getJobResultsReference().pop_front();
        }
    }

    /*!
     * Add \p maxCount proxies from the \p proxies list to this job, starting at
     * \p it. The iterator \p is modified and will point to the proxy after the
     * last one added to this job (i.e. can be used for multiple calls to this
     * method).
     *
     * These proxies are checked for collisions with other proxies when \ref run
     * is called.
     */
    void BroadPhaseHierarchicalGridJob2::addProxies(
            const std::list<Proxy*>& proxies,
            std::list<Proxy*>::const_iterator& it, int maxCount) {
        int count = 0;
        while (it != proxies.end() && count < maxCount) {
            mProxies.push_back(*it);
            count++;
            ++it;
        }
    }

    /*!
     * Start this ThreadJob. Called automatically by the \ref ThreadPool, do not
     * call this directly. See also \ref start and \ref
     * PipelineThreadJobCollection::addJob
     */
    void BroadPhaseHierarchicalGridJob2::run() {
        for (ListNode<Proxy*>* node = mProxies.getFirstNode(); node; node = node->getNext()) {
            checkProxyForCollisions(node->getData());
        }
        setSingleProxyOnly(false);
    }

    /*!
     * Check \p proxy for collisions with other proxies. Duplicated collisions
     * are removed by this method.
     */
    void BroadPhaseHierarchicalGridJob2::checkProxyForCollisions(Proxy* proxy) {
        if (!proxy) {
            return;
        }

        mSet.clear();

        const BoundingVolume* bv1 = proxy->getBvHierarchyNode()->getBoundingVolume();
        if (!bv1) {
            // AB: throwing an exception from inside a job is a bad idea... (it
            //     should be an exception, but the main program would not be
            //     able to catch it, so we just emit an error)
            error() << "NULL BoundingVolume for proxy " << proxy;
            return;
        }

        BroadPhaseHierarchicalGridProxyData* data =
                static_cast<BroadPhaseHierarchicalGridProxyData*>
                (proxy->getBroadPhaseProxyData());
        const List<HierarchicalGrid*>& grids = data->getOccupiedGrids();
        for (ListNode<HierarchicalGrid*>* gridNode = grids.getFirstNode(); gridNode; gridNode = gridNode->getNext()) {
            List<const Proxy*>* allMembers = gridNode->getData()->getGridMembers();
            for (ListNode<const Proxy*>* memberNode = allMembers->getFirstNode(); memberNode; memberNode = memberNode->getNext()) {
                const Proxy* proxy2 = memberNode->getData();

                // AB: note: if A and B collide with each other, then there's
                // one jobs that finds the collision (A,B) and another that
                // finds (B,A).
                // to avoid duplicates, we only look at collisions where the
                // pointer to A is < the pointer to B and ignore the rest.
                // -> if they collide, we either already found that collision,
                //    or will find it later.
                if ((!mSingleProxyOnly) && (proxy >= proxy2)) {
                    continue;
                }

                const BoundingVolume* bv2 = proxy2->getBvHierarchyNode()->getBoundingVolume();
                if (!bv2) {
                    // see above (-> bv1): should be exception, but we can emit
                    // an error only
                    error() << "NULL BoundingVolume for proxy2 " << proxy2;
                    continue;
                }

                // don't calculate collisions for two fixed proxies
                if (proxy->getProxyType() & PROXYTYPE_FIXED &&
                    proxy2->getProxyType() & PROXYTYPE_FIXED) {
                    continue;
                }

                if (!bv1->collidesWith(*bv2)) {
                    continue;
                }

                std::pair<std::set<const Proxy*>::iterator,bool> pair = mSet.insert(proxy2);
                bool inserted = pair.second;
                //bool inserted = mSet.insert(proxy2);
                if (!inserted) {
                    // duplicate.
                    continue;
                }

                bool hasDeformableProxy = false;
                if (proxy->getProxyType() & PROXYTYPE_DEFORMABLE || proxy2->getProxyType() & PROXYTYPE_DEFORMABLE) {
                    hasDeformableProxy = true;
                }
                mBroadPhaseCollisions->addCollisionPairToJobResultsOnly(bv1, bv2, hasDeformableProxy, &mResults);

            }
        }
    }

}
/*
 * vim: et sw=4 ts=4
 */
