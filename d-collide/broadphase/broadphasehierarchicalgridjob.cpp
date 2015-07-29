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


#include "broadphasehierarchicalgridjob.h"

#include "broadphasehierarchicalgrid.h"
#include "hierarchicalgrid.h"
#include "broadphasecollisions.h"
#include "bvhnode.h"
#include "boundingvolumes/boundingvolume.h"
#include "dcollide-config.h"
#include "proxy.h"
#include "debug.h"

namespace dcollide {

    /*!
     * \brief c'tor for broadphasejob: gets collisions
     */
    BroadPhaseHierarchicalGridJob::BroadPhaseHierarchicalGridJob(
            unsigned int intendedJobPoolIndex,
            HierarchicalGrid* grid,
            BroadPhaseHierarchicalGrid* bp)
            : BroadPhaseJob(intendedJobPoolIndex, bp) {
        doCollisionCheck = true;
        doUpdate = false;
        mGrid = grid;
    }

    /*!
     * \brief c'tor for broadphasejob: performs update
     */
    BroadPhaseHierarchicalGridJob::BroadPhaseHierarchicalGridJob(unsigned int
            intendedJobPoolIndex, BroadPhaseHierarchicalGrid* bp)
            : BroadPhaseJob(intendedJobPoolIndex, bp) {
        doCollisionCheck = false;
        doUpdate = true;
        mBP = bp;
    }

    /*!
     * \brief d'tor for broadphasejob
     */
    BroadPhaseHierarchicalGridJob::~BroadPhaseHierarchicalGridJob() {
    }

    /*!
     * \brief this method is called by threadpool!
     */
    void BroadPhaseHierarchicalGridJob::run() {
        if (doCollisionCheck && mGrid) {
            resolveChecklist(mBroadPhaseCollisions, mGrid);
        } else if (doUpdate && mBP) {
            static_cast<BroadPhaseHierarchicalGrid*>(mBP)->update();
        }
    }

    /*!
     * \brief go through the grid and collect all members
     * recursive
     * this method is thread safe
     * \param This pointer is the container in which the possible collisions are
     *        saved
     * \param The Grid
     * \author Maximilian Hegele <maximilian.hegele@cs.uni-dortmund.de>
     */
    void BroadPhaseHierarchicalGridJob::resolveChecklist(BroadPhaseCollisions*
        BPCollisions, HierarchicalGrid* grid) {

        // do we have Children in the grid?
        if (!grid->isLeaf()) {
            std::vector<HierarchicalGrid*> allChildren =
                grid->getChildren();

            // go through all children and call this method again:
            for (std::vector<HierarchicalGrid*>::iterator iter =
                        allChildren.begin();
                    iter != allChildren.end();
                    ++iter) {
                resolveChecklist(BPCollisions,*iter);
             }
        }

        // If no children, fill the container:
        else {

            #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "Found " << grid->getGridMembers()->size()
                << " Members in Grid " << grid->getGridMin() <<
                grid->getGridMax()
                <<", Depth: " << grid->getGridDepth()
                << std::endl;
            #endif

            // If more than 1 member:
            if (grid->getGridMembers()->size() >= 2) {
                #ifdef DCOLLIDE_BROADPHASE_DEBUG
                std::cout << "\tChecking for possible Collisions"
                    << std::endl;
                #endif
                List<const Proxy*>* allMembers = grid->getGridMembers();
                for (ListNode<const Proxy*>* proxy1 =
                        allMembers->getFirstNode();
                        proxy1 != 0; proxy1 = proxy1->getNext()) {

                    // Check if proxy should be ignored:
                    if (proxy1->getData()->getProxyType() & PROXYTYPE_IGNORE) {
                        continue;
                    }

                    // Get BV of proxy1:
                    const BoundingVolume* bv1 = proxy1->getData()->
                            getBvHierarchyNode()->getBoundingVolume();

                    // Here we use the iterator bv1, we want to start in
                    // bv2 not earlier then bv1 is at the moment
                    for (ListNode<const Proxy*>* proxy2 = proxy1->getNext();
                            proxy2 != 0; proxy2 = proxy2->getNext()) {

                        // Checking for NullPointer:
                        if (!(proxy2->getData())) {
                            std::cerr << dc_funcinfo << "WARNING: Proxy is Null-Pointer!" << std::endl;
                            break;
                        }
                        if (!(proxy2->getData()->getBvHierarchyNode())) {
                            std::cerr << dc_funcinfo << "WARNING: No BVHierarchyNode!" << std::endl;
                            break;
                        }

                        // Check if proxy should be ignored:
                        if (proxy2->getData()->getProxyType() & PROXYTYPE_IGNORE) {
                            continue;
                        }

                        // don't calculate collisions for two fixed proxies
                        if (proxy1->getData()->getProxyType() & PROXYTYPE_FIXED
                                && proxy2->getData()->getProxyType() &
                                PROXYTYPE_FIXED) {
                            continue;
                        }

                        // Get BV of proxy2:
                        const BoundingVolume* bv2 = proxy2->getData()->
                                getBvHierarchyNode()->getBoundingVolume();

                        // if bv's are intersecting, put them in container:
                        if (bv1->collidesWith(*bv2)) {

                            // Locking Mutex:
//                            MutexLocker lock(&mMutex); // AB: only this job
//                            (i.e. only this thread) can lock this...

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
                            if (proxy1->getData()->getProxyType() & PROXYTYPE_DEFORMABLE ||
                                    proxy2->getData()->getProxyType() & PROXYTYPE_DEFORMABLE) {
                                hasDeformableProxy = true;
                            }
                            BPCollisions->addCollisionPair(bv1,bv2,
                                    hasDeformableProxy, &mResults);
                        } // END if (bv1->collidesWith(*bv2))

                    } // END for (; proxy2 != allMembers->end();++proxy2

                } // END for proxy1

            } // END if (grid->getGridMembers()->size() >= 2)

        } // END else
    }
}
/*
 * vim: et sw=4 ts=4
 */
