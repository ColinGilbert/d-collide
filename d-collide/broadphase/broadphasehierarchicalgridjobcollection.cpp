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


#include "broadphasehierarchicalgridjobcollection.h"
#include "debug.h"
#include "../exceptions/exception.h"
#include "broadphasehierarchicalgridjob.h"
#include "broadphasehierarchicalgridjob2.h"
#include "broadphasehierarchicalgrid.h"
#include "broadphasecollisions.h"
#include "pipeline.h"
#include "world.h"
#include "debugstream.h"

namespace dcollide {
    BroadPhaseHierarchicalGridJobCollection::
            BroadPhaseHierarchicalGridJobCollection(Pipeline* pipeline)
            : PipelineThreadJobCollection(pipeline, Pipeline::PHASE_BROADPHASE)
    {
        mBroadPhase = 0;

        // AB: whether the jobs in this collection are
        // BroadPhaseHierarchicalGridJob or BroadPhaseHierarchicalGridJob2
        mBroadPhaseJob2Added = false;
    }

    BroadPhaseHierarchicalGridJobCollection::
            ~BroadPhaseHierarchicalGridJobCollection() {
    }

    void BroadPhaseHierarchicalGridJobCollection::
            setBroadPhase(BroadPhaseHierarchicalGrid* b) {
        mBroadPhase = b;
    }

    /*!
     * Create and add (see \ref addJob) the \ref BroadPhaseJob objects used for
     * collision detection.
     *
     * Note that the jobs are not yet started (i.e. not yet added to the \ref
     * ThreadPool). You need to call \ref start to do that.
     */
    void BroadPhaseHierarchicalGridJobCollection::addBroadPhaseJobs(const
            std::vector<HierarchicalGrid*>& startNodes,
            unsigned int jobPoolIndex) {
        if (getJobsInCollectionCount() > 0) {
            throw Exception("BroadPhase jobs already added to collection");
        }
        // should be a noop, because allJobsCompleted() already does so.
        // just in case.
        resetCollection();

        for (std::vector<HierarchicalGrid*>::const_iterator iter =
            startNodes.begin(); iter != startNodes.end(); ++iter) {

            // Add Job for each child of root-Grid:
            BroadPhaseHierarchicalGridJob* job = new
                BroadPhaseHierarchicalGridJob(jobPoolIndex, *iter, mBroadPhase);
            addJob(job);
        }

        mBroadPhaseJob2Added = false;
        setAllJobsAreAdded();
    }

    /*!
     * \brief adds only one job which checks for one proxy only
     * \param jobPoolIndex
     * \param proxy The Proxy for which the check should be performed
     */
    void BroadPhaseHierarchicalGridJobCollection::
            addBroadPhaseJobsProxyOnly(unsigned int jobPoolIndex,
            Proxy* proxy) {
        // FIXME: Do we need this if we check only for one proxy?
        if (getJobsInCollectionCount() > 0) {
            throw Exception("BroadPhase jobs already added to collection");
        }
        // should be a noop, because allJobsCompleted() already does so.
        // just in case.
        resetCollection();

        BroadPhaseHierarchicalGridJob2* job;
        if (mHierarchicalGridJob2AllocationPool.empty()) {
            job = new BroadPhaseHierarchicalGridJob2(jobPoolIndex, mBroadPhase);
            job->setDeleteAfterCompletionHint(false);
        } else {
            job = mHierarchicalGridJob2AllocationPool.front();
            mHierarchicalGridJob2AllocationPool.pop_front();
        }
        job->clear();
        job->addProxy(proxy);
        job->setSingleProxyOnly(true);
        addJob(job);
        mBroadPhaseJob2Added = true;
        setAllJobsAreAdded();
    }

    /*!
     * \brief checks all proxies for collisions
     * \param jobPoolIndex
     */
    void BroadPhaseHierarchicalGridJobCollection::addBroadPhaseJobs2(
            unsigned int jobPoolIndex) {
        if (getJobsInCollectionCount() > 0) {
            throw Exception("BroadPhase jobs already added to collection");
        }
        // should be a noop, because allJobsCompleted() already does so.
        // just in case.
        resetCollection();

        int proxyCount = mBroadPhase->getWorld()->getNumberOfTopLevelProxies();
        if (proxyCount == 0) {
            setAllJobsAreAdded();
            return;
        }

        int totalJobCount = 8; // TODO: make dependent on thread count!

        if (totalJobCount * 2 > proxyCount) {
            totalJobCount = proxyCount / 2;
        }

        totalJobCount = std::max(totalJobCount, 1);

        const std::list<Proxy*>& proxies = mBroadPhase->getWorld()->getTopLevelProxies();
        std::list<Proxy*>::const_iterator iterator = proxies.begin();
        for (int jobNumber = 0; jobNumber < totalJobCount; jobNumber++) {
            BroadPhaseHierarchicalGridJob2* job;
            if (mHierarchicalGridJob2AllocationPool.empty()) {
                job = new BroadPhaseHierarchicalGridJob2(jobPoolIndex,
                    mBroadPhase);
                job->setDeleteAfterCompletionHint(false);
            } else {
                job = mHierarchicalGridJob2AllocationPool.front();
                mHierarchicalGridJob2AllocationPool.pop_front();
            }
            job->clear();
            int proxiesForJob = proxyCount / totalJobCount;
            job->addProxies(proxies, iterator, proxiesForJob);
            if (jobNumber == totalJobCount - 1 && iterator != proxies.end()) {
                // add all remaining proxies to this job.
                job->addProxies(proxies, iterator, proxyCount);
            }
            addJob(job);
        }

        mBroadPhaseJob2Added = true;
        setAllJobsAreAdded();
    }

    /*!
     * See \ref ThreadJobCollection::allJobsCompleted.
     *
     * This implementation additionally adds maintenance jobs that may modify
     * the \ref BroadPhase datastructures (but do not influence the broadphase
     * results anymore)
     */
    void BroadPhaseHierarchicalGridJobCollection::allJobsCompleted() {
        // AB: combine the results from all jobs into the broadphase.
        //     do this BEFORE calling
        //     PipelineThreadJobCollection::allJobsCompleted(), as this function
        //     may need the results already.
        if (mBroadPhaseJob2Added) {
            BroadPhaseCollisions* collisions = mBroadPhase->getBroadPhaseCollisions();
            if (!collisions->getResults().empty()) {
                throw Exception("BroadPhase results should be empty at this point");
            }
            for (ListNode<ThreadJob*>* jobNode = getJobs().getFirstNode(); jobNode; jobNode = jobNode->getNext()) {
                BroadPhaseHierarchicalGridJob2* job =
                        static_cast<BroadPhaseHierarchicalGridJob2*>
                        (jobNode->getData());
                const List<CollisionPair*>& jobResults = job->getJobResultsReference();
                for (ListNode<CollisionPair*>* resultNode = jobResults.getFirstNode(); resultNode; resultNode = resultNode->getNext()) {
                    CollisionPair* p = resultNode->getData();
                    collisions->addCollisionPairNoCheckDuplicatesNotThreadSafe(*p);
                }
            }
        }

        PipelineThreadJobCollection::allJobsCompleted();

        if (mBroadPhaseJob2Added) {
            for (ListNode<ThreadJob*>* jobNode = getJobs().getFirstNode(); jobNode; jobNode = jobNode->getNext()) {
                BroadPhaseHierarchicalGridJob2* job =
                        static_cast<BroadPhaseHierarchicalGridJob2*>
                        (jobNode->getData());
                mHierarchicalGridJob2AllocationPool.push_back(job);
            }
        }

        resetCollection();

        mBroadPhase->addUpdateJob();

        // note: we cannot delete the jobs here, because we have not yet read
        // the results!
        // -> only once all middlephase jobs have been created can we delete the
        //    broadphase jobs.
    }

}

/*
 * vim: et sw=4 ts=4
 */
