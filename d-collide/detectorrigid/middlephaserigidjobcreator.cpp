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

#include "middlephaserigidjobcreator.h"

#include "detectorrigid/middlephaserigidjob.h"
#include "collisionpair.h"
#include "boundingvolumes/boundingvolume.h"
#include "worldcollisions.h"
#include "broadphase/broadphasejob.h"
#include "timing.h"
#include "debugstream.h"
#include "world.h"
#include "proxy.h"
#include "thread/threadpool.h"
#include "dcollide-config.h"
#include "pipeline.h"
#include "bvhnode.h"
#include "narrowphase/narrowphase.h"
#include "collisioncache.h"

#include <list>
#include <vector>

const unsigned int maxRigidItemCount = 10;

namespace dcollide {
    MiddlePhaseJobCreator::MiddlePhaseJobCreator(World* world, Pipeline* pipeline, unsigned int jobPoolIndexRigid) {
        mPipeline = pipeline;
        mJobPoolMiddlePhaseRigid = jobPoolIndexRigid;
        mRigidJob = 0;
        mWorld = world;
    }

    MiddlePhaseJobCreator::~MiddlePhaseJobCreator() {
        for (ListNode<MiddlePhaseRigidJob*>* node = mAllocatedRigidJobs.getFirstNode(); node; node = node->getNext()) {
            delete node->getData();
        }
    }

    void MiddlePhaseJobCreator::createSelfCollisionJobs(const std::list<Proxy*>& selfCollisionProxies) {
        mRigidItemCountInJob = 0;
        for (std::list<Proxy*>::const_iterator iter = selfCollisionProxies.begin();
                    iter != selfCollisionProxies.end(); ++iter) {
            Proxy* p = *iter;
            if (!(p->getProxyType() & PROXYTYPE_SELFCOLLIDABLE)) {
                continue;
            }


            if (p->getProxyType() & PROXYTYPE_RIGID) {
                if (mWorld->getUseCollisionCaching()) {
                    bool applied = mWorld->getCollisionCache()->applyCacheIfAvailable(p, p);
                    if (applied) {
                        continue;
                    }
                }
                if (!mRigidJob) {
                    mRigidJob = createMiddlePhaseRigidJob();
                }
                mRigidJob->addInput(p);
                mRigidItemCountInJob++;
                if (mRigidItemCountInJob >= maxRigidItemCount) {
                    finalizeRigidJob();
                }
            } else {
                // obsolete, don't use.
                // the Pipeline already creates deformable self-collision jobs.
            }
        }

        finalizeRigidJob();
    }

    void MiddlePhaseJobCreator::createJobs(BroadPhaseJob* broadPhaseJob) {
        mRigidItemCountInJob = 0;

        List<CollisionPair*>& jobResults = broadPhaseJob->getJobResultsReference();

        for (ListNode<CollisionPair*>* jobResultsNode = jobResults.getFirstNode(); jobResultsNode; jobResultsNode = jobResultsNode->getNext()) {
            CollisionPair* collisionPair = jobResultsNode->getData();

            if (!collisionPair->bvol1 || !collisionPair->bvol2 ) {
                throw NullPointerException("one of the BoundingVolumes from the collisionPair");
            }
            if (!collisionPair->bvol1->getHierarchyNode() || !collisionPair->bvol2->getHierarchyNode()) {
                throw NullPointerException("one of the BvhNodes from the collisionPair");
            }

            dcollide::Proxy* p1 = 0;
            dcollide::Proxy* p2 = 0;
            p1 = collisionPair->bvol1->getHierarchyNode()->getProxy();
            p2 = collisionPair->bvol2->getHierarchyNode()->getProxy();
            if (!p1 || !p2) {
                throw NullPointerException("one of the Proxies from the collisionPair");
            }

            // note: the called methods MUST take ownership of collisionPair
            if ((p1->getProxyType() & dcollide::PROXYTYPE_RIGID) &&
                    (p2->getProxyType() & dcollide::PROXYTYPE_RIGID)) {
                addPairToRigidJobs(*collisionPair);
            } else {
                // obsolete, deformable objects are not handled by this class.
            }
        }

        finalizeRigidJob();
    }

    /*!
     * Add \p collisionPair to the internal list of rigid collisionPairs. Once a
     * certain number of pairs has been reached, \ref finalizeRigidJob is called
     * to create a single job from all pairs that have been added since \ref
     * finalizeRigidJob has been called the last time.
     *
     * This method is meant to "collect" multiple collisionpairs that should be
     * handled equally (i.e. by the same job), so we do not create multiple jobs
     * (and thus a lot of "new" calls) for them. Note however, that it is
     * absolutely possible to create one job per collisionPair object, by simply
     * setting the threshold accordingly.
     */
    void MiddlePhaseJobCreator::addPairToRigidJobs(const CollisionPair& collisionPair) {
        // TODO make CollisionPair store Proxy pointers.
        //      we usually dont care about the BVs when reading the pairs!!
        Proxy* p1 = collisionPair.bvol1->getHierarchyNode()->getProxy();
        Proxy* p2 = collisionPair.bvol2->getHierarchyNode()->getProxy();
        if (mWorld->getUseCollisionCaching()) {
            bool applied = mWorld->getCollisionCache()->applyCacheIfAvailable(p1, p2);
            if (applied) {
                return;
            }
        }

        if (!mRigidJob) {
            mRigidJob = createMiddlePhaseRigidJob();
        }
        mRigidJob->addInput(collisionPair);
        mRigidItemCountInJob++;
        if (mRigidItemCountInJob >= maxRigidItemCount) {
            finalizeRigidJob();
        }
    }

    /*!
     * Clean up internal data structures and optionally create (rigid) job(s)
     * from them.
     *
     * This is called at least at the end of \ref createJobs and \ref
     * createSelfCollisionJobs.
     */
    void MiddlePhaseJobCreator::finalizeRigidJob() {
        if (!mRigidJob) {
            return;
        }
        mPipeline->addJobForPhase(Pipeline::PHASE_MIDDLEPHASE, mRigidJob);
        mRigidItemCountInJob = 0;
        mRigidJob = 0;
    }

    /*!
     * Takes a completed job and either deletes it or keeps it internally as a
     * pre-allocated job for future use.
     *
     * \param job A job created by \ref MiddlePhaseJobCreator that is not needed
     * anymore. It must NOT be in use anywhere else anymore and might get
     * deleted immediately!
     */
    void MiddlePhaseJobCreator::returnCompletedJob(ThreadJob* job) {
        if (!job) {
            return;
        }
        if (job->getType() != ThreadJob::THREAD_JOB_TYPE_PIPELINE) {
            error() << dc_funcinfo << "invalid job type " << job->getType();
            delete job;
            return;
        }
        MiddlePhaseRigidJob* rigidJob = dynamic_cast<MiddlePhaseRigidJob*>(job);
        if (!rigidJob) {
            error() << dc_funcinfo << "only rigid jobs should be handled by this class!";
            delete job;
            return;
        }
        rigidJob->clear();
        mAllocatedRigidJobs.push_back(rigidJob);
    }

    void MiddlePhaseJobCreator::clearJob(MiddlePhaseRigidJob* job) {
        job->clear();
    }

    /*!
     * Creates a MiddlePhaseRigidJob, either a new one using "new" or a
     * previously created, currently unused, one - see \ref returnCompletedJob.
     **/
    MiddlePhaseRigidJob* MiddlePhaseJobCreator::createMiddlePhaseRigidJob() {
        MiddlePhaseRigidJob* job;
        if (mAllocatedRigidJobs.empty()) {
            job = new MiddlePhaseRigidJob(true, mJobPoolMiddlePhaseRigid);
            job->setDeleteAfterCompletionHint(false);
        } else {
            job = mAllocatedRigidJobs.front();
            mAllocatedRigidJobs.pop_front();
        }
        clearJob(job);

        return job;
    }
}


/*
 * vim: et sw=4 ts=4
 */
