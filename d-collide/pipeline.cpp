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


#include "pipeline.h"

#include "world.h"
#include "worldcollisions.h"
#include "detectorrigid/middlephaserigidjob.h"
#include "detectorrigid/middlephaserigidjobcreator.h"
#include "detectordeform/detectordeformmanager.h"
#include "detectordeform/detectordeformalgorithm.h"
#include "detectordeform/trianglepair.h"
#include "broadphase/broadphasejob.h"
#include "broadphase/broadphase.h"
#include "narrowphase/narrowphasejob.h"
#include "timing.h"
#include "debug.h"
#include "debugstream.h"
#include "thread/threadpool.h"
#include "thread/threadjobcollection.h"
#include "bvhnode.h"
#include "debuglog.h"
#include "boundingvolumes/boundingvolume.h"
#include "collisionpair.h"
#include "proxy.h"
#include "collisioncache.h"
#include "dcollide-config.h"
#include "dcollide-defines.h"

#include <climits>

// TODO:
// atm we still provide public access to World::getWorkerPool(), but that is an
// open invitation to using ThreadPool::addJob(), which should not be used
// anymore (use Pipeline::addJobForPhase() instead).
//
// At least the BroadPhase requires the workerpool however, because it needs a
// jobpool index. So we should let the Pipeline manage jobpools instead, and
// make the BroadPhase retrieve the index from here.

namespace dcollide {
    /*!
     * \brief Helper class to \ref Pipeline
     *
     * This class is used to notify some algorithms about the broadphase being
     * completed, see e.g. \ref
     * DetectorDeformManager::notifyAllCollisionPairsAreAdded.
     *
     * This is a dummy class, there are (currently?) no actual jobs in it, only
     * \ref allJobsCompleted is used. The sole purpose of this job collection is
     * to be added to the broadphase job collection using \ref
     * ThreadJobCollection::addStartOnceCompleted, so that this collection is
     * started once the broadphase is completed.
     */
    class BroadPhasePostProcessingJobCollection : public ThreadJobCollection {
        public:
            BroadPhasePostProcessingJobCollection();

            void setDetectorDeformManager(DetectorDeformManager* manager);
            void setSkipMiddlePhase(bool s);

        protected:
            virtual void allJobsCompleted();

        private:
            DetectorDeformManager* mDetectorDeformManager;
            bool mSkipMiddlePhase;
    };

    BroadPhasePostProcessingJobCollection::BroadPhasePostProcessingJobCollection()
            : ThreadJobCollection() {
        mDetectorDeformManager = 0;
        mSkipMiddlePhase = false;
    }

    void BroadPhasePostProcessingJobCollection::setSkipMiddlePhase(bool s) {
        mSkipMiddlePhase = s;
    }

    void BroadPhasePostProcessingJobCollection::setDetectorDeformManager(DetectorDeformManager* manager) {
        mDetectorDeformManager = manager;
    }

    // FIXME: ThreadJobCollection::allJobsCompleted() always needs to be called.
    //        maybe provide a public interface different from the
    //        implementation()?
    //        -> i.e. provide notifyAllJobsCompleted() as a public method
    //           (non-virtual) and allJobsCompleted() as a protected and
    //           virtual. the latter would be empty in ThreadJobCollection.
    void BroadPhasePostProcessingJobCollection::allJobsCompleted() {
        ThreadJobCollection::allJobsCompleted();
        if (!mSkipMiddlePhase) {
            if (mDetectorDeformManager) {
                mDetectorDeformManager->notifyAllCollisionPairsAreAdded();
            }
        }

        // TODO: most job collection seem to do that here
        // -> move into ThreadJobCollection::allJobsCompleted() ?
        resetCollection();
    }


    Pipeline::Pipeline(World* world) {
        mWorld = world;
        mWorkerPool = mWorld->getWorkerPool();
        mDetectorDeformManager = 0;
        mBroadPhasePostProcessingJobCollection = new BroadPhasePostProcessingJobCollection();

        mCurrentPhase = PHASE_NONE;
        mUsePipelining = true;
        mSkipMiddlePhase = false;
        mSkipNarrowPhase = false;

        mBroadPhaseJobs = &mPipelinedJobs;
        mMiddlePhaseJobs = &mPipelinedJobs;
        mNarrowPhaseJobs = &mPipelinedJobs;

        mMinCollisionsPerBroadPhaseJob = UINT_MAX;
        mMaxCollisionsPerBroadPhaseJob = 0;
        mTotalBroadPhaseCollisions = 0;

        // AB: the job creator may manage a pool of ThreadJob objects internally,
        // so it does not always have to delete+recreate them
        // therefore we need to store an instance of this class in World,
        // instead of creating an instance whenever needed.
        mMiddlePhaseJobCreator = new MiddlePhaseJobCreator(world, this, mWorld->mJobPoolMiddlePhaseRigid);
    }

    Pipeline::~Pipeline() {
        delete mMiddlePhaseJobCreator;
        delete mBroadPhasePostProcessingJobCollection;
    }

    /*!
     * Internal method, called by \ref World only.
     */
    void Pipeline::setDetectorDeformManager(DetectorDeformManager* manager) {
        mDetectorDeformManager = manager;
        mBroadPhasePostProcessingJobCollection->setDetectorDeformManager(mDetectorDeformManager);
    }

    // COLLISIONFLAG_USE_THREADS and COLLISIONFLAG_DISABLE_PIPELINING are completely ignored!!
    // -> World should use these flags only.
    /*!
     * Calculate all collisions in the \ref World, according to \p flags and
     * place the results in \p worldCollisions.
     *
     * If \p getCollisionsWith is non-NULL, only collision pairs that \p
     * getCollisionsWith is part of are considered, instead of all collisions
     * in the \p World. This can be useful to test whether \p getCollisionsWith
     * collides with some proxy when other colliding proxies do not matter.
     *
     * \param getCollisionsWith If NULL this method returns all collisions of all
     * proxies. If non-NULL, only collisions with \p getCollisionsWith are
     * returned, all proxies not colliding with this one are ignored. The proxy
     * must be a toplevel proxy, child-proxies (i.e. a \ref Proxy that is a
     * child of another \ref Proxy) are not supported.
     */
    void Pipeline::calculateAllCollisions(unsigned int flags, WorldCollisions* worldCollisions, Proxy* getCollisionsWith) {
        mSkipMiddlePhase = false;
        mSkipNarrowPhase = false;
        mCollisionInfos.clear();
        if (getUsePipelining() && mWorkerPool->getDisableThreads()) {
            setUsePipelining(false);
        }
        if (!mBroadPhaseJobs->empty() || !mMiddlePhaseJobs->empty() || !mNarrowPhaseJobs->empty()) {
            throw Exception("Pipeline: internal error: not all job lists empty at beginning of collision detection");
        }

        if (flags & World::COLLISIONFLAG_SKIP_MIDDLE_PHASE) {
            mSkipMiddlePhase = true;

            // skip middle phase => skip narrow phase
            flags |= World::COLLISIONFLAG_SKIP_NARROW_PHASE;
        }
        if (flags & World::COLLISIONFLAG_SKIP_NARROW_PHASE) {
            mSkipNarrowPhase = true;
        }

        // AB: when pipelined, we use a single list and start jobs whenever they
        //     are created.
        //     when not pipelined, we have to differ between new middle and
        //     narrow phase jobs and therefore use different lists.
        if (getUsePipelining()) {
            mCurrentPhase = PHASE_NONE;
            mBroadPhaseJobs = &mPipelinedJobs;
            mMiddlePhaseJobs = &mPipelinedJobs;
            mNarrowPhaseJobs = &mPipelinedJobs;
        } else {
            mCurrentPhase = PHASE_BROADPHASE;
            mBroadPhaseJobs = &mUnpipelinedBroadPhaseJobs;
            mMiddlePhaseJobs = &mUnpipelinedMiddlePhaseJobs;
            mNarrowPhaseJobs = &mUnpipelinedNarrowPhaseJobs;
        }

        mMinCollisionsPerBroadPhaseJob = UINT_MAX;
        mMaxCollisionsPerBroadPhaseJob = 0;
        mTotalBroadPhaseCollisions = 0;

        Timing time;
        mWorld->getBroadPhase()->createBroadPhaseJobs(getCollisionsWith);
        if (!mWorld->getBroadPhase()->getJobCollection()) {
            throw NullPointerException("mWorld->getBroadPhase()->getJobCollection()");
        }

        PipelineThreadJobCollection* broadPhaseJobCollection = mWorld->getBroadPhase()->getJobCollection();
        if (broadPhaseJobCollection->getJobsInCollectionCount() == 0) {
            // AB: we essentially skip the broadphase here.
            // this is
            // a) a little hack to enforce the usage of the job collection
            //    (instead of plain ThreadJobs) in the broadphase
            //    -> some algorithms (e.g. the spatialhash) require this, to
            //       get notified about completion of the broadphase
            // b) a small optimization for the case that the broadphase
            //    already knows that nothing will ever collide and thus
            //    won't need to create jobs at all
            //    (e.g. no collidable proxies in the world)
            // of course b) doesnt really matter :-)
            mSkipMiddlePhase = true;
            mSkipNarrowPhase = true;
        } else {
            broadPhaseJobCollection->start();
            worldCollisions->setBroadPhaseCollisions(mWorld->getBroadPhase()->getBroadPhaseCollisions());
        }

        mWorld->getBroadPhase()->getJobCollection()->addStartOnceCompleted(mBroadPhasePostProcessingJobCollection);
        mBroadPhasePostProcessingJobCollection->setSkipMiddlePhase(mSkipMiddlePhase);

        // mBroadPhasePostProcessingJobCollection is just a dummy job collection
        // without any actual jobs
        // -> we just need the allJobsCompleted() method
        mBroadPhasePostProcessingJobCollection->setAllJobsAreAdded();

        if (!mSkipMiddlePhase) {
            mMiddlePhaseJobCreator->createSelfCollisionJobs(mWorld->getSelfcollisionProxies());

            if (mDetectorDeformManager) {
                // note: once this is called, internal job collections of the
                // DetectorDeformAlgorithm objects will add their jobs to the
                // pipeline - but the _pipeline_ will actually start the jobs
                mDetectorDeformManager->notifyPipelineStarted();
            }

            // add deformable self-collision jobs
            if (mDetectorDeformManager) {
                const std::list<Proxy*>& selfCollisionProxies = mWorld->getSelfcollisionProxies();
                for (std::list<Proxy*>::const_iterator it = selfCollisionProxies.begin(); it != selfCollisionProxies.end(); ++it) {
                    if (!((*it)->getProxyType() & PROXYTYPE_DEFORMABLE)) {
                        continue;
                    }

                    if (mWorld->getUseCollisionCaching()) {
                        bool cacheApplied = mWorld->getCollisionCache()->applyCacheIfAvailable(*it, *it);
                        if (cacheApplied) {
                            continue;
                        }
                    }

                    DetectorDeformAlgorithm* algorithm = mDetectorDeformManager->pickAlgorithmFor(*it, *it);
                    if (algorithm) {
                        CollisionPair pair;

                        // FIXME: make CollisionPair store Proxy pointers, not
                        // BoundingVolume pointers.
                        pair.bvol1 = (*it)->getBvHierarchyNode()->getBoundingVolume();
                        pair.bvol2 = pair.bvol1;
                        algorithm->createCollisionJobFor(pair);
                    }
                }
            }
        }

        if (!getUsePipelining()) {
            completeCurrentPhase();
            time.stop();
            mWorld->getCurrentDebugLogEntry()->addTiming("BroadPhase", time);

            time.restart();
            mCurrentPhase = PHASE_MIDDLEPHASE;
            completeCurrentPhase();
            time.stop();
            mWorld->getCurrentDebugLogEntry()->addTiming("MiddlePhase", time);

            time.restart();
            mCurrentPhase = PHASE_NARROWPHASE;
            completeCurrentPhase();
            time.stop();
            mWorld->getCurrentDebugLogEntry()->addTiming("NarrowPhase", time);
        } else {
            completeCurrentPhase();
            time.stop();

            // in a pipelined run we cannot differ between phases
            // AB: note: "BroadPhase", "MiddlePhase" and "NarrowPhase" timings
            //     can still be retrieved from the log - since we havent added
            //     them, they will be nullified Timing objects.
            mWorld->getCurrentDebugLogEntry()->addTiming("Pipeline", time);
        }

        if (mDetectorDeformManager) {
            // reset the job collections of the algorithms
            // note: at this point all jobs must already be completed!
            mDetectorDeformManager->notifyPipelineCompleted();
        }

        // pipeline has been completed at this point. add results to
        // worldCollisions.
        worldCollisions->setRigidBoundingVolumeCollisions(&mMiddlePhaseRigidResults);
        worldCollisions->addNarrowPhaseCollisions(&mCollisionInfos);

        // post-pipeline algorithms
        time.restart();
        startCompletelyUnthreadedAlgorithms(worldCollisions);
        time.stop();
        mWorld->getCurrentDebugLogEntry()->addTiming("UnthreadedPhase", time);

        mWorld->getCurrentDebugLogEntry()->setUIntVariable("BroadPhase job count", mWorld->getBroadPhase()->getJobCollection()->getJobsInCollectionCount());
        mWorld->getCurrentDebugLogEntry()->setUIntVariable("BroadPhase min collisions per job", mMinCollisionsPerBroadPhaseJob);
        mWorld->getCurrentDebugLogEntry()->setUIntVariable("BroadPhase max collisions per job", mMaxCollisionsPerBroadPhaseJob);
        mWorld->getCurrentDebugLogEntry()->setUIntVariable("BroadPhase collisions", mTotalBroadPhaseCollisions);

        // sanity check
        mWorkerPool->waitForCompletion();
        if (mWorkerPool->hasCompletedJobs()) {
            std::cerr << dc_funcinfo << "ERROR: completed jobs left" << std::endl;
            while (mWorkerPool->hasCompletedJobs()) {
                delete mWorkerPool->retrieveCompletedJob();
            }
        }

        mCurrentPhase = PHASE_INVALID;

        if (!mBroadPhaseJobs->empty() || !mMiddlePhaseJobs->empty() || !mNarrowPhaseJobs->empty()) {
            throw Exception("Pipeline: internal error: not all job lists empty at end of collision detection");
        }
    }

    /*!
     * Start algorithms that do not yet use the \ref ThreadJob design.
     *
     * This method is supposed primarily for testing purposes, actual algorithms
     * should be implemented using \ref ThreadJob. Note that for using \ref
     * ThreadJob your algorithms does not have to be multithreaded: simply
     * create a single job only.
     *
     * When this method is called, the normal collision detection pipeline
     * (including narrowphase) has already been completed (unless it has been
     * skipped partially).
     */
    void Pipeline::startCompletelyUnthreadedAlgorithms(WorldCollisions* worldCollisions) {
        DCOLLIDE_UNUSED(worldCollisions);
    }


    /*!
     * Called when at least one \ref ThreadJob has been completed, i.e. \ref
     * ThreadPool::hasCompletedJobs of the \ref getWorkerPool is TRUE.
     *
     * This method reads all completed jobs from the \ref getWorkerPool and
     * reads their results. For example if there are completed broadphase jobs
     * available, this method will retrieve them from the workerpool and create
     * middlephase jobs for all pairs in the broadphase results provided by that
     * job.
     *
     * Usually "reading the result" means simply calling \ref
     * PipelineThreadJob::processResults, if the jobs are of type \ref
     * PipelineThreadJob. The jobs are then responsible to return their results
     * to the correct data structures.
     */
    void Pipeline::processCompletedJobs() {
        while (mWorkerPool->hasCompletedJobs()) {
            std::list<ThreadJob*> jobs;
            mWorkerPool->retrieveCompletedJobs(&jobs);
            for (std::list<ThreadJob*>::iterator it = jobs.begin(); it != jobs.end(); ++it) {
                ThreadJob* job = *it;
                switch (job->getType()) {
                    case ThreadJob::THREAD_JOB_TYPE_PIPELINE:
                    {
                        PipelineThreadJob* p = static_cast<PipelineThreadJob*>(job);
                        p->processResults(this);
                        break;
                    }
                    default:
                        error() << dc_funcinfo << "unrecognized job type " << job->getType();
                        break;
                }
            }

            // AB: we _first_ have to call job->processResults() or whatever
            //     result-reading method is used.
            //     _then_ we can call ThreadJobCollection::processCompletedJobs()
            //     --> in particular this may call allJobsCompleted() and it
            //         would be very counter-intuitive, if at that point the
            //         jobs still would be needed for result reading!
            //
            //         for example: if the last broadphase job is among jobs,
            //         and we would _first_ call allJobsCompleted(), then
            //         DetectorDeformManager::notifyAllCollisionPairsAreAdded()
            //         would be called, _before_ the pairs from that job would
            //         have been added!
            ThreadJobCollection::processCompletedJobs(jobs);

            for (std::list<ThreadJob*>::iterator it = jobs.begin(); it != jobs.end(); ++it) {
                ThreadJob* job = *it;
                if (job->getDeleteAfterCompletionHint()) {
                    delete job;
                }
            }
        }
    }

    /*!
     * This method is meant to take the results from a \ref PipelineThreadJob
     * subclass and store it as middlephase results.
     *
     * This version of this method takes a list of \ref BoundingVolumeCollision
     * objects, containing a pair of \ref BvhNode.
     *
     * \ref NarrowPhaseJobs will be created (unless the user requested to skip
     * the narrowphase) that will intersect the \ref Triangle objects from both
     * \ref BvhNode with each other and produce \ref NarrowPhase results.
     */
    void Pipeline::processAndSpliceMiddlePhaseResults(std::list<BoundingVolumeCollision>& results) {
        if (!mSkipNarrowPhase) {
            createNarrowPhaseJobs(results);
        }

        if (mWorld->getUseCollisionCaching()) {
            mWorld->getCollisionCache()->addToCacheElement(results);
        }

        // AB: this method claims that results is spliced, i.e. is cleared after
        //     calling this method
        // TODO: splice the results to some kind of "MiddlePhaseResults" in
        // WorldCollisions!
        // TODO: remember to also fill the "MiddlePhaseResults" from the
        // collisioncache!
        results.clear();
    }

    /*!
     * \overload
     *
     * This version of this method takes a list of \ref PotentialCollidingSets objects.
     * \ref NarrowPhaseJobs will be created (unless the user requested to skip
     * the narrowphase) that will intersect all pairs of triangles: \n
     * (x,y) : x e SetOne, y e SetTwo .
     */
    void Pipeline::processAndSpliceMiddlePhaseResults(std::list<PotentialCollidingSets>& results) {
        if (!mSkipNarrowPhase) {
            createNarrowPhaseJobs(results);
        }

        /*Collision- caching is disabled for now
        if (mWorld->getUseCollisionCaching()) {
            mWorld->getCollisionCache()->addToCacheElement(results);
        }
        */

        for(std::list<PotentialCollidingSets>::iterator i = results.begin(); 
            i != results.end(); ++i ) {
         
           (*i).setOne.clear();
           (*i).setTwo.clear();
        }
    }

    /*!
     * \overload
     *
     * This version of this method takes a list of \ref TrianglePair objects.
     * \ref NarrowPhaseJobs will be created (unless the user requested to skip
     * the narrowphase) that will intersect both \ref Triangle objects in a
     * pair with each other and produce \ref NarrowPhase results.
     */
    void Pipeline::processAndSpliceMiddlePhaseResults(std::list<TrianglePair>& results) {
        if (!mSkipNarrowPhase) {

            createNarrowPhaseJobs(results);
            //debug() << dc_funcinfo << "TODO";
            // TODO: create NarrowPhaseJobs
            //createNarrowPhaseJobs(results);
        }

        if (mWorld->getUseCollisionCaching()) {
            mWorld->getCollisionCache()->addToCacheElement(results);
        }

        // AB: this method claims that results is spliced, i.e. is cleared after
        //     calling this method
        // TODO: splice the results to some kind of "MiddlePhaseResults" in
        // WorldCollisions!
        // TODO: remember to also fill the "MiddlePhaseResults" from the
        // collisioncache!
        results.clear();
    }

    /*!
     * \overload
     *
     * This version of this method takes a list of \ref CollisionInfo objects,
     * i.e. it takes results that already qualify as \ref NarrowPhase results.
     * They will appear as narrowphase results and no \ref NarrowPhase job will
     * be created.
     */
    void Pipeline::processAndSpliceMiddlePhaseResults(std::list<CollisionInfo>& results) {
        // AB: collision cache is handled by spliceNarrowPhaseResults()
        //
        spliceNarrowPhaseResults(results);
    }

    /*!
     * Convenience function used by \ref MiddlePhaseRigidJob::processResults.
     *
     * This function has mainly historic reasons: previously this code was in
     * \ref processCompletedJobs directly.
     */
    void Pipeline::processCompletedMiddlePhaseRigidJob(MiddlePhaseRigidJob* job) {
        // AB: we should use processAndSpliceMiddlePhaseResults() here, but atm
        // that method does not store middlephase results (because atm we have a
        // mMiddlePhase_Rigid_Results only)
        if (!job->getCollisions().empty()) {
#if 1
            if (!mSkipNarrowPhase) {
                createNarrowPhaseJobs(job->getCollisions());
            }
            if (mWorld->getUseCollisionCaching()) {
                mWorld->getCollisionCache()->addToCacheElement(job->getCollisions());
            }
            mMiddlePhaseRigidResults.splice(mMiddlePhaseRigidResults.end(), job->getCollisions());
#else
            processAndSpliceMiddlePhaseResults(job->getCollisions());
#endif
        }
        mMiddlePhaseJobCreator->returnCompletedJob(job);
    }

    /*!
     * Called internally by \ref CollisionCache. You should never call this
     * directly!
     *
     * This reads the cached results from \p element and adds them to the
     * results of the current collision detection run.
     */
    void Pipeline::processCollisionCacheElement(CollisionCacheElement* element) {
        mMiddlePhaseRigidResults.insert(mMiddlePhaseRigidResults.end(), element->mBoundingVolumeCollisions.begin(), element->mBoundingVolumeCollisions.end());
        mCollisionInfos.insert(mCollisionInfos.end(), element->mNarrowPhaseCollisions.begin(), element->mNarrowPhaseCollisions.end());
        // TODO mTrianglePairs.insert(mTrianglePairs.end(), element->mTrianglePairCollisions.begin(), element->mTrianglePairCollisions.end());
    }

    /*!
     * Convenience function used by \ref BroadPhaseJob::processResults.
     *
     * This function has mainly historic reasons: previously this code was in
     * \ref processCompletedJobs directly.
     */
    void Pipeline::processCompletedBroadPhaseJob(BroadPhaseJob* job) {
        if (!mSkipMiddlePhase) {
            unsigned int count = job->getJobResultsReference().size();
            mMinCollisionsPerBroadPhaseJob = std::min(mMinCollisionsPerBroadPhaseJob, count);
            mMaxCollisionsPerBroadPhaseJob = std::max(mMaxCollisionsPerBroadPhaseJob, count);
            mTotalBroadPhaseCollisions += count;
            createMiddlePhaseJobs(job);
        }
    }

    void Pipeline::createNarrowPhaseJobs(std::list<PotentialCollidingSets>& collisions) {
        NarrowPhaseJob* newPCSJob = new NarrowPhaseJob(0, mWorld->mNarrowPhaseShapeStrategies);
        newPCSJob->assignInput(collisions);
        addJobForPhase(PHASE_NARROWPHASE, newPCSJob);
    }


    void Pipeline::createNarrowPhaseJobs(const std::list<TrianglePair>& collisions) {
        int jobPoolIndex = 0;
        int maxcount = 50;

        NarrowPhaseJob* job = new NarrowPhaseJob(jobPoolIndex, mWorld->mNarrowPhaseShapeStrategies);

        int trianglesPerJobCount = 0;
        for (std::list<TrianglePair>::const_iterator i = collisions.begin(); i != collisions.end(); ++i) {
            if (trianglesPerJobCount >= maxcount ) {
                addJobForPhase(PHASE_NARROWPHASE, job);

                job = new NarrowPhaseJob(jobPoolIndex, mWorld->mNarrowPhaseShapeStrategies);
            }

            job->addInput(*i);
            trianglesPerJobCount++;
        }
        addJobForPhase(PHASE_NARROWPHASE,job);

    }
    void Pipeline::createNarrowPhaseJobs(const std::list<BoundingVolumeCollision>& collisions) {
        // AB: a single job processes at most maxCount BoundingVolumeCollision
        //     objects.
        //     note that a 1:1 relationship (i.e. maxCount==1) may not be
        //     wise, since a single collision is expected to be fast and
        //     thus the job overhead may be too significant.
        const unsigned int maxCount = 20;

        NarrowPhaseJob* job = 0;
        unsigned int count = 0;

        for (std::list<BoundingVolumeCollision>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
            if (!job) {
                unsigned int jobPoolIndex = 0; // TODO: store job index somewhere (and pick a proper index)
                job = new NarrowPhaseJob(jobPoolIndex, mWorld->mNarrowPhaseShapeStrategies);
            }

            job->addInput(*it);
            count++;
            if (count >= maxCount) {
                addJobForPhase(PHASE_NARROWPHASE, job);
                job = 0;
            }

        }
        if (job) {
            addJobForPhase(PHASE_NARROWPHASE, job);
        }
    }

    /*!
     * Start all jobs in \p list, i.e. add them to the \ref getWorkerPool
     */
    void Pipeline::startJobs(List<ThreadJob*>* list) {
        for (ListNode<ThreadJob*>* node = list->getFirstNode(); node; node = node->getNext()) {
            mWorkerPool->addJob(node->getData());
        }
        list->clear();
    }

    /*!
     * Create middlephase jobs according to the \ref BroadPhaseJob.
     *
     * See also \ref MiddlePhaseJobCreator and \ref
     * DetectorDeformManager::pickAlgorithmFor
     */
    void Pipeline::createMiddlePhaseJobs(BroadPhaseJob* broadPhaseJob) {
        List<CollisionPair*>& jobResults = broadPhaseJob->getJobResultsReference();
        for (ListNode<CollisionPair*>* node = jobResults.getFirstNode(); node; node = node->getNext()) {
            CollisionPair* pair = node->getData();

            // TODO make CollisionPair store Proxy pointers.
            //      we usually dont care about the BVs when reading the pairs!!
            Proxy* p1 = pair->bvol1->getHierarchyNode()->getProxy();
            Proxy* p2 = pair->bvol2->getHierarchyNode()->getProxy();

            if (p1->getProxyType() & PROXYTYPE_DEFORMABLE ||
                    p2->getProxyType() & PROXYTYPE_DEFORMABLE) {

                if (mWorld->getUseCollisionCaching()) {
                    bool cacheApplied = mWorld->getCollisionCache()->applyCacheIfAvailable(p1, p2);
                    if (cacheApplied) {
                        continue;
                    }
                }

                DetectorDeformAlgorithm* algorithm = mDetectorDeformManager->pickAlgorithmFor(p1, p2);
                if (algorithm) {
                    // TODO: fix API.
                    //       createCollisionJobFor() uses CollisionPair,
                    //       pickAlgorithmFor() uses two Proxy pointers.
                    //       choose one!
                    algorithm->createCollisionJobFor(*pair);
                }
            }
        }


        mMiddlePhaseJobCreator->createJobs(broadPhaseJob);
    }

    /*!
     * This waits for completion of all jobs of the current phase
     * (broad/middle/narrow).
     *
     * If the completion of a job triggers the creation of one or more new jobs,
     * they are started only if they are part of the current phase.
     *
     * This method should be used only if \ref getUsePipelining returns FALSE.
     */
    void Pipeline::completeCurrentPhase() {
        startJobsOfPhase(mCurrentPhase);
        int waitForCompletionCount = 0;
        if (mCurrentPhase == PHASE_BROADPHASE) {
            waitForCompletionCount = 1;
        } else {
            waitForCompletionCount = 10;
        }
        mWorkerPool->waitForCompletion(waitForCompletionCount);
        while (mWorkerPool->hasWorkLeft() || mWorkerPool->hasCompletedJobs()) {
            processCompletedJobs();

            startJobsOfPhase(mCurrentPhase);

            // wait for completion of newly added jobs (if any)
            mWorkerPool->waitForCompletion(waitForCompletionCount);
        }
    }

    /*!
     * Start all jobs of \p phase that have been added to the pipeline (see \ref
     * addJobForPhase) as well as jobs from \em all previous phases.
     */
    void Pipeline::startJobsOfPhase(Phase phase) {
        switch (phase) {
            case PHASE_NONE: // pipelined
                startJobs(&mPipelinedJobs);
                break;
            case PHASE_BROADPHASE:
                startJobs(mBroadPhaseJobs);
                break;
            case PHASE_MIDDLEPHASE:
                // AB: start from broadphase too, in case some job was
                // accidentally added there (should be completed already)
                startJobs(mBroadPhaseJobs);
                startJobs(mMiddlePhaseJobs);
                break;
            case PHASE_NARROWPHASE:
                // AB: start from broad/middlephase too, in case some job was
                // accidentally added there (should be completed already)
                startJobs(mBroadPhaseJobs);
                startJobs(mMiddlePhaseJobs);
                startJobs(mNarrowPhaseJobs);
                break;
            case PHASE_INVALID:
                throw Exception("Cannot start jobs of phase \"invalid\"");
                break;
        }
    }

    /*!
     * Add a job to the pipeline and request it to be executed for phases \p
     * phase.
     *
     * Note that in pipelined mode all jobs in the pipeline are started at the
     * same time.
     */
    void Pipeline::addJobForPhase(Phase phase, ThreadJob* job) {
        if (mCurrentPhase == PHASE_INVALID) {
            throw("Cannot add jobs to the pipeline before pipeline has been started.");
            return;
        }
        switch (phase) {
            default:
            case PHASE_INVALID:
                throw("Cannot add jobs to \"invalid\" phase");
                break;
            case PHASE_NONE:
                throw("Cannot add jobs to \"none\" phase");
                break;
            case PHASE_BROADPHASE:
                mBroadPhaseJobs->push_back(job);
                break;
            case PHASE_MIDDLEPHASE:
                mMiddlePhaseJobs->push_back(job);
                if (mSkipMiddlePhase) {
                    std::cout << "WARNING: a job has been added to middlephase, although middlephase is requested to be skipped" << std::endl;
                }
                break;
            case PHASE_NARROWPHASE:
                mNarrowPhaseJobs->push_back(job);
                if (mSkipNarrowPhase) {
                    std::cout << "WARNING: a job has been added to narrowphase, although narrowphase is requested to be skipped" << std::endl;
                }
                break;
        }

        if (phase == mCurrentPhase) {
            startJobsOfPhase(mCurrentPhase);
        }
    }

    void Pipeline::spliceNarrowPhaseResults(std::list<CollisionInfo>& results) {
        if (mWorld->getUseCollisionCaching()) {
            mWorld->getCollisionCache()->addToCacheElement(results);
        }
        mCollisionInfos.splice(mCollisionInfos.end(), results);
    }


    PipelineThreadJob::PipelineThreadJob(unsigned int jobPool)
            : ThreadJob(jobPool) {
    }

    /*!
     * Derived classes do \em not need to reimplement this method. If they do,
     * \ref processResults will not be called when the job is completed.
     *
     * \return \ref THREAD_JOB_TYPE_PIPELINE
     */
    int PipelineThreadJob::getType() const {
        return THREAD_JOB_TYPE_PIPELINE;
    }

    /*!
     * Create \ref PipelineThreadJobCollection object that uses \p
     * pipeline nd starts its jobs in \p phase (see \ref Pipeline::Phase).
     */
    PipelineThreadJobCollection::PipelineThreadJobCollection(Pipeline* pipeline, Pipeline::Phase phase) {
        mPipeline = pipeline;
        if (!mPipeline) {
            throw NullPointerException("mPipeline");
        }
        mPhase = phase;
    }

    PipelineThreadJobCollection::~PipelineThreadJobCollection() {
    }

    /*!
     * Like \ref ThreadJobCollection::startJob, but instead of starting the jobs
     * directly, this method adds them to the \ref Pipeline in the phase
     * specified by the constructor of this class.
     *
     * You should \em not use this method directly - use \ref start instead.
     * Using this method directly causes undefine behaviour!
     */
    void PipelineThreadJobCollection::startJob(ThreadJob* job) {
        mPipeline->addJobForPhase(mPhase, job);
    }

}
/*
 * vim: et sw=4 ts=4
 */
