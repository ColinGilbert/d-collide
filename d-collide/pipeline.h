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

#ifndef DCOLLIDE_PIPELINE_H
#define DCOLLIDE_PIPELINE_H

#include "dcollide-global.h"
#include "datatypes/list.h"
#include "thread/threadjobcollection.h"
#include "thread/threadpool.h"
#include "detectordeform/trianglepair.h"
#include "detectordeform/potentialcollidingsets.h"
#include <list>

namespace dcollide {
    class World;
    class ThreadPool;
    class ThreadJob;
    class BroadPhase;
    class BroadPhaseJob;
    class NarrowPhaseJob;
    class DetectorDeform;
    class WorldCollisions;
    class Proxy;
    class PipelineThreadJobCollection;
    struct BoundingVolumeCollision;
    struct CollisionInfo;
    class DetectorDeformManager;
    class MiddlePhaseRigidJob;
    class BroadPhasePostProcessingJobCollection;
    struct TrianglePair;
    class MiddlePhaseJobCreator;
    struct CollisionCacheElement;

    /*!
     * \brief Collision detection pipeline of the \ref World
     *
     * This class is used by \ref World and represents the collision detection
     * pipeline.
     *
     * All collision detection algorithms in d-collide are meant to be
     * implemented using \ref ThreadJob objects (unthreaded algorithms can be
     * implemented by using a single object only). This class is based on the
     * assumption that all algorithms use \ref ThreadJob objects.
     *
     * In d-collide there are three phases:
     * \li The \ref BroadPhase which finds all pairs that possibly collide with
     *     each other. This is only very rough collision detection meant to
     *     reduce the O(n^2) possible pairs to a much smaller amount.
     * \li The middle phase, consisting of rigid and deformable collision
     *     detection (usually separate algorithms for pairs with rigid proxies
     *     only and pairs with at least one deformable proxies are used). The
     *     middlephase takes the results of the \ref BroadPhase as input and for
     *     each pair it is meant to find out which parts of the two \ref Proxy
     *     hierarchies collide with each other (if at all). The output of the
     *     middlephase is usually not "exact", i.e. does not output the exact
     *     collision points, but rather more abstract, such as colliding
     *     bounding volumes or possibly colliding triangle pairs.
     * \li The narrow phase takes the output of the middlephase and performs the
     *     final (exact) collision detection, i.e. mainly triangle-triangle
     *     tests (for \ref Mesh - \ref Mesh collisions) and outputs the final
     *     results.
     *     Note that some middlephase algorithms may already output narrowphase
     *     results - obviously the narrowphase is skipped for these cases.
     *
     * The entire Pipeline is "pipelined", that is all phases can run in
     * parallel. For example once the \ref BroadPhase has returned a few
     * collision pairs, the middlephase can already start working, even before
     * the \ref BroadPhase has finished completely. Of course this makes only
     * sense if multithreading is activated.
     *
     * See also \ref PipelineThreadJob
     */
    class Pipeline {
        public:
            enum Phase {
                PHASE_BROADPHASE = 0,
                PHASE_MIDDLEPHASE = 1,
                PHASE_NARROWPHASE = 2,
                PHASE_NONE,
                PHASE_INVALID

            };
        public:
            explicit Pipeline(World* world);
            ~Pipeline();

            void setDetectorDeformManager(DetectorDeformManager* manager);

            void calculateAllCollisions(unsigned int flags, WorldCollisions* worldCollisions, Proxy* getCollisionsWith = 0);

            inline ThreadPool* getWorkerPool() const;

            inline void setUsePipelining(bool use);
            inline bool getUsePipelining() const;

            void addJobForPhase(Phase phase, ThreadJob* job);

            // AB: internal note: if you add another result type, make sure the
            //     CollisionCache knows about that type too!
            //Frederick: the CollisionCache does not know about the PotentialCollidingSets
            //up to now

            void processAndSpliceMiddlePhaseResults(std::list<BoundingVolumeCollision>& results);
            void processAndSpliceMiddlePhaseResults(std::list<PotentialCollidingSets>& result);

            //This method is depecreated by the PotentialColldidingSets  result type
            //and will be removed, when all deformable algorithms have switched to the new framework
            void processAndSpliceMiddlePhaseResults(std::list<TrianglePair>& results);
            void processAndSpliceMiddlePhaseResults(std::list<CollisionInfo>& results);

            void processCollisionCacheElement(CollisionCacheElement* element);

            void processCompletedMiddlePhaseRigidJob(MiddlePhaseRigidJob* job);
            void processCompletedBroadPhaseJob(BroadPhaseJob* job);
            void spliceNarrowPhaseResults(std::list<CollisionInfo>& results);

        protected:
            void startCompletelyUnthreadedAlgorithms(WorldCollisions*);

            void processCompletedJobs();
            void createNarrowPhaseJobs(std::list<PotentialCollidingSets>& collisions);
            void createNarrowPhaseJobs(const std::list<TrianglePair>& collisions);
            void createNarrowPhaseJobs(const std::list<BoundingVolumeCollision>& collisions);
            void createMiddlePhaseJobs(BroadPhaseJob* broadPhaseJob);

            void completeCurrentPhase();
            void startJobsOfPhase(Phase phase);
            void startJobs(List<ThreadJob*>* list);

        private:
            Phase mCurrentPhase;
            World* mWorld;
            BroadPhasePostProcessingJobCollection* mBroadPhasePostProcessingJobCollection;
            DetectorDeformManager* mDetectorDeformManager;
            MiddlePhaseJobCreator* mMiddlePhaseJobCreator;
            ThreadPool* mWorkerPool;
            bool mUsePipelining;
            bool mSkipMiddlePhase;
            bool mSkipNarrowPhase;

            List<ThreadJob*>* mBroadPhaseJobs;
            List<ThreadJob*>* mNarrowPhaseJobs;
            List<ThreadJob*>* mMiddlePhaseJobs;
            List<ThreadJob*> mPipelinedJobs;
            List<ThreadJob*> mUnpipelinedBroadPhaseJobs;
            List<ThreadJob*> mUnpipelinedMiddlePhaseJobs;
            List<ThreadJob*> mUnpipelinedNarrowPhaseJobs;

            std::list<CollisionInfo> mCollisionInfos;
            std::list<BoundingVolumeCollision> mMiddlePhaseRigidResults;

            unsigned int mMinCollisionsPerBroadPhaseJob;
            unsigned int mMaxCollisionsPerBroadPhaseJob;
            unsigned int mTotalBroadPhaseCollisions;
    };

    /*!
     * Like \ref ThreadJob, but provides additional means to read the results
     * once the job has been completed. This class should be preferred over \ref
     * ThreadJob, when writing a job for use in the \ref Pipeline (which is
     * normally the case!)
     *
     * Note that \ref getType should \em not be reimplemented by derived classes!
     */
    class PipelineThreadJob : public ThreadJob {
        public:
            explicit PipelineThreadJob(unsigned int jobPool = 0);

            virtual int getType() const;

            /*!
             * Called when the job completed execution and when the pipeline is
             * ready to take the results from it.
             *
             * In particular this means that no other (pipeline-)thread is
             * running currently, i.e. the job can write the results without any
             * mutex lock back to more global data structures.
             */
            virtual void processResults(Pipeline* pipeline) = 0;
    };

    /*!
     * \ref ThreadJobCollection that knows about \ref Pipeline and honors the
     * phases, i.e. when \ref start is called the jobs are added to the \ref
     * Pipeline instead of the \ref ThreadPool (the \ref Pipeline adds them
     * there).
     *
     * You should always prefer this class over \ref ThreadJobCollection when
     * you need a job collection for collision detection.
     */
    class PipelineThreadJobCollection : public ThreadJobCollection {
        public:
            PipelineThreadJobCollection(Pipeline* pipeline, Pipeline::Phase phase);
            virtual ~PipelineThreadJobCollection();

        protected:
            virtual void startJob(ThreadJob*);

        private:
            Pipeline* mPipeline;
            Pipeline::Phase mPhase;
    };


    ThreadPool* Pipeline::getWorkerPool() const {
        return mWorkerPool;
    }

    inline void Pipeline::setUsePipelining(bool use) {
        mUsePipelining = use;
    }

    inline bool Pipeline::getUsePipelining() const {
        return mUsePipelining;
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
