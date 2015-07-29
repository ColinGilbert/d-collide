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


#ifndef DCOLLIDE_THREADJOBCOLLECTION_H
#define DCOLLIDE_THREADJOBCOLLECTION_H

#include "../datatypes/list.h"
#include "thread.h" // Mutex
#include <list>

/*
 * design comments:
 * - setJobCompleted() is NOT called automatically.
 *   this is mainly because I have no idea where to do that.
 *   I want it to be called from the main thread only, because it may trigger a
 *   call to allJobsCompleted() and consequently a call to
 *   ThreadJobCollection::start(), which usually adds jobs to the pool.
 *   but calling ThreadPool::addJob() from outside the main thread is most
 *   likely not a wise idea.
 *   furthermore it may simplify some tasks to be done in allJobsCompleted() by
 *   some derived classes if we know that at most one collection can enter
 *   allJobsCompleted() at the same time.
 *   therefore i cannot call setJobCompleted() when the job is actually
 *   completed and neither from ThreadPool::retrieveCompletedJobs(), as that one
 *   does not guarantee to be called from the main thread only.
 * - something like removeJob() is not provided in order to prevent stupid
 *   performance bugs. removeJob() would require to traverse the mJobs list,
 *   which requires O(n) runtime. if this class would provide such a method, it
 *   would probably be used in places where it should not be used. so instead we
 *   require to reset() the whole thing and re-add all jobs that should remain
 *   in the collection: this is better, because usually you want to remove ALL
 *   jobs from the collection, or none at all. not providing a removeJob()
 *   method makes it more natural to pick one of these 2 ways.
 */

namespace dcollide {
    class ThreadJob;
    class ThreadPool;

    /*!
     * \brief A collection of multiple \ref ThreadJob objects
     *
     * Normally you should prefer \ref PiplineThreadJobCollection over this
     * class.
     *
     * A ThreadJobCollection object groups multiple jobs together. They can be
     * started all at once using \ref start and the ThreadJobCollection will be
     * notified once they are completed. Once all jobs are completed \ref
     * allJobsCompleted is called, which can trigger other ThreadJobCollection
     * object to get started, see \ref addStartOnceCompleted.
     *
     * An example usage might look like this:
     * \code
     * mBroadPhaseJobCollection = new ThreadJobCollection();
     * mBroadPhaseJobCollection->addJob(myJob1);
     * mBroadPhaseJobCollection->addJob(myJob2);
     * //...
     *
     * mSpatialHashJobCollection = new ThreadJobCollection();
     * mSpatialHashJobCollection->addJob(mSpatialHashJob);
     * mBroadPhaseJobCollection->addStartOnceCompleted(mSpatialHashJobCollection);
     *
     * // ...
     *
     * // start collision detection (broadphase + spatialhash)
     * mBroadPhaseJobCollection->start();
     * \endcode
     * In this example the "mBroadPhaseJobCollection" is started with a certain
     * number of jobs. Once the entire broadphase (i.e. all jobs in the
     * collection) is completed, another collection, "mSpatialHashJobCollection"
     * is started, with only one job. This job can now assume that the
     * broadphase is already completed and thus all results are available.
     *
     * As seen in the example depicted above the main point of a collection is
     * to define simple dependencies among jobs.
     *
     * Another useful aspect may be result processing: by deriving from this
     * class and reimplementing \ref allJobsCompleted, you can do certain
     * post-processing of the jobs in the collection, for example you can delete
     * all jobs (and thus free the memory allocated by them) or create new jobs.
     * One simple example:
     * \code
     * void BroadPhaseJobCollection::allJobsCompleted() {
     *     // always call the base implementation!
     *     ThreadJobCollection::allJobsCompleted();
     *
     *     // the broadphase is completed at this point, so postprocessing of
     *     // the broadphase datastructures can be done now (the "update" job)
     *     mBroadPhase->addUpdateJob();
     * }
     * \endcode
     *
     * Most methods in this class are NOT thread safe - it is up to the caller
     * to ensure thread safety.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class ThreadJobCollection {
        public:
            ThreadJobCollection();
            virtual ~ThreadJobCollection();

            void setPool(ThreadPool* pool);
            void start();

            void addJob(ThreadJob*);
            void setAllJobsAreAdded();
            inline bool getAllJobsAreAdded() const;
            inline  unsigned int getJobsInCollectionCount() const;
            const List<ThreadJob*>& getJobs() const;

            inline bool getAllJobsCompleted() const;

            virtual void setJobCompleted(ThreadJob* job);

            void resetCollection();
            virtual void resetCompleted();

            void addStartOnceCompleted(ThreadJobCollection* collection);

            static void processCompletedJobs(const std::list<ThreadJob*>& jobs);
            static void processCompletedJob(ThreadJob* job);

        protected:
            virtual void allJobsCompleted();

            virtual void startJob(ThreadJob* job);

        private:
            ThreadPool* getPool() const;

        private:
            Mutex mMutex;
            ThreadPool* mThreadPool;
            unsigned int mCompletedJobs;
            bool mAllJobsAreAdded;
            bool mAllJobsCompleted;
            bool mStarted;
            List<ThreadJob*> mJobs;
            List<ThreadJobCollection*> mStartOnceCompleted;
    };

    /*!
     * \return The number of jobs in this collection, see \ref addJob.
     */
    inline unsigned int ThreadJobCollection::getJobsInCollectionCount() const {
        return mJobs.size();
    }

    /*!
     * \return TRUE when all jobs have been added to the collection (i.e. no
     * jobs can be added anymore), otherwise FALSE. See also \ref
     * setAllJobsAreAdded and \ref resetCompleted.
     */
    inline bool ThreadJobCollection::getAllJobsAreAdded() const {
        return mAllJobsAreAdded;
    }

    /*!
     * \return TRUE if all jobs in this collection (see \ref addJob and \ref
     * getJobsInCollectionCount) have been completed.
     *
     * See also \ref allJobsCompleted.
     */
    inline bool ThreadJobCollection::getAllJobsCompleted() const {
        return mAllJobsCompleted;
    }


}



#endif
/*
 * vim: et sw=4 ts=4
 */
