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


#include "threadjobcollection.h"
#include "threadpool.h"
#include "debug.h"
#include "../exceptions/exception.h"

namespace dcollide {
    ThreadJobCollection::ThreadJobCollection() {
        mThreadPool = 0;
        mCompletedJobs = 0;
        mAllJobsAreAdded = false;
        mAllJobsCompleted = false;
        mStarted = false;
    }

    ThreadJobCollection::~ThreadJobCollection() {
    }

    /*!
     * Start all jobs that have been added to this collection (see \ref addJob).
     * If a job gets added after this point, it will get started immediately
     * (unless \ref resetCompleted has been called in between, which also resets
     * the started flag).
     */
    void ThreadJobCollection::start() {
        if (mStarted) {
            return;
        }
        if (getAllJobsCompleted()) {
            throw Exception("Cannot start collection - all jobs already marked as completed");
        }
        mStarted = true;
        if (mJobs.empty()) {
            allJobsCompleted();
        } else {
            for (ListNode<ThreadJob*>* node = mJobs.getFirstNode(); node; node = node->getNext()) {
                startJob(node->getData());
            }
        }
    }

    /*!
     * Set the thread pool that will execute the jobs of this collection (see
     * \ref start)
     */
    void ThreadJobCollection::setPool(ThreadPool* pool) {
        mThreadPool = pool;
    }

    /*!
     * \return The thread pool that will execute the jobs of this collection.
     * See also \ref start and \ref setPool.
     *
     * Derived classes should normally \em not use this. In particular \ref
     * ThreadPool::addJob should not be used, instead use \ref
     * Pipeline::addJobForPhase.
     */
    ThreadPool* ThreadJobCollection::getPool() const {
        return mThreadPool;
    }

    /*!
     * Reset the collection, that is
     * \li Call \ref resetCompleted
     * \li Remove all previously added jobs (see \ref addJob) from this
     * collection
     */
    void ThreadJobCollection::resetCollection() {
        resetCompleted();
        mJobs.clear();
    }

    /*!
     * Reset all "job completed" information, in particular reset the completed
     * jobs counter to 0 and purges the "start once completed" list.
     * This method also sets the value of \ref
     * getAllJobsAreAdded back to FALSE.
     *
     * No jobs are removed from this collection by this method, only the
     * "completed" information and the "start once completed" list are
     * discarded.
     */
    void ThreadJobCollection::resetCompleted() {
        mAllJobsAreAdded = false;
        mAllJobsCompleted = false;
        mCompletedJobs = 0;
        mStarted = false;

        mStartOnceCompleted.clear();
    }

    /*!
     * Mark \p job as completed, in particular increase the internal "completed
     * jobs" counter. The caller must make sure this method is called at most
     * once per job, ays it is not ensured by this class for
     * performance reasons!
     *
     * See also \ref resetCompleted to reset the information set by this method.
     *
     * This method is thread safe.
     */
    void ThreadJobCollection::setJobCompleted(ThreadJob* job) {
        MutexLocker lock(&mMutex);
        if (job->getJobCollection() != this) {
            throw Exception("job does not belong to this job collection");
        }
        mCompletedJobs++;
        if (mCompletedJobs == getJobsInCollectionCount()) {
            allJobsCompleted();
        }
    }

    /*!
     * Internally called by \ref ThreadJob when the \ref
     * ThreadJob::getJobCollection is set. Do not call directly.
     *
     * Ownership of \p job is NOT taken, the pointer will not be deleted by this
     * class.
     *
     * A single job cannot be removed from a job collection, jobs can be removed
     * using \ref resetCollection only.
     */
    void ThreadJobCollection::addJob(ThreadJob* job) {
        if (getAllJobsAreAdded()) {
            throw Exception("ThreadJob::addJob() called after setAllJobsAreAdded() was called");
        }
        if (!job) {
            throw NullPointerException("job");
        }
        job->setJobCollection(this);
        mJobs.push_back(job);

        if (mStarted) {
            startJob(job);
        }
    }

    /*!
     * Notify this collection that no more jobs will be added (see \ref addJob)
     * to this collection. From this point on \ref allJobsCompleted might get
     * called (as soon as all jobs in the collection are completed).
     *
     * This flag can be reset using \ref resetCompleted
     */
    void ThreadJobCollection::setAllJobsAreAdded() {
        mAllJobsAreAdded = true;
        if (mCompletedJobs == getJobsInCollectionCount() && mCompletedJobs > 0) {
            allJobsCompleted();
        }
     }

    /*!
     * Called once all jobs in this collection have been completed, i.e.:
     * \li every job that was added using \ref addJob also called \ref setJobCollection
     * \li all jobs that belong to this collection have been added (see \ref setAllJobsAreAdded)
     *
     * Derived classes that reimplement this method \em must call the base
     * implementation!
     */
    void ThreadJobCollection::allJobsCompleted() {
        mAllJobsCompleted = true;
        for (ListNode<ThreadJobCollection*>* node = mStartOnceCompleted.getFirstNode(); node; node = node->getNext()) {
            node->getData()->start();
        }
    }

    /*!
     * Start \p collection once \ref allJobsCompleted of this collection is
     * called.
     */
    void ThreadJobCollection::addStartOnceCompleted(ThreadJobCollection* collection) {
        mStartOnceCompleted.push_back(collection);
    }

    /*!
     * Add \p job to the \ref ThreadPool, see \ref setPool.
     *
     * This method is NOT thread safe and assumes that it will always be called
     * from the main thread only.
     */
    void ThreadJobCollection::startJob(ThreadJob* job) {
        if (!mThreadPool) {
            throw NullPointerException("mThreadPool");
        }

        mThreadPool->addJob(job);
    }


    /*!
     * This static method iterates over all jobs in the \p jobs list and calls
     * \ref setJobCompleted for it, if the job has a job collection assigned.
     *
     * This method is meant to be called exactly once for each list of
     * (completed) jobs that
     * has been retrieved from the \ref ThreadPool, before further processing
     * the results.
     */
    void ThreadJobCollection::processCompletedJobs(const std::list<ThreadJob*>& jobs) {
        for (std::list<ThreadJob*>::const_iterator it = jobs.begin(); it != jobs.end(); ++it) {
            processCompletedJob(*it);
        }
    }

    /*!
     * Like \ref processCompletedJobs, but for a single job.
     */
    void ThreadJobCollection::processCompletedJob(ThreadJob* job) {
        if (job->getJobCollection()) {
            job->getJobCollection()->setJobCompleted(job);
        }
    }

    /*!
     * \return All jobs in this collection, i.e. that have been added using \ref
     * addJob
     */
    const List<ThreadJob*>& ThreadJobCollection::getJobs() const {
        return mJobs;
    }
}

/*
 * vim: et sw=4 ts=4
 */
