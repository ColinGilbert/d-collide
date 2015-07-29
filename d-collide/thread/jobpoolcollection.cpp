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


#include "jobpoolcollection.h"
#include "jobpool.h"
#include "debug.h"
#include "threadpool.h" // ThreadJob

#include <iostream>


namespace dcollide {
    JobPoolCollection::JobPoolCollection() {
        mUnprocessedJobsCount = 0;
        mCompletedJobsCount = 0;
        mPendingJobsCount = 0;

        addJobPool(""); // default job pool
    }

    JobPoolCollection::~JobPoolCollection() {
        mMutex.lock();
        for (std::vector<JobPool*>::iterator it = mJobPools.begin(); it != mJobPools.end(); ++it) {
            delete *it;
        }
        mJobPools.clear();
        mMutex.unlock();

        mUnprocessedJobsCount = 0;
        mCompletedJobsCount = 0;
        mPendingJobsCount = 0;
    }

    /*!
     * Add a new job pool to the this class and return the index of the job
     * pool.
     *
     * \param name A unique string identifying the job pool (an empty is always
     * the default job pool). This parameter is meant to make sure that a single
     * class won't create multiple job pools for the same task (which would be
     * unintended), even if multiple objects of that class are created. If the
     * same name is used twice with this method, the same index will be returned
     * both times (and no new pool is added the second time).
     */
    unsigned int JobPoolCollection::addJobPool(const std::string& name) {
        unsigned int index = 0;
        for (std::vector<JobPool*>::iterator it = mJobPools.begin(); it != mJobPools.end(); ++it) {
            if ((*it)->getName() == name) {
                return index;
            }
            index++;
        }
        MutexLocker lock(&mMutex);
        index = mJobPools.size();
        mJobPools.push_back(new JobPool(name));
        return index;
    }

    /*!
     * \return The number of job pools available, see \ref
     * addJobPool. This does not say anything on whether there are any jobs in
     * the pool!
     */
    unsigned int JobPoolCollection::getJobPoolCount() const {
        MutexLocker lock(&mMutex);
        return mJobPools.size();
    }

    /*!
     * \return The index of the job pool \name (see \ref addJobPool) or 0 if the
     * pool was not found (0 is the default job pool).
     */
    unsigned int JobPoolCollection::getJobPoolIndex(const std::string& name) const {
        MutexLocker lock(&mMutex);
        unsigned int index = 0;
        for (std::vector<JobPool*>::const_iterator it = mJobPools.begin(); it != mJobPools.end(); ++it) {
            if ((*it)->getName() == name) {
                return index;
            }
            index++;
        }
        return 0;
    }

    /*!
     * Like \ref addJob, but adds multiple jobs at once.
     *
     * See \ref ThreadPool::addJob and \ref ThreadPool::addJobs.
     *
     * This method takes ownership of the given jobs.
     *
     * This method is thread safe.
     */
    void JobPoolCollection::addJobs(const std::list<ThreadJob*>& jobs) {
        MutexLocker lock(&mMutex);
        for (std::list<ThreadJob*>::const_iterator it = jobs.begin(); it != jobs.end(); ++it) {
            ThreadJob* job = *it;
            unsigned int jobPool = job->getJobPoolIndex();
            if (jobPool >= mJobPools.size()) {
                throw ArrayIndexOutOfBoundsException("std::vector<JobPool*> mJobPools", mJobPools.size(), jobPool);
            }
            JobPool* pool = mJobPools[jobPool];
            pool->addJob(*it);
            (*it)->setJobPoolIndex(jobPool);
            mUnprocessedJobsCount++;
        }
    }

    /*!
     * See \ref ThreadPool::addJob and \ref ThreadPool::addJobs.
     *
     * This method takes ownership of the given job.
     *
     * This method is thread safe.
     */
    void JobPoolCollection::addJob(ThreadJob* job) {
        MutexLocker lock(&mMutex);
        unsigned int jobPool = job->getJobPoolIndex();
        if (jobPool >= mJobPools.size()) {
            throw ArrayIndexOutOfBoundsException("std::vector<JobPool*> mJobPools", mJobPools.size(), jobPool);
        }
        JobPool* pool = mJobPools[jobPool];
        pool->addJob(job);
        job->setJobPoolIndex(jobPool);
        mUnprocessedJobsCount++;
    }

    /*!
     * See \ref ThreadPool::addCompletedJob.
     *
     * This method takes ownership of the given job.
     *
     * This method is thread safe.
     */
    void JobPoolCollection::addCompletedJob(ThreadJob* job) {
        MutexLocker lock(&mMutex);
        JobPool* pool = mJobPools[job->getJobPoolIndex()];
        pool->addCompletedJob(job);
        mCompletedJobsCount++;
    }


    void JobPoolCollection::completePendingJob(ThreadJob* job) {
        MutexLocker lock(&mMutex);
        if (mJobPools[job->getJobPoolIndex()]->completePendingJob(job)) {
            mPendingJobsCount--;
            mCompletedJobsCount++;
        }
    }

    bool JobPoolCollection::hasUnprocessedJobs() const {
        // AB: this method is not thread safe, however it does not need to be!
        //     -> if mUnprocessedJobsCount is changed while we compare it, it
        //        might make our result invalid, however the same is true if it
        //        is changed right after the mutex is unlocked (which may still happen
        //        if we use a mutex)
        //        --> we can safe a mutex lock without losing anything.
        if (mUnprocessedJobsCount == 0) {
            return false;
        }
        return true;
    }

    bool JobPoolCollection::hasCompletedJobs() const {
        // AB: this method is not thread safe, however it does not need to be!
        //     the same argument as for hasUnprocessedJobs() holds here:
        //     mCompletedJobsCount may be changed right after unlocking the
        //     mutex, so the mutex won't make the result of this method any more
        //     correct.
        if (mCompletedJobsCount == 0) {
            return false;
        }
        return true;
    }

    bool JobPoolCollection::hasUnprocessedJobs(unsigned int jobPool) const {
        // AB: do we need this mutex?
        MutexLocker lock(&mMutex);
        if (jobPool >= mJobPools.size()) {
            throw ArrayIndexOutOfBoundsException("std::vector<JobPool*> mJobPools", mJobPools.size(), jobPool);
        }
        JobPool* pool = mJobPools[jobPool];
        return pool->hasUnprocessedJobs();
    }

    bool JobPoolCollection::hasCompletedJobs(unsigned int jobPool) const {
        // AB: do we need this mutex?
        MutexLocker lock(&mMutex);
        if (jobPool >= mJobPools.size()) {
            throw ArrayIndexOutOfBoundsException("std::vector<JobPool*> mJobPools", mJobPools.size(), jobPool);
        }
        JobPool* pool = mJobPools[jobPool];
        return pool->hasCompletedJobs();
    }

    bool JobPoolCollection::hasWorkLeft() const {
        // AB: this method is not thread safe, however it does not need to be!
        //     the same argument as for hasUnprocessedJobs() holds here:
        //     mUnprocessedJobsCount or mPendingJobsCount
        //     may be changed right after unlocking the
        //     mutex, so the mutex won't make the result of this method any more
        //     correct.
        if (mUnprocessedJobsCount > 0 || mPendingJobsCount > 0) {
            return true;
        }
        return false;
    }

    bool JobPoolCollection::hasWorkLeft(unsigned int jobPool) const {
        MutexLocker lock(&mMutex);
        if (jobPool >= mJobPools.size()) {
            throw ArrayIndexOutOfBoundsException("std::vector<JobPool*> mJobPools", mJobPools.size(), jobPool);
        }
        JobPool* pool = mJobPools[jobPool];
        return pool->hasWorkLeft();
    }

    unsigned int JobPoolCollection::getCompletedJobsCount() const {
        return mCompletedJobsCount;
    }


    /*!
     * This method is used by \ref WorkerThread to get something to do (through
     * \ref ThreadPool).
     *
     * The returned jobs is added to the pending list, see \ref
     * JobPool::retrieveUnprocessedJob
     *
     * The caller must not delete the job.
     *
     * \return NULL if unprocessed jobs are available, otherwise one of the
     * unprocessed jobs (see \ref addJobs).
     */
    // TODO: jobPool parameter? priorities? or just go through all pools
    ThreadJob* JobPoolCollection::retrieveUnprocessedJob() {
        MutexLocker lock(&mMutex);
        for (unsigned int poolIndex = 0; poolIndex < mJobPools.size(); poolIndex++) {
            JobPool* pool = mJobPools[poolIndex];
            if (!pool->hasUnprocessedJobs()) {
                continue;
            }

            ThreadJob* job = pool->retrieveUnprocessedJob();
            if (job) {
                mUnprocessedJobsCount--;
                mPendingJobsCount++;
                return job;
            }
        }
        return 0;
    }

    ThreadJob* JobPoolCollection::retrieveCompletedJob() {
        MutexLocker lock(&mMutex);
        if (mCompletedJobsCount == 0) {
            return 0;
        }

        ThreadJob* job = 0;
        for (std::vector<JobPool*>::iterator it = mJobPools.begin(); it != mJobPools.end(); ++it) {
            job = (*it)->retrieveCompletedJob();
            if (job) {
                mCompletedJobsCount--;
                return job;
            }
        }

        return 0;
    }

    void JobPoolCollection::retrieveCompletedJobs(std::list<ThreadJob*>* jobs) {
        MutexLocker lock(&mMutex);
        if (mCompletedJobsCount == 0) {
            return;
        }

        for (std::vector<JobPool*>::iterator it = mJobPools.begin(); it != mJobPools.end(); ++it) {
            (*it)->retrieveCompletedJobs(jobs);
        }
        mCompletedJobsCount = 0;
    }

    ThreadJob* JobPoolCollection::retrieveCompletedJobFromPool(unsigned int jobPool) {
        MutexLocker lock(&mMutex);
        if (jobPool >= mJobPools.size()) {
            throw ArrayIndexOutOfBoundsException("std::vector<JobPool*> mJobPools", mJobPools.size(), jobPool);
        }

        JobPool* pool = mJobPools[jobPool];
        ThreadJob* job = pool->retrieveCompletedJob();
        if (job) {
            mCompletedJobsCount--;
        }
        return job;
    }

    void JobPoolCollection::retrieveCompletedJobsFromPool(std::list<ThreadJob*>* jobs, unsigned int jobPool) {
        MutexLocker lock(&mMutex);
        if (jobPool >= mJobPools.size()) {
            throw ArrayIndexOutOfBoundsException("std::vector<JobPool*> mJobPools", mJobPools.size(), jobPool);
        }
        JobPool* pool = mJobPools[jobPool];
        unsigned int count = pool->getCompletedJobsCount();
        pool->retrieveCompletedJobs(jobs);
        mCompletedJobsCount -= count;
    }
}

/*
 * vim: et sw=4 ts=4
 */
