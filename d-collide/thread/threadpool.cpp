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


#include "threadpool.h"
#include "jobpool.h"
#include "jobpoolcollection.h"
#include "timing.h"
#include "debug.h"
#include "../exceptions/exception.h"

#include <list>
#include <vector>
#include <iostream>

#define MINIMIZE_MUTEX_USAGE 1


namespace dcollide {
#ifdef DCOLLIDE_USE_THREADS
    WorkerThread::WorkerThread(ThreadPool* pool)
            : Thread() {
        mThreadPool = pool;
        mPoolMutex = 0;
        mContinueThreadCondition = 0;
        mSignalThreadIsIdle = 0;
        mWantQuit = false;
    }

    WorkerThread::~WorkerThread() {
    }

    void WorkerThread::setPoolMutexAndCondition(pthread_mutex_t* poolMutex, pthread_cond_t* continueThreadCondition, pthread_cond_t* signalThreadIsIdle) {
        mPoolMutex = poolMutex;
        mContinueThreadCondition = continueThreadCondition;
        mSignalThreadIsIdle = signalThreadIsIdle;
    }

    /*!
     * \internal
     *
     * This tells the thread to quit once the the process currently in
     * processing is completed.
     *
     * Do not call yourself, unless you know exactly what you're doing. The
     * ThreadPool assumes that all threads it created remain running. Quitting
     * one without proper synchronization etc. results in undefined behavior.
     */
    void WorkerThread::setWantQuit(bool q) {
        mWantQuit = q;
    }

    bool WorkerThread::getWantQuit() const {
        return mWantQuit;
    }

    void WorkerThread::run() {
        if (!mPoolMutex) {
            throw NullPointerException("member variable mPoolMutex");
        }
        if (!mContinueThreadCondition) {
            throw NullPointerException("member variable mContinueThreadCondition");
        }
        if (!mSignalThreadIsIdle) {
            throw NullPointerException("member variable mSignalThreadIsIdle");
        }
        if (!mThreadPool) {
            throw NullPointerException("member variable mThreadPool");
        }
        while (!getWantQuit()) {
            int code = pthread_mutex_lock(mPoolMutex);
            if (code != 0) {
                CHECK_MUTEX_LOCK_CODE(code);
                pthread_exit(0);
            }
            if (getWantQuit()) { // check again, in case it changed before we locked the mutex
                code = pthread_mutex_unlock(mPoolMutex);
                CHECK_MUTEX_UNLOCK_CODE(code);
                break;
            }
            mThreadPool->increaseThreadIdleCounter();
            pthread_cond_signal(mSignalThreadIsIdle);
            code = pthread_cond_wait(mContinueThreadCondition, mPoolMutex);
            CHECK_CONDITION_WAIT_CODE(code);
            code = pthread_mutex_unlock(mPoolMutex);
            CHECK_MUTEX_UNLOCK_CODE(code);

            while (mThreadPool->hasUnprocessedJobs() && !getWantQuit()) {
                ThreadJob* job = mThreadPool->retrieveUnprocessedJob();
                if (job) {
                    job->run();
                    mThreadPool->completePendingJob(job);
                }
            }
        }
    }
#endif // DCOLLIDE_USE_THREADS


    /*!
     * \param jobPool The job pool index of \ref ThreadPool that this job is
     * meant to go into. Use \ref ThreadPool::addJobPool to retrieve an index.
     * Using a random index (which does not match an actual job pool in \ref
     * ThreadPool) causes undefined behavior.
     */
    ThreadJob::ThreadJob(unsigned int jobPool) {
        mJobCollection = 0;
        mJobPoolIndex = jobPool;
        mDeleteAfterCompletionHint = true;
    }

    ThreadJob::~ThreadJob() {
    }

    /*!
     * Set the \ref ThreadJobCollection this job belongs to. Initially a job
     * belongs to no collection, once a collection has been set, it cannot be
     * changed anymore!
     *
     * Calling this method twice with different collection parameters causes
     * an excpetion!
     */
    void ThreadJob::setJobCollection(ThreadJobCollection* collection) {
        if (mJobCollection && mJobCollection != collection) {
            throw Exception("Cannot set job collection: has already been set!");
        }
        mJobCollection = collection;
    }

    /*!
     * Set the "delete after completion" hint, see \ref
     * getDeleteAfterCompletionHint for details.
     *
     * Note that this a \em hint only, neither the ThreadJob, nor the \ref
     * ThreadPool will use this value!
     */
    void ThreadJob::setDeleteAfterCompletionHint(bool del) {
        mDeleteAfterCompletionHint = del;
    }

    /*!
     * The type of a ThreadJob may be used to identify a specify class type of a
     * job (e.g. as an alternative to dynamic_class). It is completely unused by
     * ThreadJob and \ref ThreadPool, it is up to the user to make use of it or
     * not.
     *
     * \return The type of this job. The default implementation always returns
     * 0.
     */
    int ThreadJob::getType() const {
        return 0;
    }

    /*!
     * \internal
     *
     * Internal use only. Do not use directly !!!
     *
     * This method is used by \ref JobPoolCollection to set the index of the
     * jobpool that was actually used for this job.
     */
    void ThreadJob::setJobPoolIndex(unsigned int index) {
        mJobPoolIndex = index;
    }



    /*!
     * Creates a ThreadPool object with \p threads worker threads. You should
     * always create at least one worker thread, otherwise the threadpool will
     * behave as if threading was disabled (see \ref setDisableThreads).
     */
    ThreadPool::ThreadPool(int threads) {
        threads = std::max(0, threads);
        mThreadCount = (unsigned int)threads;
        mThreadsIdleCounter = mThreadCount;
        mWaitForJobCount = 0;
        mDisableThreads = false;
        mJobPoolCollection = new JobPoolCollection();


#ifdef DCOLLIDE_USE_THREADS
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        int code = pthread_mutex_init(&mPoolMutex, &attr);
        if (code != 0) {
            throw ThreadInitializeException("could not initialize pool mutex", code);
        }
        pthread_mutexattr_destroy(&attr);

        code = pthread_cond_init(&mContinueThreadCondition, NULL);
        if (code != 0) {
            throw ThreadInitializeException("could not initialize mContinueThreadCondition", code);
        }
        code = pthread_cond_init(&mSignalThreadIsIdle, NULL);
        if (code != 0) {
            throw ThreadInitializeException("could not initialize mSignalThreadIsIdle condition", code);
        }
#else // DCOLLIDE_USE_THREADS
        mDisableThreads = true;
        mThreadCount = 0;
#endif // DCOLLIDE_USE_THREADS

        createThreads(threads);
    }

    ThreadPool::~ThreadPool() {
        quitThreads();

#ifdef DCOLLIDE_USE_THREADS
        int code = pthread_mutex_destroy(&mPoolMutex);
        if (code != 0) {
            throw ThreadDestroyException("could not destroy pool mutex", code);
        }
        code = pthread_cond_destroy(&mContinueThreadCondition);
        if (code != 0) {
            throw ThreadDestroyException("could not destroy mContinueThreadCondition", code);
        }
        code = pthread_cond_destroy(&mSignalThreadIsIdle);
        if (code != 0) {
            throw ThreadDestroyException("could not destroy mSignalThreadIsIdle", code);
        }
#endif // DCOLLIDE_USE_THREADS


        delete mJobPoolCollection;
    }

    /*!
     * Change the number of threads to \p threadCount.
     *
     * This method can be called \em only if \em no jobs (neither unprocessed
     * nor pending nor completed) are left in the pool and \em only if \em all
     * threads are idle (no jobs left should imply that all threads are idle).
     * If this condition is not ensured, this method will throw an exception and
     * not change the number of threads.
     *
     * This method works by quitting all existing threads and re-creating \p
     * threadCount threads, so for performance reasons this method should not be
     * called often. The number of threads is meant to be a constant number once
     * set.
     *
     * This method is meant primarily for debugging purposes, to experimentally
     * find the optimal number of threads in an application.
     */
    void ThreadPool::changeThreadCount(int threadCount) {
        threadCount = std::max(0, threadCount);
#ifdef DCOLLIDE_USE_THREADS
        int code = pthread_mutex_lock(&mPoolMutex);
        CHECK_MUTEX_LOCK_CODE(code);

        if (mThreadsIdleCounter != mThreadCount) {
            code = pthread_mutex_unlock(&mPoolMutex);
            CHECK_MUTEX_UNLOCK_CODE(code);
            throw Exception("not all threads idle. cannot change thread count.");
        }
        if (mJobPoolCollection->hasWorkLeft() || mJobPoolCollection->hasCompletedJobs()) {
            // AB: we consider completed jobs in the collection an error, too:
            //     this indicates that we are trying to change the thread count
            //     during a collision detection run, which is not a wise idea.
            //     so for sanity this is an error.
            //     (unprocessed or pending jobs are an error anyway)
            code = pthread_mutex_unlock(&mPoolMutex);
            CHECK_MUTEX_UNLOCK_CODE(code);
            throw Exception("still jobs in the jobpoolcollection left. cannot change thread count");
        }

        quitThreads();

        mThreadCount = (unsigned int)threadCount;
        mThreadsIdleCounter = mThreadCount;
        createThreads(threadCount);

        code = pthread_mutex_unlock(&mPoolMutex);
        CHECK_MUTEX_UNLOCK_CODE(code);
#endif // DCOLLIDE_USE_THREADS
    }

    /*!
     * Disable threads temporarily. No threads are deleted (or created when
     * reenabling threads), therefore this method is very fast. It is however
     * meant primarily for debugging: to compare the efficiency of threads
     * versus non-threads. If you want to disable threads permanently it is
     * recommended to request 0 threads in the \ref ThreadPool constructor or to
     * use \ref changeThreadCount.
     *
     * This works dependable \em only if there are currently \em no jobs in the
     * pool. You may want to call \ref waitForCompletion to ensure this.
     *
     * When threads are disabled, the jobs are processed only when \ref
     * waitForCompletion is called and they are called by that method in a loop
     * (i.e. by the "main thread").
     */
    void ThreadPool::setDisableThreads(bool disable) {
#ifdef DCOLLIDE_USE_THREADS
        mDisableThreads = disable;
        if (!mDisableThreads) {
            pthread_mutex_lock(&mPoolMutex);
            if (!mDisableThreads) {
                if (hasUnprocessedJobs()) {
                    pthread_cond_broadcast(&mContinueThreadCondition);
                    mThreadsIdleCounter = 0;
                }
            }
            pthread_mutex_unlock(&mPoolMutex);
        }
#endif // DCOLLIDE_USE_THREADS
    }

    /*!
     * \return Whether threads are currently disabled or not. See \ref
     * setDisableThreads for details.
     */
    bool ThreadPool::getDisableThreads() const {
        return mDisableThreads;
    }

    /*!
     * Add a new job pool to the this class and return the index of the job
     * pool.
     *
     * This "thread pool" also provides "job pools": every job has a "type"
     * assigned (by default 0) which defines in which "job pool" it is added to.
     * This "type" is an index in the job pools of this class - every "job
     * pool", i.e. every job type must be added to this class before it can be
     * used.
     *
     * All "job pools" are essentially equal, with one exception: the threads of
     * this thread pool can have priorities assigned which define which jobs
     * they process first. If no job of the first priority is available, the
     * threads will choose a thread of the second priority and so on.
     *
     *
     * Note: many methods in this class depend on the number of job pools - the
     * more job pools, the slower this class (because e.g. all pools have to be
     * iterated to find an unprocessed job). You are meant to keep the number of
     * job pools low (e.g. around 5-10).
     *
     * \param name A unique string identifying the job pool (an empty is always
     * the default job pool). This parameter is meant to make sure that a single
     * class won't create multiple job pools for the same task (which would be
     * unintended), even if multiple objects of that class are created. If the
     * same name is used twice with this method, the same index will be returned
     * both times (and no new pool is added the second time).
     */
    unsigned int ThreadPool::addJobPool(const std::string& name) {
        return mJobPoolCollection->addJobPool(name);
    }

    /*!
     * \return The number of job pools available, see \ref
     * addJobPool. This does not say anything on whether there are any jobs in
     * the pool!
     */
    unsigned int ThreadPool::getJobPoolCount() const {
        return mJobPoolCollection->getJobPoolCount();
    }

    /*!
     * \return The index of the job pool named \ref name (see \ref addJobPool).
     * An empty string is always the default pool.
     *
     * This method requires string comparisons and thus is considered to be
     * slow. Do NOT use this for all jobs - instead use this once in the class
     * that creates the jobs and cache the return value.
     */
    unsigned int ThreadPool::getJobPoolIndex(const std::string& name) const {
        return mJobPoolCollection->getJobPoolIndex(name);
    }

    void ThreadPool::createThreads(int threads) {
#ifdef DCOLLIDE_USE_THREADS
        if (threads <= 0) {
            // disable threading, use main thread only.
            mThreadCount = 0;
            return;
        }

        if (!mThreads.empty()) {
            throw ThreadInitializeException("threads already created.", 0);
        }

        mThreads.reserve(mThreadCount);
        for (unsigned int i = 0; i < mThreadCount; i++) {
            WorkerThread* t = new WorkerThread(this);
            mThreads.push_back(t);

            t->setPoolMutexAndCondition(&mPoolMutex, &mContinueThreadCondition, &mSignalThreadIsIdle);
        }

        for (unsigned int i = 0; i < mThreadCount; i++) {
            WorkerThread* t = mThreads[i];

            // should start && go to sleep immediately (or start processing
            // jobs)
            t->start(true);
        }
#endif // DCOLLIDE_USE_THREADS
    }

    bool ThreadPool::quitThreads() {
#ifdef DCOLLIDE_USE_THREADS
        // TODO: make sure only the main thread can call this

        if (hasUnprocessedJobs()) {
            throw ThreadDestroyException("cannot quit threads while there are unprocessed jobs left.", 0);
        }

        int code = pthread_mutex_lock(&mPoolMutex);
        CHECK_MUTEX_LOCK_CODE(code);
        for (std::vector<WorkerThread*>::iterator it = mThreads.begin(); it != mThreads.end(); ++it) {
            (*it)->setWantQuit(true);
        }
        pthread_cond_broadcast(&mContinueThreadCondition);
        code = pthread_mutex_unlock(&mPoolMutex);
        CHECK_MUTEX_UNLOCK_CODE(code);

        for (std::vector<WorkerThread*>::iterator it = mThreads.begin(); it != mThreads.end(); ++it) {
            (*it)->join();
        }
        for (std::vector<WorkerThread*>::iterator it = mThreads.begin(); it != mThreads.end(); ++it) {
            delete *it;
        }
        mThreads.clear();
#endif // DCOLLIDE_USE_THREADS
        return true;
    }

    /*!
     * Add jobs to the pool. The jobs are queued for execution by one of the
     * threads, but it is NOT guaranteed they are executed in any particular
     * order!
     *
     * I repeat: the jobs do NOT honor any dependencies!
     *
     * This class takes ownership of all pointers in the list.
     */
    void ThreadPool::addJobs(const std::list<ThreadJob*>& jobs) {
        // TODO: currently should be called from main thread only. check if it
        // can be called from worker threads, too!

        mJobPoolCollection->addJobs(jobs);

#ifdef DCOLLIDE_USE_THREADS
#ifndef MINIMIZE_MUTEX_USAGE
        pthread_mutex_lock(&mPoolMutex);
#endif
        if (!mDisableThreads) {
            pthread_cond_broadcast(&mContinueThreadCondition);
            mThreadsIdleCounter = 0;
        }
#ifndef MINIMIZE_MUTEX_USAGE
        pthread_mutex_unlock(&mPoolMutex);
#endif
#endif // DCOLLIDE_USE_THREADS
    }

    /*!
     * Behaves like \ref addJobs, but adds a single job only.
     */
    void ThreadPool::addJob(ThreadJob* job) {
        // TODO: currently should be called from main thread only. check if it
        // can be called from worker threads, too!

        mJobPoolCollection->addJob(job);

#ifdef DCOLLIDE_USE_THREADS
#ifndef MINIMIZE_MUTEX_USAGE
        pthread_mutex_lock(&mPoolMutex);
#endif
        if (!mDisableThreads) {
            pthread_cond_broadcast(&mContinueThreadCondition);
            mThreadsIdleCounter = 0;
        }
#ifndef MINIMIZE_MUTEX_USAGE
        pthread_mutex_unlock(&mPoolMutex);
#endif
#endif
    }

    /*!
     * Internal method used by \ref WorkerThread. This is called after a job has
     * been started (and thus is "pending", see \ref retrieveUnprocessedJob) to
     * notify the ThreadJob that this job has now been completed.
     */
    void ThreadPool::completePendingJob(ThreadJob* job) {
        mJobPoolCollection->completePendingJob(job);

#ifdef DCOLLIDE_USE_THREADS
        if ((int)mJobPoolCollection->getCompletedJobsCount() >= mWaitForJobCount) {
            pthread_cond_signal(&mSignalThreadIsIdle); // TODO: rename signal
        }
#endif // DCOLLIDE_USE_THREADS
    }

    /*!
     * \internal
     *
     * Used by \ref WorkerThread to signal the pool that it is now idle. Once
     * all threads are idel, the main thread wakes up from \ref
     * waitForCompletion again.
     *
     * DO NOT USE MANUALLY unless you know exactly what you're doing.
     *
     * The caller must lock the internal mutex properly.
     */
    void ThreadPool::increaseThreadIdleCounter() {
        mThreadsIdleCounter++;
    }

    /*!
     * \return TRUE if there are jobs available that have been completed, see
     * \ref retrieveJob.
     */
    bool ThreadPool::hasCompletedJobs() const {
        return mJobPoolCollection->hasCompletedJobs();
    }

    /*!
     * \return TRUE if there are jobs in \p jobPool available that have
     * been completed, see \ref retrieveJob.
     */
    bool ThreadPool::hasCompletedJobsInPool(unsigned int jobPool) const {
        return mJobPoolCollection->hasCompletedJobs(jobPool);
    }

    /*!
     * \internal
     *
     * WARNING: if this method returns TRUE, there may still be uncompleted jobs
     * left, which are currently being processed. Usually you should prefer \ref
     * hasWorkLeft to this method.
     *
     * This method is used internally by \ref WorkerThread to decide whether
     * there it can get something to do.
     */
    bool ThreadPool::hasUnprocessedJobs() const {
        return mJobPoolCollection->hasUnprocessedJobs();
    }

    bool ThreadPool::hasWorkLeft() const {
        return mJobPoolCollection->hasWorkLeft();
    }

    bool ThreadPool::hasWorkLeftInPool(unsigned int jobPool) const {
        return mJobPoolCollection->hasWorkLeft(jobPool);
    }

    unsigned int ThreadPool::getThreadCount() const {
        return mThreadCount;
    }

    /*!
     * Blocks and waits until all jobs have been completed by the worker
     * threads.
     *
     * \param waitForJobCount If greater than zero, this function will wait
     * until at least \p waitForJobCount jobs have been completed, otherwise (\p
     * waitForJobCount is less than or equal to zero) this method waits until
     * ALL jobs are completed. Note that this parameter has only an effect if
     * \ref getThreadCount is at least 1.
     *
     * In a unthreaded configuration, this method actually executes the jobs.
     */
    void ThreadPool::waitForCompletion(int waitForJobCount) {
        // TODO: make sure only the main thread can call this

        mWaitForJobCount = waitForJobCount;
#ifdef DCOLLIDE_USE_THREADS
        if (mThreadCount > 0 && !mDisableThreads) {
            pthread_mutex_lock(&mPoolMutex);
            while ((hasWorkLeft() || mThreadsIdleCounter != mThreadCount) &&
                        (mWaitForJobCount <= 0 || mJobPoolCollection->getCompletedJobsCount() < (unsigned int)mWaitForJobCount)) {

                pthread_cond_broadcast(&mContinueThreadCondition);
                mThreadsIdleCounter = 0;
                while (mThreadsIdleCounter < mThreadCount
                            && (mWaitForJobCount <= 0 || mJobPoolCollection->getCompletedJobsCount() < (unsigned int)mWaitForJobCount)) {
                    int code = pthread_cond_wait(&mSignalThreadIsIdle, &mPoolMutex);
                    CHECK_CONDITION_WAIT_CODE(code);
                }

            }
            pthread_mutex_unlock(&mPoolMutex);
        } else
#endif // DCOLLIDE_USE_THREADS
        {
            // TODO: waitForJobCount is ignored here
            while (hasWorkLeft()) {
                ThreadJob* job = retrieveUnprocessedJob();
                if (job) {
                    job->run();
                    completePendingJob(job);
                }
            }
        }
        mWaitForJobCount = 0;
//        std::cout << dc_funcinfo << "done" << std::endl;
    }

    /*!
     * \internal
     *
     * This method is used by \ref WorkerThread to get something to do.
     *
     * The caller must not delete the job.
     *
     * See also \ref JobPoolCollection::retrieveUnprocessedJob.
     *
     * \return NULL if unprocessed jobs are available, otherwise one of the
     * unprocessed jobs (see \ref addJobs).
     */
    ThreadJob* ThreadPool::retrieveUnprocessedJob() {
        // AB: make sure that if a thread has just been woken up by
        // pthread_cond_broadcast/signal(), that only that thread is allowed to
        // remove jobs from the pool.
#ifndef MINIMIZE_MUTEX_USAGE
        pthread_mutex_lock(&mPoolMutex);
#endif

        ThreadJob* job = mJobPoolCollection->retrieveUnprocessedJob();

#ifndef MINIMIZE_MUTEX_USAGE
        pthread_mutex_unlock(&mPoolMutex);
#endif

        return job;
    }

    /*!
     * This method transfers ownership of the returned job to the calling
     * method.
     *
     * \return NULL if no completed job available, otherwise one of the jobs
     * that have been completed already.
     */
    ThreadJob* ThreadPool::retrieveCompletedJob() {
        return mJobPoolCollection->retrieveCompletedJob();
    }

    void ThreadPool::retrieveCompletedJobs(std::list<ThreadJob*>* jobs) {
        mJobPoolCollection->retrieveCompletedJobs(jobs);
    }
}


/*
 * vim: et sw=4 ts=4
 */
