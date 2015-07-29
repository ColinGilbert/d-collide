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


#ifndef DCOLLIDE_THREADPOOL_H
#define DCOLLIDE_THREADPOOL_H

#include "thread.h"

#include <vector>
#include <list>

#include "dcollide-config.h"

#ifdef DCOLLIDE_USE_THREADS
 #include <pthread.h>
 #include <errno.h>
#endif // DCOLLIDE_USE_THREADS


namespace dcollide {
    class ThreadPool;
    class JobPool;
    class JobPoolCollection;
    class ThreadJobCollection;

#ifdef DCOLLIDE_USE_THREADS
    /*!
     * \brief Thread implementation that executes \ref ThreadJob objects
     *
     * Used internally by \ref ThreadPool. This class creates a thread and
     * processes \ref ThreadJob objects from \ref
     * WorkerPool::retrieveUnprocessedJob until there are no jobs left or until
     * \ref getWantQuit returns TRUE.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class WorkerThread : public Thread {
        public:
            explicit WorkerThread(ThreadPool* pool);
            virtual ~WorkerThread();

            void setPoolMutexAndCondition(pthread_mutex_t* poolMutex, pthread_cond_t* continueThreadCondition, pthread_cond_t* signalThreadIsIdle);
            void setWantQuit(bool);
            bool getWantQuit() const;

        protected:
            virtual void run();

        private:
            bool mWantQuit;
            ThreadPool* mThreadPool;
            pthread_mutex_t* mPoolMutex;
            pthread_cond_t* mContinueThreadCondition;
            pthread_cond_t* mSignalThreadIsIdle;
    };
#endif // DCOLLIDE_USE_THREADS


    // note: do NOT add a MyThreadPool pointer to this class!
    //       it may be helpful sometimes (e.g. to create a new Job that uses the
    //       results of this job), but it could easily lead to race conditions:
    //       imagine code like this
    //         Job* = new Job(data);
    //         workerPool()->addJob(job);
    //         mResult = data->isFinished;
    //       i.e. the job still uses some data, that may also be used by the new
    //       job. Correct code would move the last line to _before_ the job was
    //       created, however in complex code this may easily be forgotten (or
    //       the data dependencies may be _much_ less obvious). Now if in the
    //       code above the new job does something with data (e.g. deletes it),
    //       the mResult=data->isFinished; line is working incorrectly.
    //
    //       instead, if we want Jobs to create new Jobs, we may add some
    //       "postprocessing" phase, which is executed _after_ run() has
    //       completed. there we could provide a MyThreadPool* parameter, so
    //       that jobs can be added.

    /*!
     * \brief A job for a \ref WorkerThread
     *
     * This class is the main class for implementing multi-threading in
     * d-collide: it represents a job, i.e. some task that should be performed
     * in a separate threads.
     *
     * A job should be added to the \ref ThreadPool and will be executed
     * automatically (i.e. the \ref run method of the job will be called).
     *
     * You may want to use \ref getType to identify the type of a completed job
     * in order to read and use the results of the job.
     *
     * Note that jobs currently do not know any kind of dependencies: it is
     * completely up to the scheduling algorithms (i.e. practically random)
     * in which order the jobs are executed. Do not add a job to the \ref
     * WorkerPool unless you are sure all of its dependencies are already met.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class ThreadJob {
        public:
            // AB: strictly speaking this does not belong here: it's a property
            // of the derived classes, not of this class.
            // however having it at a central place is convenient.
            /*!
             * Enum for the \ref ThreadJob::getType values. Note that these
             * values are not used by \ref ThreadJob or \ref ThreadPool itself,
             * only by the calling methods.
             *
             * These values may help to identify the type of a completed job
             * (and consequently cast it to the correct subclass and read the
             * results).
             */
            enum ThreadJobType {
                /*!
                 * Default job type. All derived classes that do not reimplement
                 * \ref getType have this type.
                 */
                THREAD_JOB_TYPE_GENERIC = 0,
                /*!
                 * Indicates that the job is a \ref PipelineThreadJob
                 */
                THREAD_JOB_TYPE_PIPELINE
            };

        public:
            explicit ThreadJob(unsigned int jobPool = 0);
            virtual ~ThreadJob();

            void setDeleteAfterCompletionHint(bool del);
            inline bool getDeleteAfterCompletionHint() const;

            unsigned int getJobPoolIndex() const;

            void setJobCollection(ThreadJobCollection* collection);
            inline ThreadJobCollection* getJobCollection() const;

            virtual int getType() const;

            /*!
             * Is called by a thread from the thread pool to execute this job.
             * This method is, where you should implement your job.
             */
            virtual void run() = 0;


            void setJobPoolIndex(unsigned int index); // internal do not use directly!!

        private:
            unsigned int mJobPoolIndex;
            ThreadJobCollection* mJobCollection;
            bool mDeleteAfterCompletionHint;
    };



    /*!
     * \brief A pool of several threads which execute \ref ThreadJob tasks.
     *
     * A thread pool provides a fixed number of "worker threads" that are meant
     * to process jobs until there are no more jobs left. For this the class
     * provides \ref addJob and \ref addJobs to add objects of \ref ThreadJob
     * derived classes (implement your job there!) to the pool.
     *
     * The jobs must not have any dependencies on each other currently! It is
     * not guaranteed that they are executed in any particular order.
     *
     * Once the jobs have been processed, the \ref WorkerThread places the
     * completed job back into the thread pool and the results can then be
     * retrieved using \ref retrieveCompletedJob.
     *
     * By using \ref waitForCompletion you can make sure that ALL jobs that you
     * added to the pool have been completed.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class ThreadPool {
        public:
            explicit ThreadPool(int threads);
            ~ThreadPool();

            void changeThreadCount(int threadCount);
            void setDisableThreads(bool disable);
            bool getDisableThreads() const;

            unsigned int addJobPool(const std::string& name);
            unsigned int getJobPoolCount() const;
            unsigned int getJobPoolIndex(const std::string& name) const;

            void addJobs(const std::list<ThreadJob*>& jobs);
            void addJob(ThreadJob* job);

            unsigned int getThreadCount() const;

            void completePendingJob(ThreadJob* job);

            void waitForCompletion(int waitForJobCount = 0);

            bool hasCompletedJobs() const;
            bool hasCompletedJobsInPool(unsigned int jobPool) const;
            bool hasWorkLeft() const;
            bool hasWorkLeftInPool(unsigned int jobPool) const;

            bool hasUnprocessedJobs() const; // internal!

            ThreadJob* retrieveCompletedJob();

            void retrieveCompletedJobs(std::list<ThreadJob*>* jobs);
            ThreadJob* retrieveUnprocessedJob();

            void increaseThreadIdleCounter();

        protected:
            bool hasBusyThreads();
            void createThreads(int threads);
            bool quitThreads();

        private:
            unsigned int mThreadsIdleCounter;
            unsigned int mThreadCount;
#ifdef DCOLLIDE_USE_THREADS
            std::vector<WorkerThread*> mThreads;
#endif
            int mWaitForJobCount;
            bool mDisableThreads;

            JobPoolCollection* mJobPoolCollection;


#ifdef DCOLLIDE_USE_THREADS
            // AB: the pthread_cond_wait() will lock the mutex automatically, so
            // we should not use the Mutex class
            pthread_mutex_t mPoolMutex;
            pthread_cond_t mContinueThreadCondition;
            pthread_cond_t mSignalThreadIsIdle;
#endif
    };



    /*!
     * \return The job pool index (see \ref ThreadPool::addJobPool) this job is
     * in (or intended to go to, if the job was not yet added to any job pool).
     */
    inline unsigned int ThreadJob::getJobPoolIndex() const {
        return mJobPoolIndex;
    }

    /*!
     * \return The \ref ThreadJobCollection this job belongs to or 0 if it does
     * not belong to any collection.
     */
    inline ThreadJobCollection* ThreadJob::getJobCollection() const {
        return mJobCollection;
    }

    /*!
     * The "delete after completion" hint is a boolean value that specifies
     * whether this job wants to be deleted (TRUE by default) after it has
     * completed its calculations and results have been read. Normally this is
     * intended and therefore the default should be kept, however if you plan to
     * re-use the ThreadJob object, you may want to set this hint to FALSE, so
     * that the code that retrieves the jobs from the \ref ThreadPool will not
     * delete the pointer afterwards.
     *
     * Note that this value is \em not used by ThreadJob or \ref ThreadPool,
     * this value is only a hint to the code that call \ref
     * ThreadPool::retrieveCompletedJobs. Whether it honors that hint can not be
     * controlled by the thread framework.
     *
     * \return The "delete after completion" hint
     */
    inline bool ThreadJob::getDeleteAfterCompletionHint() const {
        return mDeleteAfterCompletionHint;
    }

}



#endif
/*
 * vim: et sw=4 ts=4
 */
