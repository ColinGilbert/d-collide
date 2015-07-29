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


#include "jobpool.h"
#include "datatypes/list.h"
#include "debug.h"
#include "threadpool.h" // ThreadJob

#include <iostream>


namespace dcollide {
    JobPool::JobPool(const std::string& name) {
        mName = name;
    }

    JobPool::~JobPool() {
        while (!mUnprocessedJobs.empty()) {
            delete mUnprocessedJobs.front();
            mUnprocessedJobs.pop_front();
        }
        while (!mPendingJobs.empty()) {
            delete mPendingJobs.front();
            mPendingJobs.pop_front();
        }
        while (!mCompletedJobs.empty()) {
            delete mCompletedJobs.front();
            mCompletedJobs.pop_front();
        }
    }

    /*!
     * See \ref ThreadPool::addJob and \ref ThreadPool::addJobs.
     *
     * This method takes ownership of the given job.
     *
     * WARNING: this method is NOT thread safe. \ref ThreadPool takes care of
     * that!
     */
    void JobPool::addJob(ThreadJob* job) {
        mUnprocessedJobs.push_back(job);
    }

    /*!
     * See \ref ThreadPool::addCompletedJob.
     *
     * This method takes ownership of the given job.
     *
     * WARNING: this method is NOT thread safe. \ref ThreadPool takes care of
     * that!
     */
    void JobPool::addCompletedJob(ThreadJob* job) {
        mCompletedJobs.push_back(job);
        mCompletedJobsCount++;
    }

    void JobPool::addPendingJob(ThreadJob* job) {
        mPendingJobs.push_back(job);
    }

    bool JobPool::completePendingJob(ThreadJob* job) {
#if USE_LIST_POOL
        ListNode<ThreadJob*>* node;
        for (node = mPendingJobs.getFirstNode(); node; node = node->mNext) {
            if (node->mData == job) {
                break;
            }
        }
        if (node == 0) {
            std::cerr << dc_funcinfo << "Cannot find job in pending list" << std::endl;
            delete job; // we claim we take ownership in addPendingJob()
            return false;
        }
        mPendingJobs.erase(node);
#else
        std::list<ThreadJob*>::iterator it;
        for (it = mPendingJobs.begin(); it != mPendingJobs.end(); ++it) {
            if ((*it) == job) {
                break;
            }
        }
        if (it == mPendingJobs.end()) {
            std::cerr << dc_funcinfo << "Cannot find job in pending list" << std::endl;
            delete job; // we claim we take ownership in addPendingJob()
            return false;
        }
        mPendingJobs.erase(it);
#endif
        addCompletedJob(job);

        return true;
    }

    /*!
     * Retrieve an unprocessed job and add it to the pending job list (see \ref
     * addPendingJob)
     *
     * \return NULL if no unprocessed job was left, otherwise a pointer to the
     * job. This class KEEPS ownership on the job! The job is stored in the
     * pending list.
     */
    ThreadJob* JobPool::retrieveUnprocessedJob() {
        if (mUnprocessedJobs.empty()) {
            return 0;
        }
        ThreadJob* job = mUnprocessedJobs.front();
        mUnprocessedJobs.pop_front();

        addPendingJob(job);

        return job;
    }

    ThreadJob* JobPool::retrieveCompletedJob() {
        if (mCompletedJobs.empty()) {
            return 0;
        }
        ThreadJob* job = mCompletedJobs.front();
        mCompletedJobs.pop_front();
        mCompletedJobsCount--;
        return job;
    }

    void JobPool::retrieveCompletedJobs(std::list<ThreadJob*>* jobs) {
        if (mCompletedJobs.empty()) {
            return;
        }
#if USE_LIST_POOL
#warning TODO
        while (!mCompletedJobs.empty()) {
            jobs->push_back(mCompletedJobs.front());
            mCompletedJobs.pop_front();
        }
#else
        jobs->splice(jobs->end(), mCompletedJobs);
#endif
        mCompletedJobsCount  = 0;
    }
}

/*
 * vim: et sw=4 ts=4
 */
