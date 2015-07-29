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


#ifndef DCOLLIDE_JOBPOOL_H
#define DCOLLIDE_JOBPOOL_H

#include <list>
#include <string>

#include "dcollide-config.h"

#define USE_LIST_POOL 0

#if USE_LIST_POOL
#include "datatypes/list.h"
#endif

namespace dcollide {
    class ThreadJob;

    /*!
     * \internal
     *
     * Helper class to \ref ThreadPool and \ref JobPoolCollection
     *
     * This class is NOT thread safe. \ref ThreadPool is meant to lock the
     * mutexes as required.
     */
    class JobPool {
        public:
            JobPool(const std::string& name);
            ~JobPool();

            inline const std::string& getName() const;

            void addJob(ThreadJob* job);
            void addCompletedJob(ThreadJob* job);
            void addPendingJob(ThreadJob* job);

            bool completePendingJob(ThreadJob* job);

            inline bool hasUnprocessedJobs() const;
            inline bool hasPendingJobs() const;
            inline bool hasCompletedJobs() const;
            inline bool hasWorkLeft() const;
            inline unsigned int getCompletedJobsCount() const;

            ThreadJob* retrieveUnprocessedJob();

            ThreadJob* retrieveCompletedJob();
            void retrieveCompletedJobs(std::list<ThreadJob*>* jobs);

        private:
            std::string mName;

#if USE_LIST_POOL
            List<ThreadJob*> mUnprocessedJobs;
            List<ThreadJob*> mCompletedJobs;
            List<ThreadJob*> mPendingJobs;
#else
            std::list<ThreadJob*> mUnprocessedJobs;
            std::list<ThreadJob*> mCompletedJobs;
            std::list<ThreadJob*> mPendingJobs;
#endif

            unsigned int mCompletedJobsCount;
    };

    inline const std::string& JobPool::getName() const {
        return mName;
    }

    inline bool JobPool::hasUnprocessedJobs() const {
        return !mUnprocessedJobs.empty();
    }

    inline bool JobPool::hasPendingJobs() const {
        return !mPendingJobs.empty();
    }

    inline bool JobPool::hasCompletedJobs() const {
        return !mCompletedJobs.empty();
    }

    inline bool JobPool::hasWorkLeft() const {
        if (!mUnprocessedJobs.empty() || !mPendingJobs.empty()) {
            return true;
        }
        return false;
    }

    inline unsigned int JobPool::getCompletedJobsCount() const {
#if USE_LIST_POOL
        return mCompletedJobs.size();
#else
        return mCompletedJobsCount;
#endif
    }

}


#endif
/*
 * vim: et sw=4 ts=4
 */
