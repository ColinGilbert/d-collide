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


#include "thread.h"
#include "timing.h"
#include "debug.h"
#include "../exceptions/exception.h"

#include <list>
#include <vector>
#include <iostream>


// TODO: non-thread implementation
// * Mutex class is a dummy class
// * WorkerThread class is not needed at all
// * WorkerPool should run all the jobs itself in waitForCompletion()
#ifdef DCOLLIDE_USE_THREADS
namespace dcollide {
    Thread::Thread() {
        mIsRunning = false;
    }

    Thread::~Thread() {
    }

    /*!
     * Start the thread and execute its main loop, i.e. \ref run.
     */
    void Thread::start(bool joinable) {
        if (isRunning()) {
            throw ThreadException("thread already running");
        }
        mIsRunning = true;
        mIsJoinable = joinable;

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        if (joinable) {
            pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
        } else {
            pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        }

        int code = pthread_create(&mThread, NULL, &Thread::runThread, this);
        if (code != 0) {
            throw ThreadInitializeException("Thread creation failed.", code);
        }
        pthread_attr_destroy(&attr);
    }

    /*!
     * \internal
     * Given as parameter to pthread_create to actually start the thread. Calls
     * \ref run
     */
    void* Thread::runThread(void* arg)  {
        Thread* thread = static_cast<Thread*>(arg);
        thread->run();
        return 0;
    }

    /*!
     * \return TRUE if the thread is currently running, otherwise FALSE. See
     * also \ref start.
     */
    bool Thread::isRunning() const {
        return mIsRunning;
    }

    /*!
     * Makes the calling thread to wait for the termination of the called
     * thread.
     */
    int Thread::join() {
        if (!isRunning()) {
            return 0;
        }
        if (!mIsJoinable) {
            throw ThreadException("thread not created as joinable, cannot join");
        }

        int status;
        int code = pthread_join(mThread, (void**)&status);
        if (code != 0) {
            if (code == EINVAL) {
                throw ThreadJoinException("thread ID does not belong to joinable thread", code);
            } else if (code == ESRCH) {
                // this implementation apparently has a bug!
                throw ThreadJoinException("thread with specified ID not found (internal error)", code);
            } else if (code == EDEADLK) {
                throw ThreadJoinException("deadlock detected of ID belongs to calling thread!", code);
            }
        }
        return status;
    }


    /*!
     * Constructs a new mutex object.
     *
     * If mutex creation fails, the application is quit, due to a fatal error.
     *
     * See your favourite mutex tutorial for details on what a mutex is.
     */
    Mutex::Mutex() {
        int code = pthread_mutex_init(&mMutex, NULL);
        if ( code != 0) {
            throw ThreadInitializeException("pthread_mutex_init() failed", code);
        }
    }
    /*!
     * Destroy the mutex.
     *
     * WARNING: make sure the mutex is not locked at this point! Destroying a
     * locked mutex has undefined behavior!
     */
    Mutex::~Mutex() {
        int code = pthread_mutex_destroy(&mMutex);
        if (code != 0) {
            if (code == EBUSY) {
                throw ThreadDestroyException("failed to destroy mutex. Mutex is still locked!", code);
            } else if (code == EINVAL) {
                throw ThreadDestroyException("failed to destroy mutex. invalid mutex", code);
            } else {
                throw ThreadDestroyException("failed to destroy mutex.", code);
            }
        }
    }

    /*!
     * Locks the mutex. If you really need documentation for this, you probably
     * should get a decent mutex tutorial.
     *
     * Note that unless explicitly specified the mutex is NOT recursive, i.e.
     * locking the mutex twice in the same thread without unlocking first causes
     * a deadlock.
     *
     * (recursive mutexes are currently not supported by this class. please fix
     * the documentation if you notice that this has changed in the meanwhile).
     *
     * See also \ref unlock and \ref tryLock
     */
    void Mutex::lock() {
        int code = pthread_mutex_lock(&mMutex);
        if (code != 0) {
            CHECK_MUTEX_LOCK_CODE(code);
            return;
        }
    }

    /*!
     * Try to lock the mutex and return immediately (even if mutex could not be
     * acquired).
     *
     * This call does NOT block if the mutex is alread locked.
     *
     * \ret TRUE if the mutex has been acquired, otherwise FALSE.
     */
    bool Mutex::tryLock() {
        int code = pthread_mutex_trylock(&mMutex);
        if (code == EBUSY) {
            return false;
        }
        if (code != 0) {
            CHECK_MUTEX_LOCK_CODE(code);
            return false;
        }
        return true;
    }

    /*!
     * Unlock the mutex. Note that unlocking a mutex that has not been locked by
     * the thread that unlocks is an error!
     */
    void Mutex::unlock() {
        int code = pthread_mutex_unlock(&mMutex);
        if (code != 0) {
            CHECK_MUTEX_UNLOCK_CODE(code);
        }
    }


    /*!
     * Create and initialize a new ReadWriteLock object. The lock is NOT locked
     * initially.
     *
     * \see lock ReadLocker WriteLocker Mutex
     */
    ReadWriteLock::ReadWriteLock() {
        int code = pthread_rwlock_init(&mLock, NULL);
        if ( code != 0) {
            throw ThreadInitializeException("pthread_rwlock_init() failed.", code);
        }
    }

    /*!
     * Destruct the rwlock. Destroying a rwlock that is still locked is an
     * error and causes undefined behaviour.
     */
    ReadWriteLock::~ReadWriteLock() {
        int code = pthread_rwlock_destroy(&mLock);
        if (code != 0) {
            if (code == EBUSY) {
                throw ThreadDestroyException("pthread_rwlock_destroy(&mLock) failed. lock is still locked!", code);
            } else if (code == EINVAL) {
                throw ThreadDestroyException("pthread_rwlock_destroy(&mLock) failed. invalid lock", code);
            } else {
                throw ThreadDestroyException("pthread_rwlock_destroy(&mLock) failed.", code);
            }
        }
    }

    /*!
     * Lock the rwlock for reading. This method blocks only if a thread
     * currently holds a write lock: multiple threads may lock for reading at
     * the same time.
     *
     * See also \ref unlock and \ref lockForWrite
     */
    void ReadWriteLock::lockForRead() {
        int code = pthread_rwlock_rdlock(&mLock);
        if (code != 0) {
            CHECK_RWLOCK_RDLOCK_CODE(code);
            return;
        }
    }

    /*!
     * Lock the rwlock for writing. A write access is always exclusive, once
     * this method returns, the calling thread owns the lock and no other thread
     * does.
     *
     * See also \ref unlock and \ref lockForRead
     */
    void ReadWriteLock::lockForWrite() {
        int code = pthread_rwlock_rdlock(&mLock);
        if (code != 0) {
            CHECK_RWLOCK_WRLOCK_CODE(code);
            return;
        }
    }

    /*!
     * Unlock any previously locked (read or write) rwlock.
     */
    void ReadWriteLock::unlock() {
        int code = pthread_rwlock_unlock(&mLock);
        if (code != 0) {
            CHECK_RWLOCK_UNLOCK_CODE(code);
            return;
        }
    }

}

#endif // DCOLLIDE_USE_THREADS

/*
 * vim: et sw=4 ts=4
 */
