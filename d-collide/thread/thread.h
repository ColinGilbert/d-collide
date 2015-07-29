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


#ifndef DCOLLIDE_THREADS_H
#define DCOLLIDE_THREADS_H

#include <vector>
#include <list>

#include "dcollide-config.h"
#include "../exceptions/exception.h"

#ifdef DCOLLIDE_USE_THREADS
 #include <pthread.h>
 #include <errno.h>
#endif // DCOLLIDE_USE_THREADS


#include <sstream>


#ifdef DCOLLIDE_USE_THREADS

/*!
 * \internal
 * Checks the error code of pthread_cond_wait() and outputs the error to stderr
 * if any.
 */
#define CHECK_CONDITION_WAIT_CODE(code) \
    if (code != 0) { \
        if (code == EINVAL) { \
            throw ConditionWaitException("invalid value in pthread_cond_wait().", code); \
        } else if (code == EPERM) { \
            throw ConditionWaitException("mutex not owned.", code); \
        } else { \
            throw ConditionWaitException("", code);\
        }\
    }

/*!
 * \internal
 * Checks the error code of pthread_mutex_lock() and exits the application on
 * error (mutex lock errors are fatal: the application assumes it received the
 * lock or blocks until it does, but on error that's not the case)
 */
#define CHECK_MUTEX_LOCK_CODE(code) \
    if (code != 0) { \
        if (code == EDEADLK) { \
            throw MutexLockFailedException("deadlock detected: thread already owns the mutex"); \
        } else if (code == EINVAL) { \
            throw MutexLockFailedException("invalid value for mutex (mutex already destructed?)"); \
        } \
    }

/*!
 * \internal
 * Checks the error code of pthread_mutex_unlock() and outputs the error if any.
 */
#define CHECK_MUTEX_UNLOCK_CODE(code) \
    if (code != 0) { \
        if (code == EPERM) { \
            throw MutexUnlockFailedException("the current thread does not own the mutex. cannot unlock."); \
        } else { \
            std::stringstream reason; \
            reason << "unknown error code: " << code << std::endl; \
            throw MutexUnlockFailedException(reason.str()); \
        } \
    }

/*!
 * \internal
 * Checks the error code of pthread_rwlock_rdlock() and exits the application on
 * error (lock errors are fatal: the application assumes it received the
 * lock or blocks until it does, but on error that's not the case)
 */
#define CHECK_RWLOCK_RDLOCK_CODE(code) \
    if (code != 0) { \
        if (code == EAGAIN) { \
            throw ReadWriteLockException(true, "maximum number of locks exceeded"); \
        } else if (code == EDEADLK) { \
            throw ReadWriteLockException(true, "deadlock detected: thread already owns the lock"); \
        } else if (code == EINVAL) { \
            throw ReadWriteLockException(true, "invalid value for rwlock (rwlock already destructed?)"); \
        } \
    }

/*!
 * \internal
 * Checks the error code of pthread_rwlock_wrlock() and exits the application on
 * error (lock errors are fatal: the application assumes it received the
 * lock or blocks until it does, but on error that's not the case)
 */
#define CHECK_RWLOCK_WRLOCK_CODE(code) \
    if (code != 0) { \
        if (code == EDEADLK) { \
            throw ReadWriteLockException(false, "deadlock detected: thread already owns the lock"); \
        } else if (code == EINVAL) { \
            throw ReadWriteLockException(false, "invalid value for rwlock (rwlock already destructed?)"); \
        } \
    }

/*!
 * \internal
 * Checks the error code of pthread_rwlock_unlock() and outputs the error if any.
 */
#define CHECK_RWLOCK_UNLOCK_CODE(code) \
    if (code != 0) { \
        if (code == EPERM) { \
            throw ReadWriteUnlockException("current thread does not hold a lock on this rwlock"); \
        } else if (code == EINVAL) { \
            throw ReadWriteUnlockException("invalid value for rwlock (rwlock already destructed?)"); \
        } else { \
            throw ReadWriteUnlockException("unknown error"); \
        } \
    }



namespace dcollide {
    class Mutex;

    /*!
     * \brief A d-collide thread
     *
     * This class is an internal thread represenation. It is not meant to be
     * used by the user directly (however nothings prevents you from doing so,
     * it should do no harm).
     *
     * See also \ref WorkerThread which is the subclass of Thread that is used
     * by \ref WorkerPool.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class Thread {
        public:
            Thread();
            virtual ~Thread();

            void start(bool joinable = false);

            bool isRunning() const;

            int join();

        protected:
            /*!
             * Derived classes are meant to implement the main loop of the
             * thread here.
             *
             * Do NOT call directly, use \ref start to start the thread,
             * otherwise your code will not run in a separate thread, but rather
             * in the calling thread.
             */
            virtual void run() = 0;

        private:
            static void* runThread(void* arg);

        private:
            pthread_t mThread;
            bool mIsRunning;
            bool mIsJoinable;

    };

    // AB: recursive mutexes are atm not supported (because I haven't needed
    // them yet)
    /*!
     * \brief A simple mutex class
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class Mutex {
        public:
            Mutex();
            ~Mutex();

            void lock();
            bool tryLock();
            void unlock();

        private:
            pthread_mutex_t mMutex;
    };

    /*!
     * \brief A locker class for \ref Mutex
     *
     * This class is meant to make locking and unlocking of mutex simpler in
     * "the C++ way". On construction this object calls \ref Mutex::lock on the
     * mutex in the parameter of the constructor, on destruction it calls \ref
     * Mutex::unlock on it. This way you can replace code like
     * \code
     * void MyClass::foo() {
     *     mMutex.lock();
     *     if (x == y) {
     *         mMutex.unlock(); // code is broken if you forget this line!!
     *         return;
     *     }
     *     int count = countSomething();
     *     for (int i = 0; i < count; i++) {
     *         if (mArray[i] == 0) {
     *             mMutex.unlock(); // code is broken if you forget this line!!
     *             return;
     *         }
     *         doSomething(mArray[i]);
     *     }
     *     mMutex.unlock(); // code is broken if you forget this line!!
     * }
     * \endcode
     * By a much simpler version:
     * \code
     * void MyClass::foo() {
     *     MutexLocker lock(&mMutex);
     *     if (x == y) {
     *         return;
     *     }
     *     int count = countSomething();
     *     for (int i = 0; i < count; i++) {
     *         if (mArray[i] == 0) {
     *             return;
     *         }
     *         doSomething(mArray[i]);
     *     }
     * }
     * \endcode
     * As you can see the MutexLocker makes sure that the mutex is always
     * unlocked properly, which is much easier to write and much, much less
     * error-prone.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class MutexLocker {
        public:
            explicit inline MutexLocker(Mutex* mutex);
            inline ~MutexLocker();

            inline void unlock();

        private:
            Mutex* mMutex;
            bool mIsLocked;

    };


    /*!
     * \brief A simple Read-Write lock class
     *
     * A Read-Write lock is essentially similar to a \ref Mutex, however any
     * number of threads can lock the ReadWriteLock for read simultaneously (see
     * \ref lockForRead) without blocking each other. Only when a thread locks
     * for writing (see \ref lockForWrite) other threads have to wait until the
     * write access is completed.
     *
     * Note that locks can neither be upgraded nor be downgraded: if a thread
     * locks for read, it first has to unlock before it can lock for write (and
     * same the other way around). Some systems may support
     * upgradeable/downgradable rwlocks, but this is not portable: e.g. if a
     * thread attempts to lock for write while holding the read lock, the thread
     * may deadlock (see manpage of pthread_rwlock_wrlock).
     *
     * \see ReadLocker WriteLocker Mutex
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class ReadWriteLock {
        public:
            ReadWriteLock();
            ~ReadWriteLock();

            void lockForRead();
            void lockForWrite();
            void unlock();

            // AB: tryLockFor*() are not implemented because they are not
            //     currently needed. feel free to implement if required.

        private:
            pthread_rwlock_t mLock;
    };

    /*!
     * \brief Read locker class for \ref ReadWriteLock. 
     *
     * This class is similar to \ref MutexLocker, but instead of \ref
     * Mutex it applies to \ref ReadWriteLock.
     *
     * This class automatically locks the ReadWriteLock for reading. See also
     * \ref WriteLocker which locks for writing.
     *
     * Sample usage:
     * \code
     * void MyClass::foo() {
     *     ReadLocker lock(&mReadWriteLock);
     *     readSomething();
     *     readMore();
     *
     *     // rwlock is automatically unlocked
     * }
     * \endcode
     *
     * \see ReadWriteLock WriteLocker Mutex
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class ReadLocker {
        public:
            explicit inline ReadLocker(ReadWriteLock* lock);
            inline ~ReadLocker();

            inline void unlock();

        private:
            ReadWriteLock* mLock;
            bool mIsLocked;
    };

    /*!
     * \brief Write locker class for \ref ReadWriteLock. 
     *
     * This class is similar to \ref MutexLocker, but instead of \ref
     * Mutex it applies to \ref ReadWriteLock.
     *
     * This class automatically locks the ReadWriteLock for writing. See also
     * \ref ReadLocker which locks for reading.
     *
     * Sample usage:
     * \code
     * void MyClass::foo() {
     *     WriteLocker lock(&mReadWriteLock);
     *     writeSomething();
     *     writeMore();
     *
     *     // rwlock is automatically unlocked
     * }
     * \endcode
     *
     * \see ReadWriteLock ReadLocker Mutex
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class WriteLocker {
        public:
            explicit inline WriteLocker(ReadWriteLock* lock);
            inline ~WriteLocker();

            inline void unlock();

        private:
            ReadWriteLock* mLock;
            bool mIsLocked;
    };


    /*!
     * Automatically locks \p mutex. The lock is unlocked on
     * destruction or by a manual call to \ref unlock
     */
    inline MutexLocker::MutexLocker(Mutex* mutex) :
            mMutex(mutex),
            mIsLocked(true) {
        mMutex->lock();
    }

    /*!
     * Unlocks the mutex. See also \ref unlock
     */
    inline MutexLocker::~MutexLocker() {
        if (mIsLocked) {
            unlock();
        }
    }

    /*!
     * Manually unlocks the mutex. Normally you do not need to call this, it is called
     * automatically on destruction. See also \ref Mutex::unlock
     */
    inline void MutexLocker::unlock() {
        mIsLocked = false;
        mMutex->unlock();
    }

    /*!
     * Automatically locks \p lock for reading. The lock is unlocked on
     * destruction or by a manual call to \ref unlock
     */
    inline ReadLocker::ReadLocker(ReadWriteLock* lock) :
            mLock(lock),
            mIsLocked(true) {
        mLock->lockForRead();
    }

    /*!
     * Unlocks the lock. See also \ref unlock
     */
    inline ReadLocker::~ReadLocker() {
        if (mIsLocked) {
            unlock();
        }
    }

    /*!
     * Manually unlocks. Normally you do not need to call this, it is called
     * automatically on destruction. See also \ref ReadWriteLock::unlock
     */
    inline void ReadLocker::unlock() {
        mIsLocked = false;
        mLock->unlock();
    }


    /*!
     * Automatically locks \p lock for writing. The lock is unlocked on
     * destruction or by a manual call to \ref unlock
     */
    inline WriteLocker::WriteLocker(ReadWriteLock* lock) :
            mLock(lock),
            mIsLocked(true) {
        mLock->lockForWrite();
    }

    /*!
     * Unlocks the lock. See also \ref unlock
     */
    inline WriteLocker::~WriteLocker() {
        if (mIsLocked) {
            unlock();
        }
    }

    /*!
     * Manually unlocks. Normally you do not need to call this, it is called
     * automatically on destruction. See also \ref ReadWriteLock::unlock
     */
    inline void WriteLocker::unlock() {
        mIsLocked = false;
        mLock->unlock();
    }
}

#else // DCOLLIDE_USE_THREADS

namespace dcollide {
    /*!
     * Dummy implementation for use without threading support.
     */
    class Mutex {
        public:
            inline void lock();
            inline bool tryLock();
            inline void unlock();
    };

    /*!
     * Dummy implementation for use without threading support.
     */
    class MutexLocker {
        public:
            explicit inline MutexLocker(Mutex* mutex);

            inline void unlock();

    };

    /*!
     * Dummy implementation for use without threading support.
     */
    class ReadWriteLock {
        public:
            inline void lockForRead();
            inline void lockForWrite();
            inline void unlock();
    };

    /*!
     * Dummy implementation for use without threading support.
     */
    class ReadLocker {
        public:
            explicit inline ReadLocker(ReadWriteLock* lock);

            inline void unlock();
    };

    /*!
     * Dummy implementation for use without threading support.
     */
    class WriteLocker {
        public:
            explicit inline WriteLocker(ReadWriteLock* lock);

            inline void unlock();
    };

    inline void Mutex::lock() {
    }
    inline bool Mutex::tryLock() {
        return true;
    }
    inline void Mutex::unlock() {
    }

    inline MutexLocker::MutexLocker(Mutex*) {
    }
    inline void MutexLocker::unlock() {
    }

    void ReadWriteLock::lockForRead() {
    }
    void ReadWriteLock::lockForWrite() {
    }
    void ReadWriteLock::unlock() {
    }

    ReadLocker::ReadLocker(ReadWriteLock*) {
    }
    void ReadLocker::unlock() {
    }

    WriteLocker::WriteLocker(ReadWriteLock*) {
    }
    void WriteLocker::unlock() {
    }
}

#endif // DCOLLIDE_USE_THREADS



#endif
/*
 * vim: et sw=4 ts=4
 */
