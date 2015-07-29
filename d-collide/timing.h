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


#ifndef DCOLLIDE_TIMING_H
#define DCOLLIDE_TIMING_H

#include "dcollide-config.h"
#include <iostream>

#ifdef HAVE_SYS_TIME_H

    #include <sys/time.h>

    /*!
     * \return The time (in micro second, i.e. us) elapsed between \p t1 and \p t2.
     */
    inline static long int compareTimes(const struct timeval& t1, const struct timeval& t2) {
         return (((t2.tv_sec - t1.tv_sec) * 1000000) + (t2.tv_usec - t1.tv_usec));
    }

#else

    // FIXME: atm we assume that we are on windows, if sys/time.h is not
    // available. are there other relevant systems without sys/time.h?
    #include "windows.h"

#endif // HAVE_SYS_TIME_H


namespace dcollide {

    class PointInTime;

    /*!
     * \brief Class to measure elapsed time
     *
     * This class is meant primarily for debugging: it measures time between a
     * call to \ref restart and \ref stop. For convenience \ref restart is
     * called by the constructor.
     *
     * WARNING: this class (currently?) measures time by using the system time
     * (currently gettimeofday())
     * and this is inherently inprecise. The returned values of this class are
     * in microseconds (us), but the actual precision (which is highly operating
     * system dependent) may be much less. Errors of several ms or more are
     * possible, however usually the system time is sufficiently precise.
     *
     * Thus you should NOT measure very short time intervals, but rather large
     * tasks with this class.
     *
     * Sample use:
     * \code
     * Timing t; // starts measuring time
     * doSomething();
     * t.stop(); // records elapsed time
     *
     * printf("Elapsed time: %d\n", t.elapsedTime());
     * \endcode
     *
     * Note that \ref elapsedTime always returns the measured time between two
     * \ref restart and \ref stop calls, it does not the time that has elapsed
     * since starting the timer - use \ref elapsedTimeSinceStart for that.
     *
     * This behavior of \ref stop and \ref elapsedTime in particular allows it
     * to store a Timing object and read the time that has elapsed for that
     * particular event at a later point. Example:
     * \code
     * Timing mInitializingTime;
     * initializeScene();
     * mInitializingTime.stop(); // records time for scene initializing
     *
     * ... // create the GUI, move objects, do collision detection, ...
     *
     * printf("Scene initializing took: %d\n", mInitializingTime.elapsedTime());
     * \endcode
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class Timing {
        public:
            inline Timing();
            inline Timing(const Timing& t);
            inline ~Timing();

            inline Timing& operator=(const Timing& t);

            inline void restart();
            inline long int stop();

            inline void nullify();
            inline long int elapsedTime() const;
            inline long int elapsedTimeSinceStart() const;

            inline bool isStopped() const;


            inline bool endedBefore(const PointInTime& t) const;
            PointInTime startedAt() const;
            PointInTime endedAt() const;

        private:
            bool mStopped;
#ifdef HAVE_SYS_TIME_H
            struct timeval mStart;
            struct timeval mEnd;
#else
            LONGLONG mFrequency;
            LONGLONG mStart;
            LONGLONG mEnd;
#endif
    };

    /*!
     * \brief Helper class for use with \ref Timing
     *
     * This class represents a specific point in time, i.e. a certain time. On
     * unix this may for example be equivalent to the value obtained by
     * gettimeofday().
     *
     * An object of this class is helpful for example to filter out "old"
     * profiling values (e.g. everything older than 2 seconds ago). Example:
     * \code
     * void foo() {
     *     Timing timing;
     *     // do something
     *     timing.stop();
     *     globalTimingList.pushBack(timing);
     * }
     *
     * void analyzeResults() {
     *     PointInTime time;
     *     time.addSeconds(-2);
     *     for (each element t of globalTimingList) {
     *         if (t.endedAt().isBefore(time)) {
     *             // discard value as it is too old
     *             continue;
     *         }
     *         displayTiming(t);
     *     }
     * }
     * \endcode
     */
    class PointInTime {
        public:
            inline PointInTime();
            inline PointInTime(const PointInTime&);

            inline void update();
            inline void addSeconds(int s);
            inline void addMs(int ms);

            inline bool isBefore(const PointInTime& t) const;
            inline PointInTime& operator=(const PointInTime& t);

            inline long int timePassedSince(const PointInTime& t) const;

        private:
#ifdef HAVE_SYS_TIME_H
            inline PointInTime(const struct timeval& time);
#else
            inline PointInTime(const LONGLONG& time);
#endif

        private:
            friend class Timing;

#ifdef HAVE_SYS_TIME_H
            struct timeval mTime;
#else
            LONGLONG mFrequency;
            LONGLONG mTime;
#endif
    };



#ifdef HAVE_SYS_TIME_H
    /*!
     * Compare two timeval structs. This operator is used by \ref Timing and
     * \ref PointInTime on systems supporting timeval structs.
     * \return TRUE if \p t1 is before \p t2, otherwise FALSE.
     */
    inline bool operator<(const struct timeval& t1, const struct timeval& t2) {
        if (t1.tv_sec < t2.tv_sec) {
            return true;
        } else if (t1.tv_sec > t2.tv_sec) {
            return false;
        }
        return (t1.tv_usec < t2.tv_usec);
    }
#endif




    /*!
     * \return TRUE if \ref stop has been called since the last call to \ref
     * restart. Only then the value of \ref elapsedTime is usable. Otherwise
     * returns FALSE.
     */
    inline bool Timing::isStopped() const {
        return mStopped;
    }

    /*!
     * \return TRUE if \ref stop was called before \p t, otherwise FALSE. If \ref
     * stop has not been called yet the return value is undefined.
     */
    inline bool Timing::endedBefore(const PointInTime& t) const {
#ifdef HAVE_SYS_TIME_H
        if (mEnd < t.mTime) {
            return true;
        }
        return false;
#else
        if (mEnd < t.mTime) {
            return true;
        }
        return false;
#endif
    }

    /*!
     * \return A \ref PointInTime object describing the moment in which \ref
     * restart was called the last time (i.e. usually the moment when this
     * Timing object was constructed).
     */
    inline PointInTime Timing::startedAt() const {
#ifdef HAVE_SYS_TIME_H
        return PointInTime(mStart);
#else
        return PointInTime(mStart);
#endif
    }

    /*!
     * \return A \ref PointInTime object describing the moment in which \ref
     * stop was called the last time. The value returned is undefined (but a
     * valid object) if \ref stop has not been called yet.
     */
    inline PointInTime Timing::endedAt() const {
#ifdef HAVE_SYS_TIME_H
        return PointInTime(mEnd);
#else
        return PointInTime(mEnd);
#endif
    }


    /*!
     * Start measuring time, i.e. call \ref restart
     */
    Timing::Timing() {
        mStopped = false;
        restart();
    }

    /*!
     * Copy the \ref Timing object \p t. If \p t has not been stopped yet (i.e.
     * \ref stop was not called yet), then this object is not stopped yet
     * either.
     */
    Timing::Timing(const Timing& t) {
        *this = t;
    }

    Timing::~Timing() {
    }

    /*!
     * Assign the timing values of \p t to this object. In particular the "has
     * been stopped" property is copied, i.e. if \p t is not stopped yet, then
     * this object won't be stopped either until \ref stop was called. This
     * affects some important methods in this class, e.g. \ref elapsedTime does
     * not return anything useful before \ref stop was called (note that \ref
     * elapsedTimeSinceStart does not require a call to \ref stop).
     */
    Timing& Timing::operator=(const Timing& t) {
        mStopped = t.mStopped;

#ifdef HAVE_SYS_TIME_H
        mStart = t.mStart;
        mEnd = t.mEnd;
#else
        mStart = t.mStart;
        mEnd = t.mEnd;
#endif // HAVE_SYS_TIME_H

        return *this;
    }

    /*!
     * Restart measuring, i.e. reset the internal \ref startedAt moment.
     *
     * Note that many methods in this class do not return anything useful after
     * a call to \ref restart, unless \ref stop has been called. This in
     * particular applies to \ref elapsedTime (but not to \ref
     * elapsedTimeSinceStart).
     */
    void Timing::restart() {
        mStopped = false;
#ifdef HAVE_SYS_TIME_H
        gettimeofday(&mStart, 0);
        mEnd.tv_sec = mStart.tv_sec;
        mEnd.tv_usec = mStart.tv_usec;
#else
        QueryPerformanceFrequency((LARGE_INTEGER*)&mFrequency);
        QueryPerformanceCounter((LARGE_INTEGER*)&mStart);
        mFrequency /= 1000000;
        mEnd = mStart;
#endif
    }

    /*!
     * Stop measuring time and returned the elapsed time since the last call to
     * \ref restart (which is also called by the constructor)
     *
     * \return See \ref elapsedTime
     */
    long int Timing::stop() {
        mStopped = true;
#ifdef HAVE_SYS_TIME_H
        gettimeofday(&mEnd, 0);
#else
        QueryPerformanceCounter((LARGE_INTEGER*)&mEnd);
#endif

        return elapsedTime();
    }

    /*!
     * Nullify this timing object, i.e. set \ref elapsedTime to 0.
     */
    inline void Timing::nullify() {
        stop();
        mEnd = mStart;
    }

    /*!
     * \return The time (in us) that has elapsed between the last call to \ref restart
     * (which is also called by the constructor) and \ref stop.
     *
     * Note: This does NOT return the time since the last \ref restart call, but the
     * time between \ref restart and \ref stop. Once \ref stop has been called,
     * this method always returns the same value.
     *
     * Note2: even though this method returns time in micro seconds (us), the
     *        actual precision is usually much less and highly dependent on the
     *        operating system. Errors of several ms or more are possible.
     */
    long int Timing::elapsedTime() const {
#ifdef HAVE_SYS_TIME_H
        return compareTimes(mStart, mEnd);
#else
        return (long int)((mEnd - mStart) / mFrequency);
#endif
    }

    /*!
     * \return The time (in us) that has elapsed since the last call to \ref
     * restart (which is also called by the constructor).
     */
    long int Timing::elapsedTimeSinceStart() const {
#ifdef HAVE_SYS_TIME_H
        struct timeval end;
        gettimeofday(&end, 0);
        return compareTimes(mStart, end);
#else
        LONGLONG end;
        QueryPerformanceCounter((LARGE_INTEGER*)&end);
        return (long int)((end - mStart) / mFrequency);
#endif
    }



    /*!
     * Construct a \ref PointInTime object representing the current time, i.e.
     * "now".
     */
    PointInTime::PointInTime() {
        update();
    }

    /*!
     * Construct a \ref PointInTime object representing the same time as \p t.
     */
    PointInTime::PointInTime(const PointInTime& t) {
        *this = t;
    }

#ifdef HAVE_SYS_TIME_H
    /*!
     * Construct a \ref PointInTime object representing the time described by \p
     * t.
     */
    PointInTime::PointInTime(const struct timeval& t) {
        mTime = t;
    }
#else
    /*!
     * Construct a \ref PointInTime object representing the time described by \p
     * t.
     */
    PointInTime::PointInTime(const LONGLONG& t) {
        QueryPerformanceFrequency((LARGE_INTEGER*)&mFrequency);
        mFrequency /= 1000000;
        mTime = t;
    }
#endif

    /*!
     * \return TRUE if the point in time described by \p t is before the point
     * in time described by ths object, otherwise FALSE.
     */
    bool PointInTime::isBefore(const PointInTime& t) const {
#ifdef HAVE_SYS_TIME_H
        return mTime < t.mTime;
#else
        return mTime < t.mTime;
#endif
    }

    /*!
     * Assign \p t to this object, i.e. make this object describe the same point
     * in time as \p t.
     */
    PointInTime& PointInTime::operator=(const PointInTime& t) {
#ifdef HAVE_SYS_TIME_H
        mTime = t.mTime;
#else
        mTime = t.mTime;
#endif
        return *this;
    }

    /*!
     * Reset the time represented by this object, i.e. make this object
     * represent the current time (i.e. "now").
     */
    void PointInTime::update() {
#ifdef HAVE_SYS_TIME_H
        gettimeofday(&mTime, 0);
#else
        QueryPerformanceFrequency((LARGE_INTEGER*)&mFrequency);
        QueryPerformanceCounter((LARGE_INTEGER*)&mTime);
        mFrequency /= 1000000;
#endif
    }

    /*!
     * Add \p s seconds to the time described by this object.
     */
    void PointInTime::addSeconds(int s) {
#ifdef HAVE_SYS_TIME_H
        mTime.tv_sec += s;
#else
        mTime += s * 1000000 * mFrequency;
#endif
    }

    /*!
     * Add \p ms milli-seconds to the time described by this object.
     */
    void PointInTime::addMs(int ms) {
#ifdef HAVE_SYS_TIME_H
        addSeconds(ms / 1000);
        ms = ms % 1000;
        long long us = ms;
        us *= 1000;
        mTime.tv_usec += us;
#else
        mTime += ms * 1000 * mFrequency;
#endif
    }

    /*!
     * \return Time (in microseconds, i.e. us) that passed since the time \p t
     * until the time in this object.
     */
    long int PointInTime::timePassedSince(const PointInTime& t) const {
#ifdef HAVE_SYS_TIME_H
        return compareTimes(t.mTime, mTime);
#else
        return (long int)((mTime - t.mTime) / mFrequency);
#endif
    }

}

#endif
/*
 * vim: et sw=4 ts=4
 */
