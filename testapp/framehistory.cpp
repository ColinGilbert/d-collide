/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,         *
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

#include "framehistory.h"

#include <debug.h>
#include <debugstream.h>
#include <timing.h>

#include <iostream>

FrameHistory::FrameHistory(const std::string& name) {
    mFrameCount = 0;
    mCurrentFrame = 0;
    mRemoveFramesOlderThan = 2000;
    mTimeSum = 0;
    mAverageLastCalled = new dcollide::PointInTime();
    mAverageLastCalled->addSeconds(-100000); // just a random, large negative number
    mSkipped = false;
    mDestructionText = "";

    setName(name);
}

FrameHistory::~FrameHistory() {
    if (mDestructionText != "") {
        dcollide::debug() << mDestructionText << " average time: " << (unsigned int)getUsAverage() << "us (" << (unsigned int)getUsAverage()/1000 << "ms) on " << mFrameCount << " frames - " << getFpsAverage() << " fps";
    }
    delete mAverageLastCalled;
    delete mCurrentFrame;
    mCurrentFrame = 0;
    for (std::list<Frame*>::iterator it = mFrames.begin(); it != mFrames.end(); ++it) {
        delete *it;
    }
}

void FrameHistory::setName(const std::string& name) {
    mName = name;
}

void FrameHistory::startFrame() {
    delete mCurrentFrame;
    mCurrentFrame = new Frame();
}

Frame* FrameHistory::stopFrame() {
    if (!mCurrentFrame) {
        return 0;
    }
    Frame* f = mCurrentFrame;
    mCurrentFrame = 0;

    f->stopTiming();
    addFrame(f);

    removeOldFrames();

    return f;
}

void FrameHistory::addManualFrame(const dcollide::Timing& t) {
    Frame* f = new Frame();
    f->setTiming(t);

    addFrame(f);

    removeOldFrames();
}

void FrameHistory::setRemoveFramesOlderThan(unsigned int ms) {
    mRemoveFramesOlderThan = std::max(ms, (unsigned int)1);

    // AB: we do some calculations in us and a 32 bit int can store only (2^31-1) of them, i.e. (2^31-1)/1000 ms.
    mRemoveFramesOlderThan = std::min(mRemoveFramesOlderThan, (unsigned int)2000000);
}

unsigned int FrameHistory::getRemoveFramesOlderThan() const {
    return mRemoveFramesOlderThan;
}

void FrameHistory::removeOldFrames() {
    dcollide::PointInTime t;
    t.addMs((int)(-1 * getRemoveFramesOlderThan()));
    while (!mFrames.empty() && mFrames.back()->getTiming().endedBefore(t)) {
        removeFrame();
    }
}

void FrameHistory::removeFrame() {
    if (mFrames.empty()) {
        return;
    }
    mTimeSum -= mFrames.back()->getTiming().elapsedTime();
    delete mFrames.back();
    mFrames.pop_back();
    mFrameCount--;
}

void FrameHistory::addFrame(Frame* f) {
    mFrames.push_front(f);
    mFrameCount++;
    mTimeSum += mFrames.front()->getTiming().elapsedTime();
}

const std::list<Frame*>& FrameHistory::getFrames() const {
    return mFrames;
}

/*!
 * \return The average frames per second of all frames currently in the history,
 * or 0.0 if the history is empty. If \p extrapolate is TRUE, we calculate how
 * many frames COULD have been achieved, if nothing but this single task would
 * have run. Otherwise (the default) the actual number of frames is calculated.
 */
double FrameHistory::getFpsAverage(bool extrapolate) const {
    *mAverageLastCalled = dcollide::PointInTime();

    if (mFrames.empty()) {
        return 0.0;
    }
    if (extrapolate) {
        double t = (double)mTimeSum;
        t /= 1000000.0;
        return ((double)mFrameCount) / t;
    }
    dcollide::PointInTime start = mFrames.back()->getTiming().startedAt();
    dcollide::PointInTime end = mFrames.front()->getTiming().endedAt();

    long int time = end.timePassedSince(start);
    if (time == 0) {
        return 0.0;
    }
    return ((double)mFrameCount) / (((double)time) / 1000000.0);
}

long int FrameHistory::getUsAverage() const {
    *mAverageLastCalled = dcollide::PointInTime();

    if (mFrameCount == 0) {
        return 0;
    }
    return mTimeSum / mFrameCount;
}

/*!
 * Set whether this frame history was "skipped" currently. For example a frame
 * history may be "skipped" if it measures physics, but the currently active
 * scene does not use physics.
 *
 * \param message An optional message that may be displayed as reason why the
 *        history was skipped. E.g. "not applicable for current scene"
 */
void FrameHistory::setSkipped(bool skipped, const std::string& message) {
    mSkipped = skipped;
    mSkipMessage = message;
}

#if 0
double FrameHistory::calculateFpsAverageOfPreviousSeconds(int seconds) const {
    dcollide::PointInTime time;
    time.addSeconds(-seconds);

    std::list<Frame*>::const_iterator afterLast = mFrames.begin();
    while (afterLast != mFrames.end() && !(*afterLast)->getTiming().endedBefore(time)) {
        ++afterLast;
    }
    return fpsAverage(mFrames.begin(), afterLast);
}

double FrameHistory::calculateFpsAverage(const std::list<Frame*>::const_iterator& itBegin, const std::list<Frame*>::const_iterator& itAfterLast) const {
    long int timeSum = 0;
    int count = 0;
    Frame* firstFrame = *itBegin;
    Frame* lastFrame = firstFrame;
    for (std::list<Frame*>::const_iterator it = itBegin; it != itAfterLast; ++it) {
        timeSum += (*it)->getTiming().elapsedTime();
        lastFrame = *it;
        count++;
    }
    if (count == 0) {
        return 0.0;
    }


}

#endif

const dcollide::PointInTime& FrameHistory::getAverageLastCalled() const {
    return *mAverageLastCalled;
}

/*!
 * HACK: text that is displayed (along with the stored value) on destruction (if
 * non-empty)
 */
void FrameHistory::setDestructionTextName_Hack(const std::string& text) {
    mDestructionText = text;
}

Frame::Frame() {
    mTiming = new dcollide::Timing();
}

Frame::~Frame() {
    delete mTiming;
}

const dcollide::Timing& Frame::getTiming() const {
    return *mTiming;
}

void Frame::stopTiming() {
    mTiming->stop();
}

void Frame::setTiming(const dcollide::Timing& t) {
    *mTiming = t;
}

/*
 * vim: et sw=4 ts=4
 */
