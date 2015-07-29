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

#ifndef DCOLLIDE_FRAMEHISTORY_H
#define DCOLLIDE_FRAMEHISTORY_H

#include <string>
#include <list>

namespace dcollide {
    class Timing;
    class PointInTime;
}

class Frame;

class FrameHistory {
    public:
        FrameHistory(const std::string& name = "");
        ~FrameHistory();

        void setName(const std::string& name);
        inline const std::string getName() const;

        void startFrame();
        Frame* stopFrame();
        void addManualFrame(const dcollide::Timing& time);

        void setSkipped(bool skipped, const std::string& skipMessage = "");
        inline bool isSkipped() const;
        inline const std::string& getSkipMessage() const;

        void setRemoveFramesOlderThan(unsigned int ms);
        unsigned int getRemoveFramesOlderThan() const;

        const std::list<Frame*>& getFrames() const;

        double getFpsAverage(bool extraPolate = false) const;
        long int getUsAverage() const;

        const dcollide::PointInTime& getAverageLastCalled() const;

        void setDestructionTextName_Hack(const std::string& text);

//        double calculateFpsAverageOfPreviousSeconds(int seconds) const;

    protected:
//        double calculateFpsAverage(const std::list<Frame*>::const_iterator& itBegin, const std::list<Frame*>::const_iterator& itAfterLast) const;
        void removeOldFrames();

        void addFrame(Frame*);
        void removeFrame();

    private:
        std::string mDestructionText;
        std::string mName;
        std::list<Frame*> mFrames;
        dcollide::PointInTime* mAverageLastCalled;
        unsigned int mFrameCount;
        Frame* mCurrentFrame;
        unsigned int mRemoveFramesOlderThan;

        long int mTimeSum;

        bool mSkipped;
        std::string mSkipMessage;
};

class Frame {
    public:
        Frame();
        ~Frame();

        void setTiming(const dcollide::Timing& t);
        void stopTiming();

        const dcollide::Timing& getTiming() const;

    private:
        dcollide::Timing* mTiming;
};

inline const std::string FrameHistory::getName() const {
    return mName;
}

/*!
 * \return TRUE if this frame history is currently being skipped (e.g. if this
 * frame history measures physics, but the current scene does not use physics).
 * FALSE otherwise (the default).
 */
inline bool FrameHistory::isSkipped() const {
    return mSkipped;
}

/*!
 * \return A message that can be displayed as reason why this frame history is
 * skipped, or an empty string if no message was provided (see \ref setSkipped).
 */
inline const std::string& FrameHistory::getSkipMessage() const {
    return mSkipMessage;
}

#endif
/*
 * vim: et sw=4 ts=4
 */
