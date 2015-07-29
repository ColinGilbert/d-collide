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

#ifndef DCOLLIDE_DEBUGLOG_H
#define DCOLLIDE_DEBUGLOG_H

#include <string>
#include <map>
#include <list>


namespace dcollide {
    class Timing;
    class DebugLogEntry;

    /*!
     * \ref Debug class that provides a logging facility
     *
     * See \ref DebugLogEntry for an example on how to use this class.
     *
     * This class is heavily based on the assumption that whatever is to be
     * debugged is some kind of recurring event and that every "run" of that
     * event works in a very similar way. In particular \ref printSummary will
     * not work as desired anymore if the most recent \ref DebugLogEntry uses
     * different variable or timing names than the other entries.
     *
     * Also it is not possible with this class to add some kind of
     * "hierarchical" entries - so if one entry is added per collision detection
     * call, you cannot add one entry per object to that entry. For such things
     * you will need another \ref DebugLog object.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class DebugLog {
        public:
            DebugLog();
            ~DebugLog();

            DebugLogEntry* addEntry();
            const std::list<DebugLogEntry*>& getEntryList() const;
            DebugLogEntry* getMostRecentEntry() const;

            void getTimingSummary(const std::string& key, unsigned int* maxElapsedTime, unsigned int* minElapsedTime, double* averageElapsedTime);

            void getIntVariableSummary(const std::string& key, int* maxValue, int* minValue, double* averageValue);
            void getUIntVariableSummary(const std::string& key, unsigned int* maxValue, unsigned int* minValue, double* averageValue);

            void printSummary();
            void printMostRecentEntry();

        private:
            std::list<DebugLogEntry*> mEntries;
            unsigned int mEntriesSize;
    };

    /*!
     * \brief Helper class to \ref DebugLog
     *
     * This class provides a single entry of a \ref DebugLog, see \ref
     * DebugLog::addEntry to create such an entry.
     *
     * One entry can store several variables and timing information, see \ref
     * addTiming and \ref setIntVariable, \ref setUIntVariable.
     * Every of these variables has a name assigned which it can be retrieved
     * with - a variable can be replaced by simply overwriting it, e.g. calling
     * \ref setIntVariable with the same name again.
     *
     * Example usage:
     * \code
     * // start collision detection run
     * DebugLogEntry* currentEntry = mDebugLog->addEntry();
     * currentEntry->setIntVariable("Objects in collision pipeline", mObjectCount);
     * Timing t;
     * doBroadPhase();
     * t.stop();
     * currentEntry->addTiming("Broadphase", t);
     *
     * t.restart();
     * doMiddlePhase();
     * t.stop();
     * currentEntry->addTiming("MiddlePhase", t);
     *
     * t.restart();
     * doNarrowPhase();
     * t.stop();
     * currentEntry->addTiming("NarrowPhase", t);
     *
     * displayTime("Broadphase took: ", currentEntry->getTiming("Broadphase"));
     * // ...
     *
     * // once several collision calls have been made:
     * double averageBroadPhase;
     * mDebugLog->getTimingSummary("Broadphase", 0, 0, &averageBroadPhase);
     * displaySummary("Broadphase average: %f", averageBroadPhase);
     *
     * double averageObjectCount;
     * mDebugLog->getIntVariableSummary("Objects in collision pipeline", 0, 0, &averageObjectCount);
     * displaySummary("Object count average: %f", averageObjectCount);
     * \endcode
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class DebugLogEntry {
        public:
            DebugLogEntry();
            ~DebugLogEntry();

            void addTiming(const std::string& name, const Timing& t);
            bool hasTiming(const std::string& name) const;
            Timing getTiming(const std::string& name) const;
            std::list<std::string> getTimingKeys() const;
            const std::map<std::string, Timing>& getTimings() const;

            void setIntVariable(const std::string& key, int value);
            void setUIntVariable(const std::string& key, unsigned int value);
            bool hasIntVariable(const std::string& key) const;
            bool hasUIntVariable(const std::string& key) const;
            int getIntVariable(const std::string& key) const;
            unsigned int getUIntVariable(const std::string& key) const;
            std::list<std::string> getIntVariableKeys() const;
            std::list<std::string> getUIntVariableKeys() const;

        private:
            std::map<std::string, Timing> mTimings;
            std::map<std::string, int> mIntVariables;
            std::map<std::string, unsigned int> mUIntVariables;
    };


}
#endif
/*
 * vim: et sw=4 ts=4
 */
