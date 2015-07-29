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

#include "debuglog.h"

#include "timing.h"
#include "exceptions/exception.h"

#define MAX_ENTRIES 50

namespace dcollide {
    DebugLog::DebugLog() {
        mEntriesSize = 0;
    }

    DebugLog::~DebugLog() {
        while (!mEntries.empty()) {
            delete mEntries.front();
            mEntries.pop_front();
        }
        mEntriesSize = 0;
    }

    DebugLogEntry* DebugLog::addEntry() {
        DebugLogEntry* entry = new DebugLogEntry();
        mEntries.push_back(entry);
        mEntriesSize++;

        if (mEntriesSize > MAX_ENTRIES) {
            DebugLogEntry* front = mEntries.front();
            delete front;
            mEntries.pop_front();
            mEntriesSize--;
        }
        return entry;
    }

    DebugLogEntry* DebugLog::getMostRecentEntry() const {
        return mEntries.back();
    }

    const std::list<DebugLogEntry*>& DebugLog::getEntryList() const {
        return mEntries;
    }

    // note: only variable names and timing names that appear in the first entry
    // are recognized here!
    void DebugLog::printSummary() {
        std::cout << "DebugLog summary";
        if (mEntriesSize == 0) {
            std::cout << ": no log entries in log." << std::endl;
            return;
        }
        std::cout << " (" << mEntriesSize << " entries in log):" << std::endl;
        if (mEntries.empty()) {
            throw Exception("internal error: mEntries is empty, but mEntriesSize is not 0");
        }


        DebugLogEntry* first = mEntries.front();
        std::list<std::string> intKeys = first->getIntVariableKeys();
        for (std::list<std::string>::iterator it = intKeys.begin(); it != intKeys.end(); ++it) {
            int max;
            int min;
            double average;
            getIntVariableSummary(*it, &max, &min, &average);

            std::cout << " " << *it << ": max=" << max << " min=" << min << " average=" << average << std::endl;
        }

        std::list<std::string> uintKeys = first->getUIntVariableKeys();
        for (std::list<std::string>::iterator it = uintKeys.begin(); it != uintKeys.end(); ++it) {
            unsigned int max;
            unsigned int min;
            double average;
            getUIntVariableSummary(*it, &max, &min, &average);

            std::cout << " " << *it << ": max=" << max << " min=" << min << " average=" << average << std::endl;
        }

        std::list<std::string> timingKeys = first->getTimingKeys();
        for (std::list<std::string>::iterator it = timingKeys.begin(); it != timingKeys.end(); ++it) {
            unsigned int maxElapsedTime;
            unsigned int minElapsedTime;
            double averageElapsedTime;
            getTimingSummary(*it, &maxElapsedTime, &minElapsedTime, &averageElapsedTime);

            std::cout << "  " << *it << " times (in us): max=" << maxElapsedTime << " min=" << minElapsedTime << " average=" << averageElapsedTime << std::endl;
            if (averageElapsedTime > 2000.0) {
                std::cout << "       (in ms): max=" << maxElapsedTime / 1000 << " min=" << minElapsedTime / 1000 << " average=" << averageElapsedTime / 1000.0 << std::endl;
            }
        }
    }

    void DebugLog::printMostRecentEntry() {
        std::cout << "DebugLog entry";
        if (mEntriesSize == 0) {
            std::cout << ": no entry available" << std::endl;
            return;
        }
        std::cout << std::endl;
        if (mEntries.empty()) {
            throw Exception("internal error: mEntries is empty, but mEntriesSize is not 0");
        }
        DebugLogEntry* last = mEntries.back();

        std::list<std::string> intKeys = last->getIntVariableKeys();
        for (std::list<std::string>::iterator it = intKeys.begin(); it != intKeys.end(); ++it) {
            std::cout << " " << *it << ": " << last->getIntVariable(*it) << std::endl;
        }

        std::list<std::string> uintKeys = last->getUIntVariableKeys();
        for (std::list<std::string>::iterator it = uintKeys.begin(); it != uintKeys.end(); ++it) {
            std::cout << " " << *it << ": " << last->getUIntVariable(*it) << std::endl;
        }

        std::list<std::string> timingKeys = last->getTimingKeys();
        for (std::list<std::string>::iterator it = timingKeys.begin(); it != timingKeys.end(); ++it) {
            unsigned int elapsed = last->getTiming(*it).elapsedTime();

            std::cout << "  " << *it << " time (in us): " << elapsed << std::endl;
            if (elapsed > 2000.0) {
                std::cout << "       (in ms): " << elapsed / 1000 << std::endl;
            }
        }

    }

    void DebugLog::getTimingSummary(const std::string& key, unsigned int* maxElapsedTime, unsigned int* minElapsedTime, double* averageElapsedTime) {
        unsigned int count = 0;
        if (maxElapsedTime) {
            *maxElapsedTime = 0;
        }
        if (minElapsedTime) {
            *minElapsedTime = UINT_MAX;
        }
        if (averageElapsedTime) {
            *averageElapsedTime= 0.0;
        }
        long long totalElapsedTime = 0;
        for (std::list<DebugLogEntry*>::iterator it = mEntries.begin(); it != mEntries.end(); ++it) {
            if (!(*it)->hasTiming(key)) {
                continue;
            }
            count++;
            unsigned int elapsed = (*it)->getTiming(key).elapsedTime();
            totalElapsedTime += elapsed;
            if (maxElapsedTime) {
                *maxElapsedTime = std::max(*maxElapsedTime, elapsed);
            }
            if (minElapsedTime) {
                *minElapsedTime = std::min(*minElapsedTime, elapsed);
            }
        }
        if (count > 0 && averageElapsedTime) {
            *averageElapsedTime = ((double)totalElapsedTime) / ((double)count);
        }
    }
    void DebugLog::getIntVariableSummary(const std::string& key, int* maxValue, int* minValue, double* averageValue) {
        if (maxValue) {
            *maxValue = 0;
        }
        if (minValue) {
            *minValue = 0;
        }
        if (averageValue) {
            *averageValue = 0.0;
        }
        if (mEntries.empty()) {
            return;
        }
        if (minValue) {
            *minValue = INT_MAX;
        }

        unsigned int count = 0;
        long long totalValue = 0;
        std::list<DebugLogEntry*>::iterator it;
        for (it = mEntries.begin(); it != mEntries.end(); ++it) {
            if (!(*it)->hasIntVariable(key)) {
                continue;
            }
            count++;
            int value = (*it)->getIntVariable(key);
            totalValue += value;
            if (maxValue) {
                *maxValue = std::max(*maxValue, value);
            }
            if (minValue) {
                *minValue= std::min(*minValue, value);
            }
        }
        if (count > 0 && averageValue) {
            *averageValue = ((double)totalValue) / ((double)count);
        }
    }
    void DebugLog::getUIntVariableSummary(const std::string& key, unsigned int* maxValue, unsigned int* minValue, double* averageValue) {
        if (maxValue) {
            *maxValue = 0;
        }
        if (minValue) {
            *minValue = 0;
        }
        if (averageValue) {
            *averageValue = 0.0;
        }
        if (mEntries.empty()) {
            return;
        }
        if (minValue) {
            *minValue = UINT_MAX;
        }

        unsigned int count = 0;
        unsigned long long totalValue = 0;
        std::list<DebugLogEntry*>::iterator it;
        for (it = mEntries.begin(); it != mEntries.end(); ++it) {
            if (!(*it)->hasUIntVariable(key)) {
                continue;
            }
            count++;
            unsigned int value = (*it)->getUIntVariable(key);
            totalValue += value;
            if (maxValue) {
                *maxValue = std::max(*maxValue, value);
            }
            if (minValue) {
                *minValue= std::min(*minValue, value);
            }
        }
        if (count > 0 && averageValue) {
            *averageValue = ((double)totalValue) / ((double)count);
        }
    }


    DebugLogEntry::DebugLogEntry() {
    }

    DebugLogEntry::~DebugLogEntry() {
    }

    void DebugLogEntry::addTiming(const std::string& name, const Timing& t) {
        mTimings.insert(std::make_pair(name, t));
    }

    bool DebugLogEntry::hasTiming(const std::string& name) const {
        if (mTimings.find(name) == mTimings.end()) {
            return false;
        }
        return true;
    }

    /*!
     * \return The \ref Timing object that has been added using \ref addTiming,
     * or a nullified (see \ref Timing::nullify) object if no \ref Timing object
     * has been added with name \p name.
     */
    Timing DebugLogEntry::getTiming(const std::string& name) const {
        if (!hasTiming(name)) {
            Timing t;
            t.nullify();
            return t;
        }
        return (*mTimings.find(name)).second;
    }

    /*!
     * \return A list of strings that \ref Timing objects are available for. See
     * also \ref getTiming and \ref addTiming
     */
    std::list<std::string> DebugLogEntry::getTimingKeys() const {
        std::list<std::string> keys;
        std::map<std::string, Timing>::const_iterator it;
        for (it = mTimings.begin(); it != mTimings.end(); ++it) {
            keys.push_back((*it).first);
        }
        return keys;
    }

    const std::map<std::string, Timing>& DebugLogEntry::getTimings() const {
        return mTimings;
    }

    void DebugLogEntry::setIntVariable(const std::string& key, int value) {
        std::map<std::string, int>::iterator it = mIntVariables.find(key);
        if (it != mIntVariables.end()) {
            (*it).second = value;
        } else {
            mIntVariables.insert(std::make_pair(key, value));
        }
    }

    void DebugLogEntry::setUIntVariable(const std::string& key, unsigned int value) {
        std::map<std::string, unsigned int>::iterator it = mUIntVariables.find(key);
        if (it != mUIntVariables.end()) {
            (*it).second = value;
        } else {
            mUIntVariables.insert(std::make_pair(key, value));
        }
    }

    /*!
     * \return TRUE if \ref setIntVariable has been called at least once with \p
     * key, otherwise FALSE.
     */
    bool DebugLogEntry::hasIntVariable(const std::string& key) const {
        if (mIntVariables.find(key) == mIntVariables.end()) {
            return false;
        }
        return true;
    }

    /*!
     * \return TRUE if \ref setUIntVariable has been called at least once with \p
     * key, otherwise FALSE.
     */
    bool DebugLogEntry::hasUIntVariable(const std::string& key) const {
        if (mUIntVariables.find(key) == mUIntVariables.end()) {
            return false;
        }
        return true;
    }

    /*!
     * \return The value that \p key has been assigned to by the last call to
     * \ref setIntVariable, or 0 if \ref setIntVariable was never called with
     * this key.
     */
    int DebugLogEntry::getIntVariable(const std::string& key) const {
        std::map<std::string, int>::const_iterator it = mIntVariables.find(key);
        if (it == mIntVariables.end()) {
            return 0;
        }
        return (*it).second;
    }

    /*!
     * \return The value that \p key has been assigned to by the last call to
     * \ref setUIntVariable, or 0 if \ref setUIntVariable was never called with
     * this key.
     */
    unsigned int DebugLogEntry::getUIntVariable(const std::string& key) const {
        std::map<std::string, unsigned int>::const_iterator it = mUIntVariables.find(key);
        if (it == mUIntVariables.end()) {
            return 0;
        }
        return (*it).second;
    }

    /*!
     * \return A list of valid int variable keys that this log entry knows about.
     * Valid keys are those where at least once \ref setIntVariable was used on
     * this object with that key.
     */
    std::list<std::string> DebugLogEntry::getIntVariableKeys() const {
        std::list<std::string> keys;
        std::map<std::string, int>::const_iterator it;
        for (it = mIntVariables.begin(); it != mIntVariables.end(); ++it) {
            keys.push_back((*it).first);
        }
        return keys;
    }

    /*!
     * \return A list of valid uint variable keys that this log entry knows about.
     * Valid keys are those where at least once \ref setUIntVariable was used on
     * this object with that key.
     */
    std::list<std::string> DebugLogEntry::getUIntVariableKeys() const {
        std::list<std::string> keys;
        std::map<std::string, unsigned int>::const_iterator it;
        for (it = mUIntVariables.begin(); it != mUIntVariables.end(); ++it) {
            keys.push_back((*it).first);
        }
        return keys;
    }


}


/*
 * vim: et sw=4 ts=4
 */
