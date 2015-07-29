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

#include "debugstream.h"
#include "math/vector.h"
#include "math/matrix.h"

#include <fstream>
#include <iostream>

#define REDIRECT_DEFAULT DebugStreamConfiguration::REDIRECT_STDOUT
#define REDIRECT_DEFAULT_FILE "dcollide_log.txt"



namespace dcollide {
    DebugStreamConfiguration* DebugStreamConfiguration::mDebugStreamConfiguration = 0;

    DebugStreamConfiguration::DebugStreamConfiguration() {
    }

    DebugStreamConfiguration::~DebugStreamConfiguration() {
        for (std::map<std::string, std::ofstream*>::iterator it = mFileName2Handle.begin(); it != mFileName2Handle.end(); ++it) {
            std::ofstream* stream = (*it).second;
            stream->close();
            delete stream;
        }
        mFileName2Handle.clear();
    }

    void DebugStreamConfiguration::initializeStatic() {
        if (!mDebugStreamConfiguration) {
            mDebugStreamConfiguration = new DebugStreamConfiguration();
            mDebugStreamConfiguration->initializeConfiguration();
        }
    }

    void DebugStreamConfiguration::destroyStatic() {
        delete mDebugStreamConfiguration;
        mDebugStreamConfiguration = 0;
    }

    DebugStreamConfiguration* DebugStreamConfiguration::getConfiguration() {
        if (!mDebugStreamConfiguration) {
            initializeStatic();
        }
        return mDebugStreamConfiguration;
    }

    void DebugStreamConfiguration::setRedirectLocation(int area, DebugStream::Severity severity, RedirectLocation location, const std::string& filename) {
        std::map<int, AreaConfig>::iterator it = mArea2Config.find(area);
        if (it == mArea2Config.end()) {
            mArea2Config.insert(std::make_pair(area, AreaConfig()));
            it = mArea2Config.find(area);
        }
        (*it).second.setRedirectLocation(severity, location, filename);
    }

    /*!
     * Primarily an internal method, used by \ref DebugStream.
     *
     * \return The redirect-location as set by \ref setRedirectLocation (or a
     * default one if none has been set for this severity/area pair).
     */
    DebugStreamConfiguration::RedirectLocation DebugStreamConfiguration::getRedirectLocation(int area, DebugStream::Severity severity) {
        std::map<int, AreaConfig>::iterator it = mArea2Config.find(area);
        if (it == mArea2Config.end()) {
            mArea2Config.insert(std::make_pair(area, AreaConfig()));
            return getRedirectLocation(area, severity);
        }
        return (*it).second.getRedirectLocation(severity);
    }

    std::string DebugStreamConfiguration::getRedirectFile(int area, DebugStream::Severity severity) {
        std::map<int, AreaConfig>::iterator it = mArea2Config.find(area);
        if (it == mArea2Config.end()) {
            mArea2Config.insert(std::make_pair(area, AreaConfig()));
            return getRedirectFile(area, severity);
        }
        return (*it).second.getRedirectFile(severity);
    }

    void DebugStreamConfiguration::AreaConfig::setRedirectLocation(DebugStream::Severity severity, RedirectLocation location, const std::string& filename) {
        if (location == REDIRECT_FILE) {
            if (filename.empty()) {
                location = REDIRECT_STDOUT;
            } else {
                mSeverity2File.insert(std::make_pair(severity, ""));
                std::map<DebugStream::Severity, std::string>::iterator it = mSeverity2File.find(severity);
                (*it).second = filename;
            }
        }
        mSeverity2Location.insert(std::make_pair(severity, REDIRECT_STDOUT));
        std::map<DebugStream::Severity, RedirectLocation>::iterator it = mSeverity2Location.find(severity);
        (*it).second = location;
    }

    DebugStreamConfiguration::RedirectLocation DebugStreamConfiguration::AreaConfig::getRedirectLocation(DebugStream::Severity severity) {
        std::map<DebugStream::Severity, RedirectLocation>::iterator it = mSeverity2Location.find(severity);
        if (it != mSeverity2Location.end()) {
            return (*it).second;
        }
        setRedirectLocation(severity, REDIRECT_DEFAULT, REDIRECT_DEFAULT_FILE);
        return getRedirectLocation(severity);
    }

    std::string DebugStreamConfiguration::AreaConfig::getRedirectFile(DebugStream::Severity severity) {
        std::map<DebugStream::Severity, std::string>::iterator it = mSeverity2File.find(severity);
        if (it != mSeverity2File.end()) {
            return (*it).second;
        }
        setRedirectLocation(severity, REDIRECT_DEFAULT, REDIRECT_DEFAULT_FILE);
        return getRedirectFile(severity);
    }

    /*!
     * \return A pointer to a \ref std::ostream object that operates on the opened file, if
     * \p fileName could be opened, otherwise a pointer to \ref std::cerr.
     */
    std::ostream* DebugStreamConfiguration::getRedirectFileStream(const std::string& fileName) {
        if (fileName.empty()) {
            return &std::cerr;
        }
        std::map<std::string, std::ofstream*>::iterator it = mFileName2Handle.find(fileName);
        if (it != mFileName2Handle.end()) {
            if ((*it).second) {
                return (*it).second;
            }
            return &std::cerr;
        }
        std::ofstream* file = new std::ofstream();
        file->open(fileName.c_str(), std::ios::out);
        if (!*file) {
            // AB: we cannot emit a debug message using debug() here, as it
            // might cause infinite recursion, if debug() is the one requesting
            // a stream.
            std::cout << "Could not open output file " << fileName << std::endl;

            delete file;
            file = 0;

            mFileName2Handle.insert(std::make_pair(fileName, (std::ofstream*)0));

            return &std::cerr;
        } else {
            mFileName2Handle.insert(std::make_pair(fileName, file));
        }
        return file;
    }


    void DebugStream::flush() {
        DebugStreamConfiguration::RedirectLocation location = DebugStreamConfiguration::getConfiguration()->getRedirectLocation(mArea, mSeverity);
        switch (location) {
            case DebugStreamConfiguration::REDIRECT_DEVNULL:
                break;
            case DebugStreamConfiguration::REDIRECT_STDOUT:
                std::cout << mStringStream.str() << std::endl;
                break;
            case DebugStreamConfiguration::REDIRECT_STDERR:
                std::cerr << mStringStream.str() << std::endl;
                break;
            case DebugStreamConfiguration::REDIRECT_FILE:
            {
                std::string file = DebugStreamConfiguration::getConfiguration()->getRedirectFile(mArea, mSeverity);
                std::ostream* stream = DebugStreamConfiguration::getConfiguration()->getRedirectFileStream(file);
                (*stream) << mStringStream.str() << std::endl;
                break;
            }

        }
        mStringStream.clear();
    }

    void DebugStream::initAreaPrefix() {
        if (mArea != 0) {
            // TODO: add an area prefix to AreaConfig which gets output here
            // instead of the number
            // -> certain submodules could register a number as their area and
            //    therefore make sure to be the only one using that area. e.g.
            //      debug(1) << "foo";
            //    is currently displayed as
            //      (1) foo
            //    it should be:
            //      (BroadPhase) foo
            //
            //    -> by using names, it can't happen that e.g. the spatialhash
            //       also uses area 1 (and the broadphase suddenly wonders where
            //       additional debug lines come from)
            //    -> if the same area is registered twice, an exception should
            //       be thrown.
            *this << "(" << mArea << ") ";
        }
    }

    /*!
     * Add a prefix to this stream, based on the severity.
     */
    void DebugStream::initSeverityPrefix() {
        switch (mSeverity) {
            case SEVERITY_DEBUG:
                break;
            case SEVERITY_INFO:
                *this << "INFO: ";
                break;
            case SEVERITY_WARNING:
                *this << "WARNING: ";
                break;
            case SEVERITY_ERROR:
                *this << "ERROR: ";
                break;
        }
    }

    /*!
     * \overload
     *
     * Write \p v into this stream.
     */
    DebugStream& DebugStream::operator<<(const Vector3& v) {
        *this << "(" << v.getX() << "," << v.getY() << "," << v.getZ() << ")";
        return *this;
    }

    /*!
     * \overload
     *
     * Write \m ino this stream.
     *
     * The \ref Matrix is encoded by a 4 lists of numbers, each describing one
     * row.
     */
    DebugStream& DebugStream::operator<<(const Matrix& m) {
        *this << "(";
        for (int i = 0; i < 4; i++) {
            *this << "["
                    << m.getElement(i, 0)
                    << ","
                    << m.getElement(i, 1)
                    << ","
                    << m.getElement(i, 2)
                    << ","
                    << m.getElement(i, 3)
                    << "]";
        }
        *this << ")";
        return *this;
    }
}

/*
 * vim: et sw=4 ts=4
 */
