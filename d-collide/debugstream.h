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


// TODO: make thread safe?
// -> slower, but would be very useful to us!
// --> i think speed is not so important for debugging for us
// TODO: area names (see initAreaPrefix() comments)
// TODO: move DebugStreamConfiguration to separate header
// TODO: DebugStreamConfiguration::setDefaultRedirect(int area, RedirectLocation location)
//       -> use this location, whenever a severity is used which has not been
//          used before
//          (same for setDefaultRedirectFile)
// TODO: DebugStreamConfiguration::setDefaultRedirect(RedirectLocation location)
//      -> replacement for REDIRECT_DEFAULT
//         (same for REDIRECT_DEFAULT_FILE)

#ifndef DCOLLIDE_DEBUGSTREAM_H
#define DCOLLIDE_DEBUGSTREAM_H

#include <string>
#include <sstream>
#include <map>

// Defined under Microsoft Windows based systems in WinErr.h
// As this leads to problems with the enumeration Severity below we undefine it
#undef SEVERITY_ERROR

#include "debug.h"


/*!
 * \file debugstream.h
 */

namespace dcollide {

    class Vector3;
    class Matrix;

    /*!
     * \brief Debug output class
     *
     * This class can be used for debug output. An object of this class is
     * usually created using \ref debug, \ref info, \ref warning or \ref error.
     *
     * It can then be used similar to std::cout, however a newline is added
     * automatically. Sample usage:
     * \code
     * debug() << "debug output";
     * \endcode
     *
     * This class knows the concept of debug "areas" which describe from which
     * module a debug message comes. The default area is 0, you can use
     * different numbers whenever you like - the debug output will be prefixed
     * with that number:
     * \code
     * debug(154) << "debug output from area 154";
     * // output would be:
     * // (154) debug output from area 154
     * \endcode
     *
     * By using the "severity" (usually by using e.g. \ref warning and \ref
     * error instead of \ref debug) you can distingush between debug output and
     * e.g. error messages. Errors and warnings also get a prefix ("ERROR: " and
     * "WARNING: ") so that they can be easily found in log files.
     *
     * The debug output of every debug area can be customized, e.g. you could
     * configure the debug output of the default area (0) to simply not be
     * displayed (i.e. be directed to /dev/null) or you could redirect the error
     * messages of area 0 to go to the file "error.log". This kind of
     * configuration is done using the \ref DebugStreamConfiguration class.
     *
     * ATTENTION:
     * When you want to use this class and Mircrosoft COM Error Codes in
     * WinError.h, please notice, that debugstream.h undefines SEVERITY_ERROR. 
     */
    class DebugStream {
        public:
            enum Severity {
                SEVERITY_DEBUG = 0,
                SEVERITY_INFO = 1,
                SEVERITY_WARNING = 2,
                SEVERITY_ERROR = 3
            };

        public:
            inline DebugStream();
            inline DebugStream(int area, Severity severity);
            inline DebugStream(const DebugStream&);
            inline ~DebugStream();


            inline DebugStream& operator<<(int n);
            inline DebugStream& operator<<(unsigned int n);
            inline DebugStream& operator<<(unsigned long n);
            inline DebugStream& operator<<(float f);
            inline DebugStream& operator<<(double f);
            inline DebugStream& operator<<(const char* s);
            inline DebugStream& operator<<(const std::string& s);
            inline DebugStream& operator<<(const void* p);

            // AB: operator<<() for some very basic d-collide classes.
            //     don't add more complex classes here, rather add them next to
            //     the class (see e.g. Vertex for an example)
            DebugStream& operator<<(const Vector3& v);
            DebugStream& operator<<(const Matrix& m);

        protected:
            void flush();
            void initAreaPrefix();
            void initSeverityPrefix();

        private:
            mutable bool mFlushOnDestruction;
            int mArea;
            Severity mSeverity;
            std::ostringstream mStringStream;
    };


    // internal class
    // TODO: move to a private header?
    // --> no need to include <map>
    /*!
     * \brief Internal class.
     *
     * This class configures where debug output of the \ref DebugStream goes to.
     *
     * Currently any non-default redirects are most conveniently done in \ref
     * initializeConfiguration, i.e. by modifying the code directly. However
     * they can also be done afterwards.
     */
    class DebugStreamConfiguration {
        public:
            enum RedirectLocation {
                REDIRECT_STDOUT = 0,
                REDIRECT_STDERR = 1,
                REDIRECT_DEVNULL = 2,
                REDIRECT_FILE = 3
            };

        public:
            ~DebugStreamConfiguration();

            static void initializeStatic();
            static void destroyStatic();
            static DebugStreamConfiguration* getConfiguration();

            void setRedirectLocation(int area, DebugStream::Severity severity, RedirectLocation location, const std::string& filename = "");
            RedirectLocation getRedirectLocation(int area, DebugStream::Severity severity); // AB: NOT const. will initialize defaults, if area was not yet used
            std::string getRedirectFile(int area, DebugStream::Severity severity); // AB: NOT const. will initialize defaults, if area was not yet used

            std::ostream* getRedirectFileStream(const std::string& filename);

        protected:
            class AreaConfig {
                public:
                    void setRedirectLocation(DebugStream::Severity severity, RedirectLocation location, const std::string& filename = "");
                    RedirectLocation getRedirectLocation(DebugStream::Severity severity);
                    std::string getRedirectFile(DebugStream::Severity severity);
                    std::string getRedirectFileHandle(DebugStream::Severity severity);

                private:
                    std::map<DebugStream::Severity, RedirectLocation> mSeverity2Location;
                    std::map<DebugStream::Severity, std::string> mSeverity2File;
            };

            void initializeConfiguration();

        private:
            DebugStreamConfiguration();

        private:
            static DebugStreamConfiguration* mDebugStreamConfiguration;
            std::map<int, AreaConfig> mArea2Config;
            std::map<std::string, std::ofstream*> mFileName2Handle;
    };

    /*!
     * Create a new debug stream with default area 0 and severity \ref
     * SEVERITY_DEBUG.
     *
     * You should normally not use this directly, but rather use \ref debug,
     * \ref info, \ref warning or \ref error
     */
    DebugStream::DebugStream()
            :
            mFlushOnDestruction(true),
            mArea(0),
            mSeverity(SEVERITY_DEBUG) {
        initAreaPrefix();
        initSeverityPrefix();
    }

    /*!
     * Create a new debug stream with area \p area and severity \p severity.
     *
     * You should normally not use this directly, but rather use \ref debug,
     * \ref info, \ref warning or \ref error
     */
    DebugStream::DebugStream(int area, Severity severity)
            :
            mFlushOnDestruction(true),
            mArea(area),
            mSeverity(severity) {
        initAreaPrefix();
        initSeverityPrefix();
    }

    /*!
     * Create a new debug stream as a copy of \p s
     */
    DebugStream::DebugStream(const DebugStream& s)
        :
            mFlushOnDestruction(true),
            mArea(s.mArea),
            mSeverity(s.mSeverity),
            mStringStream(s.mStringStream.str(), std::ios::app) {
        // this stream is now responsible for printing the string - not the old
        // one anymore
        // note that we have to bypass the const here :-(
        s.mFlushOnDestruction = false;
    }

    /*!
     * Destruct the stream. Usually \ref flush is called automatically here.
     */
    DebugStream::~DebugStream() {
        if (mFlushOnDestruction) {
            flush();
        }
    }

    /*!
     * Write the string \p s to this debug stream.
     *
     * Note that the stream only gets written to its location when \p flush is
     * called (e.g. when this stream is destructed).
     */
    inline DebugStream& DebugStream::operator<<(const std::string& s) {
        mStringStream << s;
        return *this;
    }

    /*!
     * \overload
     *
     * This version converts the number \p n to a string and writes it to this
     * stream. For example:
     * \code
     * debug() << 1;
     * // output: "1"
     * \endcode
     */
    inline DebugStream& DebugStream::operator<<(int n) {
        mStringStream << n;
        return *this;
    }

    /*!
     * \overload
     */
    inline DebugStream& DebugStream::operator<<(unsigned int n) {
        mStringStream << n;
        return *this;
    }

    /*!
     * \overload
     */
    inline DebugStream& DebugStream::operator<<(unsigned long s) {
        mStringStream << s;
        return *this;
    }

    /*!
     * \overload
     */
    inline DebugStream& DebugStream::operator<<(float f) {
        mStringStream << f;
        return *this;
    }

    /*!
     * \overload
     */
    inline DebugStream& DebugStream::operator<<(double f) {
        mStringStream << f;
        return *this;
    }

    /*!
     * \overload
     *
     * This version writes a C-style string to this stream, for example:
     * \code
     * debug() << "foo";
     * // output: "foo"
     * \endcode
     */
    inline DebugStream& DebugStream::operator<<(const char* s) {
        mStringStream << s;
        return *this;
    }


    /*!
     * \overload
     */
    inline DebugStream& DebugStream::operator<<(const void* p) {
        mStringStream << p;
        return *this;
    }




    // AB: implemented as functions (not methods), so that we can write
    //   debug() << foo;
    // instead of
    //   DebugStream::debug() << foo;
    /*!
     * Debug function that provides a more powerful replacement of std::cout.
     * Usage:
     * \code
     * debug() << "foo " << 2 << " bar";
     * // output will be "foo 2 bar"
     * \endcode
     *
     * Submodules (e.g. broadphase, spatialhash, ...) with a lot of debug output
     * may want to use their own debug area, which allows them to redirect the
     * output of that area to a different location than other output. TO do
     * this, simply provide the area number as parameter to this function:
     * \code
     * debug(1) << "debug output to area 1";
     * // output will be "(1) debug output to area 1"
     * \endocde
     *
     * Note that when using different severities than debug (e.g. by using \ref
     * info, \ref error or \ref warning), each severity of an area can be
     * redirected to a separate location (but does not have to).
     *
     * For redirecting output to a different location, see \ref
     * DebugStreamConfiguration.
     *
     * \param area An optional parameter to distinguish this debug output from
     * output of other areas. The default is area 0. The area is displayed in
     * front of every debug line, i.e. "(1) foo" instead of "foo" for area 1.
     * Areas can be redirected to 
     */
    inline DebugStream debug(int area = 0) {
        return DebugStream(area, DebugStream::SEVERITY_DEBUG);
    }

    /*!
     * Like \ref debug, but with \ref DebugStream::Severity \ref SEVERITY_INFO.
     * This will add an additional "INFO: " before the output.
     *
     * See \ref debug for details.
     */
    inline DebugStream info(int area = 0) {
        return DebugStream(area, DebugStream::SEVERITY_INFO);
    }
    /*!
     * Like \ref debug, but with \ref DebugStream::Severity \ref
     * SEVERITY_WARNING.
     * This will add an additional "WARNING: " before the output.
     *
     * See \ref debug for details.
     */
    inline DebugStream warning(int area = 0) {
        return DebugStream(area, DebugStream::SEVERITY_WARNING);
    }
    /*!
     * Like \ref debug, but with \ref DebugStream::Severity \ref SEVERITY_ERROR.
     * This will add an additional "ERROR: " before the output.
     *
     * See \ref debug for details.
     */
    inline DebugStream error(int area = 0) {
        return DebugStream(area, DebugStream::SEVERITY_ERROR);
    }

}

#endif
/*
 * vim: et sw=4 ts=4
 */

