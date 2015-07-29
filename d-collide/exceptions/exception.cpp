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

#include "exception.h"

#include "dcollide-config.h"

#ifdef HAVE_EXECINFO_H
#   include <execinfo.h>
#endif

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <cstdio>
namespace dcollide {
    Exception::Exception(const std::string& errorMessage)
            : mErrorMessage(errorMessage) {
#ifdef HAVE_EXECINFO_H
        // generate stack backtrace here
        mBacktraceSymbols = (char **)NULL;
        mBacktraceSize = backtrace(mBacktraceAdresses, MAX_BACKTRACE_SIZE);
        mBacktraceSymbols = backtrace_symbols(mBacktraceAdresses, mBacktraceSize);
#else
        mBacktraceSize = 0;
        mBacktraceSymbols = 0;
#endif
    }

    Exception::Exception(const Exception& e) {
        mBacktraceSize = e.mBacktraceSize;
        for (unsigned int i = 0; i < MAX_BACKTRACE_SIZE; i++) {
            mBacktraceAdresses[i] = e.mBacktraceAdresses[i];
        }
#ifdef HAVE_EXECINFO_H
        mBacktraceSymbols = backtrace_symbols(mBacktraceAdresses, mBacktraceSize);
#else
        mBacktraceSymbols = 0;
#endif
        mErrorMessage = e.mErrorMessage;
    }

    Exception::~Exception() {
        if (mBacktraceSymbols) {
            free(mBacktraceSymbols);
            mBacktraceSymbols = 0;
        }
    }

    /*! \brief Returns a textual description of what has gone wrong
     *  The exception class provides a standard behaviour; if a more complicated
     *  behaviour is needed, it is possible to redefine the getErrorMessage method
     *  in the appropriate subclass
     */
    const std::string& Exception::getErrorMessage() {
        return mErrorMessage;
    }

    /*!
     * \brief prints the framestack/backtrace generated in the constructor
     *
     * Quote from the backtrace_symbols documentation:
     * "Currently, the function name and offset only be obtained on systems that
     * use the ELF binary format for programs and libraries. On other systems,
     * only the hexadecimal return address will be present. Also, you may need
     * to pass additional flags to the linker to make the function names
     * available to the program. (For example, on systems using GNU ld, you must
     * pass (-rdynamic.)
     */
    void Exception::printBacktrace() {
        std::cout << "backtrace:" << std::endl;

#       ifdef HAVE_EXECINFO_H
            for (unsigned int i = 0; i < mBacktraceSize; i++) {
                char cmdline[1024 + 64];
                char fline[1024];
                char func[1024];
                char filename[1024];
                sprintf(fline,"??");
                sprintf(func,"??");
                if (mBacktraceSymbols[i]) {
                    char* fend = strpbrk(mBacktraceSymbols[i],"( ");
                    if (fend) {
                        int fsize = fend-mBacktraceSymbols[i];
                        if (fsize < 1023) {
                            strncpy(filename, mBacktraceSymbols[i], fsize);
                        } else {
                            fsize = 0;
                        }
                        filename[fsize] = 0;
                    }
                }
                sprintf(cmdline, "addr2line -e %s 0x%10.10lx", filename,
                        (unsigned long)mBacktraceAdresses[i]);
                FILE* p = popen(cmdline, "r");
                if (p) {
                    fgets(fline, 1023, p);
                    pclose(p);
                }
                sprintf(cmdline, "addr2line -C -f -e %s 0x%10.10lx", filename,
                        (unsigned long)mBacktraceAdresses[i]);
                p = popen(cmdline, "r");
                if (p) {
                    fgets(func, 1023, p);
                    pclose(p);
                }
                int c = 0;
                while (fline[c] != 0) {
                    if (fline[c] == 10) {
                        fline[c] = 0;
                        break;
                    }
                    c++;
                }
                c = 0;
                while (func[c] != 0) {
                    if (func[c] == 10) {
                        func[c] = 0;
                        break;
                    }
                    c++;
                }
                fprintf(stderr,"%s in function %s\n", fline, func);
            }
#       else
            std::cout << "(execinfo.h not found during compilation - no backtrace information available)" << std::endl;
#       endif
    }


    TypeMismatchException::TypeMismatchException(const std::string& errorMessage)
            : Exception(errorMessage) {
    }

    InvalidWorldException::InvalidWorldException(
            const std::string& errorMessage) : Exception(errorMessage) {
    }

    UnsupportedShapeException::UnsupportedShapeException(
            const std::string& errorMessage) : Exception(errorMessage) {
    }

    MeshDeformException::MeshDeformException() : Exception("") {
        std::stringstream s;
        s << "Deforming the Mesh failed";
        s << " because the vector size was wrong";
        mErrorMessage = s.str();
    }

    MeshPartProxyException::MeshPartProxyException()
            : Exception("") {
        std::stringstream s;
        s << "Cannot create Proxy with MeshPart as Shape - ";
        s << "MeshParts are for internal use only!";
        mErrorMessage = s.str();
    }

    MeshTopologyException::MeshTopologyException
        (const std::string& errorMessage) : Exception(errorMessage) {
    }

    NullPointerException::NullPointerException
            (const std::string& nullPointerName) : Exception("") {
        std::stringstream s;
        s << "Error: <<" << nullPointerName << ">> is NULL";
        mErrorMessage = s.str();
    }
    
    ArrayIndexOutOfBoundsException::ArrayIndexOutOfBoundsException(const std::string& arrayName, int size, int accessIndex)
        : Exception("") {
        std::stringstream errorStream;
        errorStream << "trying to access array " 
                << arrayName << "[" << accessIndex << "], ";
        if (size == 0) {
            errorStream << "but there are no valid indices for "
                << arrayName
                << " (size==0)";
        } else {
            errorStream
                    << "but valid indices are restricted to " 
                    << arrayName <<"[0] .. " << arrayName << "[" << (size-1) <<"].";
        }
        mErrorMessage = errorStream.str();
    }
    
    ThreadInitializeException::ThreadInitializeException(const std::string& reason,
                                                         int errorCode) 
    : Exception(""){
        std::stringstream errorStream;
        errorStream << "FATAL pthreads error: " << reason
                    << " pthreads error code:" << errorCode
                    << std::endl;
        mErrorMessage = errorStream.str();
    }
    
    ThreadDestroyException::ThreadDestroyException(const std::string& reason,
            int errorCode) 
    : Exception(""){
        std::stringstream errorStream;
        errorStream << "FATAL pthreads error: " << reason
                    << " pthreads error code:" << errorCode
                    << std::endl;
        mErrorMessage = errorStream.str();
    }
    
    MutexLockFailedException::MutexLockFailedException(const std::string& reason) 
        : Exception(""){
        std::stringstream s;
        s << "FATAL: Mutex locking failed. ";
        s << reason;
        mErrorMessage = s.str();
    }
    
    MutexUnlockFailedException::MutexUnlockFailedException(const std::string& reason) 
        : Exception(""){
        std::stringstream s;
        s << "FATAL: Mutex unlocking failed. ";
        s << reason;
        mErrorMessage = s.str();
    }
    
    ReadWriteLockException::ReadWriteLockException(bool readLockFailed, const std::string& reason)
        : Exception("") {
        std::stringstream s;
        if (readLockFailed) {
            s << "FATAL rwlock locking for read failed. ";
        } else {
            s << "FATAL rwlock locking for write failed. ";
        }
        s << reason;
        mErrorMessage = s.str();
    }
    
    ReadWriteUnlockException::ReadWriteUnlockException(const std::string& reason)
    : Exception("") {
        std::stringstream s;
        s << "FATAL: rwlock unlocking failed. ";
        s << reason;
        mErrorMessage = s.str();
    }
    
    ConditionWaitException::ConditionWaitException(const std::string& reason,
                                                    int errorCode) 
    : Exception(""){
        std::stringstream errorStream;
        errorStream << "FATAL: error in pthread_cond_wait(). " << reason
                    << " pthreads error code:" << errorCode
                    << std::endl;
        mErrorMessage = errorStream.str();
    }
    
    ThreadJoinException::ThreadJoinException(const std::string& reason,
            int errorCode) 
    : Exception(""){
        std::stringstream errorStream;
        errorStream << "FATAL: error in pthread_join(). " << reason
                << " pthreads error code:" << errorCode
                << std::endl;
        mErrorMessage = errorStream.str();
    }
    
    ThreadException::ThreadException(const std::string& reason) : Exception(""){
        mErrorMessage = reason;
    }

    UnsupportedTypeException::UnsupportedTypeException
        (const std::string& reason) : Exception("") {
    
        mErrorMessage = reason;
    }
}

/*
 * vim: et sw=4 ts=4
 */
