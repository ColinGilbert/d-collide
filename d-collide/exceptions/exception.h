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

#ifndef DCOLLIDE_EXCEPTION_H
#define DCOLLIDE_EXCEPTION_H

//-------------------------------------
//-------#include directives-----------

#include <string>

//-------------------------------------
//-------using directives--------------

namespace dcollide {

    /*!
     * \brief
     * All Exceptions should derive from dcollide:Exception and provide a
     * meaningful error message.
     */

    class Exception {
        private:
            static const unsigned int MAX_BACKTRACE_SIZE = 25;
            
            unsigned int mBacktraceSize;
            char ** mBacktraceSymbols;
            void * mBacktraceAdresses[MAX_BACKTRACE_SIZE];
        protected:
            std::string mErrorMessage;

        public:
            Exception(const std::string& errorMessage);
            Exception(const Exception& e);
            virtual ~Exception();

            virtual const std::string& getErrorMessage();
            void printBacktrace();
    };


    /*!
     * \brief Every time a runtime type check fails, an object of type "TypeMismatchException" is thrown
     *
     */
    class TypeMismatchException : public Exception {
        public:
            TypeMismatchException(const std::string& errorMessage);
    };

    /*!
     * \brief Thrown if two objects from different \ref World objects are
     * combined
     */
    class InvalidWorldException : public Exception {
        public:
            InvalidWorldException(const std::string& errorMessage);
    };
    /*!
     *  \brief thrown if a yet unsupported shape was tried to adjust to
     *  Aabb AdjustTo-Method throws this exception if a yet unsupported shape
     * is given to it
     */
    class UnsupportedShapeException : public Exception {
        public:
            UnsupportedShapeException(const std::string& errorMessage);
    };

    /*!
     * \brief thrown if the mesh-deform function is called with a vector
     * of the wrong size
     * (meaning different size compared to the number of vertices)
     */
    class MeshDeformException : public Exception {
        public:
            MeshDeformException();
    };

    /*!
     *  \brief thrown if a Proxy is created with a MeshPart as Shape
     *  MeshParts are for internal use only
     */
    class MeshPartProxyException : public Exception {
        public:
            MeshPartProxyException();
    };

    /*!
     * \brief thrown if a mesh is created with invalid topology
     * Example: A triangle with less than three vertices
     */
    class MeshTopologyException : public Exception {
        public:
            MeshTopologyException(const std::string& errorMessage);
    };
    
    /*!
     * \brief thrown if a parameter which must be non-NULL is NULL
     */
    class NullPointerException : public Exception {
        public:
            NullPointerException(const std::string& nullPointerName);
    };

    /*!
     * \brief thrown if an invalid/out-of-range array-access is detected 
     */
    class ArrayIndexOutOfBoundsException : public Exception {
        public:
            ArrayIndexOutOfBoundsException(const std::string& arrayName, int size, int accessIndex);
    };
    
    //---------- Thread Exceptions---------------//
    
    /*!
     * \brief thrown if creation of a thread or a thread-related object fails
     */
    class ThreadInitializeException : public Exception {
        public:
            ThreadInitializeException(const std::string& reason, int errorCode);
    };
    
    /*!
     * \brief thrown if destruction of a thread or a thread-related object fails
     */
    class ThreadDestroyException : public Exception {
        public:
            ThreadDestroyException(const std::string& reason, int errorCode);
    };

    /*!
     * \brief thrown if mutex-locking fails
     */
    class MutexLockFailedException : public Exception {
        public:
            MutexLockFailedException(const std::string& reason);
    };

    /*!
     * \brief thrown if mutex-unlocking fails
     */
    class MutexUnlockFailedException : public Exception {
        public:
            MutexUnlockFailedException(const std::string& reason);
    };
    
    /*!
     * \brief thrown if rwlock locking fails
     */
    class ReadWriteLockException : public Exception {
        public:
            ReadWriteLockException(bool readLockFailed, const std::string& reason);
    };

    /*!
     * \brief thrown if rwlock unlocking fails
     */
    class ReadWriteUnlockException : public Exception {
        public:
            ReadWriteUnlockException(const std::string& reason);
    };
    
    /*!
     * \brief thrown if condition wait errors occur
     */
    class ConditionWaitException : public Exception {
        public:
            ConditionWaitException(const std::string& reason, int code);
    };
    
    /*!
     * \brief thrown if thread-join fails
     */
    class ThreadJoinException : public Exception {
        public:
            ThreadJoinException(const std::string& reason, int code);
    };
    
    /*!
     * \brief general thread exception
     * Thrown if something goes wrong in the multithreading code
     * that is not handled by more specific thread exceptions.
     * 
     * \ref ThreadInitializeException
     * \ref ThreadDestroyException
     * \ref ThreadInitializeException
     * \ref MutexLockFailedException
     * \ref MutexUnlockFailedException
     * \ref ReadWriteLockException
     * \ref ReadWriteUnlockException
     * \ref ConditionWaitException
     */
    class ThreadException : public Exception {
        public:
            ThreadException(const std::string& reason);
    };

    /*!
     * \brief General exception, when a value of an enum isn't supported
     * 
     * Thrown if the value of the enum isn't supported at this point.
     * For example the BroadPhaseFactry throws such an exceptuon, when the
     * enum is set to a type which isn't implemented yet.
     */
    class UnsupportedTypeException : public Exception {
        public:
            UnsupportedTypeException(const std::string& reason);
    };
}

#endif // DCOLLIDE_EXCEPTION_H

/*
 * vim: et sw=4 ts=4
 */
