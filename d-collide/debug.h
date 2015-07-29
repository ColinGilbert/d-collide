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



#ifndef DCOLLIDE_DEBUG_H
#define DCOLLIDE_DEBUG_H

#include <string>

/*!
 * \file debug.h
 * \brief File containing helper functions for debugging
 */

/*!
 * \def dc_funcinfo_long
 * This macro is meant as an alternative to dc_funcinfo. It outputs the complete
 * name of the function, including all namespaces.
 *
 * See \ref dc_funcinfo for details.
 */

/*! \def dc_funcinfo
 * This macro is meant to be used in debug output statements and prints the name
 * of the function the statement is in. In contrast to \ref dc_funcinfo_long
 * this macro also removes dcollide:: substrings and a few other undesired
 * substrings.
 *
 * Sample use:
 * \code
 * std::cout << dc_funcinfo << "this is a test" << std::endl;
 * \endcode
 */

#ifdef __GNUC__
    #define dc_funcinfo_long __PRETTY_FUNCTION__ << ": "
    #define dc_funcinfo dcollide::Debug::simplifyPrettyFunction(__PRETTY_FUNCTION__) << ": "
#else // __GNUC__
    #define dc_funcinfo_long __FILE__ << " at line " << __LINE__ << ": "
    #define dc_funcinfo __FILE__ << " at line " << __LINE__ << ": "
#endif // __GNUC__


/*!
 * \brief Macro to avoid unused parameter warnings
 *
 * A simple macro that makes sure the specified variable "is used" in some way,
 * so that the compiler does not give a "unused parameter" or similar warning.
 *
 * This macro may be useful whenever you want to declare a variable, but (by
 * intention) never use it. For example for documentation purposes.
 */
#define DCOLLIDE_UNUSED(x) (void)(x)

namespace dcollide {
    /*!
     * This class contains helper method for debugging
     */
    class Debug {
        public:
            static std::string simplifyPrettyFunction(const char* string);
    };
}

#endif
/*
 * vim: et sw=4 ts=4
 */

