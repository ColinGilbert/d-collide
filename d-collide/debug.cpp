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

#include "debug.h"

namespace dcollide {
    /*!
     * The d-collide library resides in a namespace, unfortunately that makes
     * the output of __PRETTY_FUNCTION__ harder to read. This method removes all
     * "dcollide::" strings from \p string, as well as a few other "undesired"
     * substrings (usually STL related).
     */
    std::string Debug::simplifyPrettyFunction(const char* string) {
        const std::string dcollideString = "dcollide::";
        std::string s = string;
        std::string::size_type index = s.find(dcollideString, 0);
        while (index != std::string::npos) {
            s.erase(index, dcollideString.size());
            index = s.find(dcollideString, index);
        }

        const std::string stlAllocatorString = "std::allocator<";
        index = s.find(stlAllocatorString, 0);
        while (index != std::string::npos) {
            std::string::size_type removeEnd = s.find('>', index + stlAllocatorString.size());
            if (removeEnd != std::string::npos) {
                if (index >= 2) {
                    if (s[index - 1] == ' ' && s[index - 2] == ',') {
                        // stlAllocatorString is 2nd parameter in a template list. remove the ", " as well
                        index -= 2;
                    }
                }
                s.erase(index, removeEnd - index + 1);
            } else {
                // error. do not remove anything.
            }


            index = s.find(stlAllocatorString, index);
        }

        return s;
    }
}

/*
 * vim: et sw=4 ts=4
 */
