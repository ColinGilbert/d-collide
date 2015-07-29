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

namespace dcollide {
    void DebugStreamConfiguration::initializeConfiguration() {
        // area 80000 is OGRE
        setRedirectLocation(80000, DebugStream::SEVERITY_DEBUG, REDIRECT_DEVNULL);
        setRedirectLocation(80000, DebugStream::SEVERITY_INFO, REDIRECT_DEVNULL);

        // area 80001 is CEGUI
        setRedirectLocation(80001, DebugStream::SEVERITY_DEBUG, REDIRECT_DEVNULL);
        setRedirectLocation(80001, DebugStream::SEVERITY_INFO, REDIRECT_DEVNULL);


        // area 501 is BroadPhaseHierarchicalGrid tree debugging
        setRedirectLocation(501, DebugStream::SEVERITY_DEBUG, REDIRECT_FILE,
            "broadphasehierarchicalgrid.txt");

        // 10 is SurfaceHierarchy
        setRedirectLocation(10, DebugStream::SEVERITY_DEBUG, REDIRECT_STDOUT);
        setRedirectLocation(10, DebugStream::SEVERITY_INFO, REDIRECT_STDOUT);
        setRedirectLocation(10, DebugStream::SEVERITY_WARNING, REDIRECT_STDOUT);
        setRedirectLocation(10, DebugStream::SEVERITY_ERROR, REDIRECT_STDOUT);
    }
}

/*
 * vim: et sw=4 ts=4
 */
