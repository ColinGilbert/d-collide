/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
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

#include "testapp.h"
#include "scenes/scenemanager.h"
#include "testappcommandline.h"

#include <ode/odewrapper.h>

#include <iostream>

int main(int argc, char** argv) {
    SceneManager::setDefaultSceneId("general");

    TestApp app;

    TestAppCommandLine cmd;
    if (!cmd.parseAndApplyArgs(argc, argv, &app)) {
        return 0;
    }

    // AB: note that even if we use ODE collision detection, we still need to
    //     use MyODEGeom as interface.
    //     this is required because a lot of functions in MyObjectNode depend on
    //     it and we'd have to write two different versions of all that code
    //     otherwise.
    //
    //     also note that as a result of this, comparing the speed of ODE and
    //     d-collide directly with the current testapp design would be an UNFAIR
    //     comparison!
    //     in dGeomMoved() we always update MyODEGeom AND ODE's geom - in a real
    //     ODE application the MyODEGeom overhead (which includes updating
    //     d-collide) would not exist.
    //     however this slows down dGeomMoved() only, not the actual collision
    //     detection.
    MyODEGeom::initializeODECollisionFunctions();

    if (!app.initialize()) {
        std::cerr << "Could not initialize the application" << std::endl;
        return 1;
    }

    if (!cmd.applyArgsAfterInitialization(&app)) {
        return 0;
    }

    app.startEventLoop();

    return 0;
}

/*
 * vim: et sw=4 ts=4
 */
