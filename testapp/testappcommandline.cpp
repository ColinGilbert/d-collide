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

#include "testappcommandline.h"
#include "testapp.h"
#include "inputhandler.h"
#include "scenes/scenemanager.h"

#include <d-collide/debug.h>

#include <iostream>
#include <string>
#include <algorithm>

using namespace dcollide;

TestAppCommandLine::TestAppCommandLine() {
    addSwitch("no-gui", "", "Disable GUI rendering");

    addStringOption("maxframes", "m", "<count>", "Quit after <count> scene frames");
    addStringOption("scene", "s", "<id>", "Load scene with identifier <id> on startup");
    addSwitch("pause", "p", "Pause the scene right after start");
}

TestAppCommandLine::~TestAppCommandLine() {
}

/*!
 * Calls \ref parseCommandLinesArgs and applies the results to \p app.
 *
 * \return TRUE normally, FALSE if the application is meant to quit afterwards
 * (e.g. an invalid argument or because of --help).
 */
bool TestAppCommandLine::parseAndApplyArgs(int argc, char** argv, TestApp* app) {
    if (!parseCommandLinesArgs(argc, argv)) {
        return false;
    }

    app->mUseGUI = true;
    if (isOptionUsed("no-gui")) {
        app->mUseGUI = false;

        if (!isOptionUsed("maxframes")) {
            // if useGUI == false and maxFrameCount has not been set, we quit by default
            // after a certain number of frames
            app->mMaxFrameCount = 500;
        }
    }

    if (isOptionUsed("maxframes")) {
        std::string s = getStringValue("maxframes");
        int frames = atoi(s.c_str());
        if (frames < 0) {
            std::cerr << "ERROR: maxframes < 0: " << frames << std::endl;
            return false;
        }
        app->mMaxFrameCount = (unsigned int)frames;
    }

    if (isOptionUsed("scene")) {
        std::string s = getStringValue("scene");
        SceneManager::setDefaultSceneId(s);
    }

    return true;
}

bool TestAppCommandLine::applyArgsAfterInitialization(TestApp* app) {
    if (isOptionUsed("pause")) {
        app->mInputHandler->setChecked("Pause", true);
    }
    return true;
}

/*
 * vim: et sw=4 ts=4
 */
