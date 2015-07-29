/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *
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

#ifndef DCOLLIDE_GUI_H
#define DCOLLIDE_GUI_H

#include <list>
#include "framehistory.h"

namespace Ogre {
    class Root;
    class Camera;
    class SceneManager;
    class SceneNode;
    class RenderWindow;
}

namespace CEGUI {
    class Renderer;
    class System;
    class Window;
	class FrameWindow;
    class EventArgs;
    class Checkbox;
    class OgreCEGUIRenderer;
}

namespace dcollide {
    class World;
    class WorldCollisions;
    class Timing;
}

class DCollideScene;
class InputHandler;
class MyWindowEventListener;
class FrameHistory;
class UiToggleAction;

class Gui {
    public:
        Gui(Ogre::Root* root, Ogre::RenderWindow* w);
        ~Gui();

        void setInputHandler(InputHandler* h);
        void setSceneManager(Ogre::SceneManager* m);

        bool initialize();

        bool updateGui(const dcollide::World* world, bool pause);
        bool updateCollisionLabels(dcollide::World* world, const dcollide::WorldCollisions* collisions);
        void setSceneDescription(const std::string& text);
        void setSceneId(const std::string& text);
        
        void addFrameHistory(const FrameHistory*, bool extrapolate = false);

    protected:
        bool handleQuit(const CEGUI::EventArgs&);
        bool handleLegend(const CEGUI::EventArgs&);
        bool handleSceneChange(const CEGUI::EventArgs&);

        void updateStats();
        bool updateObjectCount(const dcollide::World* world);

    private:
        InputHandler* mInputHandler;

        Ogre::Root* mOgreRoot;
        Ogre::RenderWindow* mRenderWindow;

        CEGUI::OgreCEGUIRenderer* mGuiRenderer;
        CEGUI::System* mGuiSystem;
        CEGUI::Window* mGuiSheet;
		CEGUI::FrameWindow* mLegendWindow;

        std::list<const FrameHistory*> mFrameHistories;
        std::list<const FrameHistory*> mExtrapolateFrameHistories;

        FrameHistory mCollisionsHistory;
        FrameHistory mBroadPhaseHistory;
        FrameHistory mMiddlePhaseHistory;
        FrameHistory mNarrowPhaseHistory;
        FrameHistory mPipelineHistory;
};

#endif
/*
 * vim: et sw=4 ts=4
 */
