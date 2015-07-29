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

#ifndef DCOLLIDE_MYINPUTHANDLER_H
#define DCOLLIDE_MYINPUTHANDLER_H

#include <cstddef> // std::size_t
#include <string>

namespace Ogre {
    class Camera;
}

class InputListener;
class SceneBase;
class UiActionCollection;
class UiToggleAction;
class TestApp;

class InputHandler {
    enum CameraSetup {
        CAMERA_SETUP_VIEW_FRONT, //view from X-Axis to origin
        CAMERA_SETUP_VIEW_SIDE, //view from Y-Axis to origin
        CAMERA_SETUP_VIEW_TOP, //view from Z-Axis to origin
        CAMERA_SETUP_VIEW_ISOMETRIC //view from 200, 200, 200 to origin
    };

    public:
        InputHandler(std::size_t winHandle);
        virtual ~InputHandler();

        UiActionCollection* getActionCollection() const;
        void initActions();

        void setScene(SceneBase* scene);
        void setCamera(Ogre::Camera* camera);
        void setWindowSize(unsigned int width, unsigned int height);
        void setTestApp(TestApp* testApp);

        void showAllBoundingVolumes(bool);
        void showAllProxies(bool);
        void toggleShowAllBoundingVolumes();
        void toggleShowAllProxies();
        void toggleUseCollisionCaching();
        void toggleShowHierarchicalGrid();
        void setUseCollisionCache(bool);
        void setShowHierarchicalGrid(bool);
        void setShowNarrowPhaseResults(bool);
        void queueQuit();
        bool getWantQuit() const;
        bool getPause() const;
        bool getUseCollisionCache() const;
        bool getUseThreads() const;
        void changeCurrentScene(const std::string& sceneId);

        void doSingleStep();
        void setSingleStep(bool s);
        bool getSingleStep() const;

        void setCameraDefaultView(CameraSetup view);

        void setChecked(const std::string& actionName, bool);
        bool isChecked(const std::string& actionName) const;
        void forceToggleCallbackActivation();

        void captureInput(float timeSinceLastFrame);
        void notifyWindowClosed();

    private:
        friend class InputListener;
        InputListener* mInputListener;
        Ogre::Camera* mCamera;
        SceneBase* mScene;
        TestApp* mTestApp;

        bool mWantQuit;

        bool mWantSingleStep;

        UiActionCollection* mActionCollection;
        UiToggleAction* mShowBoundingVolumesAction;
        UiToggleAction* mShowProxiesAction;
        UiToggleAction* mPauseAction;
        UiToggleAction* mUseCollisionCacheAction;
        UiToggleAction* mShowHierarchicalGridAction;
        UiToggleAction* mUseThreads;
};

#endif

/*
 * vim: et sw=4 ts=4
 */
