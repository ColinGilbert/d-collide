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

#include "inputhandler.h"
#include "inputlistener.h"
#include "uiaction.h"
#include "testapp.h"
#include "scenes/scenebase.h"
#include "dcollide-config_testapp.h"

#include <OgreCamera.h>

#include <d-collide/debug.h>
#include <d-collide/world.h>

#include <iostream>

/*!
 * Construct an OIS based event handler system.
 */
InputHandler::InputHandler(std::size_t winHandle) {
    mCamera = 0;
    mScene = 0;
    mActionCollection = new UiActionCollection();
    mInputListener = new InputListener(winHandle, this);

    mWantQuit = false;

    mShowBoundingVolumesAction = 0;
    mShowProxiesAction = 0;
    mPauseAction = 0;
    mUseCollisionCacheAction = 0;
    mShowHierarchicalGridAction = 0;
    mUseThreads = 0;
    mWantSingleStep = false;
}

InputHandler::~InputHandler() {
    delete mInputListener;
    delete mActionCollection;
}

UiActionCollection* InputHandler::getActionCollection() const {
    return mActionCollection;
}

/*!
 * Initialize \ref UiAction objects - in particular this method requires that
 * CEGUI is set up properly, so that buttons can be added.
 */
void InputHandler::initActions() {
    mShowProxiesAction = new UiToggleAction(getActionCollection(), "ShowProxies");
    mShowProxiesAction->addToggleCallback(&InputHandler::showAllProxies, this);
    mShowProxiesAction->setOisShortcut(OIS::KC_S);
    mShowProxiesAction->setChecked(true);

    mShowBoundingVolumesAction = new UiToggleAction(getActionCollection(), "ShowBoundingVolumes");
    mShowBoundingVolumesAction->addToggleCallback(&InputHandler::showAllBoundingVolumes, this);
    mShowBoundingVolumesAction->setOisShortcut(OIS::KC_B);

    mPauseAction = new UiToggleAction(getActionCollection(), "Pause");
    mPauseAction->setOisShortcut(OIS::KC_P);

    new UiToggleAction(getActionCollection(), "PauseMovements");

    mUseCollisionCacheAction = new UiToggleAction(getActionCollection(), "UseCollisionCache");
    mUseCollisionCacheAction->addToggleCallback(&InputHandler::setUseCollisionCache, this);
    mUseCollisionCacheAction->setOisShortcut(OIS::KC_C);

    mShowHierarchicalGridAction = new UiToggleAction(getActionCollection(),
        "ShowOctree");
    mShowHierarchicalGridAction->addToggleCallback(
        &InputHandler::setShowHierarchicalGrid, this);
    mShowHierarchicalGridAction->setOisShortcut(OIS::KC_O);

    mUseThreads = new UiToggleAction(getActionCollection(), "UseThreads");
    mUseThreads->setOisShortcut(OIS::KC_T);

    new UiToggleAction(getActionCollection(), "UsePipelining");

    UiToggleAction* narrowResults = new UiToggleAction(getActionCollection(), "NarrowPhaseResults");
    narrowResults->addToggleCallback(&InputHandler::setShowNarrowPhaseResults, this);
    narrowResults->setOisShortcut(OIS::KC_N);

    UiToggleAction* skipNarrowPhase = new UiToggleAction(getActionCollection(), "SkipNarrowPhase");
    skipNarrowPhase->setOisShortcut(OIS::KC_A); // N_a_rrowPhase

    UiToggleAction* skipMiddlePhase = new UiToggleAction(getActionCollection(), "SkipMiddlePhase");
    skipMiddlePhase->setOisShortcut(OIS::KC_M);

    UiPushButtonAction* stepButton = new UiPushButtonAction(getActionCollection(), "StepButton");
    stepButton->addActionCallback(&InputHandler::doSingleStep, this);
}

/*!
 * Called once per frame - triggers OIS to update the input device states.
 */
void InputHandler::captureInput(float timeSinceLastFrame) {
    if (mInputListener) {
        mInputListener->captureInput(timeSinceLastFrame);
    }
}

/*!
 * Request to quit the application. The request will be processed once the event
 * loop is re-entered.
 */
void InputHandler::queueQuit() {
    mWantQuit = true;
}

/*!
 * \return TRUE if the user indicated that he wants to quit the application.
 * Otherwise FALSE.
 */
bool InputHandler::getWantQuit() const {
    return mWantQuit;
}

/*!
 * Should be called when the window that this input handler is assigned to has
 * been closed. From that point on, no further input is being processed, \ref
 * captureInput is a noop.
 *
 * This is important on linux, to ensure proper destruction: the OIS objects
 * must be deleted before the window is destroyed.
 */
void InputHandler::notifyWindowClosed() {
    delete mInputListener;
    mInputListener = 0;
}

void InputHandler::setWindowSize(unsigned int width, unsigned int height) {
    if (mInputListener) {
        mInputListener->setWindowSize(width, height);
    }
}

void InputHandler::setCamera(Ogre::Camera* camera) {
    mCamera = camera;
}

void InputHandler::setScene(SceneBase* scene) {
    mScene = scene;
}

void InputHandler::setTestApp(TestApp* testapp) {
    mTestApp = testapp;
}

void InputHandler::showAllBoundingVolumes(bool show) {
    if (mScene) {
        mScene->setAllBoundingVolumesVisible(show);
    }
}

void InputHandler::showAllProxies(bool show) {
    if (mScene) {
        mScene->setAllProxiesVisible(show);
    }
}

bool InputHandler::getPause() const {
    return isChecked("Pause");
}

bool InputHandler::getUseCollisionCache() const {
    if (!mScene || !mScene->getCollisionWorld()) {
        return false;
    }
    return mScene->getCollisionWorld()->getUseCollisionCaching();
}

void InputHandler::setUseCollisionCache(bool c) {
    if (mScene && mScene->getCollisionWorld()) {
        mScene->getCollisionWorld()->setUseCollisionCaching(c);
    }
}

void InputHandler::setShowHierarchicalGrid(bool show) {
    if (mScene) {
        mScene->setHierarchicalGridVisible(show);
    }
}

void InputHandler::setShowNarrowPhaseResults(bool show) {
    if (mScene) {
        mScene->setNarrowPhaseResultsVisible(show);
    }
}

bool InputHandler::getUseThreads() const {
    if (mUseThreads) {
        return mUseThreads->isChecked();
    }
    return true;
}

/*!
 * See \ref UiActionCollection::setChecked
 */
void InputHandler::setChecked(const std::string& actionName, bool c) {
    mActionCollection->setChecked(actionName, c);
}

/*!
 * \return UiActionCollection::isChecked
 */
bool InputHandler::isChecked(const std::string& actionName) const {
    return mActionCollection->isChecked(actionName);
}

/*!
 * Force activation of all callbacks of the \ref UiToggleAction objects. See
 * \ref UiToggleAction::forceToggleCallbackActivation.
 *
 * This should be called whenever the current scene has been changed completely,
 * so that the current values of \ref UiToggleAction::isChecked of all toggle
 * objects can be applied to that scene.
 */
void InputHandler::forceToggleCallbackActivation() {
    const std::list<UiAction*>& actions = mActionCollection->getActions();
    for (std::list<UiAction*>::const_iterator it = actions.begin(); it != actions.end(); ++it) {
        UiToggleAction* toggle = dynamic_cast<UiToggleAction*>(*it);
        if (toggle) {
            toggle->forceToggleCallbackActivation();
        }
    }
}


void InputHandler::changeCurrentScene(const std::string& sceneId) {
    if (mTestApp) {
        mTestApp->changeCurrentScene(sceneId);
    }
}

void InputHandler::setCameraDefaultView(CameraSetup view){
    switch (view) {
        case CAMERA_SETUP_VIEW_FRONT: {
            mCamera->setPosition(Ogre::Vector3(200, 0, 0));
            mCamera->lookAt(Ogre::Vector3(0, 0, 0));
        } break;
        case CAMERA_SETUP_VIEW_SIDE: {
            mCamera->setPosition(Ogre::Vector3(0.0f, 200.0f, 0.0f));
            mCamera->lookAt(Ogre::Vector3(0, 0, 0));
        } break;
        case CAMERA_SETUP_VIEW_TOP: {
            mCamera->setPosition(Ogre::Vector3(0, 0, 200));
            mCamera->lookAt(Ogre::Vector3(0, 0, 0));
        } break;
        case CAMERA_SETUP_VIEW_ISOMETRIC: {
            mCamera->setPosition(Ogre::Vector3(200, 200, 200));
            mCamera->lookAt(Ogre::Vector3(0, 0, 0));
        } break;
    }

}

void InputHandler::doSingleStep() {
    setSingleStep(true);
}

void InputHandler::setSingleStep(bool s) {
    mWantSingleStep = s;
}

bool InputHandler::getSingleStep() const {
    return mWantSingleStep;
}

// TODO: windowResize() etc.
// -> mouse clipping area

/*
 * vim: et sw=4 ts=4
 */
