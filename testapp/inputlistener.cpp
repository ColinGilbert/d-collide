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

#include "inputlistener.h"
#include "inputhandler.h"
#include "uiaction.h"
#include "dcollide-config_testapp.h"
#include "scenes/scenebase.h"

#include <OgreVector3.h>
#include <OgreCamera.h>

#include <CEGUI/CEGUISystem.h>

#define DCOLLIDE_NO_X11_GRAB

CEGUI::MouseButton convertOisMouseButtonToCegui(int buttonId) {
    switch (buttonId) {
        case 0: return CEGUI::LeftButton;
        case 1: return CEGUI::RightButton;
        case 2: return CEGUI::MiddleButton;
        case 3: return CEGUI::X1Button;
        default: return CEGUI::LeftButton;
    }
}


InputListener::InputListener(std::size_t winHandle, InputHandler* handler) {
    mInputHandler = handler;
    mMouseLook = false;

    OIS::ParamList params;
    std::ostringstream win;
    win << winHandle;
    params.insert(std::make_pair(std::string("WINDOW"), win.str()));

    // for debugging (gdb)
#ifdef DCOLLIDE_NO_X11_GRAB
    params.insert(std::make_pair(std::string("x11_keyboard_grab"), std::string("false")));
    params.insert(std::make_pair(std::string("x11_mouse_grab"), std::string("false")));
    params.insert(std::make_pair(std::string("XAutoRepeatOn"), std::string("true")));
#endif
    mInputManager = OIS::InputManager::createInputSystem(params);

    // mMouseObject = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, false));
    mMouseObject = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, true));
    mKeyboardObject = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));

    mMouseObject->setEventCallback(this);
    mKeyboardObject->setEventCallback(this);
}

InputListener::~InputListener() {
    delete mMouseObject;
    delete mKeyboardObject;
    OIS::InputManager::destroyInputSystem(mInputManager);
}

void InputListener::setWindowSize(unsigned int width, unsigned int height) {
    if (mMouseObject) {
        const OIS::MouseState &state = mMouseObject->getMouseState();
        // AB: OIS::MouseState::width and height are declared mutable, so we can
        //     (and must) change it even in a const reference.
        //
        //     don't ever ask me what I think about this style.
        state.width = width;
        state.height = height;
    }
}

/*!
 * Called once per frame - triggers OIS to update the input device states.
 */
void InputListener::captureInput(float timeSinceLastFrame) {
    mMouseObject->capture();
    mKeyboardObject->capture();

    if (!mInputHandler->mCamera) {
        return;
    }

    const float moveBy = 150.0f * timeSinceLastFrame;
    Ogre::Vector3 moveVector(0, 0, 0);
    float rotationX = 0.0f;
    float rotationY = 0.0f;
    if (mKeyboardObject->isKeyDown(OIS::KC_UP)) {
        moveVector += Ogre::Vector3(0.0f, 0.0f, -moveBy);
    }
    if (mKeyboardObject->isKeyDown(OIS::KC_DOWN)) {
        moveVector += Ogre::Vector3(0.0f, 0.0f, moveBy);
    }
    if (mKeyboardObject->isKeyDown(OIS::KC_PGUP)) {
        moveVector += Ogre::Vector3(0.0f, moveBy, 0.0f);
    }
    if (mKeyboardObject->isKeyDown(OIS::KC_PGDOWN)) {
        moveVector += Ogre::Vector3(0.0f, -moveBy, 0.0f);
    }
    if (mKeyboardObject->isKeyDown(OIS::KC_LEFT)) {
        moveVector += Ogre::Vector3(-moveBy, 0.0f, 0.0f);
    }
    if (mKeyboardObject->isKeyDown(OIS::KC_RIGHT)) {
        moveVector += Ogre::Vector3(moveBy, 0.0f, 0.0f);
    }

    if (mMouseLook) {
        const static float rotateAmount = 0.1f;
        const OIS::MouseState& state = mMouseObject->getMouseState();
        
        if (state.X.rel != 0) {
            rotationY += -state.X.rel * rotateAmount;
        }
        if (state.Y.rel != 0) {
            rotationX += -state.Y.rel * rotateAmount;
        }
        
        mInputHandler->mCamera->pitch(Ogre::Degree(rotationX));
        mInputHandler->mCamera->yaw(Ogre::Degree(rotationY));
    }

    mInputHandler->mCamera->moveRelative(moveVector);
}

bool InputListener::keyPressed(const OIS::KeyEvent& event) {
    if (event.key == OIS::KC_1) {
        mInputHandler->setCameraDefaultView(InputHandler::CAMERA_SETUP_VIEW_FRONT);
    } else if (event.key == OIS::KC_2) {
        mInputHandler->setCameraDefaultView(InputHandler::CAMERA_SETUP_VIEW_SIDE);
    } else if (event.key == OIS::KC_3) {
        mInputHandler->setCameraDefaultView(InputHandler::CAMERA_SETUP_VIEW_TOP);
    } else if (event.key == OIS::KC_4) {
        mInputHandler->setCameraDefaultView(InputHandler::CAMERA_SETUP_VIEW_ISOMETRIC);
    } else if (event.key == OIS::KC_R) {
        mInputHandler->mScene->restart();
    } else if (event.key == OIS::KC_SPACE) {
        mInputHandler->mScene->action();
    }
    
    if (mKeyboardObject->isKeyDown(OIS::KC_PERIOD)) {
        //emulate stepping button
    	mInputHandler->doSingleStep();
    }
    return true;
}

bool InputListener::keyReleased(const OIS::KeyEvent& event) {
    if (event.key == OIS::KC_ESCAPE || event.key == OIS::KC_Q) {
        mInputHandler->queueQuit();
    }
    if (mInputHandler && mInputHandler->getActionCollection()) {
        mInputHandler->getActionCollection()->activateShortcut(event.key);
    }
    return true;
}

bool InputListener::mouseMoved(const OIS::MouseEvent& event) {
    CEGUI::System::getSingleton().injectMouseMove(event.state.X.rel,
                                                  event.state.Y.rel);
    CEGUI::System::getSingleton().injectMouseWheelChange(event.state.Z.rel); 
    return true;
}

bool InputListener::mousePressed(const OIS::MouseEvent&, OIS::MouseButtonID id) {
    if (id == OIS::MB_Right) {
        mMouseLook = true;
    }
    CEGUI::System::getSingleton().injectMouseButtonDown(convertOisMouseButtonToCegui(id));
    return true;
}

bool InputListener::mouseReleased(const OIS::MouseEvent&, OIS::MouseButtonID id) {
    if (id == OIS::MB_Right) {
        mMouseLook = false;
    }
    CEGUI::System::getSingleton().injectMouseButtonUp(convertOisMouseButtonToCegui(id));
    return true;
}

/*
 * vim: et sw=4 ts=4
 */
