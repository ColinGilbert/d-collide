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

#include "uiaction.h"

#include <d-collide/debugstream.h>

#include <CEGUI/CEGUIWindowManager.h>
#include <CEGUI/elements/CEGUICheckbox.h>
#include <CEGUI/elements/CEGUIPushButton.h>
#include <OISKeyboard.h>

#include <iostream>

#define CEGUI_CAST(windowName, classType, castTo) \
    (CEGUI::WindowManager::getSingleton().getWindow(windowName)->getType() == classType) ? \
    (static_cast<castTo>(CEGUI::WindowManager::getSingleton().getWindow(windowName))) : \
    0

UiActionSlotBase::~UiActionSlotBase() {
}


UiActionCollection::UiActionCollection() {
}

UiActionCollection::~UiActionCollection() {
    for (std::list<UiAction*>::iterator it = mActions.begin(); it != mActions.end(); ++it) {
        delete *it;
    }
}

/*!
 * Add an action to the collection. This is automatically called by the \ref
 * UiAction constructor, do not call manually.
 *
 * Ownership of the pointer is taken.
 */
void UiActionCollection::addAction(UiAction* action) {
    mActions.push_back(action);
}

void UiActionCollection::activateShortcut(int oisShortcut) {
    if (oisShortcut == OIS::KC_UNASSIGNED) {
        return;
    }
    for (std::list<UiAction*>::iterator it = mActions.begin(); it != mActions.end(); ++it) {
        if ((*it)->getOisShortcut() == oisShortcut) {
            (*it)->activate();
        }
    }
}

/*!
 * If \p actionName is a \ref UiToggleAction in this collection (see \ref
 * addAction) this function sets the \ref UiToggleAction::isChecked state (see
 * \ref UiToggleAction::setChecked) to \p c.
 *
 * If \p actionName is not a \ref UiToggleAction in this collection, this method
 * emits a warning and does nothing otherwise.
 */
void UiActionCollection::setChecked(const std::string& actionName, bool c) {
    if (actionName == "") {
        dcollide::warning() << dc_funcinfo << "cannot use empty name";
        return;
    }
    for (std::list<UiAction*>::const_iterator it = mActions.begin(); it != mActions.end(); ++it) {
        if ((*it)->getName() == actionName) {
            UiToggleAction* toggle = dynamic_cast<UiToggleAction*>((*it));
            if (toggle) {
                toggle->setChecked(c);
                return;
            }
        }
    }
    dcollide::warning() << dc_funcinfo << actionName << ": no such toggle action found";
}

/*!
 * \return TRUE if \p actionName is \ref UiToggleAction in this collection
 * (see \ref addAction) and \ref UiToggleAction::isChecked is TRUE. Otherwise
 * FALSE.
 */
bool UiActionCollection::isChecked(const std::string& actionName) const {
    if (actionName == "") {
        dcollide::warning() << dc_funcinfo << "cannot use empty name";
        return false;
    }
    for (std::list<UiAction*>::const_iterator it = mActions.begin(); it != mActions.end(); ++it) {
        if ((*it)->getName() == actionName) {
            UiToggleAction* toggle = dynamic_cast<UiToggleAction*>((*it));
            if (toggle) {
                return toggle->isChecked();
            }
        }
    }
    dcollide::warning() << dc_funcinfo << actionName << ": no such toggle action found";
    return false;
}

/*!
 * \return All actions managed by this collection. See \ref addAction.
 */
const std::list<UiAction*>& UiActionCollection::getActions() const {
    return mActions;
}

/*!
 * \param name A unique identifier for this action. This name is used internally
 * and is never displayed anywhere (i.e. should not get translated/i18n'ed). In
 * particular this string is used to chose a CEGUI widget for this action.
 */
UiAction::UiAction(UiActionCollection* collection, const std::string& name) {
    mName = name;
    collection->addAction(this);
    mOisShortcut = OIS::KC_UNASSIGNED;
}

UiAction::~UiAction() {
    for (std::list<UiActionSlotBase*>::iterator it = mSlots.begin(); it != mSlots.end(); ++it) {
        delete *it;
    }
}

const std::string& UiAction::getName() const {
    return mName;
}

void UiAction::addSlot(UiActionSlotBase* slot) {
    mSlots.push_back(slot);
}

void UiAction::setText(const std::string& text) {
    mText = text;
}

const std::string& UiAction::getText() const {
    return mText;
}

void UiAction::setOisShortcut(int s) {
    mOisShortcut = s;

    // update CEGUI shortcut display (if any)
    setText(getText());
}

int UiAction::getOisShortcut() const {
    return mOisShortcut;
}

std::string UiAction::getOisShortcutString() const {
    switch (((OIS::KeyCode)mOisShortcut)) {
        // AB: I am not sure about some of these keycodes (i.e. which button
        // they are), others simply should not be used at all and thus have no
        // name.
        // either way these are imho "not important" keys, they should not
        // matter to us.
        default:
        case OIS::KC_APPS:
        case OIS::KC_POWER:
        case OIS::KC_SLEEP:
        case OIS::KC_WAKE:
        case OIS::KC_WEBSEARCH:
        case OIS::KC_WEBFAVORITES:
        case OIS::KC_WEBREFRESH:
        case OIS::KC_WEBSTOP:
        case OIS::KC_WEBFORWARD:
        case OIS::KC_WEBBACK:
        case OIS::KC_MYCOMPUTER:
        case OIS::KC_MAIL:
        case OIS::KC_MEDIASELECT:
        case OIS::KC_DIVIDE:
        case OIS::KC_SYSRQ:
        case OIS::KC_RMENU:
        case OIS::KC_MUTE:
        case OIS::KC_CALCULATOR:
        case OIS::KC_PLAYPAUSE:
        case OIS::KC_MEDIASTOP:
        case OIS::KC_VOLUMEDOWN:
        case OIS::KC_VOLUMEUP:
        case OIS::KC_WEBHOME:
        case OIS::KC_AX:
        case OIS::KC_UNLABELED:
        case OIS::KC_NEXTTRACK:
        case OIS::KC_PREVTRACK:
        case OIS::KC_AT:
        case OIS::KC_KANJI:
        case OIS::KC_KANA:
        case OIS::KC_ABNT_C1:
        case OIS::KC_CONVERT:
        case OIS::KC_NOCONVERT:
        case OIS::KC_YEN:
        case OIS::KC_ABNT_C2:
        case OIS::KC_DECIMAL:
        case OIS::KC_OEM_102:
        case OIS::KC_ADD:
        case OIS::KC_SUBTRACT:
        case OIS::KC_NUMLOCK:
        case OIS::KC_SCROLL:
        case OIS::KC_MULTIPLY:
        case OIS::KC_LMENU:
        case OIS::KC_CAPITAL:
        case OIS::KC_APOSTROPHE:
        case OIS::KC_GRAVE:
        case OIS::KC_BACK:
        case OIS::KC_LBRACKET:
        case OIS::KC_RBRACKET:
        case OIS::KC_UNASSIGNED:
            return std::string();

        case OIS::KC_ESCAPE:
            return std::string("ESC");
        case OIS::KC_1:
            return std::string("1");
        case OIS::KC_2:
            return std::string("2");
        case OIS::KC_3:
            return std::string("3");
        case OIS::KC_4:
            return std::string("4");
        case OIS::KC_5:
            return std::string("5");
        case OIS::KC_6:
            return std::string("6");
        case OIS::KC_7:
            return std::string("7");
        case OIS::KC_8:
            return std::string("8");
        case OIS::KC_9:
            return std::string("9");
        case OIS::KC_0:
            return std::string("0");
        case OIS::KC_MINUS:
            return std::string("-");
        case OIS::KC_EQUALS:
            return std::string("=");
        case OIS::KC_TAB:
            return std::string("TAB");
        case OIS::KC_Q:
            return std::string("Q");
        case OIS::KC_W:
            return std::string("W");
        case OIS::KC_E:
            return std::string("E");
        case OIS::KC_R:
            return std::string("R");
        case OIS::KC_T:
            return std::string("T");
        case OIS::KC_Y:
            return std::string("Y");
        case OIS::KC_U:
            return std::string("U");
        case OIS::KC_I:
            return std::string("I");
        case OIS::KC_O:
            return std::string("O");
        case OIS::KC_P:
            return std::string("P");
        case OIS::KC_RETURN:
            return std::string("RETURN");
        case OIS::KC_LCONTROL:
            return std::string("Left CTRL");
        case OIS::KC_A:
            return std::string("A");
        case OIS::KC_S:
            return std::string("S");
        case OIS::KC_D:
            return std::string("D");
        case OIS::KC_F:
            return std::string("F");
        case OIS::KC_G:
            return std::string("G");
        case OIS::KC_H:
            return std::string("H");
        case OIS::KC_J:
            return std::string("J");
        case OIS::KC_K:
            return std::string("K");
        case OIS::KC_L:
            return std::string("L");
        case OIS::KC_SEMICOLON:
            return std::string(";");
        case OIS::KC_LSHIFT:
            return std::string("Left Shift");
        case OIS::KC_BACKSLASH:
            return std::string("\\");
        case OIS::KC_Z:
            return std::string("Z");
        case OIS::KC_X:
            return std::string("X");
        case OIS::KC_C:
            return std::string("C");
        case OIS::KC_V:
            return std::string("V");
        case OIS::KC_B:
            return std::string("B");
        case OIS::KC_N:
            return std::string("N");
        case OIS::KC_M:
            return std::string("M");
        case OIS::KC_COMMA:
            return std::string(",");
        case OIS::KC_PERIOD:
            return std::string(".");
        case OIS::KC_SLASH:
            return std::string("/");
        case OIS::KC_RSHIFT:
            return std::string("Right Shift");
        case OIS::KC_SPACE:
            return std::string("Space");
        case OIS::KC_F1:
            return std::string("F1");
        case OIS::KC_F2:
            return std::string("F2");
        case OIS::KC_F3:
            return std::string("F3");
        case OIS::KC_F4:
            return std::string("F4");
        case OIS::KC_F5:
            return std::string("F5");
        case OIS::KC_F6:
            return std::string("F6");
        case OIS::KC_F7:
            return std::string("F7");
        case OIS::KC_F8:
            return std::string("F8");
        case OIS::KC_F9:
            return std::string("F9");
        case OIS::KC_F10:
            return std::string("F10");
        case OIS::KC_NUMPAD7:
            return std::string("Numpad 7");
        case OIS::KC_NUMPAD8:
            return std::string("Numpad 8");
        case OIS::KC_NUMPAD9:
            return std::string("Numpad 9");
        case OIS::KC_NUMPAD4:
            return std::string("Numpad 4");
        case OIS::KC_NUMPAD5:
            return std::string("Numpad 5");
        case OIS::KC_NUMPAD6:
            return std::string("Numpad 6");
        case OIS::KC_NUMPAD1:
            return std::string("Numpad 1");
        case OIS::KC_NUMPAD2:
            return std::string("Numpad 2");
        case OIS::KC_NUMPAD3:
            return std::string("Numpad 3");
        case OIS::KC_NUMPAD0:
            return std::string("Numpad 0");
        case OIS::KC_F11:
            return std::string("F11");
        case OIS::KC_F12:
            return std::string("F12");
        case OIS::KC_F13:
            return std::string("F13");
        case OIS::KC_F14:
            return std::string("F14");
        case OIS::KC_F15:
            return std::string("F15");
        case OIS::KC_NUMPADEQUALS:
            return std::string("Numpad =");
        case OIS::KC_COLON:
            return std::string(":");
        case OIS::KC_UNDERLINE:
            return std::string("_");
        case OIS::KC_STOP:
            return std::string("");
        case OIS::KC_NUMPADENTER:
            return std::string("Numpad Enter");
        case OIS::KC_RCONTROL:
            return std::string("");
        case OIS::KC_NUMPADCOMMA:
            return std::string("Numpad ,");
        case OIS::KC_PAUSE:
            return std::string("Pause");
        case OIS::KC_HOME:
            return std::string("Home");
        case OIS::KC_UP:
            return std::string("Up");
        case OIS::KC_PGUP:
            return std::string("PgUp");
        case OIS::KC_LEFT:
            return std::string("Left");
        case OIS::KC_RIGHT:
            return std::string("Right");
        case OIS::KC_END:
            return std::string("End");
        case OIS::KC_DOWN:
            return std::string("Down");
        case OIS::KC_PGDOWN:
            return std::string("PgDown");
        case OIS::KC_INSERT:
            return std::string("Insert");
        case OIS::KC_DELETE:
            return std::string("Delete");
        case OIS::KC_LWIN:
            return std::string("Left Win/Meta");
        case OIS::KC_RWIN:
            return std::string("Left Right/Meta");
    }
    return std::string();
}

void UiAction::activate() {
    // TODO: "normal" (i.e. non-toggle) callbacks?
    //       atm only UiToggleAction does something useful.
    //       UiAction should be a non-toggle button-type action, e.g. for things
    //       like the "quit" button.
}

UiPushButtonAction::UiPushButtonAction(UiActionCollection* collection, const std::string& name)
        : UiAction(collection, name) {
    mButton = 0;

    if (!getName().empty()) {
        mButton = CEGUI_CAST(getName(), "DCollide/Button", CEGUI::PushButton*);
        if (mButton) {
            mButton->subscribeEvent(CEGUI::PushButton::EventClicked,
                    CEGUI::Event::Subscriber(&UiPushButtonAction::ceguiCallbackStateChanged, this));
            setText(mButton->getText().c_str());
        } else {
            std::cerr << dc_funcinfo << "could not find CEGUI element " << getName() << std::endl;
        }
    }
}

UiPushButtonAction::~UiPushButtonAction() {
}

void UiPushButtonAction::activate() {
    UiAction::activate();
    executeActionCallbacks();
}

void UiPushButtonAction::executeActionCallbacks() {
    for (std::list<UiActionSlotBase*>::iterator it = mActionCallbacks.begin(); it != mActionCallbacks.end(); ++it) {
        (*it)->call();
    }
}

bool UiPushButtonAction::ceguiCallbackStateChanged(const CEGUI::EventArgs&) {
    if (!mButton) {
        return true;
    }
    executeActionCallbacks();
    return true;
}

UiToggleAction::UiToggleAction(UiActionCollection* collection, const std::string& name)
        : UiAction(collection, name) {
    mChecked = false;
    mCheckBox = 0;

    if (!getName().empty()) {
        mCheckBox = CEGUI_CAST(getName(), "DCollide/Checkbox", CEGUI::Checkbox*);
        if (mCheckBox) {
            mChecked = mCheckBox->isSelected();
            mCheckBox->subscribeEvent(CEGUI::Checkbox::EventCheckStateChanged,
                    CEGUI::Event::Subscriber(&UiToggleAction::ceguiCallbackStateChanged, this));
            setText(mCheckBox->getText().c_str());
        } else {
            std::cerr << dc_funcinfo << "could not find CEGUI element " << getName() << std::endl;
        }
    }
}

UiToggleAction::~UiToggleAction() {
}

void UiToggleAction::setChecked(bool c) {
    if (mChecked == c) {
        return;
    }
    
    if (mCheckBox) {
        mCheckBox->setSelected(c);
    }
    mChecked = c;

    executeToggleCallbacks();
}

bool UiToggleAction::isChecked() const {
    return mChecked;
}

void UiToggleAction::setText(const std::string& text) {
    UiAction::setText(text);
    if (mCheckBox) {
        CEGUI::String string = text;
        std::string s = getOisShortcutString();
        if (!s.empty()) {
            if (!string.empty()) {
                string += " ";
            }
            string += "(";
            string += s;
            string += ")";
            mCheckBox->setText(string);
        }
    }
}

/*!
 * Calls \ref toggle
 */
void UiToggleAction::activate() {
    UiAction::activate();
    toggle();
}

void UiToggleAction::toggle() {
    setChecked(!isChecked());
}

/*!
 * Force execution of all registered toggel callbacks (see \ref
 * addToggleCallback).
 *
 * The internal state of the toggle action (i.e. the value of \ref isChecked) is
 * \em not changed by this.
 *
 * This function may be used to force initialization using the value of \ref
 * isChecked, by forcing the callbacks to be executed with that initial value.
 */
void UiToggleAction::forceToggleCallbackActivation() {
    executeToggleCallbacks();
}

bool UiToggleAction::ceguiCallbackStateChanged(const CEGUI::EventArgs&) {
    if (!mCheckBox) {
        return true;
    }
    
    //FIXME: CEGUI seems to have a bug here
    
    /* Normally we would evaluate the value of mCheckbox->isSelected() here,
     * but as CEGUI under Windows returns '1' every time you call the afore
     * mentioned method we need to do this workaround for now.
     */ 
    mChecked = !isChecked();
    executeToggleCallbacks();
    return true;
}

void UiToggleAction::addToggleCallbackInternal(UiActionSlotBase* slot) {
    mToggleCallbacks.push_back(slot);
    addSlot(slot);

    executeToggleCallbacks();
}

void UiToggleAction::executeToggleCallbacks() {
    for (std::list<UiActionSlotBase*>::iterator it = mToggleCallbacks.begin(); it != mToggleCallbacks.end(); ++it) {
        (*it)->call(isChecked());
        (*it)->call();
    }
}

/*
 * vim: et sw=4 ts=4
 */
