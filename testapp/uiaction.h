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

#ifndef DCOLLIDE_UIACTION_H
#define DCOLLIDE_UIACTION_H

#include <string>
#include <list>

namespace CEGUI {
    class Renderer;
    class System;
    class Window;
    class EventArgs;
    class Checkbox;
    class PushButton;
}

/*!
 * \internal
 */
class UiActionSlotBase {
    public:
        virtual ~UiActionSlotBase();
        virtual void call(bool parameter) = 0;
        virtual void call() = 0;
};

/*!
 * \internal
 */
template <class T> class UiActionSlot : public UiActionSlotBase {
    public:
        typedef void (T::*MemberFunctionVoidParameter)();
        typedef void (T::*MemberFunctionBoolParameter)(bool parameter);

    public:
        inline UiActionSlot(MemberFunctionVoidParameter function, T* object);
        inline UiActionSlot(MemberFunctionBoolParameter function, T* object);

        inline virtual void call();
        inline virtual void call(bool parameter);

    private:
        T* mObject;
        MemberFunctionVoidParameter mFunctionVoidParameter;
        MemberFunctionBoolParameter mFunctionBoolParameter;
};

template <class T> inline UiActionSlot<T>::UiActionSlot(MemberFunctionVoidParameter function, T* object) {
    mObject = object;
    mFunctionVoidParameter = function;
    mFunctionBoolParameter = 0;
}

template <class T> inline UiActionSlot<T>::UiActionSlot(MemberFunctionBoolParameter function, T* object) {
    mObject = object;
    mFunctionBoolParameter = function;
    mFunctionVoidParameter = 0;
}

template <class T> inline void UiActionSlot<T>::call(bool parameter) {
    if (mFunctionBoolParameter) {
        (mObject->*mFunctionBoolParameter)(parameter);
    }
}

template <class T> inline void UiActionSlot<T>::call() {
    if (mFunctionVoidParameter) {
        (mObject->*mFunctionVoidParameter)();
    }
}

class UiAction;
class UiActionCollection {
    public:
        UiActionCollection();
        ~UiActionCollection();

        void addAction(UiAction* action);

        void activateShortcut(int oisShortcut);

        void setChecked(const std::string& actionName, bool c);
        bool isChecked(const std::string& actionName) const;

        const std::list<UiAction*>& getActions() const;

    private:
        std::list<UiAction*> mActions;
};


/*!
 * \brief Action class for UI actions
 *
 * An UI "Action" is some kind of "one shot" event that the user has caused,
 * usually by clicking some button or pressing a shortcut (note: "moving the
 * mouse" is not a "one shot" event and thus not meant to be a UiAction).
 *
 * Such actions can provide different means to be activated, for example in a
 * typical graphical text editor there is a menu entry "file open" and a
 * shortcut "CTRl+O" which both do the same thing, i.e. are the same action.
 *
 * This class is meant to encapsulate such actions for our in the testapp and
 * provide several means to activate the action: in particular usually a CEGUI
 * widget and a keyboard shortcut (using OIS).
 *
 * See \ref UiToggleAction which implements the most important type of actions,
 * i.e. checkbox-type actions.
 *
 * \author Andreas Beckermann <b_mann@gmx.de>
 */
class UiAction {
    public:
        UiAction(UiActionCollection* collection, const std::string& actionName);
        virtual ~UiAction();

        const std::string& getName() const;

        virtual void setText(const std::string& text);
        const std::string& getText() const;

        void setOisShortcut(int shortcut);
        int getOisShortcut() const;

        std::string getOisShortcutString() const;

        virtual void activate();

    protected:
        void addSlot(UiActionSlotBase* slot);

    private:
        std::list<UiActionSlotBase*> mSlots;
        std::string mName;
        std::string mText;
        int mOisShortcut;
};

class UiPushButtonAction : public UiAction {
    public:
        UiPushButtonAction(UiActionCollection* collection, const std::string& actionName);
        virtual ~UiPushButtonAction();

        template<class T> inline void addActionCallback( void (T::*MemberFunction)(), T* object );

        virtual void activate();

    private:
        void executeActionCallbacks();
        bool ceguiCallbackStateChanged(const CEGUI::EventArgs&);

    private:
        CEGUI::PushButton* mButton;
        std::list<UiActionSlotBase*> mActionCallbacks;
};

/*!
 * \brief Toggle action class for UI actions
 *
 * \author Andreas Beckermann <b_mann@gmx.de>
 */
class UiToggleAction : public UiAction {
    public:
        UiToggleAction(UiActionCollection* collection, const std::string& actionName = "");
        ~UiToggleAction();

        void setChecked(bool c);
        bool isChecked() const;

        virtual void setText(const std::string& text);

        virtual void activate();

        void toggle();

        void forceToggleCallbackActivation();

        template<class T> inline void addToggleCallback( void (T::*MemberFunction)(), T* object );
        template<class T> inline void addToggleCallback( void (T::*MemberFunction)(bool parameter), T* object );

    private:
        void addToggleCallbackInternal(UiActionSlotBase*);
        bool ceguiCallbackStateChanged(const CEGUI::EventArgs&);
        void executeToggleCallbacks();

    private:
        std::list<UiActionSlotBase*> mToggleCallbacks;
        bool mChecked;

        CEGUI::Checkbox* mCheckBox;
};

template<class T> inline void UiPushButtonAction::addActionCallback( void (T::*MemberFunction)(), T* object ) {
    UiActionSlot<T>* s = new UiActionSlot<T>(MemberFunction, object);
    mActionCallbacks.push_back(s);
    addSlot(s);
}

template<class T> inline void UiToggleAction::addToggleCallback( void (T::*MemberFunction)(), T* object ) {
    addToggleCallbackInternal(new UiActionSlot<T>(MemberFunction, object));
}

template<class T> inline void UiToggleAction::addToggleCallback( void (T::*MemberFunction)(bool parameter), T* object ) {
    addToggleCallbackInternal(new UiActionSlot<T>(MemberFunction, object));
}

#endif
/*
 * vim: et sw=4 ts=4
 */
