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

#include "gui.h"
#include "inputhandler.h"
#include "framehistory.h"
#include "scenes/scenemanager.h"
#include "dcollide-config_testapp.h"

#include <d-collide/world.h>
#include <d-collide/worldcollisions.h>
#include <d-collide/collisioninfo.h>
#include <d-collide/bvhnode.h>
#include <d-collide/timing.h>
#include <d-collide/debug.h>
#include <d-collide/debuglog.h>
#include <d-collide/debugstream.h>
#include <d-collide/broadphase/broadphase.h>
#include <d-collide/broadphase/broadphasecollisions.h>

#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreWindowEventUtilities.h>
#include <OgreFrameListener.h>
#include <OgreCEGUIRenderer.h>

#include <CEGUI/CEGUISystem.h>
#include <CEGUI/CEGUILogger.h>
#include <CEGUI/CEGUIWindow.h>
#include <CEGUI/CEGUIExceptions.h>
#include <CEGUI/CEGUISchemeManager.h>
#include <CEGUI/CEGUIWindowManager.h>
#include <CEGUI/elements/CEGUIPushButton.h>
#include <CEGUI/elements/CEGUICombobox.h>
#include <CEGUI/elements/CEGUIListboxTextItem.h>
#include <CEGUI/elements/CEGUIFrameWindow.h>

#include <iostream>

#include "do_benchmark_hack.h"

dcollide::DebugStream operator<<(dcollide::DebugStream stream, const CEGUI::String& string) {
    stream << string.c_str();
    return stream;
}

/*!
 * CEGUI on windows is compiled without rtti information. thus we can't
 * use dynamic_cast on CEGUI objects.
 * in order to not write many
 *   if (o->getType()=="..:") {x = static_cast<...>(o);}
 * statements over and over again, we use this macro.
 * The ?: operator is used so that we can use it in simply
 *   myVariable = CEGUI_CAST(...)
 * lines.
 *
 * This macor either returns a pointer of type \p castTo, or a NULL pointer,
 * depending whether \p windowName actually exists and is a window of type \p
 * classType
 */
#define CEGUI_CAST(windowName, classType, castTo) \
    (CEGUI::WindowManager::getSingleton().getWindow(windowName)->getType() == classType) ? \
    (static_cast<castTo>(CEGUI::WindowManager::getSingleton().getWindow(windowName))) : \
    0


// AB: atm a dummy logger that simply discards all log messages.
// this is atm mainly intended to stop cegui from creating a CEGUI.log without
// asking.
class MyCEGUILogger : public CEGUI::Logger {
    public:
        MyCEGUILogger();
        ~MyCEGUILogger();

        virtual void logEvent(const CEGUI::String& message, CEGUI::LoggingLevel level = CEGUI::Standard);
        virtual void setLogFilename(const CEGUI::String& filename, bool append = false);
};

MyCEGUILogger::MyCEGUILogger() {
}

MyCEGUILogger::~MyCEGUILogger() {
}

void MyCEGUILogger::logEvent(const CEGUI::String& message, CEGUI::LoggingLevel level) {
    const int ceguiDebugArea = 80001;

    switch (level) {
        case CEGUI::Errors:
            dcollide::error(ceguiDebugArea) << message;
            break;
        case CEGUI::Standard:
            dcollide::debug(ceguiDebugArea) << message;
            break;
        case CEGUI::Informative:
            dcollide::info(ceguiDebugArea) << message;
            break;
        default:
        case CEGUI::Insane:
            break;
    }
}

void MyCEGUILogger::setLogFilename(const CEGUI::String& filename, bool append) {
    DCOLLIDE_UNUSED(filename);
    DCOLLIDE_UNUSED(append);
}


class MyWindowEventListener : public Ogre::WindowEventListener {
    public:
        MyWindowEventListener(Ogre::RenderWindow* window) {
            mWindow = window;
            mInputHandler = 0;
            Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);
        }

        ~MyWindowEventListener() {
            Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
        }

        void setInputHandler(InputHandler* input) {
            mInputHandler = input;
        }

        virtual void windowResized(Ogre::RenderWindow* window) {
            if (window != mWindow) {
                return;
            }
            if (mInputHandler) {
                unsigned int width;
                unsigned int height;
                unsigned int colorDepth;
                int left;
                int top;
                window->getMetrics(width, height, colorDepth, left, top);
                mInputHandler->setWindowSize(width, height);
            }
        }

        virtual void windowClosed(Ogre::RenderWindow* window) {
            if (window != mWindow) {
                return;
            }
            if (mInputHandler) {
                mInputHandler->notifyWindowClosed();
            }
        }
    private:
        InputHandler* mInputHandler;
        Ogre::RenderWindow* mWindow;
};

/*!
 * Constructs a Gui object, however it does not initialize it. You must call
 * \ref initialize before using it.
 */
Gui::Gui(Ogre::Root* root, Ogre::RenderWindow* w) {
    mOgreRoot = root;
    mRenderWindow = w;
    mInputHandler = 0;
    mGuiRenderer = 0;
    mGuiSystem = 0;
    mGuiSheet = 0;

    // SET TO 0 TO ENABLE BENCHMARKING (and then start testapp using --maxframes <number>
#if !DCOLLIDE_DO_BENCHMARK_HACK
    const int maxMs = 1000; // we average over maxMs ms
#else
#warning HACK: setRemoveFramesOlderThan() uses large dummy value
    const int maxMs = 200000000; // we average over maxMs ms
#endif
    mCollisionsHistory.setRemoveFramesOlderThan(maxMs);
    mBroadPhaseHistory.setRemoveFramesOlderThan(maxMs);
    mMiddlePhaseHistory.setRemoveFramesOlderThan(maxMs);
    mNarrowPhaseHistory.setRemoveFramesOlderThan(maxMs);
    mPipelineHistory.setRemoveFramesOlderThan(maxMs);
}

Gui::~Gui() {
    delete mGuiSystem;
    delete mGuiRenderer;

    // AB: do NOT delete the MyCEGUILogger object
    //     -> CEGUI will automatically delete the singleton
}

void Gui::setInputHandler(InputHandler* h) {
    mInputHandler = h;
}

void Gui::setSceneManager(Ogre::SceneManager* m) {
    mGuiRenderer->setTargetSceneManager(m);
}

/*!
 * Setup the CEGUI-based user interface. This needs an OGRE::SceneManager.
 *
 * \return true on success, false otherwise
 */
bool Gui::initialize() {
    using namespace CEGUI;

    if (!mInputHandler) {
        std::cerr << dc_funcinfo << "ERROR: NULL input handler" << std::endl;
        return false;
    }

    // add resource path for GUI-scheme and layout
    mOgreRoot->addResourceLocation(DCOLLIDE_TESTAPP_INSTALL_PREFIX + std::string("/share/d-collide/resources/cegui-skin"), "FileSystem");
    mOgreRoot->addResourceLocation(DCOLLIDE_TESTAPP_SOURCE_PREFIX + std::string("/resources/cegui-skin"), "FileSystem");
    mOgreRoot->addResourceLocation("../resources/cegui-skin", "FileSystem");

    mOgreRoot->addResourceLocation(DCOLLIDE_TESTAPP_INSTALL_PREFIX + std::string("/share/d-collide/resources"), "FileSystem");
    mOgreRoot->addResourceLocation(DCOLLIDE_TESTAPP_SOURCE_PREFIX + std::string("/resources"), "FileSystem");
    mOgreRoot->addResourceLocation("../resources", "FileSystem");

    // AB: do NOT delete the object: CEGUI does so in CEGUI::System::~System()
    new MyCEGUILogger();

    mGuiRenderer = new OgreCEGUIRenderer(mRenderWindow);
    mGuiSystem = new System(mGuiRenderer);

    // load GUI scheme
    try {
        SchemeManager::getSingleton().loadScheme((utf8*)"DCollideSkin.scheme");
    } catch (CEGUI::Exception e) {
        std::cout << dc_funcinfo << "CEGUI exception when loading scheme: " << e.getMessage() << std::endl;
        return false;
    }

    // set default cursor and font
    mGuiSystem->setDefaultMouseCursor((utf8*)"DCollide", (utf8*)"MouseArrow");
    MouseCursor::getSingleton().setImage("DCollide", "MouseMoveCursor");
    mGuiSystem->setDefaultFont((utf8*)"Vera");
    
    // Move CEGUI mouse to (0,0)
    CEGUI::Point mousePos = CEGUI::MouseCursor::getSingleton().getPosition();  
    CEGUI::System::getSingleton().injectMouseMove(-mousePos.d_x,-mousePos.d_y);
    // load and enable GUI sheet
    try {
        mGuiSheet = WindowManager::getSingleton().loadWindowLayout("testapp.layout");
    } catch (CEGUI::Exception e) {
        std::cout << dc_funcinfo << "CEGUI exception when loading window layout: " << e.getMessage() << std::endl;
        return false;
    }
    mGuiSystem->setGUISheet(mGuiSheet);

    WindowManager::getSingleton().getWindow((utf8*)"QuitButton")
        ->subscribeEvent(
            PushButton::EventClicked,
            Event::Subscriber(&Gui::handleQuit, this));

    WindowManager::getSingleton().getWindow((utf8*)"LegendButton")
        ->subscribeEvent(
            PushButton::EventClicked,
            Event::Subscriber(&Gui::handleLegend, this));

    // fill the combobox with all available scenes
    Combobox* sceneBox = CEGUI_CAST("SceneSelectionBox", "DCollide/Combobox", CEGUI::Combobox*);
    if (!sceneBox) {
        std::cerr << dc_funcinfo << "cannot find TestApp/SceneSelectionBox" << std::endl;
        return false;
    }
    
    std::list<std::string> sceneTitles = SceneManager::getSceneManager()->getAvailableSceneTitles();
    unsigned int index = 0; // we use index in the list as ID for the combobox item
    colour white(1.0, 1.0, 1.0, 1.0);
    for (std::list<std::string>::iterator it = sceneTitles.begin(); it != sceneTitles.end(); ++it) {
        ListboxTextItem* item = new ListboxTextItem((utf8*)(*it).c_str(), index);
        
        // make the current selection in the combobox visible
        item->setSelectionColours(white);
        item->setTooltipText((*it).c_str());
        item->setSelectionBrushImage("DCollide", "ListboxSelectionBrush");
        
        if ((*it) == SceneManager::getDefaultSceneId()) {
            item->setSelected(true);
        }
        
        sceneBox->addItem(item);
        index++;
    }
    //sceneBox->setText(SceneManager::getDefaultSceneId().c_str());

    sceneBox->subscribeEvent(Combobox::EventListSelectionAccepted,
            Event::Subscriber(&Gui::handleSceneChange, this));
    
    mLegendWindow = CEGUI_CAST("LegendWindow", "DCollide/FrameWindow", CEGUI::FrameWindow*);
    mLegendWindow->hide();
    mLegendWindow->subscribeEvent(CEGUI::FrameWindow::EventCloseClicked,
                                  Event::Subscriber(&Gui::handleLegend, this));

    return true;
}

void Gui::addFrameHistory(const FrameHistory* h, bool extrapolate) {
    if (extrapolate) {
        mExtrapolateFrameHistories.push_back(h);
    } else {
        mFrameHistories.push_back(h);
    }
}

bool Gui::handleQuit(const CEGUI::EventArgs&) {
    if (mInputHandler) {
        mInputHandler->queueQuit();
    }
    return true;
}

bool Gui::handleLegend(const CEGUI::EventArgs&) {
    if (mLegendWindow->isVisible()) {
        mLegendWindow->hide();
    } else {
        mLegendWindow->show();
    }
    
    return true;
}

bool Gui::handleSceneChange(const CEGUI::EventArgs&) {
    CEGUI::Combobox* sceneBox = CEGUI_CAST("SceneSelectionBox", "DCollide/Combobox", CEGUI::Combobox*);
    if (!sceneBox) {
        std::cerr << dc_funcinfo << "cannot find TestApp/SceneSelectionBox" << std::endl;
        return false;
    }
    CEGUI::ListboxItem* item = sceneBox->getSelectedItem();
    if (!item) {
        return true;
    }
    unsigned int id = item->getID();
    std::list<std::string> scenes = SceneManager::getSceneManager()->getAvailableSceneIds();

    // AB: note: we don't use getText(), but rather the ID to identify the new
    //     scene.
    //     the text might not be the ID, but e.g. a i18n'ed name of it.
    std::string newScene;
    unsigned int i = 0;
    for (std::list<std::string>::iterator it = scenes.begin(); it != scenes.end(); ++it) {
        if (i == id) {
            newScene = *it;
            break;
        }
        i++;
    }

    if (newScene.empty()) {
        std::cout << dc_funcinfo << "scene with combobox ID=" << id << " not found" << std::endl;
        return true;
    }

    if (mInputHandler) {
        mInputHandler->changeCurrentScene(newScene);
    }

    sceneBox->setText(item->getText());

    return true;
}

/*!
 * Called once per frame to display the current state (i.e. update labels etc.
 *
 * \return TRUE on success, FALSE on failure, indicating that the program should
 * be aborted.
 */
bool Gui::updateGui(const dcollide::World* world, bool pause) {
    updateStats();
    //GJ: we update object count also in pause mode to update the labels on
    //    scene changes in pause mode
    return updateObjectCount(world);
}

/*!
 * Update stats displayed on screen (ie. fps).
 */
void Gui::updateStats() {
    using CEGUI::utf8;

    if (mFrameHistories.empty() && mExtrapolateFrameHistories.empty()) {
        return;
    }

    // we update every 2 seconds only
    dcollide::PointInTime t;
    t.addSeconds(-1);
    if (!mFrameHistories.empty() && !mFrameHistories.front()->getAverageLastCalled().isBefore(t)) {
        return;
    } else if (!mExtrapolateFrameHistories.empty() && !mExtrapolateFrameHistories.front()->getAverageLastCalled().isBefore(t)) {
        return;
    }

    std::stringstream ss;
    for (std::list<const FrameHistory*>::const_iterator it = mFrameHistories.begin(); it != mFrameHistories.end(); ++it) {
        if (it != mFrameHistories.begin()) {
            ss << "\n";
        }
        double avg = (*it)->getFpsAverage(false);
        ss.precision(2);
        ss << (*it)->getName() << ": " << std::fixed << avg << " fps";
    }
    for (std::list<const FrameHistory*>::const_iterator it = mExtrapolateFrameHistories.begin(); it != mExtrapolateFrameHistories.end(); ++it) {
        if (!mFrameHistories.empty() || it != mExtrapolateFrameHistories.begin()) {
            ss << "\n";
        }

        // AB: these FPS values are extrapolated only, i.e. we display the FPS
        // that _would_ have been achieved, if _only_ this task would have been
        // performed.
        // e.g. if we rendered 20 frames in one second, we have 20 FPS.
        //      but if each frame took 1 ms to be rendered, we _could_ have
        //      achieved 1000 FPS.
        double avg = (*it)->getFpsAverage(true);
        ss.precision(2);
        ss << (*it)->getName() << ": " << std::fixed << avg << " fps (extrapolated)";
    }

    CEGUI::WindowManager::getSingleton().getWindow(
            "FpsLabel")->setText(
                (utf8*)ss.str().c_str());
}

/*!
 * \return TRUE on success, FALSE on failure, indicating that the program should
 * be aborted.
 */
bool Gui::updateCollisionLabels(dcollide::World* world, const dcollide::WorldCollisions* worldCollisions) {
    using namespace CEGUI;

    if (!worldCollisions) {
        std::cerr << dc_funcinfo << "ERROR: NULL collisions" << std::endl;
        return false;
    }

    dcollide::DebugLogEntry* entry = 0;
    if (world->getDebugLog()) {
        entry = world->getDebugLog()->getMostRecentEntry();
    }
    if (entry && entry->hasTiming("calculateAllCollisions()")) {
        mCollisionsHistory.addManualFrame(entry->getTiming("calculateAllCollisions()"));
    }
    if (entry && entry->hasTiming("BroadPhase")) {
        mBroadPhaseHistory.addManualFrame(entry->getTiming("BroadPhase"));
    }
    if (entry && entry->hasTiming("MiddlePhase")) {
        mMiddlePhaseHistory.addManualFrame(entry->getTiming("MiddlePhase"));
    }
    if (entry && entry->hasTiming("NarrowPhase")) {
        mNarrowPhaseHistory.addManualFrame(entry->getTiming("NarrowPhase"));
    }
    if (entry && entry->hasTiming("Pipeline")) {
        mPipelineHistory.addManualFrame(entry->getTiming("Pipeline"));
    }

    // we update every 0.5 seconds only
    dcollide::PointInTime t;
    t.addMs(-500);
    if (!mCollisionsHistory.getAverageLastCalled().isBefore(t)) {
        return true;
    }

    Window* collisionsHeader = WindowManager::getSingleton().getWindow(
            (utf8*)"CollisionResultsHeader");
    Window* broadphaseLabel = WindowManager::getSingleton().getWindow(
            (utf8*)"CollisionsBroadphase");
    Window* middlephaseLabel = WindowManager::getSingleton().getWindow(
            (utf8*)"CollisionsMiddlephase");
    Window* narrowphaseLabel = WindowManager::getSingleton().getWindow(
            (utf8*)"CollisionsNarrowphase");
    Window* pipelineLabel = WindowManager::getSingleton().getWindow(
            (utf8*)"TimingPipeline");
    if (!collisionsHeader) {
        std::cerr << dc_funcinfo << "ERROR: NULL collisionsHeader label" << std::endl;
        return false;
    }
    if (!broadphaseLabel) {
        std::cerr << dc_funcinfo << "ERROR: NULL broadphase label" << std::endl;
        return false;
    }
    if (!middlephaseLabel) {
        std::cerr << dc_funcinfo << "ERROR: NULL middlephase label" << std::endl;
        return false;
    }
    if (!narrowphaseLabel) {
        std::cerr << dc_funcinfo << "ERROR: NULL narrowphase label" << std::endl;
        return false;
    }
    if (!pipelineLabel) {
        std::cerr << dc_funcinfo << "ERROR: NULL pipeline label" << std::endl;
        return false;
    }
    std::stringstream collisionsHeaderText;
    std::stringstream broadphaseText;
    std::stringstream middlephaseText;
    std::stringstream narrowphaseText;
    std::stringstream pipelineText;
    collisionsHeaderText << "Collisions";
    broadphaseText << "Broadphase: ";
    middlephaseText << "Middlephase: ";
    narrowphaseText << "Narrowphase: ";
    pipelineText << "Pipeline";


    const dcollide::BroadPhaseCollisions* broadCollisions = worldCollisions->getBroadPhaseCollisions();
    const std::list<dcollide::BoundingVolumeCollision>& rigidCollisions = worldCollisions->getRigidBoundingVolumeCollisions();
    const std::list<dcollide::BoundingVolumeCollision>& deformableCollisions = worldCollisions->getDeformableBoundingVolumeCollisions();
    const std::list<dcollide::CollisionInfo>& narrowCollisions = worldCollisions->getNarrowPhaseCollisions();

    // AB: we always display the counts of the most current frame, only the
    // times is averaged!
    broadphaseText << broadCollisions->getResults().size();
    middlephaseText << (rigidCollisions.size() + deformableCollisions.size());
    narrowphaseText << narrowCollisions.size();

    collisionsHeaderText << " (total time: ";
    broadphaseText << "  time: ";
    middlephaseText << "  time: ";
    narrowphaseText << "  time: ";
    pipelineText << " time: ";

    collisionsHeaderText << mCollisionsHistory.getUsAverage() << "us)";


    // FIXME: we display "not available" when the values are not available in
    // the _current_ frame, but when they are, we display an average over the
    // previous few frames!
    // -> if you switch between pipelined and non-pipeline mode, you have very
    //    bad numbers for a few frames!
    //    --> TODO: clear the history when we switch between pipelined and
    //              non-pipelined mode
    //              ideally even clear e.g. the middlephase history when we
    //              switch between skipping and not-skipping of the middlephase.
    //              maybe keep a boolean at this point whether the values for x
    //              (with x being, broad/middle/narrow phase and pipeline) were
    //              displayed last frame and if that boolean differs from the
    //              current value, clear the history (except for the most recent
    //              value)?

#if DCOLLIDE_DO_BENCHMARK_HACK
    mCollisionsHistory.setDestructionTextName_Hack("Collisions");
#endif

    if (entry && entry->hasTiming("BroadPhase")) {
#if DCOLLIDE_DO_BENCHMARK_HACK
        mBroadPhaseHistory.setDestructionTextName_Hack("BroadPhase");
#endif
        broadphaseText << mBroadPhaseHistory.getUsAverage() << "us";
    } else {
        mBroadPhaseHistory.setDestructionTextName_Hack("");
        broadphaseText << "(not available)";
    }
    if (entry && entry->hasTiming("MiddlePhase")) {
#if DCOLLIDE_DO_BENCHMARK_HACK
        mMiddlePhaseHistory.setDestructionTextName_Hack("MiddlePhase");
#endif
        middlephaseText << mMiddlePhaseHistory.getUsAverage() << "us";
    } else {
        mMiddlePhaseHistory.setDestructionTextName_Hack("");
        middlephaseText << "(not available)";
    }
    if (entry && entry->hasTiming("NarrowPhase")) {
#if DCOLLIDE_DO_BENCHMARK_HACK
        mNarrowPhaseHistory.setDestructionTextName_Hack("NarrowPhase");
#endif
        narrowphaseText << mNarrowPhaseHistory.getUsAverage() << "us";
    } else {
        mNarrowPhaseHistory.setDestructionTextName_Hack("");
        narrowphaseText << "(not available)";
    }
    if (entry && entry->hasTiming("Pipeline")) {
#if DCOLLIDE_DO_BENCHMARK_HACK
        mPipelineHistory.setDestructionTextName_Hack("Pipeline");
#endif
        pipelineText << mPipelineHistory.getUsAverage() << "us";
    } else {
        mPipelineHistory.setDestructionTextName_Hack("");
        pipelineText << "(not available)";
    }



    collisionsHeader->setText((utf8*)collisionsHeaderText.str().c_str());
    broadphaseLabel->setText((utf8*)broadphaseText.str().c_str());
    middlephaseLabel->setText((utf8*)middlephaseText.str().c_str());
    narrowphaseLabel->setText((utf8*)narrowphaseText.str().c_str());
    pipelineLabel->setText((utf8*)pipelineText.str().c_str());

    return true;
}

bool Gui::updateObjectCount(const dcollide::World* world) {
    if (!world) {
        return false;
    }

    CEGUI::Window* proxies = CEGUI::WindowManager::getSingleton().getWindow("ProxyCount");
    if (!proxies) {
        std::cerr << dc_funcinfo << "NULL ProxyCount label" << std::endl;
        return false;
    }
    std::stringstream proxiesText;
    proxiesText << "Proxies (t/r/d): ";
    proxiesText << world->getTopLevelProxies().size();
    proxiesText << "/";
    proxiesText << world->getRigidProxies().size();
    proxiesText << "/";
    proxiesText << world->getDeformableProxies().size();
    proxies->setText(proxiesText.str().c_str());

    return true;
}

void Gui::setSceneDescription(const std::string& text) {
    CEGUI::Window* label = CEGUI_CAST("SceneDescription", "DCollide/StaticText", CEGUI::Window*);
    if (label) {
        label->setText(text);
    }
}

void Gui::setSceneId(const std::string& text) {
    CEGUI::Combobox* sceneBox = CEGUI_CAST("SceneSelectionBox", "DCollide/Combobox", CEGUI::Combobox*);
    if (!sceneBox) {
        std::cerr << dc_funcinfo << "cannot find SceneSelectionBox" << std::endl;
        return;
    }
    sceneBox->setText(SceneManager::getSceneManager()->getSceneTitle(text));
}

/*
 * vim: et sw=4 ts=4
 */
