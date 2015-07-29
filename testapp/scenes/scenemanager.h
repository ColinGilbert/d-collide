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

#ifndef DCOLLIDE_SCENEMANAGER_H
#define DCOLLIDE_SCENEMANAGER_H

#include <string>
#include <list>
#include <map>

namespace dcollide {
}

namespace Ogre {
    class Root;
}

class SceneBase;

/*!
 * \brief Class to manage creation and deletion of \ref SceneBase derived
 * objects
 *
 * This class is meant to know about all \ref SceneBase derived classes and
 * provide means to create objects of these classes from "ID" strings.
 *
 * These ID strings could be provided by code (e.g. \ref setDefaultSceneId
 * provides an easy way to change the scene that should be created on program
 * startup) or for example could be configured by a GUI or stored in a
 * configuration file.
 *
 * \author Andreas Beckermann <b_mann@gmx.de>
 */
class SceneManager {
    public:
        ~SceneManager();

        static void createSceneManager();
        static SceneManager* getSceneManager();
        static void deleteSceneManager();
        static void setDefaultSceneId(const std::string& id);
        static const std::string& getDefaultSceneId();

        SceneBase* createScene(const std::string& id, Ogre::Root* root);
        void deleteScene(SceneBase*);
        std::string getSceneId(SceneBase* scene) const;
        std::string getSceneTitle(const std::string& sceneId) const;
        
        std::list<std::string> getAvailableSceneIds() const;
        std::list<std::string> getAvailableSceneTitles() const;
        
    private:
        SceneManager();

    private:
        static SceneManager* mSceneManager;
        static std::string mDefaultSceneId;

        std::map<std::string, SceneBase*> mScenes;
        std::map<std::string, std::string> mSceneTitles;
};

#endif
/*
 * vim: et sw=4 ts=4
 */
