/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-users@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,          *
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

// Code taken from http://www.ogre3d.org/wiki/index.php/ManualObject_AxisObject
//
//
//  Filename : AxisObject.h

#ifndef DCOLLIDE_AXIS_OBJECT_H
#define DCOLLIDE_AXIS_OBJECT_H

#include "dcollide-config_testapp.h" // fixes some compiler warnings caused by ogre
#include <OgreString.h>
#include <OgreBlendMode.h>

namespace Ogre {
    class ManualObject;
    class SceneManager;
}

/*!
 *\brief shows coordinate system directions
 */
class AxisObject {
    enum BoxParts {
        BOX_NONE    = 0x00,
        BOX_TOP     = 0x01,
        BOX_BOT     = 0x02,
        BOX_FRONT   = 0x04,
        BOX_BACK    = 0x08,
        BOX_LEFT    = 0x10,
        BOX_RIGHT   = 0x20,
        BOX_ALL     = 0xFF
    };
    private:
        void addMaterial(const Ogre::String& mat, Ogre::ColourValue &clr, Ogre::SceneBlendType sbt);
        void addBox(Ogre::ManualObject* obj, Ogre::Vector3 dim, Ogre::Vector3 pos, Ogre::ColourValue color, short boxMask);

    public:
        Ogre::ManualObject*createAxis(Ogre::SceneManager* scene, const Ogre::String& name, Ogre::Real scale);
};

#endif //--_AXIS_OBJECT_H_

/*
 * vim: et sw=4 ts=4
 */
