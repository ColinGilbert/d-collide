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
// Code taken fromhttp://www.ogre3d.org/wiki/index.php/ManualObject_AxisObject
// and adapted to make it compile
//
//  Filename : AxisObject.cpp

#include "axisobject.h"
#include "Ogre.h"
#include "OgreMaterial.h"

using namespace Ogre;

void AxisObject::addBox(ManualObject* obj, Vector3 dim, Vector3 pos, ColourValue color, short boxMask) {
    if (!obj) {
        return;
    }

    obj->begin("Axis", Ogre::RenderOperation::OT_TRIANGLE_LIST); 

    dim/=2;

    Ogre::Real l = dim.x;
    Ogre::Real h = dim.y;
    Ogre::Real w = dim.z;
    
    obj->position(Ogre::Vector3(-l, h, w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(-l, -h, w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(l, -h, w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(l, h, w) + pos);

    obj->position(Ogre::Vector3(-l, h, -w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(-l, -h, -w) + pos); 
    obj->colour(color);
    obj->position(Ogre::Vector3(l, -h, -w) + pos);
    obj->colour(color);
    obj->position(Ogre::Vector3(l, h, -w) + pos);

    // front back
    if (boxMask & BOX_FRONT) {
        obj->quad(0, 1, 2, 3);
    }
    if (boxMask & BOX_BACK) {
        obj->quad(7, 6, 5, 4);
    }

    // top bottom
    if (boxMask & BOX_TOP) {
        obj->quad(0, 3, 7, 4);
    }
    if (boxMask & BOX_BOT) {
        obj->quad(2, 1, 5, 6);
    }

    // end caps
    if (boxMask & BOX_RIGHT) {
        obj->quad(1, 0, 4, 5);
    }
    if (boxMask & BOX_LEFT) {
        obj->quad(3, 2, 6, 7);
    }

    obj->end();
}

void AxisObject::addMaterial(const Ogre::String& mat, Ogre::ColourValue& clr, Ogre::SceneBlendType sbt) {
    static int init=false;
    if (init) {
        return;
    } else {
        init=true;
    }

    Ogre::MaterialPtr matptr = Ogre::MaterialManager::getSingleton().create(mat, "General"); 
    matptr->setReceiveShadows(false); 
    matptr->getTechnique(0)->setLightingEnabled(true);
    matptr->getTechnique(0)->getPass(0)->setDiffuse(clr); 
    matptr->getTechnique(0)->getPass(0)->setAmbient(clr); 
    matptr->getTechnique(0)->getPass(0)->setSelfIllumination(clr); 
    matptr->getTechnique(0)->getPass(0)->setSceneBlending(sbt);
    matptr->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    matptr->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
}

Ogre::ManualObject* AxisObject::createAxis(Ogre::SceneManager* scene, const Ogre::String& name, Ogre::Real scale) {
    const Ogre::String axisMaterialName = "Axis";
    Ogre::ColourValue colour(1,1,1,.75);
    addMaterial(axisMaterialName, colour, Ogre::SBT_TRANSPARENT_ALPHA);

    Ogre::ManualObject* axis    = scene->createManualObject(name); 

    Ogre::Real len=scale;
    Ogre::Real scl=len*.1;
    Ogre::Real loc=len/2+scl/2;
    Ogre::Real fade=.5;
    Ogre::Real solid=.8;
    
    addBox(axis, Vector3(len, scl, scl), Vector3(loc,0,0), ColourValue(0, 0, solid, solid), (BOX_ALL & ~BOX_RIGHT)); 
    addBox(axis, Vector3(len, scl, scl), Vector3(-loc,0,0), ColourValue(0, 0, fade, fade), (BOX_ALL & ~BOX_LEFT)); 

    addBox(axis, Vector3(scl, len, scl), Vector3(0,loc,0), ColourValue(0, solid, 0, solid), (BOX_ALL & ~BOX_BOT)); 
    addBox(axis, Vector3(scl, len, scl), Vector3(0,-loc,0), ColourValue(0, fade, 0, fade), (BOX_ALL & ~BOX_TOP)); 

    addBox(axis, Vector3(scl, scl, len), Vector3(0,0,loc), ColourValue(solid, 0, 0, solid), (BOX_ALL & ~BOX_BACK)); 
    addBox(axis, Vector3(scl, scl, len), Vector3(0,0,-loc), ColourValue(fade, 0, 0, fade), (BOX_ALL & ~BOX_FRONT)); 

    axis->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

    return axis;
}

/*
 * vim: et sw=4 ts=4
 */
