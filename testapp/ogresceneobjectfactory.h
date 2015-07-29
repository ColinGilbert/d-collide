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

#ifndef DCOLLIDE_OGRESCENEOBJECTFACTORY_H
#define DCOLLIDE_OGRESCENEOBJECTFACTORY_H

#include "dcollide-config_testapp.h" // fixes some compiler warnings caused by ogre
#include <d-collide/math/vector.h>
#include <OgreString.h>

class MyObjectNode;

namespace Ogre {
    class SceneManager;
    class SceneNode;
    class ManualObject;
}

namespace dcollide {
    class Proxy;
    class Shape;
    class BvhNode;
    class BoundingVolume;
    class Aabb;
    class Obb;
    class Kdop;
    class BoundingSphere;
    class HierarchicalGrid;
}

namespace ModelLoader {
    class TextureInformation;
}


/*!
 * \brief Creates OGRE objects from d-collide proxies
 *
 * This class is meant to take a (toplevel) d-collide proxy(-hierarchy) and
 * generate an OGRE hierarchy from it consisting of \ref Ogre::SceneNode and
 * some kind of \ref Ogre::MovableObject (e.g. \ref Ogre::ManualObject) objects.
 *
 */
class OgreSceneObjectFactory {
    public:
        OgreSceneObjectFactory(Ogre::SceneManager* manager);
        virtual ~OgreSceneObjectFactory();

        Ogre::SceneNode* createSceneHierarchy(MyObjectNode* objectHierarchy);
        Ogre::SceneNode* createBoundingVolumeSceneHierarchy(const dcollide::BvhNode* boundingVolumeNode);
        Ogre::SceneNode* createHierarchicalGrid(
                const dcollide::HierarchicalGrid* grid);

        static void specifyBoundingVolumeLines(Ogre::ManualObject* object, const dcollide::BoundingVolume* bv);
        static void specifyHierarchicalGrid(Ogre::ManualObject* object,
                const dcollide::HierarchicalGrid* grid);
        static void specifyTrianglesForShape(Ogre::ManualObject* object, const dcollide::Shape* shape, const ModelLoader::TextureInformation& textureInformation, bool useTextures = true);

        static void initOgreObjectForBoundingVolume(Ogre::ManualObject* object, const dcollide::BoundingVolume* boundingVolume);

    protected:
        Ogre::String createUniqueObjectName(const dcollide::Shape* shape);
        Ogre::String createUniqueObjectName(const dcollide::BoundingVolume* boundingVolume);
        Ogre::ManualObject* createManualObject(const dcollide::Shape* shape, const ModelLoader::TextureInformation& textureInformation, bool useCulling = true, bool useTextures = true);
        Ogre::ManualObject* createManualObject(const dcollide::BoundingVolume* boundingVolume);
        static void initOgreObjectForAabb(Ogre::ManualObject* object, const dcollide::Aabb* bv);
        static void initOgreObjectForKdop(Ogre::ManualObject* object, const dcollide::Kdop* bv);
        static void initOgreObjectForSphere(Ogre::ManualObject* object, const dcollide::BoundingSphere* bv);
        static void initOgreObjectForObb(Ogre::ManualObject* object, const
                dcollide::Obb* obb);

        static void specifyBoundingVolumeLinesForAabb(Ogre::ManualObject* object, const dcollide::Aabb* bv);
        static void specifyBoundingVolumeLinesForObb(Ogre::ManualObject*
                object, const dcollide::Obb* obb);
        static void specifyBoundingVolumeLinesForKdop(Ogre::ManualObject* object, const dcollide::Kdop* bv);
        static void specifyBoundingVolumeTrianglesForSphere(Ogre::ManualObject* object, const dcollide::BoundingSphere* bv);

        static void specifyHierarchicalGridPoints(Ogre::ManualObject* object,
                const dcollide::HierarchicalGrid* grid);

    private:
        Ogre::SceneManager* mOgreSceneManager;
        static int mObjectNameNumber;
};

#endif // DCOLLIDE_OGRESCENEOBJECTFACTORY_H
/*
 * vim: et sw=4 ts=4
 */
