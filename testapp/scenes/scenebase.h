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

#ifndef DCOLLIDE_SCENEBASE_H
#define DCOLLIDE_SCENEBASE_H

#include <string>
#include <list>
#include <set>

namespace Ogre {
    class Root;
    class Light;
    class Camera;
    class SceneManager;
    class SceneNode;
    class ManualObject;
}

namespace dcollide {
    class World;
    class WorldParameters;
    class Proxy;
    class Mesh;
    class WorldCollisions;
    class Vector3;
    struct CollisionInfo;
    class WorldCollisions;
}

class OgreSceneObjectFactory;
class MyObjectNode;
class MyDeformableObjectNode;

class SceneBase {
    public:
        SceneBase(Ogre::Root* root);
        virtual ~SceneBase();

        virtual bool initScene();
        void beginNextSceneFrame();

        Ogre::Camera* getCamera() const;
        inline Ogre::SceneManager* getSceneManager() const;

        /*!
         * \return A short description (at most a few lines) about this scene,
         * what it is meant for and maybe what it is not meant for.
         *
         * It is recommended for subclasses to keep the description simple and
         * generic: don't add too many details, unless you are sure these
         * details will stay in this scene as-is (think of e.g. a description
         * like "two moving boxes" but at one point a third box is added. you
         * would have to change the description now, or keep a wrong
         * description. Rather use "moving boxes").
         *
         * The description is mean to be displayed on the screen.
         */
        virtual std::string getSceneDescription() const = 0;

        /*!
         * Start a new frame of the scene, i.e. moves all movable object by a bit.
         *
         * This is NOT necessarily equal to a graphic frame! In fact the scene frames
         * should depend on time only, not on CPU or GPU power as the graphic frames do.
         */
        
        virtual void applyCollisions(const dcollide::WorldCollisions& collisions);
        void setAllProxiesVisible(bool v);
        void setAllBoundingVolumesVisible(bool v);
        void setHierarchicalGridVisible(bool v);
        void setNarrowPhaseResultsVisible(bool v);

        void updateOgreRepresentations(bool forced=false);
        
        inline dcollide::World* getCollisionWorld() const;

        void removeAllObjects();
        void removeObject(MyObjectNode*);

        virtual void restart();
        virtual void action();

        virtual void calculateCollisions(unsigned int flags);
        virtual inline bool getUsePhysics() const;
        dcollide::WorldCollisions* getWorldCollisions() const;

    protected:
        /*!
         * \return The initial world size, i.e. the size that is given to the
         * \ref World constructor. This can be any size that seems appropriate
         * to your scene. You can change the world size on runtime using \ref
         * World::setWorldDimension.
         */
        virtual dcollide::Vector3 initialWorldDimension() const = 0;

        virtual void addInitialWorldParameters(dcollide::WorldParameters& parameters) const;
        virtual void addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const;

        /*!
         * Called by \ref initScene. Derived classes should implement
         * this to initialize their scene.
         */
        virtual bool initializeScene() = 0;

        /*!
         * Called by \ref startNextSceneFrame. Derived classes should implement
         * this to implement their movements.
         */
        virtual void startNextSceneFrame() = 0;

        MyObjectNode* create3dsModel(const std::string& file, int proxyType=-1) const;
        MyObjectNode* create3dsModelWithOneMesh(const std::string& file, int proxyType=-1) const;
        MyObjectNode* createPlyModel(const std::string& file, int proxyType=-1, double scale=1.0f) const;
        MyObjectNode* createOgreModel(const std::string& filename,
                                      const std::string& entityName,
                                      int proxyType=-1) const;
    
        void createManyRigidBoxes();
        virtual void addTopLevelObject(MyObjectNode* object);

        void setProxyAndChildsVisible(dcollide::Proxy* proxy, bool v);

        void recalculateHierarchicalGrid();

        inline Ogre::Root* getOgreRoot() const;
        inline OgreSceneObjectFactory* getOgreSceneObjectFactory() const;

        virtual void visualizeBroadPhaseCollisions(const std::list<MyObjectNode*>& broadPhaseCollisions, bool display);
        virtual void visualizeRigidBoundingVolumeCollisions(const std::list<Ogre::ManualObject*>& rigidBoundingVolumeCollisions, bool display);
        virtual void visualizeNarrowPhaseCollisions(const std::set<MyObjectNode*>& collisions, bool display);
        virtual void addNarrowPhaseCollision(MyObjectNode* o1, MyObjectNode* o2, const dcollide::CollisionInfo& c);

        void deleteCollisions();

    protected:
        Ogre::SceneManager* mSceneManager;
        dcollide::World* mCollisionWorld;
        Ogre::Camera* mOgreCamera;

        dcollide::WorldCollisions* mWorldCollisions;

    private:
        void updateOgreObject(MyObjectNode* object, bool forced=false);
        std::string getModelPath(const std::string& filename) const;
        void initToplevelObject(MyObjectNode* object);

    private:
        bool mInitializeSceneCalled;
        Ogre::Light* mOgreLight;
        Ogre::Root* mOgreRoot;
        Ogre::SceneNode* mBoundingVolumes;
        Ogre::SceneNode* mHierarchicalGridSceneNode;
        Ogre::SceneNode* mNarrowPhaseResults;
        bool mBoundingVolumesVisible;
        bool mNarrowPhaseResultsVisible;

        OgreSceneObjectFactory* mOgreSceneObjectFactory;

        std::list<MyObjectNode*> mBroadPhaseCollisions;
        std::list<Ogre::ManualObject*> mMiddlePhaseRigidCollisions;
        std::set<MyObjectNode*> mNarrowPhaseCollisionObjects;
};

Ogre::SceneManager* SceneBase::getSceneManager() const {
    return mSceneManager;
}

inline dcollide::World* SceneBase::getCollisionWorld() const {
    return mCollisionWorld;
}

inline Ogre::Root* SceneBase::getOgreRoot() const {
    return mOgreRoot;
}

inline OgreSceneObjectFactory* SceneBase::getOgreSceneObjectFactory() const {
    return mOgreSceneObjectFactory;
}
inline bool SceneBase::getUsePhysics() const{
    return false;
}
#endif
/*
 * vim: et sw=4 ts=4
 */
