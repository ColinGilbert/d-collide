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

#ifndef DCOLLIDE_MYOBJECTNODE_H
#define DCOLLIDE_MYOBJECTNODE_H

#include "dcollide-config_testapp.h" // fixes some compiler warnings caused by ogre

#include <d-collide/dcollide-global.h>
#include <d-collide/math/vector.h>
#include <d-collide/math/matrix.h>

#include <ode/ode.h>
#include <modelloader/textureinformation.h>

#include <list>

class MyODETestAppGeom;

namespace dcollide {
    class Proxy;
    class Shape;
    class BvhNode;
    class World;
    struct CollisionInfo;
}
namespace Ogre {
    class SceneNode;
    class ManualObject;
}

class MyObjectNode {
    public:
        MyObjectNode(dcollide::World* world, dcollide::Shape* shape, int proxyType = dcollide::PROXYTYPE_RIGID, bool useCulling = true, bool useTextures = true);
        MyObjectNode(dcollide::Proxy* proxy, bool useCulling = true, bool useTextures = true);
        virtual ~MyObjectNode();

        dcollide::Proxy* getProxy() const;

        void addChild(MyObjectNode* child);
        const std::list<MyObjectNode*>& getChildren() const;

        void resetTransformation();

        virtual void translate(float x, float y, float z, bool respectOrientation = true);
        virtual void translate(const dcollide::Vector3& translation, bool respectOrientation = true);
        virtual void setPosition(float x, float y, float z, bool movePhysicsBody = true);
        //void setPosition(const dcollide::Vector3& position, bool movePhysicsBody = true);

        virtual void rotate(float angle, float x, float y, float z, bool respectOrientation = true);
        virtual void rotate(const dcollide::Matrix& rotationMatrix, bool respectOrientation = true);
        void setRotation(float angle, float x, float y, float z, bool movePhysicsBody = true);
        virtual void setRotation(const dcollide::Matrix& rotationMatrix, bool movePhysicsBody = true);

        dcollide::Vector3 getTranslation() const;
        dcollide::Vector3 getWorldPosition() const;
        dcollide::Matrix getRotation() const;

        virtual inline bool isDeformable() const;

        // auto-movement functions
        // (may make implementation of simple (!) movements simpler - for
        // advanced movements, you should NOT use these)
        void setRepeatedMovement(const dcollide::Vector3& start, const dcollide::Vector3& end, float speed);
        void setRepeatedMovementSpeed(float speed);
        inline const dcollide::Vector3& getRepeatedMovementStart() const;
        inline const dcollide::Vector3& getRepeatedMovementEnd() const;
        inline float getRepeatedMovementSpeed() const;
        // auto movement functions end

        inline Ogre::SceneNode* getOgreSceneNode() const;
        inline bool hasOgreBVSceneNodes() const;
        inline Ogre::SceneNode* getOgreNarrowPhaseResultsSceneNode() const;

        void setOgreSceneNode(Ogre::SceneNode* node);
        void setOgreBVSceneNode(Ogre::SceneNode* node);
        void setOgreBVSceneNodes(const std::list<Ogre::SceneNode*>& nodes);
        void setOgreNarrowPhaseResultsSceneNode(Ogre::SceneNode* node);

        inline bool getBoundingVolumeDirty() const;
        void recalculateBoundingVolumeOgreObjects();

        inline bool getUseTextures() const;
        inline bool getUseCulling() const;
        inline const ModelLoader::TextureInformation& getTextureInformation() const;
        inline void setTextureInformation(const ModelLoader::TextureInformation& t);

        void setBroadPhaseCollision(bool c);
        void addNarrowPhaseCollision(const dcollide::CollisionInfo& c);
        void setPenetratingObject(bool isPenetrating);
        void setPenetratedObject(bool isPenetrated);
        void createNarrowPhaseCollisions();
        void clearNarrowPhaseCollisions();

        void updateMaterialForNarrowPhaseResults(bool showNarrowPhase);
        
        virtual void createPhysicsBody(dWorldID odeWorld, dReal totalMass);
        void setPhysicsBody(dBodyID body);
        dBodyID getPhysicsBody() const;
        dcollide::Vector3 calculateBodyPosition();
        
        bool mPrintDebugOutput;
        std::string mDebugOutputPrefix;
        inline virtual void printDebugOutput(bool b, std::string prefix = "");
        
    protected:
        void recalculateBoundingVolumeOgreObjects(const dcollide::BvhNode* node);

        static std::string generateUniqueObjectName();


    private:
        void init();
        void destroyOgreSceneHierarchy(Ogre::SceneNode* top);
        void destroyAllOgreMaterials(Ogre::SceneNode* top);

    protected:
        dcollide::World* mWorld;
        dcollide::Proxy* mProxy;
        std::list<MyObjectNode*> mChildren;

        std::string mUniqueObjectName;

        Ogre::SceneNode* mSceneNode;
//        Ogre::SceneNode* mBVSceneNode;
        std::list<Ogre::SceneNode*> mBVSceneNodes;
        Ogre::SceneNode* mNarrowPhaseResultsSceneNode;
        Ogre::ManualObject* mNarrowPhaseResultsObject;

        dcollide::Vector3 mMovementStart;
        dcollide::Vector3 mMovementEnd;
        float mMovementSpeed;

        bool mBoundingVolumeDirty;

        ModelLoader::TextureInformation mTextureInformation;
        bool mUseCulling;
        bool mUseTextures;

        std::list<dcollide::CollisionInfo> mNarrowPhaseResults;
        bool mNarrowPhaseIsPenetrating;
        bool mNarrowPhaseIsPenetrated;

        // ODE:
        MyODETestAppGeom* mODEGeom;
        bool mUseODE;
};


Ogre::SceneNode* MyObjectNode::getOgreSceneNode() const {
    return mSceneNode;
}

/*!
 * \return TRUE if this object has OGRE objects representing the \ref BvhNode
 * objects. See \ref setOgreBVSceneNodes. FALSE otherwise.
 */
bool MyObjectNode::hasOgreBVSceneNodes() const {
    return !mBVSceneNodes.empty();
}

Ogre::SceneNode* MyObjectNode::getOgreNarrowPhaseResultsSceneNode() const {
    return mNarrowPhaseResultsSceneNode;
}

inline const dcollide::Vector3& MyObjectNode::getRepeatedMovementStart() const {
    return mMovementStart;
}

inline const dcollide::Vector3& MyObjectNode::getRepeatedMovementEnd() const {
    return mMovementEnd;
}

inline float MyObjectNode::getRepeatedMovementSpeed() const {
    return mMovementSpeed;
}

/*!
 * \return TRUE if the bounding volume may have changed (e.g. because the proxy
 * was moved, see \translate) since the last call to \ref
 * recalculateBoundingVolumeOgreObjects(). Otherwise FALSE.
 */
inline bool MyObjectNode::getBoundingVolumeDirty() const {
    return mBoundingVolumeDirty;
}

inline bool MyObjectNode::getUseCulling() const {
    return mUseCulling;
}

inline bool MyObjectNode::getUseTextures() const {
    return mUseTextures;
}

/*!
 * \return Information about the texture used by the shape of this MyObjectNode.
 * This has no meaning if this MyObjectNode has no shape.
 */
inline const ModelLoader::TextureInformation& MyObjectNode::getTextureInformation() const {
    return mTextureInformation;
}

/*!
 * Set the information about the texture of the shape of this MyObjectNode. See
 * \ref getTextureInformation
 */
inline void MyObjectNode::setTextureInformation(const ModelLoader::TextureInformation& t) {
    mTextureInformation = t;
}

inline bool MyObjectNode::isDeformable() const {
    return false;
}

inline void MyObjectNode::printDebugOutput(bool b, std::string prefix) {
    mPrintDebugOutput = b;
    mDebugOutputPrefix = prefix;
}

#endif

/*
 * vim: et sw=4 ts=4
 */
