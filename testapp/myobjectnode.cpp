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

#include "myobjectnode.h"
#include "mydeformableobjectnode.h"
#include "ogresceneobjectfactory.h"
#include "ogrematerials.h"
#include "odewrapper.h" 

#include <d-collide/world.h>
#include <d-collide/proxy.h>
#include <d-collide/bvhnode.h>
#include <d-collide/boundingvolumes/boundingvolume.h>
#include <d-collide/collisioninfo.h>
#include <d-collide/debugstream.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/exceptions/exception.h>
#include <d-collide/detectordeform/detectordeformmanager.h>
#include <d-collide/detectordeform/detectordeformproxydata.h>

#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreQuaternion.h>
#include <ode/odewrapper.h>

#include <iostream>
#include <math.h>

#include <d-collide/debugstream.h>

#define DO_OGRE_DEBUG 1

//GJ: scales the displayed length of visualized penetration vectors
//TODO the user should be able to change this at runtime
#define PENETRATION_DEPTH_SCALING_FACTOR 1

/*!
 * Create a new MyObjectNode object for \p shape. A \ref Proxy is created
 * containing \p shape, with type flags \p proxyType.
 *
 * \param proxyType The \ref dcollide::Proxy::getProxyType() flags. If negative
 * (the default) \ref dcollide::PROXYTYPE_RIGID is used.
 */
MyObjectNode::MyObjectNode(dcollide::World* world, dcollide::Shape* shape, int proxyType, bool useCulling, bool useTextures) {

    mWorld = world;
    mUseTextures = useTextures;
    mUseCulling = useCulling;
    mProxy = world->createProxy(shape, (dcollide::ProxyTypes)proxyType); // takes ownership of shape

    mUseODE = false;
    mODEGeom = 0;

    mPrintDebugOutput = false;
    mDebugOutputPrefix = "";

    init();
}

/*!
 * \overload
 *
 * This constructor takes an already created Proxy.
 */
MyObjectNode::MyObjectNode(dcollide::Proxy* proxy, bool useCulling, bool useTextures) {
    if (!proxy) {
        throw dcollide::NullPointerException("parameter proxy");
    }

    mUseTextures = useTextures;
    mUseCulling = useCulling;
    mProxy = proxy;
    mWorld = mProxy->getWorld();

    mUseODE = false;
    mODEGeom = 0;

    mPrintDebugOutput = false;
    mDebugOutputPrefix = "";

    init();
}

void MyObjectNode::init() {
    mProxy->mUser1 = this;

    mUniqueObjectName = generateUniqueObjectName();

    mSceneNode = 0;
    mNarrowPhaseResultsSceneNode = 0;
    mNarrowPhaseResultsObject = 0;
    mBoundingVolumeDirty = true;

    if (mProxy && mProxy->getShape()) {
        // textures
        switch (mProxy->getShape()->getShapeType()) {
            case dcollide::Shape::SHAPE_TYPE_BOX:
            {
                // we have a special texture for simple boxes

                mTextureInformation.setTextured(true);

                mTextureInformation.setTextureFileName("box.tga");
                std::vector<dcollide::Vector3> texels;
                texels.reserve(6 * 2 * 3); // 6 sides, each 2 triangles, each 3 vertices
                // front side (vertices 6,5,4,5,6,7)
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));

                // right side (vertices 3,1,5,3,5,7)
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));

                // back side (vertices 2,0,1,2,1,3)
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));

                // left side (vertices 0,2,4,2,6,4)
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));

                // top side (vertices 0,4,1,1,4,5)
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));

                // bottom side (vertices 2,3,6,3,7,6)
                texels.push_back(dcollide::Vector3(0.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 0.0, 0.0));
                texels.push_back(dcollide::Vector3(1.0, 1.0, 0.0));
                texels.push_back(dcollide::Vector3(0.0, 1.0, 0.0));

                mTextureInformation.setTexels(texels, false);
                break;
            }
            default:
                break;
        }
    }

}

MyObjectNode::~MyObjectNode() {
    if (mNarrowPhaseResultsSceneNode) {
        Ogre::SceneManager* manager = mNarrowPhaseResultsSceneNode->getCreator();
        if (mNarrowPhaseResultsObject) {
            manager->destroyManualObject(mNarrowPhaseResultsObject);
        }
        manager->destroySceneNode(mNarrowPhaseResultsSceneNode->getName());
    }

    for (std::list<MyObjectNode*>::const_iterator it = mChildren.begin(); it != mChildren.end(); ++it) {
        delete *it;
    }

    if (mSceneNode) {
        // AB: we currently assume that every ManualObject owns all
        // materials that it uses!
        // therefore we need to delete them, too
        destroyAllOgreMaterials(mSceneNode);

        destroyOgreSceneHierarchy(mSceneNode);
        mSceneNode = 0;
    }

    for (std::list<Ogre::SceneNode*>::const_iterator it = mBVSceneNodes.begin(); it != mBVSceneNodes.end(); ++it) {
        // AB: materials are shared among BV scene nodes of different
        // MyObjectNode objects. therefore we dont need to delete them here.
        destroyOgreSceneHierarchy(*it);
    }
    mBVSceneNodes.clear();

    if (!mProxy->isInHierarchy()) {
        delete mProxy;
    }

    delete mODEGeom;
}

std::string MyObjectNode::generateUniqueObjectName() {
    static unsigned int objectNumber = 0;
    std::stringstream ss;
    ss << "MyObjectNode_";
    ss << objectNumber;
    objectNumber++;
    return ss.str();
}

dcollide::Vector3 MyObjectNode::calculateBodyPosition(){
    //code duplication, used in createphysicsbody also
    dcollide::Vector3 transformedGravityOffset;
    //transformedGravityOffset = nodeRotation * mODEGeom->getGravityOffset();
    getRotation().transform(&transformedGravityOffset, mODEGeom->getGravityOffset());
    return getWorldPosition() + transformedGravityOffset;
}

void MyObjectNode::resetTransformation() {
    if (!getProxy()) {
        return;
    }

    dcollide::Matrix identity;
    mProxy->setTransformation(identity);

    mBoundingVolumeDirty = true;
}

/*!
 * \brief translates the Proxy,the OGRE node, and (if it exists) the physics body
 *
 */
void MyObjectNode::translate(float x, float y, float z, bool respectOrientation) {
    translate(dcollide::Vector3(x, y, z), respectOrientation);
}

/*!
 * \brief translates the Proxy,the OGRE node, and (if it exists) the physics body
 *
 */
void MyObjectNode::translate(const dcollide::Vector3& translation, bool respectOrientation) {
    if (mPrintDebugOutput) {
        std::cout << mDebugOutputPrefix << "MyObjectNode::translate(translation: " << translation << ", respectOrientation: " << respectOrientation << ")" << std::endl;
    }
    if (!getProxy()) {
        return;
    }

    if (mPrintDebugOutput) {
        std::cout << mDebugOutputPrefix << "really translating!" << std::endl;
    }
    mProxy->translate(translation, respectOrientation);
    
    mBoundingVolumeDirty = true;
    
    if (getPhysicsBody()) {
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "translating physics body!" << std::endl;
        }

        //calculate correct body position (with offset!)
        dcollide::Vector3 bodyPosition = calculateBodyPosition();
        
        mODEGeom->setMoveNodeOnBodyMoved(false);
        dBodySetPosition(getPhysicsBody(),  bodyPosition.getX(),
                                            bodyPosition.getY(),
                                            bodyPosition.getZ());
        mODEGeom->setMoveNodeOnBodyMoved(true);
    }
}

/*!
 * \brief sets the absolute position of the Proxy,the node, and the physics body
 * 
 * \param movePhysicsBody controls wether the physics body (if it exists) should
 *        be influenced by this command or not (default: true)
*/
void MyObjectNode::setPosition(float x, float y, float z, bool movePhysicsBody){
    if (!getProxy()) {
        return;
    }

    mProxy->setPosition(x, y, z);

    mBoundingVolumeDirty = true;

    if (getPhysicsBody() && movePhysicsBody) {
        dcollide::Vector3 bodyPosition = calculateBodyPosition();

        mODEGeom->setMoveNodeOnBodyMoved(false);
        dBodySetPosition(getPhysicsBody(),  bodyPosition.getX(),
                                            bodyPosition.getY(),
                                            bodyPosition.getZ());
        mODEGeom->setMoveNodeOnBodyMoved(true);
    }
}


void MyObjectNode::rotate(float angle, float x, float y, float z, bool respectOrientation) {
    dcollide::Matrix m;
    m.rotate(angle, x, y, z);
    rotate(m, respectOrientation);
}

void MyObjectNode::setRotation(float angle, float x, float y, float z, bool movePhysicsBody) {
    dcollide::Matrix m;
    m.rotate(angle, x,y,z);
    setRotation(m, movePhysicsBody);
}

/*!
 * \brief rotates the Proxy,the OGRE node, and (if it exists) the physics body
 *
 * \param movePhysicsBody controls wether the physics body (if it exists) should
 *        be influenced by this command or not (default: true)
 */
void MyObjectNode::setRotation(const dcollide::Matrix& rotation, bool movePhysicsBody) {
    if (!getProxy()) {
        return;
    }

    mProxy->setRotation(rotation);

    mBoundingVolumeDirty = true;

    if (getPhysicsBody() && movePhysicsBody) {
        //rotating the node translates the body
        //dBodySetPosition and dBodySetRotation both influence the node position
        //prevent that the node is moved while doing this
        mODEGeom->setMoveNodeOnBodyMoved(false);

        dcollide::Vector3 bodyPosition = calculateBodyPosition();

        //convert rotation matrix to ODE rotation matrix
        dMatrix3 oderotation; //dReal[12]
        convertDcollideMatrix2Ode(getRotation(), oderotation);

        dBodySetRotation(getPhysicsBody(), oderotation);
        dBodySetPosition(getPhysicsBody(),  bodyPosition.getX(),
                                            bodyPosition.getY(),
                                            bodyPosition.getZ());
        mODEGeom->setMoveNodeOnBodyMoved(true);
    }

}

/*!
 * \brief rotates the Proxy,the OGRE node, and (if it exists) the physics body
 *
 *
 */
void MyObjectNode::rotate(const dcollide::Matrix& rotation, bool respectOrientation) {
    if (!getProxy()) {
        return;
    }
    
    mProxy->rotate(rotation, respectOrientation);
    
    mBoundingVolumeDirty = true;
    
    if (getPhysicsBody()) {
        dcollide::Vector3 bodyPosition = calculateBodyPosition();
        dMatrix3 odeRotation;
        convertDcollideMatrix2Ode(rotation, odeRotation);
        
        mODEGeom->setMoveNodeOnBodyMoved(false);
        dBodySetRotation(getPhysicsBody(), odeRotation);
        dBodySetPosition(getPhysicsBody(),  bodyPosition.getX(),
                                            bodyPosition.getY(),
                                            bodyPosition.getZ());
        mODEGeom->setMoveNodeOnBodyMoved(true);
    }

}

const std::list<MyObjectNode*>& MyObjectNode::getChildren() const {
    return mChildren;
}

// WARNING: must be called BEFORE SceneBase::addTopLevelObject() is called!
void MyObjectNode::addChild(MyObjectNode* child) {
    if (!child) {
        return;
    }
    if (!child->getProxy()) {
        std::cerr << dc_funcinfo << "ERROR: NULL proxy" << std::endl;
        return;
    }
    mChildren.push_back(child);
    if (child->getProxy()->getParent() == 0) {
        mProxy->addChild(child->getProxy());
    }

    mBoundingVolumeDirty = true;
}

/*!
 * \return The internal dcollide proxy that represents this object.
 */
dcollide::Proxy* MyObjectNode::getProxy() const {
    return mProxy;
}

/*!
 * obsolete.
 */
void MyObjectNode::setRepeatedMovement(const dcollide::Vector3& start, const dcollide::Vector3& end, float speed) {
    mMovementStart = start;
    mMovementEnd = end;
    mMovementSpeed = speed;
}

/*!
 * obsolete.
 */
void MyObjectNode::setRepeatedMovementSpeed(float speed) {
    mMovementSpeed = speed;
}

void MyObjectNode::recalculateBoundingVolumeOgreObjects() {
    if (!getProxy()) {
        std::cerr << "ERROR: NULL proxy" << std::endl;
        return;
    }

    std::list<const dcollide::BvhNode*> bvhNodes;
    if (getProxy()->getBvHierarchyNode()) {
        bvhNodes.push_back(getProxy()->getBvHierarchyNode());
    }

    if (getProxy()->getProxyType() & dcollide::PROXYTYPE_DEFORMABLE) {
        for (unsigned int index = 0; index < mWorld->getDetectorDeformManager()->getAlgorithms().size(); index++) {
            dcollide::DetectorDeformProxyData* data = getProxy()->getDetectorDeformProxyData(index);
            if (data && data->getDeformableBvhNode()) {
                bvhNodes.push_back(data->getDeformableBvhNode());
            }
        }
    }

    for (std::list<const dcollide::BvhNode*>::const_iterator it = bvhNodes.begin(); it != bvhNodes.end(); ++it) {
        recalculateBoundingVolumeOgreObjects(*it);
    }
    mBoundingVolumeDirty = false;
}

void MyObjectNode::recalculateBoundingVolumeOgreObjects(const dcollide::BvhNode* node) {
    if (!node) {
        std::cerr << "ERROR: NULL BvhNode" << std::endl;
        return;
    }
    if (!node->mUser) {
        dcollide::error() << dc_funcinfo << "NULL mUser pointer in BvhNode";
        return;
    }

    Ogre::ManualObject* ogreObject = (Ogre::ManualObject*)node->mUser;
    if (!node->getBoundingVolume()) {
        dcollide::error() << dc_funcinfo << "NULL BoundingVolume";
        ogreObject->clear();
        return;
    }

    int volumeType = Ogre::any_cast<int>(ogreObject->getUserAny());
    //std::cout << "volumeType" << volumeType << std::endl;
    //std::cout << "volumeType_bvhnode" << node->getBoundingVolume()->getVolumeType() << std::endl;
    if (node->getBoundingVolume()->getVolumeType() != volumeType) {
        dcollide::error() << dc_funcinfo << "volumetype of node has changed since initial creation!";
        ogreObject->clear();
        OgreSceneObjectFactory::initOgreObjectForBoundingVolume(ogreObject, node->getBoundingVolume());
    } else if (ogreObject->getNumSections() < 1) {
        ogreObject->clear();
        OgreSceneObjectFactory::initOgreObjectForBoundingVolume(ogreObject, node->getBoundingVolume());
    } else {
        ogreObject->beginUpdate(0);
            OgreSceneObjectFactory::specifyBoundingVolumeLines(ogreObject, node->getBoundingVolume());
        ogreObject->end();
    }


    const std::list<dcollide::BvhNode*>& children = node->getChildren();
    for (std::list<dcollide::BvhNode*>::const_iterator it = children.begin(); it != children.end(); ++it) {
        recalculateBoundingVolumeOgreObjects(*it);
    }
}

/*!
 * Set whether this MyObjectNode object is part of the broadphase collisions.
 *
 * If \p c is TRUE, this class may decide to display this object differently, if
 * requested by the application (e.g. the boundingvolumes may be displayed in a
 * different color.
 */
void MyObjectNode::setBroadPhaseCollision(bool c) {
    // AB: for broadphase we color the bounding volume around the object
    //     I would prefer to color the actual object, but that would be way
    //     more difficult: we'd have to (recurisvely) color _all_
    //     ManualObjects of that object, which could be a lot

    if (!getProxy()) {
        return;
    }
    const dcollide::BvhNode* bvhNode = getProxy()->getBvHierarchyNode();
    if (!bvhNode) {
        return;
    }
    Ogre::ManualObject* manual = static_cast<Ogre::ManualObject*>(bvhNode->mUser);
    if (c) {
        if (getProxy()->getParent()) {
            std::cerr << dc_funcinfo << "can have broadphase collisions on toplevel proxies only" << std::endl;
            return;
        }
        for (unsigned int i = 0; i < manual->getNumSections(); i++) {
            manual->setMaterialName(i, OgreMaterials::getBoundingVolumeMaterial(true, false));
        }
    } else {
        for (unsigned int i = 0; i < manual->getNumSections(); i++) {
            manual->setMaterialName(i, OgreMaterials::getBoundingVolumeMaterial(false, false));
        }
    }
}

/*!
 * Create ogre objects representing all narrowphase collisions added using \ref
 * addNarrowPhaseCollision.
 */
void MyObjectNode::createNarrowPhaseCollisions() {
    if (!mNarrowPhaseResultsSceneNode) {
        std::cerr << dc_funcinfo << "NULL mNarrowPhaseResultsSceneNode" << std::endl;
        return;
    }
    if (mNarrowPhaseResults.empty()) {
        return;
    }
    if (!mNarrowPhaseResultsObject) {
        mNarrowPhaseResultsObject = mNarrowPhaseResultsSceneNode->getCreator()->createManualObject(std::string("NarrowPhaseCollisionPoints_") + mUniqueObjectName);
        mNarrowPhaseResultsSceneNode->attachObject(mNarrowPhaseResultsObject);
    }

    bool createNew = false;
    if (mNarrowPhaseResultsObject->getNumSections() < 2) {
        createNew = true;
        mNarrowPhaseResultsObject->clear();
    }

    { // collision points
        if (createNew) {
            mNarrowPhaseResultsObject->begin(OgreMaterials::getNarrowPhaseCollisionPointsMaterial(), Ogre::RenderOperation::OT_POINT_LIST);
        } else {
            mNarrowPhaseResultsObject->beginUpdate(0);
        }

        for (std::list<dcollide::CollisionInfo>::iterator it= mNarrowPhaseResults.begin(); it != mNarrowPhaseResults.end(); ++it) {
            dcollide::Vector3 p = (*it).collisionPoint;
            mNarrowPhaseResultsObject->position(p.getX(), p.getY(), p.getZ());
        }

        mNarrowPhaseResultsObject->end();
    }
    { // collision normals
        if (createNew) {
            mNarrowPhaseResultsObject->begin(OgreMaterials::getNarrowPhaseNormalsMaterial(), Ogre::RenderOperation::OT_LINE_LIST);
        } else {
            mNarrowPhaseResultsObject->beginUpdate(1);
        }

        for (std::list<dcollide::CollisionInfo>::iterator it= mNarrowPhaseResults.begin(); it != mNarrowPhaseResults.end(); ++it) {
            //TODO: define the penetration vector consistently
            // to fit all shape-shape collisions and adapt the visualization
            dcollide::Vector3 p = (*it).collisionPoint;
                    //- (*it).normal * ((*it).penetrationDepth / 2);

            // AB: note that we assume the normal is normalized!
            // AB: we scale the length of the normal by the penetrationDepth. I
            //     have no idea if that has any meaning whatsoever, but I don't
            //     see any other way to visualize the penetration depth.
            dcollide::Vector3 p2 = (*it).collisionPoint + (*it).normal *
                ((*it).penetrationDepth) * PENETRATION_DEPTH_SCALING_FACTOR;

            mNarrowPhaseResultsObject->position(p.getX(), p.getY(), p.getZ());
            mNarrowPhaseResultsObject->position(p2.getX(), p2.getY(), p2.getZ());
        }

        mNarrowPhaseResultsObject->end();
    }
}

void MyObjectNode::clearNarrowPhaseCollisions() {
    mNarrowPhaseResults.clear();
    setPenetratingObject(false);
    setPenetratedObject(false);
    if (mNarrowPhaseResultsObject) {
        mNarrowPhaseResultsObject->clear();
    }
}

/*!
 * Store a narrowphase collision in this object for later display. Usually only
 * one of the two objects in \p c should display a collision.
 */
void MyObjectNode::addNarrowPhaseCollision(const dcollide::CollisionInfo& c) {
    mNarrowPhaseResults.push_back(c);
}

/*!
 * Set the OGRE scenenode that this object is in. The scenenode in particular
 * represents the translations and rotations of this proxy in OGRE.
 *
 * The given \p node should contain the entire hierarchy of this MyObjectNode,
 * i.e. the \ref getChildren of this node should be represented in \p node by
 * children, too. Also every \ref Shape object should be represented by one \ref
 * Ogre::ManualObject in the appropriate node in the hierarchy.
 *
 * This class takes ownership of \p node and the entire hierarchy (i.e.
 * including child-nodes and all \ref Ogre::ManualObject objects) and will
 * delete it on destruction.
 */
void MyObjectNode::setOgreSceneNode(Ogre::SceneNode* node) {
    if (mSceneNode) {
        std::cerr << dc_funcinfo << "object already has an ogre scene node. cannot set a new one." << std::endl;
        return;
    }
    if (!node) {
        std::cerr << dc_funcinfo << "NULL node" << std::endl;
        return;
    }
    mSceneNode = node;
}

/*!
 * Set the list of OGRE scene nodes for the \ref BvhNode objects of this object.
 *
 * The \p nodes list contains exactly one \ref Ogre::SceneNode for each
 * (toplevel) \ref BvhNode in \ref getProxy. A toplevel \ref BvhNode is either
 * \ref Proxy::getBvHierarchyNode or a \ref
 * DetectorDeformProxyData::getDeformableBvhNode (one such node per deformable
 * algorithm may exist). If \ref getProxy is not deformable, there must be
 * exactly one entry in \p nodes only.
 *
 * The first entry in \p nodes alway (!) must be the node corresponding to \ref
 * Proxy::getBvHierarchyNode. It is expected that the order of the remaining
 * nodes matches the order of \ref DetectorDeformManager::getAlgorithms() and
 * that at most one node per algorithm appears. Also it is expected that the
 * number of nodes does never change for this object.
 *
 * The list of nodes is used to destroy the nodes on destruction of this
 * \ref MyObjectNode.
 */
void MyObjectNode::setOgreBVSceneNodes(const std::list<Ogre::SceneNode*>& nodes) {
    if (!mBVSceneNodes.empty()) {
        std::cerr << dc_funcinfo << "object already has ogre BV scene nodes. cannot set new ones." << std::endl;
        return;
    }
    mBVSceneNodes = nodes;

    if (mBVSceneNodes.empty()) {
        std::cerr << dc_funcinfo << "empty list of nodes has been set! expected at least one entry for Proxy::getBvHierarchyNode()!" << std::endl;
    }
}

/*!
 * Set a scene node that should contain the narrowphase results of this object,
 * i.e. everything created by \ref createNarrowPhaseCollisions (which uses \ref
 * addNarrowPhaseCollision as input)
 *
 * This class takes ownership of \p node and will delete it on destruction.
 */
void MyObjectNode::setOgreNarrowPhaseResultsSceneNode(Ogre::SceneNode* node) {
    if (mNarrowPhaseResultsSceneNode) {
        std::cerr << dc_funcinfo << "object already has an narrowphase scene node. cannot set a new one." << std::endl;
        return;
    }
    if (!node) {
        std::cerr << dc_funcinfo << "NULL node" << std::endl;
        return;
    }
    mNarrowPhaseResultsSceneNode = node;
}

/*!
 * \brief get local coordinates of the node
 */
dcollide::Vector3 MyObjectNode::getTranslation() const {
    if (!getProxy()) {
        return dcollide::Vector3();
    }
    return getProxy()->getPosition();
}

/*!
 * \brief get world coordinates of the node
 */
dcollide::Vector3 MyObjectNode::getWorldPosition() const {
    if (!getProxy()) {
        return dcollide::Vector3();
    }
    return getProxy()->getWorldTransformation().getPosition();
}

// FIXME: Misleading name. How can we distinguish between local and global rotation?
dcollide::Matrix MyObjectNode::getRotation() const {
    if (!getProxy()) {
        return dcollide::Matrix();
    }
    return getProxy()->getRotation();
}

/*!
 * \brief puts a physical, moveable representation into the physics simulation
 *
 * Will create a dBodyID and put it into the given world. The object will have
 * the given total mass. The weight distribution around the reference point
 * depends on the shape of this node's \ref dcollide::Proxy
 *
 * NOTE:    this function should be used for basic geometry only
 *          (i.e Boxes, Spheres, Cylinders etc.).
 *          For triangle Meshes, create your own dBodyID and
 *          use \ref setPhysicsBody() instead.
 *
 * \param odeWorld the id of the (already created) ode world.
 * \param space Deprecated. Use 0.
 * \param totalMass the mass of the object.
 */
void MyObjectNode::createPhysicsBody(dWorldID odeWorld, dReal totalMass) {
    //dcollide::debug() << dc_funcinfo;
    if (!odeWorld) {
        throw dcollide::NullPointerException("dWorldID odeWorld");
    }

    if (!mProxy->getShape()) {
        std::cerr << "WARNING: cannot create physics body for a shapeless Proxy."<<std::endl;
        return;
    }

    //do not create a body if it already exists
    if (mODEGeom) {
        std::cerr << "WARNING: physics body was already created for this Proxy."<<std::endl;
        return;
    }

    if (mProxy->isInHierarchy()) {
        dcollide::warning() << dc_funcinfo << "must be called before addTopLevelObject";
        return;
    }

    mUseODE=true;
    //create physical body for the shape

    mODEGeom = new MyODETestAppGeom(this);
    
    dBodyID body = dBodyCreate(odeWorld);
    dMass mass;

    switch (mProxy->getShape()->getShapeType()) {
        case dcollide::Shape::SHAPE_TYPE_BOX:
            {
                dcollide::Box* box = (dcollide::Box*)mProxy->getShape();
                dMassSetBoxTotal(&mass, totalMass,  box->getDimension().getX(),
                                                    box->getDimension().getY(),
                                                    box->getDimension().getZ());
                //the mass is now distributed around the reference point, which
                //is the lower left corner of the box. To actually represent the
                //box, we need to translate the mass to the center of the box
                mODEGeom->setGravityOffset( dcollide::Vector3(
                                                box->getDimension().getX() / 2,
                                                box->getDimension().getY() / 2,
                                                box->getDimension().getZ() / 2)
                                           );

                break;
            }
            case dcollide::Shape::SHAPE_TYPE_SPHERE:
            {
                dcollide::Sphere* sphere = (dcollide::Sphere*)mProxy->getShape();
                dMassSetSphereTotal(&mass, totalMass,  sphere->getRadius());
                break;
            }
            case dcollide::Shape::SHAPE_TYPE_CYLINDER:
            {
                dcollide::Cylinder* cylinder = (dcollide::Cylinder*)mProxy->getShape();
                dMassSetCylinderTotal(&mass, totalMass,  3, cylinder->getRadius(), cylinder->getHeight());
                //Translate mass from reference point (bottom sphere center) to
                //center of gravity
                mODEGeom->setGravityOffset( dcollide::Vector3(0, 0,cylinder->getHeight() / 2));
                break;
            }
            //TODO: other shapes, see
            //http://opende.sourceforge.net/wiki/index.php/Manual_%28Support_Functions%29
            default:
            {
                std::cout<<"no special case found, creating aabb-box dBody" << std::endl;
                //By default, use box mass adjustment, AABB-oriented
                dcollide::Vector3 aabbMin = mProxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbMin();

                dcollide::Vector3 aabbDim = mProxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbExtents();
                
                dMassSetBoxTotal(&mass, totalMass, aabbDim.getX(), aabbDim.getZ(), aabbDim.getZ());
                mODEGeom->setGravityOffset( dcollide::Vector3(
                                                aabbMin.getX() + aabbDim.getX() / 2,
                                                aabbMin.getY() + aabbDim.getY() / 2,
                                                aabbMin.getZ() + aabbDim.getZ() / 2)
                                           );
            }
                break;
    }//end switchcase


    dBodySetMass(body, &mass);
    dGeomSetBody((dGeomID)mODEGeom, body);
    mODEGeom->setMoveNodeOnBodyMoved(false);
    //move the body to the position of the node (+offset)
    //go to node position + (node rotation matrix) * offset
    dcollide::Vector3 bodyPosition = calculateBodyPosition();
    dBodySetPosition(body, bodyPosition[0], bodyPosition[1], bodyPosition[2]);
    
    dMatrix3 odeRotation;
    convertDcollideMatrix2Ode(getRotation(), odeRotation);
    dBodySetRotation(body, odeRotation);
    
    //tell the mOdeTestappgeom that it is now at its valid position
    //all future movements of the physics body will now move the node too
    mODEGeom->setMoveNodeOnBodyMoved(true);

    // should be pretty much a noop (we set the values that are already set)
    dGeomMoved((dGeomID)mODEGeom);
}

/*!
 *\brief use the given body for physics simulation
 *
 * OWNERSHIP notice: The Node takes ownership of the dBodyID
 */
void MyObjectNode::setPhysicsBody(dBodyID body) {
    if (!mProxy->getShape()) {
        std::cerr << "WARNING: cannot create physics body for a shapeless Proxy."<<std::endl;
        return;
    }

    //do not create a body if it already exists
    if (mODEGeom) {
        std::cerr << "WARNING: physics body was already created for this Proxy."<<std::endl;
        return;
    }

    //encapsulate the body in an MyODEGeom object and use it
    mODEGeom = new MyODETestAppGeom(this);
    dGeomSetBody((dGeomID)mODEGeom, body);
}

dBodyID MyObjectNode::getPhysicsBody() const {
    if (mODEGeom) {
        return mODEGeom->getBody();
    }

    return 0;
}

void MyObjectNode::destroyOgreSceneHierarchy(Ogre::SceneNode* top) {
    if (!top) {
        return;
    }

    Ogre::SceneManager* manager = top->getCreator();

    std::list<Ogre::SceneNode*> nodes;
    nodes.push_back(top);
    while (!nodes.empty()) {
        Ogre::SceneNode* node = nodes.front();
        nodes.pop_front();
        for (int i = 0; i < node->numChildren(); i++) {
            nodes.push_back(static_cast<Ogre::SceneNode*>(node->getChild(i)));
        }

        if (node->numAttachedObjects() == 0) {
            continue;
        }
        if (node->numAttachedObjects() > 1) {
            dcollide::error() << "More than one object attached to an Ogre::SceneNode of MyObjectNode";
        }

        Ogre::MovableObject* object = node->getAttachedObject(0);
        if (object->getMovableType() == Ogre::ManualObjectFactory::FACTORY_TYPE_NAME) {
            Ogre::ManualObject* manual = static_cast<Ogre::ManualObject*>(object);
            manager->destroyManualObject(manual);
        }
    }
    top->removeAndDestroyAllChildren();
    top->getCreator()->destroySceneNode(top->getName());
}

void MyObjectNode::destroyAllOgreMaterials(Ogre::SceneNode* top) {
    if (!top) {
        return;
    }

    std::list<Ogre::SceneNode*> nodes;
    nodes.push_back(top);
    while (!nodes.empty()) {
        Ogre::SceneNode* node = nodes.front();
        nodes.pop_front();
        for (int i = 0; i < node->numChildren(); i++) {
            nodes.push_back(static_cast<Ogre::SceneNode*>(node->getChild(i)));
        }

        if (node->numAttachedObjects() == 0) {
            continue;
        }
        if (node->numAttachedObjects() > 1) {
            dcollide::error() << "More than one object attached to an Ogre::SceneNode of MyObjectNode";
        }

        Ogre::MovableObject* object = node->getAttachedObject(0);
        if (object->getMovableType() == Ogre::ManualObjectFactory::FACTORY_TYPE_NAME) {
            Ogre::ManualObject* manual = static_cast<Ogre::ManualObject*>(object);
            for (unsigned int i = 0; i < manual->getNumSections(); i++) {
                OgreMaterials::destroyShapeMaterial(manual->getSection(i)->getMaterialName());
            }
        }
    }
}

void MyObjectNode::setPenetratingObject(bool isPenetrating) {
    mNarrowPhaseIsPenetrating = isPenetrating;
}

void MyObjectNode::setPenetratedObject(bool isPenetrated) {
    mNarrowPhaseIsPenetrated = isPenetrated;
}

void MyObjectNode::updateMaterialForNarrowPhaseResults(bool displayNarrowPhaseResults) {
    if (!mSceneNode) {
//        dcollide::debug() << dc_funcinfo << "have no mSceneNode object";
        return;
    }
    if (mSceneNode->numAttachedObjects() == 0) {
        return;
    }
    if (mSceneNode->numAttachedObjects() > 1) {
        dcollide::error() << "More than one object attached to an Ogre::SceneNode of MyObjectNode";
    }

    Ogre::MovableObject* object = mSceneNode->getAttachedObject(0);
    if (object->getMovableType() == Ogre::ManualObjectFactory::FACTORY_TYPE_NAME) {
        Ogre::ManualObject* manual = static_cast<Ogre::ManualObject*>(object);
        for (unsigned int i = 0; i < manual->getNumSections(); i++) {
            Ogre::Pass* pass = manual->getSection(i)->getMaterial()->getTechnique(0)->getPass(0);
            if (displayNarrowPhaseResults) {
                pass->setDiffuse(OgreMaterials::getShapeDiffuseColor(getTextureInformation(), mNarrowPhaseIsPenetrating, mNarrowPhaseIsPenetrated));
            } else {
                pass->setDiffuse(OgreMaterials::getShapeDiffuseColor(getTextureInformation()));
            }
        }
    }
}

/*
 * vim: et sw=4 ts=4
 */
