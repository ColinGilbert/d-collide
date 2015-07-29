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

#include "scenes/scenebase.h"

#include "dcollide-config_testapp.h"
#include "odewrapper.h"
#include "myobjectnode.h"
#include "mydeformableobjectnode.h"
#include "ogrematerials.h"
#include "ogresceneobjectfactory.h"
#include "modelloader/modelloader3ds.h"
#include "modelloader/modelloaderply.h"

#include <d-collide/dcollide-global.h>
#include <d-collide/broadphase/broadphasehierarchicalgrid.h>
#include <d-collide/exceptions/exception.h>
#include <d-collide/proxy.h>
#include <d-collide/bvhnode.h>
#include <d-collide/boundingvolumes/boundingvolume.h>
#include <d-collide/world.h>
#include <d-collide/worldcollisions.h>
#include <d-collide/collisionpair.h>
#include <d-collide/broadphase/broadphasecollisions.h>
#include <d-collide/narrowphase/narrowphase.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/detectordeform/detectordeformmanager.h>
#include <d-collide/detectordeform/detectordeformproxydata.h>
#include <d-collide/debugstream.h>

#include <OgreRoot.h>
#include <OgreLight.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreMeshManager.h>

/*!
 * Construct a new d-collide based scene object.
 *
 * The scene itself is not yet created, only the necessary data
 * structures. Call \ref initializeScene to actually create a scene.
 *
 * \param root A pointer to the ogre root object. Ownership is NOT
 * taken, the pointer will not be deleted by this class.
 */
SceneBase::SceneBase(Ogre::Root* root) {
    mOgreRoot = root;

    mInitializeSceneCalled = false;
    mSceneManager = 0;
    mOgreLight = 0;
    mOgreCamera = 0;
    mBoundingVolumes = 0;
    mHierarchicalGridSceneNode = 0;
    mNarrowPhaseResults = 0;
    mOgreSceneObjectFactory = 0;
    mCollisionWorld = 0;

    mBoundingVolumesVisible = true;
    mNarrowPhaseResultsVisible = false;

    mWorldCollisions = 0;


    // AB: do NOT create World here!
    //     -> we need initialWorldDimension() for that which is virtual (and
    //        thus should/can not be called by the c'tor)
}

SceneBase::~SceneBase() {
    delete mWorldCollisions;
    mWorldCollisions = 0;
    removeAllObjects();

    delete mOgreSceneObjectFactory;
    delete mCollisionWorld;
}

void SceneBase::removeAllObjects() {
    std::list<dcollide::Proxy*> topLevelProxies = mCollisionWorld->getTopLevelProxies();
    for (std::list<dcollide::Proxy*>::iterator it = topLevelProxies.begin(); it != topLevelProxies.end(); ++it) {
        MyObjectNode* n = static_cast<MyObjectNode*>((*it)->mUser1);
        removeObject(n);
    }
}

/*
 * \brief removes the object from the world/scene and deletes it
 * 
 * NOTE: the /p object pointer is no longer valid after calling this function
 */
void SceneBase::removeObject(MyObjectNode* object) {
    dcollide::Proxy* proxy = object->getProxy();
    if (!proxy) {
        throw dcollide::NullPointerException("object->getProxy()");
    }
    if (!proxy->isInHierarchy()) {
        delete object;
        return;
    }
    if (proxy->getParent()) {
        std::cerr << dc_funcinfo << "d-collide does not yet support removing a child from its parent!" << std::endl;

        // TODO:
        // proxy->getParent()->removeChild(proxy);
        // delete object;
        return;
    }

    // toplevel object.
    mCollisionWorld->removeProxy(proxy);
    delete object;

    // TODO: delete ogre nodes?
}


/*!
 * \brief Adds parameters for a initial World configuration based on the type of
 *        the scene
 * 
 * This function should be overriden by derived SceneBase classes which are ment
 * to build own scene subclasses (as the physics scenes do). It can be used to
 * declare some scene class specific parameters which could be overriden by the
 * scene specific parameters. 
 */
void SceneBase::addInitialWorldParameters(dcollide::WorldParameters& parameters) const {
    //parameters.setRigidBoundingVolumeType(dcollide::BV_TYPE_OBB);
    //parameters.setRigidBoundingVolumeType(dcollide::BV_TYPE_SPHERE);
}

/*!
 * \brief Adds scene specific parameters - dafault implemtation is a NOP
 * 
 * This method should be overriden a specific scene classes which want to set a
 * specific algorithm, a specific BV type or something like this.
 */
void SceneBase::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    /* this method should be overriden by a scene specific implementation */
}

std::string SceneBase::getModelPath(const std::string& filename) const {
    Ogre::FileInfoListPtr fileInfoList = Ogre::ResourceGroupManager::getSingleton().findResourceFileInfo("DCollideModels", filename);
    if (fileInfoList.get()->empty()) {
        dcollide::error() << "Cannot find file " << filename;
        return 0;
    }

    Ogre::Archive* archive = fileInfoList.get()->front().archive;
    if (!archive) {
        dcollide::error() << "NULL archive";
        return 0;
    }
    if (archive->getName().empty()) {
        dcollide::error() << "archive has empty name";
        return 0;
    }
#ifndef WIN32
    if (archive->getName()[0] != '/') {
        dcollide::error() << "Archive::getName() does not start with /";
        return 0;
    }
#endif
    std::string file = std::string(archive->getName()) + std::string("/") + filename;
    return file;
}

MyObjectNode* SceneBase::create3dsModel(const std::string& filename, int proxyType) const {
    std::string file = getModelPath(filename);
    if (file.empty()) {
        dcollide::error() << "Cannot load " << filename;
    }
    ModelLoader3ds loader;
    MyObjectNode* object = loader.loadFromFile(getCollisionWorld(), file.c_str(), proxyType);
    if (!object) {
        std::cout << "Could not load 3ds model " << file << std::endl;
        return 0;
    }
    return object;
}

MyObjectNode* SceneBase::create3dsModelWithOneMesh(const std::string& filename, int proxyType) const {
    std::string file = getModelPath(filename);
    if (file.empty()) {
        dcollide::error() << "Cannot load " << filename;
    }
    ModelLoader3ds loader;
    MyObjectNode* object = loader.loadFromFileInOneMesh(getCollisionWorld(), file.c_str(), proxyType);
    if (!object) {
        std::cout << "Could not load 3ds model " << file << std::endl;
        return 0;
    }
    return object;
}

MyObjectNode* SceneBase::createPlyModel(const std::string& filename, int proxyType, double scale) const {
    std::string file = getModelPath(filename);
    if (file.empty()) {
        dcollide::error() << "Cannot load " << filename;
    }
    ModelLoaderPly loader;
    MyObjectNode* object = loader.loadFromFile(getCollisionWorld(), file.c_str(), proxyType, scale);
    if (!object) {
        std::cout << "Could not load PLY model " << file << std::endl;
        return 0;
    }
    return object;
}

MyObjectNode* SceneBase::createOgreModel(const std::string& filename,
                                         const std::string& entityName,
                                         int proxyType) const {
    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().load(filename, "DCollideModels");
    Ogre::Entity* entity = mSceneManager->createEntity(entityName, filename);
    
    if (!entity) {
        dcollide::error() << "Can't load Ogre-mesh file " << filename;
        return 0;
    }
    
    dcollide::debug() << "Success loading Ogre-mesh file " << filename;
    
    MyObjectNode* node = new MyObjectNode(getCollisionWorld(),
                                          new dcollide::Box(),
                                          proxyType);
    
    Ogre::SceneNode* sceneNode = mSceneManager->createSceneNode();
    node->setOgreSceneNode(sceneNode);
    sceneNode->attachObject((Ogre::MovableObject*) entity);
    node->resetTransformation();
    return node;
}

bool SceneBase::initScene() {
    if (mInitializeSceneCalled) {
        // already initialized
        return true;
    }
    std::cout << dc_funcinfo << std::endl;

    if (!OgreMaterials::init()) {
        std::cerr << "Could not initialize OgreMaterials" << std::endl;
        // FIXME: exception?
        return false;
    }

    dcollide::WorldParameters param;

    addInitialWorldParameters(param);
    addSceneSpecificWorldParameters(param);
    param.setWorldDimension(initialWorldDimension());

    mCollisionWorld = new dcollide::World(param);

    // AB: We let OGRE pick the best scene manager for us (which will probably
    // be the default one, since we did not load any scene manager plugins).
    // we just tell OGRE what kind of scene we will have.
    // Since we don't know anything about our scene yet, we have a "generic"
    // scene.
    // For possible scene types, see the SceneType enum in OgreSceneManager.h
    mSceneManager = mOgreRoot->createSceneManager(Ogre::ST_GENERIC);
    mOgreSceneObjectFactory = new OgreSceneObjectFactory(mSceneManager);

    mBoundingVolumes = mSceneManager->createSceneNode();
    if (mBoundingVolumesVisible) {
        mSceneManager->getRootSceneNode()->addChild(mBoundingVolumes);
        mBoundingVolumes->setVisible(true);
    }
    mNarrowPhaseResults = mSceneManager->createSceneNode();
    mNarrowPhaseResultsVisible = false;
    mSceneManager->setAmbientLight( Ogre::ColourValue(0.5, 0.5, 0.5) );


    mOgreLight = mSceneManager->createLight( "DirectionalLight" );

    mOgreLight->setType( Ogre::Light::LT_DIRECTIONAL );
    mOgreLight->setDiffuseColour( 0.5, 0.5, 0.5 );
    mOgreLight->setSpecularColour( 1, 1, 1 );
    mOgreLight->setDirection( 0, -1, -5 );


    // AB: the camera is created by the sceneManager in Ogre, so we create it
    // here.
    // note that this is imho not very nice: the scene and the view on the scene
    // should not both be handled here.
    mOgreCamera = mSceneManager->createCamera("Camera1");
    mOgreCamera->setFixedYawAxis(false);

    if (!initializeScene()) {
        return false;
    }
    mInitializeSceneCalled = true;

    getCollisionWorld()->prepareSimulation();

    dcollide::BroadPhase* broad_ = mCollisionWorld->getBroadPhase();
    dcollide::BroadPhaseHierarchicalGrid* broad =
            dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>(broad_);
    if (broad) {
        const dcollide::HierarchicalGrid* grid = broad->getHierarchicalGrid();
        if (!grid) {
            std::cerr << dc_funcinfo << "oops" << std::endl;
            return false;
        }
        mHierarchicalGridSceneNode =
                mOgreSceneObjectFactory->createHierarchicalGrid(grid);
        if (!mHierarchicalGridSceneNode) {
            std::cerr << dc_funcinfo << "NULL mHierarchicalGridSceneNode"
                    << std::endl;
            return false;
        }
    }

    std::list<dcollide::Proxy*> topLevelProxies = mCollisionWorld->getTopLevelProxies();
    for (std::list<dcollide::Proxy*>::iterator it = topLevelProxies.begin(); it != topLevelProxies.end(); ++it) {
        MyObjectNode* object = static_cast<MyObjectNode*>((*it)->mUser1);
        if (!object) {
            continue;
        }
        if (object->hasOgreBVSceneNodes()) {
            throw dcollide::Exception("Object already has an ogre BV scene node, before scene initializing has been completed");
        }
        initToplevelObject(object);
    }
    
    return true;
}

/*!
 * Calls \ref startNextSceneFrame and does additional tasks that are required
 * when a new (scene) frame begins (such as updating the hierarchical grid).
 *
 * This should be called once per (scene) frame, you should never call \ref
 * startNextSceneFrame directly. Note that a scene frame is not necessarily a
 * graphic frame, i.e. you could render the scene more often or less often than
 * you change the position of the objects in it.
 */
void SceneBase::beginNextSceneFrame() {
	//dcollide::debug() << "----- beginNextSceneFrame --------";
	mCollisionWorld->startNextStep();
    // call implementation of derived classes
    startNextSceneFrame();

    // update grid
    if (mHierarchicalGridSceneNode &&
            mHierarchicalGridSceneNode->getParentSceneNode()) {
         // node is visible
        recalculateHierarchicalGrid();
    }
}

/*!
 * Update the Ogre SceneNode and the bounding volumes of all non-fixed objects
 * in the mTopLevelObjects list.
 *
 * \param forced Usually Ogre-scenenodes are only updated if the corresponding
 * proxy has been transformed in any way. If set to true the update is forced
 * (needed for initialization).
 */
void SceneBase::updateOgreRepresentations(bool forced) {

    std::list<dcollide::Proxy*>
    	topLevelProxies
    		= mCollisionWorld->getTopLevelProxies();

    for (std::list<dcollide::Proxy*>::iterator it = topLevelProxies.begin();
         it != topLevelProxies.end(); ++it) {

        if (!((*it)->getProxyType() & dcollide::PROXYTYPE_FIXED) || forced) {
            MyObjectNode* object = static_cast<MyObjectNode*>((*it)->mUser1);

            if (object->getBoundingVolumeDirty() || forced) {
                updateOgreObject(object, forced);
                
                if (mBoundingVolumesVisible) {
                    object->recalculateBoundingVolumeOgreObjects();
                }
            }   
        }
    }
}

void SceneBase::updateOgreObject(MyObjectNode* object, bool forced) {
    if (object->mPrintDebugOutput) {
        std::cout << object->mDebugOutputPrefix << ": updateOgreObject(forced: " << forced << ")" << std::endl;
    }

    if (object->getOgreSceneNode()) {
        int flags = object->getProxy()->getMoveFlags();

        if (object->mPrintDebugOutput) {
            std::cout << object->mDebugOutputPrefix << "has a OgreSceneNode!" << std::endl;
        } 

        if (forced || (flags != dcollide::Proxy::MOVEFLAG_UNMOVED)) {
            
            // apply deformations to Ogre scenenode
            if (   (flags & dcollide::Proxy::MOVEFLAG_DEFORMED)
                || (forced && object->isDeformable())) {

                if (object->getOgreSceneNode()->numAttachedObjects() > 0) {

                    Ogre::MovableObject* ogreMovable
                        = object->getOgreSceneNode()->getAttachedObject(0);

                    if (ogreMovable->getMovableType() == Ogre::ManualObjectFactory::FACTORY_TYPE_NAME) {
                        Ogre::ManualObject* ogreObject
                            = static_cast<Ogre::ManualObject*>(ogreMovable);

                        if (!ogreObject) {
                            throw dcollide::NullPointerException("ogreObject");
                        }

#ifdef __GNUC__
#warning FIXME: transform vertices instead of rebuilding the mesh
#endif
                        ogreObject->beginUpdate(0);
                        OgreSceneObjectFactory::specifyTrianglesForShape(ogreObject,
                                                                         object->getProxy()->getShape(),
                                                                         object->getTextureInformation(),
                                                                         object->getUseTextures());
                        ogreObject->end();

                        if (object->mPrintDebugOutput) {
                            std::cout << object->mDebugOutputPrefix << "updated through specifyTriangles!" << std::endl;
                        }
                    }
                }
            }

            // apply rotation to Ogre scenenode
            if ((flags & dcollide::Proxy::MOVEFLAG_ROTATED) || forced) {

                Ogre::Matrix3 ogreRotation;
                dcollide::Matrix rotation = object->getRotation();

                for (int i = 0; i < 3; i++) {
                    Ogre::Vector3 column;
                    column[0] = rotation.getElement(0, i);
                    column[1] = rotation.getElement(1, i);
                    column[2] = rotation.getElement(2, i);
                    ogreRotation.SetColumn(i, column);
                }
                object
                    ->getOgreSceneNode()
                    ->setOrientation(Ogre::Quaternion(ogreRotation));

                if (object->mPrintDebugOutput) {
                    std::cout << object->mDebugOutputPrefix << "updated through setOrientation!" << std::endl;
                }
            }

            // apply translation to Ogre scenenode
            if ((flags & dcollide::Proxy::MOVEFLAG_TRANSLATED) || forced) {

                dcollide::Vector3 position = object->getTranslation();
                object
                    ->getOgreSceneNode()
                    ->setPosition(position.getX(),
                                  position.getY(),
                                  position.getZ());

                if (object->mPrintDebugOutput) {
                    std::cout << object->mDebugOutputPrefix << "updated through setPosition!" << std::endl;
                }
            }
        }

        // traverse to children and update them too
        const std::list<MyObjectNode*>& children = object->getChildren();

        for(std::list<MyObjectNode*>::const_iterator child = children.begin();
            child != children.end();
            ++child) {

            updateOgreObject(*child, forced);
        }
    }
}


/*!
 * Adds \p object to the internal data structures and triggers creation of the
 * ogre data structures.
 *
 * \param object The object to be added to the internal data structures. It must
 * not have any parent and it must not have been added already before, i.e.
 * call this method only once per proxy.
 *
 * OWNERSHIP NOTICE:
 * The ownership of \p object and of the \p object->getProxy() object. Both are
 * deleted on destruction of this class.
 */
void SceneBase::addTopLevelObject(MyObjectNode* object) {
    if (!object) {
        return;
    }
    if (!object->getProxy()) {
        std::cerr << "NULL proxy!" << std::endl;
        delete object;
        return;
    }
    if (object->getProxy()->getParent()) {
        std::cerr << "TopLevel proxy must not have any parent!" << std::endl;

        delete object->getProxy();
        delete object;
        
        return;
    }

    mCollisionWorld->addProxy(object->getProxy());
    
    Ogre::SceneNode* sceneNode = object->getOgreSceneNode();
    if (!sceneNode) {
        sceneNode = mOgreSceneObjectFactory->createSceneHierarchy(object);
        
        // mark the Ogre mesh "dynamic" for deformables
        if (object->isDeformable()) {
            if (sceneNode->numAttachedObjects() > 0) {
                Ogre::ManualObject* manualObject = static_cast<Ogre::ManualObject*>(sceneNode->getAttachedObject(0));
                manualObject->setDynamic(true);
            }
        }
        object->setOgreSceneNode(sceneNode);
    }
    
    mSceneManager->getRootSceneNode()->addChild(sceneNode);
    
    if (mInitializeSceneCalled) {
        initToplevelObject(object);
    }
}

/*!
 * Internal method - you should never call this directly.
 *
 * Initialize \p object for use in the scene. This method should be called for
 * all (toplevel) objects when the scene is initialized and from that point on
 * whenever a (toplevel) object is added to the scene.
 *
 * This method in particular initializes OGRE objects to represent the bounding
 * volume hierarchie(s) of \ref object.
 *
 * See also \ref initScene
 * and \ref addTopLevelObject
 */
void SceneBase::initToplevelObject(MyObjectNode* object) {
    std::list<Ogre::SceneNode*> ogreBvhSceneNodes;

    const dcollide::BvhNode* boundingVolumeNode = object->getProxy()->getBvHierarchyNode();
    Ogre::SceneNode* boundingVolumeSceneNode = mOgreSceneObjectFactory->createBoundingVolumeSceneHierarchy(boundingVolumeNode);
    if (boundingVolumeSceneNode) {
        mBoundingVolumes->addChild(boundingVolumeSceneNode);
        ogreBvhSceneNodes.push_back(boundingVolumeSceneNode);
    }

    if (object->getProxy()->getProxyType() & dcollide::PROXYTYPE_DEFORMABLE) {
        unsigned int algorithms = mCollisionWorld->getDetectorDeformManager()->getAlgorithms().size();
        for (unsigned int index = 0; index < algorithms; index++) {
            dcollide::DetectorDeformProxyData* data = object->getProxy()->getDetectorDeformProxyData(index);
            if (!data) {
                continue;
            }
            const dcollide::BvhNode* boundingVolumeNodeDeform = data->getDeformableBvhNode();
            if (boundingVolumeNodeDeform) {
                Ogre::SceneNode* boundingVolumeSceneNodeDeform = mOgreSceneObjectFactory->createBoundingVolumeSceneHierarchy(boundingVolumeNodeDeform);

                if (boundingVolumeSceneNodeDeform) {
                    mBoundingVolumes->addChild(boundingVolumeSceneNodeDeform);
                    ogreBvhSceneNodes.push_back(boundingVolumeSceneNodeDeform);
                }
            }
        }
    }
    object->setOgreBVSceneNodes(ogreBvhSceneNodes);
}

Ogre::Camera* SceneBase::getCamera() const {
    return mOgreCamera;
}

void SceneBase::applyCollisions(const dcollide::WorldCollisions& collisions) {
    const dcollide::BroadPhaseCollisions* broad = collisions.getBroadPhaseCollisions();
    const std::list<dcollide::BoundingVolumeCollision>& rigid = collisions.getRigidBoundingVolumeCollisions();
    const std::list<dcollide::CollisionInfo>& narrow = collisions.getNarrowPhaseCollisions();

    visualizeBroadPhaseCollisions(mBroadPhaseCollisions, false);
    mBroadPhaseCollisions.clear();

    const std::list<dcollide::CollisionPair>& BPResults= broad->getResults();
    for (std::list<dcollide::CollisionPair>::const_iterator it = BPResults.begin(); it != BPResults.end(); ++it) {
        dcollide::CollisionPair pair = *it;
        if (!pair.bvol1 || !pair.bvol2) {
            std::cerr << dc_funcinfo << "NULL BV" << std::endl;
            continue;
        }
        dcollide::Proxy* p1 = pair.bvol1->getHierarchyNode()->getProxy();
        dcollide::Proxy* p2 = pair.bvol2->getHierarchyNode()->getProxy();
        if (!p1 || !p2) {
            std::cerr << dc_funcinfo << "NULL proxy" << std::endl;
            continue;
        }
        if (!p1->mUser1 || !p2->mUser1) {
            std::cerr << dc_funcinfo << "NULL mUser1" << std::endl;
            continue;
        }
        MyObjectNode* o1 = (MyObjectNode*)p1->mUser1;
        MyObjectNode* o2 = (MyObjectNode*)p2->mUser1;

        mBroadPhaseCollisions.push_back(o1);
        mBroadPhaseCollisions.push_back(o2);
    }
    visualizeBroadPhaseCollisions(mBroadPhaseCollisions, true);


    visualizeRigidBoundingVolumeCollisions(mMiddlePhaseRigidCollisions, false);
    mMiddlePhaseRigidCollisions.clear();

    for (std::list<dcollide::BoundingVolumeCollision>::const_iterator it = rigid.begin(); it != rigid.end(); ++it) {
        const dcollide::BvhNode* node1 = (*it).node1;
        const dcollide::BvhNode* node2 = (*it).node2;
        Ogre::ManualObject* o1 = (Ogre::ManualObject*)node1->mUser;
        Ogre::ManualObject* o2 = (Ogre::ManualObject*)node2->mUser;
        while (!o1 && node1->getParent()) {
            node1 = node1->getParent();
            o1 = (Ogre::ManualObject*)node1->mUser;
        }
        while (!o2 && node2->getParent()) {
            node2 = node2->getParent();
            o2 = (Ogre::ManualObject*)node2->mUser;
        }

        mMiddlePhaseRigidCollisions.push_back(o1);
        mMiddlePhaseRigidCollisions.push_back(o2);
    }
    visualizeRigidBoundingVolumeCollisions(mMiddlePhaseRigidCollisions, true);

    // TODO: deformable middlephase collisions


    visualizeNarrowPhaseCollisions(mNarrowPhaseCollisionObjects, false);
    mNarrowPhaseCollisionObjects.clear();
    if (mNarrowPhaseResultsVisible) {
        for (std::list<dcollide::CollisionInfo>::const_iterator it = narrow.begin(); it != narrow.end(); ++it) {
            const dcollide::Proxy* p1 = (*it).penetratingProxy;
            const dcollide::Proxy* p2 = (*it).penetratedProxy;
            MyObjectNode* o1 = static_cast<MyObjectNode*>(p1->mUser1);
            MyObjectNode* o2 = static_cast<MyObjectNode*>(p2->mUser1);

            addNarrowPhaseCollision(o1, o2, *it);
        }
        visualizeNarrowPhaseCollisions(mNarrowPhaseCollisionObjects, true);
    }
}

void SceneBase::visualizeBroadPhaseCollisions(const std::list<MyObjectNode*>& collisions, bool display) {
    for (std::list<MyObjectNode*>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
        (*it)->setBroadPhaseCollision(display);
    }
}

void SceneBase::visualizeRigidBoundingVolumeCollisions(const std::list<Ogre::ManualObject*>& collisions, bool display) {
    if (display) {
        for (std::list<Ogre::ManualObject*>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
            for (unsigned int i = 0; i < (*it)->getNumSections(); i++) {
                (*it)->setMaterialName(i, OgreMaterials::getBoundingVolumeMaterial(false, true));
            }
        }
    } else {
        for (std::list<Ogre::ManualObject*>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
            for (unsigned int i = 0; i < (*it)->getNumSections(); i++) {
                (*it)->setMaterialName(i, OgreMaterials::getBoundingVolumeMaterial());
            }
        }
    }
}

void SceneBase::visualizeNarrowPhaseCollisions(const std::set<MyObjectNode*>& collisions, bool display) {
    if (display) {
        for (std::set<MyObjectNode*>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
            if (!(*it)->getOgreNarrowPhaseResultsSceneNode()) {
                Ogre::SceneNode* node = mSceneManager->createSceneNode();
                (*it)->setOgreNarrowPhaseResultsSceneNode(node);
            }

            (*it)->createNarrowPhaseCollisions();
            (*it)->updateMaterialForNarrowPhaseResults(true);

            if (!(*it)->getOgreNarrowPhaseResultsSceneNode()->getParentSceneNode()) {
                mNarrowPhaseResults->addChild((*it)->getOgreNarrowPhaseResultsSceneNode());
            }
        }
    } else {
        for (std::set<MyObjectNode*>::const_iterator it = collisions.begin(); it != collisions.end(); ++it) {
            (*it)->clearNarrowPhaseCollisions();
            (*it)->updateMaterialForNarrowPhaseResults(false);
            if ((*it)->getOgreNarrowPhaseResultsSceneNode()) {
                mNarrowPhaseResults->removeChild((*it)->getOgreNarrowPhaseResultsSceneNode());
            }
        }
    }
}

void SceneBase::addNarrowPhaseCollision(MyObjectNode* penetratingObject, MyObjectNode* penetratedObject, const dcollide::CollisionInfo& c) {
    // AB: o1 and o2 might be used by the MyObjectNode to decide whether a
    // collision should be displayed.
    //
    // e.g. the user could click on a proxy and only those collisions that
    // belong to that proxy are displayed (i.e. either o1 or o2 is that proxy).
    // for that we'd need to store o1 and o2 in the MyObjectNode.
    //
    // however atm that is not supported, so we simply add the collision to one
    // proxy.

    penetratingObject->addNarrowPhaseCollision(c);
    penetratingObject->setPenetratingObject(true);
    penetratedObject->setPenetratedObject(true);
    mNarrowPhaseCollisionObjects.insert(penetratingObject);
    mNarrowPhaseCollisionObjects.insert(penetratedObject);
}

// AB: WARNING: we show/hide the whole scenenode, so don't attach other ogre
// objects to it!
/*!
 * Hides or shows all proxies (i.e. actual objects) in the scene. This does NOT
 * affect bounding volumes. This method may be useful to look at bounding
 * volumes of objects only.
 */
void SceneBase::setAllProxiesVisible(bool v) {
    const std::list<dcollide::Proxy*>& proxies = mCollisionWorld->getTopLevelProxies();
    std::list<dcollide::Proxy*>::const_iterator it;
    for (it = proxies.begin(); it != proxies.end(); ++it) {
        setProxyAndChildsVisible(*it, v);
    }
}

// AB: WARNING: we show/hide the whole scenenode, so don't attach other ogre
// objects to it!
void SceneBase::setProxyAndChildsVisible(dcollide::Proxy* proxy, bool v) {
    if (!proxy) {
        return;
    }
    if (!proxy->mUser1) {
        std::cerr << "Oops: proxy " << proxy << " not properly handled: NULL mUser1 pointer" << std::endl;
        return;
    }
    MyObjectNode* object = static_cast<MyObjectNode*>(proxy->mUser1);
    Ogre::SceneNode* node = object->getOgreSceneNode();

    node->setVisible(v, true);
}

void SceneBase::setAllBoundingVolumesVisible(bool v) {
    mBoundingVolumesVisible = v;

    mBoundingVolumes->setVisible(mBoundingVolumesVisible);
    if (mBoundingVolumesVisible) {
        if (!mBoundingVolumes->getParentSceneNode()) {
            mSceneManager->getRootSceneNode()->addChild(mBoundingVolumes);
        }
    } else {
        if (mBoundingVolumes->getParentSceneNode()) {
            mSceneManager->getRootSceneNode()->removeChild(mBoundingVolumes);
        }
    }

    if (mBoundingVolumesVisible) {
        // objects might have been moved while the BVs were not visible, so we
        // need to update them all.
        //
        // note that in particular the mMovableObjects list might have changed,
        // too, so we don't even know which objects have moved since the last BV
        // update
        std::list<MyObjectNode*> allTopObjects;

        const std::list<dcollide::Proxy*>& topLevelProxies = mCollisionWorld->getTopLevelProxies();
        for (std::list<dcollide::Proxy*>::const_iterator it = topLevelProxies.begin(); it != topLevelProxies.end(); ++it) {
            if ((*it)->mUser1) {
                allTopObjects.push_back(static_cast<MyObjectNode*>((*it)->mUser1));
            }
        }

        while (!allTopObjects.empty()) {
            MyObjectNode* topObject = allTopObjects.front();
            allTopObjects.pop_front();

            bool update = false;
            std::list<MyObjectNode*> objects;
            objects.push_back(topObject);
            while (!objects.empty() && !update) {
                MyObjectNode* n = objects.front();
                objects.pop_front();

                update = n->getBoundingVolumeDirty();
                if (!update) {
                    dcollide::Proxy* p = n->getProxy();
                    const std::list<dcollide::Proxy*>& children = p->getChildProxies();
                    for (std::list<dcollide::Proxy*>::const_iterator it = children.begin(); it != children.end(); ++it) {
                        if ((*it)->mUser1) {
                            objects.push_back(static_cast<MyObjectNode*>((*it)->mUser1));
                        }
                    }
                }
            }

            if (update) {
                // either this object, or one of its children (or
                // grand-children, or ...) needs a BV update.
                // -> do a recursive update.
                topObject->recalculateBoundingVolumeOgreObjects();
            }
        }
    }
}

void SceneBase::setHierarchicalGridVisible(bool visible) {
    if (!mHierarchicalGridSceneNode) {
        return;
    }
    if (visible) {
        if (!mHierarchicalGridSceneNode->getParentSceneNode()) {
            mSceneManager->getRootSceneNode()->
                addChild(mHierarchicalGridSceneNode);
        }

        recalculateHierarchicalGrid();
    } else {
        if (mHierarchicalGridSceneNode->getParentSceneNode()) {
            mSceneManager->getRootSceneNode()->
                removeChild(mHierarchicalGridSceneNode);
        }
    }
}

void SceneBase::setNarrowPhaseResultsVisible(bool visible) {
    mNarrowPhaseResultsVisible = visible;
    if (mNarrowPhaseResultsVisible) {
        if (!mNarrowPhaseResults->getParentSceneNode()) {
            mSceneManager->getRootSceneNode()->addChild(mNarrowPhaseResults);
        }

        // TODO: retrieve narrowphase results for current frame somehow?

    } else {
        if (mNarrowPhaseResults->getParentSceneNode()) {
            mSceneManager->getRootSceneNode()->removeChild(mNarrowPhaseResults);
        }
    }

    for (std::set<MyObjectNode*>::const_iterator it = mNarrowPhaseCollisionObjects.begin(); it != mNarrowPhaseCollisionObjects.end(); ++it) {
        (*it)->updateMaterialForNarrowPhaseResults(visible);
    }
}

void SceneBase::recalculateHierarchicalGrid() {
    if (!mHierarchicalGridSceneNode) {
        return;
    }
    if (mHierarchicalGridSceneNode->numAttachedObjects() != 1) {
        std::cerr << dc_funcinfo
                << "ERROR: expected exactly 1 attached object for grid scene"
                << "node. have: "
                << mHierarchicalGridSceneNode->numAttachedObjects()
                << std::endl;
        return;
    }
    Ogre::MovableObject* movable =
            mHierarchicalGridSceneNode->getAttachedObject(0);
    if (movable->getMovableType() != Ogre::ManualObjectFactory::FACTORY_TYPE_NAME) {
        std::cerr << dc_funcinfo << "attached object is not a ManualObject" << std::endl;
        return;
    }
    dcollide::BroadPhase* broad_ = mCollisionWorld->getBroadPhase();
    dcollide::BroadPhaseHierarchicalGrid* broad =
            dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>(broad_);
    if (!broad) {
        // only broadphases of type HIERARCHICALGRID have a grid
        return;
    }
    const dcollide::HierarchicalGrid* grid = broad->getHierarchicalGrid();
    if (!grid) {
        std::cerr << dc_funcinfo << "oops" << std::endl;
        return;
    }
    Ogre::ManualObject* ogreObject = static_cast<Ogre::ManualObject*>(movable);
    ogreObject->beginUpdate(0);
    OgreSceneObjectFactory::specifyHierarchicalGrid(ogreObject, grid);
    ogreObject->end();
}

/*!
* Creates and adds many rigid boxes to the scene on a (internally defined)
 * grid. The boxes are rotated in various directions/angles.
 */
void SceneBase::createManyRigidBoxes() {
    const float minX = -500.0f;
    const float minY = -500.0f;
    const float maxX = -minX;
    const float maxY = -minY;
    float rotX = 0.0f;
    float rotY = 0.0f;
    float rotZ = 0.0f;
    for (float x = minX; x <= maxX; x += 30.0f) {
        for (float y = minY; y <= maxY; y += 50.0f) {
            MyObjectNode* box =
                new MyObjectNode(
                    getCollisionWorld(),
                    //new dcollide::Wedge(10.0, 20.0, 5.0));
                    //new dcollide::Cylinder(10.0, 25.0, 3.0));
                    //new dcollide::Sphere(10.0, 3.0));
                    //new dcollide::Cone(7.0, 10.0, 2));
                    new dcollide::Box(dcollide::Vector3(10.0, 10.0, 10.0)));
            dcollide::Vector3 translation = dcollide::Vector3(x, y, 0.0f);
            dcollide::Matrix rot;
            rot.rotate(rotX, 1.0f, 0.0f, 0.0f);
            rot.rotate(rotY, 0.0f, 1.0f, 0.0f);
            rot.rotate(rotZ, 0.0f, 0.0f, 1.0f);
            box->translate(translation);
            box->rotate(rot);
            addTopLevelObject(box);

            rotX += 10.0f;
            if (rotX >= 360.0f) {
                rotX = 0.0f;
                rotY += 10.0f;
                if (rotY >= 360.0f) {
                    rotY = 0.0f;
                    rotZ += 10.0f;
                    if (rotZ >= 360.0f) {
                        rotZ = 0.0f;
                    }
                }
            }
        }
    }
}

void SceneBase::calculateCollisions(unsigned int flags) {
    delete mWorldCollisions;
    mWorldCollisions = new dcollide::WorldCollisions(getCollisionWorld()->calculateAllCollisions(flags));
}

dcollide::WorldCollisions* SceneBase::getWorldCollisions() const {
    return mWorldCollisions;
}

/*!
 * \brief deletes all collision related information from the current frame
 */
void SceneBase::deleteCollisions() {
    //delete old collisions
    delete mWorldCollisions;
    mWorldCollisions=0;
    mBroadPhaseCollisions.clear();
    mMiddlePhaseRigidCollisions.clear();
    mNarrowPhaseCollisionObjects.clear();

}

void SceneBase::restart() {
    std::cout << "INFO: restart feature is not implemented for this scene." <<std::endl;
}

void SceneBase::action() {
    std::cout << "INFO: action feature is not implemented for this scene." <<std::endl;
}
/*
 * vim: et sw=4 ts=4
 */
