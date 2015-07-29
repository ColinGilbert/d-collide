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


#include "world.h"

#include "worldcollisions.h"
#include "broadphase/broadphasefactory.h"
#include "broadphase/broadphase.h"
#include "narrowphase/narrowphasejob.h"
#include "proxy.h"
#include "bvhnode.h"
#include "timing.h"
#include "debug.h"
#include "collisioncache.h"
#include "thread/threadpool.h"
#include "thread/threadjobcollection.h"
#include "dcollide-config.h"
#include "dcollide-defines.h"
#include "proxyfactory.h"
#include "pipeline.h"
#include "debuglog.h"
#include "debugstream.h"
#include "detectordeform/detectordeformmanager.h"
#include "meshsplitter.h"

#include <list>
#include <vector>

namespace dcollide {
    /*!
     * \brief Constructor for world-class
     *
     */
    World::World() {
        WorldParameters parameters;
        init(parameters);
    }

    /*!
     * \brief Constructor for world-class
     *
     * See \ref World::setWorldMinMax for hints about the worldsize
     *
     * \param worldParameters Parameter to this world object, see \ref
     * WorldParameters
     */
    World::World(const WorldParameters& worldParameters) {
        init(worldParameters);
    }

    /*!
     * \overload
     *
     *
     * See \ref World::setWorldMinMax for hints about the worldsize
     *
     * See also \ref WorldParameters::setWorldDimension
     */
    World::World(const Vector3& worldDimension) {
        WorldParameters parameters;
        parameters.setWorldDimension(worldDimension);
        init(parameters);
    }

    /*!
     * \overload
     *
     *
     * See \ref World::setWorldMinMax for hints about the worldsize
     *
     * See also \ref WorldParameters::setWorldDimension
     */
    World::World(const Vector3& worldMin, const Vector3& worldMax) {
        WorldParameters parameters;
        parameters.setWorldMinMax(worldMin, worldMax);
        init(parameters);
    }

    void World::init(const WorldParameters& worldParameters) {
        mDebugLog = new DebugLog();
        mCurrentDebugLogEntry = 0;
        mWorldParameters = worldParameters;
        mProxyFactory = new ProxyFactory(this);
        mWorkerPool = new ThreadPool(worldParameters.getWorkerThreadCount());
        if (worldParameters.getWorkerThreadCount() == 0) {
            mWorkerPool->setDisableThreads(true);
        }
        mJobPoolMiddlePhaseRigid = mWorkerPool->addJobPool("Rigid");
        mPrepareSimulationCalled = false;
        mNumberOfTopLevelProxies = 0;
        mPipeline = new Pipeline(this);

        mUseCollisionCaching = true;
        mCollisionCache = new CollisionCache(mPipeline);

        mBroadphase = BroadPhaseFactory(this).createBroadPhase(
                          worldParameters.getBroadPhaseType()
                      );
        mDetectorDeformManager = 0;
        setDetectorDeformManager(new DetectorDeformManager(this));
    }

    /*!
     * \brief d'tor for world-class
     */
    World::~World() {
        delete mProxyFactory;
        delete mWorkerPool;
        delete mBroadphase;
        delete mCollisionCache;
        delete mDetectorDeformManager;
        delete mPipeline;
        for (std::list<Proxy*>::const_iterator iter =
            mTopLevelProxies.begin();
            iter != mTopLevelProxies.end();
            ++iter) {
            delete *iter;
        }
        delete mDebugLog;
    }

    /*! \brief add a proxy to the world.
     * OWNERSHIP NOTICE: the world takes ownership of the proxy
     */
    void World::addProxy(Proxy* proxy) {
        if (!proxy) {
            throw NullPointerException("Parameter \"Proxy* p\"");
        }
        if (proxy->getWorld() != this) {
            throw InvalidWorldException("Parameter proxy was not created by this World");
        }
        if (proxy->isInHierarchy()) {
            throw Exception("addProxy: proxy has already been added to a hierarchy");
        }

        // add it to the all-Proxies-list:
        mTopLevelProxies.push_back(proxy);
        proxy->setIsInHierarchy(true);

        // Checking the type of the proxy and add it to special-proxy-lists:
        if (proxy->getProxyType() & PROXYTYPE_RIGID) {
            mRigidProxies.push_back(proxy);
        }
        if (proxy->getProxyType() & PROXYTYPE_DEFORMABLE) {
            mDeformableProxies.push_back(proxy);

            if (mPrepareSimulationCalled) {
                mDetectorDeformManager->addTopLevelProxy(proxy);
            }
        }
        if (proxy->getProxyType() & PROXYTYPE_SELFCOLLIDABLE) {
            mSelfcollisionProxies.push_back(proxy);     
        }

        // increae proxyCounter:
        ++mNumberOfTopLevelProxies;

        // Add proxy to broadphase if broadphase is already running:
        if (mBroadphase->getInitCalled()) {
            mBroadphase->addProxy(proxy);
        }
    }

    /*!
     * \brief remove proxy from the world.
     *
     * OWNERSHIP NOTICE: In this case the caller takes ownership, as you may
     * want to re-add this proxy again!
     *
     * FIXME: replace remove (linear time ) with something else, perhaps
     * save an iterator (in the world) of each list the proxy was added
     * to and use this to list::erase the proxy
     */
    void World::removeProxy(Proxy* proxy) {

        // Null-Pointer Check:
        if (!proxy) {
            throw NullPointerException("parameter \"Proxy* proxy\"");
        }
        if (proxy->getParent()) {
            throw Exception("Cannot remove non-toplevel proxies from world (must be removed from parent instead!)");
        }
        if (!proxy->isInHierarchy()) {
            return;
        }

        // Checking type of the proxy and remove it from special-proxy-lists:
        if (proxy->getProxyType() & PROXYTYPE_RIGID) {
            mRigidProxies.remove(proxy);
        }
        if (proxy->getProxyType() & PROXYTYPE_DEFORMABLE) {
            mDeformableProxies.remove(proxy);

            if (mPrepareSimulationCalled) {
                mDetectorDeformManager->removeTopLevelProxy(proxy);
            }
        }
        if (proxy->getProxyType() & PROXYTYPE_SELFCOLLIDABLE) {
            mSelfcollisionProxies.remove(proxy);
        }

        // remove it from the all-Proxies-list:
        mTopLevelProxies.remove(proxy);

        mProxyStateChanged.removeAll(proxy);

        proxy->setIsInHierarchy(false);

        // Add proxy to broadphase if broadphase is already running:
        if (mBroadphase->getInitCalled()) {
            mBroadphase->removeProxy(proxy);
        }

        // decrease proxyCounter:
        --mNumberOfTopLevelProxies;
    }

    /*! \brief calls all preprocessing functions
     * The user must call this function once, after setting up the world
     */
    void World::prepareSimulation() {

        // only do this if not already called:
        if (mPrepareSimulationCalled) {
            return;
        }

        //TODO all modules: call your preprocessing functions from here

        //BroadPhase: Build the Oct-Tree and fill it with the Top-Level-BVs
        mBroadphase->init();
        //Deformable: preprocessing for all elements in mDeformableProxies
        mDetectorDeformManager->prepareAlgorithmsForSimulation();

        //save initial states of the proxies
        startNextStep();

        // Set flag mPrepareSimulationCalled:
        mPrepareSimulationCalled = true;
    }

    /*!
     *\brief start next simulation step

     * Calling this method enables you to use \ref undoStep and makes e.g. \ref
     * Proxy::getPreviousWorldMatrix return meaningful values.
     */
    void World::startNextStep() {
        for (ListNode<Proxy*>* node = mProxyStateChanged.getFirstNode(); node; node = node->getNext()) {
            node->getData()->discardPreviousState();
        }
        mProxyStateChanged.clear();
    }

    /*!
     * \brief \em Internal method called by \ref Proxy.
     *
     * This method should \em not be called manually. It is provided for use by
     * \ref Proxy only, which notifies the World that \p proxy or one of its
     * children has changed since
     * the last call to \ref startNextStep and needs to be restored, if \ref
     * undoStep is called.
     */
    void World::notifyProxyStateChangedSincePreviousStep(Proxy* proxy) {
        if (proxy->getParent()) {
            throw Exception("Expected toplevel proxy");
        }
        if (!proxy->isInHierarchy()) {
            throw Exception("Expected Proxy from hierarchy");
        }
        mProxyStateChanged.push_back(proxy);
    }

    /*!
     *
     * See \ref Proxy::restorePreviousState for details on what exactly is
     * restored.
     */
    void World::undoStep() {
        for (ListNode<Proxy*>* node = mProxyStateChanged.getFirstNode(); node; node = node->getNext()) {
            node->getData()->restorePreviousState();
        }
        mProxyStateChanged.clear();
    }

    void World::setUseCollisionCaching(bool activate) {
        //std::cout << "setUseCollicionCaching called"<<std::endl;
        if (mUseCollisionCaching && !activate) {
            //std::cout << "deactivated collision-caching"<<std::endl;
            mCollisionCache->clear();
        } else if (!mUseCollisionCaching && activate) {
            //std::cout << "activated collision-caching"<<std::endl;
        }
        mUseCollisionCaching = activate;
    }

    /*! \brief get all collisions of the great wide world
     * - Asks Broadphase
     * - delivers result to middlephase
     * - delivers result to narrowphase
     *
     * \param flags A set of \ref CollisionFlags, OR'ed together. These flags
     *        control the behaviour of this method.
     *
     * \param onlyCollisionsWith If NULL this method returns all collisions of all
     * proxies. If non-NULL, only collisions with \p onlyCollisionsWith are
     * returned, all proxies not colliding with this one are ignored.
     *
     * \returns A list of all collisions
     */
    WorldCollisions World::calculateAllCollisions(unsigned int flags, Proxy* onlyCollisionsWith) {
        mCurrentDebugLogEntry = mDebugLog->addEntry();

        bool usePipeline = true; // AB: NEVER set this to true again once it has been set to false!

        if (!(flags & COLLISIONFLAG_USE_THREADS)) {
            mWorkerPool->waitForCompletion(); // make sure all threads are sleeping
            mWorkerPool->setDisableThreads(true); // make sure threads won't get woken up

            // AB: this is important: no threads implies no pipeline!
            //     -> we will use usePipeline afterwards, so this must be set
            //        correctly
            usePipeline = false;
        } else {
            if (mWorkerPool->getDisableThreads()) {
                mWorkerPool->setDisableThreads(false);
            }
        }
        if (flags & COLLISIONFLAG_DISABLE_PIPELINING) {
            usePipeline = false;
        }
        if (flags & COLLISIONFLAG_SKIP_MIDDLE_PHASE) {
            // skip middle phase => skip narrow phase
            flags |= COLLISIONFLAG_SKIP_NARROW_PHASE;

            // AB: without middle and narrow phase, only the broadphase is left,
            // so pipelining makes no sense
            // (and in fact, the broadphase is only completed by later phases if
            // pipelining is enabled, so we must disable it here)
            flags |= COLLISIONFLAG_DISABLE_PIPELINING;
            usePipeline = false;
        }

        // Check if prepareSimulation was called, if not: call it!
        if (!mPrepareSimulationCalled) {
            prepareSimulation();
            std::cout << dc_funcinfo << "WARNING: World::prepareSimulation " <<
                "should be called by User!" << std::endl;
        }

        Timing time;

        mPipeline->setUsePipelining(usePipeline);

        WorldCollisions worldCollisions;
        mPipeline->calculateAllCollisions(flags, &worldCollisions, onlyCollisionsWith);

        time.stop();
        mCurrentDebugLogEntry->addTiming("calculateAllCollisions()", time);

        mCurrentDebugLogEntry = 0;

        return worldCollisions;
    }

    /*! \brief get all collisions of given \ref proxy
     * just calls \ref calculateAllCollisions with the correct params
     * \param onlyCollisionsWith check collisions for this proxy
     * \param flags A set of \ref CollisionFlags, OR'ed together. These flags
     *        control the behaviour of this method.
     *
     * \returns A list of all collisions
     */
    WorldCollisions World::calculateAllCollisionsWith(
            Proxy* onlyCollisionsWith, unsigned int flags) {
        return calculateAllCollisions(flags, onlyCollisionsWith);
    }

    /*! \brief Sets the size of the world
     *
     * Use this only to update the worldsize, set initial worldsize in
     * c'tor
     *
     * See \ref World::setWorldMinMax for hints about the worldsize
     *
     * The values provided should be something of pow(2,n), however if other
     * values are provided, they are automatically adjusted to the next power of 2.
     */
    void World::setWorldDimension(const Vector3& worldDimension) {
        bool changed = mWorldParameters.setWorldDimension(worldDimension);

        if (changed) {
            #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "WorldDimension changed, recreating Grid! " << std::endl;
            #endif
            mBroadphase->notifyWorldSizeChanged();
        }
    }

    /*! \brief Sets the min and max of the world
     *
     * The values provided should be something of pow(2,n), however if other
     * values are provided, the max values are automatically adjusted to the
     * next and the min values to the last power of 2.
     *
     * This function also updates the world dimension.
     *
     * The Worldsize is needed by the broadphase to build the internal
     * datastructure which holds all the objects.
     * Please note that the defined worldsize should have at least the same size
     * as the scene you are using. Even though the library will increase the
     * world's size automatically if something goes beyond the borders, this may
     * lead to a short time lag.
     *
     * \param adjustPower if false, values are not adjusted to the next power
     */
    void World::setWorldMinMax(const Vector3& worldMin, const Vector3& worldMax,
        const Vector3* proxyPosition) {
        bool changed = mWorldParameters.setWorldMinMax(
            worldMin, worldMax,proxyPosition);

        // Now update Broadphase if Worldmin/max has changed:
        if (changed) {
            #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "WorldDimension changed, recreating Grid! " << std::endl;
            #endif
            mBroadphase->notifyWorldSizeChanged();
        }
    }

    /*!
     * \return A pointer to the \ref ProxyFactory of this \ref World object.
     * This  pointer may be used to create new \ref Proxy objects.
     * This pointer remains valid for the lifetime of the \ref World
     * object and therefore may be stored for convenience.
     */
    ProxyFactory* World::getProxyFactory() const {
        return mProxyFactory;
    }

    /*!
     * Convenience method for \ref ProxyFactory::createProxy
     */
    Proxy* World::createProxy(ProxyTypes type) const {
        return getProxyFactory()->createProxy(type);
    }
    /*!
     * Convenience method for \ref ProxyFactory::createProxy
     */
    Proxy* World::createProxy(Shape* shape, ProxyTypes type) const {
        return getProxyFactory()->createProxy(shape, type);
    }

    Pipeline* World::getPipeline() const {
        return mPipeline;
    }

    /*!
     * Makes \p manager the new manager of deformable collision detection
     * algorithms. Any previous manager is deleted.
     *
     * OWNERSHIP NOTICE: this class takes ownership of \p manager and deletes it
     * on destruction.
     */
    void World::setDetectorDeformManager(DetectorDeformManager* manager) {
        delete mDetectorDeformManager;
        mDetectorDeformManager = manager;

        if (mPrepareSimulationCalled) {
            mDetectorDeformManager->createAlgorithms(getWorldParameters().getDeformableAlgorithms());
            mDetectorDeformManager->prepareAlgorithmsForSimulation();
        } else {
            // TODO: probably remove this line
            //       -> algorithms should be created by prepareSimulation(), NOT
            //          on construction, so that the user has a chance to
            //          replace the manager before the algorithms have been
            //          created
            mDetectorDeformManager->createAlgorithms(getWorldParameters().getDeformableAlgorithms());
        }

        mPipeline->setDetectorDeformManager(mDetectorDeformManager);
    }

    /*!
     * \return The currently set manager of deformable collision detection
     * algorithms, see \ref setDetectorDeformManager
     */
    DetectorDeformManager* World::getDetectorDeformManager() const {
        return mDetectorDeformManager;
    }

    /*!
     * \return A new MeshSplitter, according to the \ref MeshSplitterFactory
     * in \ref WorldParameters::getMeshSplitterFactory.
     *
     * OWNERSHIP NOTICE: the caller takes ownership of the returned object.
     */
    MeshSplitter* World::createMeshSplitter(Mesh* mesh, BvhNode* topBvhNode) {
        MeshSplitterFactory* factory = mWorldParameters.getMeshSplitterFactory();
        return factory->createMeshSplitter(this, mesh, topBvhNode);
    }
}
/*
 * vim: et sw=4 ts=4
 */
