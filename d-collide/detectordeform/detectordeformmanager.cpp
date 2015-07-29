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

#include "detectordeformmanager.h"

#include "detectordeformalgorithm.h"
#include "surfacehierarchy/surfacehierarchyalgorithm.h"
#include "spatialhash/spatialhashalgorithm.h"
#include "bvhierarchy/bvhierarchyalgorithm.h"
#include "proxy.h"
#include "world.h"
#include "exceptions/exception.h"
#include "debugstream.h"
#include "debug.h"

#include <algorithm>

namespace dcollide {
    DetectorDeformManager::DetectorDeformManager(World* world) {
        mPrepareSimulationCalled = false;
        mWorld = world;
    }

    /*!
     * Destuct this manager and delete all algorithms managed by this object.
     */
    DetectorDeformManager::~DetectorDeformManager() {
        for (std::vector<DetectorDeformAlgorithm*>::iterator it =
                mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            delete *it;
        }
    }

    /*!
     * Register \p listener as listener for \ref Proxy changes, i.e. ensure that
     * the methods
     * \li \ref DetectorDeformAlgorithm::translateProxy
     * \li \ref DetectorDeformAlgorithm::rotateProxy
     * \li \ref DetectorDeformAlgorithm::setProxyMatrix
     * \li \ref DetectorDeformAlgorithm::deformProxy
     * are called when appropriate. See these methods for details.
     *
     * You normally do not need to call this method: it is automatically called
     * by \ref addAlgorithm depending on \ref
     * DetectorDeformAlgorithm::getListenForProxyChanges.
     */
    void DetectorDeformManager::registerProxyChangesListener(DetectorDeformAlgorithm* listener) {
        if (!listener) {
            throw NullPointerException("listener");
        }
        if (std::find(mProxyChangesListeners.begin(), mProxyChangesListeners.end(), listener) != mProxyChangesListeners.end()) {
            return;
        }
        mProxyChangesListeners.push_back(listener);
    }

    /*!
     * Remove \p listener from the listener list. See also \ref
     * registerProxyChangesListener.
     */
    void DetectorDeformManager::unregisterProxyChangesListener(DetectorDeformAlgorithm* listener) {
        if (!listener) {
            throw NullPointerException("listener");
        }
        std::list<DetectorDeformAlgorithm*>::iterator it = mProxyChangesListeners.begin();
        while (it != mProxyChangesListeners.end()) {
            if ((*it) == listener) {
                it = mProxyChangesListeners.erase(it);
            } else {
                ++it;
            }
        }
    }

    /*!
     * This method decides which algorithm is used for the pair (\p proxy1,
     * \p proxy2). At least one of \p proxy1 and \p proxy2 must be deformable.
     *
     * The current implementation of this method is very simple:
     * \li Check if \p proxy1 == \p proxy2. In that case return the first
     *      self-collsions algorithm available.
     * \li Else (i.e. if \p proxy1 != \p proxy2) return the first pair algorithm
     *     available (i.e. an algorithm with \ref
     *     DetectorDeformAlgorithm::supportPairCollisions() == TRUE)
     * \li In both cases: if no such algorithm is available, throw an exception.
     */
    DetectorDeformAlgorithm* DetectorDeformManager::pickAlgorithmFor(Proxy* proxy1, Proxy* proxy2) {
        if (!proxy1) {
            throw NullPointerException("proxy1");
        }
        if (!proxy2) {
            throw NullPointerException("proxy2");
        }
        if (proxy1 == proxy2) {
            if (mSelfCollisionAlgorithms.empty()) {
                throw Exception("Self-collision algorithms for deformable proxies required, but non available");
            }
            return mSelfCollisionAlgorithms.front();
        }
        if (!mPairAlgorithms.empty()) {
            return mPairAlgorithms.front();
        }
        throw Exception("No pair-collision algorithm for deformable proxies available");
    }

    /*!
     * Create and add one \ref DetectorDeformAlgorithm object for every type
     * in \p list. Note that if \p list contains duplicates, only the first
     * occurance of every type is used, all others are ignored.
     *
     * See also \ref createAlgorithm and \ref addAlgorithm
     */
    void DetectorDeformManager::createAlgorithms(const std::list<DeformableAlgorithmType>& list) {
        std::list<DeformableAlgorithmType> _list;
        for (std::list<DeformableAlgorithmType>::const_iterator it = list.begin(); it != list.end(); ++it) {
            if (std::find(_list.begin(), _list.end(), *it) == _list.end()) {
                // not a duplicate
                _list.push_back(*it);
            }
        }

        for (std::list<DeformableAlgorithmType>::const_iterator it = _list.begin(); it != _list.end(); ++it) {
            DetectorDeformAlgorithm* algorithm = createAlgorithm(*it);
            if (algorithm) {
                addAlgorithm(algorithm);
            } else {
                std::stringstream ss;
                ss << "Unknown deformable algorithm of type " << *it;
                throw Exception(ss.str());
            }
        }
    }

    /*!
     * Create a new algorithm of type \p type.
     *
     * \return a new algorithm of type \p type or NULL if \p type is not
     * recognized.
     */
    DetectorDeformAlgorithm* DetectorDeformManager::createAlgorithm(DeformableAlgorithmType type) const {
        switch (type) {
            case DEFORMABLE_TYPE_SPATIAL_HASH:
                return new SpatialHashAlgorithm(mWorld->getPipeline());
            case DEFORMABLE_TYPE_SURFACE_HIERARCHY:
                return new SurfaceHierarchyAlgorithm(mWorld, mWorld->getPipeline());
            case DEFORMABLE_TYPE_BV_HIERARCHY:
                return new BvHierarchyAlgorithm(mWorld, mWorld->getPipeline());
        }
        return 0;
    }

    /*!
     * Add \p algorithm to the internal list of available algorithms.
     *
     * OWNERSHIP NOTICE: this method takes ownership of the \p algorithm object
     */
    void DetectorDeformManager::addAlgorithm(DetectorDeformAlgorithm* algorithm) {
        if (!algorithm) {
            throw NullPointerException("algorithm");
        }

        algorithm->setProxyDataIndex(mAllAlgorithms.size());
        mAllAlgorithms.push_back(algorithm);

        if (algorithm->supportsSelfCollisions()) {
            mSelfCollisionAlgorithms.push_back(algorithm);
        }
        if (algorithm->supportsPairCollisions()) {
            mPairAlgorithms.push_back(algorithm);
        }

        if (mPrepareSimulationCalled) {
            algorithm->prepareSimulation();
        }

        if (algorithm->getListenForProxyChanges()) {
            registerProxyChangesListener(algorithm);
        }
    }

    /*!
     * Prepare all algorithms that have been added to this manager for the
     * simulation. That usually means to create data structures (e.g. bounding
     * volumes) required for the algorithm. See also \ref
     * DetectorDeformAlgorithm::prepareSimulation.
     *
     * This is called exactly once by the \ref World object that this manager
     * belongs to, when \ref World::prepareSimulation is called.
     */
    void DetectorDeformManager::prepareAlgorithmsForSimulation() {
        if (mPrepareSimulationCalled) {
            return;
        }
        mPrepareSimulationCalled = true;
        const std::list<Proxy*>& deformableProxies = mWorld->getDeformableProxies();
        for (std::list<Proxy*>::const_iterator it = deformableProxies.begin(); it != deformableProxies.end(); ++it) {
            initializeProxy(*it);
        }
        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it = mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            DetectorDeformAlgorithm* algorithm = *it;
            algorithm->prepareSimulation();
        }
    }

    /*!
     * Initialize \p proxy to be used by the \ref DetectorDeformAlgorithm
     * objects. This in particular means to create \ref DetectorDeformProxyData
     * objects for \p proxy and all of its children.
     */
    void DetectorDeformManager::initializeProxy(Proxy* proxy) {
        if (!proxy) {
            throw NullPointerException("proxy");
        }
        std::vector<DetectorDeformProxyData*> datas;
        datas.reserve(mAllAlgorithms.size());
        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it =
                mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            datas.push_back((*it)->createProxyData(proxy));
        }
        proxy->setDetectorDeformProxyData(datas);
        for (std::list<Proxy*>::const_iterator it = proxy->getChildProxies().begin();
                it != proxy->getChildProxies().end(); ++it) {
            initializeProxy(*it);
        }
    }

    /*!
     * Internal function, used by \ref Pipeline. You should not need to call
     * this.
     *
     * Called when the pipeline has been started for a new collision detection
     * run. This triggers calls to the \ref PipelineThreadJobCollection::start
     * function of the job collections of the algorithms, which allows the job
     * collections to add their jobs to the \ref Pipeline.
     *
     * See \ref DetectorDeformAlgorithm::notifyPipelineStarted for details.
     */
    void DetectorDeformManager::notifyPipelineStarted() {
        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it = mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            DetectorDeformAlgorithm* algorithm = *it;
            algorithm->notifyPipelineStarted();
        }
    }

    /*!
     * Internal function, used by \ref Pipeline. You should not need to call
     * this.
     *
     * Called when the pipeline has completed a collision detection
     * run. This triggers calls to the \ref
     * PipelineThreadJobCollection::resetCollection
     * function of the job collections of the algorithms, so that they remove
     * the "completed" flag and are prepared for the next pipeline run.
     *
     * Note that at this point all jobs \em must already be completed and their
     * results must have been read alreaedy.
     *
     * See \ref DetectorDeformAlgorithm::notifyPipelineCompleted for details.
     */
    void DetectorDeformManager::notifyPipelineCompleted() {
        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it = mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            DetectorDeformAlgorithm* algorithm = *it;
            algorithm->notifyPipelineCompleted();
        }
    }

    void DetectorDeformManager::notifyAllCollisionPairsAreAdded() {
        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it = mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            DetectorDeformAlgorithm* algorithm = *it;
            algorithm->notifyAllCollisionPairsAreAdded();
        }
    }

    /*!
     * Called by \ref World::addProxy (if \ref prepareSimulation was already
     * called) to notify the manager that a new proxy has been added. This
     * method will initialize the datastructures of all algorithms for the \p
     * proxy.
     */
    void DetectorDeformManager::addTopLevelProxy(Proxy* proxy) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        if (proxy->getParent()) {
            throw Exception("Proxy is not a toplevel proxy!");
        }
        initializeProxy(proxy);
        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it =
                mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            (*it)->addTopLevelProxy(proxy);
        }
    }

    /*!
     * Called by \ref World::removeProxy (if \ref prepareSimulation was already
     * called) to notify the manager that a proxy has been removed.
     *
     * This
     * method will clean up the datastructures of all algorithms for the \p
     * proxy.
     */
    void DetectorDeformManager::removeTopLevelProxy(Proxy* proxy) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        if (proxy->getParent()) {
            throw Exception("Proxy is not a toplevel proxy!");
        }

        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it =
                mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            (*it)->removeTopLevelProxy(proxy);
        }

        // clean DetectorDeformProxyData objects
        proxy->setDetectorDeformProxyData(std::vector<DetectorDeformProxyData*>());
    }

    /*!
     * Called by \ref Proxy::addProxy (if \ref prepareSimulation was already
     * called and the \p parent is already added to the \ref World, see \ref
     * World::addProxy) to notify the manager that a new proxy has been added to
     * another proxy. This
     * method will initialize the datastructures of all algorithms for the \p
     * child proxy.
     */
    void DetectorDeformManager::addChildProxy(Proxy* parent, Proxy* child) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        if (!child->getParent()) {
            throw Exception("Proxy is not a child proxy!");
        }
        initializeProxy(child);
        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it =
                mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            (*it)->addChildProxy(parent, child);
        }
    }

    /*!
     * Called by \ref Proxy::removeChild (if \ref prepareSimulation was already
     * called and the \p parent is already in the \ref World, see \ref
     * World::addProxy) to notify the manager that a proxy has been removed from
     * its parent.
     *
     * This
     * method will clean up the datastructures of all algorithms for the \p
     * child proxy.
     */
    void DetectorDeformManager::removeChildProxy(Proxy* parent, Proxy* child) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        if (!child->getParent()) {
            throw Exception("Proxy is not a child proxy!");
        }

        for (std::vector<DetectorDeformAlgorithm*>::const_iterator it =
                mAllAlgorithms.begin(); it != mAllAlgorithms.end(); ++it) {
            (*it)->removeChildProxy(parent, child);
        }

        // clean DetectorDeformProxyData objects
        child->setDetectorDeformProxyData(std::vector<DetectorDeformProxyData*>());
    }

    /*!
     * Called by \ref Proxy::translate if the following conditions are met:
     * \li The \p proxy is deformable (see \ref Proxy::getProxyType)
     * \li \ref prepareSimulation was already called
     * \li The \p proxy is already in the hierarchy (see \ref World::addProxy
     *     and \ref Proxy::addChild)
     */
    void DetectorDeformManager::translateProxy(Proxy* proxy, const Vector3& translateBy, bool respectOrientation) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        for (std::list<DetectorDeformAlgorithm*>::const_iterator it = mProxyChangesListeners.begin(); it != mProxyChangesListeners.end(); ++it) {
            (*it)->translateProxy(proxy, translateBy);
        }
    }

    /*!
     * Like \ref translateProxy, but this method is called by \ref
     * Proxy::rotate. See \ref translateProxy for details.
     */
    void DetectorDeformManager::rotateProxy(Proxy* proxy, const Matrix& rotation, bool respectOrientation) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        for (std::list<DetectorDeformAlgorithm*>::const_iterator it = mProxyChangesListeners.begin(); it != mProxyChangesListeners.end(); ++it) {
            (*it)->rotateProxy(proxy, rotation);
        }
    }

    /*!
     * Like \ref translateProxy, but this method is called by \ref
     * Proxy::setTransformation. See \ref translateProxy for details.
     */
    void DetectorDeformManager::setProxyMatrix(Proxy* proxy, const Matrix& newMatrix) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        for (std::list<DetectorDeformAlgorithm*>::const_iterator it = mProxyChangesListeners.begin(); it != mProxyChangesListeners.end(); ++it) {
            (*it)->setProxyMatrix(proxy, newMatrix);
        }
    }

    /*!
     * Like \ref translateProxy, but this method is called by \ref
     * Proxy::deform. See \ref translateProxy for details.
     */
    void DetectorDeformManager::deformProxy(Proxy* proxy, const std::vector<Vector3>& vertexMoveArray) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        for (std::list<DetectorDeformAlgorithm*>::const_iterator it = mProxyChangesListeners.begin(); it != mProxyChangesListeners.end(); ++it) {
            (*it)->deformProxy(proxy, vertexMoveArray);
        }
    }
    
    /*!
     * Like \ref translateProxy, but this method is called by \ref
     * Proxy::deform. See \ref translateProxy for details.
     */
    void DetectorDeformManager::deformProxy(Proxy* proxy, unsigned int vertexIndex, const Vector3& vertexPosition) {
        if (!mPrepareSimulationCalled) {
            return;
        }
        for (std::list<DetectorDeformAlgorithm*>::const_iterator it = mProxyChangesListeners.begin(); it != mProxyChangesListeners.end(); ++it) {
            (*it)->deformProxy(proxy, vertexIndex, vertexPosition);
        }
    }
}

/*
 * vim: et sw=4 ts=4
 */
