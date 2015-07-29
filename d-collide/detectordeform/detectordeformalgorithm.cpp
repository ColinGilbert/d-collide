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

#include "detectordeformalgorithm.h"

#include "pipeline.h"
#include "debugstream.h"
#include "debug.h"
#include "math/vector.h"
#include "math/matrix.h"
#include "proxy.h"

namespace dcollide {
    DetectorDeformAlgorithm::DetectorDeformAlgorithm(Pipeline* pipeline) {
        mPipeline = pipeline;
        mJobCollection = 0;
        mProxyDataIndex = 0;
    }

    DetectorDeformAlgorithm::~DetectorDeformAlgorithm() {
        delete mJobCollection;
    }

    /*!
     * Called by \ref DetectorDeformManager::addTopLevelProxy when a new \ref
     * Proxy was added to the \ref World (see \ref World::addProxy). This method
     * is only called for proxies added after \ref prepareSimulation was called.
     *
     * Derived classes may do preprocessing or other initialization on \p proxy.
     * The default implementation does nothing.
     */
    void DetectorDeformAlgorithm::addTopLevelProxy(Proxy* proxy) {
        DCOLLIDE_UNUSED(proxy);
    }

    /*!
     * Called by \ref DetectorDeformManager::removeTopLevelProxy when a \ref
     * Proxy was removed from the \ref World (see \ref World::removeProxy).
     * This method is only called for proxies removed after \ref
     * prepareSimulation was called.
     *
     * Derived classes may do final cleanups here.
     * Note that deletion of the \ref DetectorDeformProxyData is not necessary,
     * since the \ref DetectorDeformManager will take care of that.
     *
     * The default implementation does nothing.
     */
    void DetectorDeformAlgorithm::removeTopLevelProxy(Proxy* proxy) {
        DCOLLIDE_UNUSED(proxy);
    }

    /*!
     * Called by \ref DetectorDeformManager::addChildProxy when a new \ref
     * Proxy was added to another \ref Proxy(see \ref Proxy::addChild). This method
     * is only called for children added after \ref prepareSimulation was called
     * and only if the \p parent has already been added to the \ref World (see also
     * \ref addTopLevelProxy).
     *
     * Derived classes may do preprocessing or other initialization on \p child.
     * The default implementation does nothing.
     */
    void DetectorDeformAlgorithm::addChildProxy(Proxy* parent, Proxy* child) {
        DCOLLIDE_UNUSED(parent);
        DCOLLIDE_UNUSED(child);
    }

    /*!
     * Called by \ref DetectorDeformManager::removeChildProxy when a \ref
     * Proxy was removed from its parent \ref Proxy (see \ref
     * Proxy::removeChild).
     * This method is only called for proxies removed after \ref
     * prepareSimulation was called and only if the \p parent is already in the
     * \ref World (see also \ref addTopLevelProxy).
     *
     * Derived classes may do final cleanups here.
     * Note that deletion of the \ref DetectorDeformProxyData is not necessary,
     * since the \ref DetectorDeformManager will take care of that.
     *
     * The default implementation does nothing.
     */
    void DetectorDeformAlgorithm::removeChildProxy(Proxy* parent, Proxy* child) {
        DCOLLIDE_UNUSED(parent);
        DCOLLIDE_UNUSED(child);
    }

    /*!
     * Called by the \ref Pipeline once all possibly colliding Proxy
     * pairs have been added to this algorithm using \ref
     * createCollisionJobFor. In other words a call to this function means that
     * the \ref BroadPhase has been completed.
     *
     * This algorithm could do some final tasks, for example
     * creating a final collision detection job, if \ref
     * createCollisionJobFor does not actually create a job, but puts
     * the pairs in a queue.
     *
     * Note in particular that algorithms that return FALSE in \ref
     * supportsPairwiseProcessing are meant to start collision detection
     * here.
     *
     * If you overwrite this class, you either have to call the base
     * implementation or do the following tasks youself:
     * \li call \ref ThreadJobCollection::setAllJobsAreAdded
     * \li call \ref ThreadJobCollection::start
     *
     * If you use \ref
     * createCollisionJobFor for collision detection (the default usage)
     * you do no need to touch this method.
     */
    void DetectorDeformAlgorithm::notifyAllCollisionPairsAreAdded() {

        // AB: WARNING:
        // derived classes may reimplement this method without calling the base
        // implementation!
        // -> dont do too much important stuff here (or make sure all
        //    implementations do that stuff, too)

        getJobCollection()->setAllJobsAreAdded();

        // AB: note: this does not necessarily begin executing the jobs already
        //     -> PipelineThreadJobCollection::start() just adds the jobs to the
        //        pipeline. it is up to the pipeline when to actually start
        //        them.
        //        (usually however they will be started immediately)
        getJobCollection()->start();
    }

    /*!
     * Called when (in the current pipeline run) collision detection for this
     * algorithm is completed. At this point all jobs of this algorithm are
     * finished and no call to \ref createCollisionJobFor will occur before the
     * next pipeline run.
     *
     * This method may be used for cleanup tasks, i.e. clearing temporary
     * variables, lists and the like.
     *
     * This method is \em not meant to read results from the completed jobs. In
     * fact, the jobs may already have been deleted at this point. Use \ref
     * PipelineThreadJob::processResults, \ref
     * PipelineThreadJobCollection::allJobsCompleted and \ref
     * PipelineThreadJobCollection::setJobCompleted to read and process results.
     */
    void DetectorDeformAlgorithm::resetAlgorithm() {
    }

    /*!
     * The default implementation returns (and if necessary creates) a normal
     * \ref PipelineThreadJobCollection.
     *
     * Derived classes may reimplement this method to use a custom \ref
     * PipelineThreadJobCollection object.
     *
     * \return The \ref PipelineThreadJobCollection of this algorithm. A new
     * object is created, if necessary.
     */
    PipelineThreadJobCollection* DetectorDeformAlgorithm::getJobCollection() {
        if (!mJobCollection) {
            mJobCollection = new PipelineThreadJobCollection(mPipeline, Pipeline::PHASE_MIDDLEPHASE);
        }
        return mJobCollection;
    }

    /*!
     * Used \em internally to notify this algorithm that a new \ref Pipeline
     * run has been started and that this algorithm is meant to start
     * processing.
     *
     * This method has merely technical reasons only, you should \em never
     * need to call it directly. Only \ref Pipeline is meant to use it.
     *
     * Technical details:\n
     * This method calls \ref PipelineThreadJobCollection::start on the
     * \ref getJobCollection and therefore causes any \ref ThreadJob
     * objects already added, or that are added from this point on, to
     * be added to the \ref Pipeline, so they can be executed by it.
     *
     * This method is \em not meant to add any \ref ThreadJob objects.
     * Use \ref createCollisionJobFor and \ref
     * notifyAllCollisionPairsAreAdded for that.
     */
    void DetectorDeformAlgorithm::notifyPipelineStarted() {
        getJobCollection()->start();
    }

    /*!
     * Used \em internally to notify this algorithm that the current
     * \ref Pipeline run has been finished, i.e. all jobs have been processed
     * and the pipeline is about to return control to the caller.
     *
     * This method has merely technical reasons only, you should \em never
     * need to call it directly. Only \ref Pipeline is meant to use it.
     *
     * Technical details:\n
     * This method calls \ref PipelineThreadJobCollection::resetCollection
     * on the \ref getJobCollection. This is meant to clean out all "completed"
     * states of the job collection, so that the next pipeline run can be
     * started again. Note that at this point the results of all jobs in the
     * \ref getJobCollection have been read already! In fact all jobs of the
     * \ref getJobCollection may already be deleted. In addition, it also call
     * \ref resetAlgorithm to tell the algorithm to clean up after itself.
     *
     * This method is \em not meant to do any post-processing of collision
     * detection results. Use e.g. PipelineThreadJob::processResults or \ref
     * PipelineThreadJobCollection::allJobsCompleted or \ref
     * PipelineThreadJobCollection::setJobCompleted for that.
     */
    void DetectorDeformAlgorithm::notifyPipelineCompleted() {
        // TODO: check if there are still unprocessed jobs left in the
        // collection. if so, throw an exception

        getJobCollection()->resetCollection();

        resetAlgorithm();
    }

    /*!
     * Create a new \ref DetectorDeformProxyData object for use in \p proxy, or
     * NULL if this algorithm does not make use of these objects.
     *
     * OWNERSHIP NOTICE: the caller takes ownership on the returned pointer.
     *
     * \return NULL in the default implementation. Derived classes should return
     * a new object of a \ref DetectorDeformProxyData derived class.
     */
    DetectorDeformProxyData* DetectorDeformAlgorithm::createProxyData(Proxy* proxy) {
        DCOLLIDE_UNUSED(proxy);
        return 0;
    }

    /*!
     * Convenience function that returns the \ref DetectorDeformProxyData object
     * of \p proxy that was created by this algorithms (see \ref
     * createProxyData).
     *
     * This method returns NULL if
     * \li A NULL DetectorDeformProxyData object was created by \ref
     *     createProxyData
     * \li \p proxy is not deformable.
     *
     * \return See \ref Proxy::getDetectorDeformProxyData and \ref
     *         getProxyDataIndex.
     */
    DetectorDeformProxyData* DetectorDeformAlgorithm::getProxyDataFor(Proxy* proxy) const {
        if (!proxy) {
            throw NullPointerException("proxy");
        }
        if (!(proxy->getProxyType() & PROXYTYPE_DEFORMABLE)) {
            return 0;
        }
        return proxy->getDetectorDeformProxyData(getProxyDataIndex());
    }

    /*!
     * Set the proxydata index, i.e. the index that this algorithm uses for the
     * \ref DetectorDeformProxyData objects in \ref Proxy. See also \ref
     * createProxyData and \ref Proxy::getDetectorDeformProxyData.
     *
     * This method is called internally by \ref DetectorDeformManager, do \em
     * not call this manually!
     */
    void DetectorDeformAlgorithm::setProxyDataIndex(unsigned int index) {
        mProxyDataIndex = index;
    }

    /*!
     * Called by \ref DetectorDeformManager::translateProxy when a \ref
     * Proxy was translated (see \ref Proxy::translate).
     *
     * This method is meant for the algorithm to update internal data
     * structures. The default implementation does nothing.
     *
     * This method is only called when the following conditions are met:
     * \li The \p proxy is deformable (see \ref Proxy::getProxyType)
     * \li \ref prepareSimulation was already called
     * \li The \p proxy is already in the hierarchy (see \ref World::addProxy
     *     and \ref Proxy::addChild)
     * \li This algorithm subscribes to \ref Proxy changes (see \ref
     *     getListenForProxyChanges).
     */
    void DetectorDeformAlgorithm::translateProxy(Proxy* proxy, const Vector3& translateBy) {
        DCOLLIDE_UNUSED(proxy);
        DCOLLIDE_UNUSED(translateBy);
    }

    /*!
     * Like \ref translateProxy, but this method is called by \ref
     * DetectorDeformManager::rotateProxy.
     *
     * See \ref translateProxy for details.
     *
     * The default implementation does nothing.
     */
    void DetectorDeformAlgorithm::rotateProxy(Proxy* proxy, const Matrix& rotation) {
        DCOLLIDE_UNUSED(proxy);
        DCOLLIDE_UNUSED(rotation);
    }

    /*!
     * Like \ref translateProxy, but this method is called by \ref
     * DetectorDeformManager::setProxyMatrix
     *
     * See \ref translateProxy for details.
     *
     * The default implementation does nothing.
     */
    void DetectorDeformAlgorithm::setProxyMatrix(Proxy* proxy, const Matrix& newMatrix) {
        DCOLLIDE_UNUSED(proxy);
        DCOLLIDE_UNUSED(newMatrix);
    }

    /*!
     * Like \ref translateProxy, but this method is called by \ref
     * DetectorDeformManager::deformProxy
     *
     * See \ref translateProxy for details.
     *
     * The default implementation does nothing.
     */
    void DetectorDeformAlgorithm::deformProxy(Proxy* proxy, const std::vector<Vector3>& vertexMoveArray) {
        DCOLLIDE_UNUSED(proxy);
        DCOLLIDE_UNUSED(vertexMoveArray);
    }
    
    /*!
     * Like \ref translateProxy, but this method is called by \ref
     * DetectorDeformManager::deformProxy
     *
     * See \ref translateProxy for details.
     *
     * The default implementation does nothing.
     */
    void DetectorDeformAlgorithm::deformProxy(Proxy* proxy, unsigned int vertexIndex, const Vector3& vertexPosition) {
        DCOLLIDE_UNUSED(proxy);
        DCOLLIDE_UNUSED(vertexIndex);
        DCOLLIDE_UNUSED(vertexPosition);
    }

}

/*
 * vim: et sw=4 ts=4
 */
