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


#include "broadphase.h"
#include "broadphasecollisions.h"
#include "thread/threadpool.h"
#include "bvhnode.h"
#include "dcollide-config.h"
#include "world.h"
#include "debug.h"
#include "collisionpair.h"
#include "pipeline.h"

#include <math.h>

namespace dcollide {
    /*!
     *  \brief Creates new BroadPhase
     *  \param World-Class to get all Aabb's
     */
    BroadPhase::BroadPhase(World* world, BroadPhaseType type) {
        mWorld = world;
        mType = type;
        mInitCalled = false;

        mWorkerPool = mWorld->getWorkerPool();
        mJobPoolIndex = mWorkerPool->addJobPool("BroadPhase");
        mJobCollection = 0; // created by getJobCollection() (unless overwritten by derived classes)

        mBPCollisions = new BroadPhaseCollisions();

        // other members will be initialized by init(), which should be called
        // by user when all proxies are added to the world
    }

    /*!
     *  \brief Destroys Broadphase
     */
    BroadPhase::~BroadPhase() {
        delete mBPCollisions;
        delete mJobCollection;
    }

    /*!
     * Initialize the broadphase according to the current state of the \ref
     * getWorld.
     *
     * This method is called when \ref World::prepareSimulation is called, i.e.
     * at this point it is expected that the user has added most of the required
     * proxies to the world and wants to begin with collision detection. The
     * BroadPhase should now initialize required datastructures (e.g. build up
     * the tree) and should keep them up-to-date from this point on (see e.g.
     * \ref addProxy, \ref removeProxy, \ref notifyProxyChanged).
     *
     * Note that methods like \ref addProxy are \em not called before the
     * broadphase has been initialized.
     *
     * Derived classes reimplementing this method are required to call the base
     * implementation!
     */
    void BroadPhase::init() {
        mInitCalled = true;
    }


    /*!
     * \brief add Proxy to grid
     */
    void BroadPhase::addProxy(Proxy*) {
    }

    /*!
     * \brief remove Proxy from grid
     */
    void BroadPhase::removeProxy(Proxy*) {
    }

    /*!
     * \return The \ref BroadPhaseCollisions object, as filled with contents by
     * the broadphase jobs created by \ref createBroadPhaseJobs. Note that the contents of
     * that object may still change until the jobs created in \ref
     * createBroadPhaseJobs have been completed by the \ref Pipeline.
     * \param proxy If given, only collisions are calculated where this
     * \ref proxy is involved.
     */
    BroadPhaseCollisions* BroadPhase::getBroadPhaseCollisions() const {
        return mBPCollisions;
    }

    void BroadPhase::notifyWorldSizeChanged() {
    }

    // might get called when
    // * children are added/removed (in particular BVs)
    // * proxy moved/rotated/etc.
    void BroadPhase::notifyProxyChanged(Proxy*) {
    }

    /*!
     * \return World::getPipeline
     */
    Pipeline* BroadPhase::getPipeline() const {
        if (!getWorld()) {
            return 0;
        }
        return getWorld()->getPipeline();
    }

    /*!
     * \return A \ref PipelineThreadJobCollection for this broadphase. If no such
     * object exists yet this method will create one. Derived classes should
     * reimplement this to create/return their own \ref PipelineThreadJobCollection
     * (if they use a custom one).
     *
     * It is assumed that the returned pointer remains valid for the lifetime of
     * this \ref BroadPhase object.
     */
    PipelineThreadJobCollection* BroadPhase::getJobCollection() {
        if (!mJobCollection) {
            mJobCollection = new PipelineThreadJobCollection(getPipeline(), Pipeline::PHASE_BROADPHASE);
        }
        return mJobCollection;
    }

}

/*
 * vim: et sw=4 ts=4
 */
