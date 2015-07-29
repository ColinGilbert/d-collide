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


#include "broadphase/broadphasebruteforce.h"
#include "broadphase/broadphasebruteforcejob.h"
#include "broadphase/broadphasecollisions.h"
#include "thread/threadpool.h"
#include "bvhnode.h"
#include "boundingvolumes/boundingvolume.h"
#include "dcollide-config.h"
#include "world.h"
#include "debug.h"
#include "collisionpair.h"
#include "pipeline.h"
#include "proxy.h"

#include <math.h>

namespace dcollide {

    /*!
     *  \brief Creates new BroadPhaseBruteForce
     *  \param World-Class to get all Aabb's
     */
    BroadPhaseBruteForce::BroadPhaseBruteForce(World* world)
            : BroadPhase(world, BROADPHASE_TYPE_BRUTEFORCE) {
    }
    /*!
     *  \brief Destroys Broadphase
     */
    BroadPhaseBruteForce::~BroadPhaseBruteForce() {
    }


    //----------other methods-----------------------

    /*!
     * \brief Initializes the broadphase
     * This should be done by c'tor or by user (to be discussed!)
     * - Calculates AverageBoundingVolumeSize
     * - create the Grid
     */
    void BroadPhaseBruteForce::init() {
        BroadPhase::init();
    }

    void BroadPhaseBruteForce::increaseWorldSize(const Vector3& proxyPosition) {

        // Adjusting world size
        getWorld()->setWorldMinMax(getWorld()->getWorldMin(),
                getWorld()->getWorldMax(),
                &proxyPosition);
    }

    /*!
     * Create \ref BroadPhaseJob objects for broadphase collision and add them
     * to the \ref Pipeline.
     */
    void BroadPhaseBruteForce::createBroadPhaseJobs(Proxy* proxy) {
        // reset broadphasecollisions-container:
        getBroadPhaseCollisions()->reset();

        getJobCollection()->resetCollection();

        // Add Job:
        BroadPhaseBruteForceJob* job = new BroadPhaseBruteForceJob(getJobPoolIndex(), this, proxy);
        getJobCollection()->addJob(job);

        getJobCollection()->setAllJobsAreAdded();

        // Getting completed Jobs and deleting them:
        // This must de done by the pipeline!
    }

    /*!
     * \brief add Proxy to Broadphase, just a dummy!
     * Increase Worldsize if Proxy does not fit. This is done for
     * compatibility reasons and to keep konsistency (does this word exist?)!
     */
    void BroadPhaseBruteForce::addProxy(Proxy* proxy) {

        // get BoundingVolume of Proxy:
        const BoundingVolume* bv =
            proxy->getBvHierarchyNode()->getBoundingVolume();

        // store the borders of bv
        Vector3 bvmin = bv->getSurroundingAabbMin();
        Vector3 bvmax = bv->getSurroundingAabbMax();

        // Getting WorldMin and WorldMax:
        Vector3 worldMin = getWorld()->getWorldMin();
        Vector3 worldMax = getWorld()->getWorldMax();

        if ( ((worldMin.getX() >= bvmax.getX())
            || (worldMax.getX() < bvmin.getX())
            || (worldMin.getY() >= bvmax.getY())
            || (worldMax.getY() < bvmin.getY())
            || (worldMin.getZ() >= bvmax.getZ())
            || (worldMax.getZ() < bvmin.getZ()))) {
            increaseWorldSize(proxy->getBoundingVolumeCenterPosition());
        }

    }
}

/*
 * vim: et sw=4 ts=4
 */
