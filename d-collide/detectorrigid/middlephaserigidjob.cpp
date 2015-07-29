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


#include "middlephaserigidjob.h"
#include "bvhtraverse.h"
#include "worldcollisions.h"
#include "timing.h"
#include "debug.h"
#include "world.h"
#include "proxy.h"
#include "thread/threadpool.h"
#include "dcollide-config.h"
#include "bvhnode.h"
#include "boundingvolumes/boundingvolume.h"
#include "collisionpair.h"
#include "debugstream.h"

#include <list>
#include <vector>

namespace dcollide {
    /*!
     * \param isRigid If TRUE, this job is considered to contain rigid
     * objects/pairs (see \ref addInput), otherwise deformable objects/pairs.
     * See also \ref processResults, which handles the results differently for
     * rigid and deformable jobs.
     */
    MiddlePhaseRigidJob::MiddlePhaseRigidJob(bool isRigid, unsigned int jobPoolIndex)
            : PipelineThreadJob(jobPoolIndex) {
        mIsRigid = isRigid;
    }

    MiddlePhaseRigidJob::~MiddlePhaseRigidJob() {
    }

    void MiddlePhaseRigidJob::addInput(const CollisionPair& pair) {
        if (!pair.bvol1) {
            throw NullPointerException("pair.bvol1");
        }
        if (!pair.bvol2) {
            throw NullPointerException("pair.bvol2");
        }
        if (!pair.bvol1->getHierarchyNode()) {
            throw NullPointerException("pair.bvol1->getHierarchyNode()");
        }
        if (!pair.bvol2->getHierarchyNode()) {
            throw NullPointerException("pair.bvol2->getHierarchyNode()");
        }
        Proxy* p1 = pair.bvol1->getHierarchyNode()->getProxy();
        Proxy* p2 = pair.bvol2->getHierarchyNode()->getProxy();
        if (!p1) {
            throw NullPointerException("p1");
        }
        if (!p2) {
            throw NullPointerException("p2");
        }

        mCollisionPairs.push_back(pair);
    }
    void MiddlePhaseRigidJob::addInput(Proxy* selfCollisionProxy) {
        if (!selfCollisionProxy) {
            throw NullPointerException("selfCollisionProxy");
        }
        mSelfCollisionProxies.push_back(selfCollisionProxy);
    }

    /*
     * \return A reference (!!) to the collision results. This class assumes
     * that the reference is used as a const reference at best while this job is
     * not yet finished. The reference is meant for fast copying of the list
     * using splice() once the job is completed.
     */
    std::list<BoundingVolumeCollision>& MiddlePhaseRigidJob::getCollisions() {
        return mCollisions;
    }

    // TODO: a success code for jobs?
    // -> what should happen if code != 0?
    //   * abort processing all other jobs, too?
    //   * continue processing, but set error flag?
    // probably at least two different codes, one which continues processing,
    // one which does not? (but then the return code would have meaning to the
    // WorkerPool only, not to the user)
    void MiddlePhaseRigidJob::run() {
        for (ListNode<CollisionPair>* node = mCollisionPairs.getFirstNode(); node; node = node->getNext()) {
            runCollision(node->getData());
        }
        for (ListNode<Proxy*>* node = mSelfCollisionProxies.getFirstNode(); node; node = node->getNext()) {
            runSelfCollision(node->getData());
        }
    }

    void MiddlePhaseRigidJob::runCollision(const CollisionPair& pair) {
        BvhTraverse traverse;

        std::list<BoundingVolumeCollision> collisions = traverse.getBoundingVolumeCollisions(pair);

        mCollisions.splice(mCollisions.end(), collisions);
    }

    void MiddlePhaseRigidJob::runSelfCollision(Proxy* selfCollisionProxy) {
        BvhTraverse traverse;

        std::list<BoundingVolumeCollision> selfColls = traverse.getMiddlePhaseSelfCollisions(selfCollisionProxy);

        mCollisions.splice(mCollisions.end(), selfColls);
    }

    /*!
     * Clear the job, i.e. clears all input and output variables as well as all
     * internal variables.
     *
     * This method is meant to set the job into an "unused" state, so that it
     * can be used later with different inputs.
     */
    void MiddlePhaseRigidJob::clear() {
        mCollisionPairs.clear();
        mSelfCollisionProxies.clear();
        mCollisions.clear();
    }

    void MiddlePhaseRigidJob::processResults(Pipeline* pipeline) {
        if (mIsRigid) {
            pipeline->processCompletedMiddlePhaseRigidJob(this);
        } else {
            pipeline->processAndSpliceMiddlePhaseResults(mCollisions);
        }
    }
}


/*
 * vim: et sw=4 ts=4
 */
