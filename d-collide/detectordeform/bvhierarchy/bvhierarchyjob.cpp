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

#include "bvhierarchyjob.h"

#include "detectorrigid/bvhtraverse.h"
#include "boundingvolumes/boundingvolume.h"
#include "debugstream.h"
#include "collisionpair.h"
#include "bvhnode.h"
#include "shapes/shape.h"
#include "proxy.h"
#include "bvhierarchyproxydata.h"
#include "detectordeform/potentialcollidingsets.h"
#include "shapes/mesh.h"

namespace dcollide {
    BvHierarchyJob::BvHierarchyJob()
            : DetectorDeformJob(DetectorDeformJob::RESULT_TYPE_TRIANGLESETS) {
        // AB: copied from MiddlePhaseRigidJob.
        //     TODO: merge both classes?!
        mIsRigid = false;

        mProxyDataIndex = -1;
    }

    BvHierarchyJob::~BvHierarchyJob() {
    }

    void BvHierarchyJob::setProxyDataIndex(int index) {
        mProxyDataIndex = index;
    }

    void BvHierarchyJob::addInput(const CollisionPair& pair) {
        if (mProxyDataIndex < 0) {
            throw Exception("mProxyDataIndex < 0");
        }
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

    void BvHierarchyJob::addInput(Proxy* selfCollisionProxy) {
        if (mProxyDataIndex < 0) {
            throw Exception("mProxyDataIndex < 0");
        }
        if (!selfCollisionProxy) {
            throw NullPointerException("selfCollisionProxy");
        }
        mSelfCollisionProxies.push_back(selfCollisionProxy);
    }

    void BvHierarchyJob::runRigidCollision(const CollisionPair& pair) {
        BvhTraverse traverse;
        std::list<BoundingVolumeCollision> collisions = traverse.getBoundingVolumeCollisions(pair);

        // _splices_ the list
        mRigidCollisions.splice(mRigidCollisions.end(), collisions);

    }

    void BvHierarchyJob::runRigidSelfCollision(Proxy* selfCollisionProxy) {
        BvhTraverse traverse;

        std::list<BoundingVolumeCollision> selfColls = traverse.getMiddlePhaseSelfCollisions(selfCollisionProxy);

        // _splices_ the list
        mRigidCollisions.splice(mRigidCollisions.end(), selfColls);
    }

    /*!
     * Clear the job, i.e. clears all input and output variables as well as all
     * internal variables.
     *
     * This method is meant to set the job into an "unused" state, so that it
     * can be used later with different inputs.
     */
    void BvHierarchyJob::clear() {
        mCollisionPairs.clear();
        mSelfCollisionProxies.clear();
        mRigidCollisions.clear();
    }

    void BvHierarchyJob::run() {
        for (ListNode<CollisionPair>* node = mCollisionPairs.getFirstNode(); node; node = node->getNext()) {
            runRigidCollision(node->getData());
        }
        for (ListNode<Proxy*>* node = mSelfCollisionProxies.getFirstNode(); node; node = node->getNext()) {
            runRigidSelfCollision(node->getData());
        }

        for (std::list<BoundingVolumeCollision>::const_iterator it = mRigidCollisions.begin(); it != mRigidCollisions.end(); ++it) {
            BoundingVolumeCollision c = *it;
            if (!c.node1) {
                // AB: we cannot throw an exception, since we are running in a
                // thread
                error() << "NULL node1";
                continue;
            }
            if (!c.node2) {
                error() << "NULL node2";
                continue;
            }
            if (!c.node1->getShape()) {
                error() << "NULL node1->getShape()";
                continue;
            }
            if (!c.node2->getShape()) {
                error() << "NULL node2->getShape()";
                continue;
            }
            Shape* s1 = c.node1->getShape();
            Shape* s2 = c.node2->getShape();

            Proxy* p1 = s1->getProxy();
            Proxy* p2 = s2->getProxy();
            if (!p1) {
                error() << "NULL p1";
                continue;
            }
            if (!p2) {
                error() << "NULL p2";
                continue;
            }
            DetectorDeformProxyData* d1 = p1->getDetectorDeformProxyData(mProxyDataIndex);
            DetectorDeformProxyData* d2 = p2->getDetectorDeformProxyData(mProxyDataIndex);
            if (!d1 && !d2) {
                warning() << "NULL proxydata for BOTH objects. not deformable objects?!";

                // we can't go any deeper in the hierarchy.
                addResults(c);
                continue;
            }

            BvHierarchyProxyData* data1 = static_cast<BvHierarchyProxyData*>(d1);
            BvHierarchyProxyData* data2 = static_cast<BvHierarchyProxyData*>(d2);

            // FIXME: do we need non-const nodes here?
            BvhNode* node1 = const_cast<BvhNode*>(c.node1);
            BvhNode* node2 = const_cast<BvhNode*>(c.node2);

            if (data1) {
                node1 = data1->mDeformableNode;
            }
            if (data2) {
                node2 = data2->mDeformableNode;
            }

            CollisionPair pair;
            pair.bvol1 = node1->getBoundingVolume();
            pair.bvol2 = node2->getBoundingVolume();

            if (!pair.bvol1) {
                error() << "node1: NULL getBoundingVolume()";
                addResults(c);
                continue;
            }
            if (!pair.bvol2) {
                error() << "node2: NULL getBoundingVolume()";
                addResults(c);
                continue;
            }

            // actual deformable collision detection
            // TODO: mixture of top-down and bottom-up
            BvhTraverse traverse;
            std::list<BoundingVolumeCollision> collisions = traverse.getBoundingVolumeCollisions(pair);
            
            for (std::list<BoundingVolumeCollision>::iterator collIter = collisions.begin();
            collIter != collisions.end(); ++collIter) {
                Shape* node1Shape =(*collIter).node1->getShape();
                Shape* node2Shape =(*collIter).node2->getShape();
                
                addResults(PotentialCollidingSets(node1Shape->getMesh()->getTriangles(), node1Shape->getProxy(),
                                                  node2Shape->getMesh()->getTriangles(), node2Shape->getProxy()));
                
            }
            addResults(collisions);
        }
    }

}

/*
 * vim: et sw=4 ts=4
 */
