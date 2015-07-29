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
#include "shapes/mesh/triangle.h"

#include "narrowphasejob.h"
#include "triangleintersector.h"

#include "collisioninfo.h"
#include "narrowphase.h"

namespace dcollide {
    NarrowPhaseJob::NarrowPhaseJob(unsigned int jobPoolIndex,
                                   const NarrowPhaseShapeStrategies& strategies)
            : PipelineThreadJob(jobPoolIndex)  {
        mNarrowPhaseShapeStrategies = strategies;
    }

    NarrowPhaseJob::~NarrowPhaseJob() {
    }

    void NarrowPhaseJob::assignInput(std::list<PotentialCollidingSets>& middlePhaseCollision) {
        //TODO: Prevent the copy here
        mSetList = middlePhaseCollision;
    }

    void NarrowPhaseJob::addInput(const TrianglePair& middlePhaseCollision) {
    	mTrianglePairs.push_back(middlePhaseCollision);
    }

    void NarrowPhaseJob::addInput(const BoundingVolumeCollision& middlePhaseCollision){
        mBoundingVolumeCollisions.push_back(middlePhaseCollision);
    }

    void NarrowPhaseJob::processResults(Pipeline* pipeline) {
        pipeline->spliceNarrowPhaseResults(getNarrowPhaseCollisions());
    }

    void NarrowPhaseJob::run() {
        NarrowPhase np(mNarrowPhaseShapeStrategies);

        //Process Bounding Volumes
        for (ListNode<BoundingVolumeCollision>* node
                    = mBoundingVolumeCollisions.getFirstNode();
                    node;
                    node = node->getNext()) {
            std::list<CollisionInfo> c = np.getIntersection(node->getData());
            mNarrowPhaseCollisions.splice(mNarrowPhaseCollisions.end(), c);
        }

        //Process Triangle Pairs
        std::list<CollisionInfo> results;
        for (ListNode<TrianglePair>* node
                = mTrianglePairs.getFirstNode();
                node;
                node = node->getNext()) {



            createCollisionInfo(node->getData().triangle1,
                                node->getData().toplevelProxy1,
                                node->getData().triangle2,
                                node->getData().toplevelProxy2,
                                results);

        }
        mNarrowPhaseCollisions.splice(mNarrowPhaseCollisions.end(), results);
        
       
        //Process potential colliding sets
        for (std::list<PotentialCollidingSets>::iterator i = mSetList.begin();
             i != mSetList.end(); ++i ) {

            std::list<CollisionInfo> c;
            
            c = np.getTriangleSetIntersection((*i));

            mNarrowPhaseCollisions.splice(mNarrowPhaseCollisions.end(), c);
        }
    }

    //TODO: This was always a hack and will be removed shortly
    void NarrowPhaseJob::createCollisionInfo(const Triangle* tri1, const Proxy* proxy1,
                                             const Triangle* tri2, const Proxy* proxy2,
                                             std::list<CollisionInfo>& results) {

           // AB: FIXME: we use const Proxy pointers in the SpatialGrid, which
           // makes perfect sense, since we do not modify Proxy in any way (nor
           // ever want to modify it)
           // however we want to store a non-const Proxy pointer in CollisionInfo,
           // because of convenience for the user.
           // thus we need to use a const_cast here - usually this is somewhat
           // unclean. is it fine in this case because of the above reasons?
           // is there a "nice" way around the const_cast?
           TriangleIntersector intersector;
           TriangleIntersection intersection;
           intersector.computeTriangleIntersection(const_cast<Triangle*>(tri1), const_cast<Triangle*>(tri2), &intersection);
           if (intersection.collision) {
               if (!intersection.coplanar) {
                   //we can create 2 collisioninfos now,
                   //start and end of intersection-line
                   CollisionInfo coll1;
                   coll1.collisionPoint = intersection.intersectionStart;
                   coll1.normal         = tri2->getNormalVector();
                   coll1.penetrationDepth = intersection.penetrationDepth;
                   coll1.penetratedProxy = const_cast<Proxy*>(proxy1);
                   coll1.penetratingProxy = const_cast<Proxy*>(proxy2);
                   coll1.penetratedTriangle = const_cast<Triangle*>(tri1);
                   coll1.penetratingTriangle = const_cast<Triangle*>(tri2);
                   results.push_back(coll1);
                   //same values, just a different intersection point
                   if (!intersection.intersectionEnd.isEqual(intersection.intersectionStart)) {
                           CollisionInfo coll2;
                           coll2.collisionPoint = intersection.intersectionEnd;
                           coll2.normal = tri2->getNormalVector();
                           coll2.penetrationDepth = intersection.penetrationDepth;
                           coll2.penetratedProxy = const_cast<Proxy*>(proxy1);
                           coll2.penetratingProxy = const_cast<Proxy*>(proxy2);
                           coll2.penetratedTriangle = const_cast<Triangle*>(tri1);
                           coll2.penetratingTriangle = const_cast<Triangle*>(tri2);
                           results.push_back(coll2);
                   }
               } else {
                    //No coplanarity for now
               }
          }
       }
    /*!
     *\return A reference to the narrowphase results. This class assumes
     * that the reference is used as a const reference at best while this job is
     * not yet finished. The reference is meant for fast copying of the list
     * using splice() once the job is completed.
     */
    std::list<CollisionInfo>& NarrowPhaseJob::getNarrowPhaseCollisions(){
        return mNarrowPhaseCollisions;
    }
}
/*
 * vim: et sw=4 ts=4
 */
