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

#include "bvhierarchybvhnode.h"

#include "boundingvolumes/boundingvolume.h"
#include "boundingvolumes/aabb.h"
#include "debugstream.h"

namespace dcollide {
    BvHierarchyBvhNode::BvHierarchyBvhNode(World* world)
            : BvhNodeDefault(world) {
        mBottomUp = true;
        mIsDirty = true;
    }

    BvHierarchyBvhNode::BvHierarchyBvhNode(World* world, Shape* shape)
            : BvhNodeDefault(world, shape, false) { // AB: we cant use setIgnoreShape==false, as we need the BV for the splitter
        mBottomUp = true;
        mIsDirty = true;
    }

    BvHierarchyBvhNode::~BvHierarchyBvhNode() {
    }

    BoundingVolume* BvHierarchyBvhNode::createBoundingVolume() {
        Aabb* aabb = new Aabb();
        aabb->setHierarchyNode(this);
        return aabb;
    }

    BoundingVolume* BvHierarchyBvhNode::createBoundingVolume(const BoundingVolume* copy) {
        if (!copy) {
            throw NullPointerException("copy");
        }
        if (copy->getVolumeType() != BV_TYPE_AABB) {
            throw Exception("Given BoundingVolume is not an Aabb");
        }
        Aabb* aabb = new Aabb(*static_cast<const Aabb*>(copy));
        aabb->setHierarchyNode(this);
        return aabb;
    }

    void BvHierarchyBvhNode::recalculateBoundingVolumes() {
        if (mBottomUp) {
            BvhNodeDefault::recalculateBoundingVolumes();
            mIsDirty = false;
        } else {
            applyTranslationAccuFromParents();
            mIsDirty = true;
        }
    }

    // MUST be thrad-safe!
    void BvHierarchyBvhNode::calculateBoundingVolume() {
        if (!mIsDirty) {
            return;
        }
        MutexLocker lock(&mMutex);

        if (mBottomUp) {
            recalculateBottomUp();
            mIsDirty = false;
        } else {
            recalculateBoundingVolumesTopDown();
        }
    }

    // caller must ensure thread-safety, if running in threads!
    // caller must call applyTranslationAccu()!
    void BvHierarchyBvhNode::recalculateBoundingVolumesTopDown() {
        for (std::list<BvHierarchyBvhNode*>::const_iterator it = mChildren.begin(); it != mChildren.end(); ++it) {
            (*it)->mIsDirty = true;
        }

#ifdef __GNUC__
#warning TODO: getIsTranslationAccuEmpty()
#endif
#if 0
        if (!mIsTranslationAccuEmpty) {
            error() << dc_funcinfo << "translation accu is not empty";
        }
#endif

        // AB: this class assumes, that _all_ children belong to the _same_
        //     proxy!
        //     -> this node supports Shape hierarchies only (i.e. consisting of
        //        MeshPart objects for the same Mesh)
        //     this simplifies things a lot: just adjust to the Shape of this
        //     node, we dont have to take children into account
        //
        //     furthermore this class assumes that all parents of this node
        //     already _are_ properly updates (top-down), i.e. once this node is
        //     updated, it does NOT need to be merged back to the parents.
        clearBoundingVolume();
        if (getShape()) {
            BoundingVolume* bv = createBoundingVolume();
            bv->adjustToShape(getShape());
            mergeBoundingVolumeWith(bv);
            delete bv;
        } else {
            error() << "NULL getShape";
        }


        mIsDirty = false;
    }

    void BvHierarchyBvhNode::setIsTopDown(bool t) {
        mBottomUp = !t;
    }

    void BvHierarchyBvhNode::reinitializeChildren() {
        mChildren.clear();
        std::list<BvhNode*> intermediateNodes;
        intermediateNodes.push_back(this);
        while (!intermediateNodes.empty()) {
            BvhNode* parent = intermediateNodes.front();
            intermediateNodes.pop_front();
            for (std::list<BvhNode*>::const_iterator it = parent->getChildren().begin(); it != parent->getChildren().end(); ++it) {
                BvHierarchyBvhNode* node = dynamic_cast<BvHierarchyBvhNode*>(*it);
                if (node) {
                    mChildren.push_back(node);
                } else {
                    intermediateNodes.push_back(*it);
                }
            }
        }
    }
}

/*
 * vim: et sw=4 ts=4
 */
