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

#include "bvhtraverse.h"
#include "boundingvolumes/boundingvolume.h"
#include "bvhnode.h"
#include "collisionpair.h"
#include "narrowphase/narrowphase.h"
#include "proxy.h"
#include "debug.h"

#include <iostream>

namespace dcollide {
    BvhTraverse::BvhTraverse() {
    }

    BvhTraverse::~BvhTraverse() {
    }

    /*!
     *  \brief Calculates BoundingVolumeCollisions for the given \ref CollisionPair
     *
     *  This method uses a simple bounding volume traverse algorithm to check
     *  for collisions in the two objects specified by \p pair. See \ref
     *  traverse for details.
     *
     * \return See \ref traverse: A list of \ref BoundingVolumeCollision, or an
     * empty list if the two objects in \p pair are found not to collide.
     */
    std::list<BoundingVolumeCollision> BvhTraverse::getBoundingVolumeCollisions(
                    const CollisionPair& pair) {
        if (!pair.bvol1 || !pair.bvol2) {
            throw NullPointerException("one of the BoundingVolumes of pair");
        }
        if (!pair.bvol1->getHierarchyNode()) {
            throw NullPointerException("pair.bvol1->getHierarchyNode()");
        }
        if (!pair.bvol2->getHierarchyNode()) {
            throw NullPointerException("pair.bvol2->getHierarchyNode()");
        }
        Proxy* p1 = pair.bvol1->getHierarchyNode()->getProxy();
        Proxy* p2 = pair.bvol2->getHierarchyNode()->getProxy();
        if (p1 && p2) {
            if (p1 == p2) {
                return getMiddlePhaseSelfCollisions(p1);
            }
        } else {
            // AB: the nodes are non-toplevel nodes.
            //     most likely some custom BV-based approach is used (i.e. most
            //     likely deformable objects).
            //     -> in that case the BvhNodes are not required to be toplevel
            //        nodes, so so this is not an error.
            //     we don't have anything to do here, we just trust that the
            //     caller knows what she's doing.
        }

        return traverse(pair.bvol1->getHierarchyNode(),
                        pair.bvol2->getHierarchyNode());
    }

    /*!
     * Like \ref getBoundingVolumeCollisions, but this checks for self-collisions
     * in \p selfCollisionProxy.
     *
     * If \p selfCollisionProxy is not self-collidable (see \ref
     * Proxy::getProxyType), this method is a noop and returns immediately.
     * Otherwise \p traverse is used to traverse the \ref BoundingVolumes
     * hierarchy (see \ref BvhNode).
     *
     * \return See \ref traverse: A list of \ref BoundingVolumeCollision, or an
     * empty list if no self-collisions are found.
     */
    std::list<BoundingVolumeCollision> BvhTraverse::getMiddlePhaseSelfCollisions(
                    const Proxy* selfCollisionProxy) {
        if (!selfCollisionProxy) {
            throw NullPointerException("selfCollisionProxy");
        }
        if (!selfCollisionProxy->getBvHierarchyNode()) {
            throw NullPointerException("selfCollisionProxy->getBvHierarchyNode()");
        }
        if (!selfCollisionProxy->getBvHierarchyNode()->getBoundingVolume()) {
            throw NullPointerException("selfCollisionProxy->getBvHierarchyNode()->getBoundingVolume()");
        }

        if (!(selfCollisionProxy->getProxyType() & PROXYTYPE_SELFCOLLIDABLE)) {
            return std::list<BoundingVolumeCollision>();
        }

        return traverse(selfCollisionProxy->getBvHierarchyNode(),
                        selfCollisionProxy->getBvHierarchyNode());
    }

    /*!
     * \brief Simple boundingvolume-hierarchy-traverse algorithm.
     *
     * This method takes two \ref BvhNode objects and checks them for
     * collisions. If they collide, their children are checked for collision,
     * and so on (recursively) until leafs are reached. If they collide. they
     * are returnd ad \ref BoundingVolumeCollision objects.
     *
     * Notes on self-collisions:
     * \li Collisions between two different proxies which share the same
     *     toplevel \ref Proxy (see \ref Proxy::getToplevelProxy) are considered
     *     self-collisions.
     * \li Self-collisions must always come from different \ref Proxy objects
     *     (in particular: from different \ref Shape objects).
     *     Self-collisions in the same \ref Proxy are \em not detected by this
     *     algorithm. Note that two proxies that are children of the same
     *     toplevel proxy \em are different proxies.\n
     *     This is sufficient for rigid self-collisions, however for deformable
     *     objects you may need additional processing.
     *
     * Note that this algorithm is primarily optimized for rigid collision. This
     * may make no difference for "normal" deformable collision, however for
     * self-collisions you my need to do additional processing.
     *
     * \return A list of \ref BoundingVolumeCollision objects, i.e. \ref
     * BoundingVolume pairs are in leaf \ref BvhNode objects and do collide with
     * each other. This list will be empty if the two hierarchies do not collide
     * with each other.
     */
    std::list<BoundingVolumeCollision> BvhTraverse::traverse(
            const BvhNode* node1,
            const BvhNode* node2) {
        // Simple Bvh-Bvh collision algorithm we traverse both hierarchies,
        // searching for collisions in their children.
        //
        // AB: Some notes about self-collisions:
        // 1. We _can_ have rigid self collisions (think e.g. of a robot with
        //    moving arms which may collide with each other or with the body)
        // 2. Rigid self-collisions are always on 2 different (child-)proxies
        //    (more precisely: leaf-proxies).
        //    In the robot example: Proxy "arm A" may collide with Proxy "arm B"
        //    or with Proxy "body", but never with "arm A". Consequently the
        //    collision results are always pairs that belong to two different
        //    shapes.
        //    Collisions inside the same shape (and thus same leaf-proxy) are
        //    only possible for deformable objects.
        //
        // As a result there are only a few places where self-collisions are
        // relevant in this algorithm:
        // 1. We reached (with both nodes) a Proxy in the hierarchy without
        //    child-proxies.
        //    Here we can stop the recursion: no self-collisions possible
        //    anymore.
        // 2. We reached (with both nodes) a Shape.
        //    Here we can stop the recursion: no self-collisions possible
        //    anymore. (Note: there may be children left in the BVH, if the
        //    Shape is a Mesh - we may have multiple MeshParts!)
        // 3. We've reached an inner node, with node1==node2.
        //    Here we need sure that we iterate the children only once, i.e.
        //    dont collide _every_ child of node1 against _every_ child of
        //    node2, but only collide every unique pair once.
        //    -> otherwise we get duplicated results, (c1,c2) and (c2,c1) with
        //       c1 and c2 child nodes that collide with each other.
        // The 1. is also covered by 2.: if we have a Proxy without child-proxies, then
        // there can be only Shape-children
        // => we'd catch this in the next reursion call.
        std::list<BoundingVolumeCollision> result;

        if (!node1 || !node2) {
            throw NullPointerException("BvhNode* node1 or node2");
        }
        if (!node1->getBoundingVolume() || !node2->getBoundingVolume()) {
            throw NullPointerException("BoundingVolume* of node1 or node2");
        }

        // if nodes don't collide with each other, we can stop the recursion.
        // for self collisions (node1==node2) we always have to go down to the
        // leafs.
        // (note: this self-collision check is only an optimization. the
        // node1!=node2 check is not required by the algorithm and won't change
        // anything, except for the speed).
        if (node1 != node2) {
            if (!node1->getBoundingVolume()->
                        collidesWith(*node2->getBoundingVolume())) {
                return result;
            }
        }

        if (node1 == node2) {
            Proxy* p = node1->getProxy();
            if (p) {
                if (p->getChildProxies().empty()) {
                    // there are no child-proxies that may collide with each
                    // other. no self-collisions here.
                    return result;
                }
            }

            Shape* s = node1->getShape();
            if (s) {
                // there are no child-proxies that may collide with each
                // other. no self-collisions here.
                return result;
            }
        }

        if (node1->isLeaf() && node2->isLeaf()) {
            if (node1 != node2) {
                BoundingVolumeCollision coll;
                coll.node1 = node1;
                coll.node2 = node2;

                result.push_back(coll);
            } else {
                // self collision.
                // we do NOT need to check self-colliding leafs: only two
                // different proxies can collide with each other (=> two
                // different BvhNodes) for rigid self-collisions.
                //
                // AB: in fact this point should never be reached, we should
                // have catched this when node1==node2 and the node was
                // responsible for a Shape.
                // -> we have this check here anyway as a sanity check.
            }
            return result;
        } else {
            //recursive call for all children, merge returned results
            if (!node1->isLeaf() && !node2->isLeaf()) {
                //test children of each node against each other
                for (std::list<BvhNode*>::const_iterator 
                                        childIter1 = node1->getChildren().begin();
                                        childIter1 != node1->getChildren().end();
                                        ++childIter1) {
                    std::list<BvhNode*>::const_iterator childIter2 = node2->getChildren().begin();
                    if (node1 == node2) { // self-collision
                        // make sure we don't duplicate the results, since both
                        // children lists are the same list
                        childIter2 = childIter1;
                    }
                    for (; childIter2 != node2->getChildren().end(); ++childIter2) {
                        std::list<BoundingVolumeCollision> r = traverse(*childIter1, *childIter2);
                        result.splice(result.end(), r);
                    }//end for child2Iterator
                }//end for child1Iterator
            } else {
                //only one of the nodes is a leave
                if (node2->isLeaf()) {
                    //swap node 1 and node2
                    const BvhNode* tmp = node1;
                    node1=node2;
                    node2=tmp;
                }
                //test node1 against all children of node2
                for (std::list<BvhNode*>::const_iterator 
                                    childIter2 = node2->getChildren().begin();
                                    childIter2 != node2->getChildren().end();
                                    ++childIter2) {
                    std::list<BoundingVolumeCollision> r = traverse(node1, *childIter2);
                    result.splice(result.end(), r);
                }//end for child2Iterator
            }
        }//end else
        return result;
    }

}
/*
 * vim: et sw=4 ts=4
 */
