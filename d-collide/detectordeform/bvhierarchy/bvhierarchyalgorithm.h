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

#ifndef DCOLLIDE_BV_HIERARCHY_ALGORITHM_H
#define DCOLLIDE_BV_HIERARCHY_ALGORITHM_H

#include "detectordeform/detectordeformalgorithm.h"

namespace dcollide {
    class World;
    class Proxy;
    class Mesh;
    class BvHierarchyBvhNode;

    /*!
     * \brief Bounding Volume hierarchy for deformable objects
     *
     * This algorithm builds up a simple (usually \ref Aabb based) \ref
     * BoundingVolume hierarchy for deformable objects. The hierarchy in a
     * hybrid top-down \em and bottom-up approach, see the paper "Collision
     * Detection for Continuously Deforming Bodies" (Larsson, Akenine-Möller)
     * which has essentially been implemented here.
     *
     * The main idea is as follows:
     * \li Initially a \ref Aabb hierarchy is constructed. This hierarchy remain
     *     unchanged for the whole simulation, only the size of the \ref Aabbs
     *     are adjusted.
     * \li Whenever the proxy/mesh deforms, the top \ref BvHierarchyBvhNode node
     *     is adjusted to the new loo of the \ref Mesh. (i.e. top-down approach,
     *     the \ref BoundingVolume objects deeper in the hierarchy are \em not
     *     yet touched at this point)
     * \li When a collision with the top node is detected, the children of the
     *     top-node are adjusted, too (top-down again).
     * \li When a certain depth has been reached in the hierarchy, the remaining
     *     nodes are updated bottom-up.
     *
     * The idea is rather simple: if you don't have any collisions, a top-down
     * update of nodes very high in the hierarchy is very fast. If you may have
     * a collision (i.e. collision detection reached a certain point in the
     * hierarchy), then a bottom-up appraoch becomes faster.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    // TODO: enforce the use of Aabb as BV! -> currently we depend on which BV
    // has been used as rigid BV
    class BvHierarchyAlgorithm : public DetectorDeformAlgorithm {
        public:
            BvHierarchyAlgorithm(World* world, Pipeline* pipeline);
            ~BvHierarchyAlgorithm();

            virtual inline bool supportsSelfCollisions() const;
            virtual inline bool supportsPairCollisions() const;
            virtual inline bool getListenForProxyChanges() const;

            virtual void prepareSimulation();

            virtual void addTopLevelProxy(Proxy* proxy);
            virtual void removeTopLevelProxy(Proxy* proxy);

            virtual void createCollisionJobFor(const CollisionPair& pair);
            virtual void notifyAllCollisionPairsAreAdded();

            virtual DetectorDeformProxyData* createProxyData(Proxy* proxy);

            virtual void translateProxy(Proxy* proxy, const Vector3& translateBy);
            virtual void rotateProxy(Proxy* proxy, const Matrix& rotation);
            virtual void setProxyMatrix(Proxy* proxy, const Matrix& newMatrix);
            virtual void deformProxy(Proxy* proxy, const std::vector<Vector3>& vertexMoveArray);
            virtual void deformProxy(Proxy* proxy, unsigned int vertexIndex, const Vector3& vertexPosition);

        protected:
            void initMesh(Proxy* parent, Mesh* mesh);

            void setTopDown(BvHierarchyBvhNode* node, int depth, int maxDepth);

        private:
            World* mWorld;
    };

    /*!
     * \return FALSE, i.e. this algorithm does not support self-collisions.
     */
    inline bool BvHierarchyAlgorithm::supportsSelfCollisions() const {
        return false;
    }

    /*!
     * \return TRUE, i.e. this algorithm supports non-self-collisions
     */
    inline bool BvHierarchyAlgorithm::supportsPairCollisions() const {
        return true;
    }

    /*!
     * \return TRUE, i.e. this algorithm wants to be informed about matrix
     *         changes and deformations of deformable proxies.
     */
    inline bool BvHierarchyAlgorithm::getListenForProxyChanges() const {
        return true;
    }
}

/*
 * vim: et sw=4 ts=4
 */
#endif
