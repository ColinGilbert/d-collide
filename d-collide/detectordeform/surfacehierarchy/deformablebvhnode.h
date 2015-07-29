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
       notice, this list of conditions and the following disclaimer.           *
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

#ifndef DCOLLIDE_DNODE_H
#define DCOLLIDE_DNODE_H

#include "bvhnode.h"
#include "proxy.h"
#include "normalcone.h"

#include "shapes/mesh/triangle.h"

#include "boundingvolumes/boundingvolume.h"
#include "boundingvolumes/boundingsphere.h"

#include <vector>

namespace dcollide {

    class SurfaceHierarchyProxyData;

    //-----------classes------------
    /*!
     * \brief node for deformable-Objects-hierarchy-tree
    *
    * These are the nodes for the tree-structure of deformable Objects
    * these node are designed for a bottom-up hierarchic structure
    */
    class DeformableBvhNode : public BvhNode {
        private:
            NormalCone mNormalCone;

            void updateNormalCone();

            /*!
            * \brief list of all adjacent DeformableBvhNodes
            */
            //std::list<DeformableBvhNode*> mAdjacentDeformableBvhNodes;
            std::set<DeformableBvhNode*> mAdjacentDeformableBvhNodes;

            //private methods
            void createBoundingSphere(const std::list<Triangle*>& triangles,                                      const Vertex* center);
            void createBoundingSphere(const std::list<DeformableBvhNode*>& bvhNodes);

            void createBoundingSpherePrecise(const std::list<DeformableBvhNode*>& bvhNodes);

        public:
            NormalCone mCone(BoundingVolume*, Vector3&, real);


            //Constructor(s) and Destructor
            DeformableBvhNode(World* world);
            DeformableBvhNode(World* world, const std::list<Triangle*>& triangles, const Vertex* center);
            DeformableBvhNode(World* world, const std::list<DeformableBvhNode*>& bvhNodes, DeformableBvhNode* centerSphere);
            DeformableBvhNode(World* world, const std::list<DeformableBvhNode*>& bvhNodes);
            DeformableBvhNode(World* world, const std::list<DeformableBvhNode*>& bvhNodes, Proxy* proxy, SurfaceHierarchyProxyData* proxyData);
            ~DeformableBvhNode();

            void updateTopDown();

            bool isNeighbourOf(DeformableBvhNode* node) const;

            void addTriangle(Triangle* triangle);

            inline const NormalCone* getNormalCone() const;

            inline const std::set<DeformableBvhNode*>& getAdjacentDeformableBvhNodes() const;

            inline void setAdjacentBoundingSpheres(const std::list<DeformableBvhNode*>& list);

            void addChildDeformableBvhNode(DeformableBvhNode* additionalChild);

            void recalculateBoundingVolumes();
    };


    //------------ Implementation of short methods -------------

    const NormalCone* DeformableBvhNode::getNormalCone() const {
        return &mNormalCone;
    }

    const std::set<DeformableBvhNode*>& DeformableBvhNode::getAdjacentDeformableBvhNodes() const {
        return mAdjacentDeformableBvhNodes;
    }

    void DeformableBvhNode::setAdjacentBoundingSpheres(const std::list<DeformableBvhNode*>& list) {

        for (std::list<DeformableBvhNode*>::const_iterator iter = list.begin();
                                                        iter != list.end();
                                                        ++iter) {
            mAdjacentDeformableBvhNodes.insert(*iter);
        }
    }
}

#endif // DCOLLIDE_DNODE_H
/*
 * vim: et sw=4 ts=4
 */
