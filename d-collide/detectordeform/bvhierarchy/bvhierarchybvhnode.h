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

#ifndef DCOLLIDE_BV_HIERARCHY_BVHNODE_H
#define DCOLLIDE_BV_HIERARCHY_BVHNODE_H

#include "bvhnodedefault.h"

namespace dcollide {

    class BvHierarchyBvhNode : public BvhNodeDefault {
        public:
            BvHierarchyBvhNode(World* world);
            BvHierarchyBvhNode(World* world, Shape* shape);
            ~BvHierarchyBvhNode();

            virtual BoundingVolume* createBoundingVolume();
            virtual BoundingVolume* createBoundingVolume(const BoundingVolume* copy);

            void reinitializeChildren();

            virtual void recalculateBoundingVolumes();

            void setIsTopDown(bool t);
            inline const std::list<BvHierarchyBvhNode*>& getBvHerarchyBvhNodeChildren() const;


        protected:
            virtual void calculateBoundingVolume();

            void recalculateBoundingVolumesTopDown();

        private:
            bool mBottomUp;
            bool mIsDirty;

            // AB: BvhNode::getChildren() is not sufficient for us, because
            // BvhNodeDefault may create intermediate nodes, which are NOT of
            // type BvHierarchyBvhNode!
            std::list<BvHierarchyBvhNode*> mChildren;

            mutable Mutex mMutex;
    };

    /*!
     * \return All \ref BvHierarchyBvhNode children. Like \ref getChildren, but
     * \ref getChildren may also contain intermediate nodes which are of a
     * different type.
     */
    const std::list<BvHierarchyBvhNode*>& BvHierarchyBvhNode::getBvHerarchyBvhNodeChildren() const {
        return mChildren;
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
