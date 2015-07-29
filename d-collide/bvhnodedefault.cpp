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

#include "bvhnodedefault.h"

#include "dcollide-defines.h"

namespace dcollide {
    /*!
     * Create a new BvhNodeDefault object for use with objects created by \p world.
     *
     * \param world Pointer to the \ref World object that this node is being
     * used in.
     */
    BvhNodeDefault::BvhNodeDefault(World* world)
            : BvhNode(world) {
    }

    /*!
     * THIS IS NOT A USER FUNCTION
     *
     * \internal
     *
     * It will be called automatically when constructing a proxy
     *
     * \param world Pointer to the \ref World object that this node is being
     * used in.
     */
    BvhNodeDefault::BvhNodeDefault(World* world, Proxy* proxy)
            : BvhNode(world, proxy) {
    }

    /*!
     * THIS IS NOT A USER FUNCTION
     *
     * \internal
     *
     * It will be called automatically when constructing a proxy
     *
     * \param world Pointer to the \ref World object that this node is being
     * used in.
     * \param ignoreShape See \ref setIgnoreShape. FALSE by default.
     */
    BvhNodeDefault::BvhNodeDefault(World* world, Shape* shape, bool ignoreShape)
            : BvhNode(world, shape, ignoreShape) {
    }

    /*!
     * Destructor
     */
    BvhNodeDefault::~BvhNodeDefault() {
    }

    /*!
     * \brief INTERNAL USE ONLY: adds a child to this bvhnode
     *
     * Behaves like \ref BvhNode::addChild, but also
     * creates a new layer of hierarchy if new childCount > MAX_CHILDREN_COUNT
     */
    void BvhNodeDefault::addChild(BvhNode* newChild) {
        BvhNode::addChild(newChild);

        if (mChildren.size() > MAX_BVHNODE_CHILDREN_COUNT) {
            //we need to create another layer of hierarchy
            //create 2 new nodes and distribute all children onto these
            BvhNodeDefault* intermediate1 = new BvhNodeDefault(getWorld());
            intermediate1->initializeNode();
            BvhNodeDefault* intermediate2 = new BvhNodeDefault(getWorld());
            intermediate2->initializeNode();

            //for now, we just take the first half of mChildren and add
            //it to intermediate1,while the second half goes into intermediate2.
            //TODO/ENHANCEMENT This could be done more intelligent,
            //       distribution depending on position/size of the BVs
            unsigned int i=0;
            while (!getChildren().empty()) {
                BvhNode* intermediateNode = intermediate1;
                if (i > MAX_BVHNODE_CHILDREN_COUNT/2) {
                    intermediateNode = intermediate2;
                }
                BvhNode* c = getChildren().front();
                removeChild(c);
                intermediateNode->addChild(c);
                ++i;
            }//end while distributing mChildren to intermediate nodes

            addChild(intermediate1);
            addChild(intermediate2);
        }
    }

}

/*
 * vim: et sw=4 ts=4
 */
