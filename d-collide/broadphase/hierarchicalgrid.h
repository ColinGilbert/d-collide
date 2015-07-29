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

#ifndef DCOLLIDE_HIERARCHICALGRID_H
#define DCOLLIDE_HIERARCHICALGRID_H

// a hierarchical grid will be divided into this many parts in the corresponding
// dimension, e.g. (2,2,2) for octree, (2,2,1) for a quadtree in the x-y-plane
#define GRID_DIVISOR_X 2
#define GRID_DIVISOR_Y 2
#define GRID_DIVISOR_Z 2
// this must be = GRID_DIVISOR_X * GRID_DIVISOR_Y * GRID_DIVISOR_Z
#define GRID_CHILDREN 8

//-------------------------------------
//-------#include directives-----------
#include "math/vector.h"
#include "real.h"

#include <datatypes/list.h>
#include <datatypes/set.h>
#include <vector>

namespace dcollide {

    class Proxy;

    /*!
     * \brief class for a HierarchicalGrid
     * This class is needed to build up a HierarchicalGrid.
     * Each grid is divided into child grids if there are more than
     * mMaxGridMembers Bounding Boxes inside AND the depth is smaller than
     * mMaxGridDepth. The number of children a grid is divided into can be
     * configured for each dimension separately using the three GRID_DIVISOR
     * constants in hierarchicalgrid.h.
     * \author Maximilian Hegele <maximilian.hegele@cs.uni-dortmund.de>
     */
    class HierarchicalGrid {
        private:
            // This is an Array of 8 (3D) Childs if there are any,
            // otherwise remains undef
            std::vector<HierarchicalGrid*> mChildren;
            // This is the parent grid
            HierarchicalGrid* mParent;
            // This is a list of BoundingBoxes which are in this grid,
            // if there are NO Childs!
            // otherwise remains undef
            List<const Proxy*> mGridMembers;
            // This is the depth of this grid
            int mGridDepth;

            // This is the Position of the grid in the world
            Vector3 mMin;
            Vector3 mMax;

        public:
            HierarchicalGrid(const Vector3& worldDimension);
            HierarchicalGrid(
                const Vector3& worldMin, const Vector3& worldMax, bool
                noChildren = false);
            HierarchicalGrid(HierarchicalGrid* parent,
                const Vector3& min, const Vector3& max);
            HierarchicalGrid(HierarchicalGrid* grid);
            ~HierarchicalGrid();

            void createChildren(HierarchicalGrid* child = 0, int position = 0);

            inline bool isLeaf() const;
            inline int getGridDepth() const;
            inline const Vector3& getGridMin() const;
            inline const Vector3& getGridMax() const;
            inline Vector3 getGridSize() const;
            inline void setGridSize(const Vector3& min,
                const Vector3& max);
            Set<Proxy*>*  getChildrenMembers(
                Set<Proxy*>* membersSet = NULL) const;
            bool addGridMember(const Proxy* proxy, int maxDepth,
                unsigned int maxMembers);
            static void removeProxyFromGrid(const Proxy* proxy,
                HierarchicalGrid* grid = 0);
            void splitToChildren(int maxDepth, unsigned int maxMembers);
            bool merge(int maxDepth, unsigned int maxMembers, int callDepth = 0,
                Set<Proxy*>* useSet = 0);
            void mergeToParent();
            const HierarchicalGrid* getDeepestEnclosingGrid(Vector3 min,
                Vector3 max, bool searchTopDown = false) const;
            inline HierarchicalGrid* getParent() const;
            inline std::vector<HierarchicalGrid*>& getChildren();
            inline List<const Proxy*>* getGridMembers();

            inline const std::vector<HierarchicalGrid*>& getChildrenConst() const;
    };

    /*!
     * \return TRUE when grid is a leaf otherwise FALSE
     */
    bool HierarchicalGrid::isLeaf() const {
        return mChildren.empty();
    }

    /*!
     * \return The griddepth of this grid
     */
    int HierarchicalGrid::getGridDepth() const {
        return mGridDepth;
    }

    /*!
     * \return mMin of this grid
     */
    const Vector3& HierarchicalGrid::getGridMin() const {
        return mMin;
    }

    /*!
     * \return mMax of this grid
     */
    const Vector3& HierarchicalGrid::getGridMax() const {
        return mMax;
    }

    /*!
     *  \return the size of the grid
     */
    Vector3 HierarchicalGrid::getGridSize() const {
        return mMax-mMin;
    }

    /*!
     * \brief Set the Size of the grid
     */
    void HierarchicalGrid::setGridSize(
        const Vector3& min,const Vector3& max) {
        mMin = min;
        mMax = max;
    }

    inline HierarchicalGrid* HierarchicalGrid::getParent() const {
        return mParent;
    }

    /*!
     * \return Returns the children
     */
    inline std::vector<HierarchicalGrid*>& HierarchicalGrid::getChildren() {
        return mChildren;
    }
    inline const std::vector<HierarchicalGrid*>& HierarchicalGrid::getChildrenConst() const {
        return mChildren;
    }

    /*!
     * \return Returns the Proxies of one grid:
     */
    inline List<const Proxy*>*
        HierarchicalGrid::getGridMembers() {
        return &mGridMembers;
    }
}

#endif // DCOLLIDE_HIERARCHICALGRID_H
/*
 * vim: et sw=4 ts=4
 */
