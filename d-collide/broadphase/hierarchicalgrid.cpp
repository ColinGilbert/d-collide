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

#include "hierarchicalgrid.h"
#include "broadphase.h"
#include "broadphasehierarchicalgridproxydata.h"
#include "proxy.h"
#include "bvhnode.h"
#include "boundingvolumes/boundingvolume.h"
#include "debugstream.h"

#include <set>

namespace dcollide {

//FIXME: remove if createchildren is reworked to dynamic calculation
    // The number of children is 4x then the number of divisions
    const real gridDivider = GRID_CHILDREN/4;

    /*!
     *  \brief Constructor for HierarchicalGrid
     */
    HierarchicalGrid::HierarchicalGrid(const Vector3& worldDimension) {
        // No Parent for the root!!
        mParent = NULL;
        // Root Depth:
        mGridDepth = 0;

        // Calculating mMin and mMax of grid;
        Vector3 gridMax = worldDimension/2;
        Vector3 gridMin = worldDimension/2;
        gridMin.scale(-1);

        // Root Size Min:
        mMin.set(gridMin);
        // Root Size Max:
        mMax.set(gridMax);
        // Create children for the root:
        createChildren();
    }
    HierarchicalGrid::HierarchicalGrid(
            const Vector3& worldMin, const Vector3& worldMax,
            bool noChildren) {
        // No Parent for the root!!
        mParent = NULL;
        // Root Depth:
        mGridDepth = 0;

        // Root Size Min:
        mMin.set(worldMin);
        // Root Size Max:
        mMax.set(worldMax);
        // Create children for the root:
        if (!noChildren) {
            createChildren();
        }
    }
    HierarchicalGrid::HierarchicalGrid(HierarchicalGrid* parent,
        const Vector3& min, const Vector3& max) {
        mParent = parent;
        mGridDepth = (parent->getGridDepth()+1);
        mMin = min;
        mMax = max;
    }
    HierarchicalGrid::HierarchicalGrid(HierarchicalGrid* grid) {
        mParent = grid->getParent();
        mGridDepth = grid->getGridDepth();
        mMin = grid->getGridMin();
        mMax = grid->getGridMax();
        mChildren = grid->getChildren();
        mGridMembers = *(grid->getGridMembers());
    }
    /*!
     *  \brief Destructor for HierarchicalGrid
     * Deletes all mChildren
     */
    HierarchicalGrid::~HierarchicalGrid() {
        for (ListNode<const Proxy*>* memberNode = mGridMembers.getFirstNode(); memberNode; memberNode = memberNode->getNext()) {
            BroadPhaseHierarchicalGridProxyData* data =
                    static_cast<BroadPhaseHierarchicalGridProxyData*>
                    (memberNode->getData()->getBroadPhaseProxyData());
            bool foundGrid = false;
            do {
                foundGrid = false;
                for (ListNode<HierarchicalGrid*>* gridNode = data->mGrids.getFirstNode(); gridNode; gridNode = gridNode->getNext()) {
                    if (gridNode->getData() == this) {
                        data->mGrids.erase(gridNode);
                        foundGrid = true;
                        break;
                    }
                }
            } while (foundGrid);
        }

        // Delete all Children if there are any:
        for (std::vector<HierarchicalGrid*>::iterator iter =
                    mChildren.begin();
                iter != mChildren.end();
                ++iter) {
            delete *iter;
        }
    }

    /*!
     *  \brief creates the children if grid should be divided
     *   mMin is lower left front corner
     *   mMax is upper right back corner
     *  \param Grid that should be added as a child
     *  \param Position where grid should be placed
     *  \section pseudocode
     *  childrens grid layout (numbering!):
     *
     *           6------7
     *          /|     /|
     *         2------3 |
     *         | 4----|-5
     *         |/     |/
     *  y  z   0------1
     *  | /
     *  |/__x
     *
     * remember, it actually looks like this, with a grid on each node in the
     * layout above:
     *
     *                 o-----------o-----------o
     *                /|          /|          /|
     *               / |         / |         / |
     *              /  |        /  |        /  |
     *             o-----------o-----------o   |
     *            /|   |      /|   |      /|   |
     *           / |   o-----/-|---o-----/-|---o
     *          /  |  /|    /  |  /|    /  |  /|
     *         o-----------o-----------o   | / |
     *         |   |/  |   |   |/  |   |   |/  |
     *         |   o-------|- -o-------|- -o   |
     *         |  /|   |   |  /|   |   |  /|   |
     *         | / |   o---|-/-|---o---|-/-|---o
     *         |/  |  /    |/  |  /    |/  |  /
     *         o-----------o-----------o   | /
     *         |   |/      |   |/      |   |/
     *         |   o-------|---o-------|---o
     *         |  /        |  /        |  /
     *         | /         | /         | /
     *         |/          |/          |/
     *  y  z   o-----------o-----------o
     *  | /
     *  |/__x                                                M@F
     *
     *
     *  \author Maximilian Hegele <maximilian.hegele@cs.uni-dortmund.de>
     */
    void HierarchicalGrid::createChildren(
                HierarchicalGrid* child, int position) {
        // First we calculate the absolut length of each side of a grid:
        Vector3 absoluteLength = mMax - mMin;
        absoluteLength.setX(absoluteLength.getX() / GRID_DIVISOR_X);
        absoluteLength.setY(absoluteLength.getY() / GRID_DIVISOR_Y);
        absoluteLength.setZ(absoluteLength.getZ() / GRID_DIVISOR_Z);

        #ifdef DCOLLIDE_BROADPHASE_DEBUG
        std::cout << "grid min:" << mMin << "grid max:" << mMax
        << "absLength:" << absoluteLength << std::endl;
        #endif

        // create a window of the size of the new child grids
        Vector3 newMin = mMin;
        Vector3 newMax = mMin + absoluteLength;
        // run through Z dimension
        for (int i = 0; i < GRID_DIVISOR_Z; i++) {
            // reset the windows Y dimension
            newMin.setY(mMin.getY());
            newMax.setY(mMin.getY() + absoluteLength.getY());

            // avoid rounding errors that may lead to undefined space in parent
            if (i == GRID_DIVISOR_Z - 1) {
                newMax.setZ(mMax.getZ());
            }

            // run through Y dimension
            for (int j = 0; j < GRID_DIVISOR_Y; j++) {
                // reset the windows X dimension
                newMin.setX(mMin.getX());
                newMax.setX(mMin.getX() + absoluteLength.getX());

                // avoid rounding errors that may lead to undefined space
                if (j == GRID_DIVISOR_Y - 1) {
                    newMax.setY(mMax.getY());
                }

                // run through X dimension
                for (int k = 0; k < GRID_DIVISOR_X; k++) {
                    // avoid rounding errors that may lead to undefined space
                    if (k == GRID_DIVISOR_X - 1) {
                        newMax.setX(mMax.getX());
                    }

                    // if no child grid is provided create one and add it
                    if (!child) {
                        mChildren.push_back(new
                            HierarchicalGrid(this, newMin, newMax));
                    } else {
                        // if a child grid is provided check its position
                        if (position != (((i * GRID_DIVISOR_Z) + j) *
                                GRID_DIVISOR_Y) + k) {
                            mChildren.push_back(new
                                HierarchicalGrid(this, newMin, newMax));
                        } else {
                            // add the provided child grid to its position
                            mChildren.push_back(child);
                        }
                    }
                    // shift the window in X dimension
                    newMin.setX(newMin.getX() + absoluteLength.getX());
                    newMax.setX(newMax.getX() + absoluteLength.getX());
                }
                // shift the window in Y dimension
                newMin.setY(newMin.getY() + absoluteLength.getY());
                newMax.setY(newMax.getY() + absoluteLength.getY());
            }
            // shift the window in Z dimension
            newMin.setZ(newMin.getZ() + absoluteLength.getZ());
            newMax.setZ(newMax.getZ() + absoluteLength.getZ());
        }
    }


    /*!
     * \brief Adds Proxy to this grid
     * Does all the job for adding Proxies into the hierarchical grid.
     * \param Proxy
     * Proxy to be added.
     * \param maxDepth
     * Maximal depth the proxy can sink in.
     * This is often but not always the current value of mMaxGridDepth in the
     * broadphase. It may also be the new value when updating the grid.
     * \param maxMembers
     * Maximal number of members (proxies) in the grid except for
     * grids of maximal depth. The same as for maxDepth applies to the usage:
     * Often but not always mMaxGridMembers in broadphase. It may also be
     * the new value when updating the grid.
     */
    bool HierarchicalGrid::addGridMember(const Proxy* proxy, int maxDepth,
            unsigned int maxMembers) {

        bool success = false;

        // get BoundingVolume of Proxy:
        const BoundingVolume* bv =
            proxy->getBvHierarchyNode()->getBoundingVolume();

        // if this no leaf
        if (!isLeaf()) {

            // store the borders of bv
            Vector3 bvmin = bv->getSurroundingAabbMin();
            Vector3 bvmax = bv->getSurroundingAabbMax();

            // go through all children
            for (std::vector<HierarchicalGrid*>::iterator iter =
                    mChildren.begin();
                    iter != mChildren.end();
                    ++iter) {

                // temporarily store the borders of the considered child grid
                Vector3 gridmin = (*iter)->getGridMin();
                Vector3 gridmax = (*iter)->getGridMax();

                // if bv occupies the grid
                if ( !((gridmin.getX() >= bvmax.getX())
                    || (gridmax.getX() < bvmin.getX())
                    || (gridmin.getY() >= bvmax.getY())
                    || (gridmax.getY() < bvmin.getY())
                    || (gridmin.getZ() >= bvmax.getZ())
                    || (gridmax.getZ() < bvmin.getZ())
                    )) {
                        // add bv to the grid
                        success = (*iter)->addGridMember(proxy, maxDepth, maxMembers);
                }
            } // END for (std::vector<HierarchicalGrid*>::iterator
        } // END if (!isLeaf())

        // If no children, try to assign bv to this grid:
        else {
            // If number of members < maxMembers:
            // add to grid:
            if (mGridMembers.size() < maxMembers) {
                mGridMembers.push_back(proxy);

                success = true;

                // DEBUG:
                #ifdef DCOLLIDE_BROADPHASE_DEBUG
                std::cout << "Added Proxy with BV "<<
                bv->getSurroundingAabbMin() << "-" <<
                bv->getSurroundingAabbMax()
                << bv->getSurroundingAabbExtents()
                << " to grid " << mMin << "-" <<
                mMax << " in depth " << getGridDepth() <<
                std::endl;
                #endif

                // Add this grid to proxy's mGrids:
                BroadPhaseHierarchicalGridProxyData* data =
                        static_cast<BroadPhaseHierarchicalGridProxyData*>
                        (proxy->getBroadPhaseProxyData());
                if (!data) {
                    throw NullPointerException("data");
                }
                data->mGrids.push_back(this);
            }
            // if not and maxdepth not reached split this grid and retry
            else if (mGridDepth < maxDepth) {
                splitToChildren(maxDepth, maxMembers);
                success = addGridMember(proxy, maxDepth, maxMembers);
            }
            // otherwise add proxy to the grid:
            else {
                mGridMembers.push_back(proxy);

                success = true;

                // DEBUG:
                #ifdef DCOLLIDE_BROADPHASE_DEBUG
                std::cout << "Added Proxy with BV "<<
                bv->getSurroundingAabbMin() << "-" <<
                bv->getSurroundingAabbMax()
                << bv->getSurroundingAabbExtents()
                << " to grid " << mMin << "-" <<
                mMax << " in depth " << getGridDepth() <<
                std::endl;
                #endif
                // Add this grid to proxy's mGrids:
                BroadPhaseHierarchicalGridProxyData* data =
                        static_cast<BroadPhaseHierarchicalGridProxyData*>
                        (proxy->getBroadPhaseProxyData());
                if (!data) {
                    throw NullPointerException("data");
                }
                data->mGrids.push_back(this);
            }
        }

        return success;
    }

    /*!
     * \brief removes Proxy from the grid
     * removes Proxy from the grid and also removes the grids the proxy was in
     * from the proxies gridlist.
     * \param proxy The proxy which should be removed
     * \param grid Optional Parameter: If a grid is specified the proxy will be
     * removed from the given grid. Otherwise it will be removed
     * from all grids occupied by the proxy.
     */
    void HierarchicalGrid::removeProxyFromGrid(const Proxy* proxy,
            HierarchicalGrid* grid) {
        // get the grids occupied by the proxy
        BroadPhaseHierarchicalGridProxyData* data =
            static_cast<BroadPhaseHierarchicalGridProxyData*>(
            proxy->getBroadPhaseProxyData());
        if (!data) {
            // AB: actually this would be a NullPointerException.
            // however simply returning here does no harm: a NULL data object
            // means this object was never added to the BroadPhase, so we do not
            // need to remove it
            return;
        }
        List<HierarchicalGrid*>& occupiedGrids = data->mGrids;

        // If grid is given, only remove Proxy from this grid
        if (grid) {
            // First deleting proxy from the grid(s):
            List<const Proxy*>* gridMembers = grid->getGridMembers();
            gridMembers->removeAll(proxy);
            // Second deleting grid from proxies gridlist
            occupiedGrids.removeAll(grid);
        }

        // If no grid given, remove Proxy from all grids it is in:
        else {
            for (ListNode<HierarchicalGrid*>* grid_it =
                    occupiedGrids.getFirstNode();
                    grid_it != 0; grid_it = grid_it->getNext()) {

                List<const Proxy*>* gridMembers = grid_it->getData()->getGridMembers();
                gridMembers->removeAll(proxy);
            }
            // Clearing list of the grids the proxy is in:
            data->mGrids.clear();

        } // END if (grid)

    }

    /*!
     * \brief Creates children and redistributes members to them
     *
     * Attention: \n You must be sure that: \n
     * - you call this function on a leaf: isLeaf() \n
     * - splitting is neccessary: mGridMembers.size() > maxMembers \n
     * - depth bound won't be violated: maxDepth > mGridDepth \n
     * \param maxDepth
     * This is often but not always the current value of mMaxGridDepth in the
     * broadphase. It may also be the new value when updating the grid.
     * \param maxMembers
     * Maximal number of members (Proxies) in the grid except for
     * grids of maximal depth. The same as for maxDepth applies to the usage:
     * Often but not always mMaxGridMembers in broadphase. It may also be
     * the new value when updating the grid.
     */
    void HierarchicalGrid::splitToChildren(int maxDepth,
            unsigned int maxMembers) {
        #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "Splitting!" << std::endl;
        #endif
        createChildren();
        // run through the members
        for (ListNode<const Proxy*>* iter = mGridMembers.getFirstNode();
                iter != 0; iter = iter->getNext()) {
            // add the current member and do further splitting if neccessary
            addGridMember(iter->getData(), maxDepth, maxMembers);
            // remove the current member from this grid
            BroadPhaseHierarchicalGridProxyData* data =
                  static_cast<BroadPhaseHierarchicalGridProxyData*>
                  (iter->getData()->getBroadPhaseProxyData());

            // run through the grids occupied by current member
            List<HierarchicalGrid*>& occupiedGrids = data->mGrids;

            for (ListNode<HierarchicalGrid*>* grid_it =
                    occupiedGrids.getFirstNode();
                    grid_it != 0; grid_it = grid_it->getNext()) {
                // remove this grid from the list of occupied grids
                if (this == grid_it->getData())  {
                    // FIXME: This surely is slow as hell!
                  occupiedGrids.erase(grid_it);
                    break;
                }
            }
        }

        //FIXME: this shouldn't be neccessary
        mGridMembers.clear();
    }

    /*!
     * \brief Performs merges as neccessary in a subtree
     *
     * Use this function instead of mergeToParent since it does all the checking
     * neccessary for merging in a subtree. It is called for the root of the
     * subtree and returns true if a merge operation into the subroot was
     * performed, denoting that merging further up might be possible. \n
     * - Call this function for a non-leaf grid without \p callDepth to merge
     * all children up to the grid or at least as high as possible. \n
     * - Call this function for a leaf with callDepth > 0 and provide the
     * correct structure to ensure all points listed for \ref mergeToParent(),
     * if you just want to replace mergeToParent() \n
     * \param maxDepth
     * This is often but not always the current value of mMaxGridDepth in the
     * broadphase. It may also be the new value when updating the grid.
     * \param maxMembers
     * Maximal number of members (bounding volumes) in the grid except for
     * grids of maximal depth. The same as for maxDepth applies to the usage:
     * Often but not always mMaxGridMembers in broadphase. It may also be
     * the new value when updating the grid.
     * \param callDepth
     * Internal use: Depth of the recursive call stack. \n
     * Call without this parameter to use the internal structure.
     * Use callDepth > 0 if you call merge() for a leaf, but make sure you
     * provide the correct structure.
     * \param useSet
     * Internal use: Collect members of sibling grids in a set for counting. \n
     * Whenever possible provide a set to save creating a hashtable and growing
     * it to the appropriate size for your scene every time this function is
     * called.
     */
    bool HierarchicalGrid::merge(int maxDepth, unsigned int maxMembers,
            int callDepth, Set<Proxy*>* useSet) {
        // this will be the return value in callDepth 0 (could be omitted in
        // callDepth != 0, but i doubt 2 additional ifs are faster than
        // allocating a bool)
        bool merged = false;

        // go through all children and call this method again
        for (std::vector<HierarchicalGrid*>::iterator iter = mChildren.begin();
                iter != mChildren.end();
                ++iter) {
            // recursive call storing the result, only the last can be true
            merged = (*iter)->merge(maxDepth, maxMembers, callDepth + 1,useSet);
            // stop since the children were merged into this in the call above
            if (merged) {
                break;
            }
        }

        // on end of recursion return if the last call led to a merge operation
        if (callDepth == 0) {
            return merged;
        }

        // if possible due to Depth (don't merge into root) and the current
        // grid is the last child (ensures calling mergeToParent only once)
        if (isLeaf() && (mGridDepth > 1) &&
                (this == mParent->getChildren().back())) {
            // if given a set for reuse, use it, this saves creating a hashtable
            // and growing it every time
            if (useSet) {
                // get the members in all siblings (get their total number)
                useSet->clear();
                mParent->getChildrenMembers(useSet);
                // merge only if neccessary according to MaxGridDepth
                // or possible according to MaxGridMembers and number of members
                // in this grid and its siblings
                if ((mGridDepth > maxDepth) || (useSet->size() <= maxMembers)) {
                    // perform merge operation and return it was performed
                    mergeToParent();
                    return true;
                }
            // otherwise create a set which is done by getchildrenmembers() and
            // delete the set afterwards
            } else {
                // get the members in all siblings (get their total number)
                Set<Proxy*>* sibMembers = mParent->getChildrenMembers();
                // merge only if neccessary according to MaxGridDepth
                // or possible according to MaxGridMembers and number of members
                // in this grid and its siblings
                if ((mGridDepth > maxDepth) ||
                        (sibMembers->size() <= maxMembers)) {
                    // perform merge operation and return it was performed
                    mergeToParent();
                    delete sibMembers;
                    return true;
                }
                // cleanup the siblings' members
                delete sibMembers;
            }
        }
//        std::cout << useSet << std::endl;
        // return that there was no merge operation
        return false;
    }

    /*!
     * \brief Merges this leaf and its siblings into parent
     *
     * ATTENTION: \n You must be sure that: \n
     * - you call this function on a leaf: isLeaf() \n
     * - you call this function ONLY on leaves which siblings are ALSO leaves \n
     * - you DON'T call this function on a child of the root: getGridDepth() \n
     * - it is only called once (i.e. for one child) \n
     * - merging is neccessary: getGridDepth() > maxGridDepth \n
     * - or (instead of the previous) there are less members than maxMembers
     *   in this grid and its siblings maxGridMembers > getChildrenMembers() \n
     * Most of this checking is done by merge(). Calling merge() on the parent
     * provides the structure to ensure all the points above.
     */
    void HierarchicalGrid::mergeToParent() {
        #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "Merging!" << std::endl;
        #endif
        // set for collecting the members from this grid and its siblings
        std::set<const Proxy*> allMembers;

        // run through grid and its siblings
        std::vector<HierarchicalGrid*>& allSiblings = mParent->getChildren();

        for (std::vector<HierarchicalGrid*>::iterator iterSiblings =
                allSiblings.begin();
                iterSiblings != allSiblings.end();
                ++iterSiblings) {
            // run through all members of current grid
            List<const Proxy*>* siblingMembers =
                (*iterSiblings)->getGridMembers();
            for (ListNode<const Proxy*>* iterMembers =
                    siblingMembers->getFirstNode();
                    iterMembers != 0; iterMembers = iterMembers->getNext()) {

                // remove current grid in the list of occupied grids by member
                const Proxy* proxy = iterMembers->getData();
                BroadPhaseHierarchicalGridProxyData* data =
                        static_cast<BroadPhaseHierarchicalGridProxyData*>
                        (proxy->getBroadPhaseProxyData());
                if (!data) {
                    throw NullPointerException("data");
                }

#ifdef __GNUC__
#   warning FIXME: AB: I believe we need this!
#endif
                // AB: I believe we need to enable REMOVE_ALL_FROM_LIST,
                // otherwise elements remain in mGrids! (see count > 1 below)
//#define REMOVE_ALL_FROM_LIST 1
                bool abort = false;
                int count = 0;
                while (!abort) {
                    abort = true;
                    for (ListNode<HierarchicalGrid*>* iterGrids =
                            data->mGrids.getFirstNode();
                            iterGrids;
                            iterGrids = iterGrids->getNext()) {
                        if (iterGrids->getData() == (*iterSiblings)) {
                            data->mGrids.erase(iterGrids);
#ifdef REMOVE_ALL_FROM_LIST
                            abort = false;
#endif
                            count++;
                            break;
                        }
                    }
                }
                if (count > 1) {
//                    debug() << "more than once in list: " << count;
                }

#ifdef REMOVE_ALL_FROM_LIST
#undef REMOVE_ALL_FROM_LIST
#endif
                // insert member into the temporary set
                allMembers.insert(iterMembers->getData());
            }

            // clear the list of members in grid and siblings
            (*iterSiblings)->getGridMembers()->clear();
        }

        // add the members to parent
        List<const Proxy*>* parentMembers = mParent->getGridMembers();
        for (std::set<const Proxy*>::iterator iterMembers =
                allMembers.begin();
                iterMembers != allMembers.end();
                ++iterMembers) {
            parentMembers->push_back(*iterMembers);

            // Add Parent Grid to List of Grids in Proxy:
            BroadPhaseHierarchicalGridProxyData* data =
                    static_cast<BroadPhaseHierarchicalGridProxyData*>
                    ((*iterMembers)->getBroadPhaseProxyData());
            if (!data) {
                throw NullPointerException("data");
            }
            data->mGrids.push_back(mParent);
        }

        // delete grid and its siblings
        for (std::vector<HierarchicalGrid*>::iterator iterSiblings =
                allSiblings.begin();
                iterSiblings != allSiblings.end();
                ++iterSiblings) {
            delete *iterSiblings;
        }

        // clear the vector of children in the former parent
        allSiblings.clear();
    }

    /*!
     * \return A set of members inside this grid
     * including the members of children recursively
     * excluding duplicates of the members
     * OWNERSHIP: caller takes ownership of the returned set
     */
    Set<Proxy*>* HierarchicalGrid::getChildrenMembers(
            Set<Proxy*>* membersSet) const {
        if (!(membersSet)) {
            membersSet = new Set<Proxy*>(251);
        }
        if (isLeaf()) {
            for (ListNode<const Proxy*>* iter = mGridMembers.getFirstNode();
                    iter != 0; iter = iter->getNext()) {
                membersSet->insert(const_cast<Proxy*>(iter->getData()));
            }
            return membersSet;
        } else {
            for (std::vector<HierarchicalGrid*>::const_iterator iter =
                    mChildren.begin();
                    iter != mChildren.end();
                    ++iter) {
                (*iter)->getChildrenMembers(membersSet);
            }
            return membersSet;
        }
    }

    /*!
     * Determines recursively the deepest grid enclosing an aabb. The aabb
     * is given by its min and max vectors, therefore you can also use
     * coordinates derived from other sources like a union of grids. Call this
     * function on a grid as starting point for the search.
     * \param min The minimal coordinates along all axes of the aabb that has to
     * be enclosed
     * \param max The maximal coordinates along all axes of the aabb that has to
     * be enclosed
     * \param searchTopDown Set this optional parameter to TRUE and call
     * on the root if there is no proper starting point.
     * \return The deepest Grid enclosing an Aabb given by its min and max
     * vectors
     */
    const HierarchicalGrid* HierarchicalGrid::getDeepestEnclosingGrid(
            Vector3 min, Vector3 max, bool searchTopDown) const {
        // decide on the search direction: bottom-up or top-down
        if (searchTopDown == false) {
            // search bottom-up
            // if this isn't the root and the aabb is at least partially outside
            if ((mGridDepth > 0) &&
                    ((mMin.getX() > min.getX()) ||
                    (mMax.getX() < max.getX()) ||
                    (mMin.getY() > min.getY()) ||
                    (mMax.getY() < max.getY()) ||
                    (mMin.getZ() > min.getZ()) ||
                    (mMax.getZ() < max.getZ()))) {
                // check the parent grid recursively
                return mParent->getDeepestEnclosingGrid(min, max);
            } else return this;
        } else {
            // search top-down
            if (isLeaf()) {
                return this;
            } else {
                // run through the children of this grid
                for (std::vector<HierarchicalGrid*>::const_iterator
                        iter = mChildren.begin();
                        iter != mChildren.end();
                        ++iter) {
                    // if child encloses the aabb completely
                    if ((mMin.getX() < min.getX()) &&
                            (mMax.getX() > max.getX()) &&
                            (mMin.getY() < min.getY()) &&
                            (mMax.getY() > max.getY()) &&
                            (mMin.getZ() < min.getZ()) &&
                            (mMax.getZ() > max.getZ())) {
                        // continue the search from the child
                        return (*iter)->getDeepestEnclosingGrid(min, max, true);
                    }
                }
                return this;
            }
        }
    }
}
/*
 * vim: et sw=4 ts=4
 */
