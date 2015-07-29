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


#include "broadphase/broadphasehierarchicalgrid.h"
#include "broadphase/broadphasehierarchicalgridjob.h"
#include "broadphase/broadphasehierarchicalgridjobcollection.h"
#include "broadphase/broadphasecollisions.h"
#include "broadphase/broadphasehierarchicalgridproxydata.h"
#include "broadphase/hierarchicalgrid.h"
#include "thread/threadpool.h"
#include "bvhnode.h"
#include "boundingvolumes/boundingvolume.h"
#include "dcollide-config.h"
#include "world.h"
#include "debug.h"
#include "collisionpair.h"
#include "pipeline.h"
#include "proxy.h"
#include "debugstream.h"

#include <math.h>
#include <algorithm>

namespace dcollide {
    //implementation of the interface

    //---------constructor and destructor----------

    /*!
     *  \brief Creates new BroadPhaseHierarchicalGrid
     *  \param World-Class to get all Aabb's
     */
    BroadPhaseHierarchicalGrid::BroadPhaseHierarchicalGrid(World* world)
            : BroadPhase(world, BROADPHASE_TYPE_HIERARCHICALGRID) {
        // Initialise Grid:
        mGrid = new HierarchicalGrid(world->getWorldMin(),world->getWorldMax());

        // set default mUpperProbabilityBound
        mUpperProbabilityBound = DEFAULT_PROBABILITY;

        // set default mMaxGridMembers
        mMaxGridMembers = calculateMaxGridMembers();

        // Set counter:
        mGetCollisionsCalls = 0;
        mUpdateIntervall = 30;

        mJobCollection = new
            BroadPhaseHierarchicalGridJobCollection(getWorld()->getPipeline());
        mJobCollection->setPool(getWorld()->getWorkerPool()); // FIXME: obsolete? -> only Pipeline should get used!
        mJobCollection->setBroadPhase(this);
        // other members will be initialized by init(), which should be called
        // by user when all proxies are added to the world
    }
    /*!
     *  \brief Destroys Broadphase
     */
    BroadPhaseHierarchicalGrid::~BroadPhaseHierarchicalGrid() {
        delete mJobCollection;
        delete mGrid;
    }


    //----------other methods-----------------------

    /*!
     * \brief Initializes the broadphase
     * This should be done by c'tor or by user (to be discussed!)
     * - Calculates AverageBoundingVolumeSize
     * - create the Grid
     */
    void BroadPhaseHierarchicalGrid::init() {
        BroadPhase::init();

        #ifdef DCOLLIDE_BROADPHASE_DEBUG
        int cnt = 0;
        #endif

        // reseting average bounding volume size
        mAverageBoundingVolumeSize.reset();

        // Get Pointer to allTopLevelProxies
        const std::list<Proxy*>& allTopLevelProxies =
            getWorld()->getTopLevelProxies();

        // Calculate average size of bounding volumes
        for (std::list<Proxy*>::const_iterator iter =
                    allTopLevelProxies.begin();
                iter != allTopLevelProxies.end();
                ++iter) {
        // Now fetch bounding volume of this proxy:
        const BoundingVolume* bv =
            (*iter)->getBvHierarchyNode()->getBoundingVolume();
        // updating average bounding volume size
        // only if BoundingVolume really exists:
        if (bv) {
            mAverageBoundingVolumeSize += bv->getSurroundingAabbExtents() /
                (real)(getWorld()->getNumberOfTopLevelProxies());
        }

        // Debug output:
        #ifdef DCOLLIDE_BROADPHASE_DEBUG
        cnt++;
        // only in iteration for debug:
        mMaxGridDepth = calculateMaxGridDepth(mUpperProbabilityBound);
        std::cout << cnt << ". Loop: MaxGridDepth: " <<
            mMaxGridDepth << "\t avgbvsize: " <<
            mAverageBoundingVolumeSize << std::endl;
        #endif

        }

        // Now we create the grid:
        createGrid();
    }

    /*!
     * \brief Calculates the initial grid tree
     * Calls addBoundingVolumeToGrid for each bv
     */
    void BroadPhaseHierarchicalGrid::createGrid() {

        // determine the maximal depth
        mMaxGridDepth = calculateMaxGridDepth(mUpperProbabilityBound);

        // determine the maximal number of proxies in a non-maximal-depth grid
        mMaxGridMembers = calculateMaxGridMembers();

        // Get Pointer to allTopLevelProxies
        const std::list<Proxy*>& allTopLevelProxies =
            getWorld()->getTopLevelProxies();

        bool success = true;

        // Go through all top-level proxies and add to grid:
        for (std::list<Proxy*>::const_iterator iter =
                    allTopLevelProxies.begin();
                iter != allTopLevelProxies.end();
                ++iter) {

            // Now we add the bv to the Grid:
            // addBoundingVolumeToGrid(bv);
            success = addProxyToGrid(*iter);
            // in the unlikely case that the newly created grid is too small
            if (!success) {
                // Increase the world, this implies recreating grid, too:
                increaseWorldSize((*iter)->getBoundingVolumeCenterPosition());
                // don't do anything to the even newer grid
                return;
            }
        }
    } // END void BroadPhaseHierarchicalGrid::createGrid() {


    /*!
     * \brief Calculates the maximal grid depth heuristically
     *
     * Calculates the maximal depth of the grid tree according to
     * the heuristic described in pseudocode
     * \section pseudocode
     * Given:                                                                 \n
     * - upper probability bound for shapes being bigger than smallest grid
     * - average size of the bounding volumes in all 3 dimensions
     *   (calculated in init(), updated in add/remove bounding volume)
     * - size of the world in all 3 dimensions
     *
     * Returns:                                                               \n
     * - depth of the hierarchical grid with grids large enough to provide the
     *   probability bound
     *
     * Based on:                                                              \n
     * - Markov inequality: Prob(s >= g) <= E(s) / g
     *    with:
     *          - s     size of bounding volume
     *          - E(s)  expectation / average size of bounding volumes
     *          - g     size of grid
     *                  (i.e. g = E(s) / probability bound as described above)
     *
     *  - Calculate lower bound of the size of smallest grid
     *  - Calculate grid size / world size ratio of all dimensions
     *  - Determine dimension with maximal grid size / world size ratio
     *  - Determine depth of hierarchical grid leading to smallest grid size in
     *    this dimension
     *  - Check if appropriate for the smallest world size dimension
     * \param
     * Bound for the probability of a shape to be bigger than a smallest
     * Grid. Recommended value: 0.5 (default)
     * \return for use as mMaxGridDepth
     * Depth of a hierarchical grid leading to a smallest grid size of
     * next higher power of 2 to
     * average size of bounding volumes / given probability bound.
     * This leads to a small number of bounding volumes bigger than the
     * grid. Those are leading to useless further separation.
     */
    int BroadPhaseHierarchicalGrid::calculateMaxGridDepth(
            real upperProbabilityBound) {
        // This will be the return value
        int depth = 0;

        // Calculate lower bound of the size of smallest grid
        Vector3 smallestGridSize = mAverageBoundingVolumeSize
                                   / upperProbabilityBound;

        // Calculate grid size / world size ratio of all dimensions
        Vector3 worldDimension = getWorld()->getWorldDimension();
        real xRatio = smallestGridSize.getX() / worldDimension.getX();
        real yRatio = smallestGridSize.getY() / worldDimension.getY();
        real zRatio = smallestGridSize.getZ() / worldDimension.getZ();

        // Determine dimension with maximal grid size / world size ratio
        // since this is the most restrictive
        if (xRatio > yRatio) {
            if (xRatio > zRatio) {
                // Maximum is X
                // Determine depth of hierarchical grid leading to smallest grid
                // size in this dimension
                depth = calculateDepthFromSize((int) worldDimension.getX(),
                                               (int) smallestGridSize.getX(),
                                               GRID_DIVISOR_X);
            } else {
                // Maximum is Z
                // Determine depth of hierarchical grid leading to smallest grid
                // size in this dimension
                depth = calculateDepthFromSize((int) worldDimension.getZ(),
                                               (int) smallestGridSize.getZ(),
                                               GRID_DIVISOR_Z);
            }
        } else {
            if (yRatio > zRatio) {
                // Maximum is Y
                // Determine depth of hierarchical grid leading to smallest grid
                // size in this dimension
                depth = calculateDepthFromSize((int) worldDimension.getY(),
                                               (int) smallestGridSize.getY(),
                                               GRID_DIVISOR_Y);
            } else {
                // Maximum is Z
                // Determine depth of hierarchical grid leading to smallest grid
                // size in this dimension
                depth = calculateDepthFromSize((int) worldDimension.getZ(),
                                               (int) smallestGridSize.getZ(),
                                               GRID_DIVISOR_Z);
            }
        }

        // Check if appropriate for the smallest world size dimension,
        // i.e. don't allow grids with size < 1 in any dimension
        int maxDepth = calculateDepthFromSize((int) std::min(std::min(
                                                        worldDimension.getX(),
                                                        worldDimension.getY()),
                                                        worldDimension.getZ()),
                                              1, (int) std::max(std::max(
                                                      GRID_DIVISOR_X,
                                                      GRID_DIVISOR_Y),
                                                      GRID_DIVISOR_Z));

        depth = std::min(depth, maxDepth);

        // check for appropriate values, keep in mind the exponential number of
        // grids with a base of 8, so a maximal maxdepth of 8 is a lot
        depth = std::max(1, std::min(depth, 8));

        return depth;
    }

    /*!
     * \brief Calculates the depth from 2 given sizes
     *
     * Calculates the depth of a tree dividing the world size into the
     * next higher power of divisor to the smallest grid size. Used for
     * calculations considering a single dimension of a hierarchical grid.
     * This function is private and advised to be inlined by the compiler. It is
     * used in 5 different cases in calculateMaxGridDepth.
     * \param worldDimension Size to be partitioned in the considered dimension
     * \param smallestGridSize Lower bound on the size of the smallest grid used
     * for partitioning
     * \param divisor Number of partitions per level of depth
     * \return Depth of the tree used for space partitioning
     * (i.e. round_down(log_divisor(worldSize / smallestGridSize)))
     */
    int BroadPhaseHierarchicalGrid::calculateDepthFromSize(int worldDimension,
            int smallestGridSize, int divisor) const {
        if (worldDimension < 0) {
            // TODO: exception
            std::cerr << dc_funcinfo << "ERROR: invalid worldDimension: " << worldDimension << std::endl;
            return 1;
        }
        if (smallestGridSize < 0) {
            // TODO: exception
            std::cerr << dc_funcinfo << "ERROR: invalid smallestGridSize: " << smallestGridSize << std::endl;
            return 1;
        }
        int depth = -1;
        while (worldDimension > smallestGridSize) {
            depth++;
            worldDimension /= divisor;
        }
        return depth;
    }

    /*!
     * \brief Calculates the maximal number of members in non maximal depth
     * grids.
     *
     * The heuristically determined value is the expected number of members
     * in smallest grids bounded by 4 and the square root of the total number of
     * members.
     * \return for use as mMaxGridMembers
     * Expected number of members in smallest possible grids bounded by 4 and
     * the square root of the total number of members.
     * This leads to at most square root of total number of members in a
     * single non-leaf grid and thus to O(n) collision pairs. But it prevents
     * separations due to numbers of members that should be expected in a grid
     * that wouldn't be separated any further.
     * Keep in mind that this bound may be violated for grids of maximal depth.
     */
    int BroadPhaseHierarchicalGrid::calculateMaxGridMembers() {
        // get all values that are needed later: the size of the world
        Vector3 worldDimension = getWorld()->getWorldDimension();
        // the total number of top-level-proxies, a.k.a. members
        real numberOfProxies = (real)getWorld()->getNumberOfTopLevelProxies();

        // create a vector denoting the number of times a grid in the preferred
        // depth is divided in each dimension
        Vector3 gridDivisor;
        gridDivisor.setX(::pow((real)GRID_DIVISOR_X, PREFERRED_DEPTH));
        gridDivisor.setY(::pow((real)GRID_DIVISOR_Y, PREFERRED_DEPTH));
        gridDivisor.setZ(::pow((real)GRID_DIVISOR_Z, PREFERRED_DEPTH));
        // determine the size of a grid in the preferred depth
        Vector3 gridSize;
        gridSize.setX(worldDimension.getX() / gridDivisor.getX());
        gridSize.setY(worldDimension.getY() / gridDivisor.getY());
        gridSize.setZ(worldDimension.getZ() / gridDivisor.getZ());

        // calculate the probability for an average member to lie in the
        // smallest grid. this is componentwise the likelihood for lying in the
        // extents of the grid in the dimension corresponding to the component
        Vector3 probability1 = (gridSize + mAverageBoundingVolumeSize);
        Vector3 probability2 = (worldDimension + mAverageBoundingVolumeSize);
        probability1.setX(probability1.getX() / probability2.getX());
        probability1.setY(probability1.getY() / probability2.getY());
        probability1.setZ(probability1.getZ() / probability2.getZ());

        // multiply all probabilities to get the probability to lie inside the
        // extents of all dimensions at the same time. i.e. inside the grid
        real probability = probability1.getX()
                            * probability1.getY()
                            * probability1.getZ();

        // Calculate the expected value
        real expectedMembers = probability * numberOfProxies;

        // lower bound 4
        int maxMembers = std::max(4,
            // upper bound square root of number of proxies
            std::min((int)expectedMembers,(int)::sqrt((real)numberOfProxies)));

        #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "newMaxGridMembers: " << maxMembers << std::endl
                << "expected Value: " << expectedMembers << std::endl
                << "square root: " << ::sqrt(numberOfProxies) << std::endl;
        #endif

        return maxMembers;
    }

    void BroadPhaseHierarchicalGrid::notifyWorldSizeChanged() {
        recreateGrid();
    }

    void BroadPhaseHierarchicalGrid::notifyProxyChanged(Proxy* toplevelProxy) {
        updateProxyAllocation(toplevelProxy);
    }

    /* \brief This should be called everytime grids needs to be rebuild
     * some scenarios: worldsize changes
     * ...
     */
    void BroadPhaseHierarchicalGrid::recreateGrid() {
        delete mGrid;
        mGrid = new HierarchicalGrid(getWorld()->getWorldMin(),
            getWorld()->getWorldMax());
        createGrid();
    }

    /*!
     * \brief Updates the broadphase
     */
    void BroadPhaseHierarchicalGrid::update() {
        #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "Update called!" << std::endl;
        #endif

        // Calculate a new MaxGridDepth but apply it after adjusting the grid
        int newMaxGridDepth = calculateMaxGridDepth(mUpperProbabilityBound);

        // Calculate a new MaxGridMembers bound and apply it as described above
        unsigned int newMaxGridMembers = calculateMaxGridMembers();

        // Adjust the grid according to newMaxGridDepth, newMaxGridMembers
        // and the present distribution of shapes
        // maybe the latter will lose importance depending on how good
        // moving, adding, removing of shapes keeps the grid up to date
        traverseGridForUpdate(mGrid, newMaxGridDepth, newMaxGridMembers);

        // Apply changes of newMaxGridDepth to mMaxGridDepth
        mMaxGridDepth = newMaxGridDepth;

        // Apply changes of MaxGridMembers
        mMaxGridMembers = newMaxGridMembers;
    }

    /*!
     * \brief Adds a BroadPhaseJob which performs an update
     * adds only a new job if
     */
    void BroadPhaseHierarchicalGrid::addUpdateJob() {

        // Check if we need to trigger update:
        if ((mGetCollisionsCalls == mUpdateIntervall)
                && (mUpdateIntervall != 0))  {
            BroadPhaseHierarchicalGridJob* job = new
                BroadPhaseHierarchicalGridJob(getJobPoolIndex(), this);
            getWorld()->getPipeline()->addJobForPhase(Pipeline::PHASE_BROADPHASE, job);
            mGetCollisionsCalls = 0;
        }
    }

    /*!
     * \brief Traverses the grid, splits crowded and merges vacant grids
     *
     * Runs through the grid recursively and splits grids or merges them
     * bottom up depending on changes of MaxGridDepth, MaxGridMembers and the
     * current number of members
     * \param grid the current grid
     * \param newMaxGridDepth new bound for the depth of the grid to be enforced
     * \param newMaxGridMembers new number of members of the grid to be enforced
     */
    void BroadPhaseHierarchicalGrid::traverseGridForUpdate(
            HierarchicalGrid* grid, int newMaxGridDepth,
            unsigned int newMaxGridMembers) {
        // traverse to the leafs
        if (!grid->isLeaf()) {
            std::vector<HierarchicalGrid*> allChildren = grid->getChildren();
            // go through all children and call this method again
            for (std::vector<HierarchicalGrid*>::iterator it =
                    allChildren.begin();
                    it != allChildren.end();
                    ++it) {
                #ifdef DCOLLIDE_BROADPHASE_DEBUG
                if (grid->getParent()) {
                    std::cout << "going down from "
                    << grid->getGridMin() << grid->getGridMax() << " to "
                    << (*it)->getGridMin() << (*it)->getGridMax()
                    << " Depth: " << (*it)->getGridDepth() << std::endl;
                }
                #endif
                // recursive call
                traverseGridForUpdate(*it, newMaxGridDepth, newMaxGridMembers);
             }
        }

        // when reaching a leaf on the way down in the recursion split it up
        // if neccessary according to MaxGridMembers and possible
        // according to MaxGridDepth
        else if ((grid->getGridDepth() < newMaxGridDepth) &&
                  (grid->getGridMembers()->size() > newMaxGridMembers)) {
            grid->splitToChildren(newMaxGridDepth, newMaxGridMembers);
        }

        #ifdef DCOLLIDE_BROADPHASE_DEBUG
            if (grid->getParent()) {
              std::cout << "going up from "
                << grid->getGridMin() << grid->getGridMax() << " to "
                << grid->getParent()->getGridMin() <<
                grid->getParent()->getGridMax()
                << " Depth: " << grid->getParent()->getGridDepth() << std::endl;
            }
        #endif

        // when the recursive call ends the control goes bottom up through the
        // recursion. On this way merge the leaves
        if (grid->isLeaf() &&
                (grid == grid->getParent()->getChildren().back())) {
            grid->merge(newMaxGridDepth, newMaxGridMembers, 1, &mProxySet);
        }
    }

    /*!
     * \brief increases Size of the world
     * this method is called if an added proxy does not fit into the grid
     * increases the world until proxy does fit
     * WARNING: recreates Grid!
     * \param Position of Proxy which does not fit
     */
    void BroadPhaseHierarchicalGrid::increaseWorldSize(
            const Vector3& proxyPosition) {
        // Adjusting world size, ATTENTION: This recreates the grid!
        getWorld()->setWorldMinMax(getWorld()->getWorldMin(),
                getWorld()->getWorldMax(),
                &proxyPosition);
    }

    //FIXME: this function may be obsolete, in fact it surely won't work with
    // a GRID_DIVISOR != 2 probably the childPosition calculation is incorrect
    /*!
     * \brief increases Size of the grid
     * this method is called if an added proxy does not fit into the grid
     * does only increase by one
     * \param Position of Proxy which does not fit
     */
    void BroadPhaseHierarchicalGrid::increaseGrid(const Vector3& proxyPosition)
    {

        // get necessary values:
        Vector3 oldGridMin = mGrid->getGridMin();
        Vector3 oldGridMax = mGrid->getGridMax();
        HierarchicalGrid* oldRootGrid = mGrid;

        // get Length of new vector beetween gridMin/max and proxyPosition:
        real minLength = (oldGridMin-proxyPosition).lengthApproxBabylonian();
        real maxLength = (oldGridMax-proxyPosition).lengthApproxBabylonian();

        // preprocessing for calc. of new gridMin/gridMax
        Vector3 newGridMin;
        Vector3 newGridMax;
        Vector3 stepper = oldGridMax-oldGridMin;

        // If length of max to pos greater, put child to the end of the
        // list of children, else to the beginning
        int childPosition;
        if (maxLength > minLength) {
            childPosition = 7;
            // calc. of new gridMin:
            newGridMin = oldGridMin-(stepper*2);
            // gridMax remains the same!
            newGridMax = oldGridMax;
        } else {
            childPosition = 0;
            // calc. of new gridMax:
            newGridMax = oldGridMax+(stepper*2);
            // gridMin remains the same!
            newGridMin = oldGridMin;
        }

        // create new root grid:
        mGrid = new HierarchicalGrid(newGridMin,newGridMax,true);

        // create children of new grid:
        mGrid->createChildren(oldRootGrid,childPosition);
    }

//FIXME: remove this function if the clever version proved to be stable
    /*!
     * \brief update Proxy Allocation
     */
    void BroadPhaseHierarchicalGrid::updateProxyAllocation2(Proxy* proxy) {
        removeProxy(proxy);
        addProxy(proxy);
    }

    /*!
     * \brief update Proxy Allocation V2
     */
    void BroadPhaseHierarchicalGrid::updateProxyAllocation(Proxy* proxy) {
        // make sure proxy is a toplevelproxy
        proxy = proxy->getToplevelProxy();

        // get the previously occupied grids
        BroadPhaseHierarchicalGridProxyData* data =
            static_cast<BroadPhaseHierarchicalGridProxyData*>
            (proxy->getBroadPhaseProxyData());
        if (!data) {
            std::cerr << "Warning in: " << dc_funcinfo <<
                "illegal state: proxy was not added to grid" << std::endl;
            addProxy(proxy);
            return;
        }
        const List<HierarchicalGrid*>& oldGrids = data->getOccupiedGrids();

        // if there were no occupied grids issue a warning and fix the problem
        if (oldGrids.empty()) {
            std::cerr << "Warning in: " << dc_funcinfo <<
            "illegal state: proxy "<< proxy << " has no occupied grids" << std::endl;
            addProxy(proxy);
            return;
        }
/* FIXME: the following calculates mergestart which is used for online updates
        // determine a theoretical aabb's min and max vectors enclosing oldgrids
        // therefore pick the first of the old grids as iterator
        std::list<HierarchicalGrid*>::const_iterator iter = oldGrids.begin();
        // take its vectors
        Vector3 min = (*iter)->getGridMin();
        Vector3 max = (*iter)->getGridMax();
        ++iter;
        // go through all oldgrids
        for (; iter != oldGrids.end(); ++iter) {
            // grow the aabb's min and max vectors in X,Y and Z if neccessary
            if (min.getX() > (*iter)->getGridMin().getX()) {
                min.setX((*iter)->getGridMin().getX());
            }
            if (min.getY() > (*iter)->getGridMin().getY()) {
                min.setY((*iter)->getGridMin().getY());
            }
            if (min.getZ() > (*iter)->getGridMin().getZ()) {
                min.setZ((*iter)->getGridMin().getZ());
            }
            if (max.getX() < (*iter)->getGridMax().getX()) {
                max.setX((*iter)->getGridMax().getX());
            }
            if (max.getY() < (*iter)->getGridMax().getY()) {
                max.setY((*iter)->getGridMax().getY());
            }
            if (max.getZ() < (*iter)->getGridMax().getZ()) {
                max.setZ((*iter)->getGridMax().getZ());
            }
        }
        // set a grid enclosing all oldgrids as starting point for merging
        HierarchicalGrid* mergeStart = const_cast<HierarchicalGrid*>(
            oldGrids.front()->getDeepestEnclosingGrid(min, max));
        if (mergeStart->isLeaf()) mergeStart = mergeStart->getParent();
//*/
        // get the updated aabb of the proxy
        Vector3 min = proxy->getBvHierarchyNode()->getBoundingVolume()->
            getSurroundingAabbMin();
        Vector3 max = proxy->getBvHierarchyNode()->getBoundingVolume()->
            getSurroundingAabbMax();
        // set a grid as starting point for adding
// FIXME: improve by using a still occupied grid instead of oldgrids->front()
        HierarchicalGrid* addStart = const_cast<HierarchicalGrid*>(
                oldGrids.front()->getDeepestEnclosingGrid(min, max));

        // remove proxy from all grids
        HierarchicalGrid::removeProxyFromGrid(proxy);

        // add proxy using assumptions about spatial coherence
        addProxyForUpdate(proxy, addStart);

/* FIXME: deactivated since online updates proved to be too slow
        // start merging from the calculated starting point
        while(mergeStart->merge(mMaxGridDepth, mMaxGridMembers, 0,&mProxySet)) {
            // stop merging if the root was reached
            if (mergeStart->getGridDepth() < 1) break;
            // continue to merge if there was a merging operation into the start
            mergeStart = mergeStart->getParent();
        }
//*/
    }

// FIXME: this may be obsolete
    /*!
     * At the moment this function does the same as removeProxy() but without
     * updating the average bounding volume size.
     */
    void BroadPhaseHierarchicalGrid::removeProxyForUpdate(Proxy* proxy) {
        BroadPhaseHierarchicalGridProxyData* data =
            static_cast<BroadPhaseHierarchicalGridProxyData*>
            (proxy->getBroadPhaseProxyData());
        if (!data) {
            // AB: actually this would be a NullPointerException.
            // however simply returning here does no harm: a NULL data object
            // means this object was never added to the BroadPhase, so we do not
            // need to remove it
            return;
        }
        const List<HierarchicalGrid*>& allGrids = data->getOccupiedGrids();

        // go through all Grids this proxy belongs to:
        for (ListNode<HierarchicalGrid*>* node = allGrids.getFirstNode();
                node; node = node->getNext()) {
            // Now remove proxy from grid:
            List<const Proxy*>* allMembers = node->getData()->getGridMembers();

            for (ListNode<const Proxy*>* proxy_it =
                    allMembers->getFirstNode();
                    proxy_it != 0; proxy_it = proxy_it->getNext()) {
                if (proxy == proxy_it->getData())  {
                // FIXME: This surely is slow as hell!
                    allMembers->erase(proxy_it);
                    break;
                }
            }
        }

        // Now delete List of Grids in Proxy:
        data->mGrids.clear();
    }

    /*!
     * At the moment this function does almost the same as addProxyToGrid() but
     * it additionally checks for proxies leaving the world.
     */
    void BroadPhaseHierarchicalGrid::addProxyForUpdate(Proxy* proxy,
            HierarchicalGrid* start) {
        bool success = false;
        success = start->addGridMember(proxy, mMaxGridDepth, mMaxGridMembers);
        if (!success) {
            // Increase the world, this implies recreating grid, too:
            increaseWorldSize(proxy->getBoundingVolumeCenterPosition());
        }
    }

    /*!
     * \brief add Proxy to grid
     */
    void BroadPhaseHierarchicalGrid::addProxy(Proxy* proxy) {
        //std::cout << "Trying to add Proxy " << proxy << "... " << std::endl;
        if (!proxy->getBroadPhaseProxyData()) {
            proxy->setBroadPhaseProxyData(new
                BroadPhaseHierarchicalGridProxyData(proxy));
        }
        int numberOfProxies = getWorld()->getNumberOfTopLevelProxies();

        // Null-Pointer-Check:
        if (proxy->getBvHierarchyNode()->getBoundingVolume()) {

            // indicator, did add succeed?
            bool successfullAdd = addProxyToGrid(proxy);

            // updating average bounding volume size if numberOfProxies > 1
            if (numberOfProxies > 1) {
                mAverageBoundingVolumeSize =
                    (mAverageBoundingVolumeSize*((real)(numberOfProxies-1))
                        + proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbExtents()) /
                    ((real)numberOfProxies);
            }

            // updating average bounding volume size if numberOfProxies == 1,
            // which means this is the first Proxy which is added.
            else if (numberOfProxies == 1) {
                mAverageBoundingVolumeSize =
                    proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbExtents();
            }

            // This case never should be reached, as the proxy counter is
            // increased before this func. is called!
            else {
               std::cerr << dc_funcinfo << "WARNING: No Proxies!" << std::endl;
                return;
            }

            // Checking if we really added Proxy:
            if (!successfullAdd) {
//                std::cerr << dc_funcinfo << "WARNING: Proxy does not fit into the grid, increasing size!" << std::endl;

                // Increase the world, this implies recreating grid, too:
                increaseWorldSize(proxy->getBoundingVolumeCenterPosition());

// FIXME: check if mAverageBoundingVolumeSize is calculated correctly in case
// of growing the world. i.e. why is the following outcommented?
                /* In this case: Re-calculate mAverageBoundingVolumeSize:
                mAverageBoundingVolumeSize =
                   (mAverageBoundingVolumeSize*((real)(numberOfProxies))
                   - proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbExtents()) /
                   ((real)numberOfProxies-1);
                return;*/
            }
        } else {
            #ifdef DCOLLIDE_BROADPHASE_DEBUG
            std::cout << "Skipping proxy, no bounding volume!"
                << std::endl;
            #endif
        }
    }

    /*!
     * \brief remove Proxy from grid
     */
    void BroadPhaseHierarchicalGrid::removeProxy(Proxy* proxy) {
        // get the number of proxies including the one to be removed
        int numberOfProxies = getWorld()->getNumberOfTopLevelProxies();

        // Null-Pointer-Check:
        if (proxy->getBvHierarchyNode()->getBoundingVolume()) {

            // updating average bounding volume size if numberOfProxies > 1:
            if (numberOfProxies > 1) {
                mAverageBoundingVolumeSize =
                    (mAverageBoundingVolumeSize*((real)(numberOfProxies))
                        - proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbExtents()) /
                    ((real)numberOfProxies-1);
            }

            // updating average bounding volume size if numberOfProxies == 1:
            else if (numberOfProxies == 1) {
                mAverageBoundingVolumeSize =
                    proxy->getBvHierarchyNode()->getBoundingVolume()->getSurroundingAabbExtents();
            }

            // This case never should be reached, as if there are no proxies,
            // you can't remove any!
            else {
                std::cerr << dc_funcinfo << "WARNING: No Proxies!" << std::endl;
                return;
            }
        }

        // remove the proxy from grid
        HierarchicalGrid::removeProxyFromGrid(proxy);
    }

    /*!
     * Create \ref BroadPhaseJob objects for broadphase collision and add them
     * to the \ref Pipeline.
     */
    void BroadPhaseHierarchicalGrid::createBroadPhaseJobs(Proxy* proxy) {
#if 0
        writeTreeToDebug();
#endif

        // reset broadphasecollisions-container:
        getBroadPhaseCollisions()->reset();

        // Increase Counter:
        ++mGetCollisionsCalls;


        // Checking if proxy is given, in this case we use job2:
        if (proxy) {
            mJobCollection->addBroadPhaseJobsProxyOnly(getJobPoolIndex(),proxy);
        } else {
#if 1
            // Go through each Child of mGrid:
            std::vector<HierarchicalGrid*>& allChildren = mGrid->getChildren();
            mJobCollection->addBroadPhaseJobs(allChildren, getJobPoolIndex());
#else
            mJobCollection->addBroadPhaseJobs2(getJobPoolIndex());
#endif
        }

        // Getting completed Jobs and deleting them:
        // This must de done by the pipeline!
    }

    /*!
     * \brief adds a Proxy to the hierarchical Grid
     * This wrapper-method is called for each proxy in the init()-process and
     * for every new proxy which is created during simulation
     * \param proxy
     */
    bool BroadPhaseHierarchicalGrid::addProxyToGrid(Proxy* proxy) {
        if (!proxy->getBroadPhaseProxyData()) {
            proxy->setBroadPhaseProxyData(new
                BroadPhaseHierarchicalGridProxyData(proxy));
        }

        bool success = false;

        // Only add proxy to grid if size of bounding volume != 0 and if
        // Bounding Volume really exists:
        if (proxy->getBvHierarchyNode()->getBoundingVolume()) {
            if (!(proxy->getBvHierarchyNode()->getBoundingVolume()
                ->getSurroundingAabbExtents().isNull())) {
                success = mGrid->addGridMember(proxy, mMaxGridDepth, mMaxGridMembers);
            } else {
                //GJ: quickfix to prevent growing the world to infinity
                return true;
                #ifdef DCOLLIDE_BROADPHASE_DEBUG
                std::cout << "Skipping proxy, size of bounding volume is zero!"
                    << std::endl;
                #endif
            }
        } else {
                //GJ: quickfix to prevent growing the world to infinity
                return true;
                #ifdef DCOLLIDE_BROADPHASE_DEBUG
                std::cout << "Skipping proxy, bounding volume does not exist!"
                    << std::endl;
                #endif
        }
        return success;
    }

    PipelineThreadJobCollection* BroadPhaseHierarchicalGrid::getJobCollection()
    {
        return mJobCollection;
    }

    // Debugging function ONLY
    // never EVER use this code anywhere else.
    //
    // if you feel that periodic checks for tree consistency are necessary
    // (outside of cppunit tests and similar), then you are doing something
    // _completely_ wrong. the tree must always be consistent without periodic
    // checks.
    bool BroadPhaseHierarchicalGrid::checkTreeConsistency() const {
        bool ret = true;

        std::set<const HierarchicalGrid*> allLeafs;
        std::list<const HierarchicalGrid*> queue;
        queue.push_back(getHierarchicalGrid());
        if (getHierarchicalGrid()->getParent() != 0) {
            error() << "HierarchicalGrid root " << getHierarchicalGrid() << " has a parent!";
            ret = false;
        }
        while (!queue.empty()) {
            const HierarchicalGrid* gridNode = queue.front();
            queue.pop_front();
            if (!gridNode->isLeaf()) {
                const std::vector<HierarchicalGrid*>& children = gridNode->getChildrenConst();
                for (std::vector<HierarchicalGrid*>::const_iterator it = children.begin(); it != children.end(); ++it) {
                    queue.push_back(*it);
                    if ((*it)->getParent() != gridNode) {
                        error() << "HierarchicalGrid node " << *it << " is child of " << gridNode << ", but claims to be child of " << (*it)->getParent();
                        ret = false;
                    }
                }
            } else {
                allLeafs.insert(gridNode);
            }
        }

        // for every grid g that the proxies are on:
        // * check if g is a leaf
        // * check if g actually contains the proxy that referenced it
        // * check if g actually belongs to the tree (in particular: _still_
        //   belongs to the tree, i.e. has not been removed in the meantime)
        // * check if g contains a proper hierarchy, i.e. (grand) parent pointers back
        //   to the root of the tree
        // note: this method may crash if there are dangling pointers either in
        // the tree or in the proxies!
        // (if it does not crash, this method should find the dangling pointer
        // and return false)
        const std::list<Proxy*>& proxies = getWorld()->getTopLevelProxies();
        for (std::list<Proxy*>::const_iterator it = proxies.begin(); it != proxies.end(); ++it) {
            BroadPhaseHierarchicalGridProxyData* data =
                static_cast<BroadPhaseHierarchicalGridProxyData*>
                ((*it)->getBroadPhaseProxyData());
            if (!data) {
                error() << "proxy " << *it
                    << "is in the World, but has no "
                    << "BroadPhaseHierarchicalGridProxyData";
                ret = false;
                continue;
            }
            const List<HierarchicalGrid*>& grids = data->getOccupiedGrids();
            for (ListNode<HierarchicalGrid*>* node = grids.getFirstNode(); node; node = node->getNext()) {
                HierarchicalGrid* g = node->getData();

                if (!g->isLeaf()) {
                    error() << "proxy " << *it << " is on a non-leaf HierarchicalGrid!";
                    ret = false;
                }

                // if the proxy claims to be in g, then g must know about that.
                bool found = false;
                List<const Proxy*>* members = g->getGridMembers();
                for (ListNode<const Proxy*>* memberNode = members->getFirstNode(); memberNode; memberNode = memberNode->getNext()) {
                    const Proxy* p = memberNode->getData();
                    if (p == *it) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    error() << "proxy " << *it << " claims to be in HierarchicalGrid " << g << ", but that node does not know about proxy " << *it;
                    ret = false;
                }

                if (allLeafs.find(g) == allLeafs.end()) {
                    error() << "HierarchicalGrid object " << g << " is not a leaf of tree " << getHierarchicalGrid();
                    ret = false;
                    continue;
                }

                while (g->getParent()) {
                    const std::vector<HierarchicalGrid*>& childrenOfParent = g->getParent()->getChildren();
                    int count = std::count(childrenOfParent.begin(), childrenOfParent.end(), g);
                    if (count != 1) {
                        error() << "HierarchicalGrid object " << g << " is " << count << " times child of " << g->getParent();
                        ret = false;
                    }
                    g = g->getParent();
                }
                if (g != getHierarchicalGrid()) {
                    error() << "toplevel HierarchicalGrid is NOT our grid!";
                    ret = false;
                }
            }
        }

        // the other way around: check if all proxies in the grid are actually
        // valid World proxies
        queue.clear();
        queue.push_back(getHierarchicalGrid());
        while (!queue.empty()) {
            const HierarchicalGrid* gridNode = queue.front();
            queue.pop_front();
            if (!gridNode->isLeaf()) {
                const std::vector<HierarchicalGrid*>& children = gridNode->getChildrenConst();
                for (std::vector<HierarchicalGrid*>::const_iterator it = children.begin(); it != children.end(); ++it) {
                    queue.push_back(*it);
                }

                if (!const_cast<HierarchicalGrid*>(gridNode)->getGridMembers()->empty()) {
                    error() << "gridNode " << gridNode << " is not a leaf, but contains " << const_cast<HierarchicalGrid*>(gridNode)->getGridMembers()->size() << " members";
                    ret = false;
                }
            }

            List<const Proxy*>* members = const_cast<HierarchicalGrid*>(gridNode)->getGridMembers();
            for (ListNode<const Proxy*>* memberNode = members->getFirstNode(); memberNode; memberNode = memberNode->getNext()) {
                Proxy* p = const_cast<Proxy*>(memberNode->getData());
                if (std::find(proxies.begin(), proxies.end(), p) == proxies.end()) {
                    error() << "proxy " << p << " in the tree is not in World";
                    ret = false;
                }
                BroadPhaseHierarchicalGridProxyData* data =
                    static_cast<BroadPhaseHierarchicalGridProxyData*>
                    (p->getBroadPhaseProxyData());
                if (!data) {
                    error() << "proxy " << p << "is in the World, but has no"
                        << "BroadPhaseHierarchicalGridProxyData";
                    ret = false;
                    continue;
                }

                bool found = false;
                const List<HierarchicalGrid*>& grids = data->getOccupiedGrids();
                for (ListNode<HierarchicalGrid*>* node = grids.getFirstNode(); node; node = node->getNext()) {
                    HierarchicalGrid* g = node->getData();
                    if (g == gridNode) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    error() << "HierarchicalGrid node " << gridNode << " claims to contain proxy " << p << " but that proxy does not know about the HierarchicalGrid node.";
                    ret = false;
                }

            }
        }

        return ret;
    }

    void BroadPhaseHierarchicalGrid::writeTreeToDebug() {
        std::string separator = "---------------------------------------------------------------------";
        debug(501) << separator;
        std::list<HierarchicalGrid*> queue;
        queue.push_back(mGrid);
        std::string indentStep = "  ";
        while (!queue.empty()) {
            HierarchicalGrid* node = queue.front();
            queue.pop_front();
            std::string indent;
            for (int i = 0; i < node->getGridDepth(); i++) {
                indent += indentStep;
            }
            debug(501) << indent << "HG " << node << " (parent "
                    << node->getParent() << ")"
                    << " at depth=" << node->getGridDepth()
                    << " with members=" << node->getGridMembers()->size()
                    << (node->isLeaf() ? " IAL" : " NAL");

            std::string proxyIndent = indent;
            proxyIndent += indentStep;
            for (ListNode<const Proxy*>* memberNode = node->getGridMembers()->getFirstNode(); memberNode; memberNode = memberNode->getNext()) {
                debug(501) << proxyIndent << " member Proxy: " << memberNode->getData();
            }

            for (std::vector<HierarchicalGrid*>::const_reverse_iterator it = node->getChildrenConst().rbegin(); it != node->getChildrenConst().rend(); ++it) {
                queue.push_front(*it);
            }
        }

        for (std::list<Proxy*>::const_iterator it = getWorld()->getTopLevelProxies().begin(); it != getWorld()->getTopLevelProxies().end(); ++it) {
            BroadPhaseHierarchicalGridProxyData* data =
                static_cast<BroadPhaseHierarchicalGridProxyData*>
                ((*it)->getBroadPhaseProxyData());
            debug(501) << "Proxy " << *it << " is in " << data->getOccupiedGrids().size() << " grid nodes";
            for (ListNode<HierarchicalGrid*>* node = data->getOccupiedGrids().getFirstNode(); node; node = node->getNext()) {
                debug(501) << indentStep << node->getData();
            }
        }
    }
}

/*
 * vim: et sw=4 ts=4
 */
