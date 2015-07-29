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

// author: Marc Schulz <shade@nightmareshadow.de>

#include "surfacehierarchy.h"

#include "surfacehierarchyalgorithm.h"
#include "surfacehierarchyproxydata.h"
#include "proxy.h"
#include "world.h"
#include "shapes/mesh.h"
#include "shapes/mesh/triangle.h"
#include "shapes/mesh/vertex.h"
#include "shapes/shape.h"
#include "real.h"
#include "deformablebvhnode.h"
#include "bvhnode.h"
#include "debugstream.h"

#include <math.h>
#include <limits>
#include <algorithm>
#include <set>

#include <stdlib.h>
#include <time.h>

// remember that this is a botton up approach, so LEVEL_ONE is the lowest level of Spheres in the hierarchy (level ZERO are the triangles)!
// #define SURFACEHIERARCHY_DEBUG_LEVEL_ONE 1
// #define SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS 1
// #define SURFACEHIERARCHY_DEBUG_LEVEL_TWO 1
// #define SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS 1

namespace dcollide {
    SurfaceHierarchyCreator::SurfaceHierarchyCreator(SurfaceHierarchyAlgorithm* algorithm, World* world, Proxy* proxy) {
        mAlgorithm = algorithm;
        mWorld = world;
        mProxy = proxy;
    }

    SurfaceHierarchyCreator::~SurfaceHierarchyCreator() {
    }

    /*!
     * Creates a hierarchy of \ref BoundingSphere objects on the proxy provided
     * in the constructor.
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchyCreator::generateSurfaceHierarchy() {

        // debug(10) << "generating surface hierarchy level 1...";

        if (!mProxy) {
            throw NullPointerException("mProxy");
        }
        if (!mProxy->getShape()) {
            throw Exception("NULL mesh->getShape()");
        }
        if (!mProxy->getShape()->getMeshIfExists()) {
            throw Exception("NULL mesh - not a deformable object");
        }

        mCreatedBoundingSphereRing = new std::list<DeformableBvhNode*>();

        if (!initializeFirstRing()) {
            delete mCreatedBoundingSphereRing;
            mCreatedBoundingSphereRing = 0;
            throw Exception("Initialization of first ring failed");
        }

        mBoundingSphereRingContainer.push_back(mCreatedBoundingSphereRing);
        mCurrentBorderOfDoneRegion = mNewBorder;


        // ***Main-Loop-Start***
        // each iteration correspondes to the creation of a new circle/ring of boundingspheres around the totally done region
        while (true) {

            mCreatedBoundingSphereRing = new std::list<DeformableBvhNode*>();

            mNewBorder.clear();

            //******************************************************************************************************************************
            //1) find the surrounding triangles that are adjacent to the border and are not allready in the mTotallyDoneTrianglesSet
            //******************************************************************************************************************************
            // These are the triangles that surround the allready done region. they are adjecent to the border with at least one EDGE,
            // -> one vertex isn't enough!
            std::list<Triangle*> surroundingTriangles = calculateSurroundingTriangles();


            //******************************************************************************************************************************
            //2) find the new working set which represents the set of vertecies which are potential centers of new spheres in this round
            //******************************************************************************************************************************
            // definition of a the workingsets of the algorithm
            // These are potential new centers of new spheres. These are all vertices which are reachable from the border in exactly ONE step.
            std::list<Vertex*> currentWorkingSetOfVertices = calculatePotentialNewCenters(surroundingTriangles);


            //******************************************************************************************************************************
            //no more potential centers found? -> then the object is totally processed , so break the while-loop!
            if (currentWorkingSetOfVertices.empty()) {
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                debug(10) << "algorithm complete";
#endif
                break;
            }
            //******************************************************************************************************************************


            // it is checked, whether the surrounding triangles allready belong to a Sphere ...
            // if no surrounding triangle ist left in the list, because they all allready belong to a sphere,
            // the vertex is removed from the potential centers and this step in working on the potential center list is aborted.
            //******************************************************************************************************************************
            // 3.1) creating a new BoundingSphere and adding the enclosed Triangles to mTotallyDoneTrianglesSet
            //******************************************************************************************************************************
            createSpheresAtPotentialCenters(currentWorkingSetOfVertices);
            currentWorkingSetOfVertices.clear();
            mCurrentBorderOfDoneRegion = mNewBorder;


            //******************************************************************************************************************************
            //4) Check whether all surroundingTriangles have been assigned to a boundingSphere and if not -> assign each of them to the
            //   adjacent sphere which grows the smallest amount while adding the triangle!
            //******************************************************************************************************************************
            findAndProcessGaps(surroundingTriangles);
            //or GapDummy(surroundingTriangles); instead.

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
            debug(10) << "#created spheres in this ring: " << mCreatedBoundingSphereRing->size();
#endif

            mBoundingSphereRingContainer.push_back(mCreatedBoundingSphereRing);

        } // while end

#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
        // print out the sizes of the mBoundingSphereRingContainer
        debug(10) << "#rings in mBoundingSphereRingContainer: " << mBoundingSphereRingContainer.size();
        int counter_for_the_rings = 0;
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_ring2 = mBoundingSphereRingContainer.begin();
                                                            iter_ring2 != mBoundingSphereRingContainer.end();
                                                            ++iter_ring2) {
            counter_for_the_rings++;
            debug(10) << "#boundingSpheres in ring " << counter_for_the_rings << ": " << (*iter_ring2)->size();
        }
#endif

        // initializeNeighboursOfBoundingSpheresInLevelOne();
        initializeNeighboursOfBoundingSpheresInLevelOne();


        //*********************************************************************************************************************
        //create a fully surfacebound hierarchy
        int mBoundingSphereRingContainer_size = mBoundingSphereRingContainer.size();
        while (mBoundingSphereRingContainer_size > 1) {

            // create level two of the hierarchy
            createHigherLevelOfHierarchy();
            // initialize neighbours of level 2 BoundingSpheres
            initializeNeighboursOfHigherBoundingSphereLevel();

            int newSize = mBoundingSphereRingContainer.size();
            if (newSize >= mBoundingSphereRingContainer_size) {
                error(10) << "mBoundingSphereRingContainer.size() has not changed after one iteration! aborting loop.";
                break;
            }
            mBoundingSphereRingContainer_size = newSize;
        }

        static_cast<SurfaceHierarchyProxyData*>(mAlgorithm->getProxyDataFor(mProxy))->mDeformableNode = *(*mBoundingSphereRingContainer.begin())->begin();

        //deleting the no mot further needed list of lists
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_clear_ring = mBoundingSphereRingContainer.begin();
                                                            iter_clear_ring != mBoundingSphereRingContainer.end();
                                                            ++iter_clear_ring) {
            delete(*iter_clear_ring);
        }
        //*********************************************************************************************************************

/*
        for (int i = 1; i <= (countSurfaceBoundLevels - 2); i++) {
            // check whether mBoundingSphereRingContainer contains only one sphere! -> hierarchy has allready merged to one sphere
            if (mBoundingSphereRingContainer.size() <= 2) {
                std::cout << "WARNING : not all requested levels could be created (reason: resolution of model too small)" << std::endl;
                break;
            }
            // create level two of the hierarchy
            createHigherLevelOfHierarchy();
            // initialize neighbours of level 2 BoundingSpheres
            initializeNeighboursOfHigherBoundingSphereLevel();
        }


        // DEBUG: create the toplevel (i.e. root) node
        // merge all spheres into one list for displaying them
        std::list< std::list<DeformableBvhNode*>* > tempRingContainer = mBoundingSphereRingContainer;
        std::list<DeformableBvhNode*> allCreatedBoundingSpheres;
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_ring = tempRingContainer.begin();
                                                            iter_ring != tempRingContainer.end();
                                                            ++iter_ring) {
            allCreatedBoundingSpheres.merge(**iter_ring);
        }

        //deleting the no mot further needed list of lists
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_clear_ring = mBoundingSphereRingContainer.begin();
                                                            iter_clear_ring != mBoundingSphereRingContainer.end();
                                                            ++iter_clear_ring) {
            delete(*iter_clear_ring);
        }

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
        debug(10) << "trying to create topLevelSphere...";
#endif

        // ... if a single DeformableBvhNode shall get hung into a new higher DeformableBvhNode just merge them in the constructor of DeformableBvhNode
        // but take care : the one sphere can be surfacebound and the otherone can be non-surfacebound
        DeformableBvhNode* node = new DeformableBvhNode(mWorld,
                allCreatedBoundingSpheres, mProxy,
                static_cast<SurfaceHierarchyProxyData*>(mAlgorithm->getProxyDataFor(mProxy)));
        node->initializeNode();

        // debug(10) << "surface hierarchy done!";
*/
    }


    /*!
     * finds and processes gaps
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchyCreator::findAndProcessGaps(std::list<Triangle*>& surroundingTriangles) {

//DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "!!!processing Gaps ...";
#endif

        while(!surroundingTriangles.empty()) {


            // assign the components of the not assigned triangles to the totallyDone-lists so that the new borders and new workingsets stay correct
            Triangle* triangle = *(surroundingTriangles.begin());

//DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
            debug(10) << "... processing Gap-Triangle ...";
#endif

            // add the triangle to the mTotallyDoneTrianglesSet
            std::set<Triangle*>::iterator result_tri = mTotallyDoneTrianglesSet.find(triangle);
            if (result_tri == mTotallyDoneTrianglesSet.end()) {
                mTotallyDoneTrianglesSet.insert(triangle);
            }

            const dcollide::array<Vertex*,3>& adja_Vertices = triangle->getVertices();

            // add the Vertices of the Triangle to the mTotallyDoneVerticesSet
            for (dcollide::array<Vertex*,3>::iterator q = adja_Vertices.begin();
                                                        q != adja_Vertices.end();
                                                        ++q) {

                std::set<Vertex*>::iterator result_vert = mTotallyDoneVerticesSet.find(*q);
                if (result_vert == mTotallyDoneVerticesSet.end()) {
                    mTotallyDoneVerticesSet.insert(*q);
                }
            }


            // assign the triangles, which do not already belong to a boundingsphere to the one which has to grow the smallest amount
            if (triangle->getBvHierarchyNodes().empty()) {

                real smallestGrowthAmount = std::numeric_limits<real>::infinity();
                DeformableBvhNode* chosenDeformableBvhNode = 0;

                // check which adjacent boundingSphere has to grow the smallest amount if the triangle is added
                const std::set<Triangle*>& neighbourTriangles = triangle->getEdgeAdjacentTriangles();
                std::set<Triangle*>::const_iterator neighbour;

                for (std::set<Triangle*>::const_iterator currentNeighbour = neighbourTriangles.begin();
                                                            currentNeighbour != neighbourTriangles.end();
                                                            ++currentNeighbour) {
                    // check if the currentNeighbour really has a boundingsphere
                    const std::list<const BvhNode*>& bvhNodes = (*currentNeighbour)->getBvHierarchyNodes();
                    if(!bvhNodes.empty()) {

                        const BvhNode* node = (const DeformableBvhNode*) bvhNodes.back();
                        const BoundingSphere* sphere = (const BoundingSphere*) (node->getBoundingVolume());
                        // doublecheck if the sphere is not NULL. Just to be sure.
                        if (sphere != NULL) {

                            // get the center of the sphere
                            const Vector3 sphereCenter = sphere->getCenterVertex()->getPosition();
                            // get the radius of the sphere
                            const real sphereRadius = sphere->getRadius();
                            // now find the farthest vertex of the unassigned triangle, relative to the center and radius
                            // i.e. has the sphere to grow if we add this triangle? -> if so, HOW MUCH?
                            real growthAmount = 0;

                            const dcollide::array<Vertex*,3>& triangleVertices = triangle->getVertices();
                            for (dcollide::array<Vertex*,3>::iterator triangleVertex = triangleVertices.begin();
                                                                        triangleVertex != triangleVertices.end();
                                                                        ++triangleVertex) {

                                // find the distance of the farthest vertex to the sphereCenter
                                Vector3 tempVector = sphereCenter - ((*triangleVertex)->getPosition());
                                if ((tempVector.length() - sphereRadius) > growthAmount) {
                                    growthAmount = tempVector.length();
                                }
                            }

                            if (growthAmount <= smallestGrowthAmount) {
                                smallestGrowthAmount = growthAmount;
                                chosenDeformableBvhNode = (DeformableBvhNode*) node;
                            }

                        } else {
                        // the sphere of the currentNeighbours bvhNode is NULL, so print it out for debug
//DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                            debug(10) << "GAP - sphere of the bvhNode is NULL!";
#endif
                        }

                    } else {
                    // the currentNeighbour had no boundingsphere, so print it out for debug
//DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                        debug(10) << ">>>>>>>>>>>>>neighbour also has no boundingsphere!!!";
#endif
                    }
                }

                if (chosenDeformableBvhNode == 0) {
//DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                    debug(10) << ">>>!!!chosenDeformableBvhNode is NULL!!!<<<";
#endif
                    surroundingTriangles.pop_front();
                } else {

                    chosenDeformableBvhNode->addTriangle(triangle);
//DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                    debug(10) << ">>>adding Triangle to neighbouring sphere!!!<<<";
#endif
                    surroundingTriangles.pop_front();
                }

            } else {

            // kick out those triangles which already have been assigned:
                surroundingTriangles.pop_front();
//DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                debug(10) << "triangle allready has a boundingvolume! -kicked";
#endif

            }
        }
        //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "size of totally done triangles:" << mTotallyDoneTrianglesSet.size();
#endif
    }



    /*!
     * sorts the workingSet
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    // sorting the new working set with neighbourship as key attribute
    std::list<Vertex*> SurfaceHierarchyCreator::sortWorkingSet(std::list<Vertex*>& workingSet) {


#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "sorting: ...";
#endif
        std::list<Vertex*> currentWorkingSetOfVertices_sorted;
        std::list<Vertex*> currentWorkingSetOfVertices_sorted_part;

        while(!workingSet.empty()) {

            currentWorkingSetOfVertices_sorted_part.clear();
            currentWorkingSetOfVertices_sorted_part.push_front(workingSet.front());
            workingSet.pop_front();

            bool found = 1;

            while(found == 1) {

                found = 0;

                std::list<Vertex*> neighbourset = (currentWorkingSetOfVertices_sorted_part.front())->getAdjacentVertices();

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                debug(10) << "item:" << currentWorkingSetOfVertices_sorted_part.size();

                printOutVertex(currentWorkingSetOfVertices_sorted_part.front());
#endif

                for(std::list<Vertex*>::iterator neighbour = workingSet.begin();
                                                    neighbour != workingSet.end();
                                                    ++neighbour) {

                    std::list<Vertex*>::iterator is_neighbour = find_if(neighbourset.begin(),
                                                                        neighbourset.end(),
                                                                        bind2nd(std::equal_to<Vertex*>(), const_cast<Vertex*>(*neighbour)));
                    if (is_neighbour != neighbourset.end()) {
                        currentWorkingSetOfVertices_sorted_part.push_front(*neighbour);
                        workingSet.erase(neighbour);
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                        debug(10) << "moved!";
#endif
                        found = 1;
                        break;
                    }
                }
            }

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
            debug(10) << "reverse direction for sorting";
#endif

            if (workingSet.empty()) {
                while(!currentWorkingSetOfVertices_sorted_part.empty()) {
                    currentWorkingSetOfVertices_sorted.push_front(currentWorkingSetOfVertices_sorted_part.front());
                    currentWorkingSetOfVertices_sorted_part.pop_front();
                }
                break;
            }

            found = 1;

            while(found == 1) {

                found = 0;

                std::list<Vertex*> neighbourset = (currentWorkingSetOfVertices_sorted_part.back())->getAdjacentVertices();

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                debug(10) << "item:" << currentWorkingSetOfVertices_sorted_part.size();

                printOutVertex(currentWorkingSetOfVertices_sorted_part.front());
#endif

                for(std::list<Vertex*>::iterator neighbour = workingSet.begin();
                                                   neighbour != workingSet.end();
                                                    ++neighbour) {

                    std::list<Vertex*>::iterator is_neighbour = find_if(neighbourset.begin(),
                                                                        neighbourset.end(),
                                                                        bind2nd(std::equal_to<Vertex*>(), const_cast<Vertex*>(*neighbour)));
                    if (is_neighbour != neighbourset.end()) {
                        currentWorkingSetOfVertices_sorted_part.push_back(*neighbour);
                        workingSet.erase(neighbour);
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                        debug(10) << "moved!";
#endif
                        found = 1;
                        break;
                    }
                }
            }


            while(!currentWorkingSetOfVertices_sorted_part.empty()) {
                currentWorkingSetOfVertices_sorted.push_front(currentWorkingSetOfVertices_sorted_part.front());
                currentWorkingSetOfVertices_sorted_part.pop_front();
            }

        }
        // sorting done!

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "size of new workingset (sorted):" << currentWorkingSetOfVertices_sorted.size();
        debug(10) << "new workingset items (sorted):";
        printOutVertexList(currentWorkingSetOfVertices_sorted);
#endif


        return currentWorkingSetOfVertices_sorted;
    }


    void SurfaceHierarchyCreator::printOutVertexList(std::list<Vertex*>& liste) {

        for (std::list<Vertex*>::iterator item = liste.begin();
                                                    item != liste.end();
                                                    ++item) {
            debug(10) << (*item)->getPosition().getX() << "," << (*item)->getPosition().getY() << "," << (*item)->getPosition().getZ();
        }

    }

    void SurfaceHierarchyCreator::printOutVertexVector(std::vector<Vertex*> vector) {

        for (std::vector<Vertex*>::iterator item = vector.begin();
                                                    item != vector.end();
                                                    ++item) {
            debug(10) << (*item)->getPosition().getX() << "," << (*item)->getPosition().getY() << "," << (*item)->getPosition().getZ();
        }

    }

    void SurfaceHierarchyCreator::printOutVertex(Vertex* vertex) {

        debug(10) << (*vertex).getPosition().getX() << "," << (*vertex).getPosition().getY() << "," << (*vertex).getPosition().getZ();

    }


    /*!
     * dummmy for early testing without closing the gaps
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    // DEBUG : just puts all surrounding triangles which not allready are into mTotallyDoneTrianglesSet and mTotallyDoneVerticesSet
    void SurfaceHierarchyCreator::GapDummy(std::list<Triangle*>& surroundingTriangles) {
        while(!surroundingTriangles.empty()){

            std::list<Triangle*>::iterator triangle = surroundingTriangles.begin();

            std::set<Triangle*>::iterator result_xyz = mTotallyDoneTrianglesSet.find(*triangle);
            if (result_xyz == mTotallyDoneTrianglesSet.end()) {
                mTotallyDoneTrianglesSet.insert(*triangle);
            }

            const dcollide::array<Vertex*,3>& adj_Vertices = (*triangle)->getVertices();

            for (dcollide::array<Vertex*,3>::iterator p = adj_Vertices.begin();
                                                        p != adj_Vertices.end();
                                                        ++p) {

                std::set<Vertex*>::iterator result_vert = mTotallyDoneVerticesSet.find(*p);
                if (result_vert == mTotallyDoneVerticesSet.end()) {
                    mTotallyDoneVerticesSet.insert(*p);
                }
            }

            surroundingTriangles.pop_front();

        }
    }


    /*!
     * first ring is created here to initialize the algorithm correctly
     *
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    //******************************************************************************************************************************
    // Doing the first step here , to initialize the Algorithm correctly
    //******************************************************************************************************************************
    bool SurfaceHierarchyCreator::initializeFirstRing() {
        // get the Triangles of the Object
        if (mProxy == 0) {
            error(10) << "FATAL ERROR : PROXY IS ZERO!!!";
            return false;
        }
        const std::vector<Triangle*>& allTriangles = mProxy->getShape()->getMeshIfExists()->getTriangles();
        // get ONE triangle of the Object
        if (allTriangles.empty()) {
            error(10) << "FATAL ERROR : MODEL NOT LOADED (MESH HOLDS NO TRIANGLES)!!!";
            return false;
        }
        std::vector<Triangle*>::const_iterator iter = allTriangles.begin();

        if ((*iter) == 0) {
            error(10) << "FATAL ERROR : FIRST TRIANGLE POINTS TO ZERO!!!";
            return false;
        }

        if ((*iter)->getBvHierarchyNodes().empty()) {
//DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
            debug(10) << "FIRST TRIANGLE HAS ZERO BOUNDINGVOLUMES!!!";
#endif
        }

        // get the Vertices of THIS triangle and use it as the center of the first sphere!
        const dcollide::array<Vertex*,3>& allVerticesOfPoly = (*iter)->getVertices();

        //get the first vertex of the triangle
        Vertex* firstCenter = *allVerticesOfPoly.begin();

        // define the max number of hops
        int seed = 10000;
        // initialize random seed from time
        srand ( time(NULL) );
        // get a random number of hobs through the vertices
        int number_of_jumps = (rand() % seed);

        for(int i = 0; i != number_of_jumps; i++) {

            const std::list<Vertex*> jumpNeighbours = firstCenter->getAdjacentVertices();

            // get random jump direction
            int jumpDirection = (rand() % jumpNeighbours.size());

            std::list<Vertex*>::const_iterator jump_direction = jumpNeighbours.begin();

            for(int j = 0; j != jumpDirection; j++) {
                ++jump_direction;
            }

            firstCenter = *jump_direction;
        }
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "first Center: " << firstCenter->getWorldPosition().getX() << ", " << firstCenter->getWorldPosition().getY() << ", " << firstCenter->getWorldPosition().getZ();
#endif

        //add the center to the mTotallyDoneVerticesSet
        mTotallyDoneVerticesSet.insert(firstCenter);
        // put the enclosed Triangles into the totally done list
        std::list<Triangle*> tempTriangles = (firstCenter)->getAdjacentTriangles();
        for (std::list<Triangle*>::iterator iter_temp_triangles = tempTriangles.begin();
                                                iter_temp_triangles != tempTriangles.end();
                                                ++iter_temp_triangles) {
            mTotallyDoneTrianglesSet.insert(*iter_temp_triangles);
        }

        //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "size of totally done triangles:" << mTotallyDoneTrianglesSet.size();
#endif

        // creating the sphere all the preprocessing was done for
        DeformableBvhNode* node = new DeformableBvhNode(mWorld, firstCenter->getAdjacentTriangles(), firstCenter);
        node->initializeNode();
        mCreatedBoundingSphereRing->push_front(node);

        // fill the border of done region list
        mNewBorder = (firstCenter->getAdjacentVertices());
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "new Border first ring:" << mNewBorder.size();
        debug(10) << "new Border items:";
        printOutVertexList(mNewBorder);
#endif

        return true;
    }



    /*!
     * calculates the surrounding triangles of the current totally done region
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    std::list<Triangle*> SurfaceHierarchyCreator::calculateSurroundingTriangles() {
        std::list<Triangle*> surroundingTriangles;

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "current border of done region items (unsorted):";
        printOutVertexList(mCurrentBorderOfDoneRegion);
        debug(10) << "/end current border.";
#endif

        // these Triangles have to be checked after one cycle, whether all of them are assigned to a BoundingSphere.
        for (std::list<Vertex*>::iterator k = mCurrentBorderOfDoneRegion.begin();
                                            k != mCurrentBorderOfDoneRegion.end();
                                            ++k) {
            // look at the surrounding triangles of this vertex
            const std::list<Triangle*> tempAdjacentTriangles = (*k)->getAdjacentTriangles();
            for (std::list<Triangle*>::const_iterator l = tempAdjacentTriangles.begin();
                                                    l != tempAdjacentTriangles.end();
                                                    ++l) {
                // is the triangle already in totallyDoneTriangles?
                std::set<Triangle*>::iterator result_1 = mTotallyDoneTrianglesSet.find(*l);

                // is the triangle already in the list of the surrounding triangles?
                std::list<Triangle*>::iterator result_2 = find_if(surroundingTriangles.begin(),
                                                                surroundingTriangles.end(),
                                                                bind2nd(std::equal_to<Triangle*>(), const_cast<Triangle*>(*l)));

                // combining the two preceding questions:
                if ((result_1 == mTotallyDoneTrianglesSet.end()) && (result_2 == surroundingTriangles.end())) {
                    //if the triangle isn't in both of the lists, then put it into the surroundingTriangles List!
                    surroundingTriangles.push_front(*l);
                }
            }

        }

        //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
        debug(10) << "#surrounding Triangles:" << surroundingTriangles.size();
#endif

        return surroundingTriangles;
    }


    /*!
     * calculates the new potential centers using the surrounding triangles and the current border
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    std::list<Vertex*> SurfaceHierarchyCreator::calculatePotentialNewCenters(std::list<Triangle*>& surroundingTriangles) {

        //DEBUG
        int triangle_counter = 0;

        std::list<Vertex*> currentWorkingSetOfVertices;

        std::list<Triangle*>::iterator o = surroundingTriangles.begin();
        while (o != surroundingTriangles.end()) {

            const dcollide::array<Vertex*,3>& verticesOfPoly = (*o)->getVertices();

            //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
            debug(10) << "surrounding Triangle #" << triangle_counter << " its vertices are:";
            printOutVertexVector(verticesOfPoly);
#endif
            triangle_counter++;

            // how many of the three vertices of this triangle will be added to the potential new centers? -> count it!
            int count_new_centercontacts = 0;

            for (dcollide::array<Vertex*,3>::iterator p = verticesOfPoly.begin();
                                                    p != verticesOfPoly.end();
                                                    ++p) {

                //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                debug(10) << "vertex in work";
                printOutVertex(*p);
#endif

                std::list<Vertex*>::iterator result_border = find_if(mCurrentBorderOfDoneRegion.begin(),
                                                                mCurrentBorderOfDoneRegion.end(),
                                                                bind2nd(std::equal_to<Vertex*>(), const_cast<Vertex*>(*p)));

                std::list<Vertex*>::iterator result_working_set = find_if (currentWorkingSetOfVertices.begin(),
                                                                    currentWorkingSetOfVertices.end(),
                                                                    bind2nd(std::equal_to<Vertex*>(), const_cast<Vertex*>(*p)));

                if (result_border == mCurrentBorderOfDoneRegion.end()) {
                    // counts the new potential centers of this triangle:
                    count_new_centercontacts++;
                }

                if ((result_border == mCurrentBorderOfDoneRegion.end()) && (result_working_set == currentWorkingSetOfVertices.end())) {
                    // the vertex is not a member of the current border, so it is added to the potential centers of new spheres
                    currentWorkingSetOfVertices.push_front(*p);
                    // counts the new potential centers of this whole round:
                }
            }

            // if TWO vertices of this triangle have been added to the potential new centers, then this triangle can be removed
            // from the surroundingTriangles because it has only one vertex in common with the border,
            // so it will be assigned to a new sphere anyway and can be removed from the surroundingTriangles list,
            // (which has to be checked, if all of its members got assigned in the last step of each round!)
            if (count_new_centercontacts > 1) {

                if (*o == surroundingTriangles.back()) {
                    std::list<Triangle*>::iterator temp_copy = surroundingTriangles.erase(o);
                    o = temp_copy;
                } else {
                    std::list<Triangle*>::iterator temp_copy = surroundingTriangles.erase(o);
                    o = temp_copy;
                }

            } else {
                ++o;
            }

        }

        if (currentWorkingSetOfVertices.empty()) {
            return currentWorkingSetOfVertices;
        } else {
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
            debug(10) << "size of new workingset (unsorted):" << currentWorkingSetOfVertices.size();
            debug(10) << "new workingset items (unsorted):";
            printOutVertexList(currentWorkingSetOfVertices);
#endif
            return(sortWorkingSet(currentWorkingSetOfVertices));
        }

    }


    /*!
     * creates the new spheres at some of the potential new centers and finds the new border
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchyCreator::createSpheresAtPotentialCenters(const std::list<Vertex*>& currentWorkingSetOfVertices_) {
        // this list will hold the vertices of the currentWorkingSet which didn't become a center. those have to be looked at to complete the border.
        std::list<Vertex*> borderCompletionBuffer;

        std::list<Vertex*> currentWorkingSetOfVertices = currentWorkingSetOfVertices_;

        while(!currentWorkingSetOfVertices.empty()){

            std::list<Vertex*>::iterator center = currentWorkingSetOfVertices.begin();

            // get surrounding Triangles which will be potentially put together into one sphere
            const std::list<Triangle*> adjacentTriangles_temp = (*center)->getAdjacentTriangles();

            // this list will only contain the triangles that are not totally done and will surely get into the new sphere.
            std::list<Triangle*> adjacentTriangles = std::list<Triangle*>();

            // put these Triangles into mTotallyDoneTrianglesSet, because they'll get a Sphere and don't have to be touched again.
            // in the following it is checked whether any surrounding triangles of the center are allready in the totally done list.
            // if this is the case, the triangle is removed from the surrounding list and in this case is discarded
            // for this upcomming sphere. In the other case it is added to the totally done list because it will be
            // included in the upcomming sphere.
            for (std::list<Triangle*>::const_iterator i = adjacentTriangles_temp.begin();
                                                i != adjacentTriangles_temp.end();
                                                i++) {

                std::set<Triangle*>::iterator result_triangle_allready_done = mTotallyDoneTrianglesSet.find(*i);
                if (result_triangle_allready_done == mTotallyDoneTrianglesSet.end()) {
                    mTotallyDoneTrianglesSet.insert(*i);
                    adjacentTriangles.push_front(*i);
                }
            }

            // check whether the list adjacentTriangles is empty. if this is the case then break this cycle for this center!
            // doing this at this point just to be on the safe side!
            if (adjacentTriangles.empty()) {
                //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                debug(10) << "ooops, all triangles allready processed!!!";
#endif
                // is break enough for this? -> want to leave THIS CYCLE of the surrounding for-structure i.e. THIS Sphere mustn't be created!
                mTotallyDoneVerticesSet.insert(*center);
                // remove this vertex from the working set - it will be processesd in the following, so it has to be removed
                currentWorkingSetOfVertices.pop_front();
                break;
            }

            // get surrounding Vertecies of the center of the future sphere and put these into totallyDoneVertecies,
            // if they are allready member of the border
            const std::list<Vertex*>& adjacentVertices = (*center)->getAdjacentVertices();

            for (std::list<Vertex*>::const_iterator j = adjacentVertices.begin();
                                                    j != adjacentVertices.end();
                                                    j++) {

                std::list<Vertex*>::const_iterator result_vertex_already_done = find_if(mCurrentBorderOfDoneRegion.begin(),
                                                                                    mCurrentBorderOfDoneRegion.end(),
                                                                                    bind2nd(std::equal_to<Vertex*>(), const_cast<Vertex*>(*j)));
                if (result_vertex_already_done != mCurrentBorderOfDoneRegion.end()) {
                    mTotallyDoneVerticesSet.insert(*j);
                }

                // entferne alle potentiellen mittelpunkte aus currentWorkingSet , die in center.getAdjecentVertices() sind.
                // ist der punkt der entfernt werden soll nicht mehr in der liste drin , dann ist der aktuell bearbeitete ring geschlossen
                // und die posentiellen mittelpunkte waren durch zwei teilbar >>> d.h. es gibt keine unregelmaessige nahtstelle.
                // ist der letzte punkt der aus currentworkingSet entfernt werden soll auch der letzte in der liste , so wird er nicht entfernt,
                // weil er der letzte mittelpunkt wird und somit als mittelpunkt automatisch aus der liste geloescht wird.
                std::list<Vertex*>::iterator result_vertex_potential_center = find_if(currentWorkingSetOfVertices.begin(),
                                                                                    currentWorkingSetOfVertices.end(),
                                                                                    bind2nd(std::equal_to<Vertex*>(), const_cast<Vertex*>(*j)));

                if (result_vertex_potential_center != currentWorkingSetOfVertices.end()) {

                    //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                    debug(10) << "found one in working set!";
#endif

                    if (*result_vertex_potential_center != currentWorkingSetOfVertices.back()) {
                        // put it into the borderCompletionBuffer, which will fill the holes in the new border
                        borderCompletionBuffer.push_front(*j);

                        // remove it from the potential centers, i.e. currentWorkingSet
                        std::list<Vertex*>::iterator temp = currentWorkingSetOfVertices.erase(result_vertex_potential_center);
                        result_vertex_potential_center = temp;

                        //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
                        debug(10) << "removed!";
#endif
                    }
                }

                // This will fill mNewBorder with the new border for the following cycle - but not completely!
                // (see borderCompletionBuffer)
                if ( (result_vertex_already_done == mCurrentBorderOfDoneRegion.end()) &&
                    (result_vertex_potential_center == currentWorkingSetOfVertices.end()) ) {
                    mNewBorder.push_front(*j);
                }
            }

            // add the center to the mTotallyDoneVerticesSet
            mTotallyDoneVerticesSet.insert(*center);
            // creating the sphere all the preprocessing was done for
            DeformableBvhNode* node = new DeformableBvhNode(mWorld, adjacentTriangles, *center);
            node->initializeNode();
            mCreatedBoundingSphereRing->push_front(node);

            // remove this vertex from the working set - it will be processesd in the following, so it has to be removed
            currentWorkingSetOfVertices.pop_front();

            //DEBUG
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_ONE
            debug(10) << "#totally done Triangles:";
            debug(10) << mTotallyDoneTrianglesSet.size();
            debug(10) << "!!!workingset items after sphere:";
            printOutVertexList(currentWorkingSetOfVertices);

#endif
        }


        // die alte currentWorkingSetOfVertices kopieren , dann kann während der erstellung der Spheres mit dieser verglichen werden
        // und on the fly die neue border berechnet werden!
        // man kann dann auf der KOPIE arbeiten und das original nur für die vergleiche benutzen oder vice versa!!!
        //******************************************************************************************************************************
        //3.2) completion of the border! SIEHE OBEN!!! GESCHIEHT ZUM TEIL BEIM BSPHERES ERSTELLEN; IST ABER UNFERTIG!!!
        //******************************************************************************************************************************
        for (std::list<Vertex*>::iterator k = borderCompletionBuffer.begin();
                                            k != borderCompletionBuffer.end();
                                            ++k) {

            std::set<Vertex*>::iterator totally_done = mTotallyDoneVerticesSet.find(*k);

            // already in mNewBorder included:
            std::list<Vertex*>::iterator newBorder_allready_done = find_if(mNewBorder.begin(),
                                                                        mNewBorder.end(),
                                                                        bind2nd(std::equal_to<Vertex*>(), const_cast<Vertex*>(*k)));

            if ((totally_done == mTotallyDoneVerticesSet.end()) && (newBorder_allready_done == mNewBorder.end())) {
                mNewBorder.push_front(*k);
            }
        }
        //******************************************************************************************************************************
    }


    /*!
     * fills the list of each DeformableBvhNode, which contains the neighbours of the sphere belonging to this Node
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    // This Function initializes the neighbours of the boundingspheres in the lowest level.
    // To do this it uses the mBoundingSphereRingContainer which contains all rings ordered from first to last
    // and each ring contains its BoundingSpheres in a closed order, or at least in ordered in parts,
    // if there were gaps or spikes sticking out in the model.
    void SurfaceHierarchyCreator::initializeNeighboursOfBoundingSpheresInLevelOne() {

        // debug(10) << "initializing neighbourships of level 1...";

#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
        // print out the sizes of the mBoundingSphereRingContainer
        debug(10) << "#rings in mBoundingSphereRingContainer: " << mBoundingSphereRingContainer.size();
        int counter_for_the_rings = 0;
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_ring2 = mBoundingSphereRingContainer.begin();
                                                            iter_ring2 != mBoundingSphereRingContainer.end();
                                                            ++iter_ring2) {
            counter_for_the_rings++;
            debug(10) << "#boundingSpheres in ring " << counter_for_the_rings << ": " << (*iter_ring2)->size();
        }
#endif
#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
        int debug_ring_counter = 0;
        int debug_sphere_counter = 0;
#endif
        // for each ring of the RingContainer ...
        for (std::list< std::list<DeformableBvhNode*>* >::iterator ring = mBoundingSphereRingContainer.begin();
                                                                    ring != mBoundingSphereRingContainer.end();
                                                                    ++ring) {
#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
            debug_ring_counter++;
            debug(10) << "RING #" << debug_ring_counter << ":";
#endif
            // for each element of the current ring ...
            for (std::list<DeformableBvhNode*>::iterator element_ring = (*ring)->begin();
                                                            element_ring != (*ring)->end();
                                                            ++element_ring) {
#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
                debug_sphere_counter++;
                debug(10) << "SPHERE #" << debug_sphere_counter << ":";
#endif
                // get the triangles this sphere contains
                const std::list<Triangle*> enclosedTriangles = (*element_ring)->getBoundingVolume()->getEnclosedTriangles();
                // get all neighbours of these contained triangles without those, which are enclosed
                // (i.e. get the surrounding triangles of the sphere)
                std::set<Triangle*> trianglesSurroundingTheSphere;

                // for all enclosed triangles ...
                for (std::list<Triangle*>::const_iterator triangle = enclosedTriangles.begin();
                                                            triangle != enclosedTriangles.end();
                                                            ++triangle) {

                    // get all neighbours of this triangle (part_1 + part_2)
                    const std::set<Triangle*> part_1 = (*triangle)->getEdgeAdjacentTriangles();
                    // for all edge adjacent triangles ...
                    for (std::set<Triangle*>::const_iterator iter_part_1 = part_1.begin();
                                                        iter_part_1 != part_1.end();
                                                        ++iter_part_1) {
                        // check if they belong to the enclosed triangles ...
                        std::list<Triangle*>::const_iterator triangle_part_1_found = find_if(enclosedTriangles.begin(),
                                                                                                enclosedTriangles.end(),
                                                                                                bind2nd(std::equal_to<Triangle*>(), (*iter_part_1)));
                        // if NOT , put it into the trianglesSurroundingTheSphere set
                        if (triangle_part_1_found == enclosedTriangles.end()) {
                            trianglesSurroundingTheSphere.insert(*iter_part_1);
                        }
                    }

                    const std::set<Triangle*> part_2 = (*triangle)->getVertexAdjacentTriangles();
                    // for all vertex adjacent triangles ...
                    for (std::set<Triangle*>::const_iterator iter_part_2 = part_2.begin();
                                                        iter_part_2 != part_2.end();
                                                        ++iter_part_2) {
                        // check if they belong to the enclosed triangles ...
                        std::list<Triangle*>::const_iterator triangle_part_2_found = find_if(enclosedTriangles.begin(),
                                                                                                enclosedTriangles.end(),
                                                                                                bind2nd(std::equal_to<Triangle*>(), (*iter_part_2)));
                        // if NOT , put it into the trianglesSurroundingTheSphere set
                        if (triangle_part_2_found == enclosedTriangles.end()) {
                            trianglesSurroundingTheSphere.insert(*iter_part_2);
                        }
                    }

                }
#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
                debug(10) << "surroundingTriangles found! #" << trianglesSurroundingTheSphere.size();
#endif
                // get the deformableBvhNodes of the trianglesSurroundingTheSphere
                std::list<DeformableBvhNode*> AdjacentDeformableBvhNodes;
                DeformableBvhNode* currentBvhNode;
                for (std::set<Triangle*>::iterator surrounding_triangle = trianglesSurroundingTheSphere.begin();
                                                                surrounding_triangle != trianglesSurroundingTheSphere.end();
                                                                ++surrounding_triangle) {
                    // get the deformableBvhNode of the triangle
                    std::list<const BvhNode*> bvhNodes = (*surrounding_triangle)->getBvHierarchyNodes();
                    if(!bvhNodes.empty()) {
                        currentBvhNode = (DeformableBvhNode*) bvhNodes.back();
                    } else {
#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
                        debug(10) << "FATAL ERROR : list of bvhNodes is empty!";
                        break;
#endif
                    }
#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
                    const BoundingSphere* sphere = (const BoundingSphere*) (const DeformableBvhNode*) bvhNodes.back()->getBoundingVolume();
                    // doublecheck if the sphere is not NULL. Just to be sure.
                    if (sphere == 0) {
                        debug(10) << "FATAL ERROR : BoundingSphere is NULL!";
                        break;
                    }
#endif

                    std::list<DeformableBvhNode*>::iterator bvhNode_already_found = find_if(AdjacentDeformableBvhNodes.begin(),
                                                                                            AdjacentDeformableBvhNodes.end(),
                                                                                            bind2nd(std::equal_to<DeformableBvhNode*>(), currentBvhNode));

                    if (bvhNode_already_found == AdjacentDeformableBvhNodes.end()) {
                        AdjacentDeformableBvhNodes.push_front(currentBvhNode);
#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
                        debug(10) << "bvhNode added to list!";
#endif
                    }
                }

#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
                debug(10) << "#neighbours in list: " <<AdjacentDeformableBvhNodes.size();
#endif

                // AdjacentDeformableBvhNodes now contains the neighbours of the current element_ring an can be written into its neighbourlist
                (*element_ring)->setAdjacentBoundingSpheres(AdjacentDeformableBvhNodes);

                AdjacentDeformableBvhNodes.clear();


#ifdef SURFACEHIERARCHY_DEBUG_LOWEST_NEIGHBOURS
                const std::list<DeformableBvhNode*> test = (*element_ring)->getAdjacentDeformableBvhNodes();
                debug(10) << "neighbours correctly added: #" << test.size();
#endif

            }
        }

    }

//******************************************************************************************************************************
//******************************************************************************************************************************
//******************************************************************************************************************************
//******************************************************************************************************************************
// Creating Higher Level Of The Hierarchy **************************************************************************************
//******************************************************************************************************************************


    /*!
     * creates the second level of the hierarchy
     * note : as this is a bottom up approach the second level is counted from the triangles up
     * (level zero are the triangles themselves!)
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchyCreator::createHigherLevelOfHierarchy() {

        // debug(10) << "generating higher surface hierarchy level (surfacebound)...";
        //if (mBoundingSphereRingContainer.size() == 1) {
        //    debug(10) << "mBoundingSphereRingContainer.size() == 1";
        //    return;
        //}

        // this list of lists will contain only the new centers of level 2 as a list of rings
        // analogue to the mBoundingSphereRingContainer from level 1...
        std::list< std::list<DeformableBvhNode*>* > centerRingContainer = mBoundingSphereRingContainer;
        // ... all other spheres will be moved to the mRemainingBoundingSphereSet set
        mRemainingBoundingSphereSet.clear();
        // the created level 2 spheres will be put into mBoundingSphereRingContainer in the same list of rings organisation
        // like in centerRingContainer, so clear the list! (a copy is already in centerRingContainer)
        mBoundingSphereRingContainer.clear();


#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
        // print out the sizes of the centerRingContainer
        debug(10) << "#rings in centerRingContainer (before sorting out): " << centerRingContainer.size();
        int counter_for_the_rings = 0;
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_ring2 = centerRingContainer.begin();
                                                            iter_ring2 != centerRingContainer.end();
                                                            ++iter_ring2) {
            counter_for_the_rings++;
            debug(10) << "#boundingSpheres in ring " << counter_for_the_rings << ": " << (*iter_ring2)->size();
        }
#endif

        // sepparate the the new CenterSpheres in the CenterRingContainer
        // all other Spheres will be moved to mRemainingBoundingSphereSet
        separateCenterSpheres(centerRingContainer);


        std::list< std::list<DeformableBvhNode*>* >::iterator container_iter;
        // create the boundingSpheres of level 2 using centerRingContainer
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_center_ring = centerRingContainer.begin();
                                                            iter_center_ring != centerRingContainer.end();
                                                            ++iter_center_ring) {

            for (std::list<DeformableBvhNode*>::iterator iter_center = (*iter_center_ring)->begin();
                                                            iter_center != (*iter_center_ring)->end();
                                                            ++iter_center) {

                if (iter_center == (*iter_center_ring)->begin()) {
                    mBoundingSphereRingContainer.push_back(new std::list<DeformableBvhNode*>());
                    container_iter = --mBoundingSphereRingContainer.end();
                }

                std::list<DeformableBvhNode*> surroundingNonCenters;
                const std::set<DeformableBvhNode*> allSurroundingSpheres = (*iter_center)->getAdjacentDeformableBvhNodes();
                for (std::set<DeformableBvhNode*>::const_iterator surrounding_spheres_iter = allSurroundingSpheres.begin();
                                                                    surrounding_spheres_iter != allSurroundingSpheres.end();
                                                                    ++surrounding_spheres_iter) {

                    std::set<DeformableBvhNode*>::iterator remove_non_center_iter = mRemainingBoundingSphereSet.find(*surrounding_spheres_iter);
                    if (remove_non_center_iter != mRemainingBoundingSphereSet.end()) {
                        surroundingNonCenters.push_back(*surrounding_spheres_iter);
                        mRemainingBoundingSphereSet.erase(*surrounding_spheres_iter);
                    }
                }
                DeformableBvhNode* node = new DeformableBvhNode(mWorld, surroundingNonCenters, (*iter_center));
                node->initializeNode();
                (*container_iter)->push_front(node);
            }
        }

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
        debug(10) << "assigning remaining level 1 spheres to level 2 boundingspheres...";
#endif

        std::list<DeformableBvhNode*>* necessaryNewSpheresFromRemaining = new std::list<DeformableBvhNode*>;
        std::set<DeformableBvhNode*>::iterator remaining_sphere_iter;
        // assigne the remaining level 1 spheres in mRemainingBoundingSphereSet to a nearby level 2 spheres
        while (!mRemainingBoundingSphereSet.empty()) {
            remaining_sphere_iter = mRemainingBoundingSphereSet.begin();

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
            debug(10) << "Center of current remaining sphere: x: " << ((BoundingSphere*) (*remaining_sphere_iter)->getBoundingVolume())->getCenterVector().getX()
                        << "    y: " << ((BoundingSphere*) (*remaining_sphere_iter)->getBoundingVolume())->getCenterVector().getY()
                        << "    z: " << ((BoundingSphere*) (*remaining_sphere_iter)->getBoundingVolume())->getCenterVector().getZ();
#endif

            const std::set<DeformableBvhNode*> levelOneNeighbours = (*remaining_sphere_iter)->getAdjacentDeformableBvhNodes();
            std::set<DeformableBvhNode*> nearbyBiggerSpheres;
            for (std::set<DeformableBvhNode*>::const_iterator level_one_neighbour_iter = levelOneNeighbours.begin();
                                                                    level_one_neighbour_iter != levelOneNeighbours.end();
                                                                    ++level_one_neighbour_iter) {
                if ((*level_one_neighbour_iter)->getParent() != 0) {
                    nearbyBiggerSpheres.insert( (DeformableBvhNode*) (*level_one_neighbour_iter)->getParent() );
                }
            }

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
            debug(10) << "Size of nearbyBiggerSpheres: " << nearbyBiggerSpheres.size();
#endif

            if (!nearbyBiggerSpheres.empty()) {

                // keeps a pointer to the bigger sphere which shall be merged with the smaller sphere
                DeformableBvhNode* chosenBiggerSphere;
                // keeps the smallest possible radius of a bigger sphere AFTER merging with the smaller sphere
                real new_radius_big = std::numeric_limits<real>::infinity();
                // which nerby bigger sphere grows the smallest amount when merging with the current remaining sphere
                for (std::set<DeformableBvhNode*>::iterator biggerSphere = nearbyBiggerSpheres.begin();
                                                            biggerSphere != nearbyBiggerSpheres.end();
                                                            ++biggerSphere) {

                    const Vector3 center_big = ((BoundingSphere*)((*biggerSphere)->getBoundingVolume()))->getCenterVertex()->getPosition();
                    const Vector3 center_small = ((BoundingSphere*)(*remaining_sphere_iter)->getBoundingVolume())->getCenterVertex()->getPosition();
                    const real radius_big = ((BoundingSphere*) (*biggerSphere)->getBoundingVolume())->getRadius();
                    const real radius_small = ((BoundingSphere*) (*remaining_sphere_iter)->getBoundingVolume())->getRadius();
                    real temp_new_radius_big = (center_big - center_small).length() + radius_small;
                    if (temp_new_radius_big > radius_big) {
                        if (temp_new_radius_big < new_radius_big) {
                            new_radius_big = temp_new_radius_big;
                            chosenBiggerSphere = (*biggerSphere);
                        }
                    } else {
                        if (radius_big < new_radius_big) {
                            new_radius_big = radius_big;
                            chosenBiggerSphere = (*biggerSphere);
                        }
                    }
                }

                // merge small sphere with the sphere in chosenBiggerSphere
                chosenBiggerSphere->addChildDeformableBvhNode(*remaining_sphere_iter);
                mRemainingBoundingSphereSet.erase(remaining_sphere_iter);
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
                debug(10) << "-> level 1 sphere assigned to level 2 sphere";
#endif

            } else {
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
                debug(10) << "-> nearbyBiggerSpheres is empty!-> creating new higher level sphere!";
#endif
                std::list<DeformableBvhNode*> surroundingNonCenters;
                const std::set<DeformableBvhNode*> allSurroundingSpheres = (*remaining_sphere_iter)->getAdjacentDeformableBvhNodes();

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
                if (allSurroundingSpheres.empty()) {
                    debug(10) << "-> has no surrounding spheres!";
                }
#endif
                for (std::set<DeformableBvhNode*>::const_iterator surrounding_spheres_iter = allSurroundingSpheres.begin();
                                                                    surrounding_spheres_iter != allSurroundingSpheres.end();
                                                                    ++surrounding_spheres_iter) {

                    std::set<DeformableBvhNode*>::iterator remove_non_center_iter = mRemainingBoundingSphereSet.find(*surrounding_spheres_iter);
                    if (remove_non_center_iter != mRemainingBoundingSphereSet.end()) {
                        surroundingNonCenters.push_back(*surrounding_spheres_iter);
                        mRemainingBoundingSphereSet.erase(*surrounding_spheres_iter);
                    }
                }
                // create own higher bigger sphere for the small sphere, because all neighbours also have no bigger sphere
                DeformableBvhNode* node = new DeformableBvhNode(mWorld, surroundingNonCenters, (*(mRemainingBoundingSphereSet.begin())));
                node->initializeNode();
                necessaryNewSpheresFromRemaining->push_front(node);
                mRemainingBoundingSphereSet.erase(mRemainingBoundingSphereSet.begin());
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
                debug(10) << "---> new level 2 sphere created for remaining sphere...";
#endif
            }
        }

        if (!(necessaryNewSpheresFromRemaining->empty())) {
            mBoundingSphereRingContainer.push_back(necessaryNewSpheresFromRemaining);
        } else {
            delete(necessaryNewSpheresFromRemaining);
        }

#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
        // print out the sizes of the mBoundingSphereRingContainer
        debug(10) << "#rings in mBoundingSphereRingContainer: " << mBoundingSphereRingContainer.size();
        int counter_cia = 0;
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_ring2 = mBoundingSphereRingContainer.begin();
                                                            iter_ring2 != mBoundingSphereRingContainer.end();
                                                            ++iter_ring2) {
            counter_cia++;
            debug(10) << "#boundingSpheres in ring " << counter_cia << ": " << (*iter_ring2)->size();
            debug(10) << "  #items in ring " << counter_cia << ":";
            for (std::list<DeformableBvhNode*>::iterator bnd = (*iter_ring2)->begin();
                                                            bnd != (*iter_ring2)->end();
                                                            ++bnd) {
                debug(10) << "  contained level 1 spheres in item: " << (*bnd)->getChildren().size();
            }

        }

        debug(10) << "least level 1 sphere: x: " << ((BoundingSphere*) mBoundingSphereRingContainer.back()->back()->getBoundingVolume())->getCenterVector().getX()
                    << "   y: " << ((BoundingSphere*) mBoundingSphereRingContainer.back()->back()->getBoundingVolume())->getCenterVector().getY()
                    << "   z: " << ((BoundingSphere*) mBoundingSphereRingContainer.back()->back()->getBoundingVolume())->getCenterVector().getZ();

        debug(10) << "has children: #" << ((mBoundingSphereRingContainer.back()->back())->getChildren()).size();
#endif

    }

    /*!
     * Helper method for \ref createHigherLevelOfHierarchy()
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchyCreator::separateCenterSpheres(std::list< std::list<DeformableBvhNode*>* >& centerRingContainer) {

        std::list< std::list<DeformableBvhNode*>* >::iterator ring_iterator;
        std::list<DeformableBvhNode*>::iterator sphere_iterator;


        int centerRingContainer_size = centerRingContainer.size();

        ring_iterator = centerRingContainer.begin();
        sphere_iterator = (*ring_iterator)->begin();


        // find all centers in centerRingContainer and move all other spheres into the centerRingContainer set
        while (ring_iterator != centerRingContainer.end()) {
            sphere_iterator = (*ring_iterator)->begin();
            int ring_size = (*ring_iterator)->size();


            while (sphere_iterator != (*ring_iterator)->end()) {

                if (ring_size >= 3) {

                    // erase, overleap, erase
                    mRemainingBoundingSphereSet.insert(*sphere_iterator);
                    std::list<DeformableBvhNode*>::iterator temp_erase = (*ring_iterator)->erase(sphere_iterator);
                    sphere_iterator = temp_erase;
                    ++sphere_iterator;
                    mRemainingBoundingSphereSet.insert(*sphere_iterator);
                    temp_erase = (*ring_iterator)->erase(sphere_iterator);
                    sphere_iterator = temp_erase;
                    ring_size = ring_size - 3;

                } else if (ring_size == 2) {

                    // overleap, erase
                    ++sphere_iterator;
                    mRemainingBoundingSphereSet.insert(*sphere_iterator);
                    std::list<DeformableBvhNode*>::iterator temp_erase = (*ring_iterator)->erase(sphere_iterator);
                    sphere_iterator = temp_erase;
                    ring_size = ring_size - 2;

                } else if (ring_size == 1) {

                    // overleap
                    ++sphere_iterator;
                    ring_size = ring_size - 1;
                }
            }


            centerRingContainer_size--;

            if (centerRingContainer_size >= 4) {
                ++ring_iterator;
                moveFromListToSet(*ring_iterator);
                (*ring_iterator)->clear();
                centerRingContainer_size--;
                ++ring_iterator;
                moveFromListToSet(*ring_iterator);
                (*ring_iterator)->clear();
                centerRingContainer_size--;
                ++ring_iterator;
                sphere_iterator = (*ring_iterator)->begin();
            } else if (centerRingContainer_size == 3) {
                ++ring_iterator;
                moveFromListToSet(*ring_iterator);
                (*ring_iterator)->clear();
                centerRingContainer_size--;
                ++ring_iterator;
                sphere_iterator = (*ring_iterator)->begin();
            } else if (centerRingContainer_size == 2) {
                ++ring_iterator;
                sphere_iterator = (*ring_iterator)->begin();
            } else if (centerRingContainer_size == 1) {
                ++ring_iterator;
                moveFromListToSet(*ring_iterator);
                (*ring_iterator)->clear();
                centerRingContainer_size--;
                ++ring_iterator;
            } else if (centerRingContainer_size == 0) {
                ++ring_iterator;
            }

        }

    }

    /*!
     * just moves all elements of a list into a set
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchyCreator::moveFromListToSet(std::list<DeformableBvhNode*>* bvhList) {

        if (bvhList->empty()) {
            return;
        }

        int i = 0;

        for (std::list<DeformableBvhNode*>::iterator iter_bvh = bvhList->begin();
                                                        iter_bvh != bvhList->end();
                                                        ++iter_bvh) {
            i++;
            mRemainingBoundingSphereSet.insert(*iter_bvh);
        }
#ifdef SURFACEHIERARCHY_DEBUG_LEVEL_TWO
        debug(10) << "moveFromListToSet: " << i << " bvhNodes moved!";
#endif
    }



    /*!
     * fills the list of each DeformableBvhNode, which contains the neighbours of the sphere belonging to this Node
     * ...
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    // This Function initializes the neighbours of the boundingspheres in the lowest level.
    // To do this it uses the mBoundingSphereRingContainer which contains all rings ordered from first to last
    // and each ring contains its BoundingSpheres in a closed order, or at least in ordered in parts,
    // if there were gaps or spikes sticking out in the model.
    void SurfaceHierarchyCreator::initializeNeighboursOfHigherBoundingSphereLevel() {

        // debug(10) << "initializing higher level neighbourships...";


#ifdef SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS
        // print out the sizes of the mBoundingSphereRingContainer
        debug(10) << "#rings in mBoundingSphereRingContainer: " << mBoundingSphereRingContainer.size();
        int counter_for_the_rings = 0;
        for (std::list< std::list<DeformableBvhNode*>* >::iterator iter_ring2 = mBoundingSphereRingContainer.begin();
                                                            iter_ring2 != mBoundingSphereRingContainer.end();
                                                            ++iter_ring2) {
            counter_for_the_rings++;
            debug(10) << "#boundingSpheres in ring " << counter_for_the_rings << ": " << (*iter_ring2)->size();
        }
#endif
#ifdef SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS
        int debug_ring_counter = 0;
        int debug_sphere_counter = 0;
        debug(10) << "mBoundingSphereRingContainer.size(): " << mBoundingSphereRingContainer.size();
#endif

        // for each ring of the RingContainer ...
        for (std::list< std::list<DeformableBvhNode*>* >::iterator ring = mBoundingSphereRingContainer.begin();
                                                                    ring != mBoundingSphereRingContainer.end();
                                                                    ++ring) {
#ifdef SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS
            debug_ring_counter++;
            debug(10) << "RING #" << debug_ring_counter << ":" << "  ;   ring size: " << (*ring)->size();
#endif
            // for each element of the current ring ...
            for (std::list<DeformableBvhNode*>::iterator element_ring = (*ring)->begin();
                                                            element_ring != (*ring)->end();
                                                            ++element_ring) {
#ifdef SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS
                debug_sphere_counter++;
                debug(10) << "SPHERE #" << debug_sphere_counter << ":" << "  ;   contained spheres: " << (*element_ring)->getChildren().size();
#endif
                // get the lower spheres this sphere contains
                const std::list<BvhNode*> enclosedLowerSpheres = (*element_ring)->getChildren();
                // get all neighbours of these contained spheres without those, which are enclosed.
                // (i.e. get the surrounding lower level spheres of this sphere)
                std::set<DeformableBvhNode*> lowerSpheresSurroundingTheSphere;

                // for all enclosed lower_spheres ...
                for (std::list<BvhNode*>::const_iterator lower_sphere = enclosedLowerSpheres.begin();
                                                            lower_sphere != enclosedLowerSpheres.end();
                                                            ++lower_sphere) {

                    // get all neighbours of this lower_sphere
                    const std::set<DeformableBvhNode*> lowerSphereNeighbours = ((DeformableBvhNode*)(*lower_sphere))->getAdjacentDeformableBvhNodes();
                    // for all of these neighbour_spheres...
                    for (std::set<DeformableBvhNode*>::const_iterator neighbour_sphere = lowerSphereNeighbours.begin();
                                                                        neighbour_sphere != lowerSphereNeighbours.end();
                                                                        ++neighbour_sphere) {
                        // check if they belong to the enclosed lower spheres...
                        std::list<BvhNode*>::const_iterator lower_sphere_found = find_if(enclosedLowerSpheres.begin(),
                                                                                            enclosedLowerSpheres.end(),
                                                                                            bind2nd(std::equal_to<BvhNode*>(), (*neighbour_sphere)));
                        // if NOT , put it into the lowerSpheresSurroundingTheSphere set
                        if (lower_sphere_found == enclosedLowerSpheres.end()) {
                            lowerSpheresSurroundingTheSphere.insert(*neighbour_sphere);
                        }
                    }
                }

#ifdef SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS
                debug(10) << "surrounding spheres found! #" << lowerSpheresSurroundingTheSphere.size();
#endif
                // now get the higher level spheres of the lower spheres in lowerSpheresSurroundingTheSphere
                std::list<DeformableBvhNode*> adjacentHighDeformableBvhNodes;

                for (std::set<DeformableBvhNode*>::iterator surrounding_sphere = lowerSpheresSurroundingTheSphere.begin();
                                                                surrounding_sphere != lowerSpheresSurroundingTheSphere.end();
                                                                ++surrounding_sphere) {
                    // get the parent of the sphere
                    DeformableBvhNode* parentSphere = ((DeformableBvhNode*)((*surrounding_sphere)->getParent()));

                    if(parentSphere == 0) {

                        // this means that not all lower spheres got assigned to a higher sphere!
                        debug(10) << "FATAL ERROR : PARENT IS ZERO! (not all lower spheres have been assigned to higher ones)";
                        break;

                    }

                    // check if the higher sphere already is in adjacentHighDeformableBvhNodes
                    std::list<DeformableBvhNode*>::iterator high_sphere_already_found = find_if(adjacentHighDeformableBvhNodes.begin(),
                                                                                                adjacentHighDeformableBvhNodes.end(),
                                                                                                bind2nd(std::equal_to<DeformableBvhNode*>(), parentSphere));

                    if (high_sphere_already_found == adjacentHighDeformableBvhNodes.end()) {
                        adjacentHighDeformableBvhNodes.push_front(parentSphere);
#ifdef SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS
                        debug(10) << "bvhNode added to list!";
#endif
                    }
                }

#ifdef SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS
                debug(10) << "#neighbours in list: " <<adjacentHighDeformableBvhNodes.size();
#endif

                // adjacentHighDeformableBvhNodes now contains the neighbours of the current element_ring an can be written into its neighbourlist
                (*element_ring)->setAdjacentBoundingSpheres(adjacentHighDeformableBvhNodes);

                adjacentHighDeformableBvhNodes.clear();


#ifdef SURFACEHIERARCHY_DEBUG_HIGHER_NEIGHBOURS
                // inline const std::list<DeformableBvhNode*>& getAdjacentDeformableBvhNodes() const;
                const std::list<DeformableBvhNode*> test = (*element_ring)->getAdjacentDeformableBvhNodes();
                debug(10) << "neighbours correctly added: #" << test.size();
#endif

            }
        }

    }

//******************************************************************************************************************************
} // end class

/*
 * vim: et sw=4 ts=4
 */
