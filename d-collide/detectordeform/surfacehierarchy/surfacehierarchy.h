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

#ifndef DCOLLIDE_SURFACE_HIERARCHY_H
#define DCOLLIDE_SURFACE_HIERARCHY_H

#include "shapes/shape.h"

#include <list>
#include <vector>
#include <set>

namespace dcollide {

    class World;
    class Proxy;
    class Triangle;
    class Vertex;
    class DeformableBvhNode;
    class SurfaceHierarchyAlgorithm;

    /*!
     * \brief Helper class to \ref SurfaceHierarchy
     *
     * This class is used to create an actual bounding sphere hierarchy on \ref
     * Proxy objects (bottom up). It is used by \ref SurfaceHierarchy only
     * and should not be used elsewhere.
     *
     * The basic idea is to create a \ref BoundingSphere hierarchy (see \ref
     * DeformableBvhNode) bound to the surface of the \ref Mesh in the following way
     * \li (1) Start with a vertex, find all triangle connected to it
     *         and create a \ref BoundingSphere around these (center = vertex).
     * \li (2) Calculate all triangles adjacent to the previously handled
     *         triangle(s). These are called the "surroundingTriangles".
     *     (3) Find all vertices of these triangles which do not belong to any
     *         triangle which has allready been processed.
     * \li (4) Create \ref BoundingSphere objects around these vertices avoiding
     *         too big overlaps. Note that every triangle must be in at least one \ref
     *         BoundingSphere object, but a single \ref BoundingSphere can
     *         contain many triangles (i.e. usually there are less \ref
     *         BoundingSphere objects created than potential vertices were found).
     * \li (5) repeat steps (2)-(3) until all triangles have been assigned to a
     *         \ref BoundingSphere.
     *         This completes the first "level" or "layer"
     * \li (6) repeat steps (1)-(4), but instead of forming spheres around the
     *         triangles of the mesh, form spheres around the spheres that were
     *         created in the previous layer.
     *         Repeat until a single (toplevel) sphere can be created, which is
     *         the top "layer" or "level" and thus the root node of the tree.
     *
     * As you can see every layer is built up by first creating an initial
     * sphere and then forming a "ring" of spheres around the first sphere. Once
     * the first ring has been completed, another "ring" of spheres is created
     * around the previous ring, until no more rings can be created.
     *
     * Since the deformations of the mesh must be limited (in particular: two
     * triangles must not move farther than a certain epsilon away from each
     * other) the update phase that is required on deformations is very simple:
     * \li Update the center points of all spheres
     * That's all. The spheres can initially be created to cover the epsilon
     * that the triangles it covers could enlarge in worst case, so no radius
     * recalculation needs to be performed.
     *
     * Collision detection can now be done using a usual
     * BoundingVolumeHierarchy-tree test.
     *
     * notice : the hierarchy achieves more homogenous results at models
     *          with a homogenous (size of triangles) mesh!
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    class SurfaceHierarchyCreator {
        public:
            SurfaceHierarchyCreator(SurfaceHierarchyAlgorithm* algorithm, World* world, Proxy* proxy);
            ~SurfaceHierarchyCreator();

            void generateSurfaceHierarchy();

        protected:
            //creating the lowest level of the hierarchy
            bool initializeFirstRing();
            std::list<Triangle*> calculateSurroundingTriangles();
            std::list<Vertex*> calculatePotentialNewCenters(std::list<Triangle*>& surroundingTriangles);
            std::list<Vertex*> sortWorkingSet(std::list<Vertex*>& workingSet);
            void createSpheresAtPotentialCenters(const std::list<Vertex*>& currentWorkingSetOfVertices);
            void findAndProcessGaps(std::list<Triangle*>& surroundingTriangles);
            void initializeNeighboursOfBoundingSpheresInLevelOne();

            //creating a higher level of the hierarchy
            void createHigherLevelOfHierarchy();
            void separateCenterSpheres(std::list< std::list<DeformableBvhNode*>* >& centerRingContainer);
            void initializeNeighboursOfHigherBoundingSphereLevel();
            void moveFromListToSet(std::list<DeformableBvhNode*>* bvhList);

            //DEBUG
            void GapDummy(std::list<Triangle*>& surroundingTriangles);
            void printOutVertexList(std::list<Vertex*>& liste);
            void printOutVertexVector(std::vector<Vertex*> vector);
            void printOutVertex(Vertex* vertex);


        private:
            SurfaceHierarchyAlgorithm* mAlgorithm;
            World* mWorld;

            Proxy* mProxy;

            //takes the spheres of level 2, which will become no center
            //used for ensuring that every level 1 sphere gets assigned to a level 2 sphere
            std::set<DeformableBvhNode*> mRemainingBoundingSphereSet;

            //container for the rings of created by each step of the algorithm for lowest level
            std::list< std::list<DeformableBvhNode*>* > mBoundingSphereRingContainer;

            //container for one created ring of BoundingSpheres!
            std::list<DeformableBvhNode*>* mCreatedBoundingSphereRing;

            //These are the triangles the allready done region consists of
            //von Kugeln umschlossene Dreiecke
            std::set<Triangle*> mTotallyDoneTrianglesSet;

            //These vertices lie COMPLETEY in the INNER of the allready done region and don't have to be touched again
            //Punkt edie komplett in einer Sphere liegen
            std::set<Vertex*> mTotallyDoneVerticesSet;

            //These vertices represent the border of the allready done region
            //Punkte auf der "Grenze" der abgearbeiten Region
            std::list<Vertex*> mCurrentBorderOfDoneRegion;

            //mNewBorder is used for fonding the new border while creating the spheres
            std::list<Vertex*> mNewBorder;

    };

}
/*
 * vim: et sw=4 ts=4
 */
#endif // DCOLLIDE_SURFACE_HIERARCHY_H
