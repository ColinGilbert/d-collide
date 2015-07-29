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

#include <iostream>
#include "math/vector.h"
#include "spatialgrid.h"

#include "boundingvolumes/aabb.h"
#include "shapes/mesh/triangle.h"

#include "debuglog.h"


namespace dcollide {
    /*!
     * \brief Constructor of SpatialGrid
     * \param real unitLength Set initial unit length to this value
     * \param SpatialHash* hash The SpatialGrid has a "pluggable" hashing function. See \ref dcollide::SpatialHash
     */
    SpatialGrid::SpatialGrid( real unitLength, SpatialHash* hash) {
        mUnitLength = unitLength;
        mSpatialHash = hash;
    }

    SpatialGrid::~SpatialGrid() {
        delete mSpatialHash;
    }

    /*!
     * \brief Call this method at the end of every frame to clear all vertices, which were added in phase one.
     */
    void SpatialGrid::reset() {
        mSpatialHash->reset();
    }

    /*!
     * \brief Classify a vertex against the spatial grid
     * This method is used during the update phase (phase one).
     * The vertex v is classified and added to the SpatialGrid. As a consequence it is available for collision detection in
     * phase two.
     * \param Vertex* v This vertex will be inserted to the grid
     * \param Proxy* p The mesh-proxy the vertex is part of. This information is needed to return information which proxies have collided
     */
    void SpatialGrid::insert(const Vertex* v, const Proxy* p) {
        IntVector3 cell;
        transformToCellCoordinates( v->getWorldPosition(), cell );

        SpatialHash::Entry e;

        e.triangles = &v->getAdjacentTriangles();
        e.p = p;
        e.v = v;
        e.spatialCell = cell;

        mSpatialHash->insert(cell, e);
    }

    void SpatialGrid::remove(const Vertex* vertex) {
        IntVector3 cell;
        transformToCellCoordinates( vertex->getWorldPosition(), cell );
        
        mSpatialHash->removeVertexFromCell(cell, vertex);
    }
    
    /*!
     * \brief This method does the actual collision detection
     * This method performs the actual collision detection.
     * \param Triangle* t The triangle which is checked for collision
     * This is done by
     * 1.)calculating the Aabb,
     * 2.)determining the set of cells covered by the Aabb.
     * 3.)each cell is queried for the vertices that were classified in phase one
     * 4.)The set of triangles is determined, which have one of the vertices as a point
     * We consider the Triangle t and this set as potential colliding and perform triangle
     * intersection on them.
     */
    void SpatialGrid::checkTriangle(const Triangle* t,
                                    const Proxy* owner,
                                    bool selfCollisions,
                                    std::list<PotentialCollidingSets>& results) {

        //Step one: Calculate the Aabb of the Triangle
        Aabb a;
        a.adjustToTriangle(t);

        IntVector3 tmin, tmax;

        //Step two: Find the set of spatial cells, which are covered by the Aabb
        //To achieve that goal, we determine the maximum and minimum cell coordinates
        //from the world coordinates of the Aabb

        transformToCellCoordinates(a.getMin(),tmin);
        transformToCellCoordinates(a.getMax(),tmax);

        int minX = tmin.getX();
        int maxX = tmax.getX();
        int minY = tmin.getY();
        int maxY = tmax.getY();
        int minZ = tmin.getZ();
        int maxZ = tmax.getZ();

        //Each triangle may potentially collide with triangles of n different
        //proxies, although rarely the case, we have to deal with this.
        std::map<Proxy*, std::set<Triangle*> > potentialColliders;
        
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) {
                    std::vector<SpatialHash::Entry>* colliders;
                    //Collect all vertices, inserted in Phase1, which are
                    //inside the region of the Aabb

                    IntVector3 requested_cell(x,y,z);
                    colliders = mSpatialHash->getEntriesFrom( requested_cell  );

                    for(std::vector<SpatialHash::Entry>::iterator i = colliders->begin();
                            i != colliders->end(); i++) {

                        //Check for HASH-Collisions. This is not related to spatial collisions.
                        if( !((*i).spatialCell == requested_cell)) {
                            // We have found a hash collision and discard this vertex
                            continue;
                        }

                        if (owner == (*i).p) {
                            //This is a self-collision
                            if (selfCollisions) {
                                //When self-collisions are enabled, we will also find collisions of
                                //the triangle with itself - of course we want to sort that out
                                if ( t->hasElement((*i).v) ) {
                                    //Yes we are checking the triangle against itself
                                    //->throw it away
                                    continue;
                                }
                                //Ignore all triangles, which are adjacent to the triangle t
                                for(std::list<Triangle*>::const_iterator j = (*i).triangles->begin();
                                    j !=  (*i).triangles->end(); ++j) {
                                        if(!(t->isVertexAdjacentTo(*j))) {
                                            potentialColliders[ const_cast<Proxy*>((*i).p) ].insert( *j );
                                        }
                                }
                            } else {
                                //We want to ignore self collisions
                                continue;
                            }
                        } else {
                            //This is not a a self collision, don't need to sort out adjacent triangles
                             potentialColliders[const_cast<Proxy*>((*i).p)].insert((*i).triangles->begin(), (*i).triangles->end());
                        }
                    }
                }
            }
        }

        for(std::map<Proxy*, std::set<Triangle*> >::iterator i = potentialColliders.begin(); 
            i != potentialColliders.end(); ++i ) {
            reportToNarrowphase(owner, t, i->first, &i->second, results);
        }
 
        potentialColliders.clear();
    }

    /*
     * \brief Generates TrianglePairs for triangle t and all triangles of the set and reports them as potential
     * colliding to the narrowphase
     *
     * See also \ref TriangleIntersector
     *
     * \param proxy1 The proxy that the \ref Triangle \p t belongs to
     * \param t The triangle that is checked for intersection with the triangles
     *        in the \p triangles list.
     * \param proxy2 The proxy that the triangles in the \p triangles list
     *        belong to.
     * \param triangles A list of triangles that are checked for (exact)
     *        intersection with \p t.
     */
    void SpatialGrid::reportToNarrowphase(
            const Proxy* proxy1, const Triangle* t,
            const Proxy* proxy2, const std::set<Triangle*>* triangles,
            std::list<PotentialCollidingSets>& results) {
            

            //There is only one triangle in set one
            std::vector<Triangle*> setOne;
            setOne.push_back(const_cast<Triangle*>(t));

            //Convert the set into a list - this is unfortunately
            //necessary because both don't share a common baseclass
            std::vector<Triangle*> setTwo;
            
            for(std::set<Triangle*>::const_iterator i = triangles->begin(); 
                i != triangles->end() ; ++i ) {
                setTwo.push_back(const_cast<Triangle*>(*i));
            }

            results.push_back(PotentialCollidingSets(setOne, 
                                                     const_cast<Proxy*>(proxy1), 
                                                     setTwo,
                                                     const_cast<Proxy*>(proxy2)));
        
    }

    
    /*!
     * \brief For a given Vector v this method will find the spatial cell v lies in, and writes the resulting coordinates in r
    * Cell coordinates have an integer value; and (x,y,z) means the xth spatial cell
    * in x direction, from the origin, the yth spatial cell in y direction and so forth
    */
    void SpatialGrid::transformToCellCoordinates(const Vector3& v, IntVector3& r) {
        real transformationFactor = (real)1.0 / mUnitLength;

        r.setX( (int)(v.getX() * transformationFactor));
        r.setY( (int)(v.getY() * transformationFactor));
        r.setZ( (int)(v.getZ() * transformationFactor));
    }
}

/*
 * vim: et sw=4 ts=4
 */
