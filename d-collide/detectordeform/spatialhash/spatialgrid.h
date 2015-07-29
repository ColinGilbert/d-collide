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


#ifndef DCOLLIDE_SPATIALGRID_H
#define DCOLLIDE_SPATIALGRID_H

#include <set>
#include "narrowphase/triangleintersector.h"
#include "narrowphase/narrowphase.h"
#include "detectordeform/trianglepair.h"
#include "detectordeform/potentialcollidingsets.h"
#include "spatialhash.h"
#include "real.h"


namespace dcollide {
    class DebugLogEntry;

    /*!
     *\brief The SpatialGrid represents a three-dimensional uniform subdivision of space
     *An easy way to think of the SpatialGrid class, is to imagine a three dimensional coordinate
     *system. Think of squared paper in three dimensions. Space is partitionend into a uniform grid
     *of unit cubes. We refer to this unit cubes as cells.
     *
     *The idea of spatial hashing is to classify vertices and triangles against this grid. Classification
     *is the process of determining the cell, or the set of cells a certain triangle or vertex is inside of.
     *Objects covering the same cells will be mapped  into the same container. This is done by the
     *\ref SpatialHash class.
     *
     *The whole idea of the SpatialGrid is to provide a fast way to determine which objects exist in a certain
     *region of space. We may query "Which objects exist in cell (1,2,1)" for example.
     *
     *The spatial grid is a datastructure of dictionary type. Basically keys, in this case three dimensional coordinates
     *are mapped to the set of objects existing in this cell.
     *
     */
    class SpatialGrid {
        public:
        SpatialGrid(real unitLength, SpatialHash* hash);
        ~SpatialGrid();


        void insert(const Vertex* vertex, const Proxy* owner);
        void remove(const Vertex* vertex);
        void checkTriangle(const Triangle* t,
                           const Proxy* owner,
                           bool selfCollisions,
                           std::list<PotentialCollidingSets>& results);

        void reset();

        inline void setUnitLength(real unitLength);
        inline real getUnitLength();

        private:
        void transformToCellCoordinates(const Vector3& v, IntVector3& r);

        void reportToNarrowphase(const Proxy* proxy1,
                                     const Triangle* t,
                                     const Proxy* proxy2,
                                     const std::set<Triangle*>* triangles,
                                     std::list<PotentialCollidingSets>& results);

        SpatialHash* mSpatialHash;
        real mUnitLength;
    };



    /*!
     * \brief Sets the  unit lenght of the spatial grid
     * Usually this method is internal because the the algorithm adapts the
     * unit length to the average side lengths \ref SpatialHashJob::calculateAverageSideLength
     * of all triangles used for deformable collision detection.
     * \param real unitLength The new unit length.
     */
    inline void SpatialGrid::setUnitLength(real unitLength) {
        mUnitLength = unitLength;
    }

    /*!
     * \brief Returns the current unit length
     */
    inline real SpatialGrid::getUnitLength() {
        return mUnitLength;
    }

}
#endif // DCOLLIDE_SPATIALGRID_H
/*
 * vim: et sw=4 ts=4
 */
