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

#ifndef DCOLLIDE_SPATIAL_HASH_H
#define DCOLLIDE_SPATIAL_HASH_H

#include "debuglog.h"
#include <vector>
#include "shapes/mesh/vertex.h"
#include "proxy.h"
#include "intvector3.h"

namespace dcollide {
    /*!
     * \brief The SpatialHash defines an abstract interface to a dictionary data structure
     * This interface was written, while having in mind, that it may be advantageous to have the
     * opportunity to switch hashing algorithms easily. Indeed all data structures, which realize
     * an (x,y,z)->Entry mapping could be used. But it is sensible to concentrate on hash mappings,
     * because of their higher performance.
     *
     * It has turned out that this degree of freedom is not necessary, because the hash implementation
     * described in "Optimized Spatial Hashing for Collision Detection of Deformable Objects" performs
     * well enough. Other approaches such as perfect hashing could be implemented, but we found that
     * the number of hash collisions is acceptable for most cases.
     */
    class SpatialHash {
        public:
        /*!
         * \brief A spatial cell is associated with this struct via the hashing function
         */
        struct Entry {  
            //SpatialCell is the coordinate of the space-cell associated with
            //this Entry. This information is used to detect hash collisions.
            IntVector3 spatialCell;
           
            //The list of triangles witch are adjacent to vertex v  
            const std::list<Triangle*>* triangles;
            //The inserted vertex v is part of the proxy p. This information
            //is needed to determine later on, which proxies have collided
            const Proxy*  p;
            const Vertex* v;
        };

        virtual std::vector<Entry>* getEntriesFrom(IntVector3& cell) = 0;
        virtual void insert(IntVector3& cell, Entry& e) = 0;
        virtual void removeVertexFromCell(IntVector3& cell, const Vertex* v) = 0;
        virtual void reset() = 0;
        virtual ~SpatialHash() = 0 ;
    };

    /*!
     * \brief The SimpleHash is the only implementaion of the SpatialHash interface
     * We reproduced the hashing algorithm described in the reference paper. We even
     * use the same numbers.
     */
    class SimpleHash : public SpatialHash {
        public:
        SimpleHash( int numberOfCells );
        ~SimpleHash();
        
        std::vector<SpatialHash::Entry>* getEntriesFrom(IntVector3& cell);
        void insert(IntVector3& cell, Entry& e);
        void removeVertexFromCell(IntVector3& cell, const Vertex* v);
        void reset();

        private:
        int hashFunction(int x, int y ,int z);      
        int mNumberOfCells;
        //These three prime numbers are used by the hash function
        //They are taken from the paper "Optimized Spatial Hashing for 
        //Collision Detection of Deformable Objects"

        const static int mPrime1 = 73856093;
        const static int mPrime2 = 19349663;
        const static int mPrime3 = 83492791;

        //The one-dimensional array
        std::vector<SpatialHash::Entry>* mCells;
        //This vector tracks the set of used cells, so that they may be freed in reset()
        std::vector<int> mUsedCells;
    };


}
#endif

