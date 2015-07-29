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

#include "spatialhash.h"

namespace dcollide {
    /*!
     * \param numberOfCells The size of the array allocated by SimpleHash. The higher the number, the less hash collisions, we will suffer from
     */
    SimpleHash::SimpleHash(int numberOfCells) {
        mNumberOfCells = numberOfCells;
      
        mCells = new std::vector<SpatialHash::Entry>[numberOfCells];
    }
    
    SimpleHash::~SimpleHash()
    {
        delete [] mCells;
    }

    /*!
     * \brief Call this function at the end of each frame.
     * The SimpleHash keeps track of all used cells in a list. This function iterates
     * that list and deletes all entries of that each cell. 
     */
    void SimpleHash::reset() {
        //Delete all entries, which were used during this run
        for(std::vector<int>::iterator i = mUsedCells.begin(); i!= mUsedCells.end(); i++ ) {
            mCells[(*i)].clear();
        }

        mUsedCells.clear();
    }

    /*!
     * \brief This is the actual hash function as taken from the paper "Optimized Spatial Hashing for Collision Detection of Deformable Objects"
     */

    int SimpleHash::hashFunction(int x, int y ,int z) {
        int cell  =   ((x * mPrime1) % mNumberOfCells) 
                    + ((y * mPrime2) % mNumberOfCells) 
                    + ((z * mPrime3) % mNumberOfCells);
        cell = cell % mNumberOfCells;

        /* Modulo arithmetic for negative hash values; they are interpreted as
         * inverse elements. If k is element of a modulo-n group, you find its
         * inverse by calculating (n-k)
         * example
         * k + (n-k) mod n = n mod n = 0 mod n
         * which is the definition of an additive inverse: k+k^-1 = 0
         */
        if (cell < 0) {
            /* Care that "cell" is negative so the "+" is actually a subtraction
             * n is "mNumberofCells" and k is named "cell"
             */
            cell = mNumberOfCells + cell;
        }
        return cell;
    }

    /*!
     * \brief Do the actual hashing of an entry
     * \param cell The cell coordinate, which is the hash key
     * \param e The entry, which is the value inserted into the hash
     *
     * This is used in phase one of the algorithm
     */
    void SimpleHash::insert(IntVector3& cell, SpatialHash::Entry& e ) {
        int index = hashFunction(cell.getX(),cell.getY(),cell.getZ());

        mCells[ index ].push_back(e);
        mUsedCells.push_back( index );
    }

    void SimpleHash::removeVertexFromCell(IntVector3& cell, const Vertex* v) {
        int index = hashFunction(cell.getX(),cell.getY(),cell.getZ());
        
        for(std::vector<Entry>::iterator i = mCells[index].begin();
            i != mCells[index].end(); ++i) {
            if( (*i).v == v) {
                //This is the entry we want to remove
                mCells[index].erase(i);
                return;
            }
        }
        
    }
    /*!
     * \brief Retrieve all entries from cell with the given coordinate
     * \param cell Get entries from that cell
     */
    std::vector<SpatialHash::Entry>* SimpleHash::getEntriesFrom(IntVector3& cell ) {
        return &(mCells[ hashFunction(cell.getX(), cell.getY(), cell.getZ() ) ]);
    }
}

/*
 * vim: et sw=4 ts=4
 */
