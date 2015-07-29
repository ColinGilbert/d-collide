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


#include "collisionpair.h"

#include <ostream>


namespace dcollide {
    
    CollisionPair::CollisionPair() {
    }
    CollisionPair::~CollisionPair() {
    }

    bool CollisionPair::operator<(const CollisionPair& cp) const {
        if (bvol1 < cp.bvol1) {
            return true;
        }
        if ((bvol1 == cp.bvol1) && (bvol2 < cp.bvol2)) {
            return true;
        }
        return false;
    }
    bool CollisionPair::operator==(const CollisionPair& cp) const {
        if (((bvol1 == cp.bvol1) && (bvol2 == cp.bvol2)) ||
            ((bvol1 == cp.bvol2) && (bvol2 == cp.bvol1))) {
            return true;
        }
        return false;
    }
    bool CollisionPair::operator<=(const CollisionPair& cp) const {
        if (std::min(bvol1,bvol2) < std::min(cp.bvol1,cp.bvol2)) {
            return true;
        }
        if ((std::min(bvol1,bvol2) == std::min(cp.bvol1,cp.bvol2))
            && (std::max(bvol1,bvol2) < std::max(cp.bvol1,cp.bvol2)))  {
            return true;
        }
        return false;
    }

    /*!
     * \brief ostream operator << for use with cppUnitTests
     * \returns the address of the 2 boundingVolumes saved in this collisionpair
     */
    std::ostream& operator<<(std::ostream& os, const dcollide::CollisionPair& cp) {
        os << "[" << cp.bvol1 << "," << cp.bvol2 << "]"; 
        return os;
    }
}

/*
 * vim: et sw=4 ts=4
 */
