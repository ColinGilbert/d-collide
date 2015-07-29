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

#ifndef DCOLLIDE_POTENTIALCOLLIDINGSETS_H
#define DCOLLIDE_POTENTIALCOLLIDINGSETS_H

#include <vector>

namespace dcollide {
    class Triangle;
    class Proxy;

    /*!
     * \brief A struct that contains two lists of triangles, which were ascertained to collide potentially during the middlephase
     *
     * Definition: Two sets are said to be potential colliding, if there exists at least
     * one polygon in one set, which is potential colliding with at least one polygon of
     * the other set. \n
     * This structure is needed to report potential collisions to the narrowphase.
     * "Bounding Volume" - based algorithms might use it to report the triangles
     * inside the leaf nodes which were found to be colliding in the traversal of the 
     * hierarchy. Analog to that regular space partioning methods may use this struct
     * to report the triangles which are known to share the same space cell. \n
     * It is also necessary to provide information to which proxies the triangles
     * belong to. It is not possible to report collisions among more than two proxies,
     * to do this another struct needs to be created.
     */
    struct PotentialCollidingSets {
        inline PotentialCollidingSets();
        inline PotentialCollidingSets(const std::vector<Triangle*>& setOne, Proxy* proxy1,
                                      const std::vector<Triangle*>& setTwo, Proxy* proxy2);
        Proxy* proxy1;
        std::vector<Triangle*> setOne;
        Proxy* proxy2;
        std::vector<Triangle*> setTwo;
    };

    inline PotentialCollidingSets::PotentialCollidingSets() {
    }

    inline PotentialCollidingSets::PotentialCollidingSets(const std::vector<Triangle*>& setOne,
                                                          Proxy* proxy1,
                                                          const std::vector<Triangle*>& setTwo, 
                                                           Proxy* proxy2) 
                                                            : proxy1(proxy1), setOne(setOne),
                                                              proxy2(proxy2), setTwo(setTwo) {
    }

}

#endif
/*
 * vim: et sw=4 ts=4
 */
