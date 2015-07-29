/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,          *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz            *
 *                                                                             *
 *  All rights reserved.                                                       *
 *                                                                             *
 *  Redistribution and use in source and binary forms, with or without         *
 *  modification, are permitted provided that the following conditions are met:*
 *   - Redistributions of source code must retain the above copyright          *
       notice, this list of conditions and the following disclaimer.           *
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



#ifndef DCOLLIDE_INTERSECTIONHELPERS_H
#define DCOLLIDE_INTERSECTIONHELPERS_H

#include <list>
#include <vector>

#include "real.h"

namespace dcollide {

    //Forward declarations
    class Matrix;
    class Vertex;
    class Plane;
    class Vector3;

    class IntersectionHelpers {
        public:
            IntersectionHelpers();
            ~IntersectionHelpers();

            int getAccuracyLevel(real radius) const;
            void calculateCirclePoints(real radius1, real distance,
                real radius2,const Matrix& state,Vector3* pointsBottom,
                Vector3* pointsTop, const int accuracy_level) const;
            bool isInsideTheBox(const Plane* planes,
                    const Vector3& point) const;
            bool isInsideOrOnTheBox(const Plane* planes,
                    const Vector3& point, real (&distances)[6]) const;
            bool isInsideTheWedge(const Plane* planes,
                    const Vector3& point) const;
            bool isInsideOrOnTheWedge(const Plane* planes,
                    const Vector3& point, real (&distances)[5]) const;
            std::pair<real,int> calculatePenetrationDepth(const Plane* planes,
                    int sizeOfPlanes, const Vector3& intersection) const;
            std::pair<real,int> calculatePenetrationDepth(const real* distances,
                    int sizeOfDistances) const;
            std::pair<real,int> calculatePenetrationDepth(const Plane* planes,
                    const real* distances,int sizeOfPlanes,
                    const Vector3& centerDiff) const;
    };
}

#endif // DCOLLIDE_INTERSECTIONHELPERS_H
/*
 * vim: et sw=4 ts=4
 */
