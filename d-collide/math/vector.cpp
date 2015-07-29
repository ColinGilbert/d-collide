/************************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:            *
 *              d-collide-users@lists.sourceforge.net                               *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,               *
 *     Martin Faßbach, Maximilian Hegele, Daniel Haus, Oliver Horst,                *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz                 *
 *  Copyright (C) 2002-2005 Rivo Laks (rivolaks@hot.ee)                             *
 *                                                                                  *
 *  All rights reserved.                                                            *
 *                                                                                  *
 *  Redistribution and use in source and binary forms, with or without              *
 *  modification, are permitted provided that the following conditions are met:     *
 *   - Redistributions of source code must retain the above copyright               *
 *     notice, this list of conditions and the following disclaimer.                *
 *   - Redistributions in binary form must reproduce the above copyright            *
 *     notice, this list of conditions and the following disclaimer in the          *
 *     documentation and/or other materials provided with the distribution.         *
 *   - Neither the name of the PG510 nor the names of its contributors may be       *
 *     used to endorse or promote products derived from this software without       *
 *     specific prior written permission.                                           *
 *                                                                                  *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS             *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT               *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR           *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER      *
 *  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,        *
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,             *
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR              *
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF          *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING            *
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS              *
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE                     *
 ************************************************************************************/

/*
 * This file has been shamelessy stolen from the Boson project, see
 *   http://boson.eu.org
 *
 * Permission to relicense to the modified BSD license (see header above)
 * granted by both authors:
 * Rivo Laks (rivolaks@hot.ee)
 * Andreas Beckermann (b_mann@gmx.de)
 */

#include "vector.h"

#include <math.h>

namespace dcollide {

    /*!
     * \return The length (aka magnitude) of the vector.
     *
     * The length of a vector v is defined as sqrt(v[0]^2 + v[1]^2 + v[2]^2) (in
     * case of 3d).
     *
     * Notice that this function actually uses sqrt(), so it is slow in
     * situations where you use it often!
     **/
    real Vector3::length() const {
        return sqrt(dotProduct());
    }

    /*!
     * \return The "not-sqrt-ed-length" (aka magnitude) of the vector.
     *
     */
    real Vector3::lengthWithoutSqrt() const {
        return dotProduct();
    }


    /*!
     * \return The angle between two vectors.
     * 
     * The angle between two vectors is given through
     * acos( dotProduct(v) / (length() * v.length) ),
     * whereby v1 and v2 are vectors.
     */
    real Vector3::angle(const Vector3& v) const {
        return acos( dotProduct(v) / (length() * v.length()) );
    }
    
    
    /*!
     * \overload
     *
     * Same as above, except that it takes 2 separate real arrays. You can use
     * this static method without a Vector3 instance - useful for comparing
     * Lib3dsVectors.
     **/
    bool Vector3::isEqual(const real* v1, const real* v2, real diff) {
        // avoid fabsf() as we don't include math.h
        real d1 = v1[0] - v2[0];
        real d2 = v1[1] - v2[1];
        real d3 = v1[2] - v2[2];
        if (d1 < 0.0f) {
          d1 = -d1;
        }
        if (d2 < 0.0f) {
            d2 = -d2;
        }
        if (d3 < 0.0f) {
            d3 = -d3;
        }
        if (d1 > diff) {
            return false;
        }
        if (d2 > diff) {
            return false;
        }
        if (d3 > diff) {
            return false;
        }
        return true;
    }


    /*!
     * The cross product of v and w is a vector that is perpendicular to the
     * surface that is perpendicular to the surface made up by v and w.
     * \return The cross product of v and w.
     **/
    Vector3 Vector3::crossProduct(const Vector3& v, const Vector3& w) {
        Vector3 r;
        r.setX((v.getY() * w.getZ()) - (v.getZ() * w.getY()));
        r.setY((v.getZ() * w.getX()) - (v.getX() * w.getZ()));
        r.setZ((v.getX() * w.getY()) - (v.getY() * w.getX()));
        return r;
    }


    /*!
     *\return distance to plane defined by point+normal vector
     * see http://mo.mathematik.uni-stuttgart.de/kurse/kurs8/seite40.html
     */
    real Vector3::distanceToPlane(  const Vector3& testPoint,
                                    const Vector3& planePoint, 
                                    const Vector3& planeNormal) {
        return fabs(dotProduct(testPoint - planePoint, planeNormal))
                    /(planeNormal.length());
    }

    /*!
     *\brief approximates length by using a fast sqrt approximation
     * Algorithm uses the Babylonian Squareroot Approximation as described here:
     * http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Babylonian_method
     * The algorithm has quadratic convergence.

     * uses initial value for length = sum of X,Y and Z (<50% initial error)
     *\param steps the number of iterations for the loop. More iterations result in higher accuracy.
                   Should be between 0 and 3.
     * GJ: Performance results for my machine (AMD Sempron single-core):
     *     Worst case test (a vector with X=Y=Z => worst initial value),
     *     myVector.length == 17.3205 , 2500
     *     0 steps: ~20% of length time, error ~70% (too big)
     *     1 step : ~40% of length time, error ~16% (too big)
     *     2 steps: ~60% of length time, error ~1%
     *     3 steps: ~80% of length time, error <0.01%
     *     4 steps: ~same time as length time, same result 
     */
    real Vector3::lengthApproxBabylonian(int steps) const {
        //special case:: vector is nullvector - return 0
        if (isNull()) {
            return 0;
        }
        //the value we want to compute the sqrt of
        real value = mData[0]*mData[0] + mData[1]*mData[1] + mData[2]*mData[2];
        //initialize length with "taxicab geometry"
        real result = fabs(mData[0]) + fabs(mData[1]) + fabs(mData[2]); 
        for (int i = 0; i<steps; ++i) {
            result = (real) (0.5 * (result + value/result));
        }
    
        return result;
    }


    std::ostream& operator<<(std::ostream& os, const dcollide::Vector3& v) {
        os << "(" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")";
        return os;
    }
}


/*
 * vim: et sw=4 ts=4
 */
