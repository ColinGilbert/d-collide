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

#include "normalcone.h"

namespace dcollide {

    /*!
     * This adds another normal cone to this normal cone and saves the result
     * in this object.
     *
     * For more information about the scheme of caculation, please refer to
     * \p NormalCone::operator+.
     */
    void NormalCone::operator+=(const NormalCone& nc) {
#ifndef NORMALCONE_EXACT
        // this is an approximation using an upper bound
        // approximate the new axis by the axis in the middle of the old axes
        mAxis += *nc.getAxis();
        mAxis.normalize();
        // calculate an upper bound for the angle
        mAngle = std::max(nc.getAngle(), mAngle);
        mAngle += mAxis.angle( *nc.getAxis() ) / 2;
#else
        // this calls the exact calculation if NORMALCONE_EXACT is defined
        NormalCone result = (*this) + nc;
        mAxis = *result.getAxis();
        mAngle = result.getAngle();
#endif
    }

//TODO: the calculation of the new normal cone results in an unneccessary growth
// especially if one normal cone is completely inside the other
    /*!
     * This adds the values of another normal cone to the values of this normal
     * cone and returns a new normal cone with the result of the calculation.
     *
     * To add two normal cones you must first calculate a new axis by adding the
     * former axis vectors together and normalizing the resulting vector.
     * As an approximation the new angle can be computed by the sum of the
     * larger angle and half of the angle between the axes.
     */
    NormalCone NormalCone::operator+(const NormalCone& nc) const {
        // define variables to store the results
        real angle;
        Vector3 axis;
        NormalCone result;

        // define min and max - Angle and Axis, assuming this is the bigger cone
        real maxAngle = mAngle;
        real minAngle = nc.getAngle();
        Vector3 maxAxis = mAxis;
        Vector3 minAxis = *nc.getAxis();
        // swap min and max - Angle and Axis if they are not correct
        if (maxAngle < minAngle) {
            maxAngle = minAngle;
            minAngle = mAngle;
            maxAxis = minAxis;
            minAxis = mAxis;
        }

        // calculate the angle between the axes
        real gapAngle = mAxis.angle(*nc.getAxis());

        // check if the bigger normal cone completely contains the smaller cone
        if (maxAngle >= gapAngle + minAngle) {
            // simply adopt the bigger normal cone
            angle = maxAngle;
            axis = maxAxis;
        } else {
#ifdef NORMALCONE_EXACT
            // calculate a new normal cones' angle enclosing both original cones
            angle = (minAngle + maxAngle + gapAngle) / 2;
            // use a presolved equation based on:
            // cos(angle - maxAngle) = axis . maxAxis / |axis|
            // with:    axis = maxAxis + x minAxis
            // determine "some" temporary values for "simplification"
            real cosg = cos(angle - maxAngle);
            real cosg_2 = cosg * cosg;
            real b_1 = maxAxis.getX();
            real b_2 = maxAxis.getY();
            real b_3 = maxAxis.getZ();
            real b_1_2 = b_1 * b_1;
            real b_2_2 = b_2 * b_2;
            real b_3_2 = b_3 * b_3;
            real b_1_3 = b_1_2 * b_1;
            real b_2_3 = b_2_2 * b_2;
            real b_3_3 = b_3_2 * b_3;
            real b_1_4 = b_1_3 * b_1;
            real b_2_4 = b_2_3 * b_2;
            real b_3_4 = b_3_3 * b_3;
            real d_1 = minAxis.getX();
            real d_2 = minAxis.getY();
            real d_3 = minAxis.getZ();
            real d_1_2 = d_1 * d_1;
            real d_2_2 = d_2 * d_2;
            real d_3_2 = d_3 * d_3;
            // calculate x
            real x = (cosg * sqrt(((-b_2_2 -b_1_2)*d_3_2 +
                                  (2*b_2*b_3*d_2 + 2*b_1*b_3*d_1)*d_3 +
                                  (-b_3_2 -b_1_2)*d_2_2 +
                                  2*b_1*b_2*d_1*d_2 +
                                  (-b_3_2 -b_2_2)*d_1_2)*cosg_2 +
                                 ((b_2_2 + b_1_2)*b_3_2 +
                                  b_2_4 + 2*b_1_2*b_2_2 + b_1_4)*d_3_2 +
                                 (((-2*b_2_3 -2*b_1_2*b_2)*b_3 -
                                   2*b_2*b_3_3)*d_2 +
                                  ((-2*b_1*b_2_2 - 2*b_1_3)*b_3 -
                                   2*b_1*b_3_3)*d_1)*d_3 +
                                 (b_3_4 + (b_2_2 + 2*b_1_2)*b_3_2 +
                                  b_1_2*b_2_2 + b_1_4)*d_2_2 +
                                 (-2*b_1*b_2*b_3_2 - 2*b_1*b_2_3 -
                                  2*b_1_3*b_2)*d_1*d_2 +
                                 (b_3_4 +(2*b_2_2 + b_1_2)*b_3_2 +
                                 b_2_4 + b_1_2*b_2_2)*d_1_2
                                ) +
                      (-b_3*d_3 - b_2*d_2 - b_1*d_1)*cosg_2 +
                      (b_3_3 + (b_2_2 + b_1_2)*b_3)*d_3 +
                      (b_2*b_3_2 + b_2_3 + b_1_2*b_2)*d_2 +
                      (b_1*b_3_2 + b_1*b_2_2 + b_1_3)*d_1) /
                     ((d_3_2 + d_2_2 + d_1_2)*cosg_2 - b_3_2*d_3_2 +
                     (-2*b_2*b_3*d_2 - 2*b_1*b_3*d_1)*d_3 -
                     b_2_2*d_2_2 -2*b_1*b_2*d_1*d_2 -
                     b_1_2*d_1_2);
            // calculate the new axis
            axis = maxAxis + (minAxis * x);
            axis.normalize();
#else
            // approximate the new axis by the one in the middle of the old axes
            axis = maxAxis + minAxis;
            axis.normalize();
            // calculate an upper bound for the angle
            angle = gapAngle/2 + maxAngle;
#endif
        }

        result.setAxis( axis );
        result.setAngle( angle );

        return result;
    }

}
