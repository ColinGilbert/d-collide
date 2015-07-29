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


#ifndef DCOLLIDE_TRIANGLEINTERSECTOR_H
#define DCOLLIDE_TRIANGLEINTERSECTOR_H

#include "real.h"
#include "math/vector.h"

namespace dcollide {

    class Triangle;

    /*!
     *\brief information of how two triangles intersect each other
     * If two triangles are not coplanar, their intersection (if there is one)
     * is a (limited) line.
     * Coplanar triangles can itersect in a point, a line or a polygon with n
     * vertices, with 3 <= n <= 5.
     * \p collision   indicates wether there is a collision or not 
     *                (indepentend of coplanarity)
     * \p coplanar    indicates wether the triangles are coplanar
     * \p intersectionStart and \p intersectionEnd are the start-and endpoints
     *                 of the intersection-line. Valid only if coplanar is false
     * NOTICE: atm, we have no way to describe coplanar intersections more 
     *          exactly. If we really need this information, i will do something
     *          about that. (GJ)
     */
    struct TriangleIntersection{
        bool collision;
        bool coplanar;
        Vector3 intersectionStart;
        Vector3 intersectionEnd;
        real penetrationDepth;
    };

    class TriangleIntersector {
    public:
        void computeTriangleIntersection(const Triangle* t1,
                                                const Triangle* t2,
                                                TriangleIntersection* result);

    private:
        void computeIntervalsOnIntersectionLine(
                            const Vector3& trianglePoint0,
                            const Vector3& trianglePoint1,
                            const Vector3& trianglePoint2,
                            real proj0, real proj1, real proj2,
                            real planeDistance0,
                            real planeDistance1,
                            real planeDistance2,
                            real multDistance01, real multDistance02,
                            real* intervalStart, real* intervalEnd,
                            Vector3& lineSegmentStart, Vector3& lineSegmentEnd);

        void calculateTriangleIntersectionWithLine(    const Vector3& point0,
                        const Vector3& point1,
                        const Vector3& point2,
                        real proj0, real proj1, real proj2,
                        real planeDistance0,
                        real planeDistance1,
                        real planeDistance2,
                        real* intervalStart, real* intervalEnd,
                        Vector3& lineSegmentStart, Vector3& lineSegmentEnd);

        int computeCoplanarIntersection3d( const Vector3& p1,
                                const Vector3& q1,
                                const Vector3& r1,
                                const Vector3& normal1, 
                                const Vector3& p2,
                                const Vector3& q2,
                                const Vector3& r2
                                /*const Vector3& normal_2*/);

        int computeIntersection2dCcw(    real p1[2], real q1[2], real r1[2], 
                                            real p2[2], real q2[2], real r2[2]);

        int checkOverlap2d(real p1[2], real q1[2], real r1[2], 
                                    real p2[2], real q2[2], real r2[2]);
    };


}

#endif // DCOLLIDE_TRIANGLEINTERSECTOR_H
/*
 * vim: et sw=4 ts=4
 */
