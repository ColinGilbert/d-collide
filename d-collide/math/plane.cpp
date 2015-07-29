/************************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:            *
 *              d-collide-users@lists.sourceforge.net                               *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,               *
 *     Martin Faßbach, Maximilian Hegele, Daniel Haus, Oliver Horst,                *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz                 *
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
 * Andreas Beckermann (b_mann@gmx.de)
 * Rivo Laks (rivolaks@hot.ee)
 */

#include "plane.h"

#include <math.h>
#include <stdio.h>
#include <iostream>

namespace dcollide {
    /*!
     * Dump plane onto the console as debug output.
     */
    void Plane::debugPlane(const Vector3& normal, real distance) {
        printf("((%f %f %f) %f\n", normal[0], normal[1], normal[2], distance);
    }

    /*!
     * This calculates the intersection line of @p plane1 and @p
     * plane2.
     *
     * In 3d space, two planes are either equal (see @ref isEqual), parallel
     * but not equal, or they intersect in a line.
     *
     * The line is represented as a point and a direction vector. Every point on the
     * line can be calculated using
     * <code>
     * Vector3 point = *intersectionPoint + a * (*inersectionVector)
     * </code>
     * where a is any real number.
     *
     * Note that we do not have a dedicated class for points and therefore use the
     * vector class here for both, however it is important to note that the point is
     * really a <em>point</em> whereas the vector is actually a vector, i.e. a
     * direction.
     *
     *
     * @return TRUE if the two planes intersect. In this case @p
     * intersectionPoint contains one point of the intersection line and @p
     * intersectionVector contains the direction of that line. Otherwise
     * (the planes do not intersect) FALSE is returned - the planes are
     * parallel then. Note that the planes can still intersect, if they are
     * equal, see @ref isEqual.
     */
    bool Plane::intersectPlane(const Plane& plane1, const Plane& plane2, Vector3* intersectionPoint, Vector3* intersectionVector) {
        const Vector3& n1 = plane1.getNormal();
        const Vector3& n2 = plane2.getNormal();
        Vector3 cross = Vector3::crossProduct(n1, n2);
        if (cross.isEqual(0.0f, 0.0f, 0.0f)) {
            // AB: note that intersections are still possible, if planes are equal
            // -> see isEqual()
            return false;
        }

        *intersectionVector = cross;

        // AB: the cross product is already 50% of the work.
        // The cross product gives a vector that is perpendicular to both, n1 and n2.
        // -> cross is perpendicular to n1, meaning it is on plane1.
        // -> cross is perpendicular to n2, meaning it is on plane2.
        // => cross is a vector that describes the direction of a line that is on both,
        //    plane1 and plane2. it is that line that we want.
        //    Now all we need is the starting point of the line.


        // A 3d plane can be described in several ways - e.g. using a normal vector "n"
        // and a point "p0" on the plane (p0 can be retrieved from our planes using
        // n*distanceFromOrigin).
        // Another way is the equation "n * ((x,y,z) - p0) = 0", where the plane is the
        // set of all solutions to that equation. Note that by "*" we mean
        // dotproduct here.
        //
        // What we have: 2 equations of the form "n * ((x,y,z) - p0) = 0" (i.e. 2 planes)
        // What we want: one solution to these equations, i.e. one point that is on both
        // planes. This is also one point of the intersection line.
        //
        // 2 equations and 3 variables.
        // -> the vector "cross" describes the direction of the intersection line.
        //    It must also intersect at least one of the x-/y-/z-planes.
        //    If a component of "cross" is non-zero, the line must intersect with the
        //    corresponding plane, so if e.g. the z component is non-zero, we can
        //    assume z=0 and have only 2 equations and 2 variables left.
        //    However due to rounding errors with float variables, we cannot safely
        //    check for a component "!= 0.0f", so we just pick the one with the largest
        //    absolute value (which is of course guaranteed to be non-zero, as the
        //    vector is non-zero).


        // AB: note that our normals are directed to the _inside_ of the frustum,
        // therefore we need to multiply be the negative normal (or by the negative
        // distance) to get a point on the plane
        //
        // (this is because this code is intended for use in conjunction with
        // viewFrustum planes)
        Vector3 p0Plane1 = n1 * (-plane1.getDistanceFromOrigin());
        Vector3 p0Plane2 = n2 * (-plane2.getDistanceFromOrigin());

        real absx = (real) fabs(cross[0]);
        real absy = (real) fabs(cross[1]);
        real absz = (real) fabs(cross[2]);

        // "n1 * p0Plane1" and "n2 * p0Plane2" is used in all three cases below
        real n1DotP0Plane1 = Vector3::dotProduct(n1, p0Plane1);
        real n2DotP0Plane2 = Vector3::dotProduct(n2, p0Plane2);

        if (absx >= absy && absx >= absz) {
            // "n * ((x,y,z) - p0) = 0" simplifies to "n * ((0,y,z) - p0) = 0" for
            // both planes, i.e.
            // "n1 * ((x,y,0) - p0Plane1) = 0" and "n2 * ((0,y,z) - p0Plane2) = 0"
            // this is a 2 equations with 2 variables which is solved below.

            intersectionPoint->setX(0.0f);
            intersectionPoint->setY((n2DotP0Plane2 * n1.getZ() - n1DotP0Plane1 * n2.getZ()) / (n1.getZ() * n2.getY() - n2.getZ() * n1.getY()));
            intersectionPoint->setZ((n2DotP0Plane2 * n1.getY() - n1DotP0Plane1 * n2.getY()) / (n1.getY() * n2.getZ() - n2.getY() * n1.getZ()));
        } else if (absy >= absx && absy >= absz) {
            intersectionPoint->setY(0.0f);
            intersectionPoint->setX((n2DotP0Plane2 * n1.getZ() - n1DotP0Plane1 * n2.getZ()) / (n1.getZ() * n2.getX() - n2.getZ() * n1.getX()));
            intersectionPoint->setZ((n2DotP0Plane2 * n1.getX() - n1DotP0Plane1 * n2.getX()) / (n1.getX() * n2.getZ() - n2.getX() * n1.getZ()));
        } else {
            intersectionPoint->setZ(0.0f);
            intersectionPoint->setX((n2DotP0Plane2 * n1.getY() - n1DotP0Plane1 * n2.getY()) / (n1.getY() * n2.getX() - n2.getY() * n1.getX()));
            intersectionPoint->setY((n2DotP0Plane2 * n1.getX() - n1DotP0Plane1 * n2.getX()) / (n1.getX() * n2.getY() - n2.getX() * n1.getY()));
        }
        return true;
    }

    /*!
     * Intersect the three planes with each other and return the intersection
     * point in \p intersectionPoint.
     *
     * \return TRUE if an intersection point was found, otherwise FALSE
     * (probably two planes are parallel to each other and thus don't intersect
     * or are equal)
     *
     * WARNING: this code has only slightly be tested, so beware of bugs.
     * However I expect this code works properly.
     */
    bool Plane::intersectPlanes(const Plane& plane1, const Plane& plane2, const Plane& plane3, Vector3* intersectionPoint) {
        const Vector3& n1 = plane1.getNormal();
        const Vector3& n2 = plane2.getNormal();
        const Vector3& n3 = plane3.getNormal();
        Vector3 cross12 = Vector3::crossProduct(n1, n2);
        Vector3 cross23 = Vector3::crossProduct(n2, n3);
        Vector3 cross31 = Vector3::crossProduct(n3, n1);
        if (cross12.isEqual(0.0f, 0.0f, 0.0f)) {
            // AB: note that intersections are still possible, if planes are equal
            // -> see isEqual()
            return false;
        }
        if (cross23.isEqual(0.0f, 0.0f, 0.0f)) {
            return false;
        }
        if (cross31.isEqual(0.0f, 0.0f, 0.0f)) {
            return false;
        }

        real l = n1.dotProduct(cross23);
        if (fabs(l) <= 0.0001) {
            return false;
        }

        Vector3 p = cross23 * plane1.getDistanceFromOrigin() * -1.0;
        p = p + cross31 * plane2.getDistanceFromOrigin() * -1.0;
        p = p + cross12 * plane3.getDistanceFromOrigin() * -1.0;

        // AB: side note: n1 DOT (n2 CROSS n3) equals the determinant of the 3x3
        //     matrix of the 3 column vectors n1,n2,n3 next to each other.
        real invScale = Vector3::dotProduct(n1, Vector3::crossProduct(n2, n3));

        if (fabs(invScale) <= 0.0001) {
            return false;
        }
        p.scale((real)(1.0 / invScale));

        *intersectionPoint = p;

        return true;
    }

    /*!
     * This returns the intersection of a line and a plane in @p intersection. Note
     * that a line always has infinite length - line segments (finite length) are
     * not handled here.
     *
     * This function assumes that an intersection actually exists, as this is what
     * we need in this file. Checking whether an intersection exists, is pretty
     * easy.
     *
     * @param linePoint Just a point on the line
     * @param lineVector The line vector, i.e. the direction of the line. This is
     * returned by @ref planes_intersect or can easily be calculated using (p0-p1)
     * if p0 and p1 are two different points on the line.
     */
    bool Plane::intersectLine(const Vector3& linePoint, const Vector3& lineVector, Vector3* intersection) const {
        real factor;
        if (!intersectLineInternal(linePoint, lineVector, &factor)) {
            return false;
        }
        *intersection = linePoint + lineVector * factor;
        return true;
    }

    bool Plane::intersectLineSegment(const Vector3& linePoint1, const Vector3& linePoint2, Vector3* intersection) const {
        real factor;
        Vector3 lineVector = (linePoint2 - linePoint1);
        if (!intersectLineInternal(linePoint1, lineVector, &factor)) {
            return false;
        }
        if (factor < 0.0 || factor > 1.0) {
            return false;
        }
        *intersection = linePoint1 + lineVector * factor;
        return true;
    }

    /*!
     * @param factor how often the @p lineVector has to be added to @p linePoint to
     * get the intersection point. If @p factor is between 0 and 1, then the
     * intersection point is on the line segment that starts at linePoint and ends
     * with linePoint + lineVector.
     * @return TRUE if the line intersects with the plane
     */
    bool Plane::intersectLineInternal(const Vector3& linePoint, const Vector3& lineVector, real* factor) const {
        // http://geometryalgorithms.com/Archive/algorithm_0104/algorithm_0104B.htm#intersect3D_SegPlane()
        // provides a nice explanation of the maths in here

        real NdotLine = Vector3::dotProduct(getNormal(), lineVector);
        if (fabs(NdotLine) <= 0.001) {
            // intersection still possible, if the line is on the plane.
            return false;
        }

        // now we know the line _does_ intersect with the plane (let's call the point p)
        // the vector from a point on the plane to p is perpendicular to the plane
        // normal.
        // That means their dot product is 0.
        //
        // i.e. normal * (point_on_plane - p) = 0
        // "p" can also be written as linePoint + a * lineVector, with a being a
        // certain real number. this makes:
        // normal * (point_on_plane - (linePoint + a * lineVector)) = 0
        // =>
        // normal * (point_on_plane - linePoint) + a * normal * lineVector = 0
        // =>
        // a = (normal * (point_on_plane - linePoint)) / (normal * lineVector)

        real foo = Vector3::dotProduct(getNormal(), calculatePointOnPlane() - linePoint);

        real a = foo / NdotLine;

        *factor = a;
        return true;
    }
}

/*
 * vim:et ts=4 sw=4
 */
