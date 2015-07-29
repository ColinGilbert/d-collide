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

#ifndef DCOLLIDE_PLANE_H
#define DCOLLIDE_PLANE_H

#include "math/vector.h"
#include "real.h"

#include <math.h>

namespace dcollide {
    class Plane {
        public:
            inline Plane();
            inline Plane(const Vector3& normal, real distance);
            inline Plane(const Vector3& normal, 
                    const Vector3& pointOnPlane, 
                    const bool normalize = true);
            inline Plane(const real* plane);
            inline Plane(const real* normal, real distance);
            inline Plane(const Plane& plane);

            inline void loadPlane(const Vector3& normal, real distance);
            inline void loadPlane(const Vector3& normal, const Vector3&
                    pointOnPlane, const bool normalize = true);
            inline void loadPlane(const real* plane);
            inline void loadPlane(const real* normal, real distance);
            inline void loadPlane(const Plane& p);

            inline const Vector3& getNormal() const;

            inline real getDistanceFromOrigin() const;
            inline Vector3 calculatePointOnPlane() const;

            inline void debugPlane();

            static void debugPlane(const Vector3& normal, real distanceFromOrigin);

            inline Plane& operator=(const Plane& p);

            inline real calculateDistance(const Vector3& p) const;
            inline real calculateDistance(real x, real y, real z) const;
            inline bool isOnPlane(real x, real y, real z) const;
            inline bool isOnPlane(const Vector3& pos, real epsilon = 0.0001) const;
            inline bool isBehindPlane(real x, real y, real z) const;
            inline bool isBehindPlane(const Vector3& pos) const;
            inline bool isInFrontOfPlane(real x, real y, real z) const;
            inline bool isInFrontOfPlane(const Vector3& pos) const;

            inline bool isEqual(const Plane& plane) const;
            inline bool isCoplanar(const Plane& plane) const;

            inline void normalize();

            inline bool intersectPlanes(const Plane& plane1, const Plane& plane2, Vector3* intersectionPoint) const;
            static bool intersectPlanes(const Plane& plane1, const Plane& plane2, const Plane& plane3, Vector3* intersectionPoint);

            inline bool intersectPlane(const Plane& plane, Vector3* intersectionPoint, Vector3* intersectionVector) const;
            static bool intersectPlane(const Plane& plane1, const Plane& plane2, Vector3* intersectionPoint, Vector3* intersectionVector);

            bool intersectLine(const Vector3& linePoint, const Vector3& lineVector, Vector3* intersection) const;
            bool intersectLineSegment(const Vector3& linePoint1, const Vector3& linePoint2, Vector3* intersection) const;

        protected:
            bool intersectLineInternal(const Vector3& linePoint, const Vector3& lineVector, real* factor) const;

        private:
            Vector3 mNormal;
            real mDistanceFromOrigin;
    };

    inline Plane::Plane() {
        mDistanceFromOrigin = 0.0;
    }

    /*!
     * Construct a plane with normal vector @p normal and a distance of @p distance
     * from the origin.
     * See @ref loadPlane
     */
    inline Plane::Plane(const Vector3& normal, real distance) {
        loadPlane(normal, distance);
    }

    /*!
     * Construct a plane with normal vector @p normal and a @p pointOnPlane.
     * See @ref loadPlane
     */
    inline Plane::Plane(const Vector3& normal, const Vector3& pointOnPlane,
            const bool normalize) {
        loadPlane(normal, pointOnPlane, normalize);
    }

    /*!
     * Construct a plane that is a copy of @p plane . See @ref loadPlane
     * See @ref loadPlane
     */
    inline Plane::Plane(const real* plane) {
        loadPlane(plane);
    }

    /*!
     * Construct a plane with normal vector @p normal and a distance of @p distance
     * from the origin.
     * See @ref loadPlane
     */
    inline Plane::Plane(const real* normal, real distance) {
        loadPlane(normal, distance);
    }

    /*!
     * Construct plane matrix that is a copy of @p plane.
     * See @ref loadPlane
     */
    inline Plane::Plane(const Plane& plane) {
        loadPlane(plane);
    }

    /*!
     * Load the plane using its @p normal and its @p distance from the
     * origin.
     */
    inline void Plane::loadPlane(const Vector3& normal, real distance) {
        mNormal = normal;
        mDistanceFromOrigin = distance;
        normalize();
    }

    inline void Plane::loadPlane(const Vector3& normal, const Vector3&
            pointOnPlane, const bool normalize) {
        // AB:
        // "pointOnPlane" (in the following I'll call it p) can be
        // interpreted as a vector from the origin to that point.
        //
        // The distanceFromOrigin of the plane can then easily be
        // calculated:
        // We have the normal and p interpreted as a vector. together
        // with the plane, they make up a triangle.
        // In that triangle, between p and the normal there is the angle
        // alpha:
        // cos(alpha) = distanceFromOrigin / |p|
        // as "distanceFromOrigin" is the length of the normal from the
        // origin to the plane. It also is the adjacent side in our
        // triangle. |p| is the hyypotenuse.
        // This can be written as
        // distanceFromOrigin = |p| * cos(alpha)
        //
        // Now if the normal is normalized (i.e. |normal| = 1) this is
        // exactly what dotProduct(p, normal) calculates.
        //
        // We need to multiply that with -1 so that the normal in the
        // triangle points into the correct direction
        // (ok, lousy explanation. i dont find a better one - consider
        // to use a piece of paper to make this clear to yourself)

        Vector3 normalizedNormal = normal;
        // Do not normalize this vector if special param false given by user:
        if (normalize) {
            normalizedNormal.normalize();
        }
        real distance = -Vector3::dotProduct(normalizedNormal, pointOnPlane);
        loadPlane(normalizedNormal, distance);
    }
    inline void Plane::loadPlane(const real* plane) {
        loadPlane(plane, plane[3]);
    }
    inline void Plane::loadPlane(const real* normal, real distance) {
        loadPlane(Vector3(normal), distance);
    }
    inline void Plane::loadPlane(const Plane& p) {
        loadPlane(p.getNormal().getData(), p.getDistanceFromOrigin());
    }

    inline const Vector3& Plane::getNormal() const {
        return mNormal;
    }

    /*!
     * @return The distance from the origin
     */
    inline real Plane::getDistanceFromOrigin() const {
        return mDistanceFromOrigin;
    }
    /*!
     * Many algorithms work by using the normal of the plane and a point on
     * the plane (instead of the @ref getDistanceFromOrigin). This method
     * provides such a point - but note that it must be created on the fly,
     * it is not stored in this class.
     * @return A point on the plane
     */
    inline Vector3 Plane::calculatePointOnPlane() const {
        return getNormal() * (-getDistanceFromOrigin());
    }

    /*!
     * Dump this plane to the console as debug output.
     */
    inline void Plane::debugPlane() {
        debugPlane(getNormal(), getDistanceFromOrigin());
    }

    /*!
     * See @ref loadPlane
     */
    inline Plane& Plane::operator=(const Plane& p) {
        loadPlane(p);
        return *this;
    }

    inline real Plane::calculateDistance(const Vector3& p) const {
        return Vector3::dotProduct(getNormal(), p) + getDistanceFromOrigin();
    }
    inline real Plane::calculateDistance(real x, real y, real z) const {
        return calculateDistance(Vector3(x, y, z));
    }

    inline bool Plane::isOnPlane(real x, real y, real z) const {
        return isOnPlane(Vector3(x, y, z));
    }
    inline bool Plane::isOnPlane(const Vector3& pos, real epsilon) const {
        return (fabs(calculateDistance(pos))) < epsilon;
    }

    inline bool Plane::isBehindPlane(real x, real y, real z) const {
        return isBehindPlane(Vector3(x, y, z));
    }
    inline bool Plane::isBehindPlane(const Vector3& pos) const {
        // FIXME maybe use an epsilon here, too?
        return (calculateDistance(pos) < 0.0f);
    }

    inline bool Plane::isInFrontOfPlane(real x, real y, real z) const {
        return isInFrontOfPlane(Vector3(x, y, z));
    }
    inline bool Plane::isInFrontOfPlane(const Vector3& pos) const {
        // FIXME maybe use an epsilon here, too?
        return (calculateDistance(pos) > 0.0f);
    }

    inline bool Plane::isEqual(const Plane& plane) const {
        if (!getNormal().isEqual(plane.getNormal(), (real) 0.001)) {
            return false;
        }
        if (fabs(getDistanceFromOrigin() - plane.getDistanceFromOrigin()) > 0.001) {
            return false;
        }
        return true;
    }

    /*!
     *\brief similar to isEqual, but allows the planes to have inverted normals
     * since \ref mDistanceFromOrigin is signed and depends on the normal, we
     * have to make sure to multiply one distance with -1 if the normals are
     * inverted.
     */
    bool Plane::isCoplanar(const Plane& plane) const {
        int distanceMultiplicator = 1;
        if (!mNormal.isEqual(plane.getNormal(), (real) 0.001)) {
            //maybe inverted?
            if (!mNormal.isEqual(plane.getNormal()*-1, (real) 0.001)) {
                return false;
            } else {
                //normals are inverted, one distance must be multip. with -1
                distanceMultiplicator = -1;
            }
        }
        if (fabs(getDistanceFromOrigin()*distanceMultiplicator
                                 - plane.getDistanceFromOrigin()) > 0.001) {
            return false;
        }
        return true;
    }

    inline void Plane::normalize() {
        real inv_length = 1.0f / getNormal().length();
        mNormal.scale(inv_length);
        mDistanceFromOrigin *= inv_length;
    }

    inline bool Plane::intersectPlane(const Plane& plane, Vector3* intersectionPoint, Vector3* intersectionVector) const {
        return Plane::intersectPlane(*this, plane, intersectionPoint, intersectionVector);
    }
    inline bool Plane::intersectPlanes(const Plane& plane1, const Plane& plane2, Vector3* intersectionPoint) const {
        return Plane::intersectPlanes(*this, plane1, plane2, intersectionPoint);
    }

}

#endif
/*
 * vim:et ts=4 sw=4
 */
