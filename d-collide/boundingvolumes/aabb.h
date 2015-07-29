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

#ifndef DCOLLIDE_AABB_H
#define DCOLLIDE_AABB_H

#include "math/vector.h"
#include "boundingvolumes/boundingvolume.h"

#include <list>
#include <vector>
#include <math.h>

namespace dcollide {
    class Vertex;

    /*!
     * \brief Axis Aligned Bounding Box
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class Aabb : public BoundingVolume {
        private:
            Vector3 mMin;
            Vector3 mMax;

        public:
            Aabb();
            Aabb(const Vector3& min, const Vector3& max);
            explicit Aabb(const std::list<Vector3>& points);
            Aabb(const Aabb& copy);
            ~Aabb();

            /*!
             * @return @ref BoundingVolume::BV_TYPE_AABB
             **/
            virtual BoundingVolumeType getVolumeType() const {
                return BV_TYPE_AABB;
            }

            virtual bool collidesWith(const BoundingVolume& other) const;

            inline const Vector3& getMin() const;
            inline const Vector3& getMax() const;

            virtual void mergeWith(const BoundingVolume* otherBV);

            Aabb* intersectWith(const Aabb& other) const;
            static Aabb* calculateIntersection( const Aabb& aabb1,
                    const Aabb& aabb2);

            void translate(const Vector3& translateBy);

            inline Vector3 getSurroundingAabbExtents() const;
            inline Vector3 getSurroundingAabbMin() const;
            inline Vector3 getSurroundingAabbMax() const;

            inline double calculateVolume() const;
            
            inline Aabb& operator=(const Aabb& otherAabb);
            inline bool operator==(const Aabb& otherAabb) const;
            
            void adjustToTriangle(const Triangle* triangle);
            bool containsVertex(const Vertex& vertex) const;

        protected:
            bool collidesWithInternal(const Aabb& other) const;

            void adjustToBox(const Matrix* worldState, const Box* box);
            void adjustToCone(const Matrix* worldState, const Cone* cone);
            void adjustToCylinder(const Matrix* worldState,
                                  const Cylinder* meshPart);
            void adjustToSphere(const Matrix* worldState, const Sphere* sphere);
            void adjustToWedge(const Matrix* worldState, const Wedge* meshPart);

            void adjustToPoints(const Matrix* worldState, const std::list<Vector3>& points);

            virtual void reset(const std::list<Vector3>& points);
            virtual void reset(const std::vector<Vertex*>& vertices);
    };



    /*!
     * \return The minimum point of this Aabb, i.e. the point with minimal x, y
     * and z values.
     */
    const Vector3& Aabb::getMin() const {
        return mMin;
    }

    /*!
     * \return The maximum point of this Aabb, i.e. the point with maximal x, y
     * and z values.
     */
    const Vector3& Aabb::getMax() const {
        return mMax;
    }

    /*!
     * \return The size of this Aabb, i.e. the diagonal vector
     */
    Vector3 Aabb::getSurroundingAabbExtents() const {
        return mMax-mMin;
    }

    Vector3 Aabb::getSurroundingAabbMin() const {
        return getMin();
    }

    Vector3 Aabb::getSurroundingAabbMax() const {
        return getMax();
    }

    Aabb& Aabb::operator=(const Aabb& otherAabb) {
        mMin = otherAabb.getMin();
        mMax = otherAabb.getMax();
        return *this;
    }

    /*!
     *  \brief checks if 2 AABBs are the same
     *  \return true, if they are the same, false otherwise
     */
    inline bool Aabb::operator==(const Aabb& otherAabb) const {
        if ( (mMin == otherAabb.getMin()) && (mMax == otherAabb.getMax()) ) {
            return true;
        } else {
            return false;
        }
    }

    double Aabb::calculateVolume() const {
        return      fabs(mMax.getX() - mMin.getX())
                *   fabs(mMax.getY() - mMin.getY())
                *   fabs(mMax.getZ() - mMin.getZ());
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Aabb& v);
}


#endif

/*
 * vim: et sw=4 ts=4
 */
