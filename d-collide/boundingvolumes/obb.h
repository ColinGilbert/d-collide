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

#ifndef DCOLLIDE_OBB_H
#define DCOLLIDE_OBB_H

#include "math/vector.h"
#include "math/matrix.h"
#include "shapes/mesh/vertex.h"
#include "boundingvolumes/boundingvolume.h"

#include <list>
#include <vector>
#include <math.h>

#define OBB_DEBUG 1
#undef OBB_DEBUG

namespace dcollide {
    class Vertex;

    /*!
     * \brief struct to represent a element in a outside set
     * used by the algorithm that creates the convex hull.
     * note that the furthestPoint is not allowed to be in the list of the
     * points!
     */
    struct OutsideSet {
        Triangle* triangle;
        std::list<std::pair<Vector3,real> > points;
        std::pair<Vector3,real> furthestPoint;
        inline OutsideSet(Triangle* tri, 
                std::list<std::pair<Vector3,real> > ps,
                std::pair<Vector3,real> furthest);
        inline OutsideSet(const struct OutsideSet& copy);
    };

    inline OutsideSet::OutsideSet(Triangle* tri, 
                std::list<std::pair<Vector3,real> > ps,
                std::pair<Vector3,real> furthest) {
        triangle = tri;
        points = ps;
        furthestPoint = furthest;
    }

    inline OutsideSet::OutsideSet(const struct OutsideSet& copy) {
        triangle = copy.triangle;
        points = copy.points;
        furthestPoint = copy.furthestPoint;
    }

    /*!
     * \brief Object Oriented Bounding Box
     */
    class Obb : public BoundingVolume {
        private:
            Vector3 mDimension;
            Matrix mState;
            Vector3 mReferencePoint;
            friend class ObbTest;

        public:
            Obb();
            Obb(const Vector3& center, const Vector3& dimension, 
                    const Matrix& rotation, const Vector3& referencePoint);
            Obb(const Vector3& dimension, const Matrix& state);

            explicit Obb(const std::list<Vector3>& points);
            Obb(const Obb& copy);
            ~Obb();

            /*!
             * @return @ref BoundingVolume::BV_TYPE_OBB
             **/
            virtual BoundingVolumeType getVolumeType() const {
                return BV_TYPE_OBB;
            }

            virtual bool collidesWith(const BoundingVolume& other) const;

            virtual void mergeWith(const BoundingVolume* otherBV);

            Obb* intersectWith(const Obb& other) const;
            static Obb* calculateIntersection( const Obb& obb1,
                    const Obb& obb2);

            inline const Vector3 getCenter() const;
            inline const Vector3& getDimension() const;
            inline const Vector3& getReferencePoint() const;
            inline const Matrix getRotation() const;
            inline const Matrix& getState() const;

            inline void translate(const Vector3& translateBy);
            inline void rotate(const Matrix& rotateBy);
            inline void transform(const Matrix& transformBy);

            Vector3 getSurroundingAabbExtents() const;
            Vector3 getSurroundingAabbMin() const;
            Vector3 getSurroundingAabbMax() const;

            inline real calculateVolume() const;

            inline Obb& operator=(const Obb& otherObb);
            inline bool operator==(const Obb& otherObb) const;

            void adjustToTriangle(const Triangle* triangle);
            Vector3* getVertices() const;

        protected:
            bool collidesWithInternal(const Obb& other) const;

            void adjustToBox(const Matrix* worldState, const Box* box);
            void adjustToCone(const Matrix* worldState, const Cone* cone);
            void adjustToCylinder(const Matrix* worldState,
                                  const Cylinder* meshPart);
            void adjustToSphere(const Matrix* worldState, const Sphere* sphere);
            void adjustToWedge(const Matrix* worldState, const Wedge* wedge);
            void adjustToMesh(const Mesh* mesh);
            void adjustToMeshPart(const MeshPart* meshpart);

            void adjustToPoints(const Matrix* worldState, const std::list<Vector3>& points);

            virtual void reset(const std::list<Vector3>& points);
            virtual void reset(const std::vector<Vertex*>& vertices);

        private:
            void adjustToConeOrCylinderInternal(const Matrix* worldState, 
                    const real& radius, const real& height);

            void adjustToTriangles(const std::vector<Triangle*>& triangles,
                    const std::vector<Vertex*>& vertices);
            void computeSpan(const Vector3* vertices, const Vector3& axis, real&
                    min, real& max) const;
            inline bool overlap(float min1, float max1, 
                    float min2, float max2) const;
            void performVectorIteration(const Matrix& A,Vector3* x,int sizeOf);
            std::vector<Triangle*> calculateConvexHull(
                    const std::vector<Triangle*>& triangles,
                    const std::vector<Vertex*>& vertices) const;

    };


    /*!
     * \returns The center of the obb
     */
    const Vector3 Obb::getCenter() const {
        return mState.getPosition();
    }

    /*!
     * \returns The dimension of the obb
     */
    const Vector3& Obb::getDimension() const {
        return mDimension;
    }

    /*!
     * \returns The reference üoint of the obb
     */
    const Vector3& Obb::getReferencePoint() const {
        return mReferencePoint;
    }

    /*!
     * \returns The rotation matrix of the obb
     */
    const Matrix Obb::getRotation() const {
        return mState.getRotationMatrix();
    }

    /*!
     * \returns The state matrix of the obb
     */
    const Matrix& Obb::getState() const {
        return mState;
    }

    /*!
     * \brief translate this obb by \p translateBy
     */
    void Obb::translate(const Vector3& translateBy) {
#ifdef OBB_DEBUG
        std::cout << "CB: " << mState.getPosition();
#endif
        mState.translate(translateBy,false);
#ifdef OBB_DEBUG
        std::cout << "; CA: " << mState.getPosition() << std::endl;
#endif
    }

    /*!
     * \brief rotate this obb by \p rotateBy
     * ATTENTION: This MUST be a valid rotation matrix!
     */
    void Obb::rotate(const Matrix& rotateBy) {
        mState.translate(-mReferencePoint);
        mState.multiply(&rotateBy);
        mState.translate(mReferencePoint);
    }

    /*!
     * \brief transform this obb by \p transformBy
     * ATTENTION: be aware that you can do nasty stuff with this method, even
     * deformation of the obb is possible!!
     */
    void Obb::transform(const Matrix& transformBy) {
        mState.multiply(&transformBy);
    }

    Obb& Obb::operator=(const Obb& otherObb) {
        mDimension = otherObb.getDimension();
        mState = otherObb.getState();
        mReferencePoint = otherObb.getReferencePoint();
        return *this;
    }

    /*!
     *  \brief checks if 2 OBBs are the same
     *  \return true, if they are the same, false otherwise
     */
    inline bool Obb::operator==(const Obb& otherObb) const {
        if (    (mDimension == otherObb.getDimension()) &&
                (mReferencePoint == otherObb.getReferencePoint()) &&
                (mState.isEqual(otherObb.getState()))) {
            return true;
        } else {
            return false;
        }
    }

    real Obb::calculateVolume() const {

        Vector3* vs = getVertices();
        real volume =
            (vs[0]-vs[1]).length()*(vs[0]-vs[4]).length()*(vs[0]-vs[2]).length();
        delete[] vs;
        return volume;
    }

    /*!
     * \brief Determine if given intervals overlap.
     */
    inline bool Obb::overlap(float min1, float max1,
            float min2, float max2) const {
        return !(min1 > max2 || max1 < min2);
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Obb& v);
}


#endif

/*
 * vim: et sw=4 ts=4
 */
