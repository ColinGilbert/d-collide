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


#ifndef DCOLLIDE_BOUNDINGSPHERE
#define DCOLLIDE_BOUNDINGSPHERE

#include "math/vector.h"
#include "boundingvolumes/boundingvolume.h"

namespace dcollide {
    class Vertex;

    //-----------classes------------

    class BoundingSphere : public BoundingVolume {
        public:
            inline BoundingSphere();
            inline BoundingSphere(const Vector3& center, real radius);
            inline explicit BoundingSphere(const std::list<Vector3>& points);
            inline BoundingSphere(const BoundingSphere& copy);
            inline ~BoundingSphere();
            inline BoundingSphere(const Vertex* centerVertex, real radius);

            inline virtual BoundingVolumeType getVolumeType() const;

            bool collidesWith(const BoundingVolume& other) const;
            //void adjustToShape(const Matrix* worldState, const Shape* shape);
            void mergeWith(const BoundingVolume* otherBV);
            void mergeWith(const BoundingSphere* other);
            void mergeWith(const std::list<BoundingSphere*> others);
            void translate(const Vector3& translateBy);

            inline Vector3 getSurroundingAabbExtents() const;
            Vector3 getSurroundingAabbMin() const;
            Vector3 getSurroundingAabbMax() const;

            inline const Vertex* getCenterVertex() const;
            const Vector3& getCenterVector() const;
            inline real getRadius() const;

            void adjustToTriangle(const Triangle* triangle);

        protected:
            void adjustToBox(const Matrix* worldState, const Box* box);
            void adjustToCylinder(const Matrix* worldState,
                                  const Cylinder* meshPart);
            void adjustToCone(const Matrix* worldState, const Cone* cone);
            void adjustToMesh(const Mesh* mesh);
            void adjustToMeshPart(const MeshPart* meshPart);
            void adjustToSphere(const Matrix* worldState, const Sphere* sphere);
            void adjustToWedge(const Matrix* worldState, const Wedge* meshPart);

            virtual void reset(const std::list<Vector3>& points);
            virtual void reset(const std::vector<Vertex*>& vertices);

        private:
            const Vertex* mCenterVertex;
            Vector3 mCenterVector;
            real mRadius;

    };


    //------------ Implementation of short methods -------------

    BoundingSphere::BoundingSphere() : BoundingVolume() {
        mCenterVertex = 0;
        mCenterVector = Vector3(0,0,0);
        mRadius = 0;
    }

    BoundingSphere::BoundingSphere(const Vector3& center, real radius)
        : BoundingVolume() {
        mCenterVertex = 0;
        mCenterVector = center;
        mRadius = radius;
    }

    BoundingSphere::BoundingSphere(const Vertex* centerVertex, real radius) : BoundingVolume() {
        mCenterVertex = centerVertex;
        mRadius = radius;
    }

    BoundingSphere::BoundingSphere(const std::list<Vector3>& points)
        : BoundingVolume() {
        mCenterVertex = 0;
        reset(points);
    }

    BoundingSphere::BoundingSphere(const BoundingSphere& copy)
        : BoundingVolume() {

        mCenterVertex = 0;
        mRadius = copy.getRadius();

        if (!copy.getCenterVertex()) {
            mCenterVector = copy.getCenterVector();
        } else {
            mCenterVertex = copy.getCenterVertex();
        }
    }

    BoundingSphere::~BoundingSphere() {
    }


    BoundingVolumeType BoundingSphere::getVolumeType() const {
        return BV_TYPE_SPHERE;
    }

    Vector3 BoundingSphere::getSurroundingAabbExtents() const {
        return Vector3(2 * mRadius, 2 * mRadius, 2 * mRadius);
    }


    const Vertex* BoundingSphere::getCenterVertex() const {
        return mCenterVertex;
    }

    real BoundingSphere::getRadius() const {
        return mRadius;
    }

}

#endif // DCOLLIDE_BOUNDINGSPHERE

/*
 * vim: et sw=4 ts=4
 */
