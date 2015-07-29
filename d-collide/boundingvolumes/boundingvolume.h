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

#ifndef DCOLLIDE_BOUNDINGVOLUME_H
#define DCOLLIDE_BOUNDINGVOLUME_H

#include <list>
#include <vector>

#include "math/matrix.h"
#include "shapes/mesh/vertex.h"
#include "exceptions/exception.h"
#include "dcollide-global.h"

namespace dcollide {

    /*! forward declarations
     */
    class Triangle;
    class BvhNode;
    class Proxy;
    class Shape;
    class Box;
    class Cone;
    class Cylinder;
    class Mesh;
    class MeshPart;
    class Sphere;
    class Wedge;
    class World;


    /*!
     * \brief Generalized interface for bounding volumes.
     */
    class BoundingVolume {
        public:
            virtual ~BoundingVolume();

            /*!
             * @return The type of the Bounding Volume, i.e. AABB, k-DOP, ...
             * See @ref BoundingVolumeType
             **/
            virtual BoundingVolumeType getVolumeType() const = 0;
            virtual bool collidesWith(const BoundingVolume& other) const = 0;

            void adjustToShape(const Shape* shape);

            // TODO: documentation
            virtual void adjustToTriangle(const Triangle* triangle) = 0;

            /*!
             * \brief adjusts this bounding volume so that the other bounding
             *        volume is also [umfasst/umhüllt]
             */
            virtual void mergeWith(const BoundingVolume* otherBV) = 0;
            virtual void translate(const Vector3& translateBy) = 0;

            //getters and setters
            inline const BvhNode* getHierarchyNode() const;
            inline void setHierarchyNode(BvhNode* node);
            virtual Vector3 getSurroundingAabbExtents() const = 0;

            /*!
             * \return The min vector of an axis aligned bounding box
             * around this bounding volume, i.e. the minimum x, y and z 
             * components of all points in this bounding volume. Note that 
             * this point is not necessarily inside this bounding volume.
             */
            virtual Vector3 getSurroundingAabbMin() const = 0;
            /*!
             * \return The max vector of an axis aligned bounding box
             * around this bounding volume, i.e. the maximum x, y and z 
             * components of all points in this bounding volume. Note that 
             * this point is not necessarily inside this bounding volume.
             */
            virtual Vector3 getSurroundingAabbMax() const = 0;

            inline const std::list<Triangle*>& getEnclosedTriangles() const;
            inline void setEnclosedTriangles(const std::list<Triangle*>& triangles);

            //Factory functions
            static BoundingVolume* createBoundingVolume(World* world, BvhNode* parent);
            static BoundingVolume* createBoundingVolume(World* world, const BoundingVolume* copy, BvhNode* parent);

            static inline const Vector3& retrieveVector3FromIterator(const std::list<Vector3>::const_iterator& it);
            static inline const Vector3& retrieveVector3FromIterator(const std::vector<Vertex*>::const_iterator& it);

            inline bool operator==(const BoundingVolume& v) const;

        protected:
            static BoundingVolumeType getCreateRigidBoundingVolumeType(World* world);

            /*!
             * \overload
             *
             * Make this \ref BoundingVolume contain exactly the points contained in
             * \p points.
             *
             * Used by \ref adjustToMesh
             */
            virtual void reset(const std::list<Vector3>& points) = 0;

            /*!
             * \overload
             *
             * Make this \ref BoundingVolume contain exactly the vertices contained in
             * \p vertices.
             *
             * The default implementation simply retrieves the position (see \ref
             * Vertex::getWorldPosition) of each vertex and calls the \ref reset method
             * that takes a list of \ref Vector3 objects.
             *
             * Used by \ref adjustToMesh
             */
            virtual void reset(const std::vector<Vertex*>& vertices) = 0;

            // TODO: documentation
            virtual void adjustToBox(const Matrix* worldState,
                                     const Box* box) = 0;
            // TODO: documentation
            virtual void adjustToCone(const Matrix* worldState,
                                      const Cone* cone) = 0;
            // TODO: documentation
            virtual void adjustToCylinder(const Matrix* worldState, 
                                          const Cylinder* meshPart) = 0;

            virtual void adjustToMesh(const Mesh* mesh);
            virtual void adjustToMeshPart(const MeshPart* meshPart);

            // TODO: documentation
            virtual void adjustToSphere(const Matrix* worldState,
                                        const Sphere* sphere) = 0;
            // TODO: documentation
            virtual void adjustToWedge(const Matrix* worldState, 
                                       const Wedge* meshPart) = 0;

            void resetToApproximatedAabbOfCone(const Matrix* worldState, const Cone* cone);
            void resetToApproximatedAabbOfCylinder(const Matrix* worldState, const Cylinder* cylinder);

        private:
            BvhNode* mHierarchyNode;
            std::list<Triangle*> mEnclosedTriangles;
    };


    //------------ Implementation of short methods -------------

    void BoundingVolume::setHierarchyNode(BvhNode* node) {
        mHierarchyNode = node;
    }

    const BvhNode* BoundingVolume::getHierarchyNode() const {
        return mHierarchyNode;
    }

    const std::list<Triangle*>& BoundingVolume::getEnclosedTriangles() const {
        return mEnclosedTriangles;
    }

    void BoundingVolume::setEnclosedTriangles(const std::list<Triangle*>& triangles) {
        mEnclosedTriangles = triangles;
    }


    const Vector3& BoundingVolume::retrieveVector3FromIterator(const std::list<Vector3>::const_iterator& it) {
        return *it;
    }
    const Vector3& BoundingVolume::retrieveVector3FromIterator(const std::vector<Vertex*>::const_iterator& it) {
        return (*it)->getWorldPosition();
    }


    bool BoundingVolume::operator==(const BoundingVolume& v) const {
        return (   (v.getVolumeType() == getVolumeType())
                && (v.getSurroundingAabbMin() == getSurroundingAabbMin())
                && (v.getSurroundingAabbMax() == getSurroundingAabbMax()));
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::BoundingVolume& v);
}

#endif // DCOLLIDE_BOUNDINGVOLUME_H
/*
 * vim: et sw=4 ts=4
 */
