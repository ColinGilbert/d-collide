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

#include "boundingvolumes/aabb.h"

#include "shapes/shapes.h"
#include "shapes/mesh/vertex.h"
#include "exceptions/exception.h"
#include "debug.h"


namespace dcollide {

    /*!
     *  \brief Constructs an empty Aabb
     */
    Aabb::Aabb() {
    }

    /*!
     * \brief Constructs an Aabb with given extends
     * Constructs an Aabb with minimal point \p min and maximal point \p max
     */
    Aabb::Aabb(const Vector3& min, const Vector3& max) {
        mMin = min;
        mMax = max;
    }
    /*!
     * \brief Constructs an Aabb for given points
     * Constructs an Aabb that bounds all given points.
     */
    Aabb::Aabb(const std::list<Vector3>& points) {
        reset(points);
    }
    /*!
     * \brief Constructs an Aabb as copy of given aabb
     */
    Aabb::Aabb(const Aabb& copy) : BoundingVolume() {
        mMin = copy.mMin;
        mMax = copy.mMax;
    }

    Aabb::~Aabb() {
    }


    /*!
     * \internal
     * \brief Calculates a new Aabb from given points
     *
     * Calculates a new Aabb from \p points. Generates an empty (i.e.
     * min=max) Aabb if \p points does not contain at least two points.
     */
    template<typename Container>
    static inline void resetInternal(const Container& points, Vector3& mMin, Vector3& mMax) {

        if (points.empty()) {
            mMin = Vector3(0.0f, 0.0f, 0.0f);
            mMax = mMin;
            return;
        }

        typename Container::const_iterator it;

        it = points.begin();
        const Vector3& p0 = BoundingVolume::retrieveVector3FromIterator(it);

        real minX = p0.getX();
        real maxX = minX;
        real minY = p0.getY();
        real maxY = minY;
        real minZ = p0.getZ();
        real maxZ = minZ;

        ++it;
        for (; it != points.end(); ++it) {
            const Vector3& point = BoundingVolume::retrieveVector3FromIterator(it);
            minX = std::min(minX, point.getX());
            maxX = std::max(maxX, point.getX());
            minY = std::min(minY, point.getY());
            maxY = std::max(maxY, point.getY());
            minZ = std::min(minZ, point.getZ());
            maxZ = std::max(maxZ, point.getZ());
        }

        mMin = Vector3(minX, minY, minZ);
        mMax = Vector3(maxX, maxY, maxZ);
    }

    void Aabb::reset(const std::list<Vector3>& points) {
        dcollide::resetInternal(points, mMin, mMax);
    }

    void Aabb::reset(const std::vector<Vertex*>& vertices) {
        dcollide::resetInternal(vertices, mMin, mMax);
    }


    /*!
     * \brief Tests for intersection with another Aabb
     * \return TRUE If this Aabb intersects with \p other. Otherwise FALSE.
     * 
     * TODO: Implement correct/faster collision detection methods for
     *       collisions between different types of bounding volumes.
     */
    bool Aabb::collidesWith(const BoundingVolume& other) const {
        if (other.getVolumeType() == getVolumeType()) {
            const BoundingVolume* otherPointer = &other;
            const Aabb* aabbPointer = static_cast<const Aabb*>(otherPointer);
            return collidesWithInternal(*aabbPointer);
        }
        return collidesWithInternal(Aabb(other.getSurroundingAabbMin(),
                                         other.getSurroundingAabbMax()));
    }

    bool Aabb::collidesWithInternal(const Aabb& other) const {
        if (other.getMin().getX() > getMax().getX()) {
            return false;
        }
        if (other.getMax().getX() < getMin().getX()) {
            return false;
        }
        if (other.getMin().getY() > getMax().getY()) {
            return false;
        }
        if (other.getMax().getY() < getMin().getY()) {
            return false;
        }
        if (other.getMin().getZ() > getMax().getZ()) {
            return false;
        }
        if (other.getMax().getZ() < getMin().getZ()) {
            return false;
        }
        return true;
    }

    /*! \brief calculates the Aabb for the given sphere
     */
    void Aabb::adjustToSphere(const Matrix* worldState, const Sphere* sphere) {
        //Calculate sphere relative to its center point
        Vector3 center;
        worldState->transform(&center, Vector3(0.0, 0.0, 0.0));
        mMin = Vector3( center.getX() - sphere->getRadius(), 
                        center.getY() - sphere->getRadius(),
                        center.getZ() - sphere->getRadius());
        mMax = Vector3( center.getX() + sphere->getRadius(), 
                        center.getY() + sphere->getRadius(),
                        center.getZ() + sphere->getRadius());
    }

    /*! \brief calculates the Aabb for the given box
     */
    void Aabb::adjustToBox(const Matrix* worldState, const Box* box) {
        DCOLLIDE_UNUSED(worldState);

        // every corner of the box can be the new minimum/maximum point, so we
        // need to transform each point by our matrix.
        // for this we first find all vertices of the box, then transform all of
        // them and finally find the min/max values from those aabbVertices.

        // Getting the correctly transformed Vertices:
        const Vector3* vertices = const_cast<Box*>(box)->getVertices();

        std::list<Vector3> aabbVertices;
        for (int i = 0; i < 8; i++) {
            aabbVertices.push_back(vertices[i]);
        }
        reset(aabbVertices);
    }

    void Aabb::adjustToCone(const Matrix* worldState, const Cone* cone) {
#if 0
        //TODO: Implement the function - but you have to solve an
        // equatation system for this (imho).

        /* Vector3 position = worldState->getPosition();
        // calculate the midpoint of the sphere
        Vector3 midpoint = position;
        midpoint.setX( position.getX() + cone->getHeight());
        worldState->transform(midpoint, midpoint);

        */
        // In order to continue the calculation I have to find a way
        // to solve a equatiation system
        //
        // details on this can be found here:
        // http://d-collide.ematia.de/trac/ticket/153
#else
        resetToApproximatedAabbOfCone(worldState, cone);
#endif
    }

    void Aabb::adjustToCylinder(const Matrix* worldState,
                                const Cylinder* cylinder) {
        resetToApproximatedAabbOfCylinder(worldState, cylinder);
    }

    void Aabb::adjustToWedge(const Matrix* worldState, const Wedge* wedge) {
        DCOLLIDE_UNUSED(worldState);

        // every corner of the wedge can be the new minimum/maximum point, so we
        // need to transform each point by our matrix.
        // for this we first find all vertices of the box, then transform all of
        // them and finally find the min/max values from those aabbVertices.

        // Getting the correctly transformed Vertices:
        const Vector3* vertices = const_cast<Wedge*>(wedge)->getVertices();

        std::list<Vector3> aabbVertices;
        for (int i = 0; i < 6; i++) {
            aabbVertices.push_back(vertices[i]);
        }
        reset(aabbVertices);
    }

    /*!
     * Adjust this Aabb to an object described by \p points, transformed by \p
     * worldState.
     *
     * This method is mainly a convenience method for other adjustTo*() methods.
     * In particular it aids at Aabb approximations: first calculate an Aabb
     * around the original (untransformed) method, retrieve all 8 points from it
     * and then call this method. The points are then transformed by the
     * transformation matrix and a new Aabb matrix is calculated from it. The
     * resulting Aabb is usually larger than necessary, but the error is
     * controllable.
     */
    void Aabb::adjustToPoints(const Matrix* worldState, const std::list<Vector3>& points) {
        std::list<Vector3> transformedPoints;
        for (std::list<Vector3>::const_iterator it = points.begin(); it != points.end(); ++it) {
            Vector3 p;
            worldState->transform(&p, *it);
            transformedPoints.push_back(p);
        }

        // calculate AABB of transformed points
        reset(transformedPoints);
    }

    /*! \brief Merges this Aabb with another one
     *  Calculates a Aabb which contains both this Aabb and \p otherBV.
     */
    void Aabb::mergeWith(const BoundingVolume* otherBV) {
        if (otherBV == 0) {
            //nothing to do
            return;
        }
        Vector3 otherMin = otherBV->getSurroundingAabbMin();
        Vector3 otherMax = otherBV->getSurroundingAabbMax();

        // calculate X Values
        mMin.setX(std::min(mMin.getX(), otherMin.getX()));
        mMax.setX(std::max(mMax.getX(), otherMax.getX()));

        // calculate Y Values
        mMin.setY(std::min(mMin.getY(), otherMin.getY()));
        mMax.setY(std::max(mMax.getY(), otherMax.getY()));

        // calculate Z Values
        mMin.setZ(std::min(mMin.getZ(), otherMin.getZ()));
        mMax.setZ(std::max(mMax.getZ(), otherMax.getZ()));
    }

    /*!
     * \brief calculates the intersection with another Aabb
     * 
     * OWNERSHIP NOTICE: The caller must take care of deleting the created Aabb
     * \return NULL if the Aabbs do not intersect
     */
    Aabb* Aabb::intersectWith(const Aabb& other) const{
        if (collidesWithInternal(other)) {
            //overlap Minimum is componentwise maximum of the 2 min vectors
            Vector3 overlapMin( std::max(mMin.getX(), other.getMax().getX()),
                                std::max(mMin.getY(), other.getMax().getY()),
                                std::max(mMin.getZ(), other.getMax().getZ()));
            //overlap Maximum is componentwise minimum of the 2 max vectors
            Vector3 overlapMax( std::min(mMax.getX(), other.getMax().getX()),
                                std::min(mMax.getY(), other.getMax().getY()),
                                std::min(mMax.getZ(), other.getMax().getZ()));
            return new Aabb(overlapMin, overlapMax);
        } else {
            return 0;
        }
    }

    /*!
     * \brief calculates the intersection of two Aabbs
     * 
     * OWNERSHIP NOTICE: The caller must take care of deleting the created Aabb
     * \return NULL if the Aabbs do not intersect
     */
    Aabb* Aabb::calculateIntersection(  const Aabb& aabb1,
                                        const Aabb& aabb2) {
        return aabb1.intersectWith(aabb2);
    };
    
    void Aabb::translate(const Vector3& translateBy) {
        mMin += translateBy;
        mMax += translateBy;
    }


    void Aabb::adjustToTriangle(const Triangle* triangle) {
        const dcollide::array<Vertex*,3>& vertices = triangle->getVertices();

        const Vector3& p1 = vertices[0]->getWorldPosition();
        const Vector3& p2 = vertices[1]->getWorldPosition();
        const Vector3& p3 = vertices[2]->getWorldPosition();

        real minX = p1.getX();
        real maxX = minX;
        real minY = p1.getY();
        real maxY = minY;
        real minZ = p1.getZ();
        real maxZ = minZ;

        minX = std::min(minX, p2.getX());
        maxX = std::max(maxX, p2.getX());
        minY = std::min(minY, p2.getY());
        maxY = std::max(maxY, p2.getY());
        minZ = std::min(minZ, p2.getZ());
        maxZ = std::max(maxZ, p2.getZ());

        minX = std::min(minX, p3.getX());
        maxX = std::max(maxX, p3.getX());
        minY = std::min(minY, p3.getY());
        maxY = std::max(maxY, p3.getY());
        minZ = std::min(minZ, p3.getZ());
        maxZ = std::max(maxZ, p3.getZ());

        mMin.setX(minX);
        mMin.setY(minY);
        mMin.setZ(minZ);
        mMax.setX(maxX);
        mMax.setY(maxY);
        mMax.setZ(maxZ);
    }

    /*!
     * \brief Checks if the point given by the Vertex lies inside the Aaab
     */
    bool Aabb::containsVertex(const Vertex& vertex) const {
        Vector3 point = vertex.getWorldPosition();
        if (mMin.getX() > point.getX() || mMax.getX() < point.getX()) {
            return false;
        }
        if (mMin.getY() > point.getY() || mMax.getY() < point.getY()) {
            return false;
        }
        if (mMin.getZ() > point.getZ() || mMax.getZ() < point.getZ()) {
            return false;
        }
        return true;
    }



    std::ostream& operator<<(std::ostream& os, const dcollide::Aabb& v) {
        os << "[" << v.getMin() << ", " << v.getMax() << "]";
        return os;
    }
}


/*
 * vim: et sw=4 ts=4
 */
