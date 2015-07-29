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

#include "boundingvolumes/obb.h"

#include "shapes/shapes.h"
#include "shapes/mesh/vertex.h"
#include "exceptions/exception.h"
#include "debug.h"
#include "debugstream.h"

namespace dcollide {

    /*!
     *  \brief Constructs an empty Obb
     */
    Obb::Obb() {
    }

    /*!
     * \brief Constructs an Obb with given extends
     */
    Obb::Obb(const Vector3& center, const Vector3& dimension, const Matrix&
            rotation, const Vector3& referencePoint) {
        mDimension = dimension;
        mState = rotation;
        mReferencePoint = referencePoint;
        mState.translate(center,false);
    }

    Obb::Obb(const Vector3& dimension, const Matrix& state) {
        mDimension = dimension;
        mState = state;
        mReferencePoint = Vector3(0,0,0);
    }


    /*!
     * \brief Constructs an Obb for given points
     * Constructs an Obb that bounds all given points.
     */
    Obb::Obb(const std::list<Vector3>& points) {
        reset(points);
    }
    /*!
     * \brief Constructs an Obb as copy of given obb
     */
    Obb::Obb(const Obb& copy) : BoundingVolume() {
        *this=copy;
    }

    Obb::~Obb() {
    }

    /*!
     * \internal
     * \brief Calculates a new Obb from given points
     *
     * Calculates a new Obb from \p points. Generates an empty (i.e.
     * min=max) Obb if \p points does not contain at least two points.
     *
     * At the moment we simply calculate a aabb and use this as obb!
     * TODO: create a real OBB!
     */
    template<typename Container>
    static inline void resetInternal(const Container& points, Vector3&
            mDimension, Vector3& mReferencePoint, Matrix& mState) {
        if (points.empty()) {
            mDimension = Vector3(0.0f, 0.0f, 0.0f);
            mReferencePoint = Vector3(0.0f, 0.0f, 0.0f);
            mState = Matrix();
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

        Vector3 min = Vector3(minX, minY, minZ);
        Vector3 max = Vector3(maxX, maxY, maxZ);
        mDimension = max-min;
        mState.loadIdentity();
        mState.translate((max+min)/2,false);
        mReferencePoint = mState.getPosition();

    }

    void Obb::reset(const std::list<Vector3>& points) {
        dcollide::resetInternal(points, mDimension, mReferencePoint,mState);
    }
    void Obb::reset(const std::vector<Vertex*>& vertices) {
        dcollide::resetInternal(vertices, mDimension, mReferencePoint,mState);
    }


    /*!
     * \return The size of the Aabb of this obb, i.e. the diagonal vector
     */
    Vector3 Obb::getSurroundingAabbExtents() const {
        Vector3 min = getSurroundingAabbMin();
        Vector3 max = getSurroundingAabbMax();
        return max-min;
    }

    /*!
     * \return The min of the aabb of this Obb
     */
    Vector3 Obb::getSurroundingAabbMin() const {
        Vector3* mv = getVertices();

        real minX = mv[0].getX();
        real minY = mv[0].getY();
        real minZ = mv[0].getZ();

        for (int i = 1; i < 8; ++i) {
            minX = std::min(minX, mv[i].getX());
            minY = std::min(minY, mv[i].getY());
            minZ = std::min(minZ, mv[i].getZ());
        }

        delete[] mv;

        return Vector3(minX,minY,minZ);
    }

    /*!
     * \return The max of the aabb of this Obb
     */
    Vector3 Obb::getSurroundingAabbMax() const {
        Vector3* mv = getVertices();

        real maxX = mv[0].getX();
        real maxY = mv[0].getY();
        real maxZ = mv[0].getZ();

        for (int i = 1; i < 8; ++i) {
            maxX = std::max(maxX, mv[i].getX());
            maxY = std::max(maxY, mv[i].getY());
            maxZ = std::max(maxZ, mv[i].getZ());
        }

        delete[] mv;

        return Vector3(maxX,maxY,maxZ);
    }

    /*!
     * \brief Tests for intersection with another Obb
     * \return TRUE If this Obb intersects with \p other. Otherwise FALSE.
     * 
     * TODO: Currently only collisions beetween obb's is supported!
     */
    bool Obb::collidesWith(const BoundingVolume& other) const {
        if (other.getVolumeType() == getVolumeType()) {
            const BoundingVolume* otherPointer = &other;
            const Obb* obbPointer = static_cast<const Obb*>(otherPointer);
            return collidesWithInternal(*obbPointer);
        } 

        return false;
    }

    /*!
     * \brief internal method to check for collisions
     * Code stolen from \ref BoxBoxIntersector::getIntersectionFast 
     */
    bool Obb::collidesWithInternal(const Obb& other) const {

        /* --------------------------------------------------- *\
                         separating axis theorem
        \* --------------------------------------------------- */

        //calculate vertices of both boxes once now, they will be used
        //in several places
        Vector3* obb1vertices = getVertices();
        Vector3* obb2vertices = other.getVertices();
        // FIXME: this is plain ugly!
        // -> obb1vertices and obb2vertices must be deleted before EVERY return
        //    statement

        Matrix otherState = other.getState();

        // store intervals and axes
        real min1[15], max1[15];
        real min2[15], max2[15];
        Vector3 axis[15];

        // test all three normals in box1
        for (int i=0; i<3; i++) {
            axis[i] = mState.getAxis(i);

            computeSpan(obb1vertices, axis[i], min1[i], max1[i]);
            computeSpan(obb2vertices, axis[i], min2[i], max2[i]);

            if (!overlap(min1[i], max1[i], min2[i], max2[i])) {
                delete[] obb1vertices;
                delete[] obb2vertices;
                return false;
            }
        }

        // test all three normals in box2
        for (int i=0; i<3; i++) {
            int j = i + 3;

            axis[j] = otherState.getAxis(i);

            computeSpan(obb1vertices, axis[j], min1[j], max1[j]);
            computeSpan(obb2vertices, axis[j], min2[j], max2[j]);

            if (!overlap(min1[j], max1[j], min2[j], max2[j])) {
                delete[] obb1vertices;
                delete[] obb2vertices;
                return false;
            }
        }

        // test all of the nine cross products
        for (int i=0; i<3; i++) {
            for (int k=0; k<3; k++) {
                int j = (3*i) + k + 6;

                axis[j] = mState.getAxis(i) * otherState.getAxis(k);

                computeSpan(obb1vertices, axis[j], min1[j], max1[j]);
                computeSpan(obb2vertices, axis[j], min2[j], max2[j]);

                if (!overlap(min1[j], max1[j], min2[j], max2[j])) {
                    delete[] obb1vertices;
                    delete[] obb2vertices;
                    return false;
                }
            }
        }

        delete[] obb1vertices;
        delete[] obb2vertices;

        return true;
    }

    /*! \brief calculates the Obb for the given sphere
     */
    void Obb::adjustToSphere(const Matrix* worldState, const Sphere* sphere) {
        mState.loadMatrix(worldState->getData());
        real radius = sphere->getRadius();
        Vector3 pos = mState.getPosition();
        Vector3 min = pos-Vector3(radius,radius,radius);
        Vector3 max = pos+Vector3(radius,radius,radius);
        mDimension = max-min;
        mReferencePoint = Vector3(0,0,0);
    }

    /*! \brief calculates the Obb for the given box
     */
    void Obb::adjustToBox(const Matrix* worldState, const Box* box) {

        // This one is the easiest shape, because we can just use the box itself

        // METHOD 1:
        //mState.loadMatrix(worldState->getRotationMatrix());
        //mState.translate(const_cast<Box*>(box)->getUnrotatedCenter(),true);
        //mDimension = box->getDimension();
        //mReferencePoint = mDimension/2.0;

        // METHOD 2:
        Matrix mr = worldState->getRotationMatrix();
        mState.loadIdentity();
        mState.translate(const_cast<Box*>(box)->getUnrotatedCenter(),false);
        mDimension = box->getDimension();
        mReferencePoint = mDimension/2.0;
        rotate(mr);
    }

    /*! \brief calculates the Obb for the given cone
     */
    void Obb::adjustToCone(const Matrix* worldState, const Cone* cone) {
        //reset(const_cast<Cone*>(cone)->getMesh()->getVertices());
        // The radius and the height of the cone
        //return;
        const real radius = cone->getRadius();
        const real height = cone->getHeight();
        adjustToConeOrCylinderInternal(worldState,radius,height);
    }

    /*! \brief calculates the Obb for a cylinder
     */
    void Obb::adjustToCylinder(const Matrix* worldState,
                                const Cylinder* cylinder) {

        //reset(const_cast<Cylinder*>(cylinder)->getMesh()->getVertices());
        // The radius and the height of the cylinder:
        //return;
        const real radius = cylinder->getRadius();
        const real height = cylinder->getHeight();
        adjustToConeOrCylinderInternal(worldState,radius,height);
    }


    /*! \brief calculates the Obb for a cone or a cylinder
     * we use the same code for both shapes, the obb is the same!
     */
    void Obb::adjustToConeOrCylinderInternal(const Matrix* worldState, 
            const real& radius, const real& height) {

        // METHOD 1:
        /*mState.loadMatrix(worldState->getRotationMatrix());
        Vector3 pos = worldState->getPosition();
        //std::cout << "P: " << pos << std::endl;
        Vector3 posCenter = Vector3(pos.getX(),pos.getY(),pos.getZ()+(height/2));
        //std::cout << "PC: " << posCenter << std::endl;
        mState.translate(posCenter,false);

        Vector3 min = Vector3(pos.getX()-radius, pos.getY()-radius,
                pos.getZ());
        Vector3 max = Vector3(pos.getX()+radius, pos.getY()+radius,
                pos.getZ()+height);

        mDimension = max-min;
        mReferencePoint = posCenter-pos;
        //std::cout << min << " - " << max << " = PD: " << mDimension << std::endl;
        */

        // METHOD 2:
        Matrix mr = worldState->getRotationMatrix();
        mState.loadIdentity();
        Vector3 pos = worldState->getPosition();
        Vector3 posCenter = Vector3(pos.getX(),pos.getY(),pos.getZ()+(height/2));
        mState.translate(posCenter,false);

        Vector3 min = Vector3(pos.getX()-radius, pos.getY()-radius,
                pos.getZ());
        Vector3 max = Vector3(pos.getX()+radius, pos.getY()+radius,
                pos.getZ()+height);

        mDimension = max-min;
        mReferencePoint = posCenter-pos;
        rotate(mr);
    }

    /*! \brief calculates the Obb for a wedge
     */
    void Obb::adjustToWedge(const Matrix* worldState, const Wedge* wedge) {

        // This one is the easiest shape, because we can just use the box itself

        // METHOD 1:
        //mState.loadMatrix(worldState->getRotationMatrix());
        //mState.translate(const_cast<Wedge*>(wedge)->getUnrotatedCenter(),false);
        //mDimension = Vector3(wedge->getDimensions());
        //mReferencePoint = mDimension/2.0;

        // METHOD 2:
        Matrix mr = worldState->getRotationMatrix();
        mState.loadIdentity();
        mState.translate(const_cast<Wedge*>(wedge)->getUnrotatedCenter(),false);
        mDimension = Vector3(wedge->getDimensions());
        mReferencePoint = mDimension/2.0;
        rotate(mr);
    }

    /*! \brief calculates the Obb for a mesh
     */
    void Obb::adjustToMesh(const Mesh* mesh) {
        reset(mesh->getVertices());
        //adjustToTriangles(mesh->getTriangles(),mesh->getVertices());
    }

    /*! \brief calculates the Obb for a meshpart
     */
    void Obb::adjustToMeshPart(const MeshPart* meshpart) {
        reset(meshpart->getVertices());
        //adjustToTriangles(meshpart->getTriangles(),meshpart->getVertices());
    }

    /*! \brief calculates the Obb out of a list of triangles
     * The concept is published in the paper "A Hierarchical Structure for Rapid
     * Interference Detection" by Gottschalk, Lin & Manocha, 1996
     */
    void Obb::adjustToTriangles(const std::vector<Triangle*>& triangles, 
            const std::vector<Vertex*>& vertices) {

        const std::vector<Triangle*> convexHull =
            calculateConvexHull(triangles,vertices);

        // First: Calculate mean ... 
        Vector3 mean;
        int nt = 0; // saves number of triangles
        for (std::vector<Triangle*>::const_iterator it = triangles.begin(); 
                it != triangles.end(); ++it) {
            const array<Vertex*,3>& verts = (*it)->getVertices();
            mean += verts[0]->getWorldPosition(); 
            mean += verts[1]->getWorldPosition(); 
            mean += verts[2]->getWorldPosition(); 
            ++nt;
        }
        mean = mean/(3*nt);

        /* ... and covariance matrix C, math formula:
         *
         * C_jk = 1/3n \sum_0^nt{(p^i_j-mean)*(p^i_k-mean) 
         *                 + (q^i_j-mean)*(q^i_k-mean) 
         *                 + (r^i_j-mean)*(r^i_k-mean)}
         * - p,k and r are the vertices of each triangle
         */
        // as temporary containers:
        real covariance[3][3]; 
        Matrix A;
        Vector3 p,q,r;

        for (int i = 0; i<nt;++i) {
            const array<Vertex*,3>& verts = triangles[i]->getVertices();
            p = verts[0]->getWorldPosition() - mean; 
            q = verts[1]->getWorldPosition() - mean; 
            r = verts[2]->getWorldPosition() - mean; 
            for (int j=0;j<3;++j) {
                for (int k=0;k<3;++k) {
                    covariance[j][k] += p[j]*p[k] + q[j]*q[k] + r[j]*r[k];
                }
            }
        }
        for (int j=0;j<3;++j) {
            for (int k=0;k<3;++k) {
                covariance[j][k] /= 3*nt;
                A.setElement(j,k,covariance[j][k]);
            }
        }

        // Calculate eigenvectors
        Vector3 eigenvectors[3];

/*        A.loadMatrix(
                Vector3(3.23f,-1.15f,1.77f),
                Vector3(-1.15f,9.25f,-2.13f),
                Vector3(1.77f,-2.13f,1.56f));
        A.debugMatrix();*/

        // Mises (Bronstein, p. 293f, 2000)
        // ----------------------------------------
        // First: find eigenvector with highest eigenvalue:
        Vector3 x[16]; // x for 16 iterations, contains eigenvector
        // step1: choose a starting vector:
        x[0] = Vector3(1.0f,0.0f,0.0f); 
        // step2: iterative calc. of x^(k+1); x^(k+1) = A*x^k
        performVectorIteration(A,x,16);
        // Voila: x[15] contains the eigenvector with the highest eigenvalue:
        eigenvectors[0] = x[15];
        eigenvectors[0].normalize();

        // Seocnd: find eigenvector with the smallest eigenvalue,
        // uses inverse of A:
        Matrix invA;
        if (!A.invert(&invA)) {
            error() << dc_funcinfo << 
                "Unable to calculate inverse of covariance matrix of mesh!";
        }
        x[0] = Vector3(1.0f,0.0f,0.0f); 
        performVectorIteration(invA,x,16);
        eigenvectors[1] = x[15];
        eigenvectors[1].normalize();

        // Third: The last one, also Mises, "Deflation":
        // choose a starting vector which is perpendicular to the first
        // eigenvector:
        x[0] = Vector3(-eigenvectors[0].getY(),eigenvectors[0].getX(),0);
        performVectorIteration(A,x,16);
        eigenvectors[2] = x[15];
        eigenvectors[2].normalize();

        // use eigenvectors as basis and find extremal vertices along each axis
        // of this basis:
        real minP[3];
        real maxP[3];
        for (int i = 0;i<3;++i) {
            minP[i] = 0.0f;
            maxP[i] = 0.0f;
        }
        Vector3 meanVertice;
        real projection;
        for (int i = 0; i<nt;++i) {
            const array<Vertex*,3>& verts = triangles[i]->getVertices();
            for (int j =0;j<3;++j) {
                // vector beetween mean of mesh and vertice:
                meanVertice = mean - verts[j]->getWorldPosition();
                // project this vector onto each eigenvector:
                for (int k =0;k<3;++k) {
                   projection = meanVertice.dotProduct(eigenvectors[j]);
                    // If projection is max -> corresponding vertice is extrema
                    if (projection > maxP[j]) {
                        maxP[j] = projection;
                    }
                    // If projection is min -> corresponding vertice is extrema
                    if (projection < minP[j]) {
                        minP[j] = projection;
                    }
                }
            }
        }

        // now as we have the eigenvectors and the extremas, we can size the
        // bounding volume:
        Vector3 minV;
        Vector3 maxV;
        for (int i = 0;i<3;++i) {
            maxV += eigenvectors[i]*maxP[i];
            minV += eigenvectors[i]*minP[i];
        }

        // size obb: 
        mDimension = maxV-minV;
        Vector3 pos = maxV+minV/2;

        // orient with the previously calculated basis: 
        mState.loadMatrix(eigenvectors[0],eigenvectors[1],eigenvectors[2]);
        mState.translate(pos,false);
    }

    /*!
     * \breif Adjust this Obb to an object described by \p points, transformed by \p worldState.
     *
     * This method is mainly a convenience method for other adjustTo*() methods.
     * In particular it aids at Obb approximations: first calculate an Obb
     * around the original (untransformed) method, retrieve all 8 points from it
     * and then call this method. The points are then transformed by the
     * transformation matrix and a new Obb matrix is calculated from it. The
     * resulting Obb is usually larger than necessary, but the error is
     * controllable.
     */
    void Obb::adjustToPoints(const Matrix* worldState, const std::list<Vector3>& points) {
        std::list<Vector3> transformedPoints;
        for (std::list<Vector3>::const_iterator it = points.begin(); it != points.end(); ++it) {
            Vector3 p;
            worldState->transform(&p, *it);
            transformedPoints.push_back(p);
        }

        // calculate AABB of transformed points
        reset(transformedPoints);
    }

    /*! \brief Merges this Obb with another one
     * TODO: atm. aabb is used to calculate merged bv!
     */
    void Obb::mergeWith(const BoundingVolume* otherBV) {
        Vector3* vs1 = getVertices();
        Vector3* vs2 = static_cast<const Obb*>(otherBV)->getVertices();
        std::list<Vector3> points;
        for (int i = 0;i<8;++i) {
            points.push_back(vs1[i]);
            points.push_back(vs2[i]);
        }
        delete[] vs1;
        delete[] vs2;
        reset(points);
    }

    /*!
     * \brief calculates the intersection with another Obb
     * 
     * OWNERSHIP NOTICE: The caller must take care of deleting the created Obb
     * \return NULL if the Obb do not intersect
     * FIXME: adjust to OBB!
     */
    Obb* Obb::intersectWith(const Obb& other) const{
        Obb* newObb = new Obb();
        error() << dc_funcinfo << ": Is not yet implemented for obb's!";
        return newObb;
    }

    /*!
     * \brief calculates the intersection of two Obb
     * 
     * OWNERSHIP NOTICE: The caller must take care of deleting the created Obb
     * \return NULL if the Obb do not intersect
     */
    Obb* Obb::calculateIntersection(  const Obb& obb1,
                                        const Obb& obb2) {
        return obb1.intersectWith(obb2);
    };

    /*
     * FIXME: Adjust to Obb
     */
    void Obb::adjustToTriangle(const Triangle* triangle) {
        /*std::vector<Triangle*> triangles;
        triangles.push_back(const_cast<Triangle*>(triangle));
        adjustToTriangles(triangles,triangle->getVertices());*/
    }

    /*!
     * \brief calculates and returns all 8 vertices in world space
     * 
     * The figure below shows the order in which the vertices are stored
     * (in respect to the coordinate system illustrated on the right hand
     * side):
     *
     *   6------7     
     *  /|     /|     y
     * 2------3 |     ^
     * | |    | |     |  / z
     * | 4------5     | /
     * |/     |/      |/
     * 0------1       +------> x
     *
     * Code stolen from \ref Box::getVetices()
     * OWNERSHIP NOTICE: The caller takes ownership of the returned array!
     */
    // FIXME: the name prefix "get" is not really nice for a method returning a
    // _new_ array
    Vector3* Obb::getVertices() const {

        Vector3* vertices = new Vector3[8];
        //GJ: to save some Matrix multiplications, we calculate 3 edge vectors
        //and generate the vertices by vector-addition

#if 1
        Vector3 foo1[8];
        foo1[0] = Vector3(-mDimension.getX() / 2.0, -mDimension.getY() / 2.0, -mDimension.getZ() / 2.0);
        foo1[1] = Vector3(mDimension.getX() / 2.0, -mDimension.getY() / 2.0, -mDimension.getZ() / 2.0);
        foo1[2] = Vector3(-mDimension.getX() / 2.0, mDimension.getY() / 2.0, -mDimension.getZ() / 2.0);
        foo1[3] = Vector3(mDimension.getX() / 2.0, mDimension.getY() / 2.0, -mDimension.getZ() / 2.0);
        foo1[4] = Vector3(-mDimension.getX() / 2.0, -mDimension.getY() / 2.0, mDimension.getZ() / 2.0);
        foo1[5] = Vector3(mDimension.getX() / 2.0, -mDimension.getY() / 2.0, mDimension.getZ() / 2.0);
        foo1[6] = Vector3(-mDimension.getX() / 2.0, mDimension.getY() / 2.0, mDimension.getZ() / 2.0);
        foo1[7] = Vector3(mDimension.getX() / 2.0, mDimension.getY() / 2.0, mDimension.getZ() / 2.0);

        // mState.translate(-mReferencePoint);
        for (int i = 0; i < 8; i++) {
            mState.transform(&vertices[i], foo1[i]);
        }
        // mState.translate(mReferencePoint);
        //Vector3 center = mState.getPosition();
        //std::cout << "center: " << center << " v0:" << vertices[0] << " v7: " <<
        //    vertices[7] << std::endl;

#else
        Vector3 center = mState.getPosition();
#ifdef OBB_DEBUG
        std::cout << "center: " << center << std::endl;
#endif

        //Matrix mr = mState.getRotationMatrix();
        //Vector3 dim2;
        //mr.transform(&dim2,mDimension/2);
        vertices[0] = center-mDimension/2;
        //mr.transform(&vertices[0],(center-mDimension/2));

        //edge vector i is the i-th column of the rotation matrix
        Vector3 xEdge(  mState.getElement(0,0) * mDimension.getX(),
                        mState.getElement(1,0) * mDimension.getX(),
                        mState.getElement(2,0) * mDimension.getX());
        Vector3 yEdge(  mState.getElement(0,1) * mDimension.getY(),
                        mState.getElement(1,1) * mDimension.getY(),
                        mState.getElement(2,1) * mDimension.getY());
        Vector3 zEdge(  mState.getElement(0,2) * mDimension.getZ(),
                        mState.getElement(1,2) * mDimension.getZ(),
                        mState.getElement(2,2) * mDimension.getZ());

        vertices[1] =  vertices[0] + xEdge;
        vertices[2] =  vertices[0] + yEdge;
        vertices[3] =  vertices[0] + xEdge + yEdge;
        vertices[4] =  vertices[0] + zEdge;
        vertices[5] =  vertices[0] + xEdge + zEdge;
        vertices[6] =  vertices[0] + yEdge + zEdge;
        vertices[7] =  vertices[0] + xEdge + yEdge +zEdge;

#ifdef OBB_DEBUG
        for (int i =0;i<8;++i) {
            std::cout << "C"<< i << ": " << vertices[i] << std::endl;
        }
#endif
#endif

        return vertices;
    }

    /*!
     * \brief Compute range of projection of a \p box onto an \p axis.
     */
    void Obb::computeSpan(const Vector3* vertices, const Vector3& axis, real& min, real& max) const {
        min = max = axis.dotProduct(vertices[0]);

        for (int i=1; i<8; i++) {
            real d = axis.dotProduct(vertices[i]);

            if (d < min) {
                min = d;
            }

            if (d > max) {
                max = d;
            }
        }
    }

    /*!
     * \brief Perform vector iteration (Mises)
     * iterative calc. of x^(k+1); x^(k+1) = A*x^k
     * \param A the matrix of which we want to calculate an eigenvector
     * \param x contains the iterated vectors
     * \param sizeOf The amount of iterations wanted, also the size of the array
     *        \p x.
     * FIXME: perhaps stop iteration when the change beetween old and new
     *        value is below a defined value (e.g. 0.001) ?
     */
    void Obb::performVectorIteration(const Matrix& A,Vector3* x,int sizeOf) {
        //std::cout << "x0: " << x[0] << std::endl;
        int i;
        for (i=1;i<sizeOf;++i) {
            x[i] = A.multiply(x[i-1]);
            // additionally following hint 1. (in Bronstein, p.295,2000) after 
            // each third iteration (normalizing x^(k+1)):
            if ((i % 3) == 0) {
                x[i] = x[i]/x[i].getX();
            }
        }
        // If amount of iterations is not modulo 3, normalize!
        if (((i-1) % 3) != 0) {
            x[i] = x[i]/x[i].getX();
        }
        //std::cout << "x" << (i-1) << ": " << x[i-1] << std::endl;
    }

    /*!
     * \brief Calculates the convex hull of a mesh given by \p triangles
     * \param triangles The triangles of the mesh of which we want to calculate 
     * the convex hull
     * 
     * Uses the quickhull algorithm by Barber, Dobkin & Huhdanpaa, presented in
     * the paper "The quickhull algorithm for convex hulls", 1989
     */
    std::vector<Triangle*> Obb::calculateConvexHull(
            const std::vector<Triangle*>& triangles, 
            const std::vector<Vertex*>& vertices) const {

        std::vector<Triangle*> convexHull;
        std::list<Vector3> unassignedPoints;
        std::vector<struct OutsideSet> outsideSet;

        // creating list of unassigned points:
        for (std::vector<Vertex*>::const_iterator it = vertices.begin(); 
            it != vertices.end(); ++it) {
            unassignedPoints.push_back((*it)->getWorldPosition());
        }

        // foreach facet F
        for (std::vector<Triangle*>::const_iterator it = triangles.begin(); 
                it != triangles.end(); ++it) {
            // creating facet:
            Vector3 normal = ((*it)->getVertices()[0]->getWorldPosition()
                * (*it)->getVertices()[1]->getWorldPosition()).normalize();
            real distanceFromOrigin = -Vector3::dotProduct(normal,
                (*it)->getVertices()[0]->getWorldPosition());
            bool hasSet = false;
            std::list<std::pair<Vector3,real> > pointList;
            std::pair<Vector3,real> furthestPoint;
            // to determine the furthest point: 
            real maxDistance = 0.0f;
            // foreach unassigned point p
            for (std::list<Vector3>::iterator p 
                    = unassignedPoints.begin(); 
                p != unassignedPoints.end(); ++p) {
                // If p lies above triangle
                real distance = (*p).dotProduct(normal) + distanceFromOrigin;
                if (distance >= 0.0f) {
                    if (distance < maxDistance) {
                        // assign p to triangles outside set
                        pointList.push_back(std::pair<Vector3,real>(*p,distance));
                    } else {
                        // add last furthest point to list, as it is no longer
                        // the furthest!
                        pointList.push_back(furthestPoint);
                        maxDistance = distance;
                        furthestPoint = std::pair<Vector3,real>(*p,distance);
                    }
                    // remove from list of unassigned points
                    unassignedPoints.erase(p);
                    hasSet = true;
                }
            }
            if (hasSet) {
                struct OutsideSet set(*it,pointList,furthestPoint);
                outsideSet.push_back(set);
            }
        } // for (std::vector<Triangle*>::const_iterator it = triangles.begin

        // foreach facet F with a non empty outside set
        for (std::vector<struct OutsideSet>::const_iterator 
                F = outsideSet.begin(); F != outsideSet.end(); ++F) {
            // select furthest point p of F's outside set:
            Vector3 p = (*F).furthestPoint.first;
            // initialize the visible set V to F:
            std::list<Triangle*> V;
            V.push_back((*F).triangle);
            // for all unvisited neighbors N of facets in V:
            std::set<Triangle*> neighbors = (*F).triangle->getEdgeAdjacentTriangles();
            for (std::set<Triangle*>::iterator 
                N = neighbors.begin(); N != neighbors.end(); ++N) {
                Vector3 normal = ((*N)->getVertices()[0]->getWorldPosition()
                    * (*N)->getVertices()[1]->getWorldPosition()).normalize();
                real distanceFromOrigin = -Vector3::dotProduct(normal,
                    (*N)->getVertices()[0]->getWorldPosition());
                // if p is above N
                real distance = p.dotProduct(normal) + distanceFromOrigin;
                if (distance >= 0.0f) {
                    // add N to V
                    V.push_back(*N);
                }
            } // END for (std::Set<Triangle*>::iterator 

            // the boundary of V is the set of horizon ridges (aka edges) H

        } // END for (std::vector<struct OutsideSet>::const_iterator 

        return convexHull;

    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Obb& v) {
        os << "[ c = " << v.getCenter() << ", d = " << v.getDimension() << "]";
        return os;
    }
}


/*
 * vim: et sw=4 ts=4
 */
