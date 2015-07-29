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

#ifndef DCOLLIDE_TRIANGLE_H
#define DCOLLIDE_TRIANGLE_H

#include "detectordeform/surfacehierarchy/normalcone.h"
#include "math/vector.h"
#include "thread/thread.h"

#include "datatypes/array.h"
#include <list>
#include <vector>
#include <set>

namespace dcollide {

    //-----------classes------------

    /*! forward declarations
     */
    class Vertex;
    class BoundingVolume;
    class BvhNode;


    /*!
     * \brief Representation of a triangle
     */
    class Triangle {
        private:
            /*!
             * \brief Tells the class that the member variable mNormals is
             * either initialized with correct values or not.
             *
             * True : mNormals is initialized
             * False: mNormals isn't initailized
             */
            bool mNormalsInitialized;

            /*!
             * \brief Tells the class that the member variable mNormalVector is
             * either initialized with correct values or not.
             *
             * True : mNormalVector is initialized
             * False: mNormalVector isn't initialized
             */
            bool mNormalVectorInitialized;

            /*!
             * \brief Tells the class that the member variable
             * mWorldCoordinatesNormalVector is either initialized with correct
             * values or not.
             *
             * True : mWorldCoordinatesNormalVector is initialized
             * False: mWorldCoordinatesNormalVector isn't initialized
             */            
            bool mWorldCoordinatesNormalVectorInitialized;

            /*!
             * \brief Tells the class that the member variable mNormalCone is
             * either initialized with correct values or not.
             *
             * True : mNormalCone is initialized
             * False: mNormalCone isn't initialized
             */
            bool mNormalConeInitialized;


            /*!
             * \brief Precalculated normal vector
             */
            Vector3 mNormalVector;

            /*!
             * \brief Cached normal vectors in world coordinates, depends on
             * \p mNormalVector.
             */
            Vector3 mWorldCoordinatesNormalVector;

            /*!
             * \brief Precalculated normal cone, depends on \p mNormalVector
             * FIXME: This will be reworked in some way, it is more sensible to move
             * the normal-cone to deformable-bvh-node
             *
             * Normal-Cones are useful to accelerate deformable collision detection
             * algorithms, which rely on a bottom-up generated hierarchy. They contain
             * information about the curvature of a region. In this case the normal-cone
             * is identical to the normal vector of the triangle, with an angle of zero.
             */
            NormalCone mNormalCone;

            /*!
             * \brief Array of exact three members which represents the three
             *        vertices of this triangle
             */
            dcollide::array<Vertex*,3> mVertices;

            /*!
             * \brief Vector of exact three members which represents the three
             *        normal vectors for each vertex of this triangle.
             */
            std::vector<Vector3*> mNormals;

            /*!
             * \brief List of all bounding volumes that encloses this triangle
             */
            std::list<const BvhNode*> mBvHierarchyNodes;

            /*!
             * \brief The direct neighbors (edge neighbors) of this triangle
             */
            std::set<Triangle*> mEdgeAdjacentTriangles;

            /*!
             * \brief The triangles that share a vertex with this triangle
             */
            std::set<Triangle*> mVertexAdjacentTriangles;

            mutable Mutex mMutex;

            /*!
             *\brief edge from vertex 0 to vertex 1 in world coordinates
             * 
             * set by \ref calculateEdgeData and used in \ref containsPoint()
             */
            Vector3 mEdge01;
            /*!
             *\brief edge from vertex 1 to vertex 2 in world coordinates
             * 
             * set by \ref calculateEdgeData
             */
            Vector3 mEdge12;
            /*!
             *\brief edge from vertex 0 to vertex 2 in world coordinates
             * 
             * set by \ref calculateEdgeData and used in \ref containsPoint()
             */
            Vector3 mEdge02;
            /*!
             *\brief dot product of mEdge01 * mEdge01
             * 
             * set by \ref calculateEdgeData and used in \ref containsPoint()
             */
            real mDot0101;
            /*!
             *\brief dot product of mEdge0 * mEdge 02
             * 
             * set by \ref calculateEdgeData and used in \ref containsPoint()
             */
            real mDot0102;

            /*!
             *\brief dot product of mEdge02 * mEdge02
             * 
             * set by \ref calculateEdgeData and used in \ref containsPoint()
             */
            real mDot0202;
            
        public:
            Triangle(Vertex* v1, Vertex* v2, Vertex* v3);

            inline Triangle(const std::vector<Vertex*>& vertices,
                            const std::vector<Vector3*>& normals);

            inline Triangle(Vertex* v1, Vector3* n1,
                            Vertex* v2, Vector3* n2,
                            Vertex* v3, Vector3* n3);

            ~Triangle();


            inline const dcollide::array<Vertex*,3>&  getVertices() const;
            inline const std::vector<Vector3*>&  getNormals();
            inline const Vector3& getNormalVector() const;
            inline const Vector3& getWorldCoordinatesNormalVector() const;
            inline const NormalCone* getNormalCone();
            inline const std::list<const BvhNode*>& getBvHierarchyNodes() const;
            inline const std::set<Triangle*>& getEdgeAdjacentTriangles() const;
            inline const std::set<Triangle*>& getVertexAdjacentTriangles() const;
            inline const Vector3& getEdge01() const;
            inline const Vector3& getEdge02() const;
            inline const Vector3& getEdge12() const;
                        
            inline void setEdgeAdjacentTriangles(Triangle* t1,
                                             Triangle* t2,
                                             Triangle* t3);
            inline void addVertexAdjacentTriangle(Triangle* t);

            void setNormals(const std::vector<Vector3*>& normals);

            inline void addBvHierarchyNode(const BvhNode* node);
            inline void removeBvHierarchyNode(const BvhNode* node);

            bool isVertexAdjacentTo(const Triangle* t) const;
            bool isEdgeAdjacentTo(const Triangle* t) const;

            bool hasElement(const Vertex* v) const;

            inline void invalidateNormals();
            inline void invalidateWorldCoordinatesNormalVector();

            void calculateEdgeData();
            bool containsPoint(const Vector3& point,
                                bool skipPlaneTest = false,
                                bool usePrecachedEdgeData = false,
                                real* outU = 0, real* outV = 0) const;
        private:
            void init(Vertex* v1, Vector3* n1,
                                Vertex* v2, Vector3* n2,
                                Vertex* v3, Vector3* n3);
            void updateNormals();
            void updateNormalVector();
            void updateNormalCone();
            void updateWorldCoordinatesNormalVector();
    };


    //------------ Implementation of short methods -------------

    /*!
     * Constructs a triangle consisting of \p vertices and \p normals.
     * 
     * The \p vertices must be a vector of exactly 3 vertex pointers - all of
     * these must be added to the same mesh as this triangle.
     * 
     * The \p normals must be a vector of excat three vector3 pointers,
     * representing the normal vectors of each vertex. 
     *
     * OWNERSHIP NOTICE:
     * This class does NOT take ownership of the vertex pointers in \p
     * vertices, but do take ownership of the vector pointers in \p normals.
     */
    Triangle::Triangle(const std::vector<Vertex*>& vertices,
                       const std::vector<Vector3*>& normals) {

        init(vertices[0], normals[0],
             vertices[1], normals[1],
             vertices[2], normals[2]);
    }

    /*!
     * \overload
     * 
     * Constructs a triangle consisting of the three vertex, normal pairs
     * (v1, n1), (v2, n2) and (v3, n3).
     * 
     * OWNERSHIP NOTICE:
     * This class does NOT take ownership of the vertex pointers in \p v1,
     * \p v2 and \p v3, but do take ownership of the vector pointers in
     * \p n1, \p n2 and \p n3.
     * 
     */
    Triangle::Triangle(Vertex* v1, Vector3* n1,
                       Vertex* v2, Vector3* n2,
                       Vertex* v3, Vector3* n3) {

        init(v1, n1, v2, n2, v3, n3);
    }


    /*!
     * \return An array of exactly 3 vertex pointers describing this triangle.
     */
    const dcollide::array<Vertex*,3>& Triangle::getVertices() const {
        return mVertices;
    }



    /*!
     * \return A vector of exactly 3 vector pointers describing the normal
     *         vectors for the vertices of this triangle. (Normally the chached
     *         normal vector is returned, when no valid cached version exists
     *         one is calculated)
     */
    const std::vector<Vector3*>&  Triangle::getNormals() {
        MutexLocker lock(&mMutex);

        if (!mNormalsInitialized) {
            updateNormals();
        }

        return mNormals;
    }

    /*!
     * \return A valid normal vector acording to the plane which is spanned by
     *         the three triangle vertices. This value is cached and so not
     *         successive calls to this function won't hurt.
     */
    const Vector3& Triangle::getNormalVector() const {
        MutexLocker lock(&mMutex);

        if (!mNormalVectorInitialized) {
            const_cast<Triangle*>(this)->updateNormalVector();
        }

        return mNormalVector;
    }

    /*!
     * \return A valid normal vector like Triangle::getNormalVector(), but
     *         transforms it into world coorinates. The calculated vectors are
     *         cached.
     */
    const Vector3& Triangle::getWorldCoordinatesNormalVector() const {
        MutexLocker lock(&mMutex);

        if (!mWorldCoordinatesNormalVectorInitialized) {
            const_cast<Triangle*>(this)->updateWorldCoordinatesNormalVector();
        }

        return mWorldCoordinatesNormalVector;
    }

    /*!
     * \return A list of all triangles sharing an edge with this triangle.
     */
    const std::set<Triangle*>& Triangle::getEdgeAdjacentTriangles() const {
        return mEdgeAdjacentTriangles;
    }

    /*!
     * \return A list of all triangles sharing at least one vertex with
     *         this triangle.
     */
    const std::set<Triangle*>& Triangle::getVertexAdjacentTriangles() const {
        return mVertexAdjacentTriangles;
    }

    /*!
     * \brief This function is used to add adjacency information to the mesh
     * Each triangle may be edge-adjacent to at least three other triangles, opposed
     * to the number of vertex-adjacent triangles, which may be arbitrary.
     * \p Triangle* t1, t2, t3
     */
    void Triangle::setEdgeAdjacentTriangles(Triangle* t1,
                                             Triangle* t2,
                                             Triangle* t3) {
        //FIXME: call this function from mesh::initAdjacency
        if (t1 != 0){
            mEdgeAdjacentTriangles.insert(t1);
        }
        if (t2 != 0){
            mEdgeAdjacentTriangles.insert(t2);
        }
        if (t3 != 0){
            mEdgeAdjacentTriangles.insert(t3);
        }
    }

    /*!
     * \return A normal cone which represents the orientation of this triangle.
     *         The calculated normal cone is cached and returned on successive
     *         calls.
     */
    const NormalCone* Triangle::getNormalCone() {
        MutexLocker lock(&mMutex);

        if (!mNormalConeInitialized)  {
            updateNormalCone();
        }

        return &mNormalCone;
    }

    /*!
     * \return The hierarchy node objects which are associated with this
     *         triangle.
     */
    const std::list<const BvhNode*>& Triangle::getBvHierarchyNodes() const {
        return mBvHierarchyNodes;
    }

    /*!
     * \brief Adds an hierarchie node to the list of nodes that enclose
     *        this triangle.
     * 
     * OWNERSHIP NOTICE:
     * The triangle doesn't take ownership of the HierarchieNode Pointer
     * \p node.
     */
    void Triangle::addBvHierarchyNode(const BvhNode* node) {
        mBvHierarchyNodes.push_back( node );
    }

    /*!
     * \brief Removes an hierarchie node from the list of nodes that enclose
     *        this triangle.
     */
    void Triangle::removeBvHierarchyNode(const BvhNode* node) {
        mBvHierarchyNodes.remove( node );
    }


    /*!
     * Invalidates all cached versions of normal vectors. This includes vertex
     * normals, normal vectors, normal cones and their corresponding transformed
     * ones in world coordinates.
     */
    void Triangle::invalidateNormals() {
        mNormalsInitialized = false;
        mNormalConeInitialized = false;
        mNormalVectorInitialized = false;
        mWorldCoordinatesNormalVectorInitialized = false;
    }

    /*!
     * Invalidates only the normal vector which was transformed in world
     * coordinates. This could be needed e.g. if the mesh which contains this
     * traingle was moved.
     */ 
    void Triangle::invalidateWorldCoordinatesNormalVector() {
        mWorldCoordinatesNormalVectorInitialized = false;
    }

    /*!
     */
    void Triangle::addVertexAdjacentTriangle(Triangle* t) {
        mVertexAdjacentTriangles.insert(t);
    }
    
    /*!
     * \brief get mEdge01. Call calculateEdgeData before calling this!
     */
    const Vector3& Triangle::getEdge01() const {
        return mEdge01;
    }
    /*!
     * \brief get mEdge02. Call calculateEdgeData before calling this!
     */
    const Vector3& Triangle::getEdge02() const {
        return mEdge02;
    }
    /*!
     * \brief get mEdge12. Call calculateEdgeData before calling this!
     */
    const Vector3& Triangle::getEdge12() const {
        return mEdge12;
    }
}

#endif // DCOLLIDE_TRIANGLE_H
/*
 * vim: et sw=4 ts=4
 */
