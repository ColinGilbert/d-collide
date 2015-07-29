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

#ifndef DCOLLIDE_VERTEX_H
#define DCOLLIDE_VERTEX_H

#include "math/vector.h"
#include "math/matrix.h"
#include "thread/thread.h"
#include "datatypes/list.h"

#include <list>
#include "assert.h"


namespace dcollide {

    //-----------classes------------

    /*! forward declarations
     */
    class Mesh;
    class Triangle;
    class DebugStream;

    /*!
     * \brief Representation of a vertex
     */
    class Vertex {
        public:
            Vertex(const Vector3& position);
            Vertex(real x, real y, real z);

            inline void setAdjacency(const std::list<Vertex*>& adjacentVertices,
                                     const std::list<Triangle*>& adjacentTriangles);

            inline void addAdjacentTriangle (Triangle* t);
            inline void addAdjacentVertex (Vertex* v);

            inline const Vector3& getPosition() const;
            const Vector3& getWorldPosition() const;
            void updateWorldPosition();
            void invalidateWorldPosition();

            void setPosition(real x, real y, real z);
            inline void setPosition(const Vector3& vector);

            inline void setContainingMesh(Mesh* mesh);
            inline Mesh* getContainingMesh() const;
            inline void setVertexIndex(int index);
            inline int getVertexIndex() const;

            void translate(const Vector3& translateBy);
            inline const std::list<Vertex*>& getAdjacentVertices() const;
            inline const std::list<Triangle*>& getAdjacentTriangles() const;

            inline bool operator==(const Vertex& v) const;

        protected:
            inline ListNode<Vertex*>* getMeshListNode();
            inline void setMeshListNode(ListNode<Vertex*>* node);

            friend class Mesh;

        private:
            /*!
             * \brief Coordinates of this vertex
             */
            Vector3 mPosition;

            /*!
             * \brief Indicator if the cached value mWorldPosition is valid
             */
            bool mIsWorldPositionValid;

            /*!
             * \brief Precalculated/cached world coordinates of this vertex
             */
            Vector3 mWorldPosition;

            /*!
             * \brief A pointer to the mesh which contains this vertex
             */
            Mesh* mContainingMesh;

            /*!
             * \brief Contains the position in the mValidWorldPositionVertices
             *        of the parental Mesh
             */
            ListNode<Vertex*>* mMeshListNode;

            /*!
             * \brief List of precalculated adjecent vertices
             */
            std::list<Vertex*> mAdjacentVertices;

            /*!
             * \brief List of all triangles which contains this vertex
             */
            std::list<Triangle*> mAdjacentTriangles;

            /*!
             * \brief Index of this vertex in the list of vertices of the \ref
             * mContainingMesh
             */
            int mVertexIndex;

            void invalidateTriangleNormals() const;
            void invalidateTriangleWorldCoordinatesNormalVector() const;

            /*!
             * \brief Mutex which should prevent the unnecessarily recalculation 
             *        of the world position in the Vertex::updateWorldPosition()
             *        method.
             */
            mutable Mutex mMutex;
    };


    //------------ Implementation of short methods -------------

    /*!
     * \brief Returns the current position/coordinates of the vertex.
     */
    const Vector3& Vertex::getPosition() const {
        return mPosition;
    }

    /*!
     * \brief Compares two Vertices semantically
     * 
     * Two vertices are equal iff their component vectors are equal
     */
    bool Vertex::operator==(const Vertex& v) const {
        if(v.getPosition() == mPosition) {
            return true;
        }
        else {
            return false;
        }
    }


    /*!
     * \brief Simple setter for the containing mesh
     *
     * Internal method, used by \ref Mesh. You should \em not call this manually
     *
     * If you call this method after the \ref Vertex has already been added (\em
     * not recommended!) you (at least) need to update the \ref getVertexIndex,
     * too.
     */
    void Vertex::setContainingMesh(Mesh* mesh) {
        mContainingMesh = mesh;
        mVertexIndex = -1;
    }

    /*!
     * \return A pointer to the \ref Mesh that owns this \ref Vertex (see also
     * the \ref Mesh constructors) or NULL if this \ref Vertex is not owned by
     * any \ref Mesh.
     */
    inline Mesh* Vertex::getContainingMesh() const {
        return mContainingMesh;
    }

    void Vertex::setPosition(const Vector3& vector) {
        setPosition(vector.getX(), vector.getY(), vector.getZ());
    }

    /*!
     * \brief Returns the list of precalculated adjecent vertices.
     */
    const std::list<Vertex*>& Vertex::getAdjacentVertices() const {
        return mAdjacentVertices;
    }

    /*!
     * \brief Returns the list of triangles which contains this vertex.
     */
    const std::list<Triangle*>& Vertex::getAdjacentTriangles() const {
        return mAdjacentTriangles;
    }

    /*!
     * \brief sets mAdjacentVertices and mAdjacentTriangles
     * call this if you have adjaceny information of the graph available.
     * If this is not called, the library will calculate the information
     * automatically, but it might take a long time to do this
     *
     * IMPORTANT: call this function for ALL vertices of the mesh or for NONE.
     *            If you called it for the first vertex of the mesh, the library
     *            expects all vertices to be initialized  - no further
     *            time-consuming validation will be done.
     *            Ignoring this warning may lead to wrong results and crashes!
     *
     *\section devnote: This is a user-only function. It changes the structure
     *                  of the graph and should not be called more than once
     */
    void Vertex::setAdjacency(  const std::list<Vertex*>& adjacentVertices,
                                const std::list<Triangle*>& adjacentTriangles) {
        mAdjacentVertices = adjacentVertices;
        mAdjacentTriangles = adjacentTriangles;
    }

    /*!
     * \internal for use in preprocessing/generating meshes
     * \brief Adds a triangle to the list of adjecent triangles
     */
    void Vertex::addAdjacentTriangle (Triangle* t) {
        mAdjacentTriangles.push_back(t);
    }

    /*!
     * \internal for use in preprocessing/generating meshes
     * \brief Adds a vertex to the list of adjecent vertices
     */
    void Vertex::addAdjacentVertex (Vertex* v) {
        mAdjacentVertices.push_back(v);
    }


    /*!
     * Set the vertexindex of this vertex. That is the index of this \ref Vertex
     * object in the list of vertices of the \ref Mesh that owns this \ref
     * Vertex. See also \ref getVertexIndex
     */
    inline void Vertex::setVertexIndex(int index) {
        mVertexIndex = index;
    }

    /*!
     * \return The index of this \ref Vertex in the list of vertices of the \ref
     * Mesh that owns this \ref Vertex (see \ref getContainingMesh) or -1 if
     * this \ref Vertex is not owned by any \ref Mesh. Note that this index
     * applies to the \ref getContainingMesh only, even if this vertex is also
     * in a different \ref Mesh (or \ref MeshPart). Only the \ref
     * getContainingMesh is the owner of this vertex.
     */
    inline int Vertex::getVertexIndex() const {
        return mVertexIndex;
    }

    inline ListNode<Vertex*>* Vertex::getMeshListNode() {
        return mMeshListNode;
    }

    inline void Vertex::setMeshListNode(ListNode<Vertex*>* node) {
        mMeshListNode = node;
    }

    DebugStream operator<<(DebugStream s, const Vertex& v);
}


#endif // DCOLLIDE_VERTEX_H
/*
 * vim: et sw=4 ts=4
 */
