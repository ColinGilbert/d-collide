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

#ifndef DCOLLIDE_MESH_H
#define DCOLLIDE_MESH_H

#include "shapes/mesh/vertex.h"
#include "shapes/shape.h"
#include "real.h"
//#include "datatypes/set.h"
#include "datatypes/list.h"
#include "thread/thread.h"
#include "math/vector.h"
#include "proxy.h"
#include "world.h"
#include "worldparameters.h"

#include <vector>
#include <list>
#include <set>

namespace dcollide {

    //-----------classes------------
    class Vertex;
    class Line;
    class Triangle;
    class DeformVector;

    /*!
     * \brief Spezialized Shape implementation in form of an mesh.
     */
    class Mesh : public Shape {
        public:
            Mesh(   const std::vector<Vertex*>& vertices,
                    const std::vector<Triangle*>& triangles);
            Mesh(   const std::vector<Vertex*>& vertices,
                    const std::vector<int>& indices);
            Mesh(   const std::vector<Vertex*>& vertices,
                    const std::vector<Vector3*>& normals,
                    const std::vector<int>& indices);
            ~Mesh();

            void deform();
            void deform(const std::vector<Vector3>& deformVector);
            void addDeformVector(unsigned int vertexIndex, const Vector3& deformVector);
            void discardDeformVectors();

            void setVertexPosition(unsigned int vertexIndex, const Vector3& position);
            void setVertexPositions(const std::vector<Vector3>& vertexPositionArray);

            void updateAverageSideLength();

            void invalidateAllWorldPositions();
            inline void addValidWorldPositionVertex(Vertex* v);

            inline virtual ShapeType getShapeType() const;

            inline const std::vector<Vertex*>& getVertices() const;
            inline const std::set<Line*>& getLines() const;
            inline const std::vector<Triangle*>& getTriangles() const;
            inline unsigned int getVertexCount() const;
            inline unsigned int getTriangleCount() const;
            inline real getAverageSideLength() const;

            inline const Mesh* getMeshIfExists() const;
            inline const Mesh* getMesh();
            
            inline bool hasLocalCoordinates() const;
            Vector3 getMidPoint() const;
            
            Mesh* cloneMesh() const;

            void storeLines();

            static Mesh* createMeshWithoutOwnership(
                    const std::vector<Vertex*>& vertices,
                    const std::vector<Triangle*>& triangles);

        protected:
            inline void invalidateVertexWorldPosition(Vertex* vertex);

            friend class Vertex;

        private:
            /*!
             * \brief This one lists all included vertices of the mesh
             */
            std::vector<Vertex*> mVertices;
            /*!
             * \brief This one lists all included lines of the mesh
             * 
             * All lines which are included in the mesh are listed here.
             * If those information are not given, the pointer is set to 0.
             */
            std::set<Line*>* mLines;
            /*!
             * \brief This one lists all included triangles of the mesh
             */
            std::vector<Triangle*> mTriangles;

            /*!
             * \brief A deformation vector for each vertex
             * 
             * Each vertex has a own deforming vector, so the size of this must
             * be equal to the size of mVertices.
             */
            std::vector<Vertex*> deformVector;
            /*!
             * \brief The Mesh knows its average side length
             * 
             * The Mesh knows its average side length \n
             * (needed in the spatial hash implementation for deformable objects)
             */
            real mAverageSideLength;
            /*!
             * \brief A list of vertices which have legal world positions
             * 
             * A list of vertices which have legal world positions.\n
             * This list is needed in the case the mesh is translated. In this
             * case you don't have to go through all vertices and tell them
             * their cached position is now illegal but only to those, which
             * have recalculated their position in the meantime and still
             * believed their position is legal.
             */
            List<Vertex*> mValidWorldPositionVertices;

            /*!
             * \brief Flag which indicates the ownership of the vertices and triangles
             */
            bool mIsVertexAndTriangleOwner;
            
            /*!
             * \brief List of deform vectors which should be applied during the
             *        next call of deform()
             */
            List<DeformVector> mDeformVectors;
            
            /*!
             * \brief Mutex to prevent that 2 Threads simultanously work with the std::vector
             */
            Mutex mMutex;
            
            //private Mesh constructor for use with meshPart
            Mesh(   const std::vector<Vertex*>& vertices,
                    const std::vector<Triangle*>& triangles, bool takeOwnership);
            
            void init(bool takeOwnership);
            void initAdjacencyLists();
    };
    /*!
     * \brief Part of a mesh. Internal use only.
     * \internal internal use only. Do not create Proxies with this as a shape! 
     * A MeshPart is build from an existing Mesh (the reference mesh) and cannot
     * exist without it.
     * It contains some of the triangles of the reference mesh.
     */
    class MeshPart : public Shape {
        public:
            MeshPart(Mesh* mesh, std::list<int>& triangleIndices); // SLOW!
            MeshPart(Mesh* mesh, std::list<int>& triangleIndices, std::vector<Vertex*>& vertices);
            MeshPart(Mesh* mesh, std::list<int>& triangleIndices,
                     int splitDirection, std::vector<Vertex*>& vertices);
            ~MeshPart();
            inline virtual ShapeType getShapeType() const;
            inline Mesh* getReferenceMesh() const;
            inline const std::list<int>& getTriangleIndices() const;
            inline const std::vector<Triangle*> getTriangles() const; 
            inline int getRigidSplitDirection() const;

            inline const Mesh* getMeshIfExists() const;
            inline const Mesh* getMesh();
            inline const std::vector<Vertex*>& getVertices() const;

        private:
            Mesh* mReferenceMesh;

            //FIXME: storing triangles AND their indices is somewhat redundant
            //       but the BvhNode splice algorithm makes heavy use of the 
            //       incices - looking up the index for a triangle would take
            //       too long.

            std::vector<Triangle*> mTriangles;
            /*!
             * \brief indices of this parts triangles in the reference mesh
             * 
             * Example: the reference Mesh consists of 5 triangles, indices 0..4
             *          if this MeshPart holds only the second and fifth
             *          triangle of the reference mesh, the list would be [1, 4]
             */
            std::list<int> mTriangleIndices;
            std::vector<Vertex*> mVertices;

            mutable Mutex mMutex;

            /*!
             *\brief indicates the direction of the binary split 
             * is -1 by default. This is used in the splitting algorithm
             * for rigid meshes.
             */
            int mRigidSplitDirection;

            mutable Mesh* mMesh;
            void generateMesh();
    };

    
    /*!
     * \brief Wrapper class to store single deformations in an list
     */
    class DeformVector {
        public:
            /*!
             * \brief Vector3 which should be used for deformation
             */
            Vector3 mDeformVector;
            
            /*!
             * \brief Index of the vertex which should be deformed
             */
            unsigned int mVertexIndex;
            
            
            inline DeformVector();
            inline DeformVector(const DeformVector& other);
            inline DeformVector(unsigned int vertexIndex, Vector3 deformVector);

            inline DeformVector& operator=(const DeformVector& other);
    };

    
    //------------ Implementation of short methods -------------

    DeformVector::DeformVector() {
        mVertexIndex = 0;
    }

    DeformVector::DeformVector(const DeformVector& other) {
        mVertexIndex = other.mVertexIndex;
        mDeformVector = other.mDeformVector;
    }

    DeformVector::DeformVector(unsigned int vertexIndex, Vector3 deformVector) {
        mVertexIndex = vertexIndex;
        mDeformVector = deformVector;
    }

    DeformVector& DeformVector::operator=(const DeformVector& other) {
        mVertexIndex = other.mVertexIndex;
        mDeformVector = other.mDeformVector;
        
        return *this;
    }


    Mesh* MeshPart::getReferenceMesh() const {
        return mReferenceMesh;
    }
    

    
    /*!
     * \brief indices of this parts triangles in the reference mesh
     * 
     * Example: the reference Mesh consists of 5 triangles, indices 0..4
     *          if this MeshPart holds only the second and fifth
     *          triangle of the reference mesh, the list would be [1, 4]
     */
    const std::list<int>& MeshPart::getTriangleIndices() const {
        return mTriangleIndices;
    }
    
    /*!
     *\brief Triangles contained in this MeshPart
     */
    const std::vector<Triangle*> MeshPart::getTriangles() const {
        return mTriangles;
    }

    Shape::ShapeType MeshPart::getShapeType() const {
        return SHAPE_TYPE_MESH_PART;
    }

    Shape::ShapeType Mesh::getShapeType() const {
        return SHAPE_TYPE_MESH;
    }

    const std::vector<Vertex*>& Mesh::getVertices() const {
        return mVertices;
    }
    
    const std::set<Line*>& Mesh::getLines() const {
        if (mLines == 0) throw NullPointerException("No line information stored!");
        return *mLines;
    }

    const std::vector<Triangle*>& Mesh::getTriangles() const {
        return mTriangles;
    }
    
    /*!
     * \brief returns the Mesh representation of this shape (here: this)
     */
    const Mesh* Mesh::getMesh() {
        return this;
    }
    /*!
     * \brief returns the Mesh representation of this shape (here: this)
     */
    const Mesh* Mesh::getMeshIfExists() const {
       return this;
    }
    
    /*!
     * \brief returns the Mesh representation of this shape, create it if needed
     */
    const Mesh* MeshPart::getMesh() {
        if (mMesh == 0) {
            generateMesh();
        }
        return mMesh;
    }
    
    const std::vector<Vertex*>& MeshPart::getVertices() const {
        return mVertices;
    }
    
    /*!
     * \brief returns the Mesh representation of this shape (can be NULL)
     */
    const Mesh* MeshPart::getMeshIfExists() const {
       return mMesh;
    }

   inline real Mesh::getAverageSideLength() const  {
      //Calculate when first called
      if (mAverageSideLength == -1) {
         //Override const declaration
         const_cast<Mesh*>(this)->updateAverageSideLength();
      }

      return mAverageSideLength;
   }

    unsigned int Mesh::getVertexCount() const {
        return mVertices.size();
    }

    unsigned int Mesh::getTriangleCount() const {
        return mTriangles.size();
    }

    int MeshPart::getRigidSplitDirection() const{
        return mRigidSplitDirection;
    }

    void Mesh::addValidWorldPositionVertex(Vertex* vertex) {
        MutexLocker lock(&mMutex);
        mValidWorldPositionVertices.push_back(vertex);
        vertex->setMeshListNode(mValidWorldPositionVertices.getLastNode());
    }

    /*!
     * \internal
     * \brief Used to remove a Vertex from the mValidWorldPositionVertices list
     * 
     * This method is called from within the Vertex class to notify the Mesh
     * about changes on the Vertex.
     */
    void Mesh::invalidateVertexWorldPosition(Vertex* vertex) {
        if (hasLocalCoordinates()) {
            if (   (!mValidWorldPositionVertices.empty())
                && (vertex->getMeshListNode())) {

                mValidWorldPositionVertices.erase(vertex->getMeshListNode());
            }
            vertex->setMeshListNode(0);
        }
    }

    bool Mesh::hasLocalCoordinates() const {
        return (     (getProxy()->getWorld()
                                ->getWorldParameters()
                                .getAllowLocalCoordinates())
                ||  !(getProxy()->getProxyType() & PROXYTYPE_DEFORMABLE));
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Mesh& v);
    std::ostream& operator<<(std::ostream& os, const dcollide::MeshPart& v);
}

#endif // DCOLLIDE_MESH_H
/*
 * vim: et sw=4 ts=4
 */
