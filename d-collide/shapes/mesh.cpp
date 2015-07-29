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

#include "exceptions/exception.h"
#include "shapes/mesh.h"

#include "mesh/vertex.h"
#include "mesh/triangle.h"
#include "mesh/line.h"
#include "math/vector.h"
#include "timing.h"
#include "debugstream.h"

#include <iostream>


namespace dcollide {

//-------------------- Mesh Implementation --------------------------//

    /*! \brief creates a Mesh with given vertices and triangles
     * OWNERSHIP NOTICE:    the Mesh takes ownership of the pointers in the two
     *                      vectors only if \p takeOwnership is true
     */
    Mesh::Mesh( const std::vector<Vertex*>& vertices,
                const std::vector<Triangle*>& triangles, bool takeOwnership) {
        mVertices = vertices;
        mTriangles = triangles;
        mLines = 0;
        init(takeOwnership);
    }

    /*! \brief creates a Mesh with given vertices and triangles
     * OWNERSHIP NOTICE:    the Mesh takes ownership of the pointers in the two
     *                      vectors
     */
    Mesh::Mesh( const std::vector<Vertex*>& vertices,
                const std::vector<Triangle*>& triangles) {
        mVertices = vertices;
        mTriangles = triangles;
        mLines = 0;

        init(true);
    }


    /*!
     * \overload
     *
     * This constructor takes a vector of indices instead of triangles. The
     * number of indices must be a multiple of 3 (if it is not, only the next
     * smaller number which is a multiple of 3 is used). The indices build up a
     * set of triangles - 3 consecutive indices are exactly one triangle. Each
     * index must be a valid index of \p vertices (indices that are out of
     * range are rounded to the next valid index).
     *
     * The missing normal vectors are generated acording to the area that is
     * defined by the vertices of one triangle.
     * OWNERSHIP NOTICE:    the Mesh takes ownership of the pointers in the two
     *                      vectors
     */
    Mesh::Mesh( const std::vector<Vertex*>& vertices,
                const std::vector<int>& indices) {
        mVertices = vertices;

        int size = indices.size();

        // ensure multiple of 3
        if (size % 3 != 0) {
            if ((size - 1) % 3 == 0) {
                size = size - 1;
            } else if ((size - 2) % 3 == 0) {
                size = size - 2;
            }
        }

        // create the triangles
        mTriangles = std::vector<Triangle*>(size / 3);
        for (int i = 0; i < size; i += 3) {
            int index1 = std::min(indices[i + 0], (int)vertices.size());
            int index2 = std::min(indices[i + 1], (int)vertices.size());
            int index3 = std::min(indices[i + 2], (int)vertices.size());

            mTriangles[i / 3] = new Triangle(vertices[index1],
                                             vertices[index2],
                                             vertices[index3]);
        }
        mLines = 0;

        init(true);
    }


    /*!
     * \overload
     *
     * This constructor takes a vector of indices instead of triangles. The
     * number of indices must be a multiple of 3 (if it is not, only the next
     * smaller number which is a multiple of 3 is used). The indices build up a
     * set of triangles - 3 consecutive indices are exactly one triangle. Each
     * index must be a valid index of \p vertices (indices that are out of
     * range are rounded to the next valid index).
     * OWNERSHIP NOTICE:    the Mesh takes ownership of the pointers in the two
     *                      vectors
     */
    Mesh::Mesh( const std::vector<Vertex*>& vertices,
                const std::vector<Vector3*>& normals,
                const std::vector<int>& indices) {
        mVertices = vertices;

        int size = std::min(indices.size(), normals.size());

        // ensure multiple of 3
        if (size % 3 != 0) {
            if ((size - 1) % 3 == 0) {
                size = size - 1;
            } else if ((size - 2) % 3 == 0) {
                size = size - 2;
            }
        }

        // create the triangles
        mTriangles = std::vector<Triangle*>(size / 3);
        for (int i = 0; i < size; i += 3) {
            int index1 = std::min(indices[i + 0], (int)vertices.size());
            int index2 = std::min(indices[i + 1], (int)vertices.size());
            int index3 = std::min(indices[i + 2], (int)vertices.size());

            mTriangles[i / 3] = new Triangle(vertices[index1], normals[i + 0],
                                             vertices[index2], normals[i + 1],
                                             vertices[index3], normals[i + 2]);
        }
        init(true);
    }

    Mesh::~Mesh() {
        if (mIsVertexAndTriangleOwner) {
            for (std::vector<Triangle*>::iterator iter = mTriangles.begin();
                    iter != mTriangles.end();
                    ++iter) {
                delete *iter;
            }
            for (std::vector<Vertex*>::iterator iter = mVertices.begin();
                    iter != mVertices.end();
                    ++iter) {
                delete *iter;
            }
        }
    }

    /*!
     * \brief convenience method for constructors
     */
    void Mesh::init(bool takeOwnership){
        mIsVertexAndTriangleOwner = takeOwnership;
        mAverageSideLength = -1;

        //set mContainingMesh for the vertices only if this is the owner
        if (mIsVertexAndTriangleOwner) {
            int vertexIndex = 0;
            for (std::vector<Vertex*>::const_iterator iter = mVertices.begin();
                            iter != mVertices.end(); ++iter) {
                if (!*iter) {
                    throw NullPointerException("Vertex* in const std::vector<Vertex*>& vertices");
                }
                (*iter)->setContainingMesh(this);
                (*iter)->setVertexIndex(vertexIndex);
                vertexIndex++;
            }
        }

        initAdjacencyLists();
    }

    /*!
     *\brief calculates adjacency and fills the mAdjacentXYZ lists
     * WARNING: runtime is O(#Triangles * #TrianglesPerVertex)
     * Executed for each mesh on initialization
     * \code
     *  //for all Vertices
     *  //  for all Triangles
     *  //     check if one of the triangles vertices is the current vertex
     *  //        if it is,
     *  //          add triangle to adjacent-triangles for current vertex
     *  //          add the other vertices of the triangle to adjacent-triangles
     *  //                       of the current vertex (prevent dublicates!)
     * \endcode
     */
    void Mesh::initAdjacencyLists() {
        //Timing time; // starts measuring time

        //for all triangles
        //for all vertices of that triangle
        //  add triangle to adjacent-triangles of currentVertex
        //  add the other vertices as neighbors, prevent duplicates
        for (std::vector<Triangle*>::iterator triangleIter = mTriangles.begin();
                triangleIter != mTriangles.end(); ++triangleIter) {
            Triangle* currentTriangle = *triangleIter;

            const array<Vertex*,3>& currentTriangleVertices = currentTriangle->getVertices();
            for (int i = 0; i<3; i++) {
                //  add triangle to adjacent-triangles of currentVertex
                currentTriangleVertices[i]->addAdjacentTriangle(currentTriangle);
                //  add the other vertices as neighbors, prevent duplicates
                //since list has no "contains" function, we need to check manually
                bool containsOther1 = false;
                bool containsOther2 = false;
                Vertex* otherV1 = currentTriangleVertices[(i+1)%3];
                Vertex* otherV2 = currentTriangleVertices[(i+2)%3];

                for (std::list<Vertex*>::const_iterator containsIter = currentTriangleVertices[i]->getAdjacentVertices().begin();
                                containsIter != currentTriangleVertices[i]->getAdjacentVertices().end(); ++containsIter) {
                    if (*containsIter == otherV1) {
                        containsOther1=true;
                    } else if (*containsIter == otherV2) {
                        containsOther2=true;
                    }
                    if (containsOther1 && containsOther2) {
                        break;
                    }
                }

                if (!containsOther1) {
                    currentTriangleVertices[i]->addAdjacentVertex(otherV1);
                }
                if (!containsOther2) {
                    currentTriangleVertices[i]->addAdjacentVertex(otherV2);
                }
            }//end for all 3 vertices of the triangle
        }//end for all triangles

        //time.stop(); // records elapsed time
        //std::cout << "Time for Mesh::initAdjacencyLists() with "<< mTriangles.size() << " triangles: " << time.elapsedTime() << " us" << std::endl;
        //init triangle - triangle neighborhood for this triangle as well
        for (std::vector<Triangle*>::iterator triangleIter = mTriangles.begin();
                triangleIter != mTriangles.end(); ++triangleIter) {
            //check the first vertex of the point
            //find the first two triangles
            Triangle* t = *triangleIter;
            Triangle* n1 = 0;
            Triangle* n2 = 0;
            Triangle* n3 = 0;
            for (std::list<Triangle*>::const_iterator vertexTriangleIter
                    = t->getVertices()[0]->getAdjacentTriangles().begin();
                    vertexTriangleIter != t->getVertices()[0]->getAdjacentTriangles().end();
                    ++vertexTriangleIter) {

                Triangle* neighborTriangle = *vertexTriangleIter;
                if (neighborTriangle != t) {
                    //the neighborTriangle shares a vertex with the triangle
                    //test all vertices of this triangle, find mVertices[1]
                    //or mVertices[2]
                    for (int i = 0; i<3; ++i) {
                        if (neighborTriangle->getVertices()[i] == t->getVertices()[1]) {
                            n1 = neighborTriangle;
                        }
                        if (neighborTriangle->getVertices()[i] == t->getVertices()[2]) {
                            n2 = neighborTriangle;
                        }
                        if (n1 != 0 && n2 != 0) {
                            break;
                        }
                    }//end for verticecheck
                }
            }
            for (std::list<Triangle*>::const_iterator vertexTriangleIter
                    =   t->getVertices()[1]->getAdjacentTriangles().begin();
                    vertexTriangleIter != t->getVertices()[1]->getAdjacentTriangles().end();
                    ++vertexTriangleIter) {
                Triangle* neighborTriangle = *vertexTriangleIter;
                if (neighborTriangle != t) {
                    //test all vertices of this triangle, find mVertices[1]
                    //or mVertices[2]
                    for (int i = 0; i<3; ++i) {
                        if (neighborTriangle->getVertices()[i] == t->getVertices()[2]) {
                            n3 = neighborTriangle;
                            break;
                        }
                    }//end for verticecheck
                }
            }
            //search triangles of second vertex for the third neighbor
            t->setEdgeAdjacentTriangles(n1, n2, n3);
        }//end for all triangles: init neighbors

        //for all vertices
        //  for all triangles at this vertex
        //      add triangle to all other triangles of this vertex
        for (std::vector<Vertex*>::iterator vertexIter = mVertices.begin();
            vertexIter != mVertices.end(); ++vertexIter) {
            for (std::list<Triangle*>::const_iterator triangleIter =
                    (*vertexIter)->getAdjacentTriangles().begin();
                    triangleIter != (*vertexIter)->getAdjacentTriangles().end();
                    ++triangleIter) {
                for (std::list<Triangle*>::const_iterator otherTriangleIter =
                        (*vertexIter)->getAdjacentTriangles().begin();
                        otherTriangleIter != (*vertexIter)->getAdjacentTriangles().end();
                        ++otherTriangleIter) {
                    if ((*triangleIter) != (*otherTriangleIter)) {
                        (*triangleIter)->addVertexAdjacentTriangle(*otherTriangleIter);
                    }
                }//end innermost otherTriangle for-loop
            }
        }//end for all vertices
    }

   /*!
    * \brief Calculates the average length of the triangle-sides of this mesh
    */
   //TODO: Maybe an iterative algorithm is better
    void Mesh::updateAverageSideLength() {
        real triangle_sum = 0;
        for (unsigned int i = 0; i < mTriangles.size(); i++) {
            Vector3 v1 = mTriangles[i]->getVertices()[0]->getPosition();
            Vector3 v2 = mTriangles[i]->getVertices()[1]->getPosition();
            Vector3 v3 = mTriangles[i]->getVertices()[2]->getPosition();

            real side1 = (v2-v1).length();
            real side2 = (v3-v2).length();
            real side3 = (v1-v3).length();

            //Average side length of this triangle
            triangle_sum += (side1 + side2 + side3) / 3;
        }

        mAverageSideLength = triangle_sum / mTriangles.size();
    }

    /*
     * Deformation applies without respectation of the mesh orientation
     */
    void Mesh::deform(const std::vector<Vector3>& deformVector) {
        // This part can only handle translation
        // for scale and rotate we need new methods
        if (mVertices.size() == deformVector.size()) {
            invalidateAllWorldPositions();

            for (unsigned int i=0; i<mVertices.size(); i++) {
                mVertices[i]->translate(deformVector[i]);
            }
        } else {
            throw MeshDeformException();
        }
    }

    /*!
     * \brief Deforms only one specific vertex of the mesh
     *
     * When you use this method you must call the method deform() after you
     * added deformation vectors for all transformation you want to do, because
     * otherwise your changes wouldn't take place.
     */
    void Mesh::addDeformVector(unsigned int vertexIndex, const Vector3& deformVector) {

        // This part can only handle translation
        // for scale and rotate we need new methods
        if (vertexIndex <= mVertices.size()) {
            mDeformVectors.push_back(DeformVector(vertexIndex, deformVector));
        } else {
            throw MeshDeformException();
        }
    }

    /*!
     * \brief Clears the internal cache of deform vectors
     *
     * All previously added deform vectors
     * (see Mesh::addDeformVector(unsigned int, const Vector3&) are removed from
     * the internal list so that a imidiately following call to Mesh::deform()
     * would have no effect.
     */
    void Mesh::discardDeformVectors() {
        mDeformVectors.clear();
    }

    /*!
     * \brief Applies the given deformation vectors which are store in
     *        mDeformVectors
     */
    void Mesh::deform() {
        invalidateAllWorldPositions();

        while (!mDeformVectors.empty()) {
            mVertices[mDeformVectors.front().mVertexIndex]
                      ->translate(mDeformVectors.front().mDeformVector);
            mDeformVectors.pop_front();
        }
    }

    void Mesh::invalidateAllWorldPositions() {
        if (hasLocalCoordinates()) {
            while (!mValidWorldPositionVertices.empty()) {
                mValidWorldPositionVertices.front()->invalidateWorldPosition();
            }
        }
    }

//-------------------- MeshPart Implementation --------------------------//

    /*!
     * \brief MeshPart c'tor
     *
     * internal use only
     *
     * \internal
     *
     * \param triangleIndices A list of triangle indices (in the \p mesh) to be
     *        used in this MeshPart. The list must not be used anymore after
     *        calling this constructor (splice() is used to copy it)
     * \param vertices A vector containing all vertices used by the triangles
     *        referenced by \p triangleIndices.
     *        This vector must not be used anymore after calling this
     *        constructor (swap() is used to copy it)
     *
     * OWNERSHIP NOTICE: the MeshPart does not take ownership of the
     *                   Triangles and their Vertices. The reference Mesh keeps
     *                   the ownership.
     *
     * This constructor takes a list of indices of the corresponding mesh and
     * creates a meshpart out of these.\n
     * Example given:\n
     * A mesh myMesh consists of 3 triangles: 1,2,3
     * so meshpart(&myMesh, [1,2]) would create a meshpart which consists of the
     * first two triangles.
     * Be aware, that you have to replace [1,2] with a std::list which includes
     * the indices one and two, instead of [1,2]
     */
    MeshPart::MeshPart(Mesh* mesh, std::list<int>& triangleIndices, std::vector<Vertex*>& vertices) {
        mProxy = mesh->getProxy();
        mReferenceMesh = mesh;
        mMesh = 0;
        mTriangleIndices.splice(mTriangleIndices.end(), triangleIndices);
        mVertices.swap(vertices);

#ifdef __GNUC__
#warning FIXME: usable for current split algorithm only
#endif
        mRigidSplitDirection = -1;

        for (std::list<int>::const_iterator iter = mTriangleIndices.begin();
                 iter != mTriangleIndices.end(); ++iter) {
             Triangle* currentTriangle = (mReferenceMesh->getTriangles())[*iter];
             mTriangles.push_back(currentTriangle);
         }
    }

    /*!
     * \overload
     *
     * This version is SLOW for large \p triangleIndices lists, as it has to
     * calculate the vertices used by \p triangleIndices.
     */
    MeshPart::MeshPart(Mesh* mesh, std::list<int>& triangleIndices) {
        mProxy = mesh->getProxy();
        mReferenceMesh = mesh;
        mMesh = 0;
        mTriangleIndices.splice(mTriangleIndices.end(), triangleIndices);

#ifdef __GNUC__
#warning FIXME: usable for current split algorithm only
#endif
        mRigidSplitDirection = -1;

        std::set<Vertex*> verticesSet;
        for (std::list<int>::const_iterator iter = mTriangleIndices.begin();
                 iter != mTriangleIndices.end(); ++iter) {
             Triangle* currentTriangle = (mReferenceMesh->getTriangles())[*iter];
             mTriangles.push_back(currentTriangle);

             const array<Vertex*,3>& vertices = currentTriangle->getVertices();
             verticesSet.insert(vertices[0]);
             verticesSet.insert(vertices[1]);
             verticesSet.insert(vertices[2]);
        }
        mVertices.reserve(verticesSet.size());
        for (std::set<Vertex*>::const_iterator it = verticesSet.begin(); it != verticesSet.end(); ++it) {
            mVertices.push_back(*it);
        }
    }

    /*!
     * \brief MeshPart c'tor with split-direction
     *
     * internal use only
     */
#ifdef __GNUC__
#   warning FIXME: first vertices, then splitOrder
#endif
    MeshPart::MeshPart(Mesh* mesh, std::list<int>& triangleIndices,
                                                int splitDirection,
                                                std::vector<Vertex*>& vertices) {
        mProxy = mesh->getProxy();
        mReferenceMesh = mesh;
        mTriangleIndices.splice(mTriangleIndices.end(), triangleIndices);
        mVertices.swap(vertices);
        mMesh = 0;

#ifdef __GNUC__
#   warning FIXME: usable for current split algorithm only
#endif
        // AB: this should not be stored here - it is actually a property of the
        // MeshSplitter.
        // probably we need some kind of MeshSplitterMeshPartData or at least
        // MeshSplitterData object, which can be stored in a MeshPart and that
        // classes deriving from MeshSplitter can create and use
        mRigidSplitDirection = splitDirection;


        for (std::list<int>::const_iterator iter = mTriangleIndices.begin();
                 iter != mTriangleIndices.end(); ++iter) {
             Triangle* currentTriangle = (mReferenceMesh->getTriangles())[*iter];
             mTriangles.push_back(currentTriangle);
         }
    }

    MeshPart::~MeshPart() {
        delete mMesh;
    }

    // AB: this might be called from a thread, so it must be thread-safe!
    void MeshPart::generateMesh() {
        MutexLocker lock(&mMutex);
        if (mMesh) {
            return;
        }
        std::vector<Vertex*> vertices;
        std::vector<Triangle*> triangles;

        //pick triangles from reference mesh
        //collect Vertices
        std::vector<Triangle*> allTriangles = mReferenceMesh->getTriangles();
        for (std::list<int>::iterator iter = mTriangleIndices.begin();
                        iter != mTriangleIndices.end(); ++iter) {

            Triangle* t = allTriangles[*iter];
            triangles.push_back(t);
            bool foundVert0 = false;
            bool foundVert1 = false;
            bool foundVert2 = false;
            //scan through vertices-list if we have already added the vertices
            for (unsigned int i = 0; i< vertices.size(); ++i) {
                if (vertices[i] == t->getVertices()[0]) {
                    foundVert0 = true;
                } else if (vertices[i] == t->getVertices()[1]) {
                    foundVert1 = true;
                } else if (vertices[i] == t->getVertices()[2]) {
                    foundVert2 = true;
                }
                if (foundVert0 && foundVert1 && foundVert2) {
                    break;
                }
            }
            if (!foundVert0) {
                vertices.push_back(t->getVertices()[0]);
            }
            if (!foundVert1) {
                vertices.push_back(t->getVertices()[1]);
            }
            if (!foundVert2) {
                vertices.push_back(t->getVertices()[2]);
            }

        }
        // generate a new mesh, but the new mesh must not become the owner
        // of the triangles and vertices - they belong to the reference mesh
        mMesh = Mesh::createMeshWithoutOwnership(vertices, triangles);
        mMesh->setProxy(mProxy);
    }


    /*!
     * WARNING: this is a very slow method! It (currently?) has to search for
     * the index of all vertices of all triangles in the vertices vector.
     *
     * \return A (deep) copy of this mesh, i.e. all vertices, triangles and
     * normals are copied.
     */
    Mesh* Mesh::cloneMesh() const {
        unsigned int verticesSize = mVertices.size();
        std::vector<Vertex*> vertices(verticesSize);
        for (unsigned int i = 0; i < verticesSize; i++) {
            vertices[i] = new Vertex(mVertices[i]->getPosition());
        }

        unsigned int trianglesSize = mTriangles.size();
        std::vector<Triangle*> triangles(trianglesSize);
        for (unsigned int triangle = 0; triangle < trianglesSize; triangle++) {
            Triangle* t = mTriangles[triangle];
            unsigned int index[3];
            index[0] = 0;
            index[1] = 0;
            index[2] = 0;
            Vector3* normals[3];
            for (int vertex = 0; vertex < 3; vertex++) {
                if (t->getNormals()[vertex]) {
                    normals[vertex] = new Vector3(*t->getNormals()[vertex]);
                } else {
                    normals[vertex] = 0;
                }
                Vertex* v = t->getVertices()[vertex];
                for (unsigned int i = 0; i < verticesSize; i++) {
                    if (mVertices[i] == v) {
                        index[vertex] = i;
                        break;
                    }
                }
            }
            triangles[triangle] = new Triangle(
                    vertices[index[0]], normals[0],
                    vertices[index[1]], normals[1],
                    vertices[index[2]], normals[2]
                    );
        }
        return new Mesh(vertices, triangles);
    }

    /*!
     * \brief Stores all lines seperately in the mesh
     *
     * This stores all lines in a seperate list in the mesh instead of having
     * them only impicitely (and duplicated) stored in the triangle list.\n
     * This is a constant function because it doesn't change the mesh itself
     * but only the representation of it.
     */
    void Mesh::storeLines() {
        // Check if the lines were calculated before and possibly abort
        if (mLines != 0) return;

        mLines = new std::set<Line*>;
        // Inserting all lines of a mesh into the set and thus making them unique
        for (std::vector<Triangle*>::iterator iter = mTriangles.begin();
                    iter != mTriangles.end(); ++iter) {
                        mLines->insert(new Line(
                            (*iter)->getVertices()[0],(*iter)->getVertices()[1]));
                        mLines->insert(new Line(
                            (*iter)->getVertices()[0],(*iter)->getVertices()[2]));
                        mLines->insert(new Line(
                            (*iter)->getVertices()[1],(*iter)->getVertices()[2]));
        }
    }

    /*!
     * \brief the mesh does not take ownership of its vertices and triangles
     *
     * internal use only.
     */
    Mesh* Mesh::createMeshWithoutOwnership(
            const std::vector<Vertex*>& vertices,
            const std::vector<Triangle*>& triangles) {
        return new Mesh(vertices, triangles, false);
    }


    /*!
     * \brief Sets the absolute position of an vertex in this mesh
     */
    void Mesh::setVertexPosition(unsigned int vertexIndex, const Vector3& position) {
        if (vertexIndex < mVertices.size()) {
            mVertices[vertexIndex]->setPosition(position);
        } else {
            throw MeshDeformException();
        }
    }

    /*!
     * \brief Sets the absolute position all vertices in this mesh
     *
     * All positions which are set with this method are ment in local mesh
     * coordinates. The Only exception is when using a deformable Proxy and the
     * WorldParameter allowLocalCoordinates == false in this case the mesh base
     * point will lie on (0,0,0) and the vertex coordinates should be
     * interpreted as "world coordinates".
     */
    void Mesh::setVertexPositions(const std::vector<Vector3>& vertexPositionArray) {

        if (mVertices.size() == vertexPositionArray.size()) {
            for (unsigned int i=0; i<mVertices.size(); i++) {
                mVertices[i]->setPosition(vertexPositionArray[i]);
            }
        } else {
            throw MeshDeformException();
        }
    }


    /*!
     * \brief Calculates the center of the whole mesh
     */
    Vector3 Mesh::getMidPoint() const {

        if (mVertices.empty()) {
            return Vector3(0,0,0);
        }

        Vector3 center;

        for(std::vector<Vertex*>::const_iterator iter = mVertices.begin();
                iter != mVertices.end(); ++iter) {
            const Vector3& point = (*iter)->getWorldPosition();
            center += point;
        }

        return center / mVertices.size();
    }


    /* This function should give a short string representation of a mesh,
     * but what is a SHORT representation of a mesh with so many vertices and
     * triangles?
     * I doubt it would be useful to print them all.
     * Nevertheless, its also not useful to have no text representation for a mesh at all
     */
    std::ostream& operator<<(std::ostream& os, const dcollide::Mesh& v) {
        os << "(Mesh: many points, many triangles)";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::MeshPart& v) {
        os << "(MeshPart: few points, few triangles)";
        return os;
    }
}
/*
 * vim: et sw=4 ts=4
 */
