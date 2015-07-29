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

#include "meshsplitter.h"

#include "bvhnodedefault.h"
#include "boundingvolumes/boundingvolume.h"
#include "debugstream.h"
#include "shapes/mesh.h"
#include "shapes/mesh/triangle.h"
#include "shapes/mesh/vertex.h"
#include "dcollide-defines.h"

#include <cstdlib>

namespace dcollide {
    MeshSplitterFactory::MeshSplitterFactory() {
    }

    MeshSplitterFactory::~MeshSplitterFactory() {
    }

    /*!
     * \return A new \ref DefaultMeshSplitter. Derived classes should
     * reimplement this method and return their own \ref MeshSplitter derived
     * object.
     *
     * OWNERSHIP NOTICE: the caller takes ownership of the returned object.
     */
    MeshSplitter* MeshSplitterFactory::createMeshSplitter(World* world, Mesh* mesh, BvhNode* topBvhNode) {
        return new DefaultMeshSplitter(world, mesh, topBvhNode);
    }

    /*!
     * This method \em MUST be reimplemented when deriving from this class!
     * Simply create a new object of your class.
     *
     * \return A new object of this class.
     *
     * OWNERSHIP NOTICE: the caller takes ownership of the returned object.
     */
    MeshSplitterFactory* MeshSplitterFactory::clone() {
        return new MeshSplitterFactory();
    }

    /*!
     * Create a new \ref MeshSplitter object. Use \ref startSplitting for the actual
     * splitting.
     *
     * \param world The \ref World that we work in. Required for the new \ref
     *        BvhNode objects
     * \param mesh The mesh that is to-be splitted.
     * \param topBvhNode The \ref BvhNode object that contains the \p mesh.
     *        The new \ref BvhNode hierarchy that is created is added to this
     *        node.
     */
    MeshSplitter::MeshSplitter(World* world, Mesh* mesh, BvhNode* topBvhNode) {
        mWorld = world;
        mMesh = mesh;
        mTopLevelBvhNode = topBvhNode;
        if (!getWorld()) {
            throw NullPointerException("getWorld()");
        }
        if (!getMesh()) {
            throw NullPointerException("getMesh()");
        }
        if (!getTopLevelBvhNode()) {
            throw NullPointerException("getTopLevelBvhNode()");
        }
        mMaxMeshPartTriangles = getWorld()->getWorldParameters().getRigidMaxMeshPartTriangles();
    }

    MeshSplitter::~MeshSplitter() {
    }

    /*!
     * \return The \ref World object provided in the constructor
     */
    World* MeshSplitter::getWorld() const {
        return mWorld;
    }

    /*!
     * \return The \ref Mesh object this splitter operates on.
     */
    Mesh* MeshSplitter::getMesh() const {
        return mMesh;
    }

    /*!
     * Note: "toplevel" refers to the fact that it is the root of the new
     * hierarchy, however the \ref BvhNode does not actually have to be
     * a toplevel node, i.e. it can have a parent. It just needs to contain the
     * \ref getMesh.
     *
     * \return The \ref BvhNode object that \ref getMesh belongs to, i.e. the
     * \ref BvhNode object that this splitter appends the new hierarchy to.
     */
    BvhNode* MeshSplitter::getTopLevelBvhNode() const {
        return mTopLevelBvhNode;
    }

    /*!
     * Creates a tree of triangles from this mesh and adds them to this
     * node.
     *
     * Derived classes normally should not need to reimplement this method -
     * reimplement \ref calculateSplit instead. However this method is virtual,
     * so if for some splitting algorithm more a direct control is required,
     * this method can be overwritten in derived classes.
     */
    void MeshSplitter::startSplitting() {
        if (getMesh()->getTriangleCount() > mMaxMeshPartTriangles) {
            std::list<int> triangleIndices;
            for (unsigned int i = 0; i < getMesh()->getTriangleCount(); i++) {
                triangleIndices.push_back(i);
            }

            // AB: MeshPart::MeshPart() uses splice and swap(), so triangleIndices
            // and vertices become invalid after this call!
            std::vector<Vertex*> vertices = getMesh()->getVertices();
            MeshPart* parentPart = new MeshPart(getMesh(), triangleIndices, vertices);
            splitMesh(getTopLevelBvhNode(), parentPart);
            delete parentPart;

        } else {
            getTopLevelBvhNode()->setIgnoreShape(false);
        }
    }

    /*!
     * This method is meant to be used internally only. Use \ref startSplitting
     * to start splitting the mesh.
     *
     * This method is called recurisively on the mesh(part) to create multiple
     * \ref MeshPart objects from it (on which we call \ref splitMesh again
     * until a threshold is reached).
     *
     * \param parentNode The BvhNode which will become the parent node of the
     *        newly created \ref BvhNode objects. The \p parentNode should be
     *        the \ref BvhNode of the \p meshPart, however this is not a
     *        requirement: for the first "split" \p meshPart does not belong to
     *        any \ref BvhNode - \p parentNode contains the actual \ref Mesh
     *        instead.
     * \param mesh The \ref MeshPart to be split into parts.
     */
    void MeshSplitter::splitMesh(BvhNode* parentNode, MeshPart* meshPart) {
        if (meshPart->getTriangleIndices().size() > mMaxMeshPartTriangles) {

            std::list<MeshPart*> parts = calculateSplit(parentNode, meshPart);

            if (parts.empty()) {
                // TODO:
                // AB: it may make sense to accept an empty list and consider it
                //     an alternative stop-criterium!
                //     i.e. we stop splitting either if triangleCount <
                //     mMaxMeshPartTriangles, or if calculateSplit()
                //     returns an empty list.
                //
                //     however atm there is no code that may make use of that.
                throw Exception("calculateSplit() did not return any MeshPart objects");
            }
            if (parts.size() == 1) {
                // AB: exactly one child makes no sense.
                delete parts.front();
                throw Exception("calculateSplit() did return one child only. want at least 2");
            }
            for (std::list<MeshPart*>::const_iterator it = parts.begin(); it != parts.end(); ++it) {
                if (!(*it)) {
                    for (std::list<MeshPart*>::const_iterator it = parts.begin(); it != parts.end(); ++it) {
                        delete *it;
                    }
                    throw NullPointerException("one of the elements of \"parts\"");
                }
                BvhNode* node = createBvhNode(*it);
                splitMesh(node, *it);
                parentNode->addChild(node);
            }

            // the BV of the parent is the union of the two new child nodes.
            // the shape of the parent (i.e. the mesh) is not required to
            // calculate the BV.
            getTopLevelBvhNode()->setIgnoreShape(true);

        } else {
            parentNode->setIgnoreShape(false);
        }
    }

    BvhNode* MeshSplitter::createBvhNode(MeshPart* part) const {
        BvhNode* node = new BvhNodeDefault(getWorld(), part);
        node->initializeNode();
        return node;
    }

    DefaultMeshSplitter::DefaultMeshSplitter(World* world, Mesh* mesh, BvhNode* topBvhNode)
            : MeshSplitter(world, mesh, topBvhNode) {
    }


    DefaultMeshSplitter::~DefaultMeshSplitter() {
    }

    /*!
     * See \ref MeshSplitter::startSplitting. This method has been reimplemented
     * for performance reason (additional pre/post-processing added).
     */
    void DefaultMeshSplitter::startSplitting() {
        mVertices.clear();
        mVertices.reserve(getMesh()->getVertexCount());
        const std::vector<Vertex*>& vertices = getMesh()->getVertices();
        for (std::vector<Vertex*>::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {
            if ((*it)->getVertexIndex() < 0) {
                throw Exception("Vertex indices not initialized");
            }
            if ((unsigned int)(*it)->getVertexIndex() >= getMesh()->getVertexCount()) {
                throw Exception("Vertex index is out of range");
            }
            mVertices.push_back(MyVertex(*it));
        }
        MeshSplitter::startSplitting();
    }

    /*!
     * If the meshpart has more than a set number of triangles, it will be split
     * \return a pair of meshparts with equally distributed triangles (approx.)
     */
    std::list<MeshPart*> DefaultMeshSplitter::calculateSplit(BvhNode* parentNode, MeshPart* parent) {
        if (!parent) {
            throw NullPointerException("parameter \"MeshPart* parent\"");
        }

        const std::list<int>& parentIndices = parent->getTriangleIndices();

        mXMinVertices.clear();
        mXMaxVertices.clear();
        mYMinVertices.clear();
        mYMaxVertices.clear();
        mZMinVertices.clear();
        mZMaxVertices.clear();
        std::list<int> xMinIndices;
        std::list<int> xMaxIndices;
        std::list<int> yMinIndices;
        std::list<int> yMaxIndices;
        std::list<int> zMinIndices;
        std::list<int> zMaxIndices;
        int xMaxIndicesSize = 0;
        int xMinIndicesSize = 0;
        int yMaxIndicesSize = 0;
        int yMinIndicesSize = 0;
        int zMaxIndicesSize = 0;
        int zMinIndicesSize = 0;

        Vector3 aabbMin = parentNode->getBoundingVolume()->getSurroundingAabbMin();
        Vector3 aabbMax = parentNode->getBoundingVolume()->getSurroundingAabbMax();
        Vector3 aabbCenter = aabbMin + (aabbMax - aabbMin) / 2.0;

        for (std::list<int>::const_iterator indexIter
                    = parentIndices.begin();
                    indexIter != parentIndices.end();
                    indexIter++) {
            //get triangle and calculate corner positions
            Triangle* triangle = parent->getReferenceMesh()->getTriangles()[*indexIter];
            Vertex* vertex1 = triangle->getVertices()[0];
            Vertex* vertex2 = triangle->getVertices()[1];
            Vertex* vertex3 = triangle->getVertices()[2];
            const Vector3& v1 = vertex1->getPosition();
            const Vector3& v2 = vertex2->getPosition();
            const Vector3& v3 = vertex3->getPosition();
            MyVertex* myVertex1 = &mVertices[vertex1->getVertexIndex()];
            MyVertex* myVertex2 = &mVertices[vertex2->getVertexIndex()];
            MyVertex* myVertex3 = &mVertices[vertex3->getVertexIndex()];

            //Separate along the X-Axis: if >2 Corners of a triangle
            //have X-Coordinates < (maxX - minX)/2, put it into left half,
            //else put it into right half
            int minCorner = 0;
            int maxCorner = 0;
            if (parent->getRigidSplitDirection() != 0) {
                if (v1.getX() < aabbCenter.getX()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (v2.getX() < aabbCenter.getX()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (v3.getX() < aabbCenter.getX()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (minCorner > maxCorner) {
                    xMinIndices.push_back(*indexIter);
                    xMinIndicesSize++;

                    // add the vertices to mXMinVertices iff they were not added
                    // before.
                    if (myVertex1->xMin == 0) {
                        mXMinVertices.push_back(vertex1);
                        if (!myVertex1->used) {
                            mUsedVertices.push_back(myVertex1);
                            myVertex1->used = true;
                        }
                    }
                    if (myVertex2->xMin == 0) {
                        mXMinVertices.push_back(vertex2);
                        if (!myVertex2->used) {
                            mUsedVertices.push_back(myVertex2);
                            myVertex2->used = true;
                        }
                    }
                    if (myVertex3->xMin == 0) {
                        mXMinVertices.push_back(vertex3);
                        if (!myVertex3->used) {
                            mUsedVertices.push_back(myVertex3);
                            myVertex3->used = true;
                        }
                    }
                    myVertex1->xMin++;
                    myVertex2->xMin++;
                    myVertex3->xMin++;
                } else {
                    xMaxIndices.push_back(*indexIter);
                    xMaxIndicesSize++;

                    // add the vertices to mXMaxVertices iff they were not added
                    // before.
                    if (myVertex1->xMax == 0) {
                        mXMaxVertices.push_back(vertex1);
                        if (!myVertex1->used) {
                            mUsedVertices.push_back(myVertex1);
                            myVertex1->used = true;
                        }
                    }
                    if (myVertex2->xMax == 0) {
                        mXMaxVertices.push_back(vertex2);
                        if (!myVertex2->used) {
                            mUsedVertices.push_back(myVertex2);
                            myVertex2->used = true;
                        }
                    }
                    if (myVertex3->xMax == 0) {
                        mXMaxVertices.push_back(vertex3);
                        if (!myVertex3->used) {
                            mUsedVertices.push_back(myVertex3);
                            myVertex3->used = true;
                        }
                    }
                    myVertex1->xMax++;
                    myVertex2->xMax++;
                    myVertex3->xMax++;
                    myVertex1->used = true;
                    myVertex2->used = true;
                    myVertex3->used = true;
                }
            }
            //Same process for Y-Axis separation
            if (parent->getRigidSplitDirection() != 1) {
                minCorner = maxCorner = 0;
                if (v1.getY() < aabbCenter.getY()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (v2.getY() < aabbCenter.getY()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (v3.getY() < aabbCenter.getY()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (minCorner > maxCorner) {
                    yMinIndices.push_back(*indexIter);
                    yMinIndicesSize++;

                    // add the vertices to mYMinVertices iff they were not added
                    // before.
                    if (myVertex1->yMin == 0) {
                        mYMinVertices.push_back(vertex1);
                        if (!myVertex1->used) {
                            mUsedVertices.push_back(myVertex1);
                            myVertex1->used = true;
                        }
                    }
                    if (myVertex2->yMin == 0) {
                        mYMinVertices.push_back(vertex2);
                        if (!myVertex2->used) {
                            mUsedVertices.push_back(myVertex2);
                            myVertex2->used = true;
                        }
                    }
                    if (myVertex3->yMin == 0) {
                        mYMinVertices.push_back(vertex3);
                        if (!myVertex3->used) {
                            mUsedVertices.push_back(myVertex3);
                            myVertex3->used = true;
                        }
                    }
                    myVertex1->yMin++;
                    myVertex2->yMin++;
                    myVertex3->yMin++;
                    myVertex1->used = true;
                    myVertex2->used = true;
                    myVertex3->used = true;
                } else {
                    yMaxIndices.push_back(*indexIter);
                    yMaxIndicesSize++;

                    // add the vertices to mYMaxVertices iff they were not added
                    // before.
                    if (myVertex1->yMax == 0) {
                        mYMaxVertices.push_back(vertex1);
                        if (!myVertex1->used) {
                            mUsedVertices.push_back(myVertex1);
                            myVertex1->used = true;
                        }
                    }
                    if (myVertex2->yMax == 0) {
                        mYMaxVertices.push_back(vertex2);
                        if (!myVertex2->used) {
                            mUsedVertices.push_back(myVertex2);
                            myVertex2->used = true;
                        }
                    }
                    if (myVertex3->yMax == 0) {
                        mYMaxVertices.push_back(vertex3);
                        if (!myVertex3->used) {
                            mUsedVertices.push_back(myVertex3);
                            myVertex3->used = true;
                        }
                    }
                    myVertex1->yMax++;
                    myVertex2->yMax++;
                    myVertex3->yMax++;
                    myVertex1->used = true;
                    myVertex2->used = true;
                    myVertex3->used = true;
                }
            }
            //Same process for Z-Axis separation
            if (parent->getRigidSplitDirection() != 2) {
                minCorner = maxCorner = 0;
                if (v1.getZ() < aabbCenter.getZ()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (v2.getZ() < aabbCenter.getZ()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (v3.getZ() < aabbCenter.getZ()) {
                    ++minCorner;
                } else {
                    ++maxCorner;
                }
                if (minCorner > maxCorner) {
                    zMinIndices.push_back(*indexIter);
                    zMinIndicesSize++;

                    // add the vertices to mZMinVertices iff they were not added
                    // before.
                    if (myVertex1->zMin == 0) {
                        mZMinVertices.push_back(vertex1);
                        if (!myVertex1->used) {
                            mUsedVertices.push_back(myVertex1);
                            myVertex1->used = true;
                        }
                    }
                    if (myVertex2->zMin == 0) {
                        mZMinVertices.push_back(vertex2);
                        if (!myVertex2->used) {
                            mUsedVertices.push_back(myVertex2);
                            myVertex2->used = true;
                        }
                    }
                    if (myVertex3->zMin == 0) {
                        mZMinVertices.push_back(vertex3);
                        if (!myVertex3->used) {
                            mUsedVertices.push_back(myVertex3);
                            myVertex3->used = true;
                        }
                    }
                    myVertex1->zMin++;
                    myVertex2->zMin++;
                    myVertex3->zMin++;
                    myVertex1->used = true;
                    myVertex2->used = true;
                    myVertex3->used = true;
                } else {
                    zMaxIndices.push_back(*indexIter);
                    zMaxIndicesSize++;

                    // add the vertices to mZMaxVertices iff they were not added
                    // before.
                    if (myVertex1->zMax == 0) {
                        mZMaxVertices.push_back(vertex1);
                        if (!myVertex1->used) {
                            mUsedVertices.push_back(myVertex1);
                            myVertex1->used = true;
                        }
                    }
                    if (myVertex2->zMax == 0) {
                        mZMaxVertices.push_back(vertex2);
                        if (!myVertex2->used) {
                            mUsedVertices.push_back(myVertex2);
                            myVertex2->used = true;
                        }
                    }
                    if (myVertex3->zMax == 0) {
                        mZMaxVertices.push_back(vertex3);
                        if (!myVertex3->used) {
                            mUsedVertices.push_back(myVertex3);
                            myVertex3->used = true;
                        }
                    }
                    myVertex1->zMax++;
                    myVertex2->zMax++;
                    myVertex3->zMax++;
                    myVertex1->used = true;
                    myVertex2->used = true;
                    myVertex3->used = true;
                }
            }
        }

        //compare the separations: best split is the one with least difference
        //of #triangles in the two parts

        std::list<MeshPart*> result;
        if (parent->getRigidSplitDirection() == -1) {
            //evaluate all splits
            if (            abs( (int)(xMinIndicesSize - xMaxIndicesSize) )
                        <= abs( (int)(yMinIndicesSize - yMaxIndicesSize) )
                    &&      abs( (int)(xMinIndicesSize - xMaxIndicesSize) )
                        <= abs( (int)(zMinIndicesSize - zMaxIndicesSize) ) ) {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mXMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), xMinIndices, 0, vertices));
                makeVertices(&vertices, mXMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), xMaxIndices, 0, vertices));
            } else if (     abs( (int)(yMinIndicesSize - yMaxIndicesSize) )
                        <= abs( (int)(xMinIndicesSize - xMaxIndicesSize) )
                        &&  abs( (int)(yMinIndicesSize - yMaxIndicesSize) )
                        <= abs( (int)(zMinIndicesSize - zMaxIndicesSize) ) ) {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mYMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), yMinIndices, 1, vertices));
                makeVertices(&vertices, mYMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), yMaxIndices, 1, vertices));
            } else {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mZMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), zMinIndices, 2, vertices));
                makeVertices(&vertices, mZMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), zMaxIndices, 2, vertices));
            }
        } else if (parent->getRigidSplitDirection() == 0) {
            //evaluate only y and z
            if (     abs( (int)(yMinIndicesSize - yMaxIndicesSize) )
                        <= abs( (int)(zMinIndicesSize - zMaxIndicesSize) )) {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mYMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), yMinIndices, 1, vertices));
                makeVertices(&vertices, mYMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), yMaxIndices, 1, vertices));
            } else {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mZMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), zMinIndices, 2, vertices));
                makeVertices(&vertices, mZMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), zMaxIndices, 2, vertices));
            }
        } else if (parent->getRigidSplitDirection() == 1) {
            //evaluate only x and z
            if (     abs( (int)(xMinIndicesSize - xMaxIndicesSize) )
                        <= abs( (int)(zMinIndicesSize - zMaxIndicesSize) )) {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mXMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), xMinIndices, 0, vertices));
                makeVertices(&vertices, mXMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), xMaxIndices, 0, vertices));
            } else {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mZMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), zMinIndices, 2, vertices));
                makeVertices(&vertices, mZMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), zMaxIndices, 2, vertices));
            }
        } else {
            //evaluate only x and y
            if (     abs( (int)(xMinIndicesSize - xMaxIndicesSize) )
                        <= abs( (int)(yMinIndicesSize - yMaxIndicesSize) )) {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mXMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), xMinIndices, 0, vertices));
                makeVertices(&vertices, mXMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), xMaxIndices, 0, vertices));
            } else {
                std::vector<Vertex*> vertices;
                makeVertices(&vertices, mYMinVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), yMinIndices, 1, vertices));
                makeVertices(&vertices, mYMaxVertices);
                result.push_back(new MeshPart(parent->getReferenceMesh(), yMaxIndices, 1, vertices));
            }

        }

        if (result.size() != 2) {
            throw Exception("DefaultMeshSplitter internal error: did not create exactly 2 MeshPart objects");
        }

        //Sanity check:
        //if all splits are bad  (even the best one has one "empty" part),
        //split into two halves, index-based
        if (result.front()->getTriangleIndices().empty() ||
            result.back()->getTriangleIndices().empty()) {
            delete result.front();
            result.pop_front();
            delete result.front();
            result.pop_front();

            std::list<int> firstIndices;
            std::list<int> lastIndices;
            unsigned int i = 0;
            unsigned int parentIndicesSize = parentIndices.size();
            for (std::list<int>::const_iterator indexIter
                        = parentIndices.begin();
                        indexIter != parentIndices.end();
                        indexIter++) {
                if (i < parentIndicesSize / 2) {
                    firstIndices.push_back(*indexIter);
                } else {
                    lastIndices.push_back(*indexIter);
                }
                ++i;
            }
            result.push_back(new MeshPart(parent->getReferenceMesh(), firstIndices));
            result.push_back(new MeshPart(parent->getReferenceMesh(), lastIndices));
        }

        // clean up for the next calculateSplit() call
        for (ListNode<MyVertex*>* node = mUsedVertices.getFirstNode(); node; node = node->getNext()) {
            node->getData()->reset();
        }
        mUsedVertices.clear();
        return result;
    }

    void DefaultMeshSplitter::makeVertices(std::vector<Vertex*>* vertices, const List<Vertex*>& list) {
        vertices->clear();
        vertices->reserve(list.size());
        for (ListNode<Vertex*>* node = list.getFirstNode(); node; node = node->getNext()) {
            vertices->push_back(node->getData());
        }
//        vertices->assign(list.begin(), list.end());
    }
}

/*
 * vim: et sw=4 ts=4
 */
