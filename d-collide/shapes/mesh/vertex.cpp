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

#include "shapes/mesh/vertex.h"

#include "shapes/mesh/triangle.h"
#include "shapes/mesh.h"
#include "proxy.h"
#include "debugstream.h"

namespace dcollide {

    /*!
     * \brief Creates a vertex which is located at \p position.
     *
     * Due to the initialization mContainingMesh is set to null and the cached
     * world position is marked as invalid.
     */
    Vertex::Vertex(const Vector3& position) {
        mPosition = position;
        mMeshListNode = 0;
        mContainingMesh = 0;
        mIsWorldPositionValid = false;
    }

    /*!
     * \overload
     * \brief Creates a vertex which is located at Vector3(\p x, \p y, \p z).
     * 
     * Due to the initialization mContainingMesh is set to null and the cached
     * world position is marked as invalid.
     */
    Vertex::Vertex(real x, real y, real z) {
        mPosition = Vector3(x, y, z);
        mMeshListNode = 0;
        mContainingMesh = 0;
        mIsWorldPositionValid = false;
    }

    /*!
     * \brief Transforms the position of the vertex into world coordinates and
     *        chaches it for later use.
     * 
     * To transform the position of this vertex into world coordinates the given 
     * mContainingMesh is used to get the transformation matrix. Which is then
     * used to transform the position. After a call to this method the cached
     * world position will be valid and further calls to
     * Vertex::getWorldPosition() won't produce great costs.
     */
    void Vertex::updateWorldPosition() {
        if (!mContainingMesh) {
            throw NullPointerException("Vertex::mContaingMesh");
        }

        if (mContainingMesh->hasLocalCoordinates()) {
            Matrix matrix = mContainingMesh->getProxy()->getWorldTransformation();

            matrix.transform(&mWorldPosition, mPosition);
        } else {
            mWorldPosition = mPosition;
        }

        mIsWorldPositionValid = true;
        mContainingMesh->addValidWorldPositionVertex(this);
    }

    /*!
     * \internal
     * \brief Invalidates the cached normal vectors of each adjecent triangle 
     */
    void Vertex::invalidateTriangleNormals() const {

        for (std::list<Triangle*>::const_iterator iter = mAdjacentTriangles.begin();
             iter != mAdjacentTriangles.end();
             ++iter) {

            (*iter)->invalidateNormals();
        }
    }

    /*!
     * \internal
     * \brief Invalidates the cached normal vectors which are transformed into
     *        world coordinates of each adjecent triangle
     */
    void Vertex::invalidateTriangleWorldCoordinatesNormalVector() const {

        for (std::list<Triangle*>::const_iterator iter = mAdjacentTriangles.begin();
             iter != mAdjacentTriangles.end();
             ++iter) {
            
            (*iter)->invalidateWorldCoordinatesNormalVector();
        }
    }

    /*!
     * \brief Invalidates the cached world position.
     * 
     * This method invalidates the cached world position of this vertex and all
     * cached normal vectors which are transformed into world coordinates of all
     * triangles that contains this vertex. 
     */
    void Vertex::invalidateWorldPosition() {
        mIsWorldPositionValid = false;

        if (mContainingMesh) {
            mContainingMesh->invalidateVertexWorldPosition(this);
        }
        invalidateTriangleWorldCoordinatesNormalVector();
    }

    /*!
     * \brief Sets the coorinates of this vertex to the the position
     * 
     * The cached world position and all depending triangle normals are
     * invalidated through a call to this function.
     */
    void Vertex::setPosition(real x, real y, real z) {
        mPosition = Vector3(x, y, z);
        mIsWorldPositionValid = false;

        if (mContainingMesh && mContainingMesh->hasLocalCoordinates()) {
            mContainingMesh->invalidateVertexWorldPosition(this);
        }
        invalidateTriangleNormals();
    }

    /*!
     * \brief Translate/moves the vertex according to the given vector
     *        \p translateBy
     * 
     * A call to this method will invalidate the cached world position of this
     * vertex and the cached normal vectors of all triangles that contains this 
     * vertex. The translationis done by simply adding the given vector
     * \p translateBy onto mPosition.
     */
    void Vertex::translate(const Vector3& translateBy) {
        mPosition = mPosition + translateBy;
        mIsWorldPositionValid = false;

        if (mContainingMesh && mContainingMesh->hasLocalCoordinates()) {
            mContainingMesh->invalidateVertexWorldPosition(this);
        }
        invalidateTriangleNormals();
    }


    /*!
     * \brief Returns the cached position of this vertex in world coordinates
     * 
     * When the cached transformed position is marked as invalid a valid one is
     * calculated through Vertex::updateWorldPosition().
     * To prevent unnecessarily recalculation of the world position a mutex lock 
     * is used every time the Vertex::updateWorldPosition() must be called.
     */
    const Vector3& Vertex::getWorldPosition() const {

        if (mContainingMesh && mContainingMesh->hasLocalCoordinates()) {

            MutexLocker lock(&mMutex);

            if (!mIsWorldPositionValid) {
                const_cast<Vertex*>(this)->updateWorldPosition();
            }
            return mWorldPosition;

        } else {
            return mPosition;
        }
    }


    DebugStream operator<<(DebugStream s, const Vertex& v) {
        s << v.getPosition();
        return s;
    }
}
/*
 * vim: et sw=4 ts=4
 */
