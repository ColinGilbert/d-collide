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

#include "proxy.h"

#include "shapes/box.h"
#include "shapes/mesh.h"
#include "shapes/mesh/meshfactory.h"
#include "boundingvolumes/aabb.h"

namespace dcollide {
    Box::~Box() {
        delete mMesh;
    }

    /*!
     * \brief Generates a simple box mesh out of twelve triangles according to
     *        \ref getDimension.
     * The method \ref MeshFactory::createBox is used for this task
     */
    void Box::generateMesh() {
        if (mMesh) {
            return;
        }
        MeshFactory meshFactory;
        mMesh = meshFactory.createBox(mDimension);
        mMesh->setProxy(mProxy);
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
     */
    const Vector3* Box::getVertices() {
        const Matrix& m = getProxy()->getWorldTransformation();

        //GJ: to save some Matrix multiplications, we calculate 3 edge vectors
        //and generate the vertices by vector-addition

        mVertices[0] = m.getPosition();

        //edge vector i is the i-th column of the rotation matrix
        Vector3 xEdge(  m.getElement(0,0) * mDimension.getX(),
                        m.getElement(1,0) * mDimension.getX(),
                        m.getElement(2,0) * mDimension.getX());
        Vector3 yEdge(  m.getElement(0,1) * mDimension.getY(),
                        m.getElement(1,1) * mDimension.getY(),
                        m.getElement(2,1) * mDimension.getY());
        Vector3 zEdge(  m.getElement(0,2) * mDimension.getZ(),
                        m.getElement(1,2) * mDimension.getZ(),
                        m.getElement(2,2) * mDimension.getZ());

        mVertices[1] =  mVertices[0] + xEdge;
        mVertices[2] =  mVertices[0] + yEdge;
        mVertices[3] =  mVertices[0] + xEdge + yEdge;
        mVertices[4] =  mVertices[0] + zEdge;
        mVertices[5] =   mVertices[0] + xEdge + zEdge;
        mVertices[6] =   mVertices[0] + yEdge + zEdge;
        mVertices[7] =   mVertices[0] + xEdge + yEdge +zEdge;

        return mVertices;
    }

    /*!
     * \brief calculates and returns all 8 vertices in world space
     *
     * \param state Calculates the vertices, assuming that \p state is the
     * actual state of the box
     */
    const Vector3* Box::getVertices(const Matrix& state) {

        //GJ: to save some Matrix multiplications, we calculate 3 edge vectors
        //and generate the vertices by vector-addition

        mVertices[0] = state.getPosition();

        //edge vector i is the i-th column of the rotation matrix
        Vector3 xEdge(  state.getElement(0,0) * mDimension.getX(),
                        state.getElement(1,0) * mDimension.getX(),
                        state.getElement(2,0) * mDimension.getX());
        Vector3 yEdge(  state.getElement(0,1) * mDimension.getY(),
                        state.getElement(1,1) * mDimension.getY(),
                        state.getElement(2,1) * mDimension.getY());
        Vector3 zEdge(  state.getElement(0,2) * mDimension.getZ(),
                        state.getElement(1,2) * mDimension.getZ(),
                        state.getElement(2,2) * mDimension.getZ());

        mVertices[1] =  mVertices[0] + xEdge;
        mVertices[2] =  mVertices[0] + yEdge;
        mVertices[3] =  mVertices[0] + xEdge + yEdge;
        mVertices[4] =  mVertices[0] + zEdge;
        mVertices[5] =   mVertices[0] + xEdge + zEdge;
        mVertices[6] =   mVertices[0] + yEdge + zEdge;
        mVertices[7] =   mVertices[0] + xEdge + yEdge +zEdge;

        return mVertices;
    }

    /*!
     * \returns the rotated center of the box
     *
     * \param vs if given, use these vertices instead of recalculating them
     */
    const Vector3 Box::getCenter(const Vector3* vs) {

        if (!vs) {
            vs = getVertices();
        }

        Vector3 center = (vs[7]+vs[0])/2;

        return center;
    };

    /*!
     * \returns the unrotated center of the box
     *
     */
    const Vector3 Box::getUnrotatedCenter() const {
        const Matrix& m = getProxy()->getWorldTransformation();

        Vector3 center = (m.getPosition()+mDimension/2);
        return center;
    }

    /*!
     * \brief calculates the planes of the box
     *
     * \param vs if given, use these vertices instead of recalculating them
     */
    const Plane* Box::getPlanes(const Vector3* vs) {


        if (!vs) {
            vs = getVertices();
        }
        // opposing planes along each axis (see apidocs of Box::getVertices()
        // for the order of the returned vertices):
        // normal = crossproduct of 2 vectors representing the face, use
        // right-hand-rule to determine in what direction the normal points
        // x: verts 0,2,4,6 vs. 1,3,5,7
        Vector3 norm;
        // plane 0: normal points OUTSIDE the box!
        mPlanes[0] = Plane((vs[2]-vs[0])*(vs[4]-vs[0]),vs[0],true);
        // plane 1: normal points OUTSIDE the box!
        norm = (vs[5]-vs[1])*(vs[3]-vs[1]);
        mPlanes[1] = Plane(norm,vs[1],true);
        // y: verts 0,1,4,5 vs. 2,3,6,7
        // plane 2: normal points OUTSIDE the box!
        norm = (vs[4]-vs[0])*(vs[1]-vs[0]);
        mPlanes[2] = Plane(norm,vs[0],true);
        // plane 3: normal points OUTSIDE the box!
        mPlanes[3] = Plane((vs[3]-vs[2])*(vs[6]-vs[2]),vs[2],true);
        // z: verts 0,1,2,3 vs. 4,5,6,7
        // plane 4: normal points OUTSIDE the box!
        mPlanes[4] = Plane((vs[1]-vs[0])*(vs[2]-vs[0]),vs[0],true);
        // plane 5: normal points OUTSIDE the box!
        norm = (vs[6]-vs[4])*(vs[5]-vs[4]);
        mPlanes[5] = Plane(norm,vs[4],true);
        return mPlanes;
    }

    /*!
     * \brief the "edges" of the box
     * 
     * \param edges Here we save the edges, each pair represents one edge
     * beetween two verteces indexed by these two ints
     */
    const void Box::getEdges(std::pair<int,int>(&edges)[12]) const {
        // defining them:
        edges[0] = std::pair<int,int>(0,1);
        edges[1] = std::pair<int,int>(1,3);
        edges[2] = std::pair<int,int>(2,3);
        edges[3] = std::pair<int,int>(2,0);
        edges[4] = std::pair<int,int>(0,4);
        edges[5] = std::pair<int,int>(1,5);
        edges[6] = std::pair<int,int>(2,6);
        edges[7] = std::pair<int,int>(3,7);
        edges[8] = std::pair<int,int>(4,5);
        edges[9] = std::pair<int,int>(5,7);
        edges[10] = std::pair<int,int>(7,6);
        edges[11] = std::pair<int,int>(6,4);
    }


    std::ostream& operator<<(std::ostream& os, const dcollide::Box& v) {
        os << "(Box: dim = " << v.getDimension() << ")";
        return os;
    }
}

/*
 * vim: et sw=4 ts=4
 */
