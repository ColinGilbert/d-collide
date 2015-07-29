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
#include "math/vector.h"
#include "math/matrix.h"
#include "math/plane.h"
#include "real.h"
#include "shapes/shapes.h"
#include "proxy.h"
#include "collisioninfo.h"
#include "narrowphase/boxboxintersector.h"
#include "narrowphase/intersectionhelpers.h"

#include "dcollide-defines.h"

#include <math.h>
#include <iostream>

#define BOXBOXDEBUG 1
#undef BOXBOXDEBUG

namespace dcollide {

    BoxBoxIntersector::BoxBoxIntersector() {}

    BoxBoxIntersector::~BoxBoxIntersector() {}


    /* Box-box collisions: FAST
    --------------------------------------------------------------------------*/

    /*!
     * \brief Compute range of projection of a \p box onto an \p axis.
     */
    void BoxBoxIntersector::computeSpan(const Vector3* vertices, const Vector3& axis, real& min, real& max) const {
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
     *  \brief Find the collision feature (face, edge or vertex) of a \p box
     *         along a \p normal given a \p state in form of a \ref Matrix.
     *
     *  The method will set \p vertices and \p n to the number of vertices
     *  returned.
     */
    void BoxBoxIntersector::findSupportingFeature(const Matrix& state,
                                            const Vector3* boxVertices,
                                            const Vector3& normal,
                                            int& n,
                                            Vector3 (&vertices)[4]) const {

        /* 1. Scan boxes for aligned faces
        --------------------------------------------------------------------*/

        for (int i=0; i<3; i++) {
            const Vector3& axis = state.getAxis(i);

            // collinear with one axis?
            if (fabs(normal.dotProduct(axis)) >= 1.0f - 1.0E-8f) {
                n = 4;

                int axisBit = 1 << i;

                // determine correct face
                real a = normal.dotProduct(boxVertices[0]);
                real b = normal.dotProduct(boxVertices[axisBit]);

                if (a > b) {
                    int pos = 0;
                    for (int j=0; j<8; j++) {
                        if (!(j & axisBit)) { // take 4 verts with "axisBit" unset

                            vertices[pos] = boxVertices[j];
                            pos++;
                        }
                    }
                } else {
                    int pos = 0;
                    for (int j=0; j<8; j++) {
                        if (j & axisBit) { // take 4 verts with "axisBit" set
                            vertices[pos] = boxVertices[j];
                            pos++;
                        }
                    }
                }

                return;
            }
        }

        /* 2. Scan boxes for perpendicular edges
        --------------------------------------------------------------------*/

        // perpendicular to the MTV, and with one vertex being the support.
        // fabs(edge.direction dot MTV.direction) <= (0.0f + tolerance)
        // edge.start dot MTV.direction = max(vertex[0...n] dot MTV.direction)

        for (int i=0; i<3; i++) {
            const Vector3& axis = state.getAxis(i);

            // not collinear with any axis, but perpendicular to (exactly) one
            if (fabs(normal.dotProduct(axis)) <= 1.0E-8f) { // ~= 0 ?
                n = 2;

                int axisBit = 1 << i;

                int index = 0;
                real val = normal.dotProduct(boxVertices[0]);

                for (int j=0; j<8; j++) {
                    if (j & axisBit) {
                        continue;
                    }

                    real d = normal.dotProduct(boxVertices[j]);
                    if (d > val) {
                        val = d;
                        index = j;
                    }
                }

                vertices[0] = boxVertices[index];
                vertices[1] = boxVertices[index + axisBit];

                return;
            }
        }


        /* 3. Determine unique support point
        --------------------------------------------------------------------*/

        // vertex dot MTV.direction = max(vertex[0...n] dot MTV.direction)
        n = 1;
        int index = 0;
        real maxval = normal.dotProduct(boxVertices[0]);
        for (int i=1; i<8; i++) {
            real d = normal.dotProduct(boxVertices[i]);
            if (d > maxval) {
                maxval = d;
                index = i;
            }
        }

        vertices[0] = boxVertices[index];
    }


    /*!
     * \brief Calculates the Intersection beetween two flats 
     *
     * The return value will be written to \p retVertices. Only the x and y
     * value of each vertex is checked, as these flats are supposed to be 
     * parallel to each other.
     *
     *        o------o
     *        |      |
     *    o---*--*=o |       given: o
     *    |   |  |   |    returned: x
     *    | o=*--*---o
     *    |      |
     *    o------o
     *
     * \param vertsFlat1 The vertices of flat 1
     * \param n1 the amount of vertices of flat 1
     * \param vertsFlat2 The vertices of flat 2
     * \param n2 the amount of vertices of flat 2
     * \param retVertices Vertices of the intersection-flat
     *
     * \returns true if an intersection was found. Beware: returns false if no
     * intersection at all OR more than 2 intersection vertices were found,
     * which means that \p vertsFlat2 lies completely inside \p vertsFlat2 !
     */
    bool BoxBoxIntersector::getFlatIntersection(Vector3* vertsFlat1,int n1,
            Vector3* vertsFlat2, int n2,Vector3 (& retVertices)[4], int& nVerts) const {

        // counter for the amount of already found vertices:
        nVerts = 0;

        // Check if we were given 4 vertices for each flat:
        if ( n1 < 4 || n2 < 4) {
            return false;
        }

        // Find vertexes of vertsFlat2 that are inside the other flat and vice
        // versa:
        // -------------------------------------------------------
        // Calc. Min and Max of verts1:
        real verts1Min[3];
        real verts1Max[3];
        getVertsMinMax(vertsFlat1,n1,verts1Min,verts1Max);

        // If 2 out of 3 x,y and z of a vertex of verts2 is beetween the 
        // intervall of min and max of verts1 => vertex found!
        for (int i = 0;i<4;++i) {
            int in = 0;
            if ((verts1Min[0] <= vertsFlat2[i].getX() && 
                    verts1Max[0] >= vertsFlat2[i].getX())) {
                ++in;
            }
            if (verts1Min[1] <= vertsFlat2[i].getY() && 
                    verts1Max[1] >= vertsFlat2[i].getY()) {
                ++in;
            }
            if (verts1Min[2] <= vertsFlat2[i].getZ() && 
                    verts1Max[2] >= vertsFlat2[i].getZ()) {
                ++in;
            }

            // insert this vertex if we have found at least 2 intervall matches:
            if (in >= 2) {

                // return if we have found more than 3: If we habe found more 
                // then 3, we are completely inside the other box
                if ((nVerts+1) > 3) {
                    return false;
                }
                retVertices[nVerts] = vertsFlat2[i];
                ++nVerts;
            }
        }

        // Do the same for vertsFlat1:
        // Calc. Min and Max of verts2:
        real verts2Min[3];
        real verts2Max[3];
        getVertsMinMax(vertsFlat2,n2,verts2Min,verts2Max);

        // If 2 out of 3 x,y and z of a vertex of verts2 is beetween the 
        // intervall of min and max of verts1 => vertex found!
        for (int i = 0;i<4;++i) {
            int in = 0;
            if (verts2Min[0] <= vertsFlat1[i].getX() && 
                    verts2Max[0] >= vertsFlat1[i].getX()) {
                ++in;
            }
            if (verts2Min[1] <= vertsFlat1[i].getY() && 
                    verts2Max[1] >= vertsFlat1[i].getY()) {
                ++in;
            }
            if (verts2Min[2] <= vertsFlat1[i].getZ() && 
                    verts2Max[2] >= vertsFlat1[i].getZ()) {
                ++in;
            }

            // insert this vertex if we have found at least 2 intervall matches:
            if (in >= 2) {

                // return if we have found more then 3: If we found more 
                // then 3, we are completely inside the other box
                if ((nVerts+1) > 3) {
                    return false;
                }
                retVertices[nVerts] = vertsFlat1[i];
                ++nVerts;
            }
        }

        // Now we must find the remaining points by doing intersection tests:
        // DISABLED! FIXME, doesn't work and I'm not sure if we need to be so
        // accurate
        /* -------------------------------------------------------
        Vector3 vertsIntersection[2];
        int nVertsIntersection = getLineIntersectionPoints(vertsFlat1,n1,
                vertsFlat2,n2,vertsIntersection);

        // if we have found intersection points, these are the vertices we 
        // looked for:
        if (nVertsIntersection > 0) {
            std::cout << "TDTDTD" << std::endl;
            for (int i = 0;i<nVertsIntersection;++i) {
                retVertices[nVerts] = vertsIntersection[i];
                ++nVerts;
            }
        }*/


        return true;
    }

    /*!
     * \brief Calculates the intersection points 
     *
     * Calc. intersection points beetween the two given flats, described by \p
     * vertsFlat1 and \p vertsFlat2. Only x and y are used for this task, as
     * these flats are parallel in z
     *
     * \param vertsFlat1 The vertices of flat 1
     * \param n1 the amount of vertices of flat 1
     * \param vertsFlat2 The vertices of flat 2
     * \param n2 the amount of vertices of flat 2
     * \param retVertices Vertices of the intersection-flat
     * \returns the amount of intersection points
     */
    int BoxBoxIntersector::getLineIntersectionPoints(Vector3* vertsFlat1, int n1,
            Vector3* vertsFlat2, int n2,
            Vector3 (& retVertices)[2]) const {

        int pointsFound = 0;
        // Only if we were given 4 vertices for each flat, otherwise we have no
        // flats!
        if (n1 != 4 || n2 != 4) {
            return pointsFound;
        }

        // First we need the 4 lines of each flat
        std::vector<real*> linesFlat1;
        std::vector<real*> linesFlat2;
        for (int l1a = 0;l1a<4;++l1a) {
            for (int l1b = l1a;l1b<4;++l1b) {
                real lines[6];
                lines[0] = vertsFlat1[l1a].getX();
                lines[1] = vertsFlat1[l1a].getY();
                lines[2] = vertsFlat1[l1a].getZ();
                lines[3] = vertsFlat1[l1b].getX();
                lines[4] = vertsFlat1[l1b].getY();
                lines[5] = vertsFlat1[l1b].getZ();
                linesFlat1.push_back(lines);
            }
        }
        for (int l2a = 0;l2a<4;++l2a) {
            for (int l2b = l2a;l2b<4;++l2b) {
                real lines[6];
                lines[0] = vertsFlat2[l2a].getX();
                lines[1] = vertsFlat2[l2a].getY();
                lines[2] = vertsFlat2[l2a].getZ();
                lines[3] = vertsFlat2[l2b].getX();
                lines[4] = vertsFlat2[l2b].getY();
                lines[5] = vertsFlat2[l2b].getZ();
                linesFlat2.push_back(lines);
            }
        } 

        // Now we must find the lines which really are edges of this flat:
        // TODO

        // Checking which lines are crossing, and if they do, also calculate
        // the intersection points:
        for (int i1 = 0; i1 < 4;++i1) {
            for (int i2 = 0; i2 < 4;++i2) {
                if (overlap((linesFlat1[i1])[0],(linesFlat1[i1])[2],
                    (linesFlat2[i2])[0],(linesFlat2[i2])[2]) && 
                    overlap((linesFlat1[i1])[1],(linesFlat1[i1])[3],
                    (linesFlat2[i2])[1],(linesFlat2[i2])[3])) {
                    // Bingo, intersection found!
                    // More than 2? BAD!
                    if ((pointsFound+1)>2) {
                        return 0; 
                    }

                    // calc. intersection point, atm this is only an
                    // approximation, but I don't know if we need this more
                    // accurate.
                    real x,y,z;
                    // We take the x-coords of the line with the greater
                    // intervall on x:
                    if (((linesFlat1[i1])[2]-(linesFlat1[i1])[0]) > 
                        ((linesFlat2[i2])[2]-(linesFlat2[i2])[0])) { 
                        // Approximation: x = (xMin+xMax)/2
                        x = ((linesFlat1[i1])[2]+(linesFlat1[i1])[0])/2;
                        // Approximation: y = (yMin+yMax)/2
                        y = ((linesFlat2[i2])[1]+(linesFlat2[i2])[3])/2;
                    } else {
                        x = ((linesFlat2[i2])[2]+(linesFlat2[i2])[0])/2;
                        y = ((linesFlat1[i1])[1]+(linesFlat1[i1])[3])/2;
                    }
                    z = vertsFlat2[i2].getZ();

                    retVertices[pointsFound] = Vector3(x,y,z);
                    ++pointsFound;

                }
            }
        }

        /* Cleanup:
        while (linesFlat1.size()) { 
            if (linesFlat1.back()) {
                delete linesFlat1.back();
            }
            linesFlat1.pop_back();
        }
        while (linesFlat2.size()) { 
            if (linesFlat1.back()) {
                delete linesFlat2.back();
            }
            linesFlat2.pop_back();
        }*/
        return pointsFound;
    }

    /*!
     * \brief Calculates the collision between two boxes.
     */
    std::list<CollisionInfo> BoxBoxIntersector::getIntersectionFast(
            const Matrix& currentState1, Box* box1, 
            const Matrix& currentState2, Box* box2) const {

        std::list<CollisionInfo> results;

        /* --------------------------------------------------- *\
                         separating axis theorem
        \* --------------------------------------------------- */

        //calculate vertices of both boxes once now, they will be used
        //in several places
        const Vector3* box1vertices = box1->getVertices();
        const Vector3* box2vertices = box2->getVertices();
        
        // store intervals and axes
        real min1[15], max1[15];
        real min2[15], max2[15];
        Vector3 axis[15];

        // test all three normals in box1
        for (int i=0; i<3; i++) {
            axis[i] = currentState1.getAxis(i);

            computeSpan(box1vertices, axis[i], min1[i], max1[i]);
            computeSpan(box2vertices, axis[i], min2[i], max2[i]);

            if (!overlap(min1[i], max1[i], min2[i], max2[i])) {
                return results;
            }
        }

        // test all three normals in box2
        for (int i=0; i<3; i++) {
            int j = i + 3;

            axis[j] = currentState2.getAxis(i);

            computeSpan(box1vertices, axis[j], min1[j], max1[j]);
            computeSpan(box2vertices, axis[j], min2[j], max2[j]);

            if (!overlap(min1[j], max1[j], min2[j], max2[j])) {
                return results;
            }
        }

        // test all of the nine cross products
        for (int i=0; i<3; i++) {
            for (int k=0; k<3; k++) {
                int j = (3*i) + k + 6;

                axis[j] = currentState1.getAxis(i) * currentState2.getAxis(k);

                computeSpan(box1vertices, axis[j], min1[j], max1[j]);
                computeSpan(box2vertices, axis[j], min2[j], max2[j]);

                if (!overlap(min1[j], max1[j], min2[j], max2[j])) {
                    return results;
                }
            }
        }


        /* Bingo, we most probably have a collision here! *\
        \* Now we need to find all intersections:         */

        // CHECK: if the SAT-algorithm above fails and "detects" a
        //        collision where there is none, should we handle
        //        this and re-check if there is a collision, considering
        //        the exact shapes?

        // Calculate minimal translation distance (MTD)
        real depth = 0.0;
        Vector3 normal;
        real length = 0.0;
        bool unset = true;

        // Walk through all 15 axes (that's three for each box plus
        // the nine cross products) and find minimum
        for (int i=0; i<15; i++) {
            real min = (min1[i] > min2[i]) ? min1[i] : min2[i];
            real max = (max1[i] < max2[i]) ? max1[i] : max2[i];

            real d = max - min;
            real l = axis[i].length();

            if (l < 1.0E-8f) { // TODO: Move constant somewhere central?
                continue;
            }

            d /= l;

            if (unset || d < depth) {
                depth = d;
                normal = axis[i];
                length = l;
                unset = false;
            }
        }

        normal.scale((real)1.0 / length);

        // Assure normal points from box2 to box1
        Vector3 diff = currentState2.getPosition() - currentState1.getPosition();
        if (diff.dotProduct(normal) > 0.0) {
            normal.scale(-1);
        }

        /* At this point we have                        *\
         * - "normal" MTV (minimum translation vector)  *
        \* - "depth" MTD (minimum translation distance) */

        // find supporting features (faces, edges or points)

        int n1 = 0;
        Vector3 verts1[4];

        findSupportingFeature(currentState1, box1vertices, -normal, n1, verts1);

        CollisionInfo info;
        if (box2->getProxy()->getMoveFlags() & Proxy::MOVEFLAG_UNMOVED) {
            info.penetratedProxy = box2->getProxy();
            info.penetratingProxy = box1->getProxy();
            info.normal = normal;
        } else {
            info.penetratedProxy = box1->getProxy();
            info.penetratingProxy = box2->getProxy();
            info.normal = -normal;
        }

        info.penetrationDepth = depth;

        // return feature (of any box) with least vertices

        if (n1 == 1) {
            info.collisionPoint = verts1[0];
            results.push_back(info);
            return results;
        }

        int n2 = 0;
        Vector3 verts2[4];
        findSupportingFeature(currentState2, box2vertices, normal, n2, verts2);

#ifdef BOXBOXDEBUG
        //* 1. debug output of found vertices:
        for (int i = 0; i<n1; ++i) {
            std::cout << "Verts1."<< i << " : " << verts1[i] << std::endl;
        }
        for (int i = 0; i<n2; ++i) {
            std::cout << "Verts2."<< i << " : " << verts2[i] << std::endl;
        }
#endif

        // checking wether we should return all vertices or only some,
        // but only if we have 4 returned vertices for each box 
        // ------------------------------------------------------
        if (n1 == 4 && n2 == 4) {


            Vector3 vertsFlat[4];
            int nVertsFlat;

            // Calculate intersection
            bool hasIntersection = false;
            hasIntersection = getFlatIntersection(verts1, n1,
                    verts2,n2,vertsFlat,nVertsFlat);

            if (hasIntersection) {
#ifdef BOXBOXDEBUG
                for (int i = 0; i<nVertsFlat; ++i) {
                    std::cout << "VertsFlat."<< i << " : " << vertsFlat[i] << std::endl;
                }
#endif
                // approximate volume enclosed of these vertices 
                real min[3];
                real max[3];
                getVertsMinMax(vertsFlat,nVertsFlat,min,max);
                // None of the intervalls should be < 1! If so, the plane
                // is nearly planar with one axis
                real i1 = (fabs(max[0]-min[0]) < 1.0) ? 1 : (max[0]-min[0]);
                real i2 = (fabs(max[1]-min[1]) < 1.0) ? 1 : (max[1]-min[1]);
                real i3 = (fabs(max[2]-min[2]) < 1.0) ? 1 : (max[2]-min[2]);
                real surfaceOfIntersection = i1*i2*i3;
#ifdef BOXBOXDEBUG
                std::cout << "Flat min: " << min[0] << " " << min[1] << " "
                    << min[2] << " max: " << max[0] << " " << max[1] << " "
                    << max[2] <<
                    " => surface = " << surfaceOfIntersection << std::endl;

#endif
                // approximate volume enclosed of the other vertices:
                getVertsMinMax(verts2,n2,min,max);
                // None of the intervalls should be < 1! If so, the plane
                // is planar with one axis
                i1 = (fabs(max[0]-min[0]) < 1.0) ? 1 : (max[0]-min[0]);
                i2 = (fabs(max[1]-min[1]) < 1.0) ? 1 : (max[1]-min[1]);
                i3 = (fabs(max[2]-min[2]) < 1.0) ? 1 : (max[2]-min[2]);
                real surfaceOfVert = i1*i2*i3;

#ifdef BOXBOXDEBUG
                std::cout << "verts min: " << min[0] << " " << min[1] << " "
                    << min[2] << " max: " << max[0] << " " << max[1] << " "
                    << max[2] <<
                    " => surface = " << surfaceOfVert << std::endl;

#endif
                // now we change the vertex list if intersectionSurface
                // is lower than half of the surface of the verts:
                if (surfaceOfVert/2 > surfaceOfIntersection ) {
                    for (int i = 0;i<nVertsFlat;++i) {
                        verts2[i] = vertsFlat[i];
                    }
                    n2=nVertsFlat;
                }

            } // END if (getFlatIntersection(verts1, n1,verts2,n2,

            // ------------------------------------------------------
            // If no FlatIntersection found, just proceed with unchanged
            // vertex-lists
            // ------------------------------------------------------

        } // END if (n1 == 4 && n2 == 4) {

#ifdef BOXBOXDEBUG
        //* 2. debug output of found vertices:
        for (int i = 0; i<n1; ++i) {
            std::cout << "AVerts1."<< i << " : " << verts1[i] << std::endl;
        }
        for (int i = 0; i<n2; ++i) {
            std::cout << "AVerts2."<< i << " : " << verts2[i] << std::endl;
        }//
#endif

        // Now really return the results:
        // return the results with less points:
        // ------------------------------------------------------
        if (n1 < n2) {
            for (int i=0; i<n1; i++) {
                info.collisionPoint = verts1[i];
                results.push_back(info);
            }

            return results;
        }

        for (int i=0; i<n2; i++) {
            info.collisionPoint = verts2[i];
            results.push_back(info);
        }

        return results;
    }

    /*!
     * \brief get min and max of a collection of Vertices 
     */
    void BoxBoxIntersector::getVertsMinMax(Vector3* verts, int nVerts, 
            real (& min)[3], real (& max)[3]) const {

        max[0] = verts[0].getX();
        max[1] = verts[0].getY();
        max[2] = verts[0].getZ();
        min[0] = verts[0].getX();
        min[1] = verts[0].getY();
        min[2] = verts[0].getZ();
        for (int i = 1;i<nVerts;++i) {
            if (max[0] < verts[i].getX()) {
                max[0] = verts[i].getX();
            }
            if (min[0] > verts[i].getX()) {
                min[0] = verts[i].getX();
            }
            if (max[1] < verts[i].getY()) {
                max[1] = verts[i].getY();
            }
            if (min[1] > verts[i].getY()) {
                min[1] = verts[i].getY();
            }
            if (max[2] < verts[i].getZ()) {
                max[2] = verts[i].getZ();
            }
            if (min[2] > verts[i].getZ()) {
                min[2] = verts[i].getZ();
            }
        }
    }




    /* Box-box collisions: ACCURATE
    --------------------------------------------------------------------------*/
    /*!
     * \brief Calculates Collisions beetween two boxes
     * 
     * this is done by checking planes vs. vertices and edges. To calculate the
     * penetrationdepth we also use the previous state of the two boxes, which
     * is somehow the famous time-coherency!
     *
     * \param box1 The first box
     * \param prevState1 The previous state of the first box
     * \param box2 The second box
     * \param prevState2 The previous state of the second box
     */
    std::list<CollisionInfo> BoxBoxIntersector::getIntersectionAccurate(
                Box* box1,const Matrix& previousState1,Box* box2,
                const Matrix& previousState2) const {
        // IntersectionHelper:
        IntersectionHelpers helpers;

        DCOLLIDE_UNUSED(previousState1);
        DCOLLIDE_UNUSED(previousState2);

        // Getting the centers and the difference of the boxes in the previous 
        /* state:
        const Vector3 prevCenter1 =
            box1->getCenter(box1->getVertices(previousState1));
        const Vector3 prevCenter2 =
            box2->getCenter(box2->getVertices(previousState2));
        const Vector3 prevCenterDiff12 = prevCenter2-prevCenter1;
        const Vector3 prevCenterDiff21 = prevCenter1-prevCenter2;

        // getting the position of the the previous states:
        const Vector3 prevPosition1 = previousState1.getPosition();
        const Vector3 prevPosition2 = previousState2.getPosition();
        const Vector3 diff1 =
            prevPosition1-box1->getProxy()->getWorldTransformation().getPosition();
        const Vector3 diff2 =
            prevPosition2-box2->getProxy()->getWorldTransformation().getPosition();
        // Now we calculate the not-sqrt-ed-length beetween old and new 
        // state, which is the same as the travelled distance beetween new 
        // and old
        real l1 = diff1.lengthWithoutSqrt();
        real l2 = diff2.lengthWithoutSqrt();*/

        // The Container for the returned Collisions:
        std::list<CollisionInfo> results;
        // tempory containers:
        CollisionInfo resultsTempBox1[8];
        CollisionInfo resultsTempBox2[8];

        /* Preparing CollisionInfo::
         -----------------------------------------------------------------*/
        CollisionInfo info;
        bool box1Penetrator = false;
        // Check which proxy is penetrator:
        // The Proxy that moved more beetween old and new state is the
        // penetrator
        //if (l1 < l2) {
        if (box1->getProxy()->getMoveFlags() & Proxy::MOVEFLAG_UNMOVED) {
            info.penetratingProxy = box2->getProxy();
            info.penetratedProxy = box1->getProxy();
            box1Penetrator = false;
        } else {
            info.penetratingProxy = box1->getProxy();
            info.penetratedProxy = box2->getProxy();
            box1Penetrator = true;
        }

        /* This is the test we are doing:
           - Creating the Planes of box1
           - Creating the Vertices and edges of box2
           - Now checking each vertex of box2 if it lies beetween all planes of
              box 1
           - Also checking the edges of box2 if they intersect with a plane of
             box 1 and if they intersect, check if the intersection point lies
             beetween all planes of box 1
           - If we found collisions, we can return
           - if not, we do the same as above, but using box2's planes and box1's
             edges and vertices
          -------------------------------------------------------------  */

        // Getting the Planes,vertices and center of the boxes:
        const Vector3* verts1 = box1->getVertices();
        const Plane* planes1 = box1->getPlanes(verts1);
        const Vector3 center1 = box1->getCenter(verts1);
        const Vector3* verts2 = box2->getVertices();
        const Plane* planes2 = box2->getPlanes(verts2);
        const Vector3 center2 = box1->getCenter(verts2);
        const Vector3 centerDiff12 = center2-center1;
        const Vector3 centerDiff21 = center1-center2;

        /* Checking the vertices:
         * Handle Special case: 4 vertices of one box are lieing 
         * directly on 4 vertices of the other box:
         * - save min and max of x,y and z of the vertices
         * - check if two of the axis have the same min-max-values
         * --> if so, change pen.depth to distance beetween these vertices
         * ----------------------------------*/
        real distances[6];
        real box1Min[3];
        real box1Max[3];
        int box1count = 0;
        real box2Min[3];
        real box2Max[3];
        int box2count = 0;

        for (int i = 0; i<8; ++i) {
            if (helpers.isInsideOrOnTheBox(planes1,verts2[i],distances)) {
                std::pair<real,int> dist = 
                    //helpers.calculatePenetrationDepth(planes1,distances,
                    //        6,centerDiff12);
                    helpers.calculatePenetrationDepth(distances, 6);

                info.normal =
                    box1Penetrator ? planes1[dist.second].getNormal() :
                    -(planes1[dist.second].getNormal());
                info.penetrationDepth = dist.first;
                info.collisionPoint = verts2[i];
                if (box1count == 0) {
                    box1Min[0] = box1Max[0] = verts2[i].getX();
                    box1Min[1] = box1Max[1] = verts2[i].getY();
                    box1Min[2] = box1Max[2] = verts2[i].getZ();
                    ++box1count;
                } else {
                    box1Min[0] = box1Min[0] > verts2[i].getX() ?
                        verts2[i].getX() : box1Min[0];
                    box1Min[1] = box1Min[1] > verts2[i].getY() ?
                        verts2[i].getY() : box1Min[1];
                    box1Min[2] = box1Min[2] > verts2[i].getZ() ?
                        verts2[i].getZ() : box1Min[2];
                    box1Max[0] = box1Max[0] < verts2[i].getX() ?
                        verts2[i].getX() : box1Max[0];
                    box1Max[1] = box1Max[1] < verts2[i].getY() ?
                        verts2[i].getY() : box1Max[1];
                    box1Max[2] = box1Max[2] < verts2[i].getZ() ?
                        verts2[i].getZ() : box1Max[2];
                    ++box1count;
                }
                resultsTempBox1[box1count-1] = info;
            } 
            else if (helpers.isInsideOrOnTheBox(planes2,verts1[i],distances)) {
                std::pair<real,int> dist = 
                    //helpers.calculatePenetrationDepth(planes2,distances,
                    //        6,centerDiff21);
                    helpers.calculatePenetrationDepth(distances, 6);

                info.normal =
                    box1Penetrator ? -planes2[dist.second].getNormal() :
                    (planes2[dist.second].getNormal());
                info.penetrationDepth = dist.first;
                info.collisionPoint = verts1[i];
                if (box2count == 0) {
                    box2Min[0] = box2Max[0] = verts1[i].getX();
                    box2Min[1] = box2Max[1] = verts1[i].getY();
                    box2Min[2] = box2Max[2] = verts1[i].getZ();
                    ++box2count;
                } else {
                    box2Min[0] = box2Min[0] > verts1[i].getX() ?
                        verts1[i].getX() : box2Min[0];
                    box2Min[1] = box2Min[1] > verts1[i].getY() ?
                        verts1[i].getY() : box2Min[1];
                    box2Min[2] = box2Min[2] > verts1[i].getZ() ?
                        verts1[i].getZ() : box2Min[2];
                    box2Max[0] = box2Max[0] < verts1[i].getX() ?
                        verts1[i].getX() : box2Max[0];
                    box2Max[1] = box2Max[1] < verts1[i].getY() ?
                        verts1[i].getY() : box2Max[1];
                    box2Max[2] = box2Max[2] < verts1[i].getZ() ?
                        verts1[i].getZ() : box2Max[2];
                    ++box2count;
                }
                resultsTempBox2[box2count-1] = info;
            } //END else  if (helpers.isInsideTheBox(planes2,verts1[i]))
        } // END for (int i = 0; i<8; ++i)

        // Handle Special case
        // -------------------
        bool adjustDistance = false;
        real newDistance = 0.0f;
        if ((box1count == box2count) && (box1count == 4)) {
            int equalCounter1,equalCounter2;
            equalCounter1 = equalCounter2 = 0;
            /*if (box1Min[0] == box2Min[0]) {equalCounter1+=1;};
            if (box1Min[1] == box2Min[1]) {equalCounter1+=2;};
            if (box1Min[2] == box2Min[2]) {equalCounter1+=4;};
            if (box1Max[0] == box2Max[0]) {equalCounter2+=1;};
            if (box1Max[1] == box2Max[1]) {equalCounter2+=2;};
            if (box1Max[2] == box2Max[2]) {equalCounter2+=4;};*/
            if (fabs(box1Min[0] - box2Min[0]) < 0.0001) {equalCounter1+=1;};
            if (fabs(box1Min[1] - box2Min[1]) < 0.0001) {equalCounter1+=2;};
            if (fabs(box1Min[2] - box2Min[2]) < 0.0001) {equalCounter1+=4;};
            if (fabs(box1Max[0] - box2Max[0]) < 0.0001) {equalCounter2+=1;};
            if (fabs(box1Max[1] - box2Max[1]) < 0.0001) {equalCounter2+=2;};
            if (fabs(box1Max[2] - box2Max[2]) < 0.0001) {equalCounter2+=4;};

            if (equalCounter1 == equalCounter2) {
                // Special Case?
                switch (equalCounter1) {
                    case 3:
                        newDistance = fabs(box1Max[2]-box2Max[2]);
                        adjustDistance = true;
                        break;
                    case 5:
                        newDistance = fabs(box1Max[1]-box2Max[1]);
                        adjustDistance = true;
                        break;
                    case 6:
                        newDistance = fabs(box1Max[0]-box2Max[0]);
                        adjustDistance = true;
                        break;
                    default:
                        /* Do nothing, no special case */
                        break;
                }
            } // END if (equalCounter1 == equalCounter2) {
        } // END if ((box1count == box2count) && (box1count == 4)) {

        // Do we need to adjust to the distance?
        if (adjustDistance) {
            for (int i = 0;i<box1count;++i) {
                (resultsTempBox1[i]).penetrationDepth = newDistance;
            }
            for (int i = 0;i<box2count;++i) {
                (resultsTempBox2[i]).penetrationDepth = newDistance;
            }
        } 

        // fill result-container:
        for (int i = 0;i<box1count;++i) {
            results.push_back(resultsTempBox1[i]);
        }
        for (int i = 0;i<box2count;++i) {
            results.push_back(resultsTempBox2[i]);
        }

        // Checking the edges:
        // ----------------------------------
        // getting them:
        std::pair<int,int> edges[12];
        box1->getEdges(edges);

        // Checking them:
        Vector3 v1_1,v1_2,v2_1,v2_2;
        Vector3 intersection;
        for (int i=0; i<12;++i) {
            // Check if the lines intersect and save the intersection-points:
            v1_1 = verts2[(edges[i]).first];
            v1_2 = verts2[(edges[i]).second];
            v2_1 = verts1[(edges[i]).first];
            v2_2 = verts1[(edges[i]).second];
            real distances[6];
            for (int j=0; j<6;++j) {
                if (planes1[j].intersectLineSegment(v1_1,v1_2, &intersection)) {

                    // Now check if one of these intersection points lies 
                    // beetween all 6^ opposing planes of the box:
                    // Here we create an array of reals, so that we can re-use
                    // the distances calc. in isInsideOrOnTheBox for our
                    // penetrationdepth

                    if (helpers.isInsideOrOnTheBox(planes1,intersection,distances)) {

                        // Bingo, we have a collision-point!
                        //std::cout <<" Bingo, we have a collision-point!" <<
                          //  std::endl;
                        std::pair<real,int> dist = 
                            helpers.calculatePenetrationDepth(distances, 6);

                        // Now we can set the collision info, but only if the point
                        // has a greater distance the maximum of distance:
                        info.normal =
                            box1Penetrator ? planes1[dist.second].getNormal()
                            : -(planes1[dist.second].getNormal());
                        info.penetrationDepth = dist.first;
                        info.collisionPoint = intersection;
                        results.push_back(info);
                    }
                } else if (planes2[j].intersectLineSegment(v2_1,v2_2, &intersection)) {
                    // Now check if one of these intersection points lies 
                    // beetween all 6^ opposing planes of the box:

                    if (helpers.isInsideOrOnTheBox(planes2,intersection,distances)) {

                        std::pair<real,int> dist = 
                            helpers.calculatePenetrationDepth(distances, 6);

                        // Now we can set the collision info, but only if the point
                        // has a greater distance the maximum of distance:
                        info.normal =
                            box1Penetrator ? -planes2[dist.second].getNormal()
                            : (planes2[dist.second].getNormal());
                        info.penetrationDepth = dist.first;
                        info.collisionPoint = intersection;
                        results.push_back(info);
                    }
                }
            } // END for (int j=0; j<6;++j) 
        } // END for (std::vector<std::pair<int,int> >::iterator iter =

        return results;
    }
}

/*
 * vim: et sw=4 ts=4
 */
