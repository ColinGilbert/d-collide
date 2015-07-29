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
#include "narrowphase/intersectionhelpers.h"

#include "dcollide-defines.h"
#include "debug.h"


#include <math.h>
#include <iostream>

/* This defines how long an intervall of the radius is:
 * example: if a circle has a radius of 2, and we have an ACCURACY_ADJUSTMENT of
 * 0.25, we can divide the radius in 2/0.25 = 8 parts. This means the circle is
 * approximated by 32 (8 for each quarter) points.
 * Note that the amount of points is always increased to a power of 2
 */
#define CIRCLE_ACCURACY_ADJUSTMENT 0.25

namespace dcollide {

    IntersectionHelpers::IntersectionHelpers() {
    }
    IntersectionHelpers::~IntersectionHelpers() {
    }

    /*!
     * \brief Calcualtes accuracy-level of the circle-approximation
     * Must be a at least 4 and a power of 2!
     * Determine a "good" accuracy level, at least 4:
     * \param radius the radius of the circle we want to approximate
     * \returns accuracy-level
     */
    int IntersectionHelpers::getAccuracyLevel(real radius) const {
        int al = 4;
        if ((radius/CIRCLE_ACCURACY_ADJUSTMENT)*4 > 4) {
           al = (int)(radius/CIRCLE_ACCURACY_ADJUSTMENT)*4;

            // adjust to next power:
            int nv = 2;
            while (nv < al) {
                nv *= 2;
            }
            al = nv;
        }
        return al;
    }

    /*!
     * \brief Calculates circle points
     *
     * If radius2 = 0, we assume no top circle, which is the case for e.g. shape
     * type cone. Currently only radius1=radius2 or radius2 = 0 is supported,
     * any other value for radius 2 is ignored atm.
     * 
     * \param radius1 the radius of the bottom circle we want to approximate
     * \param distance the distance beetween the twao circles we want 
     * \param radius2 the radius of the top circle we want to approximate
     * \param state The state of the shape the circles belong to
     * \param bottomPoints contains the returned bottom points
     * \param topPoints contains the returned top points
     * \returns accuracy-level
     */
    void IntersectionHelpers::calculateCirclePoints(real radius1, real distance,
            real radius2,const Matrix& state,Vector3* pointsBottom,
            Vector3* pointsTop, const int accuracy_level) const {

        // Get the (unrotated!!) center of the circle on top:
        Vector3 centerTop =
            state.getPosition();
        centerTop.setZ(centerTop.getZ()+distance);
        // the center of the bottom circle == position!
        Vector3 centerBottom = state.getPosition();

        // Radius1:
        real cr = radius1;

        /* Get the points on the edges of the circles by projecting the centers
         * to the circle-edges:
         *
         * The ordering is like this:
         *
         *                    y
         *   _..--0--.._      ^
         *  3_    c    _1     |
         *    ''--2--''       |
         *                    -------> x
         ----------------------------------------*/
        real bx = centerBottom.getX();
        real by = centerBottom.getY();
        real bz = centerBottom.getZ();
        // We must have at least 4 points for each circle, calculate them:
        // In these vectors we save the vectors we have to use to calculate the
        // bisecting lines in each iteration of the bisecting-lines-creator:
        std::vector<std::pair<int,int> > pairs;
        std::vector<std::pair<int,int> > pairs_new;
        std::pair<int,int> pair;
        pointsBottom[0] = Vector3(bx,by+cr,bz);
        pair.first = 0; pair.second= 1;
        pairs.push_back(pair);
        pointsBottom[1] = Vector3(bx+cr,by,bz); 
        pair.first = 1; pair.second = 2;
        pairs.push_back(pair);
        pointsBottom[2] = Vector3(bx,by-cr,bz); 
        pair.first = 2; pair.second = 3;
        pairs.push_back(pair);
        pointsBottom[3] = Vector3(bx-cr,by,bz); 
        pair.first = 3; pair.second = 0;
        pairs.push_back(pair);

        // now calc. the bottom-offset-vectors, we need them to rotate the 
        // points around the center of the cylinder, not around the global 
        // coordinate system.
        Vector3* offsetBottom = new Vector3[accuracy_level];
        Vector3 offsetBottomTop = centerTop-centerBottom;
        for (int i=0;i<4;++i) {
            offsetBottom[i] = pointsBottom[i]-centerBottom;
        }

        // create the bisecting lines:
        int c = 4;
        std::pair<int,int> p;
        std::pair<int,int> pn;
        while (c != accuracy_level) {
            for (int i = 0; i<(c*2-c);++i) {
                p = pairs[i];
                offsetBottom[i+c] = offsetBottom[p.first]+offsetBottom[p.second];
                offsetBottom[i+c].normalize();
                offsetBottom[i+c].scale(cr);
                pn.first = p.first; pn.second = i+c;
                pairs_new.push_back(pn);
                pn.first = i+c; pn.second = p.second;
                pairs_new.push_back(pn);
            }
            pairs = pairs_new;
            pairs_new.clear();
            c *= 2;
        }

        /* Now rotate the offset-vectors and after that add each one to the
         * centerBottom point.
         * The obtained by adding the rotated offset beetween
         * centerTop and centerBottom to the centerBottom and then add the
         * previously calc. rotatet offset
         * ----------------------------------------*/
        Matrix rotationMatrix = state.getRotationMatrix();
        // Forst rotate the offset vector from bottom to top:
        Vector3 rotOffset;
        rotationMatrix.transform(&rotOffset,offsetBottomTop);
        offsetBottomTop = rotOffset;

        // Setting the top point if no top circle (cone):
        if (radius2 == 0) {
            pointsTop[0] = centerBottom+offsetBottomTop;
        }

        for (int i=0; i<accuracy_level;++i) {

            // Bottom:
            Vector3 circleOffsetPoint;
            rotationMatrix.transform(&circleOffsetPoint,offsetBottom[i]);
            pointsBottom[i] = centerBottom+circleOffsetPoint;

            // Top, only if radius2 is also != 0!
            if (radius2 != 0) {
                pointsTop[i] = centerBottom+offsetBottomTop;
                pointsTop[i] = pointsTop[i]+circleOffsetPoint;
            }

        }

        // Cleanup
        delete[] offsetBottom;
    }

    /*!
     * \param planes A box described by its planes
     * \param point The point to check
     * \returns true if the given \p point lies inside the given \p planes
     */
    bool IntersectionHelpers::isInsideTheBox(const Plane* planes,
            const Vector3& point) const {

        /* we must check all planes to determine wether we have a collision 
         * or not, we have a collision only if the point is beetween ALL 
         * planes!
         *
         * All normals are pointing OUTSIDE! (inFrontOf)
         */
        return (
                //planes[0].isBehindPlane(point) &&
                planes[0].isInFrontOfPlane(point) &&
                //planes[1].isBehindPlane(point) &&
                planes[1].isInFrontOfPlane(point) &&
                //planes[2].isBehindPlane(point) && 
                planes[2].isInFrontOfPlane(point) &&
                //planes[3].isBehindPlane(point) &&
                planes[3].isInFrontOfPlane(point) &&
                //planes[4].isBehindPlane(point) &&
                planes[4].isInFrontOfPlane(point) &&
                //planes[5].isBehindPlane(point));
                planes[5].isInFrontOfPlane(point));
    }

    /*!
     * \param planes A box described by its planes
     * \param point The point to check
     * \param distances Here we can return the distances used by this calc. for
     * later re-use
     * \returns true if the given \p point lies inside or on the given \p planes
     */
    bool IntersectionHelpers::isInsideOrOnTheBox(const Plane* planes,
            const Vector3& point, real (&distances)[6]) const {

        // First getting the distances:
        for (int i=0;i<6;++i) {
            distances[i] = planes[i].calculateDistance(point);
        }

        /* we must check all planes to determine wether we have a collision 
         * or not, we have a collision only if the point is beetween ALL 
         * planes!
         *
         * All normals are pointing OUTSIDE! (inFrontOf)
         */
        return (
                (distances[0] >= 0.0f) &&
                (distances[1] >= 0.0f) &&
                (distances[2] >= 0.0f) &&
                (distances[3] >= 0.0f) &&
                (distances[4] >= 0.0f) &&
                (distances[5] >= 0.0f)); 
    }

    /*!
     * \param planes A wedge described by its planes
     * \param point The point to check
     * \returns true if the given \p point lies inside the given \p planes
     */
    bool IntersectionHelpers::isInsideTheWedge(const Plane* planes,
            const Vector3& point) const {

        /* we must check all planes to determine wether we have a 
           collision or not, we have a collision only if the point is 
           beetween ALL planes!
         * all normals point outside!
         * */
        return (
            planes[0].isBehindPlane(point) &&
            //planes[0].isInFrontOfPlane(point) &&
            planes[1].isBehindPlane(point) &&
            //planes[1].isInFrontOfPlane(point) &&
            planes[2].isBehindPlane(point) &&
            //planes[2].isInFrontOfPlane(point) &&
            planes[3].isBehindPlane(point) &&
            //planes[3].isInFrontOfPlane(point) &&
            planes[4].isBehindPlane(point));
            //planes[4].isInFrontOfPlane(point));
    }

    /*!
     * \param planes A wedge described by its planes
     * \param point The point to check
     * \param distance with this param we can return the created distances for
     * later re-use
     * \returns true if the given \p point lies inside the given \p planes
     */
    bool IntersectionHelpers::isInsideOrOnTheWedge(const Plane* planes,
        const Vector3& point, real (&distances)[5]) const {

        // First getting the distances:
        for (int i=0;i<5;++i) {
            distances[i] = planes[i].calculateDistance(point);
        }

        /* we must check all planes to determine wether we have a collision 
         * or not, we have a collision only if the point is beetween ALL 
         * planes!
         *
         * All normals are pointing OUTSIDE! (inFrontOf)
         */
        return (
                (distances[0] <= 0.0f) &&
                (distances[1] <= 0.0f) &&
                (distances[2] <= 0.0f) &&
                (distances[3] <= 0.0f) &&
                (distances[4] <= 0.0f));
    }

    /*!
     * \brief calculate the penetrationdepth of a collisionpoint
     *
     * the pair.first is the penetrationdepth, the pair.second tells you which
     * one of the planes was finally used to calculate the depth
     *
     * \param planes A shape described by its planes
     * \param sizeOfPlanes The size of the given array
     * \param point The intersection point
     * \returns a std::pair<real,int>
     */
    std::pair<real,int> IntersectionHelpers::calculatePenetrationDepth(
            const Plane* planes, int sizeOfPlanes, 
            const Vector3& intersection) const {
        real distance =
            fabs(planes[0].calculateDistance(intersection));
        int indexOfPlane = 0;
        for (int j=1; j<sizeOfPlanes;++j) {
            real newDistance =
                fabs(planes[j].calculateDistance(intersection));
            if (distance > newDistance) {
                indexOfPlane = j;
                distance = newDistance;
            }
        }

        return std::pair<real,int>(distance,indexOfPlane);
    }

    /*!
     * \brief calculate the penetrationdepth of a collisionpoint
     *
     * the pair.first is the penetrationdepth, the pair.second tells you which
     * one of the planes was finally used to calculate the depth
     *
     * \param distances previously calc. distances of a point to each plane
     * \param sizeOfDistances The size of the given array
     * \returns a std::pair<real,int>
     */
    std::pair<real,int> IntersectionHelpers::calculatePenetrationDepth(
            const real* distances,int sizeOfDistances) const {

        real distance = fabs(distances[0]);
        /*real lastValueNeqZero = distance;
        // to count how often we had a distance = 0. If this occurs exactly 2
        // times, we have a vertex travelling down an edge!
        // times --> Don't use 0, but the next positive value.
        int zeroCounter = 0;
        if (distance == 0) {
            ++zeroCounter;
        }
        */
        int indexOfPlane = 0;
        //std::cout << "D0: " << distance << ";";
        for (int j=1; j<sizeOfDistances;++j) {
            real newDistance = fabs(distances[j]);
            /*std::cout << "D" << j << ": "<< newDistance << ";";
            if (newDistance == 0) {
                ++zeroCounter;
            } else {
                if (newDistance < lastValueNeqZero || lastValueNeqZero == 0) {
                    lastValueNeqZero = newDistance;
                }
            }*/
            if (distance > newDistance) {
                indexOfPlane = j;
                distance = newDistance;
            }
        }
        /*std::cout << "\n\tDistance == 0: " << zeroCounter << " times; distance = "
            << distance << " lastValueNeqZero = " << lastValueNeqZero << std::endl;
        if (zeroCounter >= 1 && zeroCounter < 3) {
            std::cout << "\t\t  ADJUSTED distance = " << lastValueNeqZero << std::endl;
            distance = lastValueNeqZero;
        }
        */
        return std::pair<real,int>(distance,indexOfPlane);
    }

    /*!
     * \brief calculate the penetrationdepth of a collisionpoint of boxbox
     *
     * the pair.first is the penetrationdepth, the pair.second tells you which
     * one of the planes was finally used to calculate the depth
     *
     * \param planes A Box described by its planes
     * \param sizeOfPlanes The size of the given array
     * \param centerDiff The difference of the two centers of the boxes
     * \returns a std::pair<real,int>
     */
    std::pair<real,int> IntersectionHelpers::calculatePenetrationDepth(
            const Plane* planes, const real* distances,int sizeOfPlanes, 
            const Vector3& centerDiff) const {

        real cmpdistance = centerDiff.dotProduct(planes[0].getNormal());
        int indexOfPlane = 0;
        for (int j=1; j<sizeOfPlanes;++j) {
            real newDistance =
                centerDiff.dotProduct(planes[j].getNormal());
            if (cmpdistance > newDistance) {
                indexOfPlane = j;
                cmpdistance = newDistance;
            }
        }

        return std::pair<real,int>(distances[indexOfPlane],indexOfPlane);
    }
}

/*
 * vim: et sw=4 ts=4
 */
