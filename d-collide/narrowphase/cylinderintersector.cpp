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
#include "narrowphase/cylinderintersector.h"
#include "narrowphase/intersectionhelpers.h"
#include "narrowphase/meshmeshintersector.h"

#include "dcollide-defines.h"
#include "debug.h"


#include <math.h>
#include <iostream>

namespace dcollide {

    /* All ctors and dtors
    --------------------------------------------------------------------------*/
    CylinderIntersector::CylinderIntersector() {}

    CylinderIntersector::~CylinderIntersector() {}

    /* Box-Cylinder collisions
    --------------------------------------------------------------------------*/

    /*!
     * \brief Calculates the collision between a box and a cylinder
     *
     * We have a collision if a point on the edge of the top and bottom circles
     * of the the \p cylinder is beetween two opposing planes of the \p box. The
     * amount of points for each circle is called ACCURACY_LEVEL and 
     * somehow defined at the top of this file.
     *
     * \param currentStateBox The current state of the box
     * \param box the box
     * \param currentStateCylinder The current state of the cylinder
     * \param cylinder the cylinder
     */
    std::list<CollisionInfo> CylinderIntersector::getIntersection(
            const Matrix& currentStateBox, Box* box, 
            const Matrix& currentStateCylinder, Cylinder* cylinder) const {

        DCOLLIDE_UNUSED(currentStateBox);

        // IntersectionHelper:
        IntersectionHelpers helpers;

        std::list<CollisionInfo> results;
        // As temporay container:
        std::list<CollisionInfo> resultsBottom;
        std::list<CollisionInfo> resultsTop;
        std::list<CollisionInfo> resultsHull;

        /* Prerequesites:
         * - we need an accuracy level
         * - we need the center of the two circles on top and bottom of the cyl
         * - project these centers to the edges of each circle
         * - we need the 6 planes of the box
         -----------------------------------------------------------------*/

        // Accuracy: How many points on the edge of the two circles on top and 
        // bottom of the cylinder do we want?
        // ------------------------------
        const int ACCURACY_LEVEL = helpers.getAccuracyLevel(cylinder->getRadius());

        // The points approximating the two circles:
        // ------------------------------
        Vector3* pointsTop = new Vector3[ACCURACY_LEVEL];
        Vector3* pointsBottom = new Vector3[ACCURACY_LEVEL];

        helpers.calculateCirclePoints(cylinder->getRadius(),
                cylinder->getHeight(),cylinder->getRadius(),
                currentStateCylinder,pointsBottom,pointsTop, ACCURACY_LEVEL);

        /*/ START DEBUG
        CollisionInfo info2;
        info2.penetratingProxy = box->getProxy();
        info2.penetratedProxy = cylinder->getProxy();
        info2.penetrationDepth = 2;
        info2.normal = Vector3(1,0,0);
        for (int i=0; i<ACCURACY_LEVEL;++i) {
            info2.collisionPoint = pointsBottom[i];
            results.push_back(info2);
            info2.collisionPoint = pointsTop[i];
            results.push_back(info2);
        }
        // END DEBUG*/

        // The planes of the box:
        // -----------------------
        const Plane* planes = box->getPlanes();

        /*/ START DEBUG
        CollisionInfo info2;
        info2.penetratingProxy = box->getProxy();
        info2.penetratedProxy = cylinder->getProxy();
        info2.penetrationDepth = 1;
        for (int i=0; i<6;++i) {
            info2.normal = planes[i].getNormal();
            info2.collisionPoint = Vector3(-100,25,200);
            results.push_back(info2);
        }
        // DEBUG*/


        /* Now we search for collision points:
         * We have a collision if a (previously calculated) point of the 
         * cylinder is beetween two opposing planes of the box:
         -----------------------------------------------------------------*/
        CollisionInfo info;
        bool cylPenetrator = false;
        // Check which proxy is penetrator:
        if (cylinder->getProxy()->getMoveFlags() & Proxy::MOVEFLAG_UNMOVED) {
            info.penetratingProxy = box->getProxy();
            info.penetratedProxy = cylinder->getProxy();
            cylPenetrator = false;
        } else {
            info.penetratedProxy = box->getProxy();
            info.penetratingProxy = cylinder->getProxy();
            cylPenetrator = true;
        }

        // Now do the real collision test:
        // ---------------------------------------------
        real maxDistanceOfBottomPoints = 0;
        real maxDistanceOfTopPoints = 0;
        real maxDistanceOfHullPoints = 0;
        real distances[6];
        for (int i=0; i<ACCURACY_LEVEL;++i) {

            /* check if one point lies beetween all planes
             *  --> collisionpoint!
             * a collisionpoint always lies beetween all planes. As a
             * consequence we only use the plane with the minimal distance to
             * that point
              -------------------------------------------------*/

            // The Top Points:
            // ------------------------------------------------
            // Does the point lie inside the shape?
            if (helpers.isInsideOrOnTheBox(planes,pointsTop[i],distances)) {

                std::pair<real,int> dist = helpers.calculatePenetrationDepth(
                        distances,6);

                // Now we can set the collision info, but only if the point
                // has a greater distance the maximum of distance:
                if (maxDistanceOfTopPoints < dist.first) {
                    info.normal =
                        cylPenetrator ? -planes[dist.second].getNormal() :
                        (planes[dist.second].getNormal());
                    info.penetrationDepth = dist.first;
                    info.collisionPoint = pointsTop[i];
                    resultsTop.push_back(info);
                    maxDistanceOfTopPoints = dist.first;
                }
            } // END if

            // The Bottom Points:
            // ------------------------------------------------
            // Does the point lie inside the shape?
            if (helpers.isInsideOrOnTheBox(planes,pointsBottom[i],distances)) {

                std::pair<real,int> dist = helpers.calculatePenetrationDepth(
                        distances,6);

                // Now we can set the collision info, but only if the point
                // has a greater distance the maximum of distance:
                if (maxDistanceOfBottomPoints < dist.first) {
                    info.normal =
                        cylPenetrator ? -planes[dist.second].getNormal() :
                        (planes[dist.second].getNormal());
                    info.penetrationDepth = dist.first;
                    info.collisionPoint = pointsBottom[i];
                    resultsBottom.push_back(info);
                    maxDistanceOfBottomPoints = dist.first;
                } 
            } // END if 

            /* Special case handling of boxes penetrating the cylinder at the 
             * hull beetween top and bottom:
             *
             * short description: 
             *  - create lines beetween the top and bottom points
             *  - intersect these lines with the planes
             *  - if the intersection point lies IN the box 
             *   --> we have a collisionpoint,normal = normal of the plane, 
             *      depth = distance beetween plane and point
             * ----------------------------------------------------------------
             */

            // Check if the lines intersect and save the intersection-points:
            Vector3 intersection;
            for (int j=0; j<6;++j) {
                if (planes[j].intersectLineSegment(pointsTop[i],pointsBottom[i],
                        &intersection)) {

                    // Now we test only the two plane-pairs where the 
                    // intersection does NOT lie on:
                    // Check the planes, here we must check behind and front because
                    // at compile time we don't know which planes we must check
                    if (helpers.isInsideOrOnTheBox(planes,intersection,distances)
                       ) {

                        // Bingo, we have a collision-point!
                        std::pair<real,int> dist = 
                            helpers.calculatePenetrationDepth(distances,6);

                        // Now we can set the collision info, but only if the point
                        // has a greater distance the maximum of distance:
                        if (maxDistanceOfHullPoints < dist.first) {
                            info.normal =
                                cylPenetrator ? -planes[dist.second].getNormal()
                                : (planes[dist.second].getNormal());
                            info.penetrationDepth = dist.first;
                            info.collisionPoint = intersection;
                            resultsHull.push_back(info);
                            maxDistanceOfHullPoints = dist.first;
                        } 
                    }
                }//  END if (planes[j].intersectLineSegment(pointsTop[i],
            } // END for (int j=0; j<6;++j) 
        } // END for (int i=0; i<ACCURACY_LEVEL;++i)


        // Now we remove all collision points from the list that are smaller
        // then the final maxDistance:
        // -----------------------------------------------------------------
        for (std::list<CollisionInfo>::iterator iter = resultsBottom.begin();
                iter != resultsBottom.end();++iter) {
            if ((*iter).penetrationDepth >= maxDistanceOfBottomPoints - 1.0E-8f) {
                results.push_back(*iter);
            }
        }
        for (std::list<CollisionInfo>::iterator iter = resultsTop.begin();
                iter != resultsTop.end();++iter) {
            if ((*iter).penetrationDepth >= maxDistanceOfTopPoints - 1.0E-8f) {
                results.push_back(*iter);
            }
        }
        for (std::list<CollisionInfo>::iterator iter = resultsHull.begin();
                iter != resultsHull.end();++iter) {
            if ((*iter).penetrationDepth >= maxDistanceOfHullPoints - 1.0E-8f) {
                results.push_back(*iter);
            }
        }

        // Cleanup:
        delete[] pointsBottom;
        delete[] pointsTop;

        return results;
    }


    /* Wedge-Cylinder collisions
    --------------------------------------------------------------------------*/

    /*!
     * \brief Calculates the collision between a wedge and a cylinder
     *
     * We have a collision if a point on the edge of the top and bottom circles
     * of the the \p cylinder is beetween the two / three opposing planes of 
     * the \p wedge. The amount of points for each circle is called 
     * ACCURACY_LEVEL and somehow defined at the top of this file.
     *
     * \param currentStateWedge The current state of the wedge
     * \param wedge the wedge
     * \param currentStateCylinder The current state of the cylinder
     * \param cylinder the cylinder
     */
    std::list<CollisionInfo> CylinderIntersector::getIntersection(
            const Matrix& currentStateWedge, Wedge* wedge, 
            const Matrix& currentStateCylinder, Cylinder* cylinder) const {

        DCOLLIDE_UNUSED(currentStateWedge);

        // IntersectionHelper:
        IntersectionHelpers helpers;

        std::list<CollisionInfo> results;
        // As temporay container:
        std::list<CollisionInfo> resultsBottom;
        std::list<CollisionInfo> resultsTop;
        std::list<CollisionInfo> resultsHull;

        /* Prerequesites:
         * - we need an accuracy level
         * - we need the center of the two circles on top and bottom of the cyl
         * - project these centers to the edges of each circle
         * - we need the 6 planes of the box
         -----------------------------------------------------------------*/

        // Accuracy: How many points on the edge of the two circles on top and 
        // bottom of the cylinder do we want?
        // ------------------------------
        const int ACCURACY_LEVEL = helpers.getAccuracyLevel(cylinder->getRadius());

        // The points approximating the two circles:
        // ------------------------------
        Vector3* pointsTop = new Vector3[ACCURACY_LEVEL];
        Vector3* pointsBottom = new Vector3[ACCURACY_LEVEL];

        helpers.calculateCirclePoints(cylinder->getRadius(),
                cylinder->getHeight(),cylinder->getRadius(),
                currentStateCylinder,pointsBottom,pointsTop, ACCURACY_LEVEL);

        /*/ START DEBUG
        CollisionInfo info2;
        info2.penetratingProxy = wedge->getProxy();
        info2.penetratedProxy = cylinder->getProxy();
        info2.penetrationDepth = 2;
        info2.normal = Vector3(1,0,0);
        for (int i=0; i<ACCURACY_LEVEL;++i) {
            info2.collisionPoint = pointsBottom[i];
            results.push_back(info2);
            info2.collisionPoint = pointsTop[i];
            results.push_back(info2);
        }
        // END DEBUG*/

        // The planes of the wedge:
        // -----------------------
        const Plane* planes = wedge->getPlanes();

        /*/ START DEBUG
        const Vector3* bvs = wedge->getVertices();
        for (int i=0; i<6;++i) {
            info2.collisionPoint = bvs[i];
            results.push_back(info2);
        }
        // END DEBUG*/

        /* Now we search for collision points:
         * We have a collision if a (previously calculated) point of the 
         * cylinder is beetween all planes of the wedge:
         -----------------------------------------------------------------*/
        CollisionInfo info;
        bool cylPenetrator = false;
        // Check which proxy is penetrator:
        if (cylinder->getProxy()->getMoveFlags() & Proxy::MOVEFLAG_UNMOVED) {
            info.penetratingProxy = wedge->getProxy();
            info.penetratedProxy = cylinder->getProxy();
            cylPenetrator = false;
        } else {
            info.penetratedProxy = wedge->getProxy();
            info.penetratingProxy = cylinder->getProxy();
            cylPenetrator = true;
        }

        // Now do the real collision test:
        // ---------------------------------------------
        real maxDistanceOfBottomPoints = 0;
        real maxDistanceOfTopPoints = 0;
        real maxDistanceOfHullPoints = 0;
        real distances[5];
        for (int i=0; i<ACCURACY_LEVEL;++i) {

            /* check if one point lies beetween all planes
             *  --> collisionpoint!
             * a point always lies beetween all opposing planes. As a
             * consequence we only use the plane with the minimal distance to
             * that point
              -------------------------------------------------*/

            // The Top Points:
            // -----------------------------------------
            // Is the point inside the shape?
            if (helpers.isInsideOrOnTheWedge(planes,pointsTop[i],distances)) {

                std::pair<real,int> dist = helpers.calculatePenetrationDepth(
                        distances,5);

                // Now we can set the collision info, but only if the point
                // has a greater distance the maximum of distance:
                if (maxDistanceOfTopPoints < dist.first) {
                    info.normal =
                        cylPenetrator ? -(planes[dist.second].getNormal()) :
                        (planes[dist.second].getNormal());
                    info.penetrationDepth = dist.first;
                    info.collisionPoint = pointsTop[i];
                    resultsTop.push_back(info);
                    maxDistanceOfTopPoints = dist.first;
                }
            } // END if

            // The Bottom Points:
            // -----------------------------------------
            // Is the point inside the shape?
            if (helpers.isInsideOrOnTheWedge(planes,pointsBottom[i],distances)) {

                std::pair<real,int> dist = helpers.calculatePenetrationDepth(
                        distances,5);

                // Now we can set the collision info, but only if the point
                // has a greater distance the maximum of distance:
                if (maxDistanceOfBottomPoints < dist.first) {
                    info.normal =
                        cylPenetrator ? -(planes[dist.second].getNormal()) :
                        (planes[dist.second].getNormal());
                    info.penetrationDepth = dist.first;
                    info.collisionPoint = pointsBottom[i];
                    resultsBottom.push_back(info);
                    maxDistanceOfBottomPoints = dist.first;
                } 
            } // END if 

            /* Special case handling of wedge penetrating the cylinder at the 
             * hull beetween top and bottom:
             *
             * short description: 
             *  - create lines beetween the top and bottom points
             *  - intersect these lines with the planes
             *  - if the intersection point lies IN the wedge 
             *   --> we have a collisionpoint,normal = normal of the plane, 
             *         depth = distance beetween plane and point
             * ---------------------------------------------------------------
             */

            // Check if the lines intersect and save the intersection-points:
            Vector3 intersection;
            for (int j=0; j<5;++j) {
                if (planes[j].intersectLineSegment(pointsTop[i],pointsBottom[i],
                        &intersection)) {
                    // Now check if the intersection point lies beetween all 5
                    // opposing planes of the wedge:
                    if (helpers.isInsideOrOnTheWedge(
                                planes,intersection,distances)) {

                        // Bingo, we have a collision-point!
                        std::pair<real,int> dist = helpers.calculatePenetrationDepth(
                            distances,5);

                        // Now we can set the collision info, but only if the point
                        // has a greater distance the maximum of distance:
                        if (maxDistanceOfHullPoints < dist.first) {
                            info.normal =
                                cylPenetrator ?
                                -(planes[dist.second].getNormal()) :
                                (planes[dist.second].getNormal());
                            info.penetrationDepth = dist.first;
                            info.collisionPoint = intersection;
                            resultsHull.push_back(info);
                            maxDistanceOfHullPoints = dist.first;
                        } 
                    }
                } // END if (planes[j].intersectLineSegment(pointsTop[i],
            } // END for (int j=0; j<5;++j) {
        } // END for (int i=0; i<ACCURACY_LEVEL;++i)


        // Now we remove all collision points from the list that are smaller
        // then the final maxDistance:
        // -----------------------------------------------------------------
        for (std::list<CollisionInfo>::iterator iter = resultsBottom.begin();
                iter != resultsBottom.end();++iter) {
            if ((*iter).penetrationDepth >= maxDistanceOfBottomPoints - 1.0E-8f) {
                results.push_back(*iter);
            }
        }
        for (std::list<CollisionInfo>::iterator iter = resultsTop.begin();
                iter != resultsTop.end();++iter) {
            if ((*iter).penetrationDepth >= maxDistanceOfTopPoints - 1.0E-8f) {
                results.push_back(*iter);
            }
        }
        for (std::list<CollisionInfo>::iterator iter = resultsHull.begin();
                iter != resultsHull.end();++iter) {
            if ((*iter).penetrationDepth >= maxDistanceOfHullPoints - 1.0E-8f) {
                results.push_back(*iter);
            }
        }

        // Cleanup:
        // -----------------------------------------------------------------
        delete[] pointsBottom;
        delete[] pointsTop;

        return results;
    }

    /* Cylinder-Cylinder collisions
    --------------------------------------------------------------------------*/

    /*!
     * \brief Calculates the collision between two cylinders
     *
     * atm this is done by performing a mesh-mesh-test.
     */
    std::list<CollisionInfo> CylinderIntersector::getIntersection(
            const Matrix& currentStateCylinder1, Cylinder* cylinder1, 
            const Matrix& currentStateCylinder2, Cylinder* cylinder2) const {

        DCOLLIDE_UNUSED(currentStateCylinder1);
        DCOLLIDE_UNUSED(currentStateCylinder2);

        std::list<CollisionInfo> results;

        MeshMeshIntersector meshIntersector;
        results = meshIntersector.getIntersection(cylinder1->getMesh() ,
                cylinder2->getMesh());

        return results;
    }

}

/*
 * vim: et sw=4 ts=4
 */
