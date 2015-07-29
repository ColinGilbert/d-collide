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
#include "math/plane.h"
#include "real.h"
#include "shapes/shapes.h"
#include "proxy.h"
#include "collisioninfo.h"
#include "narrowphase/wedgeintersector.h"
#include "narrowphase/intersectionhelpers.h"

#include "dcollide-defines.h"

#include <math.h>
#include <iostream>

namespace dcollide {

    WedgeIntersector::WedgeIntersector() {}

    WedgeIntersector::~WedgeIntersector() {}

    /* Wedge-Wedge collisions
    --------------------------------------------------------------------------*/
    /*!
     * \brief Calculates Collisions beetween two wedges
     * 
     * this is done by checking planes vs. vertices and edges
     *
     * \param wedge1 The first wedge
     * \param wedge2 The second wedge
     */
    std::list<CollisionInfo> WedgeIntersector::getIntersection(
            Wedge* wedge1, Wedge* wedge2) const {
        // IntersectionHelper:
        IntersectionHelpers helpers;

        // The Container for the returned Collisions:
        std::list<CollisionInfo> results;

        /* Preparing CollisionInfo::
         -----------------------------------------------------------------*/
        CollisionInfo info;
        bool wedge1Penetrator = false;
        // Check which proxy is penetrator:
        if (wedge1->getProxy()->getMoveFlags() & Proxy::MOVEFLAG_UNMOVED) {
            info.penetratingProxy = wedge2->getProxy();
            info.penetratedProxy = wedge1->getProxy();
            wedge1Penetrator = false;
        } else {
            info.penetratingProxy = wedge1->getProxy();
            info.penetratedProxy = wedge2->getProxy();
            wedge1Penetrator = true;
        }

        /*/ START DEBUG
        CollisionInfo info2;
        info2.penetratingProxy = wedge1->getProxy();
        info2.penetratedProxy = wedge2->getProxy();
        info2.penetrationDepth = 2;
        info2.normal = Vector3(1,0,0);
        // END DEBUG*/

        /* This is the test we are doing:
           - Creating the Planes of wedge1 and wedge2
           - Creating the Vertices and edges of wedge1 and  wedge2
           - Now checking each vertex of wegde1 if it lies beetween all 
             planes of wedge2, and vice versa
           - Also checking the edges of wedge1 if they intersect with a plane of
             wedge2 and if they intersect, check if the intersection point lies
             beetween all planes of wedge2, and vice versa
          -------------------------------------------------------------  */

        // Getting the Planes and vertices of the wedges:
        const Vector3* verts1 = wedge1->getVertices();
        const Plane* planes1 = wedge1->getPlanes(verts1);
        const Vector3* verts2 = wedge2->getVertices();
        const Plane* planes2 = wedge2->getPlanes(verts2);

        /*/ START DEBUG
        for (int i=0; i<6;++i) {
            info2.collisionPoint = verts1[i];
            results.push_back(info2);
            info2.collisionPoint = verts2[i];
            results.push_back(info2);
        }
        // END DEBUG*/


        // Checking the vertices of each wedge:
        // ----------------------------------
        real distancesW[5];
        for (int i = 0; i<6; ++i) {
            if (helpers.isInsideOrOnTheWedge(planes1,verts2[i],distancesW)) {
                std::pair<real,int> dist = 
                    helpers.calculatePenetrationDepth(distancesW, 5);

                info.normal =
                    wedge1Penetrator ? planes1[dist.second].getNormal() :
                    -(planes1[dist.second].getNormal());
                info.penetrationDepth = dist.first;
                info.collisionPoint = verts2[i];
                results.push_back(info);
            }
            else if (helpers.isInsideOrOnTheWedge(planes2,verts1[i],distancesW)) {
                std::pair<real,int> dist = 
                    helpers.calculatePenetrationDepth(distancesW, 5);

                info.normal =
                    wedge1Penetrator ? planes2[dist.second].getNormal() :
                    -(planes2[dist.second].getNormal());
                info.penetrationDepth = dist.first;
                info.collisionPoint = verts1[i];
                results.push_back(info);
            } //END if (helpers.isInsideTheWedge(planes1,verts2[i]))
        } // END for (int i = 0; i<6; ++i)

        // Checking the edges:
        // ----------------------------------
        // getting them:
        std::pair<int,int> edgesW[9];
        wedge1->getEdges(edgesW);

        // Checking them:
        Vector3 v1_1,v1_2,v2_1,v2_2;
        Vector3 intersection;
        for (int i=0; i<9;++i) {
            // Check if the lines intersect and save the intersection-points:
            v1_1 = verts1[(edgesW[i]).first];
            v1_2 = verts1[(edgesW[i]).second];
            v2_1 = verts2[(edgesW[i]).first];
            v2_2 = verts2[(edgesW[i]).second];
            for (int j=0; j<5;++j) {
                if (planes2[j].intersectLineSegment(v1_1,v1_2, &intersection)) {

                    // Now check if one of these intersection points lies 
                    // beetween all 6^ opposing planes of the Wedge:
                    // Here we create an array of reals, so that we can re-use
                    // the distances calc. in isInsideOrOnTheWedge for our
                    // penetrationdepth

                    if (helpers.isInsideOrOnTheWedge(planes2,
                            intersection,distancesW)) {

                        // Bingo, we have a collision-point!
                        //std::cout <<" Bingo, we have a collision-point!" <<
                          //  std::endl;
                        std::pair<real,int> dist = 
                            helpers.calculatePenetrationDepth(distancesW, 5);

                        // Now we can set the collision info, but only if the point
                        // has a greater distance the maximum of distance:
                        info.normal =
                            wedge1Penetrator ? planes2[dist.second].getNormal()
                            : -(planes2[dist.second].getNormal());
                        info.penetrationDepth = dist.first;
                        info.collisionPoint = intersection;
                        results.push_back(info);
                    }
                } 
                if (planes1[j].intersectLineSegment(v2_1,v2_2, &intersection)) {

                    // Now check if one of these intersection points lies 
                    // beetween all 6^ opposing planes of the Wedge:
                    // Here we create an array of reals, so that we can re-use
                    // the distances calc. in isInsideOrOnTheWedge for our
                    // penetrationdepth
                    if (helpers.isInsideOrOnTheWedge(planes1,
                            intersection,distancesW)) {

                        // Bingo, we have a collision-point!
                        //std::cout <<" Bingo, we have a collision-point!" <<
                          //  std::endl;
                        std::pair<real,int> dist = 
                            helpers.calculatePenetrationDepth(distancesW, 5);

                        // Now we can set the collision info, but only if the point
                        // has a greater distance the maximum of distance:
                        info.normal =
                            wedge1Penetrator ? planes1[dist.second].getNormal()
                            : -(planes1[dist.second].getNormal());
                        info.penetrationDepth = dist.first;
                        info.collisionPoint = intersection;
                        results.push_back(info);
                    }
                } 
            } // END for (int j=0; j<5;++j) 
        } // END for (int i=0; i<9;++i) 

        return results;
    }


    /* Box-Wedge collisions
    --------------------------------------------------------------------------*/
    /*!
     * \brief Calculates Collisions beetween a box and a wedge
     * 
     * this is done by checking planes vs. vertices and edges
     *
     * \param box The box
     * \param wedge The wedge
     */
    std::list<CollisionInfo> WedgeIntersector::getIntersection(
            Box* box, Wedge* wedge) const {
        // IntersectionHelper:
        IntersectionHelpers helpers;

        // The Container for the returned Collisions:
        std::list<CollisionInfo> results;

        /* Preparing CollisionInfo::
         -----------------------------------------------------------------*/
        CollisionInfo info;
        bool boxPenetrator = false;
        // Check which proxy is penetrator:
        if (box->getProxy()->getMoveFlags() & Proxy::MOVEFLAG_UNMOVED) {
            info.penetratingProxy = wedge->getProxy();
            info.penetratedProxy = box->getProxy();
            boxPenetrator = false;
        } else {
            info.penetratingProxy = box->getProxy();
            info.penetratedProxy = wedge->getProxy();
            boxPenetrator = true;
        }

        /*/ START DEBUG
        CollisionInfo info2;
        info2.penetratingProxy = box->getProxy();
        info2.penetratedProxy = wedge->getProxy();
        info2.penetrationDepth = 2;
        info2.normal = Vector3(1,0,0);
        // END DEBUG*/

        /* This is the test we are doing:
           - Creating the Planes,vertices and edges of the box and the wedge
           - Now checking each vertex of the box if it lies beetween all planes
             of the wegde, and vice versa
           - Also checking the edges of the box if they intersect with a plane 
             of the wedge and if they intersect, check if the intersection 
             point lies beetween all planes of the wedge, and vice versa
          -------------------------------------------------------------  */

        // Getting the Planes and vertices:
        const Vector3* vertsBox = box->getVertices();
        const Plane* planesBox = box->getPlanes(vertsBox);
        const Vector3* vertsWedge = wedge->getVertices();
        const Plane* planesWedge = wedge->getPlanes(vertsWedge);

        /*/ START DEBUG
        for (int i=0; i<6;++i) {
            info2.collisionPoint = vertsWedge[i];
            results.push_back(info2);
        }
        // END DEBUG*/


        // Checking the vertices of the box:
        // ----------------------------------
        real distancesW[5];
        for (int i = 0; i<8; ++i) {
            if (helpers.isInsideOrOnTheWedge(planesWedge,vertsBox[i],distancesW)) {
                std::pair<real,int> dist = 
                    helpers.calculatePenetrationDepth(distancesW, 5);

                info.normal =
                    boxPenetrator ? planesWedge[dist.second].getNormal() :
                    -(planesWedge[dist.second].getNormal());
                info.penetrationDepth = dist.first;
                info.collisionPoint = vertsBox[i];
                results.push_back(info);
            } 
        }
        real distancesB[6];
        for (int i = 0; i<6; ++i) {
            if (helpers.isInsideOrOnTheBox(planesBox,vertsWedge[i],distancesB)) {
                std::pair<real,int> dist = 
                    helpers.calculatePenetrationDepth(distancesB, 6);

                info.normal =
                    boxPenetrator ? -planesBox[dist.second].getNormal() :
                    (planesBox[dist.second].getNormal());
                info.penetrationDepth = dist.first;
                info.collisionPoint = vertsWedge[i];
                results.push_back(info);
            } //END else  if (helpers.isInsideTheBox(planes2,verts1[i]))
        } // END for (int i = 0; i<8; ++i)

        // Checking the edges of the box:
        // ----------------------------------
        // getting them:
        std::pair<int,int> edgesB[12];
        box->getEdges(edgesB);

        // Checking them:
        Vector3 v1_1,v1_2,v2_1,v2_2;
        Vector3 intersection;
        for (int i=0; i<12;++i) {
            // Check if the lines intersect and save the intersection-points:
            v1_1 = vertsBox[(edgesB[i]).first];
            v1_2 = vertsBox[(edgesB[i]).second];
            for (int j=0; j<5;++j) {
                if (planesWedge[j].intersectLineSegment(v1_1,v1_2, &intersection)) {

                    // Now check if one of these intersection points lies 
                    // beetween all 6^ opposing planes of the Wedge:
                    // Here we create an array of reals, so that we can re-use
                    // the distances calc. in isInsideOrOnTheWedge for our
                    // penetrationdepth

                    if (helpers.isInsideOrOnTheWedge(planesWedge,
                            intersection,distancesW)) {

                        // Bingo, we have a collision-point!
                        //std::cout <<" Bingo, we have a collision-point!" <<
                          //  std::endl;
                        std::pair<real,int> dist = 
                            helpers.calculatePenetrationDepth(distancesW, 5);

                        // Now we can set the collision info, but only if the point
                        // has a greater distance the maximum of distance:
                        info.normal =
                            boxPenetrator ? planesWedge[dist.second].getNormal()
                            : -(planesWedge[dist.second].getNormal());
                        info.penetrationDepth = dist.first;
                        info.collisionPoint = intersection;
                        results.push_back(info);
                    }
                } 
            } // END for (int j=0; j<5;++j) 
        } // END for (int i=0; i<12;++i) 

        // Checking the edges of the Wedge:
        // ----------------------------------
        // getting them:
        std::pair<int,int> edgesW[9];
        wedge->getEdges(edgesW);

        // Checking them:
        for (int i=0; i<9;++i) {
            // Check if the lines intersect and save the intersection-points:
            v1_1 = vertsWedge[(edgesW[i]).first];
            v1_2 = vertsWedge[(edgesW[i]).second];
            for (int j=0; j<6;++j) {
                if (planesBox[j].intersectLineSegment(v1_1,v1_2, &intersection)) {

                    // Now check if one of these intersection points lies 
                    // beetween all 6^ opposing planes of the box:
                    // Here we create an array of reals, so that we can re-use
                    // the distances calc. in isInsideOrOnTheBox for our
                    // penetrationdepth

                    if (helpers.isInsideOrOnTheBox(planesBox,
                            intersection,distancesB)) {

                        // Bingo, we have a collision-point!
                        //std::cout <<" Bingo, we have a collision-point!" <<
                          //  std::endl;
                        std::pair<real,int> dist = 
                            helpers.calculatePenetrationDepth(distancesB, 6);

                        // Now we can set the collision info, but only if the point
                        // has a greater distance the maximum of distance:
                        info.normal =
                            boxPenetrator ? planesBox[dist.second].getNormal()
                            : -(planesBox[dist.second].getNormal());
                        info.penetrationDepth = dist.first;
                        info.collisionPoint = intersection;
                        results.push_back(info);
                    }
                } 
            } // END for (int j=0; j<6;++j) 
        } // END for (int i=0; i<9;++i) 

        return results;
    }

}

/*
 * vim: et sw=4 ts=4
 */
