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

#include "triangleintersector.h"

#include "shapes/mesh/triangle.h"
#include "shapes/mesh/vertex.h"


#include "math/plane.h"

#define EPSILON 0.001

/* sort so that a<=b */
#define SORT2(a,b,smallest)       \
             if (a>b)       \
             {             \
               float c;    \
               c=a;        \
               a=b;        \
               b=c;        \
               smallest=1; \
             }             \
             else smallest=0;

namespace dcollide {

    /*!
     *\brief calculate triangle-triangle intersections
     * algorithm is described here:
     * http://www.cs.lth.se/home/Tomas_Akenine_Moller/pubs/tritri.pdf
     * implementation is similar to the one at
     * http://jgt.akpeters.com/papers/Moller97/tritri.html
     * with adjustments to use it in our library
     *      - calculation of penetration depth
     *      - use vectors instead of arrays
     *      - use our plane class
     *      - replace macros with functions
     *      - a nicer return-value
     *      - renamed functions and variables to make more readable
     * A note on penetration depth: We interprete t1 as penetrator. Thus,
     * the penetration depth depth is the maximal negative distance a vertex of
     * \p t1 has to the plane defined by \p t2 
     */
    void TriangleIntersector::computeTriangleIntersection(
                                                const Triangle* t1,
                                                const Triangle* t2,
                                                TriangleIntersection* result) {
        //Prepare result
        result->collision = false;
        result->coplanar  = false;
    
        Vector3 t10 = t1->getVertices()[0]->getWorldPosition();
        Vector3 t11 = t1->getVertices()[1]->getWorldPosition();
        Vector3 t12 = t1->getVertices()[2]->getWorldPosition();

        Vector3 t20 = t2->getVertices()[0]->getWorldPosition();
        Vector3 t21 = t2->getVertices()[1]->getWorldPosition();
        Vector3 t22 = t2->getVertices()[2]->getWorldPosition();

        //Calculate plane p1 of the triangle t1 (normal vectors is given)
        //initialize plane with normal vector and point on plane.
        //TODO Is there a faster way to do this? Yes: don't let Plane class
        // normalize this vector
        Plane p1(t1->getWorldCoordinatesNormalVector(), t10, false);
        //compute signed distances of t2's vertices to p1

        real distance_t20_p1 = p1.calculateDistance(t20);
        real distance_t21_p1 = p1.calculateDistance(t21);
        real distance_t22_p1 = p1.calculateDistance(t22);

        //Coplanarity robustness check, set distances to zero if very small
        if (fabs(distance_t20_p1) < EPSILON) {
            distance_t20_p1 = 0;
        }
        
        if (fabs(distance_t21_p1) < EPSILON) { 
            distance_t21_p1 = 0;
        }
        
        if (fabs(distance_t22_p1) < EPSILON) {
            distance_t22_p1 = 0;
        }

        //Test if all vertices of t1 are on the same side of p2 =>no collision

        // same sign on all of them + not equal 0 ?
        //we store the values so that we can reuse them later
        real multDistances_t20_t21 = distance_t20_p1 * distance_t21_p1;
        real multDistances_t20_t22 = distance_t20_p1 * distance_t22_p1;
        if (    multDistances_t20_t21 > 0 
             && multDistances_t20_t22 > 0) {
            return;
        }

        //same for t1 and p2
        Plane p2(t2->getWorldCoordinatesNormalVector(), t20, false);
        //compute signed distances of t1's vertices to p2
        real distance_t10_p2 = p2.calculateDistance(t10);
        real distance_t11_p2 = p2.calculateDistance(t11);
        real distance_t12_p2 = p2.calculateDistance(t12);

        //Coplanarity robustness check, set distances to zero if very small
        if (fabs(distance_t10_p2) < EPSILON) {
            distance_t10_p2=0;
        }
        if (fabs(distance_t11_p2) < EPSILON) {
            distance_t11_p2=0;
        }
        if (fabs(distance_t12_p2) < EPSILON) {
            distance_t12_p2=0;
        }

        //Test if all vertices of t1 are on the same side of p2 =>no collision
        // same sign on all of them + not equal 0 ?
        //we store the values so that we can reuse them later
        real multDistances_t10_t11 = distance_t10_p2 * distance_t11_p2;
        real multDistances_t10_t12 = distance_t10_p2 * distance_t12_p2;

        if (    multDistances_t10_t11 > 0 
            &&  multDistances_t10_t12 > 0) {
            return;
        }

        //calculate penetration depth: 
        //absolute value of the minimum of the 3 distances
        result->penetrationDepth = - std::min(distance_t10_p2, 
                                    std::min(distance_t11_p2, distance_t12_p2));

        //Intersection line L = O + t*D
        //with O:some point on L, D: direction

        Vector3 intersectionPoint;
        Vector3 intersectionVector;

        bool planeIntersection = p1.intersectPlane(p2, &intersectionPoint, &intersectionVector);
        //if the planes do not intersect, they might still be equal/coplanar
        Vector3 p2invertedNormal = p2.getNormal() * -1;
        if (!planeIntersection && p1.isCoplanar(p2)     ) {
            result->coplanar = true;
            //TODO coplanar results?
            result->collision = computeCoplanarIntersection3d(
                                                t10, t11, t12, p1.getNormal(),
                                                t20, t21, t22
                                                ) == 1;
            return;
        }

        //now we need to project the points onto the line
        //calculate minimum and maximum point on the line for each triangles,
        //and then see if there is an overlap

        //to do this, we use an approximation described in the code
        //GJ : i do not understand why they are doing this, but it works :-)

        //pick the largest (absolute) component of D (the intersectionVector)
        int maxComponentIndex = 0;
        real maxComponentAbsoluteValue = fabs(intersectionVector.getX());
        real yComponentAbsoluteValue = fabs(intersectionVector.getY());
        real zComponentAbsoluteValue = fabs(intersectionVector.getZ());
        if (yComponentAbsoluteValue > maxComponentAbsoluteValue) {
            maxComponentIndex = 1;
            maxComponentAbsoluteValue = yComponentAbsoluteValue;
        }

        if (zComponentAbsoluteValue > maxComponentAbsoluteValue) {
            maxComponentIndex = 2;
            maxComponentAbsoluteValue = zComponentAbsoluteValue;
        }

        //Calculate (simplified) projection of the triangles onto L
        real proj_t10 = t10[maxComponentIndex];
        real proj_t11 = t11[maxComponentIndex];
        real proj_t12 = t12[maxComponentIndex];

        real proj_t20 = t20[maxComponentIndex];
        real proj_t21 = t21[maxComponentIndex];
        real proj_t22 = t22[maxComponentIndex];

        //compute interval for triangle 1
        //if the planes are coplanar, we would have detected that at the 
        //plane-intersection step
        //to speed up the calculation we can reuse some values calculated before
        Vector3 t1LineSegmentStart;
        Vector3 t1LineSegmentEnd;
        real interval1[2];
        computeIntervalsOnIntersectionLine(t10, t11, t12, //the triangle points
                            proj_t10, proj_t11, proj_t12, //the projections
                            distance_t10_p2, distance_t11_p2,distance_t12_p2,
                            multDistances_t10_t11, multDistances_t10_t12,
                            &interval1[0], &interval1[1], //input values
                            t1LineSegmentStart, t1LineSegmentEnd
                        );

        //compute interval for triangle 2

        Vector3 t2LineSegmentStart;
        Vector3 t2LineSegmentEnd;
        real interval2[2];
        computeIntervalsOnIntersectionLine(t20, t21, t22, //the triangle points
                            proj_t20, proj_t21, proj_t22, //the projections
                            distance_t20_p1, distance_t21_p1,distance_t22_p1,
                            multDistances_t20_t21, multDistances_t20_t22, 
                            &interval2[0], &interval2[1],
                            t2LineSegmentStart, t2LineSegmentEnd 
                        );

        //check if intervals overlap
        //if yes, calculate start and endpoint of overlap 
        int smallest1 = 0;
        int smallest2 = 0;

#ifdef __GNUC__
#warning FIXME: uninitialized values
#endif
        SORT2(interval1[0], interval1[1], smallest1);
        SORT2(interval2[0], interval2[1], smallest2);

#ifdef __GNUC__
#warning FIXME: uninitialized values
#endif
        if ( interval1[1] < interval2[0] || interval2[1] < interval1[0]) {
            return;
        }

        //now we know the segments overlap
        result->collision = true;
        //set the vectors for start and endpoint
        if (interval2[0] < interval1[0]) {
            if (smallest1 == 0) {
                result->intersectionStart = t1LineSegmentStart;
            } else { 
                result->intersectionStart = t1LineSegmentEnd;
            }
    
            if (interval2[1] < interval1[1]) {
                if (smallest2 == 0) {
                    result->intersectionEnd = t2LineSegmentEnd;
                } else {
                    result->intersectionEnd = t2LineSegmentStart;
                }
            } else {
                if (smallest1 == 0) {
                    result->intersectionEnd = t1LineSegmentEnd;
                } else {
                    result->intersectionEnd = t1LineSegmentStart;
                }
            }
        } else {
            if (smallest2 == 0) {
                result->intersectionStart = t2LineSegmentStart;
            } else {
                    result->intersectionStart = t2LineSegmentEnd;
            }

            if (interval2[1] > interval1[1]) {
                if (smallest1 == 0) {
                    result->intersectionEnd = t1LineSegmentEnd;
                } else {
                    result->intersectionEnd = t1LineSegmentStart;
                }
            } else {
                if (smallest2 == 0) {
                    result->intersectionEnd = t2LineSegmentEnd;
                } else {
                    result->intersectionEnd = t2LineSegmentStart;
                } 
            }
        }

    }

/*!
 *\brief calculates Intervals of a triangles points on a line.
 * 
 * this function does not check if the triangles are coplanar, it expects that
 * they are not!
 */
    void TriangleIntersector::computeIntervalsOnIntersectionLine(
                                const Vector3& trianglePoint0,
                                const Vector3& trianglePoint1,
                                const Vector3& trianglePoint2,
                                real proj0, real proj1, real proj2,
                real planeDistance0, real planeDistance1, real planeDistance2,
                real multDistance01, real multDistance02,//D0D1, D0D2
                real* intervalStart, real* intervalEnd, //isect0, isect1
                Vector3& lineSegmentStart, Vector3& lineSegmentEnd //isectpoint0, isectpoint1
                ) {
    if ( multDistance01 > 0.0f )  {
        /* here we know that D0D2<=0.0 */
        /* that is D0, D1 are on the same side, D2 on the other or on the plane */
        calculateTriangleIntersectionWithLine(
                trianglePoint2, trianglePoint0, trianglePoint1,
                proj2, proj0, proj1,
                planeDistance2, planeDistance0, planeDistance1,
                intervalStart, intervalEnd,
                lineSegmentStart, lineSegmentEnd);
    } else if ( multDistance02 > 0.0f ) {
        /* here we know that d0d1<=0.0 */
        calculateTriangleIntersectionWithLine(
                trianglePoint1, trianglePoint0, trianglePoint2,
                proj1, proj0, proj2,
                planeDistance1, planeDistance0, planeDistance2,
                intervalStart, intervalEnd,
                lineSegmentStart, lineSegmentEnd);
    } else if (     ( planeDistance1 * planeDistance2 > 0.0f )
                ||  ( planeDistance0 != 0.0f) ) {
        /* here we know that d0d1<=0.0 or that D0!=0.0 */
        calculateTriangleIntersectionWithLine(
                trianglePoint0, trianglePoint1, trianglePoint2,
                proj0, proj1, proj2,
                planeDistance0, planeDistance1, planeDistance2,
                intervalStart, intervalEnd,
                lineSegmentStart, lineSegmentEnd);
    } else if ( planeDistance1 != 0.0f ) {
       calculateTriangleIntersectionWithLine(
                trianglePoint1, trianglePoint0, trianglePoint2,
                proj1, proj0, proj2,
                planeDistance1, planeDistance0, planeDistance2,
                intervalStart, intervalEnd,
                lineSegmentStart, lineSegmentEnd);
    }  else if ( planeDistance2 != 0.0f )  {
        calculateTriangleIntersectionWithLine(
                trianglePoint2, trianglePoint0, trianglePoint1,
                proj2, proj0, proj1,
                planeDistance2, planeDistance0, planeDistance1,
                intervalStart, intervalEnd,
                lineSegmentStart, lineSegmentEnd);
    }
    }

    /*!
     *\brief calculates the intersection of a triangle with a line L = S + t*D
     * The following parameters will be set by this function
     *\p intervalStart and \p intervalEnd are min and max values for t
     *\p lineSegmentStart and \p lineSegmentEnd the Coordinates of these positions
     */
    void TriangleIntersector::calculateTriangleIntersectionWithLine(
                const Vector3& point0,
                const Vector3& point1,
                const Vector3& point2,
                real proj0, real proj1, real proj2,
                real planeDistance0, real planeDistance1, real planeDistance2,
                real* intervalStart, real* intervalEnd,
                Vector3& lineSegmentStart, Vector3& lineSegmentEnd ) {
        //calculate Start
        real tmp = planeDistance0 / (planeDistance0 - planeDistance1);
        *intervalStart = proj0 + (proj1 - proj0) * tmp;
        Vector3 diff = (point1 - point0) * tmp;
        lineSegmentStart = diff + point0;
        //calculate end
        tmp = planeDistance0 / (planeDistance0 - planeDistance2);
        *intervalEnd = proj0 + (proj2 - proj0) * tmp;
        diff = (point2 - point0) * tmp;
        lineSegmentEnd = point0 + diff;
    }


/*
*  Coplanarity tests by Philippe Guigue, Olivier Devillers
*  http://home.arcor.de/philippe.guigue/triangle_triangle_intersection.htm
*  Two dimensional Triangle-Triangle Overlap Test    
*/

/* some 2D macros */

#define ORIENT_2D(a, b, c)  ((a[0]-c[0])*(b[1]-c[1])-(a[1]-c[1])*(b[0]-c[0]))


#define INTERSECTION_TEST_VERTEX(P1, Q1, R1, P2, Q2, R2) {\
  if (ORIENT_2D(R2,P2,Q1) >= 0.0f)\
    if (ORIENT_2D(R2,Q2,Q1) <= 0.0f)\
      if (ORIENT_2D(P1,P2,Q1) > 0.0f) {\
    if (ORIENT_2D(P1,Q2,Q1) <= 0.0f) return 1; \
    else return 0;} else {\
    if (ORIENT_2D(P1,P2,R1) >= 0.0f)\
      if (ORIENT_2D(Q1,R1,P2) >= 0.0f) return 1; \
      else return 0;\
    else return 0;}\
    else \
      if (ORIENT_2D(P1,Q2,Q1) <= 0.0f)\
    if (ORIENT_2D(R2,Q2,R1) <= 0.0f)\
      if (ORIENT_2D(Q1,R1,Q2) >= 0.0f) return 1; \
      else return 0;\
    else return 0;\
      else return 0;\
  else\
    if (ORIENT_2D(R2,P2,R1) >= 0.0f) \
      if (ORIENT_2D(Q1,R1,R2) >= 0.0f)\
    if (ORIENT_2D(P1,P2,R1) >= 0.0f) return 1;\
    else return 0;\
      else \
    if (ORIENT_2D(Q1,R1,Q2) >= 0.0f) {\
      if (ORIENT_2D(R2,R1,Q2) >= 0.0f) return 1; \
      else return 0; }\
    else return 0; \
    else  return 0; \
 };



#define INTERSECTION_TEST_EDGE(P1, Q1, R1, P2, Q2, R2) { \
  if (ORIENT_2D(R2,P2,Q1) >= 0.0f) {\
    if (ORIENT_2D(P1,P2,Q1) >= 0.0f) { \
        if (ORIENT_2D(P1,Q1,R2) >= 0.0f) return 1; \
        else return 0;} else { \
      if (ORIENT_2D(Q1,R1,P2) >= 0.0f) { \
    if (ORIENT_2D(R1,P1,P2) >= 0.0f) return 1; else return 0;} \
      else return 0; } \
  } else {\
    if (ORIENT_2D(R2,P2,R1) >= 0.0f) {\
      if (ORIENT_2D(P1,P2,R1) >= 0.0f) {\
    if (ORIENT_2D(P1,R1,R2) >= 0.0f) return 1;  \
    else {\
      if (ORIENT_2D(Q1,R1,R2) >= 0.0f) return 1; else return 0;}}\
      else  return 0; }\
    else return 0; }}

int TriangleIntersector::computeIntersection2dCcw(
                real p1[2], real q1[2], real r1[2], 
                real p2[2], real q2[2], real r2[2]) {
    if ( ORIENT_2D(p2,q2,p1) >= 0.0f ) {
        if ( ORIENT_2D(q2,r2,p1) >= 0.0f ) {
            if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) { 
                return 1;
            } else {
                INTERSECTION_TEST_EDGE(p1, q1, r1, p2, q2, r2);
            } //end if  ORIENT_2D(r2,p2,p1) > 0
        } else {  
            if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) {
                INTERSECTION_TEST_EDGE(p1, q1, r1, r2, p2, q2);
            } else {
                INTERSECTION_TEST_VERTEX(p1, q1, r1, p2, q2, r2);
            }
        }
    } else {
        if ( ORIENT_2D(q2,r2,p1) >= 0.0f ) {
            if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) {
                INTERSECTION_TEST_EDGE(p1,q1,r1,q2,r2,p2);
            } else {
                INTERSECTION_TEST_VERTEX(p1,q1,r1,q2,r2,p2);
            }
        } else {
            INTERSECTION_TEST_VERTEX(p1,q1,r1,r2,p2,q2);
        }
    }
};


int TriangleIntersector::checkOverlap2d(
                real p1[2], real q1[2], real r1[2], 
                real p2[2], real q2[2], real r2[2]) {
    if ( ORIENT_2D(p1,q1,r1) < 0.0f ) {
        if ( ORIENT_2D(p2,q2,r2) < 0.0f ) {
            return computeIntersection2dCcw(p1, r1, q1,
                                            p2, r2, q2);
        } else {
            return computeIntersection2dCcw(p1, r1, q1,
                                            p2, q2, r2);
        }
    } else {
        if ( ORIENT_2D(p2,q2,r2) < 0.0f ) {
            return computeIntersection2dCcw(p1, q1, r1,
                                            p2, r2, q2);
        } else {
            return computeIntersection2dCcw(p1, q1, r1,
                                            p2, q2, r2);
        }
    }

}

int TriangleIntersector::computeCoplanarIntersection3d(const Vector3& p1, 
                                            const Vector3& q1,
                                            const Vector3& r1,
                                            const Vector3& normal1, 
                                            const Vector3& p2,
                                            const Vector3& q2,
                                            const Vector3& r2
                                            /*const Vector3& normal_2*/) {
  
    real P1[2], Q1[2], R1[2];
    real P2[2], Q2[2], R2[2];
    
    real absoluteNormalX, absoluteNormalY, absoluteNormalZ;
    
    absoluteNormalX = fabs(normal1[0]);
    absoluteNormalY = fabs(normal1[1]);
    absoluteNormalZ = fabs(normal1[2]);
    
    
    /* Projection of the triangles in 3D onto 2D such that the area of
        the projection is maximized. */
    
    
    if (        ( absoluteNormalX > absoluteNormalZ ) 
          &&    ( absoluteNormalX >= absoluteNormalY ) ) {
        // Project onto plane YZ
    
        P1[0] = q1[2];
        P1[1] = q1[1];
        
        Q1[0] = p1[2];
        Q1[1] = p1[1];
        
        R1[0] = r1[2];
        R1[1] = r1[1]; 
        
        P2[0] = q2[2];
        P2[1] = q2[1];
        
        Q2[0] = p2[2];
        Q2[1] = p2[1];
        
        R2[0] = r2[2];
        R2[1] = r2[1]; 
    
    } else if (         ( absoluteNormalY > absoluteNormalZ ) 
                  &&    ( absoluteNormalY >= absoluteNormalX ) ) {
        // Project onto plane XZ
    
        P1[0] = q1[0];
        P1[1] = q1[2];
        
        Q1[0] = p1[0];
        Q1[1] = p1[2];
        
        R1[0] = r1[0];
        R1[1] = r1[2]; 
    
        P2[0] = q2[0];
        P2[1] = q2[2];
        
        Q2[0] = p2[0];
        Q2[1] = p2[2];
        
        R2[0] = r2[0];
        R2[1] = r2[2]; 

    } else {
        // Project onto plane XY

        P1[0] = p1[0];
        P1[1] = p1[1]; 
        
        Q1[0] = q1[0];
        Q1[1] = q1[1]; 
        
        R1[0] = r1[0];
        R1[1] = r1[1]; 

        P2[0] = p2[0];
        P2[1] = p2[1];
         
        Q2[0] = q2[0];
        Q2[1] = q2[1]; 
        
        R2[0] = r2[0];
        R2[1] = r2[1]; 
    }

    return checkOverlap2d(P1, Q1, R1,
                          P2, Q2, R2);

}

}

/*
 * vim: et sw=4 ts=4
 */
