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

#include "spheresphereintersector.h"

#include <shapes/sphere.h>
#include <math/vector.h>
#include <math/matrix.h>
#include "collisioninfo.h"

#include "debugstream.h"

#include <math.h>

namespace dcollide {

/*!
 *\brief calculates the collision between two spheres
 *\section pseudocode (for accuracy level 4)
 *  - center points are a function of t
 *      (do a linear interpolation between prev and current state)
 *  - solve mathematically for t=(0,1]:
 *     distance of center1(t) - center2(t)  = sum of radii
 *      this is a quadratic function of t, that needs to be solved
 *  - if there is a solution for t (if there are two, take the smaller one)
 *          calculate centers of spheres at time tColl
 *          the collision normal n = (center1(tColl)-center2(tColl)),
 *          the collision point p = center2(tColl)+ radius2 * n
 *    proceed with calculating penetration depth
 *  - penetration depth is max distance of sphere1 to p, along n at time t=1
 */
std::list<CollisionInfo> SphereSphereIntersector::calculateIntersection(NarrowPhaseStrategy strategy,
        const Matrix& prevState1, const Matrix& currentState1,
        const Sphere* sphere1,    const Matrix& prevState2,
        const Matrix& currentState2, const Sphere* sphere2) const {
    


        //the simple test for spheres is so easy that it cannot get much faster
        //so, we treat the first two levels exactly the same, differing only in
        //the accuracy of the length-calculations
    if (strategy == NP_STRATEGY_FASTEST_SLOPPY ||
        strategy == NP_STRATEGY_FAST_CONSIDER_SHAPE) {
        return calculateSimpleIntersection(
                currentState1, sphere1,
                currentState2, sphere2);
    }
/*
    if (strategy == NP_STRATEGY_SLOW_BACKTRACKING) {
        std::cerr << "Not implemented yet" <<std::endl;
        //TODO
        //linear interpolation, 4 Steps
        //for tColl=1/4, 1/2, 3/4, 1, check for collision
        //return center of overlap at the first state with collision
        return result;
    }//end if lvl3
*/
    //accuracy level 4 is needed (exactly solve for tColl)
    return calculateExactIntersection(prevState1, currentState1,
            sphere1,prevState2,
            currentState2, sphere2);
}
/*!
 * \brief calculate collisions based on current state only
 * 
 */
std::list<CollisionInfo> SphereSphereIntersector::calculateSimpleIntersection(
        const Matrix& currentState1, const Sphere* sphere1,
        const Matrix& currentState2, const Sphere* sphere2) const {
    std::list<CollisionInfo> result;
    //calc. center points of both spheres at their current/previous state
    Vector3 prev1;
    Vector3 prev2;
    Vector3 current1;
    Vector3 current2;

    currentState1.transform(&current1, Vector3(0.0, 0.0, 0.0));
    currentState2.transform(&current2, Vector3(0.0, 0.0, 0.0));
    
    //do we have a collision now?
    Vector3 currentCenterConnection =current2 - current1;

    real currentDistance = currentCenterConnection.length();
    if (currentDistance > sphere1->getRadius() + sphere2->getRadius()) {
        //no collision
        return result;
    }

    //We have a collision
    
    return generateCollisionInfo(sphere1, current1, sphere2,
            currentCenterConnection, currentDistance);


}

std::list<CollisionInfo> SphereSphereIntersector::generateCollisionInfo(
        const Sphere* sphere1, Vector3& current1,
        const Sphere* sphere2, Vector3& currentCenterConnection, real currentDistance) const{
    std::list<CollisionInfo> result;
    
    //return the center of the overlap region as collision point
    real distanceToOverlapCenterFromSphere1 = (currentDistance + sphere1->getRadius() - sphere2->getRadius())/2;
    Vector3 normalizedCenterConnection = currentCenterConnection;
    normalizedCenterConnection.normalize();
    Vector3 collisionPoint = current1 + normalizedCenterConnection * distanceToOverlapCenterFromSphere1;
    
    //Collision normal is the normalised version of the center-connection
    //we can do this only if that vector is not (0,0,0) i.e the centerpoints are equal
    //if the centerpoints are equal, we can pick an arbitrary
    //normal direction
    Vector3 collisionNormal;
    if (currentCenterConnection.isNull()) {
        collisionNormal = Vector3(1, 0, 0);
    } else {
        collisionNormal = currentCenterConnection;
        collisionNormal.normalize();
    }
    
    CollisionInfo coll;
    coll.normal = collisionNormal;
    coll.collisionPoint = collisionPoint;
    coll.penetratedProxy = sphere1->getProxy();
    coll.penetratingProxy = sphere2->getProxy();
    //now calculate penetration depth: length of overlap-area
    coll.penetrationDepth =    sphere1->getRadius()
                            +  sphere2->getRadius()
                            -  currentCenterConnection.length();
    
    result.push_back(coll);
    return result;
}

std::list<CollisionInfo> SphereSphereIntersector::calculateExactIntersection(
        const Matrix& prevState1, const Matrix& currentState1,
        const Sphere* sphere1,    const Matrix& prevState2,
        const Matrix& currentState2, const Sphere* sphere2) const {
    std::list<CollisionInfo> result;
    //calc. center points of both spheres at their current/previous state
    Vector3 prev1;
    Vector3 prev2;
    Vector3 current1;
    Vector3 current2;

    prevState1.transform(&prev1, Vector3(0.0, 0.0, 0.0));
    prevState2.transform(&prev2, Vector3(0.0, 0.0, 0.0));
    currentState1.transform(&current1, Vector3(0.0, 0.0, 0.0));
    currentState2.transform(&current2, Vector3(0.0, 0.0, 0.0));

    //first, check if we started at a collision. In this case we need
    // special handling to not produce irritating results
    //if there was a collision before....
    //       ...and there is no collision in the current state,
    //              return "no collision" (since we are "on the way out"
    //       ...and there is still a collision, we are not moving far enough
    //           out, and our standard algorithm won't find a valid solution
    //debug() << "-- calculating sphere-sphere collision---";
    Vector3 currentCenterConnection = current2-current1;
    real currentDistance = currentCenterConnection.length();
    Vector3 prevCenterConnection = prev2-prev1;
    real prevDistance = prevCenterConnection.length();
        
    real radiusSum = sphere1->getRadius() + sphere2->getRadius();
    bool previousCollision=false;
    /*
    debug() << "-- sum of radiuses spheres: " << radiusSum
                << "\n-- previous distance of sphere centers: " << prevDistance
                << "\n-- current distance of sphere centers: " << currentDistance;
    */
    if (prevDistance <= radiusSum) {
        previousCollision=true;
        //debug() <<  "previous state collision..."; 
        
        if (currentCenterConnection.length() >= radiusSum) {
            //std::cout << ".. but no current collision";
            //collision on the way out=>ignore it
            return result;
        } else {
            //current collision and previous collision
            //debug() << "..and still a collision";
            //TODO if we had access to the last steps collisioninfo,we could
            //calculate better results by just returning the cached info and
            //recalculating the penetration depth
            return generateCollisionInfo(sphere1, current1, sphere2,
                        currentCenterConnection, currentDistance);
        }

    }


    //calculate velocities of the spheres (linear approximation)
    
    Vector3 v1 = current1 - prev1;
    Vector3 v2 = current2 - prev2;
    //TODO check if the spheres move in the same direction at the same speed
    //If they do, the distance between the two points will always stay
    //the same, and we need special case handling
    if (v1 == v2) {
        //decide wether there is a collision or not: we can just use any
        //time for the test, so we can also reuse the information gained
        //before:
        //if there was a previous collision before, the previous section
        //would have handled the case
        //=> there was no previous collision, and we cannot have a collision
        //      now as well
        return result;
    }

    //solve distance-of-line equitations for center points
    //the distance equitation for given g1(t) and g2(t) as lines of center:
    // |g1(t) - g2(t)|^2 = (radius1 + radius2)^2
    // with g1(t) = prev1 + t * (current1 - prev1) = prev1 + t * v1
    // and  g2(t) = prev2 + t * (current2 - prev2) = prev2 + t * v2

    // <=>  (prev1.X + t*v1.X - (prev2.X + t*v2.X))^2
    //     +(prev1.Y + t*v1.Y - (prev2.Y + t*v2.Y)^2
    //     +(prev1.Z + t*v1.X - (prev2.Z + t*v2.Z))^2
    //     -(radius1^2 + 2*radius1*radius2 + radius2^2) = 0

    // <=>  t^2 * (   v1.X^2 + v2.X^2 + v1.Y^2 + v2.Y^2 + v1.Z^2 + v2.Z^2
    //              - 2*v1.X*v2.X - 2*v1.Y*v2.Y - 2*v1.Z*v2.Z)

    //    + t   * ( +2*prev1.X*v1.X - 2*prev1.X*v2.X
    //              -2*prev2.X*v1.X + 2*prev2.X*v2.X
    //              +2*prev1.Y*v1.Y - 2*prev1.Y*v2.Y
    //              -2*prev2.Y*v1.Y + 2*prev2.Y*v2.Y
    //              +2*prev1.Z*v1.Z - 2*prev1.Z*v2.Z
    //              -2*prev2.Z*v1.Z + 2*prev2.Z*v2.Z )

    //    + prev1.X^2 - 2*prev1.X*prev2.X + prev2.X^2
    //    + prev1.Y^2 - 2*prev1.Y*prev2.Y + prev2.Y^2
    //    + prev1.Z^2 - 2*prev1.Z*prev2.Z + prev2.Z^2
    //    - radius1^2 - 2*radius1*radius2 - radius2^2
    //    = 0
    //  solve this with the abc-formula


    real a =      v1.getX()*v1.getX() + v2.getX()*v2.getX()
                + v1.getY()*v1.getX() + v2.getY()*v2.getY()
                + v1.getZ()*v1.getZ() + v2.getZ()*v2.getZ()
                - 2*v1.getX()*v2.getX()
                - 2*v1.getY()*v2.getY()
                - 2*v1.getZ()*v2.getZ();

    real b =      2*prev1.getX()*v1.getX() - 2*prev1.getX()*v2.getX()
                - 2*prev2.getX()*v1.getX() + 2*prev2.getX()*v2.getX()
                + 2*prev1.getY()*v1.getY() - 2*prev1.getY()*v2.getY()
                - 2*prev2.getY()*v1.getY() + 2*prev2.getY()*v2.getY()
                + 2*prev1.getZ()*v1.getZ() - 2*prev1.getZ()*v2.getZ()
                - 2*prev2.getZ()*v1.getZ() + 2*prev2.getZ()*v2.getZ() ;

    real c =      prev1.getX()*prev1.getX() - 2*prev1.getX()*prev2.getX()
                                            + prev2.getX()*prev2.getX()
                + prev1.getY()*prev1.getY() - 2*prev1.getY()*prev2.getY()
                                            + prev2.getY()*prev2.getY()
                + prev1.getZ()*prev1.getZ() - 2*prev1.getZ()*prev2.getZ()
                                            + prev2.getZ()*prev2.getZ()
                - sphere1->getRadius()*sphere1->getRadius()
                - 2*sphere1->getRadius()*sphere2->getRadius()
                - sphere2->getRadius()*sphere2->getRadius();

    //  calculate the determinant to see how many solutions are possible
    //solving a*t^2 + b*t + c = 0
    //first, do some sanity checks to maybe not use the abc-formula

    //std::cout   << "Sphere-sphere collision routine: a,b, and c have been calculated:"
    //        << "a=" << a << ", b=" << b << ", c=" << c << std::endl;

    real tColl=-1;
    if (a==0) {
        if (b==0) {//no solution
            return result;
        }
        tColl=-c/b;
    }
    if (b==0 && -c/a <0) {
        //we know a!=0 here
        //sqrt of negative number-> no solution
        return result;
    }
    //if tColl is still -1, we need to calculate it with the abc-formula
    if (tColl==-1) {
        real determinant = b * b - 4*a*c;
        if (determinant < 0) {//no solution=>no collision
            return result;
        }

        if (determinant == 0) {//one solution, no need to calculate sqrt(0)
            tColl = - b / (2*a);
        } else {//two collisions, pick the earlier one with tColl still > 0
            real sqrtDeterminant = sqrt(determinant);
            tColl = (- b - sqrtDeterminant)/ (2*a);
            if (tColl < 0) {
                tColl = (- b + sqrtDeterminant)/ (2*a);
            }
        }
    }

    //debug()<< "Detected collision at intermediate tColl = "<< tColl;

    //reject collisions with t<0 or  t>1 (no collision in this time step)
    //TODO  collisions with t>1 hint to future collisions
    if (tColl < 0 || (tColl > 1 && !previousCollision)) {
            return result;
    }

    //calculate collision normal and point
    //ENHANCEMENT maybe we could use internal results from the above
    //                  calculation for this


    //Since we want the collisioninfo to have a penetrating and a penetrated
    //Proxy, we need to take the faster one as intruder
    //The penetration normal and depth depends on this interpretation
    Vector3 collNormal;
    bool sphere1isPenetrator = v1.length() >= v2.length();
    if (sphere1isPenetrator) {
        collNormal = prev1 + v1*tColl- (prev2 + v2*tColl);
        
    } else {
        collNormal = prev2 + v2*tColl- (prev1 + v1*tColl);
    }

    collNormal.normalize();
    Vector3 collPoint;
    if (sphere1isPenetrator) {
         collPoint = prev2 + v2*tColl + collNormal*sphere2->getRadius();
    } else {
        collPoint = prev2 + v2*tColl - collNormal*sphere2->getRadius();
    }


    //calculate penetration depth
    //the end point will have the maximal penetration depth
    //  maximal point m = current1 + radius1 * -n
    //  penetration depth = distance of m to the plane
    //                      defined by collision point p and normal vector n
    Vector3 maxPenetrationPoint;
    if (sphere1isPenetrator) {
        maxPenetrationPoint = current1 - collNormal*sphere1->getRadius();
    } else {
        maxPenetrationPoint = current2 - collNormal*sphere2->getRadius();
    }

    CollisionInfo coll;
    coll.normal = collNormal;
    coll.collisionPoint = collPoint;
    if (sphere1isPenetrator) {
        coll.penetratingProxy = sphere1->getProxy();
        coll.penetratedProxy = sphere2->getProxy();
    } else {
        coll.penetratingProxy = sphere2->getProxy();
        coll.penetratedProxy = sphere1->getProxy();
    }

    coll.penetrationDepth = Vector3::distanceToPlane(   maxPenetrationPoint,
                                                        collPoint,
                                                        collNormal);

    result.push_back(coll);
    return result;

}
}//end namespace dcollide    
