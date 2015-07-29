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

#include "spheretriangleintersector.h"

#include "d-collide/math/plane.h"
#include "d-collide/math/vector.h"
#include "d-collide/math/matrix.h"
#include "d-collide/shapes/mesh/triangle.h"
#include "d-collide/shapes/mesh/vertex.h"
#include "d-collide/shapes/sphere.h"
#include "d-collide/proxy.h"
#include "d-collide/collisioninfo.h"

#include "d-collide/debugstream.h"

namespace dcollide {
CollisionInfo* SphereTriangleIntersector::getIntersection(const Sphere* sphere,
                                         const Triangle* triangle,
                                         Proxy* triangleProxy) {
    const dcollide::array<Vertex*,3> vertices = triangle->getVertices();
    
    Vector3 sphereCenter = sphere->getProxy()->getWorldTransformation().getPosition();
    
    //Using plane constructor with point-on plane and normal
    Plane triPlane(triangle->getWorldCoordinatesNormalVector(), vertices[0]->getWorldPosition(),false);
    
    //GJ: i assume that this is the signed distance
    real distanceSpherePlane = triPlane.calculateDistance(sphereCenter);
    
    if (fabs(distanceSpherePlane) > sphere->getRadius()) {
        return 0;
    }
    //dcollide::debug () << "sphere penetrates the triangle plane";
    //The sphere penetrates the triangle plane.
    //Now check if the projection is in the triangle
    Vector3 projection = sphereCenter - triPlane.getNormal() * distanceSpherePlane;
    
    real u; //u and v will be set by triangle->containsPoint()
    real v;
    triangle->containsPoint(projection, true, false, &u, &v);
    
    // Check if point is in triangle
    //return (u > 0) && (v > 0) && (u + v < 1)
    //dcollide::debug() << " u = " << u << ", v = " << v;
    
    if (u > 0) {
        if (v > 0) {
            if (u + v <1) {
                //dcollide::debug() << "sphere-triangle face collision";
                //face collision
                CollisionInfo* coll = new CollisionInfo();
                coll->collisionPoint = projection;
                coll->normal = triPlane.getNormal();
                coll->penetrationDepth = sphere->getRadius() - distanceSpherePlane;
                
                coll->penetratedProxy = triangleProxy;
                coll->penetratedTriangle = const_cast<Triangle*>(triangle);
                coll->penetratingProxy = sphere->getProxy();
                return coll;
            } else {
                //outside of edge12
                return testEdge(vertices[1]->getWorldPosition(), triangle->getEdge12(),
                                sphereCenter, triangle, triangleProxy, sphere);
                
            }
        }else {
            //outside out edge02
            return testEdge(vertices[0]->getWorldPosition(), triangle->getEdge02(),
                            sphereCenter, triangle, triangleProxy, sphere);
        }
    } else {
        //outside of edge01
        return testEdge(vertices[0]->getWorldPosition(), triangle->getEdge01(),
                        sphereCenter, triangle, triangleProxy, sphere);
    }
    
    return 0;
}

CollisionInfo* SphereTriangleIntersector::testEdge(const Vector3& edgeStart, const Vector3& edgeDir,
        const Vector3& sphereCenter,
        const Triangle* triangle, Proxy* triangleProxy, const Sphere* sphere){
    //compute parameter t of "foot point" on ray which is closest to the sphere center
    //if t is in [0,1], we could have an edge collision
    //ray = 
    // 
    // footpoint (on ray) F = edgeStart + t * edgeDir
    // minimal distance: skalar product (footpoint - sphereCenter) * edgeDir = 0
    // =>    edgeDirX * (edgeStartX + t*edgeDirX - sphereCenterX)
    //      +edgeDirY * (edgeStartY + t*edgeDirY - sphereCenterY)
    //      +edgeDirZ * (edgeStartZ + t*edgeDirZ - sphereCenterZ) = 0
    //
    // solving for t yields
    //  t=(  edgeDirZ*sphereCenterZ
    //      +edgeDirY*sphereCenterY
    //      +edgeDirX*sphereCenterX
    //      -edgeDirZ*edgeStartZ
    //      -edgeDirY*edgeStartY
    //      -edgeDirX*edgeStartX)
    //      /
    //      (edgeDirZ^2+edgeDirY^2+edgeDirX^2)
    real t = (  edgeDir.getZ() * sphereCenter.getZ()
               +edgeDir.getY() * sphereCenter.getY()
               +edgeDir.getX() * sphereCenter.getX()
               -edgeDir.getZ() * edgeStart.getZ()
               -edgeDir.getY() * edgeStart.getY()
               -edgeDir.getX() * edgeStart.getX()
              )/(
                edgeDir.getZ() * edgeDir.getZ()
               +edgeDir.getY() * edgeDir.getY()
               +edgeDir.getX() * edgeDir.getX()
              );
    if (t>0){
        if (t<=1) {
            //closest point is on edge, compare distance with radius
            //compute footpoint
            Vector3 footpoint = edgeStart + edgeDir * t;
            real distance = (sphereCenter - footpoint).length();
            if (distance < sphere->getRadius()) {
                //edge collision
                CollisionInfo* coll = new CollisionInfo();
                coll->collisionPoint = footpoint;
                coll->normal = sphereCenter - footpoint;
                coll->normal.normalize();
                coll->penetrationDepth = sphere->getRadius() - distance;
                
                coll->penetratedProxy = triangleProxy;
                coll->penetratedTriangle = const_cast<Triangle*>(triangle);
                coll->penetratingProxy = sphere->getProxy();
                return coll;
            }
        }else{
            //check vertex collision with edgeEnd
            return testVertex(edgeStart+edgeDir, sphereCenter, triangle, triangleProxy, sphere);
        }
    } else {
        //check vertex collision with edgeStart
        return testVertex(edgeStart, sphereCenter, triangle, triangleProxy, sphere);
    }
    return 0;
}

/*!
 * \brief create a collision if the vertex is within the sphere
 *
 */
CollisionInfo* SphereTriangleIntersector::testVertex(const Vector3& vertex,
        const Vector3& sphereCenter,
        const Triangle* triangle, Proxy* triangleProxy, const Sphere* sphere){
    //this is really basic, just a simple point-in triangle test
    Vector3 connection = sphereCenter - vertex;
    real distance = connection.length();
    if (distance < sphere->getRadius()) {
        //vertex collision
        CollisionInfo* coll = new CollisionInfo();
        coll->collisionPoint = vertex;
        coll->normal = connection;
        coll->normal.normalize();
        coll->penetrationDepth = sphere->getRadius() - distance;
        
        coll->penetratedProxy = triangleProxy;
        coll->penetratedTriangle = const_cast<Triangle*>(triangle);
        coll->penetratingProxy = sphere->getProxy();
        return coll;
    }
    return 0;
}
}//end namespace
/*
 * vim: et sw=4 ts=4
 */
