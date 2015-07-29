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
#include "narrowphase.h"

#include "math/vector.h"
#include "math/matrix.h"
#include "real.h"
#include "shapes/shapes.h"
#include "boundingvolumes/boundingvolume.h"
#include "triangleintersector.h"
#include "spheretriangleintersector.h"
#include "boxsphereintersector.h"
#include "boxboxintersector.h"
#include "wedgeintersector.h"
#include "coneintersector.h"
#include "cylinderintersector.h"
#include "spheresphereintersector.h"
#include "meshmeshintersector.h"
#include "bvhnode.h"
#include "proxy.h"
#include "collisioninfo.h"
#include "detectordeform/potentialcollidingsets.h"

#include "dcollide-defines.h"
#include "debugstream.h"
#include <math.h>
#include <iostream>
#include <limits>

#define BOXBOXDEBUG 1
#undef BOXBOXDEBUG

namespace dcollide {

    NarrowPhase::NarrowPhase(const NarrowPhaseShapeStrategies& strategies) {
        mStrategySphereSphere = strategies.mStrategySphereSphere;
        mStrategyBoxBox = strategies.mStrategyBoxBox;
        mStrategyMeshMesh = strategies.mStrategyMeshMesh;
        mStrategyBoxSphere = strategies.mStrategyBoxSphere;
        mStrategySphereMesh = strategies.mStrategySphereMesh;
    }

    NarrowPhase::~NarrowPhase() {
    }

    /*!
     *  \brief perform the intersection and return all collisions
     *  This method will be called by the middlephases (rigid and deformable)
     *  \param collision the BoundingVolumeCollision info produced by middlephases
     *  \return list of Intersection points,
     *              including penetration-normal and -depth
     */
    std::list<CollisionInfo>
            NarrowPhase::getIntersection(const BoundingVolumeCollision& collision) const{
        if (!collision.node1 || !collision.node2) {
            throw NullPointerException("one of the BvhNodes of the BoundingVolumeCollision");
        }
        if (!collision.node1->getShape() || !collision.node2->getShape()) {
            throw NullPointerException("one of the Shapes of the nodes of the BoundingVolumeCollision");
        }

        Shape* shape1 = collision.node1->getShape();
        Shape* shape2 = collision.node2->getShape();

        //switch on the types of the shapes referenced in collision
        //call private methods for the types

        //By default, we will use getMesh function of the shapes and the
        // Mesh-Intersection code
        switch (shape1->getShapeType()) {
            case Shape::SHAPE_TYPE_SPHERE: {
                switch (shape2->getShapeType()) {
                    case Shape::SHAPE_TYPE_SPHERE: {
                        SphereSphereIntersector intersector;
                        return intersector.calculateIntersection(mStrategySphereSphere,
                                    shape1->getProxy()->getPreviousWorldTransformation(),
                                    shape1->getProxy()->getWorldTransformation(),
                                    (Sphere*) shape1,
                                    shape2->getProxy()->getPreviousWorldTransformation(),
                                    shape2->getProxy()->getWorldTransformation(),
                                    (Sphere*) shape2);
                    }
                    case Shape::SHAPE_TYPE_BOX: {
                        return getBoxSphereIntersection(
                                    shape2->getProxy()->getPreviousWorldTransformation(),
                                    shape2->getProxy()->getWorldTransformation(),
                                    (Box*) shape2,
                                    shape1->getProxy()->getPreviousWorldTransformation(),
                                    shape1->getProxy()->getWorldTransformation(),
                                    (Sphere*) shape1);
                    }
                    default:
                        // as default we always use SphereMeshIntersection, this
                        // is much faster as MeshMesh
                        return getSphereMeshIntersection(
                                    (Sphere*) shape1,
                                    shape2->getMesh()->getTriangles(),
                                    shape2->getProxy());
                        break;
                }//end switch on shapetype 2
                break;
            }//endcase shape 1 is a sphere
            case Shape::SHAPE_TYPE_BOX: {
                switch (shape2->getShapeType()) {
                    case Shape::SHAPE_TYPE_SPHERE: {
                        return getBoxSphereIntersection(
                                    shape1->getProxy()->getPreviousWorldTransformation(),
                                    shape1->getProxy()->getWorldTransformation(),
                                    (Box*) shape1,
                                    shape2->getProxy()->getPreviousWorldTransformation(),
                                    shape2->getProxy()->getWorldTransformation(),
                                    (Sphere*) shape2);
                    }//endcase shape2 is a sphere
                    case Shape::SHAPE_TYPE_BOX: {
                        return getBoxBoxIntersection(
                                shape1->getProxy()->getPreviousWorldTransformation(),
                                shape1->getProxy()->getWorldTransformation(),
                                (Box*) shape1,
                                shape2->getProxy()->getPreviousWorldTransformation(),
                                shape2->getProxy()->getWorldTransformation(),
                                (Box*) shape2);
                    } // endcase shape2 is a box
                    case Shape::SHAPE_TYPE_WEDGE: {
                        WedgeIntersector WedgeIntersector;
                        return WedgeIntersector.getIntersection(
                                (Box*) shape1,
                                (Wedge*) shape2);
                    } // endcase shape2 is a wedge
                    case Shape::SHAPE_TYPE_CYLINDER: {
                        CylinderIntersector boxCylinderIntersector;
                        return boxCylinderIntersector.getIntersection(
                                shape1->getProxy()->getWorldTransformation(),
                                (Box*) shape1,
                                shape2->getProxy()->getWorldTransformation(),
                                (Cylinder*) shape2);
                    } // endcase shape2 is a cylinder
                    case Shape::SHAPE_TYPE_CONE: {
                        ConeIntersector ConeIntersector;
                        return ConeIntersector.getIntersection(
                                shape1->getProxy()->getWorldTransformation(),
                                (Box*) shape1,
                                shape2->getProxy()->getWorldTransformation(),
                                (Cone*) shape2);
                    } // endcase shape2 is a cone
                    default:
                        break;
                }//end switch on shapetype2
                break;
            }//end shape1 is a box
            case Shape::SHAPE_TYPE_CYLINDER: {
                switch (shape2->getShapeType()) {
                    case Shape::SHAPE_TYPE_BOX: {
                        CylinderIntersector CylinderIntersector;
                        return CylinderIntersector.getIntersection(
                                shape2->getProxy()->getWorldTransformation(),
                                (Box*) shape2,
                                shape1->getProxy()->getWorldTransformation(),
                                (Cylinder*) shape1);
                    } // endcase shape2 is a box
                    case Shape::SHAPE_TYPE_SPHERE: {
                        return getSphereMeshIntersection(
                                    (Sphere*) shape2,
                                    shape1->getMesh()->getTriangles(),
                                    shape1->getProxy());
                    } // endcase shape2 is a sphere
                    case Shape::SHAPE_TYPE_WEDGE: {
                        CylinderIntersector CylinderIntersector;
                        return CylinderIntersector.getIntersection(
                                shape2->getProxy()->getWorldTransformation(),
                                (Wedge*) shape2,
                                shape1->getProxy()->getWorldTransformation(),
                                (Cylinder*) shape1);
                    } // endcase shape2 is a wedge
                    case Shape::SHAPE_TYPE_CYLINDER: {
                        CylinderIntersector CylinderIntersector;
                        return CylinderIntersector.getIntersection(
                                shape1->getProxy()->getWorldTransformation(),
                                (Cylinder*) shape1,
                                shape2->getProxy()->getWorldTransformation(),
                                (Cylinder*) shape2);
                    } // endcase shape2 is a cylinder
                    default:
                        break;
                } // end switch on shape2
            } // end shape1 is a cylinder
            case Shape::SHAPE_TYPE_CONE: {
                switch (shape2->getShapeType()) {
                    case Shape::SHAPE_TYPE_CONE: {
                        ConeIntersector ConeIntersector;
                        return ConeIntersector.getIntersection(
                                shape1->getProxy()->getWorldTransformation(),
                                (Cone*) shape1,
                                shape2->getProxy()->getWorldTransformation(),
                                (Cone*) shape2);
                    } // endcase shape2 is a cone
                    case Shape::SHAPE_TYPE_SPHERE: {
                        return getSphereMeshIntersection(
                                    (Sphere*) shape2,
                                    shape1->getMesh()->getTriangles(),
                                    shape1->getProxy());
                    } // endcase shape2 is a sphere
                    case Shape::SHAPE_TYPE_BOX: {
                        ConeIntersector ConeIntersector;
                        return ConeIntersector.getIntersection(
                                shape2->getProxy()->getWorldTransformation(),
                                (Box*) shape2,
                                shape1->getProxy()->getWorldTransformation(),
                                (Cone*) shape1);
                    } // endcase shape2 is a box
                    case Shape::SHAPE_TYPE_WEDGE: {
                        ConeIntersector ConeIntersector;
                        return ConeIntersector.getIntersection(
                                shape2->getProxy()->getWorldTransformation(),
                                (Wedge*) shape2,
                                shape1->getProxy()->getWorldTransformation(),
                                (Cone*) shape1);
                    } // endcase shape2 is a wedge
                    default:
                        break;
                } // end switch on shape2
            } // end shape1 is a cone
            case Shape::SHAPE_TYPE_WEDGE: {
                switch (shape2->getShapeType()) {
                    case Shape::SHAPE_TYPE_SPHERE: {
                        //Use Mesh of the wedge
                        return getSphereMeshIntersection(
                                    (Sphere*) shape2,
                                    shape1->getMesh()->getTriangles(),
                                    shape1->getProxy());
                    }
                    case Shape::SHAPE_TYPE_CYLINDER: {
                        CylinderIntersector CylinderIntersector;
                        return CylinderIntersector.getIntersection(
                                shape1->getProxy()->getWorldTransformation(),
                                (Wedge*) shape1,
                                shape2->getProxy()->getWorldTransformation(),
                                (Cylinder*) shape2);
                    } // endcase shape2 is a cylinder
                    case Shape::SHAPE_TYPE_CONE: {
                        ConeIntersector ConeIntersector;
                        return ConeIntersector.getIntersection(
                                shape1->getProxy()->getWorldTransformation(),
                                (Wedge*) shape1,
                                shape2->getProxy()->getWorldTransformation(),
                                (Cone*) shape2);
                    } // endcase shape2 is a cone
                    case Shape::SHAPE_TYPE_BOX: {
                        WedgeIntersector WedgeIntersector;
                        return WedgeIntersector.getIntersection(
                                (Box*) shape2,
                                (Wedge*) shape1);
                    } // endcase shape2 is a box
                    case Shape::SHAPE_TYPE_WEDGE: {
                        WedgeIntersector WedgeIntersector;
                        return WedgeIntersector.getIntersection(
                                (Wedge*) shape1,
                                (Wedge*) shape2);
                    } // endcase shape2 is a wedge
                    default:
                        break;
                }//end switch on shapetype 2
            } // end shape1 is a wedge
            case Shape::SHAPE_TYPE_MESH: {
                switch (shape2->getShapeType()) {
                    case Shape::SHAPE_TYPE_SPHERE: {
                        return getSphereMeshIntersection(
                                    (Sphere*) shape2,
                                    shape1->getMesh()->getTriangles(),
                                    shape1->getProxy());
                    }//endcase shape 2 is a sphere
                    default:
                        break;
                }//end switch on shapetype 2
            } //endcase shape1 is a mesh
            case Shape::SHAPE_TYPE_MESH_PART: {
                switch (shape2->getShapeType()) {
                    case Shape::SHAPE_TYPE_SPHERE: {
                        return getSphereMeshIntersection(
                                (Sphere*) shape2,
                                shape1->getMesh()->getTriangles(),
                                shape1->getProxy());
                    }//endcase shape 2 is a sphere
                    default:
                        break;
                }//end switch on shapetype 2
            } //endcase shape1 is a meshpart
            default:
                break;
        }//end switch on shapetype1

        //if we reach this point, the shape combination is not covered by
        //specialized algorithms. use getMesh and mesh-collision code by default
        return getMeshMeshIntersection( shape1->getMesh(), shape2->getMesh());
        
        //Ticket #340: activate this instead of the last line to test
        //             the alternative mesh-mesh collison
        //return getTriangleSetIntersection(  shape1->getMesh()->getTriangles(), shape1->getProxy(),
        //                                    shape2->getMesh()->getTriangles(), shape2->getProxy());
    }


    //-- private methods to calculate the intersection of the given types-----//

    std::list<CollisionInfo> NarrowPhase::getSphereMeshIntersection(
            const Sphere* sphere,
            const std::vector<Triangle*> triangles,
            Proxy* meshProxy) const {
        std::list<CollisionInfo> result;

        SphereTriangleIntersector intersector;
        //iterate through all triangles and collect results
        for (std::vector<Triangle*>::const_iterator triIter = triangles.begin();
                                triIter!=triangles.end();
                                triIter++) {
            CollisionInfo* coll = intersector.getIntersection(sphere, *triIter, meshProxy);
            if (coll) {
                result.push_back(*coll);
            }
        }
        return result;
/*
        Vector3 spherePos = currentStateSphere.getPosition();

        switch (mStrategySphereMesh) {
            case NP_STRATEGY_SLOWEST_EXTENSIVE: // TODO: Implementing of the
            case NP_STRATEGY_SLOW_BACKTRACKING: // more detailed detections
            case NP_STRATEGY_FAST_CONSIDER_SHAPE:
                       break;
            case NP_STRATEGY_FASTEST_SLOPPY:
*/
        return result;
    }

    //-------------Box-Sphere-collision-----------------------//
    std::list<CollisionInfo> NarrowPhase::getBoxSphereIntersection(
                const Matrix& prevStateBox, const Matrix& currentStateBox,
                const Box* box,
                const Matrix& prevStateSphere, const Matrix& currentStateSphere,
                const Sphere* sphere) const {
    if (mStrategyBoxSphere == NP_STRATEGY_FASTEST_SLOPPY) {
        return getBvOverlapCollision(
                box->getProxy()->getBvHierarchyNode()->getBoundingVolume(),
                sphere->getProxy()->getBvHierarchyNode()->getBoundingVolume(),
                box->getProxy(), sphere->getProxy());
    }

    //STRATEGY_FAST_CONSIDER_SHAPE is implemented here.
    BoxSphereIntersector intersector;
    return intersector.getIntersection(currentStateBox, box,
                                currentStateSphere, sphere,
                                !prevStateSphere.isEqual(currentStateSphere));

    //GJ: algorithms with backtracking etc won't be done for now
    }

    //-------------Mesh-Mesh collisions-----------------------------//
    /*!
     *\brief calculates the collision between two Meshes
     * 
     */
    std::list<CollisionInfo> NarrowPhase::getMeshMeshIntersection(
                    const Mesh* mesh1, const Mesh* mesh2) const {
        MeshMeshIntersector intersector;
        return intersector.getIntersection(mesh1,mesh2);
    }
    
    /*!
     * \brief calculates collisions beetween several triangles
     * 
     * DEVELOPER NOTE: This should be called from the deformable middlephase
     *                 algorithms directly. If the algorithm works good, it 
     *                 might replace \ref NarrowPhase::getMeshMeshIntersection
     *                 as well.
     * 
     * generates a CollisionInfo for each vertex that is behind each Triangle 
     * of the other proxy
     * \param triangles1 the triangles of Proxy* \p proxy1
     * \param triangles2 the triangles of Proxy* \p proxy1
     */
    std::list<CollisionInfo> NarrowPhase::getTriangleSetIntersection(
            PotentialCollidingSets& sets) const {
//#define USE_PLANE_ALGO 
#ifdef USE_PLANE_ALGO
        MeshMeshIntersector intersector;
        return intersector.getIntersection( sets.setOne, sets.proxy1,
#else
        //debug() << dc_funcinfo;
        //debug() << "triangles1.size=="<<triangles1.size()<<", triangles2.size=="<<triangles2.size();
        
        //first, check vertices of triangles1 against planes defined by triangles2
        //then vice versa
        //we will decide afterwards which of the collisions we need to return
        
        // open hull triangles can collide only from one side.
        // The triangle face normals always define "front" and behind
        std::list<CollisionInfo> part1Result = testVerticesAgainstPlanes(   sets.setOne, sets.proxy1,
                                                                            sets.setTwo, sets.proxy2);
        std::list<CollisionInfo> part2Result = testVerticesAgainstPlanes(sets.setTwo, sets.proxy2,
                                                                         sets.setOne, sets.proxy1);
        /*
        //some debug output: content of the two part-results
        for (std::list<CollisionInfo>::iterator iter = part1Result.begin(); iter != part1Result.end(); iter++) {
            debug() << "in part1: "<< (proxy1Open?"vertex of solid":"vertex of open hull") <<" collision with normal " << (*iter).normal;
        }
        for (std::list<CollisionInfo>::iterator iter = part2Result.begin(); iter != part2Result.end(); iter++) {
            debug() << "in part2: "<< (proxy2Open?"vertex of solid":"vertex of open hull") <<" collision with normal " << (*iter).normal;        }
        */
        
        if (part1Result.empty()) {
            return part2Result;
        } 
        if (part2Result.empty()) {
            return part1Result;
        }
        
        //Both results are not empty at this point.
        part1Result.splice(part1Result.end(), part2Result);
        return part1Result;
    /*
        //experimental: If both parts have results,
        //just pick the one with the lower total penetration depth
        
        real part1Total = 0;
        real part2Total = 0;
        
        for (std::list<CollisionInfo>::iterator iter = part1Result.begin(); iter != part1Result.end(); iter++) {
            part1Total += (*iter).penetrationDepth;
        }
        for (std::list<CollisionInfo>::iterator iter = part2Result.begin(); iter != part2Result.end(); iter++) {
            part2Total += (*iter).penetrationDepth;
        }
        
        if (part1Total < part2Total) {
            //debug() << "two nonempty parts, returning part1";
            return part1Result;
        } else {
            //debug() << "two nonempty parts, returning part2";
            return part2Result;
        }
*/
#endif
    }

    /*!
     * \brief finds the vertices that are behind the Triangles in planeTriangles
     *
     * Behind is always defined by the triangle face normals.
     */
    std::list<CollisionInfo> NarrowPhase::testVerticesAgainstPlanes(
            const std::vector<Triangle*>& vertexTriangles, Proxy* vertexProxy,
            const std::vector<Triangle*>& planeTriangles, Proxy* planeProxy) const {
        std::list<CollisionInfo> backCollisions;
        
        //Pseudocode:
        //calculate Planes defined by triangles in \p planeTriangles
        //find the vertices of the triangles in \p vertexTriangles
        //for each vertex: check if it is behind every plane
        // if it is, generate a CollisionInfo for that point. Collision normal
        // will be defined by the plane that has the minimal distance to the point 
        
        //calculate Planes defined by the penetrated triangles
        std::vector<Plane> penetratedPlanes;
        penetratedPlanes.reserve(planeTriangles.size());
        
        for (std::vector<Triangle*>::const_iterator tri2Iter = planeTriangles.begin();
                                                    tri2Iter!=planeTriangles.end();
                                                    tri2Iter++){
            Triangle* tri = *tri2Iter;
            penetratedPlanes.push_back(Plane(   tri->getWorldCoordinatesNormalVector(),
                                                tri->getVertices()[0]->getWorldPosition(),
                                                false) //normal vector is already normalized
                                       );
            tri->calculateEdgeData(); //we will need it later on several times
            
        }
        
        //calculate all Vertices of the triangles from penetrating Proxy
        //these are the potentially penetrating vertices
        std::set<Vertex*> penetratingVertices;
        for (std::vector<Triangle*>::const_iterator tri1Iter = vertexTriangles.begin();
                                                            tri1Iter!=vertexTriangles.end();
                                                            tri1Iter++){
            Triangle* tri = *tri1Iter;
            penetratingVertices.insert(tri->getVertices()[0]);
            penetratingVertices.insert(tri->getVertices()[1]);
            penetratingVertices.insert(tri->getVertices()[2]);
        }
        
        real totalPenetrationDepthBack = 0;
        
        //now check each vertex of the set against each plane
        
        //the algorithm behaviour depends on 
        // - wether the planeProxy is convex or not

        //convex:
        //  check if the vertex is behind each plane.
        //  of all planes, remember the one with distance negative but closest
        //  to zero
        //concave: 
        //  of all triangles this vertex is behind, find the one with highest
        //  penetration depth. The vertex does not need to be behind all planes
        
        bool convex = (planeProxy->getProxyType() & PROXYTYPE_CONVEX);
        //if (!convex) debug() << "testing against a set of nonconvex planes!";
        
        for (std::set<Vertex*>::iterator vertexIter = penetratingVertices.begin();
                                         vertexIter!=penetratingVertices.end();
                                         vertexIter++){
            Vertex* testVertex = *vertexIter;
            Vector3 vertexPosition = testVertex->getWorldPosition();
            
            //debug() << "Testing vertex at " << vertexPosition << " against all planes";

            int backCollisionPlaneIndex = -1;
            real bestBackCollisionDistance = -10000;

            
            //debug() << "numeric_min = " << maxNegativeDistance;
            //debug() << "numeric_max = " << minPositiveDistance;
            for (unsigned int planeIndex=0; planeIndex< penetratedPlanes.size(); planeIndex++){
                //debug() << "test against plane"<<planeIndex<<":";
                //penetratedPlanes[planeIndex].debugPlane();
                Plane& plane = penetratedPlanes[planeIndex];
                real distanceToPlane = plane.calculateDistance(vertexPosition);
                //debug() << "distance to plane " << planeIndex <<" is " << distanceToPlane;
                if (distanceToPlane < 0){
                    //we have a potential collision with this plane
                    
                    //Exact solution: collision with triangle only if projection
                    //of point to plane is in the triangle
                    
                    Vector3 projection = vertexPosition + plane.getNormal() * distanceToPlane;
                    
                    // Check if projection is in triangle
                    bool projectionInTriangle = planeTriangles[planeIndex]->containsPoint(projection, true, true);
                    
                    //but we still need to check if this is the one we want
                    if (        projectionInTriangle 
                            &&  distanceToPlane > bestBackCollisionDistance) {
                            //debug()<< "found new candidate plane";
                        bestBackCollisionDistance = distanceToPlane;
                        backCollisionPlaneIndex = planeIndex;
                    }
                } else { //distance > 0, break loop for convex hulls
                    if (convex) {
                        //found vertex outside of a plane=>
                        //no collision with the plane, so the vertex
                        //does not penetrate the shape formed by all planes
                        //set to remembered index to -1 so we
                        //won't generate a collision for this vertex
                        
                        //bdebug() << "found noncolliding plane, cancelling test";
                        backCollisionPlaneIndex       = -1;
                        break; //we can stop testing against the other planes here.
                    }
                }
            }
            if (backCollisionPlaneIndex != -1) {
                //generate CollisionInfo
                CollisionInfo coll;
                coll.collisionPoint = vertexPosition;
                coll.normal = penetratedPlanes[backCollisionPlaneIndex].getNormal();
                coll.penetrationDepth = bestBackCollisionDistance * -1;
                coll.penetratedTriangle = planeTriangles[backCollisionPlaneIndex];
                coll.penetratingVertex = testVertex;
                
                coll.penetratingProxy = vertexProxy;
                coll.penetratedProxy = planeProxy;
                backCollisions.push_back(coll);
                totalPenetrationDepthBack += coll.penetrationDepth;
                
            }
            
        }
        
        
        //we have checked only the back sides of the triangles
        //  (OR the totalPenetrationDepth of the back-collisions is lower
        //  (OR concave and majority of the vertices are in front of the planes)
        return backCollisions;
        

    }
    /*!
     *\brief returns a collision based on boundingvolume overlap
     * DEVELOPER NOTICE: this is intended to be used as return value for
     *                   most ShapeType combinations at accuracy level 1.
     */
    std::list<CollisionInfo> NarrowPhase::getBvOverlapCollision(
                    const BoundingVolume* vol1,
                    const BoundingVolume* vol2,
                    Proxy* proxy1, Proxy* proxy2) const {
        //collision point: center of AABB overlap area
        //collision normal: vector from overlap.Min to overlap.Max
        //penetration depth: lenght of connection : overlap.min to overlap.max
        Vector3 vol1Min = vol1->getSurroundingAabbMin();
        Vector3 vol2Min = vol2->getSurroundingAabbMin();
        Vector3 vol1Max = vol1->getSurroundingAabbMax();
        Vector3 vol2Max = vol2->getSurroundingAabbMax();

        //overlap Minimum is componentwise maximum of the 2 min vectors
        Vector3 overlapMin( std::max(vol1Min.getX(), vol2Min.getX()),
                            std::max(vol1Min.getY(), vol2Min.getY()),
                            std::max(vol1Min.getZ(), vol2Min.getZ()));
        //overlap Maximum is componentwise minimum of the 2 max vectors
        Vector3 overlapMax( std::min(vol1Max.getX(), vol2Max.getX()),
                            std::min(vol1Max.getY(), vol2Max.getY()),
                            std::min(vol1Max.getZ(), vol2Max.getZ()));

        Vector3 overlapConnection = overlapMax - overlapMin;

        CollisionInfo coll;
        coll.collisionPoint = overlapMin + overlapConnection/2;
        coll.normal = overlapConnection;
        coll.penetrationDepth = overlapConnection.lengthApproxBabylonian(1);
        coll.penetratingProxy = proxy1;
        coll.penetratedProxy = proxy2;

        std::list<CollisionInfo> result;
        result.push_back(coll);
        return result;
    }


    /* Box-box collisions
    --------------------------------------------------------------------------*/

    /*!
     * \brief Calculates the collision between two boxes.
     */
    std::list<CollisionInfo> NarrowPhase::getBoxBoxIntersection(
            const Matrix& prevState1, const Matrix& currentState1,
            Box* box1, const Matrix& prevState2,
            const Matrix& currentState2, Box* box2) const {

        //DCOLLIDE_UNUSED(prevState1);
        //DCOLLIDE_UNUSED(prevState2);

        std::list<CollisionInfo> results;


        switch (mStrategyBoxBox) {

        /* Level 1: "Fastest & sloppy"
         *
         * Really fast (no more than ~10 simple operations (+, -, *, /).
         * Maybe not even based on shapetype but on bounding-boxes. No sqrt
         * operations, no projections, no iteration over single triangles.
         * Based only on the current State of the shapes.
         */
        case NP_STRATEGY_FASTEST_SLOPPY: {
            Proxy* proxy1 = box1->getProxy();
            Proxy* proxy2 = box2->getProxy();

            return getBvOverlapCollision(
                proxy1->getBvHierarchyNode()->getBoundingVolume(),
                proxy2->getBvHierarchyNode()->getBoundingVolume(),
                proxy1, proxy2
            );
        }

        // TODO: Implement strategies and remove these two lines
        case NP_STRATEGY_SLOW_BACKTRACKING:

        /* Level 2: "Fast, consider shape"
         *
         * Probably still based only on current state of the shapes. Iteration
         * on triangles allowed if not overdone.
         */
        case NP_STRATEGY_FAST_CONSIDER_SHAPE: {

            BoxBoxIntersector boxBoxIntersector;
            return boxBoxIntersector.getIntersectionFast(
                    currentState1,box1,currentState2,box2);
            //return boxBoxIntersector.getIntersectionAccurate(box1,prevState1,
            //        box2,prevState2);

        }

        /* Level 3: "Slower w\backtracking"
         *
         * Take previous state into account. Simple trajectories/backtracking
         * and therefore more accurate determination of collision point(s) and
         * penetration depth.
         */
        //case NP_STRATEGY_SLOW_BACKTRACKING:
        //    return results;

        /* Level 4: "Slowest & extensive"
         *
         * Anything allowed. Make sure to calculate everything as accurate as
         * possible, (almost) no matter how long it takes. Find the point in
         * time between the previous and the current state where the shapes
         * collide (without penetration), and determine the collision point(s).
         * Then, step to the current state and see how far the shapes penetrated
         * each other with reference to these points.
         */
        case NP_STRATEGY_SLOWEST_EXTENSIVE: {
            BoxBoxIntersector boxBoxIntersector;
            return boxBoxIntersector.getIntersectionAccurate(box1,prevState1,
                    box2,prevState2);
        }

        default: // hugh hugh! (((:~{>
            break;
        }

        return results;
    }

}

/*
 * vim: et sw=4 ts=4
 */
