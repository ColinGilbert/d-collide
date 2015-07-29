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
#include "meshmeshintersector.h"

#include "triangleintersector.h"
#include "dcollide-global.h"

#include "../math/plane.h"
#include "../math/vector.h"
#include "../math/matrix.h"
#include "../shapes/mesh.h"
#include "../proxy.h"
#include "../collisioninfo.h"
#include "bvhnode.h"
#include "shapes/mesh/triangle.h"
#include "boundingvolumes/boundingvolume.h"
#include "datatypes/set.h"

#include "debugstream.h"

namespace dcollide {
    MeshMeshIntersector::MeshMeshIntersector(NarrowPhaseStrategy strategy) {
        mStrategy = strategy;
    }

    MeshMeshIntersector::~MeshMeshIntersector() {
    }

    /*!
     * \brief calculates a list of collision infos for two meshes or meshparts.
     * This function performs a complete triangle-triangle intersection
     * calculation, determines the regression plane from the resulting collision
     * points, assigns the halfspaces defined by the plane to the proxies,
     * calculates the penetration depths in the not-assigned halfspaces and
     * finally reduces the number of collision points to at most six.
     * \param mesh1 mesh or meshpart belonging to the first proxy. if \p swapped
     * is set to true this is forced to be the penetrating proxy's mesh.
     * \param mesh2 mesh or meshpart belonging to the second proxy.
     * \param swapped for internal use: if the first proxy can't be the
     * penetrating one they are swapped.
     * (e.g. if mesh1 belongs to a fixed proxy).
     * standard value: FALSE
     * Set this TRUE to force \p mesh1 to be the penetrating proxy's mesh.
     * \return list of at most 6 collision info, including usefull
     * penetration-normal and -depth
     */
    std::list<CollisionInfo> MeshMeshIntersector::getIntersection(
            const Mesh* mesh1, const Mesh* mesh2, bool swapped) const {
        // make sure mesh1 belongs to the penetrating proxy
        if (!swapped &&
                ((mesh1->getProxy()->getMoveFlags() & Proxy::MOVEFLAG_UNMOVED)
                || (mesh1->getProxy()->getProxyType() & PROXYTYPE_FIXED))) {
            return getIntersection(mesh2, mesh1, true);
        }

        // prepare a list to store the results
        std::list<CollisionInfo> result;

        // calculate triangle intersections using triangle intersector
        calculateTriangleIntersections(mesh1->getTriangles(),
                mesh2->getTriangles(), result, true);

        // calculate and set usefull penetration normal and depth
        unifyCollisionInfo(result);

        // reduce the number of triangle intersection points to at most 6
#ifndef MESHMESHPLANEDEBUG
        thinOutCollisionInfo(result);
#endif

        return result;
    }

    /*!
     * \brief calculates a list of collision infos for a list of bounding
     * volume collisions.
     * This function performs triangle-triangle intersection calculation for
     * triangles inside the colliding bounding volumes. After that the same
     * post-processing is perfomed as in
     * \ref MeshMeshIntersector::getIntersection(
     * const Mesh* mesh1, const Mesh* mesh2, bool swapped) .
     * \param bvCollisions list of bounding volume collisions to be processed.
     * \return list of at most 6 collision info, including usefull
     * penetration-normal and -depth
     */
    std::list<CollisionInfo> MeshMeshIntersector::getIntersection(
            std::list<BoundingVolumeCollision>& bvCollisions) const {
        // prepare a list to store the results
        std::list<CollisionInfo> result;

        // run through the colliding bounding volumes
        for (std::list<BoundingVolumeCollision>::iterator
                iterBv = bvCollisions.begin();
                iterBv != bvCollisions.end();
                ++iterBv) {
            // prepare a temporary result list for 2 colliding bounding volumes
            std::list<CollisionInfo> tempResult;

            // calculate triangle intersections for the enclosed triangles of
            // the colliding bounding volumes using triangle intersector
            calculateBvIntersections((*iterBv), tempResult);

            // calculate and set usefull penetration normal and depth
            unifyCollisionInfo(tempResult);

            // reduce the number of triangle intersection points to at most 6
#ifndef MESHMESHPLANEDEBUG
            thinOutCollisionInfo(tempResult);
#endif

            // append the temporary results to the result list
            result.splice(result.end(), tempResult);
        }
        return result;
    }

    /*!
     * \brief calculates a list of collision infos for given triangles
     * This function performs a full triangle-triangle intersection test for
     * the given triangles. After that the same post-processing is perfomed
     * as in \ref MeshMeshIntersector::getIntersection(
     * const Mesh* mesh1, const Mesh* mesh2, bool swapped) .
     * \param triangles1 first vector of pointers to triangles to be processed.
     * \param triangles2 second vector of pointers to triangles to be processed.
     * \return list of at most 6 collision info, including usefull
     * penetration-normal and -depth
     */
    std::list<CollisionInfo> MeshMeshIntersector::getIntersection(
            std::vector<Triangle*>& triangles1, Proxy* proxy1,
            std::vector<Triangle*>& triangles2, Proxy* proxy2) const {
        // prepare a list to store the results
        std::list<CollisionInfo> result;

        // calculate triangle intersections using triangle intersector
        // info about possible colliding triangles is assumed to be not present
        calculateTriangleIntersections(triangles1, triangles2, result, true);

        // calculate and set usefull penetration normal and depth
        if ((PROXYTYPE_RIGID & proxy1->getProxyType()) &&
                (PROXYTYPE_RIGID & proxy2->getProxyType())) {
            unifyCollisionInfo(result);
        } else {
            debug() << " calculating penetrating vertices";
            unsigned long pointCount = 0;
            Plane regressionPlane = calculateRegressionPlane(result, &pointCount);

            //we will create new collisions for penetrating vertices,
            //get rid of the triangle collisions
#ifndef MESHMESHPLANEDEBUG
            result.clear();
#else
            for (std::list<CollisionInfo>::iterator iter = result.begin();
                    iter != result.end(); ++iter) {
                (*iter).penetratingProxy = proxy1;
                (*iter).penetratedProxy = proxy2;
                (*iter).normal = regressionPlane.getNormal();
                (*iter).penetrationDepth = 0;
            }
#endif

            if (pointCount <= COLLISION_POINTS_THRESHOLD) {
                debug() << "not enough points ("<< pointCount <<") to calculate a useful regression plane";
                result.clear();
                return result;
            }

            Vector3 center1;
            std::set<Vertex*> vertices1 = collectVertices(triangles1, center1);

            Vector3 center2;
            std::set<Vertex*> vertices2 = collectVertices(triangles2, center2);

            //overwrite centerpoint if not selfcollision
            if (proxy1 != proxy2) {
                center1 = proxy1->getBoundingVolumeCenter();
                center2 = proxy2->getBoundingVolumeCenter();
            }

            //TODO: calculate forceDividing properly
            bool forceDividing = false;
            bool isSameSide = !assignHalfSpaces(regressionPlane, center1, center2,
                    forceDividing);
            calculatePenetratingVertices(vertices1, regressionPlane,
                    true, result, proxy1, proxy2);
            calculatePenetratingVertices(vertices2, regressionPlane,
                    isSameSide, result, proxy1, proxy2);
        }

        // reduce the number of triangle intersection points to at most 6
#ifndef MESHMESHPLANEDEBUG
        thinOutCollisionInfo(result);
#endif

        return result;
    }

    /*!
     * \brief post-procession of collision points.
     * Determines the regression plane from the given collision points,
     * assigns the halfspaces defined by the plane to the proxies and
     * calculates the penetration depths in the not-assigned halfspaces.
     * \param result list of collision info as containers for the points.
     * ATTENTION: original penetration -normals and -depths in each collision
     * info will be discarded and replaced by new values that are equal for
     * all collision info.
     */
    void MeshMeshIntersector::unifyCollisionInfo(
            std::list<CollisionInfo>& result) const {
        // if there are too few points the plane is a bad approximation
        // we better wait for more points than make a mistake
        // below is a faster check for sloppy strategy
        if (false && !(mStrategy == NP_STRATEGY_FASTEST_SLOPPY) &&
                isTooFewOrTooClose(result)) {
            result.clear();
            return;
        }

        // calculate regression plane for the intersection points
        unsigned long pointCount = 0;
        Plane regressionPlane = calculateRegressionPlane(result, &pointCount);

        // if there are too few points the plane is a bad approximation
        // we better wait for more points than make a mistake
        if ((pointCount <= COLLISION_POINTS_THRESHOLD) &&
                (true || (mStrategy == NP_STRATEGY_FASTEST_SLOPPY))) {
            result.clear();
            return;
        }

        // get the meshes for calculation of the penetration depth
        const Mesh* mesh1 =
                (*result.begin()).penetratingProxy->getShape()->getMesh();
        const Mesh* mesh2 =
                (*result.begin()).penetratedProxy->getShape()->getMesh();

        Vector3 center1 = mesh1->getProxy()->getBoundingVolumeCenter();
        Vector3 center2 = mesh2->getProxy()->getBoundingVolumeCenter();
        // determine if both proxies are convex, if so they can't be
        // non-colliding on the same side of the plane and will be divided by it
        // this leads to a higher tolerance for deep penetration
        // e.g. due to fast movement
        bool forceDividing = (PROXYTYPE_CONVEX &
                mesh1->getProxy()->getProxyType() &
                mesh2->getProxy()->getProxyType());

        // assign the half spaces defined by the regression plane to the proxies
        // afterwards the normal will point towards mesh1
        bool isSameSide = !assignHalfSpaces(regressionPlane, center1, center2,
                forceDividing);


        // determine deepest points in the half space assigned to other mesh
        real depth1 = calculateHighestDepth(mesh1, regressionPlane,
                true, result);
        real depth2 = calculateHighestDepth(mesh2, regressionPlane,
                isSameSide, result);
        real penetrationDepth = std::min(depth1, depth2);

        // adjust the collision infos to:
        // mesh1 belongs to the penetrating proxy,
        // the normal of the regression plane and
        // depth of the deepest points
        for (std::list<CollisionInfo>::iterator iter = result.begin();
                iter != result.end(); ++iter) {
            (*iter).penetratingProxy = mesh1->getProxy();
            (*iter).penetratedProxy = mesh2->getProxy();
            (*iter).normal = regressionPlane.getNormal();
            (*iter).penetrationDepth = penetrationDepth;
        }
    }

    /*!
     * \brief post-procession of collision points.
     * Reduces the number of collision infos to at most 6, by discarding all but
     * those collision infos, that are spanning an AABB aorund the collision
     * points.
     * \param result list of collision info to be reduced.
     * ATTENTION: original data is discarded and therefore lost.
     */
    void MeshMeshIntersector::thinOutCollisionInfo(
            std::list<CollisionInfo>& result) const {

        if (result.empty()) {
            return;
        }

        // set up the initialization and do a special step of the loop
        // count the collision infos to abort if there are already few of them
        long count = 1;

        // define the iterator which runs through the collision infos
        std::list<CollisionInfo>::iterator iter = result.begin();

        // the first collision point is maximal and minimal in X, Y and Z so far
        Vector3 min = (*iter).collisionPoint;
        Vector3 max = (*iter).collisionPoint;
        CollisionInfo* minXCollInfo = &(*iter);
        CollisionInfo* maxXCollInfo = &(*iter);
        CollisionInfo* minYCollInfo = &(*iter);
        CollisionInfo* maxYCollInfo = &(*iter);
        CollisionInfo* minZCollInfo = &(*iter);
        CollisionInfo* maxZCollInfo = &(*iter);
        // end the special treatment of the first step of the loop
        ++iter;
        // run through the other collision infos
        for (; iter != result.end(); ++iter) {
            // update the minimal point and the corresponding info along X-axis
            if (min.getX() > (*iter).collisionPoint.getX()) {
                min.setX((*iter).collisionPoint.getX());
                minXCollInfo = &(*iter);
            }
            // update for Y-axis
            if (min.getY() > (*iter).collisionPoint.getY()) {
                min.setY((*iter).collisionPoint.getY());
                minYCollInfo = &(*iter);
            }
            // update for Z-axis
            if (min.getZ() > (*iter).collisionPoint.getZ()) {
                min.setZ((*iter).collisionPoint.getZ());
                minZCollInfo = &(*iter);
            }
            // update for maximal and X-axis
            if (max.getX() < (*iter).collisionPoint.getX()) {
                max.setX((*iter).collisionPoint.getX());
                maxXCollInfo = &(*iter);
            }
            // update for Y-axis
            if (max.getY() < (*iter).collisionPoint.getY()) {
                max.setY((*iter).collisionPoint.getY());
                maxYCollInfo = &(*iter);
            }
            // update for Z-axis
            if (max.getZ() < (*iter).collisionPoint.getZ()) {
                max.setZ((*iter).collisionPoint.getZ());
                maxZCollInfo = &(*iter);
            }
            // count the collision infos
            ++count;
        }
        // abort if there already are only a few collision infos
        if (count < 4) return;
        // from here on count the maximal/minimal collision points
        count = 0;
        // define a set to avoid duplicates of the collision infos
        Set<CollisionInfo*> tempSet(13);
        // if the minimal point along X-axis can be inserted to the set
        // add it to the front of the result list and count it
        if (tempSet.insert(minXCollInfo)) {
            result.push_front(CollisionInfo(*minXCollInfo));
            ++count;
        }
        // add for Y-axis
        if (tempSet.insert(minYCollInfo)) {
            result.push_front(CollisionInfo(*minYCollInfo));
            ++count;
        }
        // add for Z-axis
        if (tempSet.insert(minZCollInfo)) {
            result.push_front(CollisionInfo(*minZCollInfo));
            ++count;
        }
        // add for maximal and X-axis
        if (tempSet.insert(maxXCollInfo)) {
            result.push_front(CollisionInfo(*maxXCollInfo));
            ++count;
        }
        // add for Y-axis
        if (tempSet.insert(maxYCollInfo)) {
            result.push_front(CollisionInfo(*maxYCollInfo));
            ++count;
        }
        // add for Z-axis
        if (tempSet.insert(maxZCollInfo)) {
            result.push_front(CollisionInfo(*maxZCollInfo));
            ++count;
        }
        // set an iterator behind the new added collision infos
        iter = result.begin();
        for (int i = 0; i < count; ++i) {
            ++iter;
        }
        // remove all other collision infos from the result list
        result.erase(iter, result.end());
    }

    /*!
     * \brief Internal function for calculation of triangle intersection.
     * Uses the triangle intersector for triangle-triangle test between
     * given triangles.
     * \param triangles1 first vector of triangles to test
     * \param triangles2 second vector of triangles to test
     * \param result reference to a list of collision info to add the results to
     * \param fullCheck set to true to perform a full check, all triangles from
     * \p triangles1 against every single from \p triangles2 . If set to false
     * (DEFAULT) only the first, second, ... from both vectors are tested
     * against each other. In this case make sure the vectors have the same size
     */
    void MeshMeshIntersector::calculateTriangleIntersections(
            const std::vector<Triangle*>& triangles1,
            const std::vector<Triangle*>& triangles2,
            std::list<CollisionInfo>& result, bool fullCheck) const {
        // use an appropriate intersector
        TriangleIntersector intersector;
        TriangleIntersection t;

        std::vector<Triangle*>::const_iterator iter2 = triangles2.begin();
        // run through all pairs of triangles
        for (std::vector<Triangle*>::const_iterator
                iter1 = triangles1.begin();
                iter1 != triangles1.end();
                ++iter1) {
            if (fullCheck) {
                iter2 = triangles2.begin();
            }
            do {
                // compute the intersection of the current triangle pair
                intersector.computeTriangleIntersection(*iter1, *iter2, &t);
                // check if they intersect
                if (t.collision) {
                    // check if they are non-coplanar
                    if (!t.coplanar) {
                        // create a collisioninfo for the starting point of the
                        // intersection line
                        CollisionInfo cInfo;
                        cInfo.collisionPoint = t.intersectionStart;
                        cInfo.penetratingProxy = triangles1[0]->
                                getVertices()[0]->
                                getContainingMesh()->getProxy();
                        cInfo.penetratedProxy = triangles2[0]->
                                getVertices()[0]->
                                getContainingMesh()->getProxy();

                        // normal and depth are computed and added later
                        cInfo.penetratingTriangle = *iter1;
                        cInfo.penetratedTriangle  = *iter2;
                        result.push_back(cInfo);
                        // check if it is an intersection line or just one point
                        if (!t.intersectionEnd.isEqual(t.intersectionStart)) {
                            // create a collision info for the ending point
                            cInfo.collisionPoint = t.intersectionEnd;
                            cInfo.penetratingProxy = triangles1[0]->
                                    getVertices()[0]->
                                    getContainingMesh()->getProxy();
                            cInfo.penetratedProxy = triangles2[0]->
                                    getVertices()[0]->
                                    getContainingMesh()->getProxy();
                            // normal and depth are computed and added later
                            cInfo.penetratingTriangle = *iter1;
                            cInfo.penetratedTriangle  = *iter2;
                            result.push_back(cInfo);
                        }
                    } else {
                        // TODO: handle coplanar triangles
                    }
                }
                // this is a end controlled loop so it executes at least once
                ++iter2;
            } while (iter2 != triangles2.end() && fullCheck);
        }
    }

    /*!
     * \brief Internal function for calculation of triangle intersection.
     * Uses the triangle intersector for a full triangle-triangle test between
     * the triangles inside of two bounding volumes.
     * \param bvCollision info from the middle phase on two bounding volumes
     * \param result reference to a list of collision info to add the results to
     */
    void MeshMeshIntersector::calculateBvIntersections(
            BoundingVolumeCollision& bvCollision,
            std::list<CollisionInfo>& result) const {
        // get the proxies
        Proxy* proxy1 = bvCollision.node1->getProxy();
        Proxy* proxy2 = bvCollision.node2->getProxy();
        // get the triangles
        std::list<Triangle*>& triangles1 = const_cast<std::list<Triangle*>&>(
                bvCollision.node1->getBoundingVolume()->getEnclosedTriangles());
        std::list<Triangle*>& triangles2 = const_cast<std::list<Triangle*>&>(
                bvCollision.node2->getBoundingVolume()->getEnclosedTriangles());
        // if proxy1 can't be the penetrating proxy ...
        if ((proxy1->getMoveFlags() & Proxy::MOVEFLAG_UNMOVED)
                || (proxy1->getProxyType() & PROXYTYPE_FIXED)) {
            // ... swap the proxies ...
            proxy1 = bvCollision.node2->getProxy();
            proxy2 = bvCollision.node1->getProxy();
            // ... and the triangles accordingly
            triangles1 = bvCollision.node2->
                    getBoundingVolume()->getEnclosedTriangles();
            triangles2 = bvCollision.node1->
                    getBoundingVolume()->getEnclosedTriangles();
        }
        // use an appropriate intersector
        TriangleIntersector intersector;
        TriangleIntersection t;
        // run through all pairs of triangles
        for (std::list<Triangle*>::const_iterator
                iter1 = triangles1.begin();
                iter1 != triangles1.end();
                ++iter1) {
            for (std::list<Triangle*>::const_iterator
                    iter2 = triangles2.begin();
                    iter2 != triangles2.end();
                    ++iter2) {
                // compute the intersection of the current triangle pair
                intersector.computeTriangleIntersection(*iter1, *iter2, &t);
                // check if they intersect
                if (t.collision) {
                // check if they are non-coplanar
                    if (!t.coplanar) {
                        // create a collisioninfo for the starting point
                        // of the intersection line
                        CollisionInfo cInfo;
                        cInfo.collisionPoint = t.intersectionStart;
                        cInfo.penetratingProxy = proxy1;
                        cInfo.penetratedProxy = proxy2;
                        // normal and depth are computed and added later
                        cInfo.penetratingTriangle = *iter1;
                        cInfo.penetratedTriangle  = *iter2;
                        result.push_back(cInfo);
                        // check if it is an intersection line or a point
                        if (!t.intersectionEnd.isEqual(t.intersectionStart)) {
                            // create a collision info for the ending point
                            cInfo.collisionPoint = t.intersectionEnd;
                            cInfo.penetratingProxy = proxy1;
                            cInfo.penetratedProxy = proxy2;
                            // normal and depth are computed and added later
                            cInfo.penetratingTriangle = *iter1;
                            cInfo.penetratedTriangle  = *iter2;
                            result.push_back(cInfo);
                        }
                    } else {
                    // TODO: handle coplanar triangles
                    }
                }
            }
        }
    }

    bool MeshMeshIntersector::isTooFewOrTooClose(
            std::list<CollisionInfo>& pointList, real minDistance) const {
#if 0
        if (pointList.empty()) {
            return true;
        }
        std::list<CollisionInfo>::iterator iter = pointList.begin();
        CollisionInfo& first = *iter;
        unsigned long count = 1;
        std::list<CollisionInfo*> further;
        for (++iter; iter != pointList.end(); ++iter) {
            if (!first.collisionPoint.isEqual((*iter).collisionPoint,
                    minDistance)) {
                bool isDuplicate = false;
                for (std::list<CollisionInfo*>::iterator iter2 =
                        further.begin(); iter2 != further.end(); ++iter2) {
                    if ((*iter2)->collisionPoint.
                            isEqual((*iter).collisionPoint, minDistance)) {
                        isDuplicate = true;
                        break;
                    }
                }
                if (isDuplicate) {
                    continue;
                }
                further.push_back(&(*iter));
                ++count;
                if (count > COLLISION_POINTS_THRESHOLD) {
                    return false;
                }
            }
        }
        return true;
#else
        if (pointList.empty()) {
            return true;
        }
        std::list<CollisionInfo>::iterator iter = pointList.begin();
        CollisionInfo& first = *iter;
        CollisionInfo* second;
        for (++iter; iter != pointList.end(); ++iter) {
            if (!first.collisionPoint.isEqual((*iter).collisionPoint,
                    minDistance)) {
                second = &(*iter);
                ++iter;
                break;
            }
        }
        for (; iter != pointList.end(); ++iter) {
            if (!first.collisionPoint.isEqual((*iter).collisionPoint,
                            minDistance) &&
                    !second->collisionPoint.isEqual((*iter).collisionPoint,
                            minDistance)) {
                return false;
            }
        }
        return true;
#endif
    }

    /*!
     * \brief Internal function for calculation of a plane dividing the proxies.
     * Uses the statistical basement of the regression plane to approximate
     * the intersecting area of the proxies by a plane.
     * \param result reference to a list of collision info to use as input
     * \param count OPTIONAL: Pointer to get additional output for free
     * \param average OPTIONAL: Pointer to get additional output for free
     * \param averageXX OPTIONAL: Pointer to get additional output for free
     * \param averageXY OPTIONAL: Pointer to get additional output for free
     * \param averageYY OPTIONAL: Pointer to get additional output for free
     * \param averageYZ OPTIONAL: Pointer to get additional output for free
     * \param averageXZ OPTIONAL: Pointer to get additional output for free
     */
    Plane MeshMeshIntersector::calculateRegressionPlane(
            std::list<CollisionInfo>& result, unsigned long* count,
            Vector3* average, real* averageXX, real* averageXY, real* averageYY,
            real* averageYZ, real* averageXZ) const {
        // if they are not provided by caller define the following values
        unsigned long tempCount = 0;
        Vector3 tempAverage = Vector3(0, 0, 0);
        real tempAverageXX = 0;
        real tempAverageXY = 0;
        real tempAverageYY = 0;
        real tempAverageYZ = 0;
        real tempAverageXZ = 0;
        if (!count) count = &tempCount;
        if (!average) average = &tempAverage;
        if (!averageXX) averageXX = &tempAverageXX;
        if (!averageXY) averageXY = &tempAverageXY;
        if (!averageYY) averageYY = &tempAverageYY;
        if (!averageYZ) averageYZ = &tempAverageYZ;
        if (!averageXZ) averageXZ = &tempAverageXZ;
        // run through all collision points updating the averages for each point
        for (std::list<CollisionInfo>::iterator iter = result.begin();
                iter != result.end(); ++iter) {
            // get the current collision point
            Vector3 point = (*iter).collisionPoint;
            // update all averages using the current point
            *average = ((*average * (*count)) + point) / ((*count) + 1);
            *averageXX = ((*averageXX * (*count))
                            + (point.getX() * point.getX()))
                        / ((*count) + 1);
            *averageXY = ((*averageXY * (*count))
                            + (point.getX() * point.getY()))
                        / ((*count) + 1);
            *averageYY = ((*averageYY * (*count))
                            + (point.getY() * point.getY()))
                        / ((*count) + 1);
            *averageYZ = ((*averageYZ * (*count))
                            + (point.getY() * point.getZ()))
                        / ((*count) + 1);
            *averageXZ = ((*averageXZ * (*count))
                            + (point.getX() * point.getZ()))
                        / ((*count) + 1);
            ++(*count);
            // skip every other point for sloppy strategy
            if ((*count > COLLISION_POINTS_THRESHOLD) &&
                    (mStrategy == NP_STRATEGY_FASTEST_SLOPPY) &&
                    (iter != result.end())) {
                ++iter;
            }
        }
        // use these averages to calculate the coefficients a, b and c
        // of the plane: z = a + bx + cy
        // calculate some temporary variables
        // note that the names have no meaning but represent a recurring pattern
        // this will be used as dividers and divisors
        real tempXX = *averageXX - (average->getX() * average->getX());
        real tempXY = *averageXY - (average->getX() * average->getY());
        real tempYY = *averageYY - (average->getY() * average->getY());
        real tempYZ = *averageYZ - (average->getY() * average->getZ());
        real tempXZ = *averageXZ - (average->getX() * average->getZ());
        // this are the fractions using the temporaries above
        real tempXYXX = tempXY / tempXX;
        real tempXZXX = tempXZ / tempXX;
        real tempYYXY = tempYY / tempXY;
        real tempYZXY = tempYZ / tempXY;
        // heyah, now it's the coefficients' turn. they are nice, promise. not.
        real c = (tempYZXY - tempXZXX) / (tempYYXY - tempXYXX);
        real b = tempXZXX - (tempXYXX * c);
        real a = average->getZ() -
                ((average->getX() * b) + (average->getY() * c));

        // convert the coefficients representation into normal representation
        // and then into an object: z = a + bx + cy    ->    ux + vy + wz = d
        // TODO: handle special cases like b or c = 0 or at least have a thought
        Vector3 normal = Vector3(b, c, -1);
        normal.normalize();
        Vector3 pointOnPlane = Vector3(0, 0, a);
        Plane resultPlane = Plane(normal, pointOnPlane);

#ifdef MESHMESHPLANEDEBUG
        real x, y, z;
        long step = 25;
        long number = 11;
        long offsetx = -100;
        long offsety = -100;
        Vector3 linePoint = Vector3();
        Vector3 lineVector = Vector3(0, 0, 1);
        Vector3 intersection = Vector3();
        for (long i = 0; i < number * number; i += 1) {
            CollisionInfo cInfo;
            if (i % 2 == 0) {
                x = ((i * step) % (step * number)) + offsetx;
                y = (((long)((i * step) / (step * number))) * step) + offsety;
                z =  a + (b * x) + (c * y);
                cInfo.collisionPoint = Vector3(x, y, z);
            }
            if (i % 2 == 1) {
                x = ((i * step) % (step * number)) + offsetx;
                y = (((long)((i * step) / (step * number))) * step) + offsety;
                linePoint.setX(x);
                linePoint.setY(y);
                resultPlane.intersectLine(linePoint, lineVector, &intersection);
                z = intersection.getZ();
                cInfo.collisionPoint = Vector3(x, y, z);
            }
            result.push_back(cInfo);
        }
#endif
        return resultPlane;
    }

    /*!
     * \brief Internal function to determine sides of a plane to proxies.
     * Uses the statistical basement of the regression plane to approximate
     * the intersecting area of the proxies by a plane.
     * \param dividingPlane the plane to decide for
     * \param mesh1 afterwards the normal of the plane will point towards this
     * \param mesh2 needed to determine if both meshes are on the same side
     * \param forceDividing forces both meshes to be on different sides, should
     * be true for convex meshes, since it improves handling of deep penetration
     * \returns true if the assignment is unambiguous meaning the meshes are
     * divided by the plane, always true for forceDividing = true
     */
    bool MeshMeshIntersector::assignHalfSpaces(Plane& dividingPlane,
            const Vector3& center1, const Vector3& center2, bool forceDividing) const {
        Vector3 dividingPlaneNormal = dividingPlane.getNormal();
        if (forceDividing) {
            // check for the more significant point if it is in front or behind
            if (fabs(dividingPlane.calculateDistance(center1)) >
                fabs(dividingPlane.calculateDistance(center2))) {
                if (dividingPlane.isBehindPlane(center1)) {
                    // set the normal pointing towards mesh1
                    dividingPlaneNormal = dividingPlaneNormal * -1;
                    dividingPlane = Plane(dividingPlaneNormal,
                            -dividingPlane.getDistanceFromOrigin());
                }
            } else {
                if (dividingPlane.isInFrontOfPlane(center2)) {
                    // set the normal pointing towards mesh1
                    dividingPlaneNormal = dividingPlaneNormal * -1;
                    dividingPlane = Plane(dividingPlaneNormal,
                            -dividingPlane.getDistanceFromOrigin());
                }
            }
        } else {
            // check for the center of mesh1 if it is in front or behind
            if (dividingPlane.isBehindPlane(center1)) {
                // set the normal pointing towards mesh1
                dividingPlaneNormal = dividingPlaneNormal * -1;
                dividingPlane = Plane(dividingPlaneNormal,
                                      -dividingPlane.getDistanceFromOrigin());
            }
            // return wether the assignment is unambiguous. if this is false
            // either one mesh is concave and the other one is on its inside
            // or they are penetrating each other deeply due to fast movement
            return dividingPlane.isBehindPlane(center2);
        }
        // if they are forced to be on different sides it's always unambiguous
        return true;
    }

    /*!
     * \brief Internal function to calculate the penetration depth.
     * Performs a full search over all vertices of the mesh for the vertex with
     * the highest distance on the not assigned side of the plane. Calls other
     * methods if appropriate.
     * \param mesh mesh to check the vertices
     * \param plane plane to measure the distance from
     * \param inFront denotes if the normal of the plane points towards the mesh
     * \param result needed for debug output or starting point for faster
     * calculation
     * \returns highest distance of a vertex on the not assigned side of plane
     */
    real MeshMeshIntersector::calculateHighestDepth(const Mesh* mesh,
            Plane& plane, bool inFront, std::list<CollisionInfo>& result) const{
        // use a faster method if appropriate
        if ((PROXYTYPE_CONVEX & mesh->getProxy()->getProxyType()) ||
                (mStrategy == NP_STRATEGY_FASTEST_SLOPPY)) {
            return calculateDepthFast(mesh, plane, inFront, (*result.begin()));
        }

#ifdef MESHMESHPLANEDEBUG
        CollisionInfo cInfo;
#endif
        // define variables for the current depth and the maximal depth so far
        real depth = 0;
        real maxDepth = 0;

        // get the vertices of the given mesh
        const std::vector<Vertex*>& vertices = mesh->getVertices();

        // run through the vertices
        for (std::vector<Vertex*>::const_iterator
                iter = vertices.begin();
                iter != vertices.end();
                ++iter) {
            // calculate the depth for this vertex
            depth = plane.calculateDistance((*iter)->getWorldPosition());
            // adjust the depth to the assigned half space of the mesh
            if (inFront) {
                depth = depth * -1;
            }
            // update the maximal depth if necessary
            if (depth > maxDepth) {
                maxDepth = depth;
#ifdef MESHMESHPLANEDEBUG
                cInfo.collisionPoint = (*iter)->getWorldPosition();
#endif
            }
        }
#ifdef MESHMESHPLANEDEBUG
        result.push_back(cInfo);
#endif
        return maxDepth;
    }

    /*!
     * \brief Internal function to calculate the penetration depth.
     * Performs a simplex-like search over vertices of the mesh for the vertex
     * with the highest distance on the not assigned side of the plane. This is
     * appropriate for convex meshes/meshparts or a fast approximation.
     * \param mesh mesh to check the vertices
     * \param plane plane to measure the distance from
     * \param inFront denotes if the normal of the plane points towards the mesh
     * \param startingPoint vertex of the mesh to start the search from
     * \returns locally highest distance of a vertex on the not assigned side
     */
    real MeshMeshIntersector::calculateDepthFast(const Mesh* mesh, Plane& plane,
            bool inFront, CollisionInfo& startingPoint) const {
        // get a vertex as starting point in the mesh
        Vertex* currentVertex;
        if (mesh->getProxy() == startingPoint.penetratingProxy) {
            currentVertex = startingPoint.penetratingTriangle->
                    getVertices()[0];
        } else {
            currentVertex = startingPoint.penetratedTriangle->
                    getVertices()[0];
        }

        // define variables for temporary depth and the maximal depths so far
        real depth = plane.calculateDistance(
                currentVertex->getWorldPosition());
        real maxDepth = depth;
        real maxNeighbourDepth = depth;

        // define a pointer and a bool for use in the iteration
        Vertex* maxNeighbour;

        // traverse the mesh along the normal of the plane
        while (true) {
            // get the vertices surrounding current vertex
            const std::list<Vertex*>& neighbours =
                    currentVertex->getAdjacentVertices();

            // calculate the depths of the surrounding vertices
            for (std::list<Vertex*>::const_iterator
                    iter = neighbours.begin();
                    iter != neighbours.end(); ++iter) {
                // calculate the depth for the current neighbour
                depth = plane.calculateDistance((*iter)->getWorldPosition());
                // adjust the depth to the assigned half space of the mesh
                if (inFront) {
                    depth = depth * -1;
                }
                // update the maximal depth in the neighbourhood if necessary
                if (depth > maxNeighbourDepth) {
                    maxNeighbourDepth = depth;
                    maxNeighbour = (*iter);
                }
            }

            // use the neighbour with highest depth as new current vertex
            if (maxNeighbourDepth > maxDepth) {
                maxDepth = maxNeighbourDepth;
                currentVertex = maxNeighbour;
            } else {
                break;
            }
        }
        return maxDepth;
    }

    /*!
     * \brief Internal function to calculate the penetration depth.
     * Calculates the depth independently for each vertex of the mesh
     * on the not assigned side. This is appropriate for deformable
     * meshes/meshparts or a vertex-precise calculation.
     * \param mesh mesh to check the vertices
     * \param plane plane to measure the distance from
     * \param inFront denotes if the normal of the plane points towards the mesh
     * \param result list to add collision info for each vertex to
     * \param penetratingProxy info to add in the collision info
     * \param penetratedProxy info to add in the collision info
     */
    void MeshMeshIntersector::calculatePenetratingVertices(
            const std::set<Vertex*>& vertices,
            Plane& plane, bool inFront, std::list<CollisionInfo>& result,
            Proxy* penetratingProxy, Proxy* penetratedProxy) const {
        // define variables for the current depth
        real depth = 0;

        // run through the vertices
        for (std::set<Vertex*>::const_iterator
                iter = vertices.begin();
                iter != vertices.end();
                ++iter) {
            // calculate the depth for this vertex
            Vector3 point = (*iter)->getWorldPosition();
            depth = plane.calculateDistance(point);
            // adjust the depth to the assigned half space of the mesh
            if (inFront) {
                depth = depth * -1;
            }
            // create a new collision info if necessary
            if (depth > 0) {
                CollisionInfo cInfo;
                cInfo.collisionPoint = point;
                cInfo.normal = plane.getNormal();
                cInfo.penetrationDepth = depth;
                cInfo.penetratingProxy = penetratingProxy;
                cInfo.penetratedProxy = penetratedProxy;
                cInfo.penetratingVertex = *iter;
                result.push_back(cInfo);
            }
        }
    }

    std::set<Vertex*> MeshMeshIntersector::collectVertices(
            std::vector<Triangle*> triangles, Vector3& centerPoint) const{
        std::set<Vertex*> vertices;

        for (std::vector<Triangle*>::const_iterator tri1Iter = triangles.begin();
                                                            tri1Iter!=triangles.end();
                                                            tri1Iter++) {
            Triangle* tri = *tri1Iter;
            std::pair<std::set<Vertex*>::iterator, bool> insertResult;

            insertResult = vertices.insert(tri->getVertices()[0]);
            if (insertResult.second) {
                centerPoint += tri->getVertices()[0]->getWorldPosition();
            }
            insertResult = vertices.insert(tri->getVertices()[1]);
            if (insertResult.second) {
                centerPoint += tri->getVertices()[1]->getWorldPosition();
            }
            insertResult = vertices.insert(tri->getVertices()[2]);
            if (insertResult.second) {
                centerPoint += tri->getVertices()[2]->getWorldPosition();
            }

        }
        centerPoint = centerPoint / vertices.size();

        return vertices;
    }
}

/*
 * vim: et sw=4 ts=4
 */
