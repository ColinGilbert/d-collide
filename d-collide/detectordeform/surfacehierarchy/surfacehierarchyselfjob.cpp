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

#include "surfacehierarchyalgorithm.h"

#include "proxy.h"
#include "world.h"
#include "shapes/shape.h"
#include "real.h"
#include "debugstream.h"
#include "exceptions/exception.h"
#include "deformablebvhnode.h"
#include "shapes/mesh/vertex.h"

#include "narrowphase/narrowphase.h"
#include "narrowphase/narrowphasestrategy.h"
#include "collisioninfo.h"

#include "detectordeform/surfacehierarchy/surfacehierarchyselfjob.h"
#include "detectordeform/surfacehierarchy/surfacehierarchyproxydata.h"

#include "detectordeform/surfacehierarchy/normalcone.h"

#include <collisionpair.h>


#define SURFACEHIERARCHY_DEBUG 1

namespace dcollide {

    SurfaceHierarchySelfJob::SurfaceHierarchySelfJob(SurfaceHierarchyProxyData* proxyData1) : DetectorDeformJob(RESULT_TYPE_TRIANGLESETS),
                                                                        mProxyData1(proxyData1) {
    }


    SurfaceHierarchySelfJob::~SurfaceHierarchySelfJob() {
    }


    void SurfaceHierarchySelfJob::run() {

        DeformableBvhNode* rootsphere = mProxyData1->mDeformableNode;

        rootsphere->updateTopDown();

        if (rootsphere->getNormalCone()->isSelfCollisionPossible()) {

            typeOneTest(rootsphere);

        }

    }

    /*!
     * the type one test just calls for each child another type one test
     * if this child has potential selfcollisionsin a recursiv form and
     * as a second thing it calls a type two test for every possible pair of these children
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchySelfJob::typeOneTest(BvhNode* node) {

        std::list<BvhNode*> children = node->getChildren();

        if (children.empty()) {

                //triangletest needed! nxn innerhalb dieses blatt-BVs
//                std::vector<Triangle*>* triangles_temp_vector = new std::vector<Triangle*>;
//                const std::list<Triangle*> triangles_temp_list = p1_work->getBoundingVolume()->getEnclosedTriangles();
//xxx
//                PotentialCollidingSets set = PotentialCollidingSets(  *mTriangles_1, mProxyData1->getProxy(),
//                              *mTriangles_2, mProxyData1->getProxy());

        } else {

            //go one step deeper in the hierarchy
            for (std::list<BvhNode*>::iterator child = children.begin();
                                                child != children.end();
                                                ++child) {

                if (((DeformableBvhNode*)(*child))->getNormalCone()->isSelfCollisionPossible()) {
                    typeOneTest(*child);
                }

            }

            //jeder mit jedem und paare >180° werden miteinander getestet (schnittest) paare <180° nicht
            while (children.size() > 1) {

                BvhNode* pivot = children.front();

                for (std::list<BvhNode*>::iterator child = ++(children.begin());
                                                    child != children.end();
                                                    ++child) {
                    //intersect pivot and child
                    typeTwoTest(pivot, *child);
                }

                children.pop_front();

            }

        }

    }

    /*!
     * the type two test checks if the given pair of BoundingSpheres is neighbouring eachother
     * if this is the case is checks further if the sum of their normalcones indicates possible
     * selfcollisions and if so it calls calculateMinimalCollisions for this pair.
     * if it indicates no selfcollisions, no further tests are done.
     * are these two BoundingSpheres no neighbours, calculateMinimalCollisions is called for them anyway.
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchySelfJob::typeTwoTest(BvhNode* neighbourOne, BvhNode* neighbourTwo) {

        //check if the two are neighbours
        if (((DeformableBvhNode*)neighbourOne)->isNeighbourOf((DeformableBvhNode*)neighbourTwo)) {
            // if they are: add their NCs and look if >180° if yes intersect all their children
            NormalCone nc = (*(((DeformableBvhNode*)neighbourOne)->getNormalCone()) + *(((DeformableBvhNode*)neighbourTwo)->getNormalCone()));
            if (nc.isSelfCollisionPossible()) {
                calculateMinimalCollisions(neighbourOne, neighbourTwo);
            }
        } else {
            // if they are no neighbours do nothing ?!?!?!?!?!
            calculateMinimalCollisions(neighbourOne, neighbourTwo);
        }

    }

//    void SurfaceHierarchySelfJob::typeThreeTest(BvhNode* separateOne, BvhNode* separateTwo) {
//
//        ;
//
//    }


    /*!
     * calculates the minimal set of colliding lowest level BoundingSpheres and
     * creates narrowphasejobs containing two triangle-sets
     * (for collisions between two deformable objects)
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchySelfJob::calculateMinimalCollisions(const BvhNode* p1_work, const BvhNode* p2_work) {

        if ( (!p1_work->getChildren().empty()) && (!p2_work->getChildren().empty()) ) {

            std::list<BvhNode*> p1_neu;
            std::list<BvhNode*> p2_neu;

            for (std::list<BvhNode*>::const_iterator iter_children_1 = p1_work->getChildren().begin();
                                                        iter_children_1 != p1_work->getChildren().end();
                                                        ++iter_children_1) {
                // p2_work collides with iter_children_1 ???
                if (intersectSpheres(p2_work, *iter_children_1)) {
                    p1_neu.push_back(*iter_children_1);
                }
            }

            for (std::list<BvhNode*>::const_iterator iter_children_2 = p2_work->getChildren().begin();
                      iter_children_2 != p2_work->getChildren().end();
                      ++iter_children_2) {
                // p1_work collides with iter_children_2 ???
                if (intersectSpheres(p1_work, *iter_children_2)) {
                    p2_neu.push_back(*iter_children_2);
                }
            }

            for (std::list<BvhNode*>::iterator iter_p1 = p1_neu.begin();
                                                iter_p1 != p1_neu.end();
                                                ++iter_p1) {

                for (std::list<BvhNode*>::iterator iter_p2 = p2_neu.begin();
                                                    iter_p2 != p2_neu.end();
                                                    ++iter_p2) {
                    if (intersectSpheres(*iter_p1, *iter_p2)) {
                        calculateMinimalCollisions(*iter_p1, *iter_p2);
                    }
                }
            }

        } else if ( ( !p1_work->getChildren().empty() ) && ( p2_work->getChildren().empty() ) ) {

            std::list<BvhNode*> p1_neu_p2;

            for (std::list<BvhNode*>::const_iterator iter_children_1_1 = p1_work->getChildren().begin();
                                                        iter_children_1_1 != p1_work->getChildren().end();
                                                        ++iter_children_1_1) {
                // p2_work collides with iter_children_1 ???
                if (intersectSpheres(p2_work, *iter_children_1_1)) {
                    p1_neu_p2.push_back(*iter_children_1_1);
                }
            }

            for (std::list<BvhNode*>::iterator iter_p1_p2 = p1_neu_p2.begin();
                                                iter_p1_p2 != p1_neu_p2.end();
                                                ++iter_p1_p2) {
                if (intersectSpheres(p2_work, *iter_p1_p2)) {
                    calculateMinimalCollisions(*iter_p1_p2, p2_work);
                }
            }

        } else if ( ( !p2_work->getChildren().empty() ) && ( p1_work->getChildren().empty() ) ) {

            std::list<BvhNode*> p2_neu_p1;

            for (std::list<BvhNode*>::const_iterator iter_children_2_2 = p2_work->getChildren().begin();
                                                        iter_children_2_2 != p2_work->getChildren().end();
                                                        ++iter_children_2_2) {
                // p1_work collides with iter_children_2 ???
                if (intersectSpheres(p1_work, *iter_children_2_2)) {
                    p2_neu_p1.push_back(*iter_children_2_2);
                }
            }

            for (std::list<BvhNode*>::iterator iter_p2_p1 = p2_neu_p1.begin();
                                                iter_p2_p1 != p2_neu_p1.end();
                                                ++iter_p2_p1) {
                if (intersectSpheres(p1_work, *iter_p2_p1)) {
                    calculateMinimalCollisions(p1_work, *iter_p2_p1);
                }
            }

        } else {

            //don't test neighbouring lowest level spheres against each other, because they have neighbouring triangles.
            if (((DeformableBvhNode*)p1_work)->isNeighbourOf((DeformableBvhNode*)p2_work)) {
                return;
            }

            std::vector<Triangle*>* mTriangles_1 = new std::vector<Triangle*>;
            std::vector<Triangle*>* mTriangles_2 = new std::vector<Triangle*>;

            //find the triangles with the same way: sphere1 contains triangles2 && sphere2 contains triangles1
            // -> put meshparts into the middlephase results from calculateMinimalCollisions()
            const std::list<Triangle*> p1_triangles = p1_work->getBoundingVolume()->getEnclosedTriangles();
            const std::list<Triangle*> p2_triangles = p2_work->getBoundingVolume()->getEnclosedTriangles();

            for (std::list<Triangle*>::const_iterator iter_tri1 = p1_triangles.begin();
                 iter_tri1 != p1_triangles.end();
                 ++iter_tri1) {

                if (intersectSphereTriangle(p2_work, *iter_tri1)) {
                //narrowphase: entweder den vector von triangles dieses aufrufes in die narrowphase pipen, oder erst sammeln für das gesamte Proxy-Paar
                    mTriangles_1->push_back(*iter_tri1);
                }
            }

            if (mTriangles_1->size() == 0) {
                return;
            }

            for (std::list<Triangle*>::const_iterator iter_tri2 = p2_triangles.begin();
                                                        iter_tri2 != p2_triangles.end();
                                                        ++iter_tri2) {

                if (intersectSphereTriangle(p1_work, *iter_tri2)) {
                    mTriangles_2->push_back(*iter_tri2);
                }
            }

            if (mTriangles_2->size() == 0) {
                return;
            }

            PotentialCollidingSets set = PotentialCollidingSets(  *mTriangles_1, mProxyData1->getProxy(), *mTriangles_2, mProxyData1->getProxy());

            addResults(set);

        }

    }

    /*!
     * tests a BoundingSphere with a triangle (exact test)
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    inline bool SurfaceHierarchySelfJob::intersectSphereTriangle(const BvhNode* sphere, Triangle* triangle) {

        Vector3 center = ((BoundingSphere*) sphere->getBoundingVolume())->getCenterVector();
        real square_radius = ((BoundingSphere*) sphere->getBoundingVolume())->getRadius();
        square_radius *= square_radius;
        dcollide::array<Vertex*,3> vertices = triangle->getVertices();
        real square_distance;

        for (dcollide::array<Vertex*,3>::iterator iter_vertex = vertices.begin();
                                                    iter_vertex != vertices.end();
                                                    ++iter_vertex) {
            square_distance = (center.getX() - (*iter_vertex)->getPosition().getX())*(center.getX() - (*iter_vertex)->getPosition().getX());
            square_distance += (center.getY() - (*iter_vertex)->getPosition().getY())*(center.getY() - (*iter_vertex)->getPosition().getY());
            square_distance += (center.getZ() - (*iter_vertex)->getPosition().getZ())*(center.getZ() - (*iter_vertex)->getPosition().getZ());

            if (square_distance < square_radius) {
                return 1;
            }
        }

        return 0;

    }

    /*!
     * check if the two BoundingVolumes in the two BvhNodes intersects
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    inline bool SurfaceHierarchySelfJob::intersectSpheres(const BvhNode* sphere1, const BvhNode* sphere2) {

        Vector3 center1 = ((BoundingSphere*) sphere1->getBoundingVolume())->getCenterVector();
        Vector3 center2 = ((BoundingSphere*) sphere2->getBoundingVolume())->getCenterVector();

        real square_distance = (center1.getX() - center2.getX())*(center1.getX() - center2.getX());
        square_distance += (center1.getY() - center2.getY())*(center1.getY() - center2.getY());
        square_distance += (center1.getZ() - center2.getZ())*(center1.getZ() - center2.getZ());

        real square_radius = (((BoundingSphere*) sphere1->getBoundingVolume())->getRadius() + ((BoundingSphere*) sphere2->getBoundingVolume())->getRadius())
                            *(((BoundingSphere*) sphere1->getBoundingVolume())->getRadius() + ((BoundingSphere*) sphere2->getBoundingVolume())->getRadius());

        if (square_distance > square_radius) {
            return 0;
        } else {
            return 1;
        }

    }


}

/*
 * vim: et sw=4 ts=4
 */
