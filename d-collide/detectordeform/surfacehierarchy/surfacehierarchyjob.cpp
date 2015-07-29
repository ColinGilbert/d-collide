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
#include "shapes/mesh.h"
#include "real.h"
#include "debugstream.h"
#include "exceptions/exception.h"
#include "deformablebvhnode.h"
#include "shapes/mesh/vertex.h"
#include "boundingvolumes/aabb.h"

#include "narrowphase/narrowphase.h"
#include "narrowphase/narrowphasestrategy.h"
#include "collisioninfo.h"

#include "detectordeform/surfacehierarchy/surfacehierarchyjob.h"
#include "detectordeform/surfacehierarchy/surfacehierarchyproxydata.h"

#include <collisionpair.h>


#define SURFACEHIERARCHY_DEBUG 1

namespace dcollide {

    SurfaceHierarchyJob::SurfaceHierarchyJob(Proxy* proxy1, Proxy* proxy2, int algoIndex) : DetectorDeformJob(RESULT_TYPE_TRIANGLESETS),
                         mProxy1(proxy1), mProxy2(proxy2), mAlgoIndex(algoIndex) {

    }


    SurfaceHierarchyJob::~SurfaceHierarchyJob() {
    }


    void SurfaceHierarchyJob::run() {

        checkCollisionType(mProxy1, mProxy2);

    }

// DetectorDeformProxyData* getDetectorDeformProxyData(unsigned int index) const;

    /*!
     * checks for the right collisiontype
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    void SurfaceHierarchyJob::checkCollisionType(Proxy* proxy1, Proxy* proxy2) {

        //checks if both Proxy are deformable and get the deformablebvhnode
        if ((proxy1->getProxyType() & PROXYTYPE_DEFORMABLE) && (proxy2->getProxyType() & PROXYTYPE_DEFORMABLE)) {

            calculateMinimalCollisions(proxy1->getDetectorDeformProxyData(mAlgoIndex)->getDeformableBvhNode(),
                                        proxy2->getDetectorDeformProxyData(mAlgoIndex)->getDeformableBvhNode(),
                                        proxy1,
                                        proxy2);

        // only the first proxy is deformable
        } else if (proxy1->getProxyType() & PROXYTYPE_DEFORMABLE) {

            if (proxy2->getShape()) {

                if (proxy2->getShape()->getShapeType() == Shape::SHAPE_TYPE_MESH) {

                    calculateMinimalCollisionsWithRigid(proxy1->getDetectorDeformProxyData(mAlgoIndex)->getDeformableBvhNode(),
                                                        proxy2->getBvHierarchyNode(),
                                                        proxy1,
                                                        proxy2);

                } else {

                    calculateMinimalCollisionsWithShape(proxy1->getDetectorDeformProxyData(mAlgoIndex)->getDeformableBvhNode(),
                                                        proxy2->getBvHierarchyNode(),
                                                        proxy1,
                                                        proxy2);
                }

            } else {

                // get the child-proxies here, check if they have shapes abd handle collisions with them
                const std::list<Proxy*> children = proxy2->getChildProxies();

                for (std::list<Proxy*>::const_iterator child = children.begin();
                                                        child != children.end();
                                                        ++child) {

                    checkCollisionType(proxy1, *child);
                }
            }

        // only the second proxy is deformable
        } else if (proxy2->getProxyType() & PROXYTYPE_DEFORMABLE) {

            if (proxy1->getShape()) {

                if (proxy1->getShape()->getShapeType() == Shape::SHAPE_TYPE_MESH) {

                    calculateMinimalCollisionsWithRigid(proxy2->getDetectorDeformProxyData(mAlgoIndex)->getDeformableBvhNode(),
                                                        proxy1->getBvHierarchyNode(),
                                                        proxy2,
                                                        proxy1);

                } else {

                    calculateMinimalCollisionsWithShape(proxy2->getDetectorDeformProxyData(mAlgoIndex)->getDeformableBvhNode(),
                                                        proxy1->getBvHierarchyNode(),
                                                        proxy2,
                                                        proxy1);
                }

            } else {

                // get the child-proxies here, check if they have shapes abd handle collisions with them
                const std::list<Proxy*> children = proxy1->getChildProxies();

                for (std::list<Proxy*>::const_iterator child = children.begin();
                                                        child != children.end();
                                                        ++child) {

                    checkCollisionType(*child, proxy2);
                }
            }
        }
    }

    /*!
     * calculates the minimal set of colliding lowest level BoundingSpheres and
     * creates narrowphasejobs containing two triangle-sets
     * (for collisions between two deformable objects)
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    // rekursiv
    // evtl. den if und else teil in eigene methoden auslagern um in der rekursion speicherplatz zu sparen
    void SurfaceHierarchyJob::calculateMinimalCollisions(const BvhNode* p1_work, const BvhNode* p2_work, Proxy* prox1, Proxy* prox2) {

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
                        calculateMinimalCollisions(*iter_p1, *iter_p2, prox1, prox2);
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
                    calculateMinimalCollisions(*iter_p1_p2, p2_work, prox1, prox2);
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
                    calculateMinimalCollisions(p1_work, *iter_p2_p1, prox1, prox2);
                }
            }

        } else {

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

            //Proxy* proxy1 = p1_work->getProxy()->getToplevelProxy();
            //Proxy* proxy2 = p2_work->getProxy()->getToplevelProxy();
            for (std::list<Triangle*>::const_iterator iter_tri2 = p2_triangles.begin();
                                                        iter_tri2 != p2_triangles.end();
                                                        ++iter_tri2) {

                if (intersectSphereTriangle(p1_work, *iter_tri2)) {
                //narrowphase: entweder den vector von triangles dieses aufrufes in die narrowphase pipen, oder erst sammeln für das gesamte Proxy-Paar
                    mTriangles_2->push_back(*iter_tri2);
                }
            }

            if (mTriangles_2->size() == 0) {
                return;
            }

            PotentialCollidingSets set = PotentialCollidingSets(  *mTriangles_1, prox1, *mTriangles_2, prox2);

            //debug(10) << "set.mTirangles_1.size()" << set.setOne.size();
            //debug(10) << "set.mTirangles_2.size()" << set.setTwo.size();

            addResults(set);

        }

    }

    /*!
     * calculates the minimal set of colliding lowest level BoundingVolumes and
     * creates narrowphasejobs containing two triangle-sets
     * (for collisions between one deformable and one rigid mesh object)
     *
     */
    void SurfaceHierarchyJob::calculateMinimalCollisionsWithRigid(const BvhNode* p1_work, const BvhNode* p2_work, Proxy* p1, Proxy* p2) {

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
                        calculateMinimalCollisionsWithRigid(*iter_p1, *iter_p2, p1, p2);
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
                    calculateMinimalCollisionsWithRigid(*iter_p1_p2, p2_work, p1, p2);
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
                    calculateMinimalCollisionsWithRigid(p1_work, *iter_p2_p1, p1, p2);
                }
            }

        } else {

            std::vector<Triangle*>* mTriangles_1 = new std::vector<Triangle*>;
            std::vector<Triangle*>* mTriangles_2 = new std::vector<Triangle*>;

            //find the triangles with the same way: sphere1 contains triangles2 && sphere2 contains triangles1
            // -> put meshparts into the middlephase results from calculateMinimalCollisions()
            const std::list<Triangle*> p1_triangles = p1_work->getBoundingVolume()->getEnclosedTriangles();
            std::vector<Triangle*> p2_triangles = p2_work->getShape()->getMesh()->getTriangles();

            for (std::list<Triangle*>::const_iterator iter_tri1 = p1_triangles.begin();
                                                        iter_tri1 != p1_triangles.end();
                                                        ++iter_tri1) {

                if (intersectBoundingVolumeTriangle(p2_work, *iter_tri1)) {
                    //narrowphase: entweder den vector von triangles dieses aufrufes in die narrowphase pipen, oder erst sammeln für das gesamte Proxy-Paar
                    mTriangles_1->push_back(*iter_tri1);
                }
            }

            if (mTriangles_1->size() == 0) {
                return;
            }

            //Proxy* proxy1 = p1_work->getProxy()->getToplevelProxy();
            //Proxy* proxy2 = p2_work->getProxy()->getToplevelProxy();
            for (std::vector<Triangle*>::const_iterator iter_tri2 = p2_triangles.begin();
                                                        iter_tri2 != p2_triangles.end();
                                                        ++iter_tri2) {

                if (intersectBoundingVolumeTriangle(p1_work, *iter_tri2)) {
                    //narrowphase: entweder den vector von triangles dieses aufrufes in die narrowphase pipen, oder erst sammeln für das gesamte Proxy-Paar
                    mTriangles_2->push_back(*iter_tri2);
                }
            }

            if (mTriangles_2->size() == 0) {
                return;
            }

            PotentialCollidingSets set = PotentialCollidingSets(*mTriangles_1,p1,*mTriangles_2,p2);

            addResults(set);

        }
    }

    /*!
     * calculates the minimal set of colliding lowest level BoundingVolumes and
     * creates narrowphasejobs containing two triangle-sets
     * (for collisions between one deformable and one rigid shape object - i.e. box/sphere/wedge/...)
     *
     */
    void SurfaceHierarchyJob::calculateMinimalCollisionsWithShape(const BvhNode* p1_work, const BvhNode* shapeNode, Proxy* p1, Proxy* p2) {

        if (intersectSpheres(p1_work,shapeNode)) {



            //checks if the bottom of the hierarchy is reached
            if (p1_work->getChildren().size() != 0) {

                for (std::list<BvhNode*>::const_iterator iter_children_1 = p1_work->getChildren().begin();
                                                            iter_children_1 != p1_work->getChildren().end();
                                                            ++iter_children_1) {
                    calculateMinimalCollisionsWithShape(*iter_children_1, shapeNode, p1, p2);
                }

            // the bottom of the hierarchy is reached and the node and the shape collides
            } else {


                std::vector<Triangle*>* mTriangles_1 = new std::vector<Triangle*>;
                std::vector<Triangle*>* mTriangles_2 = new std::vector<Triangle*>;

                const std::list<Triangle*> p1_triangles = p1_work->getBoundingVolume()->getEnclosedTriangles();
                std::vector<Triangle*> p2_triangles = p2->getShape()->getMesh()->getTriangles();

                for (std::list<Triangle*>::const_iterator iter_tri1 = p1_triangles.begin();
                                                            iter_tri1 != p1_triangles.end();
                                                            ++iter_tri1) {

                    //if (intersectBoundingVolumeTriangle(p2_work, *iter_tri1)) {
                    mTriangles_1->push_back(*iter_tri1);
                    //}
                }


                if (mTriangles_1->size() == 0) {
                    return;
                }

                for (std::vector<Triangle*>::const_iterator iter_tri2 = p2_triangles.begin();
                                                            iter_tri2 != p2_triangles.end();
                                                            ++iter_tri2) {

                    //if (intersectBoundingVolumeTriangle(p1_work, *iter_tri2)) {
                    mTriangles_2->push_back(*iter_tri2);
                    //}
                }

                if (mTriangles_2->size() == 0) {
                    return;
                }

                PotentialCollidingSets set = PotentialCollidingSets(*mTriangles_1, p1, *mTriangles_2, p2);

                addResults(set);
            }
        }
    }

    /*!
     * tests a boundingvolume with a triangle, based on the AABB of that triangle
     *
     */
    inline bool SurfaceHierarchyJob::intersectBoundingVolumeTriangle(const BvhNode* node, Triangle* triangle) {

        if (node->getBoundingVolume() != NULL) {
            if (node->getBoundingVolume()->getVolumeType() == BV_TYPE_SPHERE) {
                return intersectSphereTriangle(node, triangle);
            }
            if (node->getBoundingVolume()->getVolumeType() == BV_TYPE_AABB) {
                // adjust Triangle to an aabb and check if collide
                Aabb* a = new Aabb();
                a->adjustToTriangle(triangle);
                return (node->getBoundingVolume())->collidesWith(*a);
            }
        }
        return 1;
    }

    /*!
     * tests a BoundingSphere with a triangle (exact test)
     *
     * \author Marc Schulz <shade@nightmareshadow.de>
     */
    inline bool SurfaceHierarchyJob::intersectSphereTriangle(const BvhNode* sphere, Triangle* triangle) {

        //std::cout << "Warning" << std::endl;

        Vector3 center = ((BoundingSphere*) sphere->getBoundingVolume())->getCenterVector();
        real square_radius = ((BoundingSphere*) sphere->getBoundingVolume())->getRadius();
        square_radius *= square_radius;
        dcollide::array<Vertex*,3> vertices = triangle->getVertices();
        real square_distance;

        for (dcollide::array<Vertex*,3>::iterator iter_vertex = vertices.begin();
                                                    iter_vertex != vertices.end();
                                                    ++iter_vertex) {

            square_distance = ( (center.getX() - (*iter_vertex)->getWorldPosition().getX()) * (center.getX() - (*iter_vertex)->getWorldPosition().getX()) );
            square_distance += ( (center.getY() - (*iter_vertex)->getWorldPosition().getY()) * (center.getY() - (*iter_vertex)->getWorldPosition().getY()) );
            square_distance += ( (center.getZ() - (*iter_vertex)->getWorldPosition().getZ()) * (center.getZ() - (*iter_vertex)->getWorldPosition().getZ()) );

            if (square_distance < square_radius) {
                return 1;
            }
        }

        return 0;

    }

    /*!
     * check if the two BoundingVolumes in the two BvhNodes intersects
     *
     */
    inline bool SurfaceHierarchyJob::intersectSpheres(const BvhNode* node1, const BvhNode* node2) {

        return (node1->getBoundingVolume())->collidesWith(*(node2->getBoundingVolume()));
    }

}

/*
 * vim: et sw=4 ts=4
 */
