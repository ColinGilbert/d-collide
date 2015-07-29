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
       notice, this list of conditions and the following disclaimer.           *
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


#include "deformablebvhnode.h"
#include "normalcone.h"
#include "shapes/mesh/triangle.h"
#include "shapes/mesh/vertex.h"
#include "surfacehierarchyproxydata.h"

#include <list>
#include <iostream>

namespace dcollide {

    // Constructor(s) and Destructor
    DeformableBvhNode::DeformableBvhNode(World* world) : BvhNode(world) {

    }

    /*!
    * creates a BoundingSphere and adds all triangles to the
    * mEnclosedTriangles variable
    */
    void DeformableBvhNode::createBoundingSphere(const std::list<Triangle*>& triangles,
                                                 const Vertex* center) {

        //std::cout << "DeformableBvhNode::createBoundingSphere(const std::list<Triangle*>& triangles, const Vertex* center)" << std::endl;

        Vertex* vertex0;
        Vertex* vertex1;
        Vertex* vertex2;

        real squareRadius = 0;
        real potentialRadiusA = 0;
        real potentialRadiusB = 0;
        real potentialRadiusC = 0;

        Vector3 centerPoint = center->getPosition();


        std::list<Triangle*>::const_iterator iter;
        iter = triangles.begin();
        for (; iter != triangles.end(); iter++) {

            vertex0 = ((*iter)->getVertices())[0];
            vertex1 = ((*iter)->getVertices())[1];
            vertex2 = ((*iter)->getVertices())[2];

            potentialRadiusA = (centerPoint - vertex0->getPosition()).dotProduct();
            potentialRadiusB = (centerPoint - vertex1->getPosition()).dotProduct();
            potentialRadiusC = (centerPoint - vertex2->getPosition()).dotProduct();

            if (potentialRadiusA >= potentialRadiusB) {
                if (potentialRadiusA >= potentialRadiusC) {
                    if (squareRadius < potentialRadiusA) {
                        squareRadius = potentialRadiusA;
                    }
                } else {
                    if (squareRadius < potentialRadiusC) {
                        squareRadius = potentialRadiusC;
                    }
                }
            } else {
                if (potentialRadiusB >= potentialRadiusC) {
                    if (squareRadius < potentialRadiusB) {
                        squareRadius = potentialRadiusB;
                    }
                } else {
                    if (squareRadius < potentialRadiusC) {
                        squareRadius = potentialRadiusC;
                    }
                }
            }
        }

        // creates an new BoundingSphere with the radius and the center
        mBoundingVolume = new BoundingSphere(center, sqrt(squareRadius));
        mBoundingVolume->setEnclosedTriangles(triangles);

        //std::cout << "Radius " << sqrt(squareRadius) << std::endl;
        //std::cout << "Center " << centerPoint.getX() << " " << centerPoint.getY() << " "
        //        << centerPoint.getZ() << std::endl;
    }

    /*!
    * creates a BoundingSphere which encloses all given bvhNodes and adds all children
    * to this parent
    */
    void DeformableBvhNode::createBoundingSphere(const std::list<DeformableBvhNode*>& bvhNodes) {

        //std::cout << "DeformableBvhNode::createBoundingSphere(const std::list<DeformableBvhNode*>& bvhNodes)" << std::endl;

        BoundingSphere* bv = 0;

        for (std::list<DeformableBvhNode*>::const_iterator iter = bvhNodes.begin();
             iter != bvhNodes.end();
             ++iter) {

            if (bv == 0) {
                bv = new BoundingSphere(*((BoundingSphere*)((*iter)->getBoundingVolume())));

            } else {
                bv->mergeWith((BoundingSphere*)((*iter)->getBoundingVolume()));
            }
        }

        mBoundingVolume = bv;
    }

    /*!
    * creates a BoundingSphere which encloses all given bvhNodes and adds all children
    * to this parent
    */
    void DeformableBvhNode::createBoundingSpherePrecise(const std::list<DeformableBvhNode*>& bvhNodes) {

        //std::cout << "DeformableBvhNode::createBoundingSphere(const std::list<DeformableBvhNode*>& bvhNodes)" << std::endl;

        int counter = 0;
        Vector3 centerVector;
        real biggestDistance = 0.0f;
        real biggestRadius = 0.0f;

        for (std::list<DeformableBvhNode*>::const_iterator iter = bvhNodes.begin();
                                                            iter != bvhNodes.end();
                                                            ++iter) {
            ++counter;
            centerVector.add(((BoundingSphere*)((*iter)->getBoundingVolume()))->getCenterVector());
        }

        centerVector = centerVector * ( 1.0f / counter);

        real tempDistance;
        real tempRadius;

        for (std::list<DeformableBvhNode*>::const_iterator iter = bvhNodes.begin();
                                                            iter != bvhNodes.end();
                                                            ++iter) {

            tempDistance = (centerVector - (((BoundingSphere*)((*iter)->getBoundingVolume()))->getCenterVector())).length();
            tempRadius = ((BoundingSphere*)((*iter)->getBoundingVolume()))->getRadius();

            if (   tempDistance+tempRadius > biggestDistance   ) {
                biggestDistance = tempDistance+tempRadius;
            }
        }

        biggestRadius = biggestDistance;

        mBoundingVolume = new BoundingSphere(centerVector, biggestRadius);
    }

    /*!
    * \brief Constructor
    *
    *  creates a new BVHNode and a BoundingSphere which has one center and
    *  contains all triangles getting from the list
    */
    DeformableBvhNode::DeformableBvhNode(World* world, const std::list<Triangle*>& triangles, const Vertex* center)
            : BvhNode(world) {
        createBoundingSphere(triangles, center);

        //sets each Triangle his enclosing BoundingNode
        std::list<Triangle*>::const_iterator iter;
        iter = triangles.begin();

        for (;iter != triangles.end(); iter++) {
            (*iter)->addBvHierarchyNode(this);
        }
    }

    /*!
    * \brief Constructor
    *
    * creates a new BVHNode with a BoundingSphere containing all given BoundingSpheres.
    * The center of this new BoundingSphere is a given Vertex from the Mesh
    */
    DeformableBvhNode::DeformableBvhNode(World* world, const std::list<DeformableBvhNode*>& bvhNodes,
                                         DeformableBvhNode* centerSphere)
            : BvhNode(world) {

        const Vertex* center = ((BoundingSphere*) centerSphere->getBoundingVolume())->getCenterVertex();
        const BoundingSphere* bs;
        real currentLength = 0.0;
        real biggestLength = 0.0;
        //calculates the Sphere with the most distance to the center

        for (std::list<DeformableBvhNode*>::const_iterator iter = bvhNodes.begin(); iter != bvhNodes.end(); iter++) {
            bs = (BoundingSphere*)(*iter)->getBoundingVolume();
            currentLength = (bs->getCenterVector() - center->getWorldPosition()).length() + (bs->getRadius());
            if (biggestLength < currentLength) {
                biggestLength = currentLength;
            }
            //adds this bvhNode its children bvhNodes
            (*iter)->setParent(this);
            mChildren.push_back(*iter);
        }
        centerSphere->setParent(this);
        mChildren.push_back(centerSphere);
        mBoundingVolume = new BoundingSphere(center, biggestLength);
    }

    /*!
    * \brief Constructor
    *
    * Calculates a BoundingSphere which includes all BoundingSpheres getting from the vector.\n
    * The center of this new BoundingSphere is also calculated.\n
    * ATTENTION: \n
    * Vectors with only 1 element makes no sense and are not possible at the moment
    */
    DeformableBvhNode::DeformableBvhNode(World* world, const std::list<DeformableBvhNode*>& bvhNodes)
            : BvhNode(world) {

        createBoundingSpherePrecise(bvhNodes);
    }

    /*!
    * \brief constructor for the root Node
    *
    * Calculates a BoundingSphere which includes all BoundingSpheres getting from the vector.\n
    * The center of this new BoundingSphere is also calculated.\n
    * The proxy is set.
    * ATTENTION: \n
    * Vectors with only 1 element makes no sense and are not possible at the moment
    */
    DeformableBvhNode::DeformableBvhNode(World* world, const std::list<DeformableBvhNode*>& bvhNodes,
                                         Proxy* proxy,
                                         SurfaceHierarchyProxyData* proxyData) : BvhNode(world, proxyData->getProxy()) {

        //std::cout << "create DeformableBvhNode with Proxy" << std::endl;
        createBoundingSpherePrecise(bvhNodes);

        std::list<DeformableBvhNode*>::const_iterator iter;
        iter = bvhNodes.begin();
        for (; iter != bvhNodes.end(); iter++) {
            mChildren.push_back(*iter);
        }

        proxyData->mDeformableNode = this;
    }

    /*!
    * \brief Adds one triangle and recalculates the BoundingSphere
    *
    */
    void DeformableBvhNode::addTriangle(Triangle* triangle) {

        //std::cout << "DeformableBvhNode::addTriangle(Triangle* triangle)" << std::endl;

        //adds the triangle to the existing list of Enclosed Triangles
        std::list<Triangle*> triangles = mBoundingVolume->getEnclosedTriangles();
        triangles.push_back(triangle);

        //sets the Triangle his enclosing BoundingNode

        triangle->addBvHierarchyNode(this);

        const Vertex* center = ((BoundingSphere*) mBoundingVolume)->getCenterVertex();
        delete(mBoundingVolume);
        createBoundingSphere(triangles, center);
    }


    /*!
    * \brief Adds a DeformableBvhNode from the sub-layer to this DeformableBvhNode, recalculates the BoundingSphere nad keeps the enclosedTrianlges
    *
    */
    void DeformableBvhNode::addChildDeformableBvhNode(DeformableBvhNode* additionalChild) {

        //FIXME const std::list<Triangle*> tempEnclosedTrianlges = this.getEnclosedTriangles();//setEnclosedTriangles
        // enthält nur die unterste ebene dei liste der enclosedTriangles????

        const Vertex* center = ((BoundingSphere*) mBoundingVolume)->getCenterVertex();
        real maxRadius = 0;
        real radiusAdditionalChild = ((BoundingSphere*) additionalChild->getBoundingVolume())->getRadius();
        real radiusThis = ((BoundingSphere*) mBoundingVolume)->getRadius();

        if ( ((center->getWorldPosition() - ((BoundingSphere*)(additionalChild->getBoundingVolume()))->getCenterVector()).length() + radiusAdditionalChild) > radiusThis ) {
            maxRadius = ((center->getWorldPosition() - ((BoundingSphere*)(additionalChild->getBoundingVolume()))->getCenterVector()).length() + radiusAdditionalChild);
        } else {
            maxRadius = radiusThis;
        }

        additionalChild->setParent(this);
        mChildren.push_back(additionalChild);
        delete(mBoundingVolume);
        mBoundingVolume = new BoundingSphere(center, maxRadius);

    }

    void DeformableBvhNode::recalculateBoundingVolumes() {
        std::cout << "___recalculateBoundingVolumes called in DeformableBvhNode___";
    }


    DeformableBvhNode::~DeformableBvhNode() {

    }


    /*!
     * \brief creates a top-down recursion stack and when returning updates
     * the normal cones bottom-up
     */
    void DeformableBvhNode::updateTopDown() {
        if (!mChildren.empty()) {
            for (std::list<BvhNode*>::iterator iter = mChildren.begin();
                    iter != mChildren.end(); ++iter) {
                (dynamic_cast<DeformableBvhNode*>(*iter))->updateTopDown();
            }
        }
        updateNormalCone();
    }

    /*!
     * \brief Caculates a new normal cone for this hierarchy node.
     *
     * If this node is an leaf (no child nodes are assigned), then the normal
     * cones of each triangle enclosed by the bounding volume of this node are
     * added up together to the new normal cone. In the other case when this
     * node isn't an leaf, the normal cones of all hierarchy childs are added
     * up to the new normal cone.
     */
    void DeformableBvhNode::updateNormalCone() {

        if (mChildren.size() > 0) {
        // iterate over the normal cones of all child nodes

            std::list<BvhNode*>::iterator iter;

            iter = mChildren.begin();
            mNormalCone = *((DeformableBvhNode*) *iter)->getNormalCone();

            ++iter;

            for (; iter != mChildren.end(); ++iter) {
                mNormalCone +=
                    *((DeformableBvhNode*) *iter)->getNormalCone();
            }

        } else {
        // iterate over the triangles enclosed by the corresponding bv

            std::list<Triangle*>::const_iterator iter;

            iter = mBoundingVolume->getEnclosedTriangles().begin();
            mNormalCone = *(*iter)->getNormalCone();

            ++iter;

            for (; iter != mBoundingVolume->getEnclosedTriangles().end();
                 ++iter) {

                mNormalCone += *(*iter)->getNormalCone();
            }
        }
    }

    /*!
     * \brief checks if the DeformableBvhNode given as argument is neighbour of this DeformableBvhNode.
     *
     * ...TODO
     */
    bool DeformableBvhNode::isNeighbourOf(DeformableBvhNode* node) const {

        std::set<DeformableBvhNode*>::const_iterator test = mAdjacentDeformableBvhNodes.find(node);
        if (test == mAdjacentDeformableBvhNodes.end()) {
            return 0;
        } else {
            return 1;
        }

    }

}
/*
 * vim: et sw=4 ts=4
 */
