/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,         *
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

#include "mydeformableobjectnode.h"

#include "ogresceneobjectfactory.h"
#include "ogrematerials.h"
#include "odewrapper.h"

#include <d-collide/dcollide-global.h>
#include <d-collide/shapes/shape.h>
#include <d-collide/shapes/mesh.h>
#include <d-collide/shapes/mesh/vertex.h>
#include <d-collide/shapes/mesh/triangle.h>
#include <d-collide/math/vector.h>
#include <d-collide/math/matrix.h>
#include <d-collide/proxy.h>
#include <d-collide/exceptions/exception.h>
#include <d-collide/datatypes/multimapelement.h>

#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreQuaternion.h>

#include <algorithm>


MyDeformableObjectNode::MyDeformableObjectNode(dcollide::World* world, dcollide::Shape* shape, int proxyType, bool useCulling, bool useTextures)
    : MyObjectNode(world, shape, proxyType, useCulling, useTextures) {

    if (!world) {
        throw dcollide::NullPointerException("parameter world");
    }
    if ((shape != 0) && !(proxyType & dcollide::PROXYTYPE_DEFORMABLE)) {
        throw dcollide::Exception("MyDeformableObjectNode can handle deformable proxies only!");
    }

    delete mODEGeom;
    mODEGeom = 0;
    mCenterPoint = 0;

    mPrintDebugOutput = false;
    mDebugOutputPrefix = "";
}

MyDeformableObjectNode::MyDeformableObjectNode(dcollide::Proxy* proxy, bool useCulling, bool useTextures)
    : MyObjectNode(proxy, useCulling, useTextures) {

    if (!proxy) {
        throw dcollide::NullPointerException("parameter proxy");
    }
    if (    (proxy->getShape() != 0)
        && !(proxy->getProxyType() & dcollide::PROXYTYPE_DEFORMABLE)) {

        throw dcollide::Exception("MyDeformableObjectNode can handle deformable proxies only!");
    }

    delete mODEGeom;
    mODEGeom = 0;
    mCenterPoint = 0;
    
    mPrintDebugOutput = false;
    mDebugOutputPrefix = "";
}

MyDeformableObjectNode::~MyDeformableObjectNode() {    
    if (!mPhysicsObjects.empty()) {
        if (mJoints) {
            dJointGroupDestroy(mJoints);
        }
    }

    if (!mCenterPoint) {
        delete mCenterPoint;
    }
    
    for(std::vector<MyODEDeformableTestAppGeom*>::iterator i = mPhysicsObjects.begin(); i != mPhysicsObjects.end(); ++i) {
       delete (*i);
    }
}

void MyDeformableObjectNode::createPhysicsBody(dWorldID odeWorld, dReal totalMass) {
    if (!odeWorld) {
        throw dcollide::NullPointerException("dWorldID odeWorld");
    }

    if (!mProxy->getShape()) {
        std::cerr << "WARNING: cannot create physics body for a shapeless Proxy."<<std::endl;
        return;
    }

    if (!mPhysicsObjects.empty()) {
        std::cerr << "WARNING: previously created a physics body for this node." << std::endl;
        return;
    }

    
    const std::vector<dcollide::Vertex*> vertices
        = mProxy->getShape()->getMesh()->getVertices();

    int verticesCount = vertices.size();


    // This object is physically equipped/enabled
    mUseODE = true;

    // Distribute the given mass to the whole body
    dReal vertexRadius = 0.5;
    dReal vertexMass = totalMass / verticesCount;

    // Create a physics mid point for the object when its a volume object
    dcollide::Vector3 meshCenter; 
    if (mProxy->getProxyType() & dcollide::PROXYTYPE_CLOSEDHULL) {
        mCenterPoint = new MyODETestAppGeom(this); 

        mCenterPoint->setMoveNodeOnBodyMoved(false);

        dMass* mass = new dMass();
        dBodyID body = dBodyCreate(odeWorld);

        dMassSetSphereTotal(mass, vertexMass, vertexRadius);

        //dBodySetMass(body, mass);
        dGeomSetBody((dGeomID)mCenterPoint, body);

        meshCenter = mProxy->getShape()->getMesh()->getMidPoint();
        dBodySetPosition(mCenterPoint->getBody(), meshCenter[0], meshCenter[1], meshCenter[2]);
    }


    // Create an physics body for each vertex of the mesh

    mPhysicsObjects.reserve(verticesCount);
    for (int index = 0; index < verticesCount; ++index) {

        mPhysicsObjects.push_back(
            new MyODEDeformableTestAppGeom(this, index)
        );

        dMass* mass = new dMass();
        dBodyID body = dBodyCreate(odeWorld);

        dMassSetSphereTotal(mass, vertexMass, vertexRadius);

        //dBodySetMass(body, mass);
        dGeomSetBody((dGeomID)mPhysicsObjects[index], body);


        // Adjust the ODE body position
        mPhysicsObjects[index]->setMoveNodeOnBodyMoved(false);

        const dcollide::Vector3& position = vertices[index]->getWorldPosition();
        dBodySetPosition(body, position[0], position[1], position[2]);

        mPhysicsObjects[index]->setMoveNodeOnBodyMoved(true);
    }


    // Connect the created physics bodies through joints

    // Create a joint groups (useful for static deformations)
    mJoints = dJointGroupCreate(0);

    // Set which avoids double creation
    std::set<dcollide::MultiMapElement<dcollide::Vertex, dJointID> > finished;
    std::set<dcollide::MultiMapElement<dcollide::Vertex, dJointID> >::iterator finished_pos;

    // Create the actual joints
    for (std::vector<dcollide::Vertex*>::const_iterator one = vertices.begin();
         one != vertices.end();
         ++one) {

        dcollide::Vertex* v1 = (*one);

        dBodyID body1 = mPhysicsObjects[v1->getVertexIndex()]->getBody();

        //const dReal* v1Pos = dBodyGetPosition(body1);
        //dcollide::debug() << "Body of vertex1 (worldPosition="<< v1->getWorldPosition()<<") is at (" << v1Pos[0] << ", " << v1Pos[1] << ", " << v1Pos[2] << ")";

        const std::list<dcollide::Vertex*>& list = (*one)->getAdjacentVertices();
        for (std::list<dcollide::Vertex*>::const_iterator two = list.begin();
             two != list.end();
             ++two) {

            dcollide::Vertex* v2 = (*two);
            dBodyID body2 = mPhysicsObjects[v2->getVertexIndex()]->getBody();

            finished_pos = finished.find(dcollide::MultiMapElement<dcollide::Vertex, dJointID>(v1, v2));
            
            if (finished_pos == finished.end()) {
                dcollide::Vector3 center  = (v1->getWorldPosition() + v2->getWorldPosition()) / 2;

                dJointID joint = dJointCreateBall(odeWorld, mJoints);
                //dJointID joint = dJointCreateFixed(odeWorld, mJoints);
                //dJointID joint = dJointCreateUniversal(odeWorld, mJoints);

                dJointAttach(joint, body1, body2);
                dJointSetBallAnchor(joint, center.getX(), center.getY(), center.getZ());
                //dJointSetFixed(joint);
/*
                dcollide::Vector3 axisOne  = (v1->getWorldPosition() - v2->getWorldPosition());
                dcollide::Vector3 axisTwo  = v1->getAdjacentTriangles().front()->getWorldCoordinatesNormalVector();

                dJointSetUniversalAnchor(joint, center.getX(), center.getY(), center.getZ());
                dJointSetUniversalAxis1(joint, axisOne.getX(), axisOne.getY(), axisOne.getZ());
                dJointSetUniversalAxis2(joint, axisTwo.getX(), axisTwo.getY(), axisTwo.getZ());
*/
                finished.insert(dcollide::MultiMapElement<dcollide::Vertex, dJointID>(v1, v2, joint));
                
/*
                // Create the so called bending joints
                std::list<dcollide::Vertex*>::const_iterator adjecent_pos;
                const std::list<dcollide::Vertex*>& farneigbours = v2->getAdjacentVertices();
                for (std::list<dcollide::Vertex*>::const_iterator neighbour = farneigbours.begin();
                     neighbour != farneigbours.end();
                     ++neighbour) {

                    dcollide::Vertex* v3 = (*neighbour);

                    adjecent_pos = find(list.begin(), list.end(), v3);
                    if (adjecent_pos == list.end()) {
                        // The current node is a far neighbour and thus should be connected

                        finished_pos = finished.find(dcollide::MultiMapElement<dcollide::Vertex, dJointID>(v2, v3));
            
                        // If it wasn't previously created -> create it!
                        if (finished_pos == finished.end()) {

                            dcollide::Vector3 center  = (v2->getWorldPosition() + v3->getWorldPosition()) / 2;

                            dJointID joint = dJointCreateBall(odeWorld, mJoints);

                            dJointAttach(joint, body1, body2);
                            dJointSetBallAnchor(joint, center.getX(), center.getY(), center.getZ());

                            finished.insert(dcollide::MultiMapElement<dcollide::Vertex, dJointID>(v2, v3, joint));
                        }
                    }
                }
*/
            }
        }

        // Connect all vertex "bodies" with the mid point
        if (mCenterPoint != 0) {
            dcollide::Vector3 center = (v1->getWorldPosition() + meshCenter) / 2;
            dcollide::Vector3 direction = v1->getWorldPosition() - meshCenter;

            dJointID joint = dJointCreateBall(odeWorld, mJoints);
            //dJointID joint = dJointCreateSlider(odeWorld, mJoints);

            dJointAttach(joint, body1, mCenterPoint->getBody());
            //dJointSetFixed(joint);
            
            dJointSetBallAnchor(joint, center.getX(), center.getY(), center.getZ());
            //dJointSetSliderAxis(joint, direction.getX(), direction.getY(), direction.getZ());
        }
    }
}

dBodyID MyDeformableObjectNode::getPhysicsBody(unsigned int vertexIndex) const {
/*
    bool empty = mPhysicsObjects.empty();
    int size = mPhysicsObjects.size();

    std::cout << std::endl
        << "--- getPhysicsBody:" << std::endl
        << " empty: " << empty << std::endl
        << " vertexIndex: " << vertexIndex << std::endl
        << "size: " << size << std::endl;
*/
    if (!mPhysicsObjects.empty() && (vertexIndex < mPhysicsObjects.size())) {
/*
        std::cout << " MyODEGeom: " << std::hex << (unsigned int) mPhysicsObjects[vertexIndex] << std::endl;
        std::cout << " dGeomId: " << std::hex << (unsigned int) mPhysicsObjects[vertexIndex]->getBody() << std::endl;
*/
        return mPhysicsObjects[vertexIndex]->getBody();
    }

    return 0;
}

dGeomID MyDeformableObjectNode::getOdeGeom(unsigned int vertexIndex) const {

    if (!mPhysicsObjects.empty() && (vertexIndex < mPhysicsObjects.size())) {
        return (dGeomID) mPhysicsObjects[vertexIndex];
    }

    return 0;
}

void MyDeformableObjectNode::setVertexPosition(unsigned int vertexIndex, const dcollide::Vector3& vector, bool movePhysicsBody) {
    /* Funktion die von jedem MyOdeDeformableGeom aufgerufen wird um die von
     * ODE übermittelte Transformation an das mesh weiter zu geben.
     */
    
    if (mPrintDebugOutput) {
        std::cout << "------ " << mDebugOutputPrefix << std::endl
                  << "MyDeformableObjectNode::setVertexPosition" << std::endl
                  << " - index : " << std::dec << vertexIndex
                  << " - vector: " << vector << std::endl;
    }

    if (!getProxy()) {
        return;
    }

    /*
    dcollide::Matrix* matrix = new dcollide::Matrix();
    if (mProxy->getTransformation().invert(matrix)) {
        dcollide::Vector3* out = new dcollide::Vector3();
        matrix->transform(out, vector);
        mProxy->setVertexPosition(vertexIndex, vector);
    }
    */
    
    mProxy->setVertexPosition(vertexIndex, vector);

    mBoundingVolumeDirty = true;

    if (movePhysicsBody) {
        updatePhysicsBodies(vertexIndex);
    }
}

void MyDeformableObjectNode::translate(const dcollide::Vector3& translation, bool respectOrientation) {
    if (mPrintDebugOutput) {
        std::cout << mDebugOutputPrefix << "MyDeformableObjectNode::translate(const dcollide::Vector3& translation)" << std::endl;
    }

    if (!getProxy()) {
        return;
    }

    mProxy->translate(translation, respectOrientation);

    mBoundingVolumeDirty = true;
    updatePhysicsBodies();
}

void MyDeformableObjectNode::setPosition(float x, float y, float z, bool movePhysicsBody) {
    if (mPrintDebugOutput) {
        std::cout << mDebugOutputPrefix << "setPosition(x: " << x <<", y: " << y << ", z: " << z << ", movePhysicsBody: " << movePhysicsBody << ")" << std::endl;
    }

    if (!getProxy()) {
        return;
    }

    mProxy->setPosition(x, y, z);

    mBoundingVolumeDirty = true;

    if (movePhysicsBody) {
        updatePhysicsBodies();
    }
}

void MyDeformableObjectNode::rotate(const dcollide::Matrix& rotation, bool respectOrientation) {
    if (mPrintDebugOutput) {
        std::cout << mDebugOutputPrefix << "MyDeformableObjectNode::rotate(const dcollide::Matrix& rotation)" << std::endl;
    }

    if (!getProxy()) {
        return;
    }

    mProxy->rotate(rotation, respectOrientation);

    mBoundingVolumeDirty = true;
    updatePhysicsBodies();
}

void MyDeformableObjectNode::setRotation(const dcollide::Matrix& rotation, bool movePhysicsBody) {
    if (mPrintDebugOutput) {
        std::cout << mDebugOutputPrefix << "MyDeformableObjectNode::setRotation(const dcollide::Matrix& rotation, bool movePhysicsBody)" << std::endl;
    }

    if (!getProxy()) {
        return;
    }

    mProxy->setRotation(rotation);

    mBoundingVolumeDirty = true;

    if (movePhysicsBody) {
        updatePhysicsBodies();
    }
}

/*!
 * \brief Performs an deformation on the deposited proxy
 * 
 * NOTICE: The size of given vector must match the Vertex count of the mesh
 *         which should be deformed.  
 */
void MyDeformableObjectNode::deform(const std::vector<dcollide::Vector3>& vertexMoveArray) {
    if (mPrintDebugOutput) {
        std::cout << mDebugOutputPrefix << "MyDeformableObjectNode::deform(const std::vector<dcollide::Vector3>& vertexMoveArray)" << std::endl;
    }

    if (!getProxy()) {
        return;
    }

    mProxy->deform(vertexMoveArray);

    mBoundingVolumeDirty = true;
}

void MyDeformableObjectNode::updatePhysicsBodies(int vertexIndex) {
    if (mPrintDebugOutput) {
        std::cout << mDebugOutputPrefix << "MyDeformableObjectNode::updatePhysicsBodies(vertexIndex: " << vertexIndex << ")" << std::endl;
    }

    if (!mPhysicsObjects.empty()) {

        unsigned int index, objectCount;

        dcollide::Vertex* vertex;
        MyODEDeformableTestAppGeom* geom;


        if (vertexIndex != -1) {
            index = vertexIndex;
            objectCount = vertexIndex+1;
        } else {
            index = 0;
            objectCount = mPhysicsObjects.size();
        }

        const std::vector<dcollide::Vertex*>& vertices
            = getProxy()->getShape()->getMesh()->getVertices();
        for (/* nop */; index < objectCount; ++index) {

            geom = mPhysicsObjects[index];
            vertex = vertices[index];

            dcollide::Vector3 bodyPosition = vertex->getWorldPosition();

            if (mPrintDebugOutput) {
                if (index == objectCount-1) {
                    const dReal* odePosition = 0;
                    odePosition = dBodyGetPosition(geom->getBody());

                    std::cout << mDebugOutputPrefix << "----------" << std::endl;
                    std::cout << mDebugOutputPrefix << "ODE Position (before move): " << odePosition[0] << ", " << odePosition[1] << ", " << odePosition[2] << std::endl;
                    std::cout << mDebugOutputPrefix << "D-Collide World Position: " << bodyPosition << std::endl << std::endl;
                }
            }

            geom->setMoveNodeOnBodyMoved(false);
            dBodySetPosition(geom->getBody(),  bodyPosition.getX(),
                                               bodyPosition.getY(),
                                               bodyPosition.getZ());
            geom->setMoveNodeOnBodyMoved(true);

            if (mPrintDebugOutput) {
                if (index == objectCount-1) {
                    const dReal* odePosition = 0;
                    odePosition = dBodyGetPosition(geom->getBody());
                    std::cout << mDebugOutputPrefix << "ODE Position (after move): " << odePosition[0] << ", " << odePosition[1] << ", " << odePosition[2] << std::endl << std::endl;
                }
            }
        }
    }
}

