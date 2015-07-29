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

#include "collision_kernel.h"

#include <ode/odewrapper.h>
#include <ode/objects.h>

#include <d-collide/math/matrix.h>
#include <d-collide/proxy.h>
#include <d-collide/shapes/shape.h>
#include <d-collide/shapes/box.h>
#include <d-collide/shapes/sphere.h>
#include <d-collide/debugstream.h>
#include <d-collide/bvhnode.h>
#include <d-collide/boundingvolumes/boundingvolume.h>
#include <d-collide/exceptions/exception.h>

dGeomID dGeomGetBodyNextDCollide (dGeomID geom);
void dGeomSetBodyDCollide (dGeomID geom, dBodyID body);
void dGeomMovedDCollide (dGeomID geom);
dBodyID dGeomGetBodyDCollide (dGeomID geom);

dGeomID dGeomGetBodyNextODE (dGeomID geom);
void dGeomSetBodyODE (dGeomID geom, dBodyID body);
void dGeomMovedODE (dGeomID geom);
dBodyID dGeomGetBodyODE (dGeomID geom);

bool MyODEGeom::mCustomCollisionDetectionInitialized = false;

MyODEGeom::MyODEGeom() {
    mCustomCollisionDetectionInitialized = true; // we cannot change the collision functions anymore
    mBody = 0;
    mMoveNodeOnBodyMoved = true;

    mODEGeom = 0;
}

MyODEGeom::~MyODEGeom() {
    if (mBody) {
        dBodyDestroy(mBody);
    }
    if (mODEGeom) {
        dGeomDestroy(mODEGeom);
    }
}

void MyODEGeom::initializeODECollisionFunctions() {
    if (mCustomCollisionDetectionInitialized) {
        dcollide::error() << dc_funcinfo << "custom collision detection already initialized - cannot change collision detection functions anymore";
        return;
    }
    dChangeCollisionDetection(&dGeomMovedDCollide, &dGeomGetBodyNextDCollide, &dGeomSetBodyDCollide, &dGeomGetBodyDCollide);
}

/*!
 * Should be called by \ref dGeomSetBody (i.e. \ref dGeomSetBodyDCollide).
 *
 * Do NOT call directly!
 */
void MyODEGeom::setBody(dBodyID body) {
    if (mBody) {
        mBody->geom = 0;
    }
    
    mBody = body;
    
    if (body) {
        body->geom = (dGeomID) this;
    }
}

void MyODEGeom::notifyBodyMoved() {
    //dcollide::debug() << dc_funcinfo;
    if (mMoveNodeOnBodyMoved) {
        //get the new state from ODE
        const dReal* odeposition = dBodyGetPosition(mBody);
        const dReal* oderotation = dBodyGetRotation(mBody); //4x3 rotation matrix
        
        dcollide::Matrix dcMatrix;
        
        //copy the 4x3 rotation matrix, fill the last row with 0 0 0 1
        //NOTE: ODE uses row-major ordering, but ODE uses colum-major ordering
        //      simple Matrix-copying will not do the trick!
        
        //ODE rotation matrix format
        // 0  1   2   3
        // 4  5   6   7
        // 8  9  10 11
        dcMatrix.setElement(0, 0, oderotation[0]);
        dcMatrix.setElement(0, 1, oderotation[1]);
        dcMatrix.setElement(0, 2, oderotation[2]);
        dcMatrix.setElement(1, 0, oderotation[4]);
        dcMatrix.setElement(1, 1, oderotation[5]);
        dcMatrix.setElement(1, 2, oderotation[6]);
        dcMatrix.setElement(2, 0, oderotation[8]);
        dcMatrix.setElement(2, 1, oderotation[9]);
        dcMatrix.setElement(2, 2, oderotation[10]);
        
        dcollide::Vector3 bodyPosition(odeposition[0], odeposition[1], odeposition[2]);
        
        //The position consists of
        //    bodyposition = nodeposition + rotation * offset
        //<=> nodeposition = bodyposition - rotation * offset
        dcollide::Vector3 tmp;
        dcMatrix.transform(&tmp, mGravityOffset);
        
        dcollide::Vector3 nodePosition = bodyPosition - tmp;
        setPosition(nodePosition[0], nodePosition[1], nodePosition[2]);
        
        //Rotation matrix does not change
        setRotation(dcMatrix);
//        dcollide:: debug() << "dGeommoved: new body position: " <<bodyPosition<<", node position:"<<nodePosition;
    }
}


/*!
 * \return The ODE geom that was created by \ref createODEGeom.
 *
 * This method makes sense only if \ref
 * setUseOriginalODECollisionDetection(TRUE) was used at the beginning of this
 * program.
 */
dGeomID MyODEGeom::getODEGeom() const {
    return mODEGeom;
}


MyDCollideODEGeom::MyDCollideODEGeom(dcollide::Proxy* p) {
    mProxy = p;
    mProxy->setTransformation(dcollide::Matrix());
}

void MyDCollideODEGeom::setPosition(float x, float y, float z) {
    mProxy->setPosition(x, y, z);
}

void MyDCollideODEGeom::setRotation(const dcollide::Matrix& m) {
    mProxy->setRotation(m);
}


/*!
 * \brief 
 * 
 * From the ODE wiki:
 * This function is called by the dynamics code to traverse the list of geoms
 * that are associated with each body. Given a geom attached to a body, it 
 * returns the next geom attached to that body, or 0 if there are no more geoms.
 * 
 * DEVELOPER NOTICE: (GJ)
 *      for now, each physics body can only have one dGeom/MyObjectNode
 */
dGeomID dGeomGetBodyNextDCollide (dGeomID geom) {
    return 0;
}

void dGeomSetBodyDCollide (dGeomID geom, dBodyID body) {
    ((MyODEGeom*)geom)->setBody(body);
}

void dGeomMovedDCollide (dGeomID geom) {
    
    MyODEGeom* node = ((MyODEGeom*) geom);
    node->notifyBodyMoved();

}

dBodyID dGeomGetBodyDCollide(dGeomID geom) {
 
    MyODEGeom* node = ((MyODEGeom*) geom);
    return node->getBody();
}

/*
 * vim: et sw=4 ts=4
 */
