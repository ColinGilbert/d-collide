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

#ifndef DCOLLIDE_MYDEFORMABLEOBJECTNODE_H
#define DCOLLIDE_MYDEFORMABLEOBJECTNODE_H

#include "myobjectnode.h"

#include <ode/ode.h>

#include <list>

namespace dcollide {
    class Vector3;
    class Proxy;
    class Shape;
    class World;
}

class MyODEDeformableTestAppGeom;

class MyDeformableObjectNode : public MyObjectNode {
    public:
        MyDeformableObjectNode(dcollide::World* world, dcollide::Shape* shape, int proxyType = -1, bool useCulling = true, bool useTextures = true);
        MyDeformableObjectNode(dcollide::Proxy* proxy, bool useCulling = true, bool useTextures = true);
        virtual ~MyDeformableObjectNode();

        void deform(const std::vector<dcollide::Vector3>& vertexMoveArray);
        void setVertexPosition(unsigned int vertexIndex, const dcollide::Vector3& vector, bool movePhysicsBody = true);

        virtual inline void translate(float x, float y, float z, bool respectOrientation = true);
        virtual void translate(const dcollide::Vector3& translation, bool respectOrientation = true);
        virtual void setPosition(float x, float y, float z, bool movePhysicsBody = true);

        virtual inline void rotate(float angle, float x, float y, float z, bool respectOrientation = true);
        virtual void rotate(const dcollide::Matrix& rotation, bool respectOrientation = true);
        virtual void setRotation(const dcollide::Matrix& rotation, bool movePhysicsBody = true);

        virtual void createPhysicsBody(dWorldID odeWorld, dReal totalMass);
        inline dBodyID getPhysicsBody() const;
        dBodyID getPhysicsBody(unsigned int vertexIndex) const;
        dGeomID getOdeGeom(unsigned int vertexIndex) const;
        inline MyODETestAppGeom* getCenterPoint() const;

        inline virtual bool isDeformable() const;
        inline bool isVolumeObject() const;  

        inline virtual void printDebugOutput(bool b, std::string prefix = "");

    private:
        dJointGroupID mJoints;
        std::vector<MyODEDeformableTestAppGeom*> mPhysicsObjects;
        
        MyODETestAppGeom* mCenterPoint;
        
        void updatePhysicsBodies(int vertexIndex = -1);
};

inline bool MyDeformableObjectNode::isDeformable() const {
    return true;
}

inline bool MyDeformableObjectNode::isVolumeObject() const {
    return (mCenterPoint != 0);
}

inline dBodyID MyDeformableObjectNode::getPhysicsBody() const {
    return 0;
}

inline MyODETestAppGeom* MyDeformableObjectNode::getCenterPoint() const {
    return mCenterPoint;
}

inline void MyDeformableObjectNode::translate(float x, float y, float z, bool respectOrientation) {
    translate(dcollide::Vector3(x, y, z), respectOrientation);
}

inline void MyDeformableObjectNode::rotate(float angle, float x, float y, float z, bool respectOrientation) {
    dcollide::Matrix m;
    m.rotate(angle, x, y, z);
    rotate(m, respectOrientation);
}

inline void MyDeformableObjectNode::printDebugOutput(bool b, std::string prefix) {
    mPrintDebugOutput = b;
    mDebugOutputPrefix = prefix;
}

#endif // DCOLLIDE_MYDEFORMABLEOBJECTNODE_H
