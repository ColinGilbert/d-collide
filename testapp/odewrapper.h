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

#ifndef DCOLLIDE_ODEWRAPPER_H
#define DCOLLIDE_ODEWRAPPER_H

#include "myobjectnode.h"
#include "mydeformableobjectnode.h"

#include <ode/ode.h>
#include <ode/odewrapper.h>

#include <iostream>

namespace dcollide{
    struct CollisionInfo;
}


//------------ classes ---------------------------------------------------------

/*!
 * \breif Represents a normal rigid body in ODE and links it to a MyObjectNode
 */
class MyODETestAppGeom : public MyODEGeom {
    public:
        inline MyODETestAppGeom(MyObjectNode* node);

    protected:
        MyObjectNode* mNode;

        virtual inline void setPosition(float x, float y, float z);
        virtual inline void setRotation(const dcollide::Matrix& matrix);

};


/*!
 * \brief Represents one vertex of an deformable Mesh in ODE
 */
class MyODEDeformableTestAppGeom : public MyODEGeom {
    private:
        unsigned int mVertexIndex;
        MyDeformableObjectNode* mParent;

    protected:
        inline MyODEDeformableTestAppGeom(MyDeformableObjectNode* parent,
                                          unsigned int vertexIndex);

        friend class MyDeformableObjectNode;

    protected:
        virtual inline void setPosition(float, float, float);
        virtual inline void setRotation(const dcollide::Matrix&);

    public:
        virtual inline ~MyODEDeformableTestAppGeom();

        inline MyDeformableObjectNode* getParent() const;
        inline unsigned int getVertexIndex() const;

};


//------------ implementation of short methods (MyODETestAppGeom) --------------

MyODETestAppGeom::MyODETestAppGeom(MyObjectNode* node) : MyODEGeom() {
    mNode = node;
}

void MyODETestAppGeom::setPosition(float x, float y, float z) {
    //NOTE: setting setPositions fourth parameter to false since we do not want
    //      setPosition to move the physics body
    
    mNode->setPosition(x, y, z, false);
}

void MyODETestAppGeom::setRotation(const dcollide::Matrix& m) {
    //NOTE: setting setRotation second parameter to false since we do not want
    //      setRotation to rotate the physics body

    mNode->setRotation(m, false);
}


//------------ implementation of short methods (MyODEDeformableTestAppGeom) ----

MyODEDeformableTestAppGeom::MyODEDeformableTestAppGeom
    (MyDeformableObjectNode* parent, unsigned int vertexIndex)
        : MyODEGeom() {
    
    // FIXME: We should guarantee that mParent wont be 0!

    mParent = parent;
    mVertexIndex = vertexIndex;
    
}

MyODEDeformableTestAppGeom::~MyODEDeformableTestAppGeom() {

}

void MyODEDeformableTestAppGeom::setPosition(float x, float y, float z) {
/*
    std::cout << "------" << std::cout;
    std::cout << "MyODEDeformableTestAppGeom::setPosition" << std::endl
              << "body id: " << std::hex << (unsigned int) mBody << " - position: " << dcollide::Vector3(x, y, z) << std::endl
              << "parent: " << std::hex << (unsigned int) mParent << " - vertex: " << std::dec << mVertexIndex << std::endl;
*/
    mParent->setVertexPosition(mVertexIndex, dcollide::Vector3(x, y, z), false);
}

void MyODEDeformableTestAppGeom::setRotation(const dcollide::Matrix& matrix) {

    // Nothing to do here, as vertices can't be rotated, and even if they
    // could be it wouldn't have any effect.
}

MyDeformableObjectNode* MyODEDeformableTestAppGeom::getParent() const {
    return mParent;
}

unsigned int MyODEDeformableTestAppGeom::getVertexIndex() const {
    return mVertexIndex;
}


//------------ helper functions ------------------------------------------------

dContactGeom convertCollisionInfo(dcollide::CollisionInfo& collision);

void convertDcollideMatrix2Ode(const dcollide::Matrix& dcMatrix, dMatrix3& output);


#endif // DCOLLIDE_ODEWRAPPER_H
/*
 * vim: et sw=4 ts=4
 */
