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

#include "odewrapper.h"

#include <d-collide/proxy.h>
#include <d-collide/collisioninfo.h>
#include <d-collide/math/matrix.h>

#include <d-collide/debugstream.h>
#include <d-collide/exceptions/exception.h>

/*!
 * \brief convert a dcollide CollisionInfo into an ODE dContactGeom
 * 
 */
dContactGeom convertCollisionInfo(dcollide::CollisionInfo& collision) {
    dContactGeom result;
    result.pos[0] = collision.collisionPoint.getX();
    result.pos[1] = collision.collisionPoint.getY();
    result.pos[2] = collision.collisionPoint.getZ();

#ifdef __GNUC__
#warning TODO remove normal length test in release version
#endif
    if (fabs(collision.normal.length() - 1) > 0.001){
        collision.normal.normalize();
        dcollide::error() << " A Collision normal vector with length != 1  has been handed to ODE. It has been normalized automatically.";
        //throw dcollide::Exception("Collision normal vector with length == 1 handed to ODE.");
    }
    
    result.normal[0] = collision.normal.getX();
    result.normal[1] = collision.normal.getY();
    result.normal[2] = collision.normal.getZ();

    result.depth = (dReal) collision.penetrationDepth;

    //Proxy.mUser1 is the pointer to the corresponding MyObjectNode
    result.g1 = (dGeomID) collision.penetratingProxy->mUser1;
    result.g2 = (dGeomID) collision.penetratedProxy->mUser1;

    return result;
}

/*!
 * \brief converts d-collide Matrix to ODE matrix
 */
void convertDcollideMatrix2Ode(const dcollide::Matrix& dcMatrix, dMatrix3& output) {
    //note: dMatrix3 is a dReal[12]
    //ODE expects the values in row-major ordering
    output[0] = dcMatrix.getElement(0, 0);
    output[1] = dcMatrix.getElement(0, 1);
    output[2] = dcMatrix.getElement(0, 2);
    output[3] = 0;
    output[4] = dcMatrix.getElement(1, 0);
    output[5] = dcMatrix.getElement(1, 1);
    output[6] = dcMatrix.getElement(1, 2);
    output[7] = 0;
    output[8] = dcMatrix.getElement(2, 0);
    output[9] = dcMatrix.getElement(2, 1);
    output[10] = dcMatrix.getElement(2, 2);
    output[11] = 0;
}

/*
 * vim: et sw=4 ts=4
 */
