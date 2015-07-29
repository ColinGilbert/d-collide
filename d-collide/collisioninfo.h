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

#ifndef DCOLLIDE_COLLISIONINFO_H
#define DCOLLIDE_COLLISIONINFO_H

#include "math/vector.h"

namespace dcollide {
    class Proxy;
    class Triangle;
    class Vertex;
    //-----------structs------------
    /*! \brief information about a collision point
     *  This struct gives detailed information about two colliding proxies.
     *  When 2 \ref Poxies collide, they share at least one point in space.
     *  For each point, a CollisionInfo struct is generated.
     * 
     * Convention: If the penetrating proxy is moved by \ref penetrationDepth 
     * units along the penetration normal, the penetration depth is reduced
     * to zero. 
     *
     */
    struct CollisionInfo {
        inline CollisionInfo();

        /*! \brief the (approx.) point where the 2 proxies would have collided
         *  The point in space the two colliding proxies share. Due to discrete
         *  time, we can only approximate this point by some kind of 
         * trajectorie-backtracking
         */
        Vector3 collisionPoint;

        /*! \brief the (approx.) penetration depth
         *  Describes how deep the proxies penetrate each other along the
         *  direction of their relative movement.
         *  Obviously, this can only be approximated.
         */
        real penetrationDepth;

        /*! \brief surface normal of the penetrated proxy at the collision point
         */
        Vector3 normal;

        /*! \brief the penetrating proxy
         *  the penetrating proxy. In this interpretation, it actively 
         *  collides with the other Proxy, which is beeing hit by this one
         */
        Proxy *penetratingProxy;

        /*! \brief the proxy hit by Proxy1
         *  the penetrated proxy.
         */
        Proxy *penetratedProxy;
        
        /*! \brief the penetrating triangle (can be NULL)
         *  only set on collisions with meshes
         * 
         * \deprecated, replaced by penetratingVertex
         */
        Triangle* penetratingTriangle;
        /*! \brief the penetrating vertex (can be NULL)
         * 
         * this is used for mesh-mesh collisions only.
         */
        Vertex* penetratingVertex;
        
        /*! \brief the penetrated triangle (can be NULL)
         *  only set on collisions with meshes
         */
        Triangle* penetratedTriangle;
    };

    inline CollisionInfo::CollisionInfo() {
        penetratingProxy = 0;
        penetratedProxy = 0;
        penetrationDepth = 0.0;
        penetratingTriangle = 0;
        penetratedTriangle = 0;
        penetratingVertex = 0;
    }
}

#endif // DCOLLIDE_COLLISIONINFO_H
/*
 * vim: et sw=4 ts=4
 */
