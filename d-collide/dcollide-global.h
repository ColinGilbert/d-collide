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

#ifndef DCOLLIDE_DCOLLIDE_GLOBAL_H
#define DCOLLIDE_DCOLLIDE_GLOBAL_H

/*!
 * This file provides enum or similar values that should be available globally.
 *
 * This file is not meant to declare any classes or functions.
 */


namespace dcollide {
    /*!
     * \brief available \ref Proxy Types
     *
     * The type of the \ref Proxy describes its behaviour. It is advised to
     * use the most restrictive type whenever possible (for performance
     * reasons). For example a proxy of type \p PROXYTYPE_FIXED can
     * employ more optimizations than a proxy of type \p PROXYTYPE_RIGID
     * because the internal algorithms can assume that the proxy will
     * never move (and thus could make heavy use of precalculations).
     */
    enum ProxyTypes {
        /*!
         * This proxies may move and rotate
         */
        PROXYTYPE_RIGID                     = 1,

        /*!
         * This proxies may move, rotate and deform
         */
        PROXYTYPE_DEFORMABLE                = 2,

        /*!
         * These proxies report self-collisions
         */
        PROXYTYPE_SELFCOLLIDABLE            = 4,

        /*!
         * Proxies, which have this type are unable to move.
         * This implies PROXYTYPE_RIGID.
         */
        // AB: note that fixed deformable makes no sense, since we can
        //     always move the proxy by deforming it.
        PROXYTYPE_FIXED                     = 8,

        /*!
         * These proxies are ignored completely by the collision
         * detection.
         */
        PROXYTYPE_IGNORE                    = 16,

        /*!
         * These proxies have a shape which has a closed hull and therefore
         * represents a volume object.
         *
         * All \ref Proxy objects with a \ref Shape other than \ref Mesh and
         * \ref MeshPart should have this flag set (should be set automatically
         * by the \ref Proxy constructor). For \ref Mesh objects, the user needs
         * to set this flag manually, if desired.
         *
         * Despite most other flags, this flag can be set in a child \ref Proxy,
         * even if it is not set in its parent (and vice versa).
         */
        PROXYTYPE_CLOSEDHULL                = 32,

        /*!
         * These proxies have a convex shape which is a special case of a closed
         * hull. all points on lines between points inside the shape are inside
         * the shape too.
         *
         * All \ref Proxy objects with a \ref Shape other than \ref Mesh and
         * \ref MeshPart should have this flag set (should be set automatically
         * by the \ref Proxy constructor). For \ref Mesh objects, the user needs
         * to set this flag manually, if desired.
         *
         * Despite most other flags, this flag can be set in a child \ref Proxy,
         * even if it is not set in its parent (and vice versa).
         */
        PROXYTYPE_CONVEX                    = 64
    };


    enum BoundingVolumeType {
        BV_TYPE_AABB                        = 1,
        BV_TYPE_KDOP                        = 2,
        BV_TYPE_SPHERE                      = 3,
        BV_TYPE_OBB                        = 4
    };


    enum BroadPhaseType {
        /*!
         * Puts all objects in a hierarchical grid and performs the collision
         * test only on objects that are in the same grid cell.
         */
        BROADPHASE_TYPE_HIERARCHICALGRID         = 1,

        /*!
         * Brute-Force method, tests succesivly all objects pairwise
         * for an actual collision.
         */
        BROADPHASE_TYPE_BRUTEFORCE     = 2
    };


    /*!
     * \brief Algorithms for collision detection with deformable proxies
     * supported by d-collide. See also \ref
     * WorldParameters::addDeformableAlgorithm()
     */
    enum DeformableAlgorithmType {
        DEFORMABLE_TYPE_SURFACE_HIERARCHY       = 1,
        DEFORMABLE_TYPE_SPATIAL_HASH            = 2,
        DEFORMABLE_TYPE_BV_HIERARCHY            = 3
    };
}

#endif
/*
 * vim: et sw=4 ts=4
 */
