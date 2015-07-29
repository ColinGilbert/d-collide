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

#ifndef DCOLLIDE_TRIANGLE_PAIR_H
#define DCOLLIDE_TRIANGLE_PAIR_H


namespace dcollide {
    class Triangle;
    class Proxy;

    /*!
     * \brief Struct that contains a pair of \ref Triangle pointers
     *
     * This struct is provided for (middlephase) algorithms that produce a set
     * of \ref Triangle pairs as output. The \ref NarrowPhase is meant to take
     * objects of this struct and collide both triangles in it with each other
     * and produce \ref CollisionInfo as output (i.e. narrowphase results).
     */
    struct TrianglePair {
        inline TrianglePair();
        inline TrianglePair(Proxy* toplevelProxy1, Triangle* t1, Proxy* toplevelProxy2, Triangle* t2);

        /*!
         * \brief The \ref Proxy::getToplevelproxy that the \ref triangle1
         * belongs to
         */
        Proxy* toplevelProxy1;

        /*!
         * \brief The \ref Proxy::getToplevelproxy that the \ref triangle2
         * belongs to
         */
        Proxy* toplevelProxy2;

        Triangle* triangle1;
        Triangle* triangle2;
    };

    inline TrianglePair::TrianglePair() :
            toplevelProxy1(0),
            toplevelProxy2(0),
            triangle1(0),
            triangle2(0) {
    }

    inline TrianglePair::TrianglePair(Proxy* top1, Triangle* t1, Proxy* top2, Triangle* t2) :
            toplevelProxy1(top1),
            toplevelProxy2(top2),
            triangle1(t1),
            triangle2(t2) {
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
