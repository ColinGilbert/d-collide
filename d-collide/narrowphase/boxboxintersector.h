/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
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



#ifndef DCOLLIDE_BOXBOXINTERSECTOR_H
#define DCOLLIDE_BOXBOXINTERSECTOR_H

#include "narrowphasestrategy.h"
#include "boundingvolumecollision.h"

#include "real.h"

#include <list>
#include <vector>


namespace dcollide {

    //Forward declarations
    class Box;
    class Matrix;
    class BvhNode;
    class Vertex;
    class BoundingVolume;
    struct CollisionInfo;
    class Proxy;
    class Vector3;



    /*!
     * \brief Class to calculate detailed intersections between boxes
     */
    class BoxBoxIntersector {
        public:
            BoxBoxIntersector();
            ~BoxBoxIntersector();

            std::list<CollisionInfo> getIntersectionFast(
                    const Matrix& currentState1, Box* box1, 
                    const Matrix& currentState2, Box* box2) const;

            std::list<CollisionInfo> getIntersectionAccurate(
                    Box* box1,const Matrix& previousState1,Box* box2,
                    const Matrix& previousState2) const;
        private:

            inline bool overlap(float min1, float max1, float min2, float max2) const;
            void computeSpan(const Vector3* vertices, const Vector3& axis, real& min, real&
                    max) const;

            bool getFlatIntersection(Vector3* vertsFlat1, int n1, 
                    Vector3* vertsFlat2, int n2,
                    Vector3 (& retVertices)[4], int& nVerts) const;

            void getVertsMinMax(Vector3* verts, int nVerts, real (& min)[3], 
                    real (& max)[3]) const;

            int getLineIntersectionPoints(Vector3* vertsFlat1, int n1,
                    Vector3* vertsFlat2, int n2,
                    Vector3 (& retVertices)[2]) const;

            void findSupportingFeature( const Matrix& state,
                                        const Vector3* boxVertices,
                                        const Vector3& normal, int& n,
                                        Vector3 (&vertices)[4]) const;

    };

    /*!
     * \brief Determine if given intervals overlap.
     */
    inline bool BoxBoxIntersector::overlap(float min1, float max1,
                              float min2, float max2) const {
        return !(min1 > max2 || max1 < min2);
    }

}

#endif // DCOLLIDE_BOXBOXINTERSECTOR_H
/*
 * vim: et sw=4 ts=4
 */
