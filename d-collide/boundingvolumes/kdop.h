/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-users@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,         *
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

#ifndef DCOLLIDE_KDOP_H
#define DCOLLIDE_KDOP_H

#include "boundingvolumes/boundingvolume.h"
#include "math/vector.h"
#include "real.h"

#include <list>

namespace dcollide {
    class Vertex;

    /*!
     * \brief k-DOP class
     * A k-DOP is a a bounding volume that is similar to an axis aligned box
     * (\ref Aabb). In fact a 6-DOP (i.e. k-DOP with k=6) is an Aabb.
     *
     * While an Aabb has 6 planes (or sides) that define the bounding volume,
     * k-DOPs have k planes. In theory they can arbitrarily oriented, but
     * usually certain special k-DOPs are used (so does this class).
     *
     * Usage of this class: create an instance and use one of 6,14,18 or 26 as
     * value for k (see constructor documentation).
     *
     * Then you can use \ref reset to create a k-DOP for a set of points and
     * \ref collidesWith to check for intersection with another k-DOP.
     *
     *
     * In the following we give a brief overview on k-DOPs as used here (in
     * particular regarding the values for k and the planes used for each k).
     * See the paper "Efficient Collision Detection Using
     * Bounding Volume Hierarchies of k-DOPs" for a detailed description of
     * k-DOPs.
     *
     * Usually k-DOPs use plane normals with components from {-1,0,1}
     * only (for performance reasons). Also usually you always use 2 planes
     * with opposite direction, i.e.
     * (0,0,-1) and (0,0,1) (this way you can use a simple [min;max] test)
     *
     * Usually k=6 (Aabb), k=14, k=18 or k=26 are being used, with the following normals/axes:
     *
     * k=6 (axis aligned bounding box, Aabb):
     * \li (-1,0,0) and (1,0,0) -> indices 0 and 3
     * \li (0,-1,0) and (0,1,0) -> indices 1 and 4
     * \li (0,0,-1) and (0,0,1) -> indices 2 and 5
     *
     * k=14 (Aabb + 8 diagonal planes that "cut off" some space of the corners):
     * \li (-1,0,0) and (1,0,0)   -> indices 0 and 7
     * \li (0,-1,0) and (0,1,0)   -> indices 1 and 8
     * \li (0,0,-1) and (0,0,1)   -> indices 2 and 9
     * \li (-1,-1,-1) and (1,1,1) -> indices 3 and 10
     * \li (-1,1,-1) and (1,-1,1) -> indices 4 and 11
     * \li (-1,-1,1) and (1,1,-1) -> indices 5 and 12
     * \li (-1,1,1) and (1,-1,-1) -> indices 6 and 13
     *
     * k=18 (Aabb + 12 diagonal planes that "cut off" some space of the edges):
     * \li (-1,0,0) and (1,0,0)  -> indices 0 and 9
     * \li (0,-1,0) and (0,1,0)  -> indices 1 and 10
     * \li (0,0,-1) and (0,0,1)  -> indices 2 and 11
     * \li (-1,-1,0) and (1,1,0) -> indices 3 and 12
     * \li (-1,0,-1) and (1,0,1) -> indices 4 and 13
     * \li (0,-1,-1) and (0,1,1) -> indices 5 and 14
     * \li (-1,1,0) and (1,-1,0) -> indices 6 and 15
     * \li (-1,0,1) and (1,0,-1) -> indices 7 and 16
     * \li (0,-1,1) and (0,1,-1) -> indices 8 and 17
     *
     * k=26 (Aabb and all additional normals from k=14 and k=18):
     * \li (-1,0,0) and (1,0,0)   -> indices 0 and 13
     * \li (0,-1,0) and (0,1,0)   -> indices 1 and 14
     * \li (0,0,-1) and (0,0,1)   -> indices 2 and 15
     * \li (-1,-1,-1) and (1,1,1) -> indices 3 and 16
     * \li (-1,1,-1) and (1,-1,1) -> indices 4 and 17
     * \li (-1,-1,1) and (1,1,-1) -> indices 5 and 18
     * \li (-1,1,1) and (1,-1,-1) -> indices 6 and 19
     * \li (-1,-1,0) and (1,1,0)  -> indices 7 and 20
     * \li (-1,0,-1) and (1,0,1)  -> indices 8 and 21
     * \li (0,-1,-1) and (0,1,1)  -> indices 9 and 22
     * \li (-1,1,0) and (1,-1,0)  -> indices 10 and 23
     * \li (-1,0,1) and (1,0,-1)  -> indices 11 and 24
     * \li (0,-1,1) and (0,1,-1)  -> indices 12 and 25
     *
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class Kdop : public BoundingVolume {
        private:
            int mK;

            /*!
             * Distance of the planes w.r.t. the origin.
             *
             * Different interpretation: min and/or max of the projection of all
             * points onto the plane normals (in fact it is the maximum w.r.t
             * \ref getPlaneNormal of the corresponding index)
             */
            real* mDistances;

        public:
            Kdop(int k = 14);
            Kdop(const Kdop& copy);
            ~Kdop();

            inline virtual BoundingVolumeType getVolumeType() const;

            int getK() const;
            real getDistanceOfPlaneToOrigin(int k) const;
            Vector3 getPlaneNormal(int k) const;
            inline Vector3 getSurroundingAabbExtents() const;

            inline Vector3 getSurroundingAabbMin() const;
            inline Vector3 getSurroundingAabbMax() const;
            virtual bool collidesWith(const BoundingVolume& other) const;
            void mergeWith( const BoundingVolume* otherBV);
            void translate(const Vector3& translateBy);

            void adjustToTriangle(const Triangle* triangle);

        protected:
            bool collidesWithInternal(const Kdop& other) const;

            void adjustToBox(const Matrix* worldState, const Box* box);
            void adjustToCylinder(const Matrix* worldState,
                                  const Cylinder* meshPart);
            void adjustToCone(const Matrix* worldState, const Cone* cone);
            void adjustToSphere(const Matrix* worldState, const Sphere* sphere);
            void adjustToWedge(const Matrix* worldState, const Wedge* meshPart);

            virtual void reset(const std::list<Vector3>& points);
            virtual void reset(const std::vector<Vertex*>& vertices);

        private:
            friend class KDOPTest;
    };

    BoundingVolumeType Kdop::getVolumeType() const {
        return BV_TYPE_KDOP;
    }

    /*!
     * \return The size of this Kdop, i.e. the maximal diagonal vector
     */
    // FIXME: untested!
    Vector3 Kdop::getSurroundingAabbExtents() const {
        const int k2 = mK / 2;
        // "size" (as in width, height depth) on the three main axes, i.e. the
        // extents of an AABB around this k-DOP.
        return Vector3(mDistances[k2] - mDistances[0],
                mDistances[k2 + 1] - mDistances[1],
                mDistances[k2 + 2] - mDistances[2]);
    }


    inline Vector3 Kdop::getSurroundingAabbMin() const {
        return Vector3(mDistances[0],
                mDistances[1],
                mDistances[2]);
    }
    inline Vector3 Kdop::getSurroundingAabbMax() const {
        const int k2 = mK / 2;
        return Vector3(mDistances[k2],
                mDistances[k2 + 1],
                mDistances[k2 + 2]);
    }

}

#endif

/*
 * vim: et sw=4 ts=4
 */
