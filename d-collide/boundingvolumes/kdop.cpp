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

#include "boundingvolumes/aabb.h"
#include "boundingvolumes/kdop.h"
#include "shapes/shapes.h"
#include "exceptions/exception.h"
#include "debug.h"
#include "debugstream.h"

#include <iostream>
#include <assert.h>


namespace dcollide {

    /*!
     * \internal
     *
     * Implements \ref Kdop::reset for k=6.
     *
     * This method can be used for k=14, k=18 and k=26 as well (since they also use
     * the planes of k=6)
     */
    template<typename Container>
    static void reset6(const Container& container, real* distances, int k) {
        typename Container::const_iterator it = container.begin();
        typename Container::const_iterator end = container.end();

        // AB: reset() already makes sure that points.empty() != true, so this
        //     is actually redundant!
        assert(it != end);

        const Vector3& p0 = BoundingVolume::retrieveVector3FromIterator(it);
        real minX = p0.getX();
        real maxX = minX;
        real minY = p0.getY();
        real maxY = minY;
        real minZ = p0.getZ();
        real maxZ = minZ;

        ++it;
        for (; it != end; ++it) {
            const Vector3& point = BoundingVolume::retrieveVector3FromIterator(it);
            minX = std::min(minX, point.getX());
            maxX = std::max(maxX, point.getX());
            minY = std::min(minY, point.getY());
            maxY = std::max(maxY, point.getY());
            minZ = std::min(minZ, point.getZ());
            maxZ = std::max(maxZ, point.getZ());
        }

        distances[0] = minX;
        distances[1] = minY;
        distances[2] = minZ;

        // AB: we don't use 4..6 but rather mK/2 + 0..2, so that we can use this
        //     method for k=6,14,18 and 26.
        distances[k/2 + 0] = maxX;
        distances[k/2 + 1] = maxY;
        distances[k/2 + 2] = maxZ;
    }

    /*!
     * \internal
     *
     * Calculates the distances of the planes of a 14-DOP for point \p p and returns
     * them in the parameters.
     *
     * In other words the
     * returned values will tell you how to place the planes so that they cover
     * point \p p.
     *
     * Note that distances 0..2 (Aabb planes) are \em NOT calculated here.
     *
     * About the parameter names: they'll suck no matter how I name them, so I won't
     * even try. Each of the d3, d4, d5, d6 parameter will return the required
     * distance of the plane to the origin.
     */
    static inline void getDistances14(const Vector3& p,
              real& d3, real& d4, real& d5, real& d6) {
        d3 = p.getX() + p.getY() + p.getZ();
        d4 = p.getX() - p.getY() + p.getZ();
        d5 = p.getX() + p.getY() - p.getZ();
        d6 = p.getX() - p.getY() - p.getZ();
    }
    /*!
     * \internal
     *
     * Like \ref getDistances14 but for a 18-DOP
     */
    static inline void getDistances18(const Vector3& p,
              real& d3, real& d4, real& d5, real& d6, real& d7, real& d8) {
        d3 = p.getX() + p.getY();
        d4 = p.getX()            + p.getZ();
        d5 =            p.getY() + p.getZ();
        d6 = p.getX() - p.getY()           ;
        d7 = p.getX()            - p.getZ();
        d8 =            p.getY() - p.getZ();
    }

    /*!
     * \internal
     *
     * This method implements \ref Kdop::reset for k=14. Aabb planes are ignored by
     * this method (i.e. also call \ref reset6)
     * \param min An array of size 7. The minimum values will be placed here.
     *            Index 0..2 will \em not be touched by this method.
     * \param max An array of size 7. The maximum values will be placed here.
     *            Index 0..2 will \em not be touched by this method.
     * \param points A list of points whose min/max values should be calculated.
     *        Must \em not be empty.
     */
    template<typename Container>
    static void calculateMinMax14(const Container& container,
                           real* min, real* max) {
        typename Container::const_iterator it = container.begin();
        typename Container::const_iterator end = container.end();

        // AB: caller already makes sure that points.empty() != true, so this
        //     is actually redundant!
        assert(it != end);

        getDistances14(BoundingVolume::retrieveVector3FromIterator(it), min[3], min[4], min[5], min[6]);
        max[3] = min[3];
        max[4] = min[4];
        max[5] = min[5];
        max[6] = min[6];

        ++it;
        for (; it != end; ++it) {
            real d3;
            real d4;
            real d5;
            real d6;
            getDistances14(BoundingVolume::retrieveVector3FromIterator(it), d3, d4, d5, d6);
            min[3] = std::min(min[3], d3);
            min[4] = std::min(min[4], d4);
            min[5] = std::min(min[5], d5);
            min[6] = std::min(min[6], d6);

            max[3] = std::max(max[3], d3);
            max[4] = std::max(max[4], d4);
            max[5] = std::max(max[5], d5);
            max[6] = std::max(max[6], d6);
        }

    }

    /*!
     * \internal
     *
     * This method implements \ref Kdop::reset for k=18. Aabb planes are ignored by
     * this method (i.e. also call \ref reset6)
     * \param min An array of size 7. The minimum values will be placed here.
     *            Index 0..2 will \em not be touched by this method.
     * \param max An array of size 7. The maximum values will be placed here.
     *            Index 0..2 will \em not be touched by this method.
     * \param points A list of points whose min/max values should be calculated.
     *        Must \em not be empty.
     */
    template<typename Container>
    static void calculateMinMax18(const Container& container,
                           real* min, real* max) {
        typename Container::const_iterator it = container.begin();
        typename Container::const_iterator end = container.end();

        // AB: caller already makes sure that points.empty() != true, so this
        //     is actually redundant!
        assert(it != end);

        getDistances18(BoundingVolume::retrieveVector3FromIterator(it), min[3], min[4], min[5], min[6], min[7], min[8]);
        max[3] = min[3];
        max[4] = min[4];
        max[5] = min[5];
        max[6] = min[6];
        max[7] = min[7];
        max[8] = min[8];

        ++it;
        for (; it != end; ++it) {
            real d3;
            real d4;
            real d5;
            real d6;
            real d7;
            real d8;
            getDistances18(BoundingVolume::retrieveVector3FromIterator(it), d3, d4, d5, d6, d7, d8);
            min[3] = std::min(min[3], d3);
            min[4] = std::min(min[4], d4);
            min[5] = std::min(min[5], d5);
            min[6] = std::min(min[6], d6);
            min[7] = std::min(min[7], d7);
            min[8] = std::min(min[8], d8);

            max[3] = std::max(max[3], d3);
            max[4] = std::max(max[4], d4);
            max[5] = std::max(max[5], d5);
            max[6] = std::max(max[6], d6);
            max[7] = std::max(max[7], d7);
            max[8] = std::max(max[8], d8);
        }
    }


    /*!
     * Internal implementation of \ref Kdop::reset.
     *
     * This function is implemented as a template method so that
     * std::list<Vector3> as well as std::vector<Vertex*> (as returned by \ref
     * Mesh::getVertices and \ref MeshPart::getVertices) can be used with it.
     *
     * Note in particular that if \ref Vertex is used with this function, \ref
     * Vertex::getWorldPosition is used to retrieve the position of the vertex,
     * wheres with \ref Vector3 the position can be used directly.
     */
    template<typename Container>
    static inline void resetInternal(const Container& points, real* distances, const int k) {
        if (points.empty()) {
            for (int i = 0; i < k; i++) {
                distances[i] = 0.0f;
            }
            return;
        }

        if (k == 6) {
            dcollide::reset6(points, distances, k);
        } else if (k == 14) {
            dcollide::reset6(points, distances, k);

            // note: 0..2 are totally ignored here!
            real min[7];
            real max[7];

            dcollide::calculateMinMax14(points, min, max);

            for (int i = 3; i < 7; i++) {
                distances[i] = min[i];
                distances[7 + i] = max[i];
            }
        } else if (k == 18) {
            dcollide::reset6(points, distances, k);

            // note: 0..2 are totally ignored here!
            real min[9];
            real max[9];

            dcollide::calculateMinMax18(points, min, max);

            for (int i = 3; i < 9; i++) {
                distances[i] = min[i];
                distances[9 + i] = max[i];
            }
        } else if (k == 26) {
            dcollide::reset6(points, distances, k);

            // note: 0..2 are totally ignored here!
            real min[26];
            real max[26];

            dcollide::calculateMinMax14(points, min, max);
            dcollide::calculateMinMax18(points, &min[4], &max[4]);

            for (int i = 3; i < 13; i++) {
                distances[i] = min[i];
                distances[13 + i] = max[i];
            }
        } else {
            throw Exception("unhandled value of Kdop::mK");
        }
    }


    /*!
     *  \brief Constructs an empty k-DOP with given k
     *  \param k The number of planes to be used in this k-DOP. Only the values
     *  6 (Aabb), 14, 18 and 26 are supported! Any other value of k will be
     *  reset to k=14.
     */
    Kdop::Kdop(int k) {
        if (k != 6 && k != 14 && k != 18 && k != 26) {
            // unsupported value for k. default to 14.
            k = 14;
        }
        mK = k;

        // we need one min value (0..k/2) and one max value (k/2..k-1) for every
        // unique plane orientation (i.e. for k/2 planes)
        mDistances = new real[mK];
        for (int i = 0; i < mK; i++) {
            mDistances[i] = 0.0f;
        }
    }

    /*!
     * \brief Constructs an Kdop as copy of given kdop
     */
    Kdop::Kdop(const Kdop& copy) : BoundingVolume() {
        mK = copy.mK;
        mDistances = new real[mK];
        for (int i = 0; i < mK; i++) {
            mDistances[i] = copy.mDistances[i];
        }
    }

    Kdop::~Kdop() {
        delete[] mDistances;
    }

    /*!
     * \brief Calculates a new KDOP from given points
     *
     * Calculates a new KDOP from \p points.
     */
    void Kdop::reset(const std::list<Vector3>& points) {
        dcollide::resetInternal(points, mDistances, mK);
    }

    void Kdop::reset(const std::vector<Vertex*>& vertices) {
        dcollide::resetInternal(vertices, mDistances, mK);
    }

    /*!
     * \return The number of planes (k) used internally. Will be the number
     * provided to the c'tor, unless an unsupported number was used.
     */
    int Kdop::getK() const {
        return mK;
    }

    void Kdop::mergeWith(const BoundingVolume* otherBV) {
        if (!otherBV) {
            //TODO do we need an exception here, ore is it ok to not merge?
            return;
        }
        if (otherBV->getVolumeType() != getVolumeType()) {
            throw TypeMismatchException("cannot merge BoundingVolumes of different types");
        }
        const Kdop* other = static_cast<const Kdop*>(otherBV);
        if (mK != other->mK) {
            // TODO: exception?
            // TODO: implement?
            return;
        }
        for (int i = 0; i < mK / 2; i++) {
            mDistances[i] = std::min(mDistances[i], other->mDistances[i]);
        }
        for (int i = mK / 2; i < mK; i++) {
            mDistances[i] = std::max(mDistances[i], other->mDistances[i]);
        }
    }

    /*!
     * This function is mainly intended for debugging purposes. Along with \ref
     * getPlaneNormal it can be used to draw the k-DOP onto the screen.
     *
     * Note: this method checks for valid parameter (k in range). This for
     * internal use it should generally be avoided for performance reasons.
     *
     * @return The distance of plane k to the origin or 0 if k is out of range
     * (see \ref getK). See also \ref getPlaneNormal.
     */
    real Kdop::getDistanceOfPlaneToOrigin(int k) const {
        if (k < 0 || k >= mK) {
            return 0.0f;
        }
        if (k >= mK/2) {
            return (real) (mDistances[k] * -1.0);
        }
        return mDistances[k];
    }

    /*!
     * \return The plane normal of plane \p k or (0,0,0) if an invalid value was
     * given.
     *
     * WARNING: this method calculates the plane normals on the fly and thus is
     * relatively slow. It should not be used for collision detection (unless
     * the returned values are cached). Instead this method is meant to give
     * code outside of this class debugging means or to display this k-DOP on
     * the screen (along with \ref getDistanceOfPlaneToOrigin).
     */
    // AB: pretty complicated function.
    //     if we actually use this internally (for non-debugging purposes), we
    //     should keep a static array in memory that stores the plane normals
    //     for k==6,14,18 and 26.
    Vector3 Kdop::getPlaneNormal(int k) const {
        if (k < 0 || k >= mK) {
            return Vector3(0.0f, 0.0f, 0.0f);
        }
        // AB: internally we use 0..(mK/2)-1 for unique directions and (mK/2)..mK-1
        //     for the opposite directions.
        //     -> calculate vector based on modulo (mK/2) and if k>=(mK/2) multiply
        //        the plane normal by -1 afterwards.
        Vector3 planeNormal;
        bool flipDirection = false;
        if (k >= mK / 2) {
            flipDirection = true;
            k = k - (mK / 2);
        }
        if (k < 3) { // all possible values of mK use the same normals in 0..2
            if (k == 0) {
                planeNormal = Vector3(-1.0f, -0.0f, -0.0f);
            } else if (k == 1) {
                planeNormal = Vector3(-0.0f, -1.0f, -0.0f);
            } else if (k == 2) {
                planeNormal = Vector3(-0.0f, -0.0f, -1.0f);
            }
        } else {
            // k==14 and k==26 are identical for 0..6 (i.e. for all k/2 unique
            //                                         planes of k==14)
            // all k==18 normals (except the first 3 ones) are also in k==26 but
            // with an offset of 7.
            // -> add (7-3) to k (since the first 3 planes are equal in k==18
            //    and k==26)
            if (mK == 18) {
                k += (7-3);
            }

            // now we can use indices of mK==26.
            // since 0..2 are already handled (and we have k<=mK/2) it suffices
            // to use 3..12
            switch (k) {
                case 3:
                    planeNormal = Vector3(-1.0f, -1.0f, -1.0f);
                    break;
                case 4:
                    planeNormal = Vector3(-1.0f, 1.0f, -1.0f);
                    break;
                case 5:
                    planeNormal = Vector3(-1.0f, -1.0f, 1.0f);
                    break;
                case 6:
                    planeNormal = Vector3(-1.0f, 1.0f, 1.0f);
                    break;
                case 7:
                    planeNormal = Vector3(-1.0f, -1.0f, 0.0f);
                    break;
                case 8:
                    planeNormal = Vector3(-1.0f, 0.0f, -1.0f);
                    break;
                case 9:
                    planeNormal = Vector3(0.0f, -1.0f, -1.0f);
                    break;
                case 10:
                    planeNormal = Vector3(-1.0f, 1.0f, 0.0f);
                    break;
                case 11:
                    planeNormal = Vector3(-1.0f, 0.0f, 1.0f);
                    break;
                case 12:
                    planeNormal = Vector3(0.0f, -1.0f, 1.0f);
                    break;
                default:
                    // TODO: throw exception!
                    std::cout << "ERROR: invalid index in " << __FILE__ << " at line " << __LINE__ << " k=" << k << std::endl;
                    break;
            }
        }

        if (flipDirection) {
            return planeNormal * -1.0f;
        }

        return planeNormal;
    }

    /*!
     * \brief Collides the given bounding volume with this bounding volume?
     * 
     * This method return true if the given bounding volume collides with this
     * bounding volume. If a bounding volume of a different type is given, than
     * the surrounding Aabb are tested for collision.
     * 
     * TODO: Implement correct/faster collision detection methods for
     *       collisions between different types of bounding volumes.
     */
    bool Kdop::collidesWith(const BoundingVolume& other) const {
        const BoundingVolume* otherPointer = &other;
        
        if (other.getVolumeType() == getVolumeType()) {
            const Kdop* kdopPointer = static_cast<const Kdop*>(otherPointer);
            return collidesWithInternal(*kdopPointer);
        } else {
            Aabb thisAabb(getSurroundingAabbMin(), getSurroundingAabbMax());
            return
                thisAabb.collidesWith(
                    Aabb(otherPointer->getSurroundingAabbMin(),
                         otherPointer->getSurroundingAabbMax()));
        }
    }

    /*!
     * \brief Tests for intersection with another k-DOP
     *
     * Bot k-DOPs (this and \p other) must use the same value for k, see \ref
     * getK.
     *
     * \return TRUE If this KDOP intersects with \p other. Otherwise FALSE.
     */
    bool Kdop::collidesWithInternal(const Kdop& other) const {
        if (mK != other.mK) {
            // TODO: exception
            return false;
        }
        int k2 = mK / 2;
        for (int i = 0; i < k2; i++) {
            real min = mDistances[i];
            real max = mDistances[k2 + i];
            real min2 = other.mDistances[i];
            real max2 = other.mDistances[k2 + i];
            if (min > max2) {
                return false;
            }
            if (max < min2) {
                return false;
            }
        }
        return true;
    }


    void Kdop::adjustToSphere(const Matrix* worldState, const Sphere* sphere) {
        Vector3 center;
        worldState->transform(&center, Vector3(0.0, 0.0, 0.0));
        const float r = sphere->getRadius();

        // min and max x,y,z
        mDistances[0] = center.getX() - r;
        mDistances[1] = center.getY() - r;
        mDistances[2] = center.getZ() - r;
        mDistances[mK / 2 + 0] = center.getX() + r;
        mDistances[mK / 2 + 1] = center.getY() + r;
        mDistances[mK / 2 + 2] = center.getZ() + r;

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)

        if (mK == 6) {
            // nothing else to do
            return;
        } else if (mK == 14) {
            /*
             * First of all: you can easily see that for a normal (a,b,c) with
             * a,b,c element of {1,-1} (not 0), the maximal and minimal points
             * of a sphere are the same as for the larges box completely inside
             * the sphere.
             * a half-edge of such a box would have length:
             * r = sqrt(x*x + y*y + z*z) = sqrt(w*w + w*w + w*w) = sqrt(3*w*w)
             * => w = r/sqrt(3)
             */
            // TODO: optimization: hardcode this value
            const real sqrt3 = (real) sqrt(3.0);

            const real w = (real)(r/sqrt3);
            const real d = 3 * w;

            mDistances[3] = center.getX() + center.getY() + center.getZ() - d;
            mDistances[4] = center.getX() - center.getY() + center.getZ() - d;
            mDistances[5] = center.getX() + center.getY() - center.getZ() - d;
            mDistances[6] = center.getX() - center.getY() - center.getZ() - d;

            mDistances[mK / 2 + 3] = center.getX() + center.getY() + center.getZ() + d; // normal: (1,1,1)
            mDistances[mK / 2 + 4] = center.getX() - center.getY() + center.getZ() + d; // normal: (1,-1,1)
            mDistances[mK / 2 + 5] = center.getX() + center.getY() - center.getZ() + d; // normal: (1,1,-1)
            mDistances[mK / 2 + 6] = center.getX() - center.getY() - center.getZ() + d; // normal: (1,-1,-1)
        } else if (mK == 18) {
            /*
             * For normals (a,b,c) with exactly two of a,b,c being element of
             * {1,-1} and the other being 0, we have essentially a simple 2D
             * circle.
             *
             * The maximal point of that circle is at 45 degree. so we have
             * x = r * cos(45)
             * y = r * sin(45)
             * and since cos(45)==sin(45) we can simplify this to cos(45).
             */
            // TODO: optimization: hardcode this value
            const real cos45 = (real) cos(DEG2RAD * 45.0);

            // FIXME: is this simplification (using cos 45 only) allowed?
            const real d = (real) (r * cos45 + r * cos45);

            mDistances[3] = center.getX() + center.getY() - d;
            mDistances[4] = center.getX() + center.getZ() - d;
            mDistances[5] = center.getY() + center.getZ() - d;
            mDistances[6] = center.getX() - center.getY() - d;
            mDistances[7] = center.getX() - center.getZ() - d;
            mDistances[8] = center.getY() - center.getZ() - d;

            mDistances[mK / 2 + 3] = center.getX() + center.getY() + d; // normal: (1,1,0)
            mDistances[mK / 2 + 4] = center.getX() + center.getZ() + d; // normal: (1,0,1)
            mDistances[mK / 2 + 5] = center.getY() + center.getZ() + d; // normal: (0,1,1)
            mDistances[mK / 2 + 6] = center.getX() - center.getY() + d; // normal: (1,-1,0)
            mDistances[mK / 2 + 7] = center.getX() - center.getZ() + d; // normal: (1,0,-1)
            mDistances[mK / 2 + 8] = center.getY() - center.getZ() + d; // normal: (0,1,-1)
        } else if (mK == 26) {
            const real sqrt3 = (real) sqrt(3.0);
            const real w = (real) (r/sqrt3);

            const real cos45 = (real) (cos(DEG2RAD * 45.0));
            const real d14 = (real) (3 * w);
            const real d18 = (real) (r * cos45 + r * cos45);

            mDistances[3] = center.getX() + center.getY() + center.getZ() - d14;
            mDistances[4] = center.getX() - center.getY() + center.getZ() - d14;
            mDistances[5] = center.getX() + center.getY() - center.getZ() - d14;
            mDistances[6] = center.getX() - center.getY() - center.getZ() - d14;
            mDistances[7] = center.getX() + center.getY() - d18;
            mDistances[8] = center.getX() + center.getZ() - d18;
            mDistances[9] = center.getY() + center.getZ() - d18;
            mDistances[10] = center.getX() - center.getY() - d18;
            mDistances[11] = center.getX() - center.getZ() - d18;
            mDistances[12] = center.getY() - center.getZ() - d18;

            mDistances[mK / 2 + 3] = center.getX() + center.getY() + center.getZ() + d14;
            mDistances[mK / 2 + 4] = center.getX() - center.getY() + center.getZ() + d14;
            mDistances[mK / 2 + 5] = center.getX() + center.getY() - center.getZ() + d14;
            mDistances[mK / 2 + 6] = center.getX() - center.getY() - center.getZ() + d14;
            mDistances[mK / 2 + 7] = center.getX() + center.getY() + d18;
            mDistances[mK / 2 + 8] = center.getX() + center.getZ() + d18;
            mDistances[mK / 2 + 9] = center.getY() + center.getZ() + d18;
            mDistances[mK / 2 + 10] = center.getX() - center.getY() + d18;
            mDistances[mK / 2 + 11] = center.getX() - center.getZ() + d18;
            mDistances[mK / 2 + 12] = center.getY() - center.getZ() + d18;
        }

#undef DEG2RAD
#undef RAD2DEG

    }

    void Kdop::adjustToBox(const Matrix* worldState, const Box* box) {

        // every corner of the box can be the new minimum/maximum point, so we
        // need to transform each point by our matrix.
        // for this we first find all vertices of the box, then transform all of
        // them and finally find the min/max values from those aabbVertices.

        const Vector3& dimension = box->getDimension();
        std::vector<Vector3> boxVertices(8);
        boxVertices[0] = Vector3(            0.0,               0.0,              0.0);
        boxVertices[1] = Vector3(            0.0,               0.0, dimension.getZ());
        boxVertices[2] = Vector3(            0.0,  dimension.getY(),              0.0);
        boxVertices[3] = Vector3(            0.0,  dimension.getY(), dimension.getZ());
        boxVertices[4] = Vector3(dimension.getX(),              0.0,              0.0);
        boxVertices[5] = Vector3(dimension.getX(),              0.0, dimension.getZ());
        boxVertices[6] = Vector3(dimension.getX(), dimension.getY(),              0.0);
        boxVertices[7] = Vector3(dimension.getX(), dimension.getY(), dimension.getZ());

        std::list<Vector3> aabbVertices;
        for (int i = 0; i < 8; i++) {
            Vector3 transformedVector;
            worldState->transform(&transformedVector, boxVertices[i]);
            aabbVertices.push_back(transformedVector);
        }
        reset(aabbVertices);
    }

    void Kdop::adjustToCone(const Matrix* worldState, const Cone* cone) {
        // AB: TODO: this essentially calculates an AABB, however for k-DOPs,
        //           much better approximations should be possible.
        //           if we ever make extensive use of cones and k-DOPs, we
        //           should write one!
        resetToApproximatedAabbOfCone(worldState, cone);
    }

    void Kdop::adjustToCylinder(const Matrix* worldState,
                                const Cylinder* cylinder) {
        // AB: TODO: this essentially calculates an AABB, however for k-DOPs,
        //           much better approximations should be possible.
        //           if we ever make extensive use of cylinders and k-DOPs, we
        //           should write one!
        resetToApproximatedAabbOfCylinder(worldState, cylinder);
    }

    void Kdop::adjustToWedge(const Matrix* worldState, const Wedge* wedge) {
        const Vector3* vertices = const_cast<Wedge*>(wedge)->getVertices();

        std::list<Vector3> aabbVertices;
        for (int i = 0; i < 6; i++) {
            aabbVertices.push_back(vertices[i]);
        }
        reset(aabbVertices);
    }

    void Kdop::adjustToTriangle( const Triangle* triangle) {
        const dcollide::array<Vertex*,3>& vertices = triangle->getVertices();

        std::list<Vector3> points;
        points.push_back(vertices[0]->getWorldPosition());
        points.push_back(vertices[1]->getWorldPosition());
        points.push_back(vertices[2]->getWorldPosition());
        reset(points);
    }


    // TODO: cppunit test!
    // -> test translations on all plane directions (positive and negative)
    // --> possibly use a "quick and dirty" test: simply use getPlaneNormal().
    //     we don't need to test if the resulting k-dops have the correct values
    //     (though that would be the ideal test):
    //     * create two proxies (NOT boxes, otherwise k-DOP==AABB. possibly spheres)
    //       at the same position
    //     * move one of them along all of the plane directions (positive and
    //       negative, i.e. mK different directions)
    //     * check for collisions
    //     -> every check should
    //     a) move the proxy as far as possible, where it still has collisions
    //     b) move the proxy as little as possible where there is no collision
    //        anymore (i.e. a little bit further than a)
    //     planes and then checked for collisions with another proxy which
    //     remains on the "center" position (from where the other proxy is
    //     moved) and then 
#ifdef __GNUC__
#warning TODO: cppunit test for Kdop::translate()
#endif
    void Kdop::translate(const Vector3& translateBy) {
        const int k2 = mK/2;
        const real tx = translateBy.getX();
        const real ty = translateBy.getY();
        const real tz = translateBy.getZ();

        // AABB planes are easy:
        mDistances[0] += tx;
        mDistances[k2 + 0] += tx;
        mDistances[1] += ty;
        mDistances[k2 + 1] += ty;
        mDistances[2] += tz;
        mDistances[k2 + 2] += tz;

        if (mK == 14 || mK == 26) {
            mDistances[3] += (tx + ty + tz);
            mDistances[k2 + 3] += (tx + ty + tz);
            mDistances[4] += (tx - ty + tz);
            mDistances[k2 + 4] += (tx - ty + tz);
            mDistances[5] += (tx + ty - tz);
            mDistances[k2 + 5] += (tx + ty - tz);
            mDistances[6] += (tx - ty - tz);
            mDistances[k2 + 6] += (tx - ty - tz);
        } else if (mK == 18 || mK == 26) {
            const int offset = (mK == 26) ? 4 : 0;
            mDistances[offset + 3] += (tx + ty);
            mDistances[offset + k2 + 3] += (tx + ty);
            mDistances[offset + 4] += (tx + tz);
            mDistances[offset + k2 + 4] += (tx + tz);
            mDistances[offset + 5] += (ty + tz);
            mDistances[offset + k2 + 5] += (ty + tz);
            mDistances[offset + 6] += (tx - ty);
            mDistances[offset + k2 + 6] += (tx - ty);
            mDistances[offset + 7] += (tx - tz);
            mDistances[offset + k2 + 7] += (tx - tz);
            mDistances[offset + 8] += (ty - tz);
            mDistances[offset + k2 + 8] += (ty - tz);
        }
    }

}

/*
 * vim: et sw=4 ts=4
 */
