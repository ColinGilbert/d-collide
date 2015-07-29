/************************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:            *
 *              d-collide-users@lists.sourceforge.net                               *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,               *
 *     Martin Faßbach, Maximilian Hegele, Daniel Haus, Oliver Horst,                *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz                 *
 *  Copyright (C) 2002-2005 Rivo Laks (rivolaks@hot.ee)                             *
 *                                                                                  *
 *  All rights reserved.                                                            *
 *                                                                                  *
 *  Redistribution and use in source and binary forms, with or without              *
 *  modification, are permitted provided that the following conditions are met:     *
 *   - Redistributions of source code must retain the above copyright               *
 *     notice, this list of conditions and the following disclaimer.                *
 *   - Redistributions in binary form must reproduce the above copyright            *
 *     notice, this list of conditions and the following disclaimer in the          *
 *     documentation and/or other materials provided with the distribution.         *
 *   - Neither the name of the PG510 nor the names of its contributors may be       *
 *     used to endorse or promote products derived from this software without       *
 *     specific prior written permission.                                           *
 *                                                                                  *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS             *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT               *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR           *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER      *
 *  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,        *
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,             *
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR              *
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF          *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING            *
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS              *
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE                     *
 ************************************************************************************/

/*
 * This file has been shamelessy stolen from the Boson project, see
 *   http://boson.eu.org
 *
 * Permission to relicense to the modified BSD license (see header above)
 * granted by both authors:
 * Rivo Laks (rivolaks@hot.ee)
 * Andreas Beckermann (b_mann@gmx.de)
 */

#ifndef DCOLLIDE_VECTOR_H
#define DCOLLIDE_VECTOR_H

#include "real.h"

#include <ostream>

namespace dcollide {
    /*!
     * \brief Vector with 3 components.
     *
     * \author Rivo Laks <rivolaks@hot.ee>
     */
    class Vector3 {
        private:
            real mData[3];

        public:
            inline Vector3();
            inline Vector3(real x, real y, real z);
            inline Vector3(const real* data);
            inline Vector3(const Vector3& v);
            inline ~Vector3();

            inline void reset();
            inline real getX() const;
            inline real getY() const;
            inline real getZ() const;

            inline void set(real x, real y, real z);
            inline void set(const Vector3& v);
            inline void set(const real* v);
            inline void setX(real x);
            inline void setY(real y);
            inline void setZ(real z);
            inline void add(const Vector3& v);

            inline Vector3& normalize();
            inline void scale(real s);

            real length() const;
            real lengthWithoutSqrt() const;
            real lengthApproxBabylonian(int steps = 2) const;

            real angle(const Vector3& v) const;
            static inline real dotProduct(const Vector3& v, const Vector3& w);
            inline real dotProduct(const Vector3& v) const;

            inline real dotProduct() const;
            static Vector3 crossProduct(const Vector3& v, const Vector3& w);
            inline Vector3 crossProduct(const Vector3& v) const;

            inline const real* getData() const;

            inline bool isNull() const;
            inline bool isEqual(real x, real y, real z, real diff = 0.001f) const;
            inline bool isEqual(const real* v, real diff = 0.001f) const;
            inline bool isEqual(const Vector3& v, real diff = 0.001f) const;
            static bool isEqual(const real* v1, const real* v2, real diff = 0.001f);
            static real distanceToPlane(    const Vector3& testPoint,
                                            const Vector3& planePoint,
                                            const Vector3& planeNormal);

            inline Vector3& operator=(const Vector3& v);
            inline Vector3& operator=(const real* v);
            inline void operator+=(const Vector3& v);
            inline real operator[](int i) const;
            inline Vector3 operator+(const Vector3& v) const;
            inline Vector3 operator-(const Vector3& v) const;
            inline Vector3 operator*(real f) const;
            inline Vector3 operator*(const Vector3& v) const;
            inline Vector3 operator/(real f) const;
            inline Vector3 operator-() const;
            inline bool operator==(const Vector3& v) const;
    };

    //------------ Implementation of short methods -------------


    inline Vector3::Vector3() {
        reset();
    }
    inline Vector3::Vector3(real x, real y, real z) {
        set(x, y, z);
    }
    inline Vector3::Vector3(const real* data) {
        set(data[0], data[1], data[2]);
    }
    inline Vector3::Vector3(const Vector3& v) {
        set(v[0], v[1], v[2]);
    }
    inline Vector3::~Vector3() {
    }

    /*!
     * Make this vector a null vector.
     **/
    inline void Vector3::reset() {
        mData[0] = mData[1] = mData[2] = 0.0f;
    }

    /*!
     * \return The first (x) coordinate of the vector.
     **/
    inline real Vector3::getX() const {
        return mData[0];
    }

    /*!
     * \return The second (y) coordinate of the vector.
     **/
    inline real Vector3::getY() const {
        return mData[1];
    }

    /*!
     * \return The third (z) coordinate of the vector.
     **/
    inline real Vector3::getZ() const {
        return mData[2];
    }

    /*!
     * Assign the values \p x, \p y, \p z to the vector.
     **/
    inline void Vector3::set(real x, real y, real z) {
        mData[0] = x;
        mData[1] = y;
        mData[2] = z;
    }

    /*!
     * \overload
     **/
    inline void Vector3::set(const Vector3& v) {
        set(v.getData());
    }

    /*!
     * \overload
     **/
    inline void Vector3::set(const real* v) {
        set(v[0], v[1], v[2]);
    }

    /*!
     * Assign the x coordinate to the vector.
     **/
    inline void Vector3::setX(real x) {
        mData[0] = x;
    }

    /*!
     * Assign the y coordinate to the vector.
     **/
    inline void Vector3::setY(real y) {
        mData[1] = y;
    }

    /*!
     * Assign the z coordinate to the vector.
     **/
    inline void Vector3::setZ(real z) {
        mData[2] = z;
    }

    /*!
     * Add \p v to this vector.
     **/
    inline void Vector3::add(const Vector3& v) {
        mData[0] += v.mData[0];
        mData[1] += v.mData[1];
        mData[2] += v.mData[2];
    }

    /*!
     * Normalize this vector.
     *
     * Normalizing a vector means to make it a so-called "unit-vector", that is
     * a vector with \ref length 1.
     *
     * Practically this means dividing all elements in the vector by the \ref
     * length of the vector.
     **/
    inline Vector3& Vector3::normalize() {
        real l = length();
        if (l != 0.0f) {
            scale(1.0f / l);
        }
        return *this;
    }

    /*!
     * Scale the vector by \p s. This is just scalar multiplication, i.e. all
     * elements/coordinates of the vector are multiplied by \p s.
     **/
    inline void Vector3::scale(real s) {
        mData[0] = mData[0] * s;
        mData[1] = mData[1] * s;
        mData[2] = mData[2] * s;
    }


    /*!
     * \return The dot product of the two vectors \p v and \p w.
     *
     * The dot product v*w is equal to |v|*|w|*cos(alpha), where alpha is the
     * angle between both vectors and |v| is the length of v.
     **/
    inline real Vector3::dotProduct(const Vector3& v, const Vector3& w) {
        return v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
    }

    inline real Vector3::dotProduct(const Vector3& v) const {
        return dotProduct(*this, v);
    }

    /*!
     * \return The dot product of this vector with itself, i.e. (v * v).
     *
     * The dot product is also equivalent to the square of the \ref length. This
     * can be important sometimes, as it might be sufficient to use the square
     * of the length (which is calculated very fast) instead of the actual
     * length (which needs a call to sqrt()).
     **/
    inline real Vector3::dotProduct() const {
        return dotProduct(*this, *this);
    }

    inline Vector3 Vector3::crossProduct(const Vector3& v) const {
        return crossProduct(*this, v);
    }

    /*!
     * \return A pointer to the internal array.
     **/
    inline const real* Vector3::getData() const {
        return mData;
    }

    /*!
     * See \ref set
     **/
    inline Vector3& Vector3::operator=(const Vector3& v) {
        set(v);
        return *this;
    }

    /*!
     * \overload
     **/
    inline Vector3& Vector3::operator=(const real* v) {
        set(v);
        return *this;
    }

    /*!
     * See \ref add
     **/
    inline void Vector3::operator+=(const Vector3& v) {
        add(v);
    }

    /*!
     * \return The component / coordinate at \p i of this vector
     **/
    inline real Vector3::operator[](int i) const {
        return mData[i];
    }

    /*!
     * \return A copy of this vector with \p v added.
     **/
    inline Vector3 Vector3::operator+(const Vector3& v) const {
        return Vector3(mData[0] + v.mData[0], mData[1] + v.mData[1], mData[2] + v.mData[2]);
    }

    /*!
     * \return A copy of this vector, \p v subtracted.
     **/
    inline Vector3 Vector3::operator-(const Vector3& v) const {
        return Vector3(mData[0] - v.mData[0], mData[1] - v.mData[1], mData[2] - v.mData[2]);
    }

    /*!
     * \return A copy of this vector, scaled by \p f.
     **/
    inline Vector3 Vector3::operator*(real f) const {
        return Vector3(mData[0] * f, mData[1] * f, mData[2] * f);
    }

    /*!
     * \return See \ref crossProduct. Cross product of this vector with \p v
     **/
    inline Vector3 Vector3::operator*(const Vector3& v) const {
        return crossProduct(*this, v);
    }

    /*!
     * \return A copy of this vector, scaled by 1 / \p f. Does NOT check if \p f
     * is 0.0 !
     **/
    inline Vector3 Vector3::operator/(real f) const {
        return Vector3(mData[0] / f, mData[1] / f, mData[2] / f);
    }

    /*!
     * \return A copy of this vector scaled by -1
     **/
    inline Vector3 Vector3::operator-() const {
        return Vector3(-mData[0], -mData[1], -mData[2]);
    }

    /*!
     * \return Whether all components of this vector are zeros
     **/
    inline bool Vector3::isNull() const {
        return ((mData[0] == 0.0f) && (mData[1] == 0.0f) && (mData[2] == 0.0f));
    }

    /*!
     * \return TRUE when the coordinates of this vector equal x,y and z,
     * otherwise FALSE.
     * \param diff The maximal difference that the elements may have to be
     * treated as "equal". note that 0.0 is a bad idea, since rounding errors
     * are _very_ probable!
     **/
    inline bool Vector3::isEqual(real x, real y, real z, real diff) const {
        real v2[3];
        v2[0] = x;
        v2[1] = y;
        v2[2] = z;
        return isEqual(mData, v2, diff);
    }

    /*!
     * \overload
     *
     * Same as above, except that it takes an array of 3 reals, such as e.g.
     * Lib3dsVector.
     **/
    inline bool Vector3::isEqual(const real* v, real diff) const {
        return isEqual(v[0], v[1], v[2], diff);
    }

    /*!
     * \overload
     **/
    inline bool Vector3::isEqual(const Vector3& v, real diff) const {
        return isEqual(v.getData(), diff);
    }

    inline bool Vector3::operator==(const Vector3& v) const {
        return isEqual(v);
    }


    std::ostream& operator<<(std::ostream& os, const dcollide::Vector3& v);
}


#endif

/*
 * vim: et sw=4 ts=4
 */
