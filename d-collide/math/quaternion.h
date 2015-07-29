/************************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:            *
 *              d-collide-users@lists.sourceforge.net                               *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,               *
 *     Martin Faßbach, Maximilian Hegele, Daniel Haus, Oliver Horst,                *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz                 *
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
 * Andreas Beckermann (b_mann@gmx.de)
 * Rivo Laks (rivolaks@hot.ee)
 */

#ifndef DCOLLIDE_QUATERNION_H
#define DCOLLIDE_QUATERNION_H

#include <math.h>

#include "vector.h"

namespace dcollide {
    class Matrix;

    /*!
     * A rotation can be represented in several ways, a quaternion is one of them.
     *
     * The by far easiest and most popular way is to store 3 different rotation
     * values, one for each axis. In OpenGL code this will look like this:
     * <pre>
     * glRotatef(angleX, 1.0f, 0.0f, 0.0f);
     * glRotatef(angleY, 0.0f, 1.0f, 0.0f);
     * glRotatef(angleZ, 0.0f, 0.0f, 1.0f);
     * </pre>
     * These angles are called euler angles.
     * They suffer from the so-called * "gimbal-lock".
     *
     * Use google to find a description on what this "gimbal lock" is  - i am not
     * qualified enough to give a correct description. It is enough to say that the
     * rotation will not occur as you want it to.
     *
     * Another representation is the angle axis representation. Here you do only one
     * rotation, by an arbitrary axis. Code:
     * <pre>
     * glRotatef(angle, axisX, axisY, axisZ);
     * </pre>
     * This does not suffer from gimbal lock but (according to some
     * tutorials/howtos) it suffers from other things, such as when you interpolate
     * between two rotations. This problem does not matter for us, as we do not
     * (yet?) use it. But it is imho hard to use and to calculate.
     *
     * The third, and probably most popular among big 3d projects, way of
     * representing rotations are quaternions.
     *
     * I will not try to explain to you what exactly quaternions are - i am not
     * qualified enough to do this. Use google and e.g.
     * http://www.gamedev.net/reference/articles/article1095.asp
     *
     * A quaternion consists of 4 floating point values - a scalar (w) and a vector
     * (v).
     * You can get a rotation matrix (see \ref matrix) from a quaternion and
     * therefore you can easily use it in glMultMatrix.
     *
     * You can convert all major means of rotation into a quaternion,
     * see \ref setRotation.
     *
     * If you \ref multiply a quat by another one you get a similar effect as
     * if you had multiplied both rotation matrices, i.e. the two rotations are
     * combined.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class Quaternion {
        public:
            inline Quaternion();
            inline Quaternion(const Quaternion& quat);
            inline Quaternion(real w, const Vector3& v);

            inline Quaternion& operator=(const Quaternion& quat);
            inline void operator*=(const Quaternion& quat);
            inline Quaternion operator+(const Quaternion& q) const;
            inline bool operator==(const Quaternion& quat) const;

            inline void set(real w, const Vector3& v);
            inline void set(const Quaternion& quat);
            inline void loadIdentity();

            inline real getW() const;
            inline const Vector3& getV() const;

            real calculateLength() const;
            Matrix calculateMatrix() const;

            inline void multiply(const Quaternion& quat);
            inline static Quaternion multiply(const Quaternion& q1, const Quaternion& q2);

            inline Quaternion conjugate() const;
            inline Quaternion inverse() const;

            void transform(Vector3* v, const Vector3* input) const;

            inline void normalize();

            inline bool isEqual(const Quaternion& quat, real diff = 0.001) const;

            void setRotation(real angle, const Vector3& axis);
            void setRotation(real angleX, real angleY, real angleZ);
            void setRotation(const Matrix& rotationMatrix); // See Q55 in the quat faq
            void setRotation(const Vector3& direction, const Vector3& up);
            inline void setRotation(const Vector3& cameraPos, const Vector3& lookAt, const Vector3& up);

            void toRotation(real* angle, Vector3* axis); // see Q 57 in quat faq

        private:
            real mW;
            Vector3 mV;
    };

    Quaternion::Quaternion()
        : mW(1.0) {
    }

    Quaternion::Quaternion(const Quaternion& quat) {
        *this = quat;
    }
    Quaternion::Quaternion(real w, const Vector3& v) {
        set(w, v);
    }

    Quaternion& Quaternion::operator=(const Quaternion& quat) {
        set(quat);
        return *this;
    }

    void Quaternion::loadIdentity() {
        mW = 1.0f;
        mV.set(0.0f, 0.0f, 0.0f);
    }

    /*!
     * \return The scalar part of this quaternion
     */
    real Quaternion::getW() const {
        return mW;
    }

    /*!
     * \return The vector part of this quaternion
     */
    const Vector3& Quaternion::getV() const {
        return mV;
    }

    /*!
     * Multiply this quaternion by another one. The identity quaternion
     * (i.e. a quat doesnt get changed if you multiply it by this one) is
     * (1,(0,0,0)), i.e. w is 1 and the vector is (0,0,0).
     *
     * This way of combining two rotations does not suffer from gimbal lock.
     *
     * Note that this quaternion, as well as \p quat should be normalized
     * quaternions! See also \ref normalize. The resulting quaternion will be
     * normalized as well.
     */
    void Quaternion::multiply(const Quaternion& quat) {
        real w = mW * quat.mW    - mV[0] * quat.mV[0] - mV[1] * quat.mV[1] - mV[2] * quat.mV[2];
        real x = mW * quat.mV[0] + mV[0] * quat.mW    + mV[1] * quat.mV[2] - mV[2] * quat.mV[1];
        real y = mW * quat.mV[1] + mV[1] * quat.mW    + mV[2] * quat.mV[0] - mV[0] * quat.mV[2];
        real z = mW * quat.mV[2] + mV[2] * quat.mW    + mV[0] * quat.mV[1] - mV[1] * quat.mV[0];
        mW = w;
        mV.set(x, y, z);
    }

    Quaternion Quaternion::multiply(const Quaternion& q1, const Quaternion& q2) {
        Quaternion q(q1);
        q.multiply(q2);
        return q;
    }

    void Quaternion::operator*=(const Quaternion& quat) {
        multiply(quat);
    }

    Quaternion Quaternion::operator+(const Quaternion& q) const {
        return Quaternion(mW + q.mW, mV + q.mV);
    }
    /*!
     * \return The conjugate of the quaternion, which is the quaternion with the
     * vector part negated.
     */
    inline Quaternion Quaternion::conjugate() const {
        return Quaternion(mW, Vector3(-mV[0], -mV[1], -mV[2]));
    }

    /*!
     * \return The inverse quaternion. This is equal to the \ref conjugate, if
     * the quaternion is normalized (see \ref normalize). You should prefer \ref
     * conjugate if you know that the quat is normalized (i.e. always)!
     */
    Quaternion Quaternion::inverse() const {
        // we assume that the quat is normalized.
        // If it is not, we would have to new_quat.mW /= quat.length()
        Quaternion q = conjugate();
        real l = calculateLength();
        q.mW /= l;
        return q;
    }
    /*!
     * Normalize the quaternion. Note that only a normalized quaternion
     * represents a rotation, meaning that non-normalized quaternions are
     * useless for us!
     */
    void Quaternion::normalize() {
        real l = calculateLength();
        mW /= l;
        mV.scale(1.0f / l);
    }

    bool Quaternion::isEqual(const Quaternion& quat, real diff) const {
        real d = (real) fabs(mW - quat.mW);
        if (d > diff) {
            return false;
        }
        return mV.isEqual(quat.mV, diff);
    }

    bool Quaternion::operator==(const Quaternion& quat) const {
        return isEqual(quat);
    }

    void Quaternion::set(real w, const Vector3& v) {
        mW = w;
        mV = v;
    }

    void Quaternion::set(const Quaternion& quat) {
        set(quat.mW, quat.mV);
    }

    /*!
     * \overload
     *
     * This takes exactly the
     * arguments that gluLookAt() takes.
     */
    void Quaternion::setRotation(const Vector3& cameraPos, const Vector3& lookAt, const Vector3& up) {
        setRotation(cameraPos - lookAt, up);
    }

}

#endif
/*
 * vim:et ts=4 sw=4
 */
