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

#include "quaternion.h"

#include "matrix.h"

// Degrees to radians conversion (AB: from mesa/src/macros.h)
#define DEG2RAD (M_PI/180.0)
// And radians to degrees conversion
#define RAD2DEG (180.0/M_PI)

/*!
 * Convert \p deg, given in degree, into radians.
 * \return \p deg as radians.
 **/
inline static dcollide::real deg2rad(dcollide::real deg) {
    return (dcollide::real)(deg * DEG2RAD);
}

/**
 * Convert \p rad, given in radians, into degree.
 * \return \p rad as degree.
 **/
inline static dcollide::real rad2deg(dcollide::real rad) {
    return (dcollide::real)(rad * RAD2DEG);
}

namespace dcollide {
    /*!
     * \return The quaternion converted into a rotation matrix
     **/
    Matrix Quaternion::calculateMatrix() const {
        Matrix m;
        const real x = mV[0];
        const real y = mV[1];
        const real z = mV[2];
        const real xx = x * x;
        const real yy = y * y;
        const real zz = z * z;
        const real xy = x * y;
        const real xz = x * z;
        const real xw = mW * x;
        const real yz = y * z;
        const real yw = mW * y;
        const real zw = mW * z;
        m.setElement(0, 0, 1.0f - 2.0f * (yy + zz));
        m.setElement(1, 0,        2.0f * (xy + zw));
        m.setElement(2, 0,        2.0f * (xz - yw));

        m.setElement(0, 1,        2.0f * (xy - zw));
        m.setElement(1, 1, 1.0f - 2.0f * (xx + zz));
        m.setElement(2, 1,        2.0f * (yz + xw));

        m.setElement(0, 2,        2.0f * (xz + yw));
        m.setElement(1, 2,        2.0f * (yz - xw));
        m.setElement(2, 2, 1.0f - 2.0f * (xx + yy));

        return m;
    }

    real Quaternion::calculateLength() const {
        return (real)sqrt(mW * mW + mV[0] * mV[0] + mV[1] * mV[1] + mV[2] * mV[2]);
    }

    /*!
     * Set the rotation according to the \p direction and the \p up vector.
     * These are compatible to the parameters used in gluLookAt, but note
     * that the \p direction differs from the lookat (aka center) vector.
     *
     * The direction is the (camera - lookat) vector.
     */
    void Quaternion::setRotation(const Vector3& direction_, const Vector3& up_) {
        Vector3 dir(direction_);
        Vector3 up(up_);
        dir.normalize();

        Vector3 x = Vector3::crossProduct(up, dir);
        Vector3 y = Vector3::crossProduct(dir, x);
        x.normalize();
        y.normalize();

        Matrix M;
        M.setElement(0, 0, x[0]);
        M.setElement(0, 1, x[1]);
        M.setElement(0, 2, x[2]);

        M.setElement(1, 0, y[0]);
        M.setElement(1, 1, y[1]);
        M.setElement(1, 2, y[2]);

        M.setElement(2, 0, dir[0]);
        M.setElement(2, 1, dir[1]);
        M.setElement(2, 2, dir[2]);

        setRotation(M);
    }

    /*!
     * \param angle The angle around \p axis, given in degree.
     */
    void Quaternion::setRotation(real angle, const Vector3& axis) {
        Vector3 normAxis = axis;
        normAxis.normalize();
        real sina = sin(deg2rad(angle / 2));
        mW = cos(deg2rad(angle / 2));
        mV.set(normAxis[0] * sina, normAxis[1] * sina, normAxis[2] * sina);
        normalize();
    }

    /*!
     * The so-called "euler rotation". This creates a quaternion for as if
     * <pre>
     * glRotatef(angleX, 1, 0, 0);
     * glRotatef(angleY, 0, 1, 0);
     * glRotatef(angleZ, 0, 0, 1);
     * </pre>
     * was called (in this order).
     * \param angleX The (euler-)angle around the x-axis. given in degree.
     * \param angleY The (euler-)angle around the y-axis. given in degree.
     * \param angleZ The (euler-)angle around the z-axis. given in degree.
     */
    void Quaternion::setRotation(real angleX, real angleY, real angleZ) {
        Quaternion x, y, z;
        // one quaternion per axis
        x.set((real)cos(deg2rad(angleX/2)), Vector3((real)sin(deg2rad(angleX/2)), 0.0f, 0.0f));
        y.set((real)cos(deg2rad(angleY/2)), Vector3(0.0f, (real)sin(deg2rad(angleY/2)), 0.0f));
        z.set((real)cos(deg2rad(angleZ/2)), Vector3(0.0f, 0.0f, (real)sin(deg2rad(angleZ/2))));
        x.multiply(y);
        x.multiply(z);
        set(x);
        normalize();
    }

    /*!
     * Convert a rotation matrix to the quaternion. A rotation matrix is
     * simply a matrix that describes a rotation.
     */
    void Quaternion::setRotation(const Matrix& rotationMatrix) {
        // See Q55 in the quat faq on http://www.j3d.org/matrix_faq/matrfaq_latest.html
        // WARNING: although they refer to a column order matrix in Q54, they use _row_
        // order here!
        real x, y, z, w;
        const real* m = rotationMatrix.getData();
        real t = (real)(1.0 + m[0] + m[5] + m[10]);
        if (t > 0.0f) {
            real s = (real)(sqrt(t) * 2.0);
            x = (m[6] - m[9]) / s;
            y = (m[8] - m[2]) / s;
            z = (m[1] - m[4]) / s;
            w = (real)(0.25 * s);
        } else if (m[0] > m[5] && m[0] > m[10]) {
            real s = (real)(sqrt(1.0f + m[0] - m[5] - m[10]) * 2.0);
            x = (real)(0.25 * s);
            y = (m[1] + m[4]) / s;
            z = (m[8] + m[2]) / s;
            w = (m[6] - m[9]) / s;
        } else if (m[5] > m[10]) {
            real s = (real)(sqrt(1.0f + m[5] - m[0] - m[10]) * 2.0);
            x = (m[1] + m[4]) / s;
            y = (real)(0.25 * s);
            z = (m[6] + m[9]) / s;
            w = (m[8] - m[2]) / s;
        } else {
            real s = (real)(sqrt(1.0f + m[10] - m[0] - m[5]) * 2.0);
            x = (m[8] + m[2]) / s;
            y = (m[6] + m[9]) / s;
            z = (real)(0.25 * s);
            w = (m[1] - m[4]) / s;
        }
        mW = w;
        mV.set(x, y, z);
    }

    /*!
     * See \ref Matrix::toRotation
     */
    void Quaternion::toRotation(real* angle, Vector3* axis) {
        // see Q 57 in quat faq on http://www.j3d.org/matrix_faq/matrfaq_latest.html
        if (!angle || !axis) {
            return;
        }
        normalize();
        const real cosa = mW;
        *angle = acos(cosa) * 2;
        *angle = rad2deg(*angle);
        real sina = (real)sqrt(1.0 - cosa * cosa);
        if (fabs(sina) < 0.0005) {
            sina = 1.0f;
        }
        axis->set(mV.getX() / sina, mV.getY() / sina, mV.getZ() / sina);
    }

    /*!
     * Rotate the vector \input and return the result into \p v.
     *
     * Of course we assume that this quaternion is normalized (see \ref
     * normalize), as only normalized quaternions represent rotations.
     */
    void Quaternion::transform(Vector3* v, const Vector3* input) const {
        Quaternion q = Quaternion(0, *input);
        Quaternion tmp = Quaternion::multiply(*this, q);
        // we assume this is a unit quaternion, then the inverse is equal to the
        // conjugate
        tmp.multiply(conjugate());
        v->set(tmp.mV);
    }

}
/*
 * vim:et ts=4 sw=4
 */
