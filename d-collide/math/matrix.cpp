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



/*
 * Some parts of this code are shamlessy stolen from mesa and have been marked
 * as such.
 * copyright notice for these parts:
 *
 *
 * Copyright (C) 1999-2003  Brian Paul   All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 */

#include "matrix.h"
#include "dcollide-config.h"

#include <math.h>
#include <stdio.h>

// Degrees to radians conversion (AB: from mesa/src/macros.h)
#define DEG2RAD (M_PI/180.0)
// And radians to degrees conversion
#define RAD2DEG (180.0/M_PI)

#ifndef HAVE_ISNAN
inline bool isnan(double x) {
    return (x != x);
}
#endif // !HAVE_ISNAN

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
     * Load the identity matrix (the "1" for matrices: M * identity = M)
     **/
    void Matrix::loadIdentity() {
        int i;
        for (i = 0; i < 16; i++) {
            mData[i] = 0.0f;
        }
        mData[0] = mData[5] = mData[10] = mData[15] = 1.0f;
    }

    /*!
     * \overload
     **/
    void Matrix::loadMatrix(const real* m) {
        for (int i = 0; i < 16; i++) {
            mData[i] = m[i];
        }
    }

    /*!
     * \overload
     * The three vectors get interpreted as <em>row</em> vectors
     **/
    void Matrix::loadMatrix(const Vector3& x, const Vector3& y, const Vector3& z) {
        setElement(0, 0, x[0]);
        setElement(0, 1, x[1]);
        setElement(0, 2, x[2]);
        setElement(0, 3, 0.0f);

        setElement(1, 0, y[0]);
        setElement(1, 1, y[1]);
        setElement(1, 2, y[2]);
        setElement(1, 3, 0.0f);

        setElement(2, 0, z[0]);
        setElement(2, 1, z[1]);
        setElement(2, 2, z[2]);
        setElement(2, 3, 0.0f);

        setElement(3, 0, 0.0f);
        setElement(3, 1, 0.0f);
        setElement(3, 2, 0.0f);
        setElement(3, 3, 1.0f);
    }

    /*!
     * \return TRUE if <em>all</em> elements of this matrix are 0. Otherwise
     * FALSE. WARNING: no error values are accepted here, the elements must be
     * exactly 0.0f
     **/
    bool Matrix::isNull() const {
        for (int i = 0; i < 16; i++) {
            if (mData[i] != 0.0f) {
                return false;
            }
        }
        return true;
    }
    /*!
     * \return TRUE if this is the identity matrix, otherwise FALSE. WARNING: no
     * error values are accepted here, the values must be exactly 0.0f and 1.0f
     **/
    bool Matrix::isIdentity() const {
        for (int i = 0; i < 16; i++) {
            if (mData[i] != 0.0f) {
                if (mData[i] != 1.0f || i % 5 != 0) {
                    return false;
                }
            }
        }
        return true;
    }

    /*!
     * \return TRUE if one of the matrix elements is NaN (not a number) and
     * therefore the matrix is invalid. Otherwise FALSE.
     **/
    bool Matrix::hasNaN() const {
        for (int i = 0; i < 16; i++) {
            if (isnan(mData[i])) {
                return true;
            }
        }
        return false;
    }


    /*!
     * Translate (i.e. move) the matrix by x,y,z.
     **/
    void Matrix::translate(real x, real y, real z, bool respectOrientation) {
        if (respectOrientation) {
            // shamelessy stolen from mesa/src/math/m_math.c
            mData[12] = mData[0] * x + mData[4] * y + mData[8]  * z + mData[12];
            mData[13] = mData[1] * x + mData[5] * y + mData[9]  * z + mData[13];
            mData[14] = mData[2] * x + mData[6] * y + mData[10] * z + mData[14];
            mData[15] = mData[3] * x + mData[7] * y + mData[11] * z + mData[15];
        } else {
            mData[12] = mData[12] + x;
            mData[13] = mData[13] + y;
            mData[14] = mData[14] + z;
        }
    }

    /*!
     * Scale the matrix by x,y,z.
     *
     * Note that if one of x,y,z is 0.0 the result will probably an invalid
     * matrix. Don't do that unless you really know what you're doing.
     **/
    void Matrix::scale(real x, real y, real z) {
        // shamelessy stolen from mesa/src/math/m_math.c
        mData[0] *= x;   mData[4] *= y;   mData[8]  *= z;
        mData[1] *= x;   mData[5] *= y;   mData[9]  *= z;
        mData[2] *= x;   mData[6] *= y;   mData[10] *= z;
        mData[3] *= x;   mData[7] *= y;   mData[11] *= z;
    }

    /*!
     * Multiply the matrix by \p mat.
     * \param mat An array as returned by \ref getData and as used by OpenGL.
     **/
    void Matrix::multiply(const real* mat) {
        // shamelessy stolen from mesa/src/math/m_math.c
        // we use matmul4() from mesa only, not matmul34(). this means we are slower
        // than mesa! (and also less complex).
        // AB: this function multiplies mData by mat and places the result into mData.
#define B(row,col)  mat[getIndexAt(row, col)]
        int i;
        for (i = 0; i < 4; i++) {
            const real ai0=getElement(i,0),  ai1=getElement(i,1),  ai2=getElement(i,2),  ai3=getElement(i,3);
            mData[getIndexAt(i, 0)] = ai0 * B(0,0) + ai1 * B(1,0) + ai2 * B(2,0) + ai3 * B(3,0);
            mData[getIndexAt(i, 1)] = ai0 * B(0,1) + ai1 * B(1,1) + ai2 * B(2,1) + ai3 * B(3,1);
            mData[getIndexAt(i, 2)] = ai0 * B(0,2) + ai1 * B(1,2) + ai2 * B(2,2) + ai3 * B(3,2);
            mData[getIndexAt(i, 3)] = ai0 * B(0,3) + ai1 * B(1,3) + ai2 * B(2,3) + ai3 * B(3,3);
        }
#undef B
    }

    /*!
     * \brief Multiply the matrix by vector \p v
     * \param v the \ref dcollide::Vector3 we want to multiply with.
     * Note: multiplies only the top-left 3x3 matrix with \p v
     * \returns the result of the multiplication
     **/
    Vector3 Matrix::multiply(const Vector3& v) const {
        Vector3 ret;
        ret.setX(mData[getIndexAt(0,0)]*v[0] + mData[getIndexAt(0,1)]*v[1]
                + mData[getIndexAt(0,2)]*v[2]);
        ret.setY(mData[getIndexAt(1,0)]*v[0] + mData[getIndexAt(1,1)]*v[1]
                + mData[getIndexAt(1,2)]*v[2]);
        ret.setZ(mData[getIndexAt(2,0)]*v[0] + mData[getIndexAt(2,1)]*v[1]
                + mData[getIndexAt(2,2)]*v[2]);
        return ret;
    }

    /*!
     * Rotate around a specified axis. \p angle specifies the angle, i.e. how
     * much it is rotated and x,y,z specify the axis.
     *
     * See also the OpenGL glRotate() which uses the same syntax.
     **/
    void Matrix::rotate(real angle, real x, real y, real z, bool respectOrientation) {
        // shamelessy stolen from mesa/src/math/m_math.c
        real mag, s, c;
        real xx, yy, zz, xy, yz, zx, xs, ys, zs, one_c;
        real m[16];

        s = (real) sin( deg2rad(angle) );
        c = (real) cos( deg2rad(angle) );

        mag = (real) sqrt( x*x + y*y + z*z ); // AB: mesa uses GL_SQRT here

        if (mag <= 1.0e-4) {
            // generate an identity matrix and return
            loadIdentity();
            return;
        }

        x /= mag;
        y /= mag;
        z /= mag;

#define M(row,col)  m[col*4+row]

        xx = x * x;
        yy = y * y;
        zz = z * z;
        xy = x * y;
        yz = y * z;
        zx = z * x;
        xs = x * s;
        ys = y * s;
        zs = z * s;
        one_c = 1.0F - c;

        M(0,0) = (one_c * xx) + c;
        M(0,1) = (one_c * xy) - zs;
        M(0,2) = (one_c * zx) + ys;
        M(0,3) = 0.0F;

        M(1,0) = (one_c * xy) + zs;
        M(1,1) = (one_c * yy) + c;
        M(1,2) = (one_c * yz) - xs;
        M(1,3) = 0.0F;

        M(2,0) = (one_c * zx) - ys;
        M(2,1) = (one_c * yz) + xs;
        M(2,2) = (one_c * zz) + c;
        M(2,3) = 0.0F;

        M(3,0) = 0.0F;
        M(3,1) = 0.0F;
        M(3,2) = 0.0F;
        M(3,3) = 1.0F;

#undef M

        multiply(m);
    }

    /*!
     * Transform the vector \p input according to this matrix and put the result
     * into \p v.
     *
     * This calculates simply does v = M * input, where M is this matrix.
     *
     * WARNING: v == input is not allowed.
     **/
    void Matrix::transform(Vector3* v, const Vector3& input) const {
        // partially shamelessy stolen from mesa/src/math/m_math.c

        // v = m * i, m is a 4x4 OpenGL matrix, r and v are both a 3x1 column vector.
        // the forth element is unused here and therefore we use silently 0.
        v->setX(getElement(0, 0) * input.getData()[0] + getElement(0, 1) * input.getData()[1] + getElement(0, 2) * input.getData()[2] + getElement(0, 3));
        v->setY(getElement(1, 0) * input.getData()[0] + getElement(1, 1) * input.getData()[1] + getElement(1, 2) * input.getData()[2] + getElement(1, 3));
        v->setZ(getElement(2, 0) * input.getData()[0] + getElement(2, 1) * input.getData()[1] + getElement(2, 2) * input.getData()[2] + getElement(2, 3));
    }


    /*!
     * Invert this matrix and place the result into \p inverse.
     * \return TRUE on success or FALSE if this is a not invertible matrix.
     **/
    bool Matrix::invert(Matrix* inverse) const {
        // shamelessy stolen from mesa/src/math/m_math.c
        // invert_matrix_general

#define SWAP_ROWS(a, b) { real *_tmp = a; (a)=(b); (b)=_tmp; }
#define MAT(m,r,c) (m)[(c)*4+(r)]
        const real *m = mData;
        real *out = inverse->mData;
        real wtmp[4][8];
        real m0, m1, m2, m3, s;
        real *r0, *r1, *r2, *r3;

        r0 = wtmp[0], r1 = wtmp[1], r2 = wtmp[2], r3 = wtmp[3];

        r0[0] = MAT(m,0,0), r0[1] = MAT(m,0,1),
        r0[2] = MAT(m,0,2), r0[3] = MAT(m,0,3),
        r0[4] = 1.0, r0[5] = r0[6] = r0[7] = 0.0,

        r1[0] = MAT(m,1,0), r1[1] = MAT(m,1,1),
        r1[2] = MAT(m,1,2), r1[3] = MAT(m,1,3),
        r1[5] = 1.0, r1[4] = r1[6] = r1[7] = 0.0,

        r2[0] = MAT(m,2,0), r2[1] = MAT(m,2,1),
        r2[2] = MAT(m,2,2), r2[3] = MAT(m,2,3),
        r2[6] = 1.0, r2[4] = r2[5] = r2[7] = 0.0,

        r3[0] = MAT(m,3,0), r3[1] = MAT(m,3,1),
        r3[2] = MAT(m,3,2), r3[3] = MAT(m,3,3),
        r3[7] = 1.0, r3[4] = r3[5] = r3[6] = 0.0;

        /* choose pivot - or die */
        if (fabs(r3[0])>fabs(r2[0])) { SWAP_ROWS(r3, r2); }
        if (fabs(r2[0])>fabs(r1[0])) { SWAP_ROWS(r2, r1); }
        if (fabs(r1[0])>fabs(r0[0])) { SWAP_ROWS(r1, r0); }
        if (0.0 == r0[0]) {
            return false;
        }

        /* eliminate first variable     */
        m1 = r1[0]/r0[0]; m2 = r2[0]/r0[0]; m3 = r3[0]/r0[0];
        s = r0[1]; r1[1] -= m1 * s; r2[1] -= m2 * s; r3[1] -= m3 * s;
        s = r0[2]; r1[2] -= m1 * s; r2[2] -= m2 * s; r3[2] -= m3 * s;
        s = r0[3]; r1[3] -= m1 * s; r2[3] -= m2 * s; r3[3] -= m3 * s;
        s = r0[4];
        if (s != 0.0) { r1[4] -= m1 * s; r2[4] -= m2 * s; r3[4] -= m3 * s; }
        s = r0[5];
        if (s != 0.0) { r1[5] -= m1 * s; r2[5] -= m2 * s; r3[5] -= m3 * s; }
        s = r0[6];
        if (s != 0.0) { r1[6] -= m1 * s; r2[6] -= m2 * s; r3[6] -= m3 * s; }
        s = r0[7];
        if (s != 0.0) { r1[7] -= m1 * s; r2[7] -= m2 * s; r3[7] -= m3 * s; }

        /* choose pivot - or die */
        if (fabs(r3[1])>fabs(r2[1])) { SWAP_ROWS(r3, r2); }
        if (fabs(r2[1])>fabs(r1[1])) { SWAP_ROWS(r2, r1); }
        if (0.0 == r1[1]) { return false; }

        /* eliminate second variable */
        m2 = r2[1]/r1[1]; m3 = r3[1]/r1[1];
        r2[2] -= m2 * r1[2]; r3[2] -= m3 * r1[2];
        r2[3] -= m2 * r1[3]; r3[3] -= m3 * r1[3];
        s = r1[4]; if (0.0 != s) { r2[4] -= m2 * s; r3[4] -= m3 * s; }
        s = r1[5]; if (0.0 != s) { r2[5] -= m2 * s; r3[5] -= m3 * s; }
        s = r1[6]; if (0.0 != s) { r2[6] -= m2 * s; r3[6] -= m3 * s; }
        s = r1[7]; if (0.0 != s) { r2[7] -= m2 * s; r3[7] -= m3 * s; }

        /* choose pivot - or die */
        if (fabs(r3[2])>fabs(r2[2])) { SWAP_ROWS(r3, r2); }
        if (0.0 == r2[2]) { return false; }

        /* eliminate third variable */
        m3 = r3[2]/r2[2];
        r3[3] -= m3 * r2[3], r3[4] -= m3 * r2[4],
        r3[5] -= m3 * r2[5], r3[6] -= m3 * r2[6],
        r3[7] -= m3 * r2[7];

        /* last check */
        if (0.0 == r3[3]) { return false; }

        s = 1.0F/r3[3];             /* now back substitute row 3 */
        r3[4] *= s; r3[5] *= s; r3[6] *= s; r3[7] *= s;

        m2 = r2[3];                 /* now back substitute row 2 */
        s  = 1.0F/r2[2];
        r2[4] = s * (r2[4] - r3[4] * m2), r2[5] = s * (r2[5] - r3[5] * m2),
        r2[6] = s * (r2[6] - r3[6] * m2), r2[7] = s * (r2[7] - r3[7] * m2);
        m1 = r1[3];
        r1[4] -= r3[4] * m1, r1[5] -= r3[5] * m1,
        r1[6] -= r3[6] * m1, r1[7] -= r3[7] * m1;
        m0 = r0[3];
        r0[4] -= r3[4] * m0, r0[5] -= r3[5] * m0,
        r0[6] -= r3[6] * m0, r0[7] -= r3[7] * m0;

        m1 = r1[2];                 /* now back substitute row 1 */
        s  = 1.0F/r1[1];
        r1[4] = s * (r1[4] - r2[4] * m1), r1[5] = s * (r1[5] - r2[5] * m1),
        r1[6] = s * (r1[6] - r2[6] * m1), r1[7] = s * (r1[7] - r2[7] * m1);
        m0 = r0[2];
        r0[4] -= r2[4] * m0, r0[5] -= r2[5] * m0,
        r0[6] -= r2[6] * m0, r0[7] -= r2[7] * m0;

        m0 = r0[1];                 /* now back substitute row 0 */
        s  = 1.0F/r0[0];
        r0[4] = s * (r0[4] - r1[4] * m0), r0[5] = s * (r0[5] - r1[5] * m0),
        r0[6] = s * (r0[6] - r1[6] * m0), r0[7] = s * (r0[7] - r1[7] * m0);

        MAT(out,0,0) = r0[4]; MAT(out,0,1) = r0[5],
        MAT(out,0,2) = r0[6]; MAT(out,0,3) = r0[7],
        MAT(out,1,0) = r1[4]; MAT(out,1,1) = r1[5],
        MAT(out,1,2) = r1[6]; MAT(out,1,3) = r1[7],
        MAT(out,2,0) = r2[4]; MAT(out,2,1) = r2[5],
        MAT(out,2,2) = r2[6]; MAT(out,2,3) = r2[7],
        MAT(out,3,0) = r3[4]; MAT(out,3,1) = r3[5],
        MAT(out,3,2) = r3[6]; MAT(out,3,3) = r3[7];

        return true;

#undef MAT
#undef SWAP_ROWS
    }


    /*!
     * \return TRUE when.. well, when this matrix is equal to \p matrix
     * \param diff The maximal difference that the elements may have to be
     * treated as "equal". note that 0.0 is a bad idea, since rounding errors
     * are _very_ probable!
     **/
    bool Matrix::isEqual(const Matrix& matrix, real diff) const {
        for (int i = 0; i < 16; i++) {
            if (fabs(mData[i] - matrix.mData[i]) > diff) {
                return false;
            }
        }
        return true;
    }

    /*!
     * Dump \p matrix onto the console as debug output.
     **/
    void Matrix::debugMatrix(const real* matrix) {
        for (int i = 0; i < 4; i++) {
            printf("%f %f %f %f\n", matrix[i], matrix[i + 4], matrix[i + 8], matrix[i + 12]);
        }
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Matrix& m) {
        os << "(";
        for (int i = 0; i < 4; i++) {
            os << "["
                    << m.getElement(i, 0)
                    << ","
                    << m.getElement(i, 1)
                    << ","
                    << m.getElement(i, 2)
                    << ","
                    << m.getElement(i, 3)
                    << "]";
        }
        os << ")";
        return os;
    }

}


/*
 * vim: et sw=4 ts=4
 */
