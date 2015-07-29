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

#ifndef DCOLLIDE_MATRIX_H
#define DCOLLIDE_MATRIX_H

#include "vector.h"
#include "real.h"

namespace dcollide {
    /*!
     * An OpenGL 4x4 matrix. note that we use (just like mesa) column major order to
     * store the matrix elements!
     *
     * This means that a matrix
     * <pre>
     * A11 A12 A13 A14
     * A21 A22 A23 A24
     * A31 A32 A33 A34
     * A41 A42 A43 A44
     * </pre>
     * Will be stored in memory like this:
     * <pre>
     * A11 A21 A31 A41 A12 A22 A32 A42 A13 A23 A33 A43 A14 A12 A13 A14
     * </pre>
     *
     * \brief A 4x4 matrix as used by OpenGL
     * \author Andreas Beckermann <b_mann@gmx.de>
     **/
    class Matrix {
        private:
            real mData[16];

        public:
            inline Matrix();
            inline Matrix(const real* matrix);
            inline Matrix(const Matrix& matrix);

            void loadIdentity();
            void loadMatrix(const real* m);
            inline void loadMatrix(const Matrix& m);
            void loadMatrix(const Vector3& row1, const Vector3& row2, const Vector3& row3);

            inline void setElement(int row, int column, real value);
            inline real getElement(int row, int column) const;

            inline const real* getData() const;

            bool isNull() const;
            bool isIdentity() const;
            bool hasNaN() const;

            void translate(real x, real y, real z, bool respectOrientation = true);
            inline void translate(const Vector3& v, bool respectOrientation = true);
            void scale(real x, real y, real z);
            void multiply(const real* mat);
            inline void multiply(const Matrix* mat);
            Vector3 multiply(const Vector3& v) const;
            void rotate(real angle, real x, real y, real z, bool respectOrientation = true);

            void transform(Vector3* v, const Vector3& input) const;

            bool invert(Matrix* inverse) const;

            bool isEqual(const Matrix& matrix, real diff = 0.001f) const;

            static inline int getIndexAt(int row, int column);

            inline void debugMatrix() const;
            static void debugMatrix(const real* matrix);

            inline real operator[](int i) const;
            inline Matrix& operator=(const Matrix& m);
            inline bool operator==(const Matrix& m) const;

            inline Vector3 getPosition() const;
            inline Vector3 getAxis(int i) const;
            inline Vector3 getXAxis() const;
            inline Vector3 getYAxis() const;
            inline Vector3 getZAxis() const;
            inline Matrix getRotationMatrix() const;
    };

    //------------ Implementation of short methods -------------
    /**
     * Construct an (identitiy) matrix. See \ref loadIdentity.
     **/
    inline Matrix::Matrix() {
        loadIdentity();
    }

    /*!
     * Construct a matrix that is a copy of \p matrix. See \ref loadMatrix
     **/
    inline Matrix::Matrix(const real* matrix) {
        loadMatrix(matrix);
    }

    /*!
     * Construct a matrix that is a copy of \p matrix. See \ref loadMatrix
     **/
    inline Matrix::Matrix(const Matrix& matrix) {
        loadMatrix(matrix);
    }

    /*!
     * \overload
     **/
    inline void Matrix::loadMatrix(const Matrix& m) {
        loadMatrix(m.getData());
    }

    /*!
     * Change the element at \p row, \p column to \p value. See also \ref
     * getElement and \ref getIndexAt
     **/
    inline void Matrix::setElement(int row, int column, real value) {
        mData[getIndexAt(row, column)] = value;
    }

    /*!
     * See also \ref getIndexAt
     * \param row 0..3 -> specifies the row (aka line) of the matrix
     * \param column 0..3 -> specifies the column of the matrix (what a
     * surprise)
     * \return The element of the matrix at the specified position
     **/
    inline real Matrix::getElement(int row, int column) const {
        return mData[getIndexAt(row, column)];
    }

    /*!
     * \return A pointer to the internal array. See also \ref getElement, \ref getIndexAt,
     * \ref setElement
     **/
    inline const real* Matrix::getData() const {
        return mData;
    }

    /*!
     * \overload
     **/
    inline void Matrix::translate(const Vector3& v, bool respectOrientation) {
        translate(v.getX(), v.getY(), v.getZ(), respectOrientation);
    }

    /*!
     * \overload
     **/
    inline void Matrix::multiply(const Matrix* mat) {
        multiply(mat->getData());
    }

    /*!
     * \return The index of the element \p row, \p column of the matrix in the
     * internal array. The array can be organized in two different ways, which
     * both are used out there in the world. We are preferring the organization
     * that is used by OpenGL/mesa
     **/
    inline int Matrix::getIndexAt(int row, int column) {
        return (column << 2) + row;
    }

    /*!
     * Dump this matrix to the console as debug output.
     **/
    inline void Matrix::debugMatrix() const {
        debugMatrix(getData());
    }

    /*!
     * \return The element at index \p i in the internal array. See \ref
     * getIndexAt.
     **/
    inline real Matrix::operator[](int i) const {
        return mData[i];
    }

    /*!
     * See \ref loadMatrix
     **/
    inline Matrix& Matrix::operator=(const Matrix& m) {
        loadMatrix(m);
        return *this;
    }

    /*!
     * See \ref isEqual
     **/
    inline bool Matrix::operator==(const Matrix& m) const {
        return isEqual(m);
    }

    /*!
     * \return The first three entries of the last column, that means the actual
     * position.
     */
    inline Vector3 Matrix::getPosition() const{
        return Vector3(mData[12], mData[13], mData[14]);
    }
    
    /*!
     * \return Axis \p i of the coordinate space the matrix describes
     */
    inline Vector3 Matrix::getAxis(int i) const {
        return Vector3(getElement(0, i), getElement(1, i), getElement(2, i));
    }
    
    /*!
     * \return The x-axis of the coordinate space the matrix describes
     */
    inline Vector3 Matrix::getXAxis() const {
        return getAxis(0);
    }
        
    /*!
     * \return The y-axis of the coordinate space the matrix describes
     */
    inline Vector3 Matrix::getYAxis() const {
        return getAxis(1);
    }
    
    /*!
     * \return The z-axis of the coordinate space the matrix describes
     */
    inline Vector3 Matrix::getZAxis() const {
        return getAxis(2);
    }

    /*!
     * \return A copy of this matrix with the last column and the last row set
     * to 0,0,0,1, i.e. only the upper left 3x3 rotation matrix. Note that this
     * method does not guarantee that this actually is a valid rotation matrix,
     * this depends on the operations made on this matrix! It is recommended to
     * use this method only if only translations and rotations were made on this
     * matrix.
     */
    inline Matrix Matrix::getRotationMatrix() const{
        Matrix rot(*this);
        rot.setElement(0, 3, 0.0);
        rot.setElement(1, 3, 0.0);
        rot.setElement(2, 3, 0.0);
        rot.setElement(3, 0, 0.0);
        rot.setElement(3, 1, 0.0);
        rot.setElement(3, 2, 0.0);
        rot.setElement(3, 3, 1.0);
        return rot;
    }


    std::ostream& operator<<(std::ostream& os, const dcollide::Matrix& m);
}

#endif // DCOLLIDE_MATRIX_H

/*
 * vim: et sw=4 ts=4
 */
