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

#ifndef DCOLLIDE_BOX_H
#define DCOLLIDE_BOX_H

#include <iostream>
#include <vector>

#include "math/vector.h"
#include "math/matrix.h"
#include "math/plane.h"

#include "shapes/shape.h"

namespace dcollide {

    class Mesh;

    //-----------classes------------

    /*!
     * \brief Spezialized Shape implementation in form of an box.
     *
     * The box shape is always axis aligned, it can be rotated in arbitrary
     * directions by rotating the \ref Proxy object that it lives in.
     */
    class Box : public Shape {
        private:
            mutable Mesh* mMesh;
            Vector3 mDimension;
            Vector3 mCenter;
            Vector3 mVertices[8];
            Plane mPlanes[6];

            void generateMesh();

        public:
            inline Box();
            inline explicit Box(Mesh* mesh);
            inline explicit Box(const Vector3& dimension);
            inline Box(real width, real height, real depth);
            ~Box();

            inline void setMesh(Mesh* mesh);

            inline const Mesh* getMeshIfExists() const;
            inline const Mesh* getMesh();

            inline virtual ShapeType getShapeType() const;
            inline const Vector3& getDimension() const;

            const Vector3* getVertices();
            const Vector3* getVertices(const Matrix& state);
            const void getEdges(std::pair<int,int>(&edges)[12]) const;
            const Plane* getPlanes(const Vector3* vs = 0); 
            const Vector3 getCenter(const Vector3* vs = 0);
            const Vector3 getUnrotatedCenter() const;

    };


    //------------ Implementation of short methods -------------

    /*!
     * Constructs an "empty" box, i.e. a box of size 0 in all directions. This
     * box is essentially a single point at (0,0,0).
     */
    Box::Box() {
        mMesh = 0;
    }

    Box::Box(Mesh* mesh) {
        setMesh(mesh);
    }

    /*!
     * Constructs a box with the size components in \p dimension.
     *
     * The lower-left-back corner (= the "minimum" point of this axis aligned
     * box) is at (0,0,0) and the right-top-front corner (= the "maximum" point)
     * is at \p dimension.
     */
    Box::Box(const Vector3& dimension)  {
        mDimension = dimension;
        mMesh = 0;
    }

    /*!
     * \overload
     */
    Box::Box(real width, real height, real depth)  {
        mDimension = Vector3(width, height, depth);
        mMesh = 0;
    }

    Shape::ShapeType Box::getShapeType() const {
        return SHAPE_TYPE_BOX;
    }

    const Vector3& Box::getDimension() const {
        return mDimension;
    }

    /*!
     * OWNERSHIP NOTICE:
     * The box takes the ownership of the given \p mesh.
     */
    void Box::setMesh(Mesh* mesh) {
        mMesh = mesh;
    }

    /*!
     * \brief returns the Mesh representation of this shape, create it if needed
     */
    const Mesh* Box::getMesh() {
        if (mMesh == 0) {
            generateMesh();
        }
        return mMesh;
    }

    /*!
     * \brief returns the Mesh representation of this shape (can be NULL)
     */
    const Mesh* Box::getMeshIfExists() const {
       return mMesh;
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Box& v);
}


#endif // DCOLLIDE_BOX_H
/*
 * vim: et sw=4 ts=4
 */
