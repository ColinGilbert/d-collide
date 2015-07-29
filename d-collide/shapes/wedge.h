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

#ifndef DCOLLIDE_WEDGE_H
#define DCOLLIDE_WEDGE_H

#include <math.h>
#include <iostream>

#include "real.h"
#include "math/vector.h"
#include "math/plane.h"

#include "shapes/shape.h"

namespace dcollide {

    class Mesh;
    //-----------classes------------

     /*!
     * \brief Spezialized Shape implementation in form of an wedge.
     */
    class Wedge : public Shape {
        private:
            /*!
             * \brief Mesh represantation of the wedge shape
             */
            Mesh* mMesh;

            /*!
             * \brief The three dimensions of the wedge
             */
            real mDimensions[3];

            /*!
             * \brief The three dimensions of the wedge
             */
            Vector3 mVertices[6];

            /*!
             * \brief The five planes of the wedge
             */
            Plane mPlanes[5];

            void generateMesh();

        public:
            inline Wedge(real x, real y, real z);
            ~Wedge();

            inline const real* getDimensions() const;

            inline virtual ShapeType getShapeType() const;

            inline void setMesh(Mesh* mesh);
            inline const Mesh* getMeshIfExists() const;
            inline const Mesh* getMesh();
            const Vector3* getVertices();
            const void getEdges(std::pair<int,int>(&edges)[9]) const;
            const Plane* getPlanes(const Vector3* vs = 0); 
            const Vector3 getUnrotatedCenter();
    };


    //------------ Implementation of short methods -------------

    /*!
     * \brief Creates an wedge with width (x), depth (y) and height (z) 
     * 
     * The wedge forms a ramp starting at the x-axis. Its inclined plane
     * rises along the y-axis.
     * 
     * For the parameters \p x, \p y and \p z only positive numbers are alowed,
     * so we take the absolute values of the given values.
     *
     * 
     * The point of reference for the wedge is the point in the lower left
     * corner
     * 
     * Sketch of the wedge.
     * 
     *         +------+     \n
     *  ^     /|     /|     \n
     *  |    / |    / | z   \n
     *  |   /  |   /  |     \n
     *  |  /   +--/---+     \n
     *  | /  *   /  *y      \n
     *  |/ *    / *         \n
     *  o------+-------->   \n
     *     x
     * 
     * 
     */
    Wedge::Wedge(real x, real y, real z) {

        mDimensions[0] = fabs(x);
        mDimensions[1] = fabs(y);
        mDimensions[2] = fabs(z);

        mMesh = 0;
    }

    /*!
     * \overload
     * \brief Returns the ShapeType of this shape
     * \returns Shape::ShapeType::SHAPE_TYPE_WEDGE
     */
    Shape::ShapeType Wedge::getShapeType() const {
        return SHAPE_TYPE_WEDGE;
    }

    /*!
     * \brief Returns the dimesnions of this wedge
     * 
     * Returns the three dimesnions as an array, with the follwing index
     * association:
     * 
     * 0 => width (\p a)
     * 1 => height (\p b)
     * 2 => depth (\p h)
     */
    const real* Wedge::getDimensions() const {
        return mDimensions;
    }

    /*!
     * \brief Sets the mesh representation to the given one.
     * 
     * OWNERSHIP NOTICE:
     * The wedge takes the ownership of the given \p mesh.
     */
    void Wedge::setMesh(Mesh* mesh) {
        mMesh = mesh;
    }

    /*!
     * \brief Returns the Mesh representation of this shape, created if needed.
     */
    const Mesh* Wedge::getMesh() {
        if (mMesh == 0) {
            const_cast<Wedge*>(this)->generateMesh();
        }
        return mMesh;
    }

    /*!
     * \brief Returns the Mesh representation of this shape (can be NULL).
     */
    const Mesh* Wedge::getMeshIfExists() const {
       return mMesh;
    }


    std::ostream& operator<<(std::ostream& os, const dcollide::Wedge& v);
}


#endif // DCOLLIDE_WEDGE_H
/*
 * vim: et sw=4 ts=4
 */
