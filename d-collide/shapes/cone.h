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

#ifndef DCOLLIDE_CONE_H
#define DCOLLIDE_CONE_H

#include <iostream>

#include "real.h"

#include "shapes/shape.h"

namespace dcollide {

    //-----------classes------------
    class Mesh;

    /*!
     * \brief Spezialized Shape implementation in form of an cone.
     * The cone is oriented as follows
     *  - its base point  (0,0,0) is the center of the bottom sphere
     *  - the tip is at (0, 0, height)
     * 
     *          /\ z
     *          |
     *          | 
     *          *
     *         /|\
     *        / | \
     *       /  |  \
     *      /   |   \
     *     /..--|--..\
     *    {_    +----_}------------> y
     *      ''-/---'' 
     *        /
     *       /
     *     |/
     *     x
     */
    class Cone : public Shape {
        private:
            /*!
             * \brief radius of the cone
             */
            real mRadius;

            /*!
             * \brief height of the cone
             */
            real mHeight;

            mutable Mesh* mMesh;
            real mAverageEdgeLength;
            bool mPrecisionSpecified;

            void generateMesh();

        public:
            inline Cone(real radius, real height, real averageEdgeLength);
            ~Cone();

            inline real getRadius() const;
            inline real getHeight() const;

            inline virtual ShapeType getShapeType() const;

            inline void setMesh(Mesh* mesh);
            inline const Mesh* getMeshIfExists() const;
            inline const Mesh* getMesh();

            inline real getAverageEdgeLength() const;
            inline void setAverageEdgeLength(real averageEdgeLength);
    };


    //------------ Implementation of short methods -------------

    /*! 
     * \brief creates a cone with given radius, height and (optional) precision
     * @param height height of the cone (location of the tip)
     * @param radius radius of the bottom base circle
     * @param averageEdgeLenght used to compute a triangle mesh for this cone.
     *                          controls how long the edges of the base circle
     *                          should be
     * 
    * The cone is oriented as follows
     *  - its base point  (0,0,0) is the center of the bottom sphere
     *  - the tip is at (0, 0, height)
     * 
     *              /\ z
     *              |
     *              | 
     * -------------*
     *   |         /|\
     *   |        / | \
     * height h  /  |  \
     *   |      /   |   \
     *   |     /..--|--..\
     *  ----- {_    +----_}------------> y
     *          ''-/---'' |
     *            / |     |
     *           /  |     |
     *         |/   |--r--|  radius r
     *         x
     * 

     */
    Cone::Cone(real radius, real height, real averageEdgeLength = -1) {
        mRadius = radius;
        mHeight = height;

        mMesh = 0;
        setAverageEdgeLength(averageEdgeLength);
    }

    Shape::ShapeType Cone::getShapeType() const {
        return SHAPE_TYPE_CONE;
    }

    /*!
     * \brief Returns the current radius of the cone.
     */
    real Cone::getRadius() const {
        return mRadius;
    }

    /*!
     * \brief Returns the current height of the cone.
     */
    real Cone::getHeight() const {
        return mHeight;
    }

    /*!
     * OWNERSHIP NOTICE:
     * The cone takes the ownership of the given \p mesh.
     */
    void Cone::setMesh(Mesh* mesh) {
        mMesh = mesh;
    }

    /*!
     * \brief returns the Mesh representation of this shape, create it if needed
     */
    const Mesh* Cone::getMesh() {
        if (mMesh == 0) {
            generateMesh();
        }
        return mMesh;
    }
    /*!
     * \brief returns the Mesh representation of this shape (can be NULL)
     */
    const Mesh* Cone::getMeshIfExists() const {
       return mMesh;
    }

    real Cone::getAverageEdgeLength() const {
        return mAverageEdgeLength;
    }

    void Cone::setAverageEdgeLength(real averageEdgeLength) {
        if (averageEdgeLength <= 0) {
            mAverageEdgeLength = 0;
            mPrecisionSpecified = false;
        } else {
            mAverageEdgeLength = averageEdgeLength;
            mPrecisionSpecified = true;
        }

    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Cone& v);
}


#endif // DCOLLIDE_CONE_H
/*
 * vim: et sw=4 ts=4
 */
