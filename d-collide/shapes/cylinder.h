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

#ifndef DCOLLIDE_CYLINDER_H
#define DCOLLIDE_CYLINDER_H

#include <math.h>
#include <iostream>

#include "real.h"

#include "shapes/shape.h"

namespace dcollide {

    //-----------classes------------

    /*!
     * \brief Spezialized Shape implementation in form of an cylinder.
     * 
     * The reference point of the cylinder is the center of the bottom circle.
     */
    class Cylinder : public Shape {
        private:
            /*!
             * \brief Radius of the cylinder
             */
            real mRadius;

            /*!
             * \brief Height of the cylinder
             */
            real mHeight;

            /*!
             * \brief Mesh represantation of the cylinder shape
             */
            mutable Mesh* mMesh;

            /*!
             * \brief Avarage length of an triangle side in a generated mesh
             */
            real mAverageEdgeLength;

            /*!
             * \brief Indicates if a mesh precision (mAverageEdgeLength) is
             *        specified or not
             */
            bool mPrecisionSpecified;

            void generateMesh();

        public:
            inline Cylinder(real radius, real heigth, real averageEdgeLength);
            ~Cylinder();

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
     * \brief Creates a cylinder eith the given \p radius and \p height.
     * 
     * Only a positive radius and a positive height are allowed so we take the
     * absolute values of the given \p radius and \p height.
     * 
     * 
     * The cylinder rotationalsymmetric axis of the cylinder is its Z-axis
     *  
     *             ^ z
     *             |
     *          r  |
     *       |<--->|
     *       |     |
     *        _..-----.._
     *  ---- (_         _)
     *   ^   | ''-----'' |
     *   |   |           |
     * h |   |           |
     *   |   |           |
     *   V   |_..-----.._|
     *  ---- (_    x    _) ----------> y
     *         ''-----''
     *         /
     *        /
     *       /|
     *      x
     * 
     * Optionally you can specify an averageEdgeLength, please refer to
     * Cylinder::setAverageEdgeLength for more details.
     */
    Cylinder::Cylinder(real radius, real height, real averageEdgeLength = -1) {
        mRadius = fabs(radius);
        mHeight = fabs(height);

        mMesh = 0;
        setAverageEdgeLength(averageEdgeLength);
    }

    /*!
     * \overload
     * \brief Returns the ShapeType of this shape
     * \returns Shape::ShapeType::SHAPE_TYPE_CYLINDER
     */
    Shape::ShapeType Cylinder::getShapeType() const {
        return SHAPE_TYPE_CYLINDER;
    }

    /*!
     * \brief Returns the current radius of the cylinder.
     */
    real Cylinder::getRadius() const {
        return mRadius;
    }

    /*!
     * \brief Returns the current height of the cylinder.
     */
    real Cylinder::getHeight() const {
        return mHeight;
    }

    /*!
     * \brief Sets the mesh representation to the given one.
     * 
     * OWNERSHIP NOTICE:
     * The cylinder takes the ownership of the given \p mesh.
     */
    void Cylinder::setMesh(Mesh* mesh) {
        mMesh = mesh;
    }

    /*!
     * \brief Returns the Mesh representation of this shape, created if needed.
     */
    const Mesh* Cylinder::getMesh() {
        if (mMesh == 0) {
            const_cast<Cylinder*>(this)->generateMesh();
        }
        return mMesh;
    }
    /*!
     * \brief Returns the Mesh representation of this shape (can be NULL).
     */
    const Mesh* Cylinder::getMeshIfExists() const {
       return mMesh;
    }

    /*!
     * \brief Returns the specified precision factor for the mesh generation
     *        algorithm.
     * 
     * This indicates the average length of a triangle edge which is generated
     * by the mesh creation algorithm.
     * 
     * This value is always greater or equal zero!
     */
    real Cylinder::getAverageEdgeLength() const {
        return mAverageEdgeLength;
    }

    /*!
     * \brief Sets the precision factor for the mesh generation alogrithm.
     * 
     * A value greater 0 is requiered here, as this represents the average
     * length of a triangle edge which is generated by the mesh creation
     * algorithm. If a value lower or equal 0 is specified mAverageEdgeLength
     * is set to 0 and mPrecisionSpecified indicates that no mesh precision
     * was specified.
     */
    void Cylinder::setAverageEdgeLength(real averageEdgeLength) {
        if (averageEdgeLength <= 0) {
            mAverageEdgeLength = 0;
            mPrecisionSpecified = false;
        } else {
            mAverageEdgeLength = averageEdgeLength;
            mPrecisionSpecified = true;
        }
    }


    std::ostream& operator<<(std::ostream& os, const dcollide::Cylinder& v);
}


#endif // DCOLLIDE_CYLINDER_H
/*
 * vim: et sw=4 ts=4
 */
