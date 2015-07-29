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

#ifndef DCOLLIDE_SPHERE_H
#define DCOLLIDE_SPHERE_H

#include <iostream>

#include "real.h"

#include "shapes/shape.h"

namespace dcollide {
    class Mesh;

     /*!
     * \brief Spezialized Shape implementation in form of an sphere.
     */
    class Sphere : public Shape {
        private:

            /*!
             * \brief Radius of the sphere
             */
            real mRadius;

            /*!
             * \brief Created or associated mesh
             */
            Mesh* mMesh;

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
            Sphere(real radius, real averageEdgeLength = -1);
            ~Sphere();

            inline real getRadius() const;

            inline virtual ShapeType getShapeType() const;

            inline void setMesh(Mesh* mesh);
            inline const Mesh* getMeshIfExists() const;
            inline const Mesh* getMesh();

            inline real getAverageEdgeLength() const;
            inline void setAverageEdgeLength(real averageEdgeLength);
    };


    //------------ Implementation of short methods -------------

    /*!
     * \overload
     * \brief Returns the ShapeType of this shape
     * \returns Shape::ShapeType::SHAPE_TYPE_SPHERE
     */
    Shape::ShapeType Sphere::getShapeType() const {
        return SHAPE_TYPE_SPHERE;
    }

    /*!
     * \brief Returns the current radius of the sphere.
     */
    real Sphere::getRadius() const {
        return mRadius;
    }

    /*!
     * \brief Sets the mesh to the given one
     * 
     * OWNERSHIP NOTICE:
     * The sphere takes the ownership of the given \p mesh.
     */
    void Sphere::setMesh(Mesh* mesh) {
        mMesh = mesh;
    }

    /*!
     * \brief Returns the Mesh representation of this shape, created if needed.
     */
    const Mesh* Sphere::getMesh() {
        if (mMesh == 0) {
            generateMesh();
        }
        return mMesh;
    }

    /*!
     * \brief Returns the Mesh representation of this shape (can be NULL).
     */
    const Mesh* Sphere::getMeshIfExists() const {
       return mMesh;
    }

    /*!
     * \brief Returns the specified precision factor for the mesh generation
     *        algorithm.
     * 
     * This indicates the average length of a triangle edge which is generated
     * by the mesh creation algorithm. 
     */
    real Sphere::getAverageEdgeLength() const {
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
    void Sphere::setAverageEdgeLength(real averageEdgeLength) {
        if (averageEdgeLength <= 0) {
            mAverageEdgeLength = 0;
            mPrecisionSpecified = false;
        } else {
            mAverageEdgeLength = averageEdgeLength;
            mPrecisionSpecified = true;
        }
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Sphere& v);
}


#endif // DCOLLIDE_SPHERE_H
/*
 * vim: et sw=4 ts=4
 */
