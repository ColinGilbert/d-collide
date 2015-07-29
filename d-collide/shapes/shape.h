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

#ifndef DCOLLIDE_SHAPE_H
#define DCOLLIDE_SHAPE_H

#include <iostream>

namespace dcollide {

    //-----------classes------------

    class Mesh;
    class Proxy;
    class DebugStream;

    /*!
     * \brief Generalized Shape implementation.
     */
    class Shape {
        public:
            /*!
             * Gives the maximal length of an polygon edge (in pixel) while
             * generating an mesh for a shape with full precision.
             * 
             * This value doesn't influence the lower bound for the number of
             * sections created by the algorithms. Please refer to the documentation
             * of the concrete implementation to identifier the concrete values of
             * the lower bound.
             */
            static const int POLYGON_EDGE_LENGTH = 3;
        
            enum ShapeType {
                SHAPE_TYPE_BOX,
                SHAPE_TYPE_CONE,
                SHAPE_TYPE_CYLINDER,
                SHAPE_TYPE_MESH,
                SHAPE_TYPE_MESH_PART,
                SHAPE_TYPE_SPHERE,
                SHAPE_TYPE_WEDGE
            };            
        
            inline Shape();
            inline virtual ~Shape();

            void setProxy(Proxy* parentProxy);
            inline Proxy* getProxy() const;

            virtual ShapeType getShapeType() const = 0;
            virtual const Mesh* getMeshIfExists() const = 0;
            virtual const Mesh* getMesh() = 0;

            virtual void invalidateAllWorldPositions();

        protected:
            Proxy* mProxy;
    };


    //------------ Implementation of short methods -------------

    Shape::Shape() {
        mProxy = 0;
    }

    Shape::~Shape() {
    }

    /*!
     * \return The "parent" proxy, i.e. the proxy that this shape object is
     * embedded in.
     */
    Proxy* Shape::getProxy() const {
        return mProxy;
    }


    std::ostream& operator<<(std::ostream& os, const dcollide::Shape::ShapeType& v);
    DebugStream operator<<(DebugStream& s, const Shape::ShapeType& v);
}


#endif // DCOLLIDE_SHAPE_H
/*
 * vim: et sw=4 ts=4
 */
