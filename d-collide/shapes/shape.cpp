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

#include "shapes/shape.h"
#include "proxy.h"
#include "mesh.h"
#include "debugstream.h"
#include "exceptions/exception.h"

#include <iostream>

namespace dcollide {

    /*!
     * Set the "parent" proxy of this shape object. Every shape must be in
     * exactly one \ref Proxy object to be of use to d-collide.
     *
     * This method can be called only once, changing the proxy afterwards is not
     * supported.
     *
     * You normally do not need to call this method, it already is called by the
     * \ref Proxy constructor. Just specify your shape object in the \ref Proxy
     * constructor.
     */
    void Shape::setProxy(Proxy* parentProxy) {
        if (mProxy) {
            // AB: we do not allow changing the proxy. if we did, we would not
            //     have to do a lot of nasty things, such as updating the
            //     proxies' bounding volumes, making sure that no dangling
            //     pointers to this shape and it's bounding volume are around
            //     and so on.
            std::cerr << "Shape already has a parent proxy" << std::endl;
            return;
        }
        mProxy = parentProxy;
    }

    /*!
     *\brief invalidates all vertices in the mesh representation of the mesh
     *
     * This happens without forcing the Mesh to be calculated. If no mesh has
     * been calculated before, this will do nothing.
     */
    void Shape::invalidateAllWorldPositions() {
        if (getMeshIfExists() != 0) {
            const_cast<Mesh*>(getMeshIfExists())->invalidateAllWorldPositions();
        }
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::Shape::ShapeType& v) {
        switch (v) {
            case dcollide::Shape::SHAPE_TYPE_BOX:
                os << "box";
                break;
            case dcollide::Shape::SHAPE_TYPE_CONE:
                os << "cone";
                break;
            case dcollide::Shape::SHAPE_TYPE_CYLINDER:
                os << "cylinder";
                break;
            case dcollide::Shape::SHAPE_TYPE_MESH:
                os << "mesh";
                break;
            case dcollide::Shape::SHAPE_TYPE_MESH_PART:
                os << "meshpart";
                break;
            case dcollide::Shape::SHAPE_TYPE_SPHERE:
                os << "sphere";
                break;
            case dcollide::Shape::SHAPE_TYPE_WEDGE:
                os << "wedge";
                break;
            default:
                throw dcollide::UnsupportedShapeException(
                    "unknown shape type can't be converted to string");
        }
        return os;
    }

    DebugStream operator<<(DebugStream& s, const Shape::ShapeType& v) {
        switch (v) {
            case dcollide::Shape::SHAPE_TYPE_BOX:
                s << "box";
                break;
            case dcollide::Shape::SHAPE_TYPE_CONE:
                s << "cone";
                break;
            case dcollide::Shape::SHAPE_TYPE_CYLINDER:
                s << "cylinder";
                break;
            case dcollide::Shape::SHAPE_TYPE_MESH:
                s << "mesh";
                break;
            case dcollide::Shape::SHAPE_TYPE_MESH_PART:
                s << "meshpart";
                break;
            case dcollide::Shape::SHAPE_TYPE_SPHERE:
                s << "sphere";
                break;
            case dcollide::Shape::SHAPE_TYPE_WEDGE:
                s << "wedge";
                break;
            default:
                throw dcollide::UnsupportedShapeException(
                    "unknown shape type can't be converted to string");
        }
        return s;
    }
}
/*
 * vim: et sw=4 ts=4
 */
