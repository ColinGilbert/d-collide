/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,          *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz            *
 *                                                                             *
 *  All rights reserved.                                                       *
 *                                                                             *
 *  Redistribution and use in source and binary forms, with or without         *
 *  modification, are permitted provided that the following conditions are met:*
 *   - Redistributions of source code must retain the above copyright          *
       notice, this list of conditions and the following disclaimer.           *
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



#ifndef DCOLLIDE_CYLINDERINTERSECTOR_H
#define DCOLLIDE_CYLINDERINTERSECTOR_H

#include "narrowphasestrategy.h"
#include "boundingvolumecollision.h"

#include <list>
#include <vector>


namespace dcollide {

    //Forward declarations
    class Box;
    class Cylinder;
    class Matrix;
    class Vertex;
    class Plane;
    struct CollisionInfo;
    class Vector3;
    class Shape;

    /*!
     * \brief Class to calculate detailed intersections between box,wedges and cylinders
     */
    class CylinderIntersector {
        public:
            CylinderIntersector();
            ~CylinderIntersector();

            std::list<CollisionInfo> getIntersection(
                    const Matrix& currentStateBox, Box* box, 
                    const Matrix& currentStateCylinder, Cylinder* cylinder) const;

            std::list<CollisionInfo> getIntersection(
                    const Matrix& currentStateWedge, Wedge* wedge, 
                    const Matrix& currentStateCylinder, Cylinder* cylinder) const;

            std::list<CollisionInfo> getIntersection(
                    const Matrix& currentStateCylinder1, Cylinder* cylinder1, 
                    const Matrix& currentStateCylinder2, Cylinder* cylinder2) const;
    };

}

#endif // DCOLLIDE_CYLINDERINTERSECTOR_H
/*
 * vim: et sw=4 ts=4
 */
