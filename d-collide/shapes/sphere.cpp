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

#include "shapes/sphere.h"
#include "shapes/mesh.h"
#include "shapes/mesh/meshfactory.h"
#include <math.h> 

namespace dcollide {

    /*!
     * \brief Creates an Sphere with given radius
     *-
     * Only a positive radius is allowed so we take the
     * absolute value of the
     * given \p radius.
     *-
     * Optionally you can specify an
     * averageEdgeLength, please refer to
     * Sphere::setAverageEdgeLength for
     * more details.
     */
    Sphere::Sphere(real radius, real averageEdgeLength) {
        mMesh = 0;
        mRadius = fabs(radius);
        setAverageEdgeLength(averageEdgeLength);
    }


    /*!
     * \brief Deletes objects acording to the ownership notices.
     */
    Sphere::~Sphere() {
        delete mMesh;
    }

    /*!
     * \brief Generates an Mesh representation/approximation for an sphere.
     *
     * This methode generates an Mesh representation for this Shape subclass of
     * type Sphere. 
     *
     * Uses \ref MeshFactory::createSphere
     *
     */
    void Sphere::generateMesh() {
        MeshFactory meshFactory;
        mMesh = meshFactory.createSphere(mRadius,mAverageEdgeLength);
        mMesh->setProxy(mProxy);
    }

    /*!
     * \brief Enables the use of std::cout for the Sphere class. When used the
     *        radius of the sphere is printed out.
     */
    std::ostream& operator<<(std::ostream& os, const dcollide::Sphere& v) {
        os << "(Sphere: r = " << v.getRadius() << ")";
        return os;
    }
}
/*
 * vim: et sw=4 ts=4
 */
