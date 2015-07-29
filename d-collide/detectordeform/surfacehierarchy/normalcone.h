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


#ifndef DCOLLIDE_NORMALCONE_H
#define DCOLLIDE_NORMALCONE_H

#include <math.h>

#include "real.h"

#include "math/vector.h"

//#define NORMALCONE_EXACT

namespace dcollide {

    //-----------classes------------

    class BoundingVolume;

    /*!
     * \brief Class which holds an normal cone of an bounding volume
     *        or triangle.
     */
    class NormalCone {
        private:
            /*!
             * \brief The vector representing the axis of the normal cone.
             */
            Vector3 mAxis;

            /*!
             * \brief The HALF of the normal cone angle.
             */
            real mAngle;

        public:
            inline NormalCone();
            inline NormalCone(const Vector3& axis, real angle);
            inline ~NormalCone();

            inline void setAxis(const Vector3& axis);
            inline void setAngle(real angle);

            inline const Vector3* getAxis() const;
            inline real getAngle() const;

            inline bool isSelfCollisionPossible() const;

            inline void operator=(const NormalCone& nc);
            void operator+=(const NormalCone& nc);
            NormalCone operator+(const NormalCone& nc) const;
    };


    //------------ Implementation of short methods -------------

    NormalCone::NormalCone() {
        mAngle = 0;
        mAxis.set(0, 0, 0);
    }

    NormalCone::NormalCone(const Vector3& axis, real angle) {
        setAngle( angle );
        setAxis( axis );
    }

    NormalCone::~NormalCone() {
    }


    void NormalCone::setAxis(const Vector3& axis) {
        mAxis = axis;
        mAxis.normalize();
    }

    /*!
     * \brief sets the angle of the normal cone.
     *
     * The angle must be specified as radian.
     */
    void NormalCone::setAngle(real angle) {
        mAngle = angle;
    }

    const Vector3* NormalCone::getAxis() const {
        return &mAxis;
    }

    real NormalCone::getAngle() const {
        return mAngle;
    }


    bool NormalCone::isSelfCollisionPossible() const {
        return (mAngle * 2 >= M_PI) ? true : false;
    }


    void NormalCone::operator=(const NormalCone& nc) {
        setAngle( nc.getAngle() );
        setAxis( *nc.getAxis() );
    }

}

#endif // DCOLLIDE_NORMALCONE_H
