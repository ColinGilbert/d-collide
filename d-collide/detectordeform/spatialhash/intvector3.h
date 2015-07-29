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

#ifndef INTVECTOR3
#define INTVECTOR3

namespace dcollide {

    /*!
     * \brief This is an integer representation of the vector3 class
     * This class was introduced to speed up vector operations in the spatial hash
     * algorithm. As the spatial hash uses whole numbers for cell coordinates, the usage
     * of the real type is not necessary.
     *
     * This class implements exactly what is needed by the spatial hash. If other
     * parts of the project want to take advantage of it the class should be extendend
     * and moved into the math module.
     */
    class IntVector3 {
        public:
            IntVector3();
            IntVector3( int x, int y, int z);
            inline bool operator==(IntVector3& v);

            inline int getX();
            inline int getY();
            inline int getZ();

            inline void setX(int value);
            inline void setY(int value);
            inline void setZ(int value);

        private:
            int mX, mY, mZ;
    };

    inline bool IntVector3::operator==(IntVector3& v) {
        return (mX == v.getX()) && ( mY == v.getY() ) && ( mZ == v.getZ() );
    }

    inline int IntVector3::getX() {
        return mX;
    }

    inline int IntVector3::getY() {
        return mY;
    }

    inline int IntVector3::getZ() {
        return mZ;
    }

    inline void IntVector3::setX(int value) {
        mX = value;
    }

    inline void IntVector3::setY(int value) {
        mY = value;
    }

    inline void IntVector3::setZ(int value) {
        mZ = value;
    }
}
#endif

