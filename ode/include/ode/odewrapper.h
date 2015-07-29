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

#ifndef ODEWRAPPER_H
#define ODEWRAPPER_H

#include <ode/common.h>

#include <d-collide/math/vector.h>

namespace dcollide {
    class Matrix;
    class Proxy;
}


//------------ classes ---------------------------------------------------------

/*!
 * \brief Interface to ODE.
 *
 * This class acts as an interface to ODE functions. Derive from this class (or
 * use \ref MyDCollideODEGeom) and implement the required methods. Then create
 * an object of this class for every physics relevant object (e.g. for every
 * \ref dcollide::Proxy) and use that object as replacement for the normal ODE
 * geoms.
 *
 * The \ref resetTransformation, \ref translate and \ref rotate functions
 * provided by this interface are called by ODE when a physics step is made.
 */
class MyODEGeom {
    public:
        MyODEGeom();
        virtual ~MyODEGeom();

        void createODEGeom(dcollide::Proxy* proxy, dSpaceID space);
        dGeomID getODEGeom() const;

        virtual void setBody(dBodyID id);
        inline dBodyID getBody() const;
        
        inline void setGravityOffset(const dcollide::Vector3& offset);
        inline const dcollide::Vector3& getGravityOffset() const;
        
        inline void setMoveNodeOnBodyMoved(bool set);
        inline bool getMoveNodeOnBodyMoved() const;

        virtual void notifyBodyMoved();

        static void initializeODECollisionFunctions();

        void markOdeGeomAsMoved();

    protected:
        /*!
         * \brief ODE physics body
         */
        dBodyID mBody;

        /*!
         * \brief Offset of bodys gravity center acording to the surounding node
         */
        dcollide::Vector3 mGravityOffset;

        /*!
         * \brief Indicates whenether the surounding node should be moved or not
         *        when the physics body moves
         * 
         * This flag is used when adjusting the physics body to a already moved
         * d-collide body for example.
         */
        bool mMoveNodeOnBodyMoved;  

        /*!
         * \brief Sets the position of the surounding node to the given values
         */
        virtual void setPosition(float, float, float) = 0;

        /*!
         * \brief Absolutely rotates the surounding node arcording to the
         *        given matrix
         */
        virtual void setRotation(const dcollide::Matrix&) = 0;

    private:
        static bool mCustomCollisionDetectionInitialized;

    private:
        /*!
         * Used for normal ODE (i.e. not d-collide) collision detection. NULL if
         * d-collide collision detection is used (the default).
         */
        dGeomID mODEGeom;
};


/*!
 * Implementation of \ref MyODEGeom that works on a \ref dcollide::Proxy object.
 *
 * Note that if your application requires additional data strutures to be
 * updated when the physics geom changes (i.e. moves/rotates/..) then you should
 * rather reimplement the required functions in \ref MyODEGeom yourself.
 *
 * This class is primarily meant for writing simple test and debugging
 * applications without any GUI for the d-collide library itself.
 */
// AB: this class is in particular meant for use in mytest applications
class MyDCollideODEGeom : public MyODEGeom {
    public:
        MyDCollideODEGeom(dcollide::Proxy* proxy);

    protected:
        virtual void setPosition(float x, float y, float z);
        virtual void setRotation(const dcollide::Matrix& m);
        
    private:
        dcollide::Proxy* mProxy;
};


//------------ implementation of short methods (MyODEGeom) ---------------------


inline dBodyID MyODEGeom::getBody() const {
    return mBody;
}

inline void MyODEGeom::setGravityOffset(const dcollide::Vector3& offset) {
    mGravityOffset = offset;
}

inline const dcollide::Vector3& MyODEGeom::getGravityOffset() const {
    return mGravityOffset;
}

inline void MyODEGeom::setMoveNodeOnBodyMoved(bool set) {
    mMoveNodeOnBodyMoved = set;
}

inline bool MyODEGeom::getMoveNodeOnBodyMoved() const {
    return mMoveNodeOnBodyMoved;
}

#endif
/*
 * vim: et sw=4 ts=4
 */
