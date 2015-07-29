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


#ifndef DCOLLIDE_WORLDCOLLISIONS_H
#define DCOLLIDE_WORLDCOLLISIONS_H

#include <list>
#include "collisioninfo.h"
#include "narrowphase/narrowphase.h"

namespace dcollide {

    class World;
    class BroadPhaseCollisions;

    /*!
     * \brief A class containing all collisions of all phases
     *
     * An object of this class is returned by \ref World::calculateAllCollisions. It
     * contains all collisions of all phases (broad, middle, narrow), assuming
     * they have all been used (the user may be able to switch phases off).
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class WorldCollisions {
        public:
            WorldCollisions();
            ~WorldCollisions();

            inline const BroadPhaseCollisions* getBroadPhaseCollisions() const;
            inline const std::list<BoundingVolumeCollision>& getRigidBoundingVolumeCollisions() const;

            // FIXME: this probably makes _no_ sense at all
            // -> different algorithms have different output format, we probably
            //    won't be able to provide a single datastructure for them.
            //   --> we probably should remove this method
            inline const std::list<BoundingVolumeCollision>& getDeformableBoundingVolumeCollisions() const;

            inline const std::list<CollisionInfo>& getNarrowPhaseCollisions() const;

        protected:
            // AB: only the World/Pipeline class is allowed to set the internal data
            // structures, everyone else has read-only access only.
            friend class World;
            friend class Pipeline;

            void setBroadPhaseCollisions(BroadPhaseCollisions*);
            void setRigidBoundingVolumeCollisions(std::list<BoundingVolumeCollision>* c);
            void setDeformableBoundingVolumeCollisions(std::list<BoundingVolumeCollision>* c);
            void addNarrowPhaseCollisions(std::list<CollisionInfo>* list);

        private:
            // TODO: add private operator=() and private copy c'tor
            // -> both are NOT supported and not recommended by this class
            BroadPhaseCollisions* mBroadPhaseCollisions;
            std::list<CollisionInfo> mNarrowPhaseCollisions;
            std::list<BoundingVolumeCollision> mRigidBoundingVolumeCollisions;
            std::list<BoundingVolumeCollision> mDeformableBoundingVolumeCollisions;
    };


    /*!
     * \returns all \ref BroadPhaseCollisions 
     */
    inline const BroadPhaseCollisions* WorldCollisions::getBroadPhaseCollisions() const {
        return mBroadPhaseCollisions;
    }

    /*!
     * \returns std::list of rigid \ref BoundingVolumeCollision
     */
    inline const std::list<BoundingVolumeCollision>& WorldCollisions::getRigidBoundingVolumeCollisions() const {
        return mRigidBoundingVolumeCollisions;
    }
    /*!
     * \returns std::list of deformable \ref BoundingVolumeCollision
     */
    inline const std::list<BoundingVolumeCollision>& WorldCollisions::getDeformableBoundingVolumeCollisions() const {
        return mDeformableBoundingVolumeCollisions;
    }
    /*!
     * \returns std::list of \ref CollisionInfo
     */
    inline const std::list<CollisionInfo>& WorldCollisions::getNarrowPhaseCollisions() const {
        return mNarrowPhaseCollisions;
    }

}

#endif
/*
 * vim: et sw=4 ts=4
 */
