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

#include "worldcollisions.h"
#include "broadphase/broadphasecollisions.h"
#include "timing.h"
#include "debug.h"

#include <iostream>

namespace dcollide {

    WorldCollisions::WorldCollisions() {
        mBroadPhaseCollisions = 0;
    }

    WorldCollisions::~WorldCollisions() {
//        delete mBroadPhaseCollisions;
    }

    /*!
     * Set the broadphase results. This class takes ownership of \p c and
     * deletes it on destruction.
     */
    void WorldCollisions::setBroadPhaseCollisions(BroadPhaseCollisions* c) {
        if (mBroadPhaseCollisions) {
            std::cerr << dc_funcinfo << "BroadPhaseCollisions have been set already" << std::endl;
//            delete c;
            return;
        }
        mBroadPhaseCollisions = c;
    }

    /*!
     * Add \p collisionInfos to the middlephase (rigid) collisions. The given parameter
     * is added as fast as possible to the results, and thus the
     * parameter may be unusable after this method has been called.
     *
     * \param collisionInfos The list that is to be added to the results. Don't
     *        use this anymore after calling this method!
     */
    void WorldCollisions::setRigidBoundingVolumeCollisions(std::list<BoundingVolumeCollision>* collisionInfos) {
        if (!collisionInfos) {
            return;
        }
        if (!mRigidBoundingVolumeCollisions.empty()) {
            std::cerr << dc_funcinfo << "rigid middlephase collisions have been set already" << std::endl;
            delete collisionInfos;
            return;
        }
        mRigidBoundingVolumeCollisions.splice(mRigidBoundingVolumeCollisions.end(), *collisionInfos);
    }

    /*!
     * Add \p collisionInfos to the middlephase (deformable) collisions. The given parameter
     * is added as fast as possible to the results, and thus the
     * parameter may be unusable after this method has been called.
     *
     * \param collisionInfos The list that is to be added to the results. Don't
     *        use this anymore after calling this method!
     */
    void WorldCollisions::setDeformableBoundingVolumeCollisions(std::list<BoundingVolumeCollision>* collisionInfos) {
        if (!collisionInfos) {
            return;
        }
        if (!mDeformableBoundingVolumeCollisions.empty()) {
            std::cerr << dc_funcinfo << "deformable middlephase collisions have been set already" << std::endl;
            delete collisionInfos;
            return;
        }
        mDeformableBoundingVolumeCollisions.splice(mDeformableBoundingVolumeCollisions.end(), *collisionInfos);
    }

    /*!
     * Add \p collisionInfos to the narrowphase collisions. The given parameter
     * is added as fast as possible to the results, and thus the
     * parameter may be unusable after this method has been called.
     *
     * \param collisionInfos The list that is to be added to the results. Don't
     *        use this anymore after calling this method! Ownership is NOT
     *        taken.
     */
    void WorldCollisions::addNarrowPhaseCollisions(std::list<CollisionInfo>* collisionInfos) {
        if (!collisionInfos) {
            return;
        }
        mNarrowPhaseCollisions.splice(mNarrowPhaseCollisions.end(), *collisionInfos);
    }
}
/*
 * vim: et sw=4 ts=4
 */
