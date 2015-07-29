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


#ifndef DCOLLIDE_WORLDPARAMETERS_H
#define DCOLLIDE_WORLDPARAMETERS_H

#include "math/vector.h"
#include "dcollide-global.h"

#include <list>

namespace dcollide {
    class MeshSplitterFactory;

    /*!
     * \brief Parameters to the \ref World object
     *
     * This class defines parameters such as the world size for the \ref World
     * object. It is meant to be created by the user of the library before
     * creating a \ref World object and then provide it as parameter to the \ref
     * World constructor.
     *
     * Once the \ref World object has been constructed, the parameters are
     * available only as a read-only version through the \ref
     * World::getWorldParameters object. They can not be changed directly
     * afterwards. However the \ref World class may provide methods for some of
     * the values that can be changed afterwards, e.g. \ref
     * World::setWorldDimension and \ref World::setWorldMinMax.
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    class WorldParameters {
        public:
            WorldParameters();
            WorldParameters(const WorldParameters& worldParameters);
            ~WorldParameters();

            WorldParameters& operator=(const WorldParameters& worldParameters);

            bool setWorldMinMax(const Vector3& worldMin,
                const Vector3& worldMax, const Vector3* proxyPosition = 0);
            bool setWorldDimension(const Vector3& worldDimension);

            void setWorkerThreadCount(unsigned int threads);

            void setAllowLocalCoordinates(bool value);
            void setBroadPhaseType(BroadPhaseType type);
            void setRigidBoundingVolumeType(BoundingVolumeType type);
            void setRigidMaxMeshPartTriangles(unsigned int max);

            void addPrimaryDeformableAlgorithm(DeformableAlgorithmType type);
            void addDeformableAlgorithm(DeformableAlgorithmType type);
            void setDeformableAlgorithms(const std::list<DeformableAlgorithmType>& list);

            void setMeshSplitterFactory(MeshSplitterFactory* splitter);

            inline const Vector3& getWorldDimension() const;
            inline const Vector3& getWorldMin() const;
            inline const Vector3& getWorldMax() const;
            unsigned int getWorkerThreadCount() const;
            inline bool getAllowLocalCoordinates() const;
            BroadPhaseType getBroadPhaseType() const;
            BoundingVolumeType getRigidBoundingVolumeType() const;
            unsigned int getRigidMaxMeshPartTriangles() const;
            const std::list<DeformableAlgorithmType>& getDeformableAlgorithms() const;
            inline MeshSplitterFactory* getMeshSplitterFactory() const;

        protected:
            Vector3 adjustToNextPower(const Vector3& vec) const;
            Vector3 adjustToLastPower(const Vector3& vec) const;

        private:
            Vector3 mWorldMin;
            Vector3 mWorldMax;
            Vector3 mWorldDimension;
            bool mAllowLocalCoordinates;
            BroadPhaseType mBroadPhaseType;
            BoundingVolumeType mRigidBoundingVolumeType;
            std::list<DeformableAlgorithmType> mDeformableAlgorithms;
            MeshSplitterFactory* mMeshSplitterFactory;
            unsigned int mWorkerThreadCount;
            unsigned int mRigidMaxMeshPartTriangles;
    };

    /*!
     * \returns Dimension of the world
     */
    inline const Vector3& WorldParameters::getWorldDimension() const {
        return mWorldDimension;
    }

    /*!
     * \returns minimun of the world
     */
    inline const Vector3& WorldParameters::getWorldMin() const {
        return mWorldMin;
    }

    /*!
     * \returns maximum of the world
     */
    inline const Vector3& WorldParameters::getWorldMax() const {
        return mWorldMax;
    }

    inline bool WorldParameters::getAllowLocalCoordinates() const {
        return mAllowLocalCoordinates;
    }

    /*!
     * \return The \ref MeshSplitterFactory used by the world. See \ref
     * setMeshSplitterFactory.
     */
    inline MeshSplitterFactory* WorldParameters::getMeshSplitterFactory() const {
        return mMeshSplitterFactory;
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
