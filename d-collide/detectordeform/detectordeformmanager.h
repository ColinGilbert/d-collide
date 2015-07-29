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

#ifndef DCOLLIDE_DETECTOR_DEFORM_MANAGER_H
#define DCOLLIDE_DETECTOR_DEFORM_MANAGER_H

#include <list>
#include <vector>
#include "dcollide-global.h"

namespace dcollide {
    class DetectorDeformAlgorithm;
    class Proxy;
    class World;
    class Vector3;
    class Matrix;

    // TODO:
    // 1. we need exactly one object per World object
    //    -> probably construct on construction of World
    //    (DONE)
    // 2. WorldParameters should take a list of deformable algorithms (integers,
    //    i.e. their type) that are requested.
    //    the actual algorithms should be constructed either on construction of
    //    this class, or when prepareSimulation() is called
    //    (DONE)
    // 3. we somehow want to allow the user to implement derived classes of the
    //    manager (to reimplement pickAlgorithmFor())
    //    -> probably create algorithms objects on prepareSimulation, this way
    //       a) the DetectorDeformManager created in the World c'tor can
    //          easily be replaced
    //       b) the requested algorithm types are used in prepareSimulation()
    //          only, not before. this the derived class is the only one that
    //          needs to know these types then.
    //    (see below - maybe do not allow subclassing, but rather provide a
    //    DetectorDeformAlgorithmSelector class)

    /*!
     * \brief Manager for \ref DetectorDeformAlgorithm objects.
     *
     * This class is used by \ref World (and \ref Pipeline) to manage \ref
     * DetectorDeformAlgorithm objects, i.e. in particular create and delete
     * them.
     *
     * Normally you do not need to use this class directly.
     *
     * Use \ref WorldParameters::addDeformableAlgorithm before creating the \ref
     * World to make this class
     * create a new \ref DetectorDeformAlgorithm object of that type.
     *
     * This class in particular decides which algorithm (if there is more than
     * one available) is used for a (proxyA,proxyB) pair that is returned by the
     * \ref BroadPhase - see \ref pickAlgorithmFor. Derived classes may want to
     * reimplement that function.
     */
    // TODO: deriving this class + reimplementing pickAlgorithmFor() is not yet
    // easily possible, because we call createAlgorithms() in World right after
    // construction of a DetectorDeformManager.
    // -> we should probably delay it until prepareSimulation(), so the user has
    //    a chance at replacing the DetectorDeformManager before the
    //    algorithms are created.
    //    TODO: or alternatively we could use a
    //    DetectorDeformAlgorithmSelector class which does
    //    pickAlgorithmFor().
    //    -> the object of this class could easily be changed on runtime!
    //    -> 2nd advantage: the user wouldn't have to subclass such a "core"
    //       class as DetectorDeformManager, just to reimplement a single
    //       method!
    class DetectorDeformManager {
        public:
            DetectorDeformManager(World* world);
            virtual ~DetectorDeformManager();

            void createAlgorithms(const std::list<DeformableAlgorithmType>& list);
            void addAlgorithm(DetectorDeformAlgorithm* algorithm);
            inline const std::vector<DetectorDeformAlgorithm*>& getAlgorithms() const;

            virtual DetectorDeformAlgorithm* pickAlgorithmFor(Proxy* proxy1, Proxy* proxy2);

            void registerProxyChangesListener(DetectorDeformAlgorithm* listener);
            void unregisterProxyChangesListener(DetectorDeformAlgorithm* listener);

            // should be used by Pipeline, World and Proxyonly, not public API:
            void prepareAlgorithmsForSimulation();
            void initializeProxy(Proxy*);
            void notifyPipelineStarted();
            void notifyPipelineCompleted();
            void notifyAllCollisionPairsAreAdded();
            void addTopLevelProxy(Proxy*);
            void removeTopLevelProxy(Proxy*);
            void addChildProxy(Proxy*, Proxy*);
            void removeChildProxy(Proxy*, Proxy*);
            void translateProxy(Proxy* proxy, const Vector3& translateBy, bool respectOrientation = true);
            void rotateProxy(Proxy* proxy, const Matrix& rotation, bool respectOrientation = true);
            void setProxyMatrix(Proxy* proxy, const Matrix& newMatrix);
            void deformProxy(Proxy* proxy, const std::vector<Vector3>& vertexMoveArray);
            void deformProxy(Proxy* proxy, unsigned int vertexIndex, const Vector3& vertexPosition);
            // should be used by Pipeline, World and Proxy only (end)

        protected:
            DetectorDeformAlgorithm* createAlgorithm(DeformableAlgorithmType type) const;

        private:
            std::vector<DetectorDeformAlgorithm*> mAllAlgorithms;

            // self-collisions
            std::vector<DetectorDeformAlgorithm*> mSelfCollisionAlgorithms;

            // non-self-collisions
            std::vector<DetectorDeformAlgorithm*> mPairAlgorithms;

            bool mPrepareSimulationCalled;

            World* mWorld;

            std::list<DetectorDeformAlgorithm*> mProxyChangesListeners;
    };

    /*!
     * \return A list of all algorithms managed by this class.
     */
    inline const std::vector<DetectorDeformAlgorithm*>& DetectorDeformManager::getAlgorithms() const {
        return mAllAlgorithms;
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
