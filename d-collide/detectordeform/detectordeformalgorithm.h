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

#ifndef DCOLLIDE_DETECTOR_DEFORM_ALGORITHM_H
#define DCOLLIDE_DETECTOR_DEFORM_ALGORITHM_H

#include <list>
#include <vector>

namespace dcollide {
    class Proxy;
    class CollisionPair;
    class PipelineThreadJobCollection;
    class Pipeline;
    class DetectorDeformProxyData;
    class Vector3;
    class Matrix;

    /*!
     * \brief Base class for deformable collision detection algorithms
     *
     * This class serves as base class/interface for all collision detection
     * algorithms that work on deformable proxies. Normally an object of this
     * class is created by the \ref DetectorDeformManager, usually through
     * the \ref WorldParameters class (see \ref
     * WorldParameters::addDeformableAlgorithm).
     *
     * Algorithms are meant to subclass this class and implement the required
     * functions. In particular \ref createCollisionJobFor should be
     * implemented, which should create a \ref ThreadJob object that does the
     * actual collision detection and stores the results.
     */
    class DetectorDeformAlgorithm {
        public:
            DetectorDeformAlgorithm(Pipeline* pipeline);
            virtual ~DetectorDeformAlgorithm();

            /*!
             * \return TRUE if this algorithm implements self-collisions, FALSE
             * otherwise
             */
            virtual bool supportsSelfCollisions() const = 0; // TODO: get prefix?

            /*!
             * Normally every algorithm should return TRUE here. The only
             * algorithms that may consider returning FALSE here are those that
             * implement self-collisions \em only.
             *
             * \return TRUE if this algorithms implements pair collisions, that
             * is collisions between different proxies. FALSE otherwise.
             */
            virtual bool supportsPairCollisions() const = 0; // TODO: get prefix?

            /*!
             * \return TRUE if this algorithm wants \ref translateProxy, \ref
             *         rotateProxy, \ref setProxyMatrix and \ref deformProxy to
             *         be called when appropriate, otherwise FALSE (no \ref
             *         Proxy movements are forwarded then).
             */
            virtual bool getListenForProxyChanges() const = 0;

            /*!
             * Called once by \ref World::prepareSimulation to initialize this
             * algorithm according to the state of the \ref World.
             *
             * At this point e.g. data structures for \ref Proxy objects in the \ref
             * World should be created. The user triggers this call as soon as a
             * significant portion of her proxies have been added to the \ref
             * World.
             *
             * Note that once this method has been called, new and removed
             * proxies as well as proxy movements have to be traced by this
             * class - there will not be another call to \ref prepareSimulation.
             */
            virtual void prepareSimulation() = 0;

            virtual void addTopLevelProxy(Proxy* proxy);
            virtual void removeTopLevelProxy(Proxy* proxy);

            virtual void addChildProxy(Proxy* parent, Proxy* child);
            virtual void removeChildProxy(Proxy* parent, Proxy* child);

            virtual void translateProxy(Proxy* proxy, const Vector3& translateBy);
            virtual void rotateProxy(Proxy* proxy, const Matrix& rotation);
            virtual void setProxyMatrix(Proxy* proxy, const Matrix& newMatrix);
            virtual void deformProxy(Proxy* proxy, const std::vector<Vector3>& vertexMoveArray);
            virtual void deformProxy(Proxy* proxy, unsigned int vertexIndex, const Vector3& vertexPosition);

            /*!
             * Called by the \ref Pipeline whenever a new possibly colliding
             * pair has been found. This method is meant to create a \ref
             * ThreadJob object and add it to the \ref getJobCollection.
             */
            virtual void createCollisionJobFor(const CollisionPair& pair) = 0;

            virtual void notifyAllCollisionPairsAreAdded();

            virtual void resetAlgorithm();

            // AB: this method is NOT virtual on purpose. see also the
            // documentation of this method.
            // this method is meant to call getJobCollection()->start(), to make
            // sure it is called in every pipeline run, even if the derived
            // class forgets to.
            void notifyPipelineStarted();

            // AB: this method is NOT virtual on purpose. see also the
            // documentation of this method.
            // this method is meant to call
            // getJobCollection()->resetCollection(), to make sure it is called
            // in every pipeline run, even if the derived class forgets to.
            void notifyPipelineCompleted();

            virtual DetectorDeformProxyData* createProxyData(Proxy* proxy);
            DetectorDeformProxyData* getProxyDataFor(Proxy* proxy) const;

            void setProxyDataIndex(unsigned int index);
            inline unsigned int getProxyDataIndex() const;

        protected:
            // AB: protected, so that we don't use it directly in Pipeline and
            // friends.
            // --> if we had access to it, we could use addStartOnceCompleted(),
            //     however the algorithms might want to use that function
            //     internally, e.g. for spatialhash:
            //     getJobCollection()==phase1:
            //     phase1->addStartOnceCompleted(phase2)
            //     --> as soon as phase 1 is completed we start phase 2.
            //
            //     however if the pipeline uses addStartOnceCompleted():
            //     deformableAlgorithm->addStartOnceCompleted(narrowPhase);
            //     --> now "narrowPhase" would be started once _phase1_ is
            //         completed, not once the algorithm is completed.
            //         to avoid this confusion, we simply disallow using adding
            //         something to the collection from outside this algorithm.
            virtual PipelineThreadJobCollection* getJobCollection();

        private:
            Pipeline* mPipeline;
            PipelineThreadJobCollection* mJobCollection;
            unsigned int mProxyDataIndex;

    };

    /*!
     * \return The index this algorithm uses for the \ref
     * Proxy::getDetectorDeformProxyData.
     */
    inline unsigned int DetectorDeformAlgorithm::getProxyDataIndex() const {
        return mProxyDataIndex;
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
