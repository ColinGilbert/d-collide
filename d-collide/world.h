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


#ifndef DCOLLIDE_WORLD_H
#define DCOLLIDE_WORLD_H

#include "worldparameters.h"
#include "math/vector.h"
#include "narrowphase/narrowphasestrategy.h"
#include "dcollide-global.h"
#include "datatypes/list.h"

#include <list>

namespace dcollide {

    //-----------classes------------

    class Shape;
    class BroadPhase;
    class Proxy;
    class ProxyFactory;
    struct CollisionInfo;
    class WorldCollisions;
    class CollisionCache;
    class ThreadPool;
    class Pipeline;
    class DebugLog;
    class DebugLogEntry;
    class DetectorDeformManager;
    class DetectorDeformAlgorithm;
    class MeshSplitter;
    class Mesh;
    class BvhNode;

    /*!
     * \brief Central d-collide class that manages proxies and their collisions
     */
    class World {
        public:
            /*!
             * Flags to control the behavior of \ref calculateAllCollisions. These are
             * meant to be OR'ed together.
             */
            // TODO: flags to enable/disable certain phases (e.g.
            // NO_NARROWPHASE) to process broad and middle phases only)
            enum CollisionFlags {
                COLLISIONFLAG_NONE = 0x00,

                /*!
                 * If set, \ref calculateAllCollisions will always use threads. If not
                 * set, alternative algorithms are used, if available.
                 *
                 * Note: this does not necessarily disable threads: if no
                 * non-threaded implementation is available, this flag is
                 * ignored. Set the number of threads in the worker pool (aka
                 * thread pool) of the World class to 0 to disable threads
                 * completely.
                 *
                 * On single core systems this flag may improve performance.
                 */
                COLLISIONFLAG_USE_THREADS = 0x01,

                /*!
                 * If NOT set and \ref COLLISIONFLAG_USE_THREADS is set, the \ref
                 * ThreadPool is filled with as many jobs as possible once the
                 * required information are available (e.g. middlephase jobs are
                 * created once the \ref BroadPhase has the first results
                 * available).
                 *
                 * If set, each phase will be processed independently, e.g.
                 * the middlephase is started only when all broadphase results
                 * are available.
                 *
                 * This flag has no effect if \ref COLLISIONFLAG_USE_THREADS is
                 * not set.
                 *
                 * This flag is meant mainly for debugging and performance
                 * evaluation purposes. Pipelining is recommended to be enabled.
                 */
                COLLISIONFLAG_DISABLE_PIPELINING = 0x02,

                /*!
                 * If set, the middle phase (both, rigid and deformable) will
                 * not be executed, only the broadphase results are calculated.
                 * This also implies COLLISIONFLAG_SKIP_NARROW_PHASE.
                 */
                COLLISIONFLAG_SKIP_MIDDLE_PHASE = 0x04,

                /*!
                 * If set, the narrow phase will
                 * not be executed. Note that depending on the algorithms used
                 * by the middle phase, there may still be some narrowphase
                 * results (some middle phase algorithms calculate the final
                 * narrowphase results directly, e.g. imagespace algorithms).
                 */
                COLLISIONFLAG_SKIP_NARROW_PHASE= 0x08
            };

            World();
            explicit World(const WorldParameters& parameters);
            explicit World(const Vector3& worldDimension);
            World(const Vector3& worldMin, const Vector3& worldMax);
            ~World();

            void addProxy(Proxy* proxy);
            void removeProxy(Proxy* proxy);
            void prepareSimulation();

            void startNextStep();
            void notifyProxyStateChangedSincePreviousStep(Proxy*);
            void undoStep();

            // TODO: due to various reasons we cannot use copy c'tor and/or
            // operator=() on WorldCollisions.
            // therefore we should return a new WorldCollisions pointer, instead
            // of an object on the stack!
            WorldCollisions calculateAllCollisions(unsigned int flags = COLLISIONFLAG_USE_THREADS, Proxy* onlyCollisionsWith = 0);
            WorldCollisions calculateAllCollisionsWith(
                    Proxy* onlyCollisionsWith,
                    unsigned int flags = COLLISIONFLAG_USE_THREADS);

            inline const WorldParameters& getWorldParameters() const;
            inline const std::list<Proxy*>& getDeformableProxies() const;
            inline const std::list<Proxy*>& getSelfcollisionProxies() const;
            inline const std::list<Proxy*>& getRigidProxies() const;
            inline const std::list<Proxy*>& getTopLevelProxies() const;
            void setWorldDimension(const Vector3& worldSize);
            void setWorldMinMax(const Vector3& worldMin, 
                                const Vector3& worldMax, const Vector3* proxyPosition = 0);
            inline const Vector3& getWorldDimension() const;
            inline const Vector3& getWorldMin() const;
            inline const Vector3& getWorldMax() const;
            inline BroadPhase* getBroadPhase() const;
            inline bool prepareSimulationWasCalled() const;
            void setUseCollisionCaching(bool activate);
            inline bool getUseCollisionCaching() const;
            inline CollisionCache* getCollisionCache() const;
            inline ThreadPool* getWorkerPool() const;
            inline int getNumberOfTopLevelProxies();

            void setDetectorDeformManager(DetectorDeformManager* manager);
            DetectorDeformManager* getDetectorDeformManager() const;

            MeshSplitter* createMeshSplitter(Mesh* mesh, BvhNode* topBvhNode);

            //Getters for the strategy-settings
            inline NarrowPhaseStrategy getNarrowPhaseStrategySphereSphere() const;
            inline NarrowPhaseStrategy getNarrowPhaseStrategyBoxBox() const;
            inline NarrowPhaseStrategy getNarrowPhaseStrategyMeshMesh() const;
            inline NarrowPhaseStrategy getNarrowPhaseStrategyBoxSphere() const;
            inline NarrowPhaseStrategy getNarrowPhaseStrategySphereMesh() const;

            //Setters for the stategy-settings
            inline void setNarrowPhaseStrategySphereSphere(NarrowPhaseStrategy newStrategy);
            inline void setNarrowPhaseStrategyBoxBox(NarrowPhaseStrategy newStrategy);
            inline void setNarrowPhaseStrategyMeshMesh(NarrowPhaseStrategy newStrategy);
            inline void setNarrowPhaseStrategyBoxSphere(NarrowPhaseStrategy newStrategy);
            inline void setNarrowPhaseStrategySphereMesh(NarrowPhaseStrategy newStrategy);

            ProxyFactory* getProxyFactory() const;
            Proxy* createProxy(ProxyTypes type = PROXYTYPE_RIGID) const;
            Proxy* createProxy(Shape* shape, ProxyTypes type = PROXYTYPE_RIGID) const;

            Pipeline* getPipeline() const;
            inline DebugLog* getDebugLog() const;
            inline DebugLogEntry* getCurrentDebugLogEntry() const;


        private:
            WorldParameters mWorldParameters;
            ProxyFactory* mProxyFactory;
            std::list<Proxy*> mDeformableProxies;
            std::list<Proxy*> mSelfcollisionProxies;
            std::list<Proxy*> mRigidProxies;

            std::list<Proxy*> mTopLevelProxies;

            int mNumberOfTopLevelProxies;

            bool mPrepareSimulationCalled;

            /*!
             * \brief switch to activate collision-saving
             */
            bool mUseCollisionCaching;
            CollisionCache* mCollisionCache;

            List<Proxy*> mProxyStateChanged;

            DetectorDeformManager* mDetectorDeformManager;

            BroadPhase* mBroadphase;

            ThreadPool* mWorkerPool;
            unsigned int mJobPoolMiddlePhaseRigid;
            Pipeline* mPipeline;

            NarrowPhaseShapeStrategies mNarrowPhaseShapeStrategies;

            DebugLog* mDebugLog;
            DebugLogEntry* mCurrentDebugLogEntry;

            friend class Pipeline; // FIXME: should not be here
            void init(const WorldParameters& worldParameters);
    };


    //------------ Implementation of short methods -------------

    const std::list<Proxy*>& World::getDeformableProxies() const {
        return mDeformableProxies;
    }

    const std::list<Proxy*>& World::getSelfcollisionProxies() const {
        return mSelfcollisionProxies;
    }

    const std::list<Proxy*>& World::getRigidProxies() const {
        return mRigidProxies;
    }

    const std::list<Proxy*>& World::getTopLevelProxies() const {
        return mTopLevelProxies;
    }

    const WorldParameters& World::getWorldParameters() const {
        return mWorldParameters;
    }

    const Vector3& World::getWorldDimension() const {
        return mWorldParameters.getWorldDimension();
    }
    const Vector3& World::getWorldMin() const {
        return mWorldParameters.getWorldMin();
    }
    const Vector3& World::getWorldMax() const {
        return mWorldParameters.getWorldMax();
    }
    BroadPhase* World::getBroadPhase() const {
        return mBroadphase;
    }
    bool World::prepareSimulationWasCalled() const{
        return mPrepareSimulationCalled;
    }
    bool World::getUseCollisionCaching() const{
        return mUseCollisionCaching;
    }
    CollisionCache* World::getCollisionCache() const{
        return mCollisionCache;
    }
    ThreadPool* World::getWorkerPool() const {
        return mWorkerPool;
    }
    int World::getNumberOfTopLevelProxies() {
        return mNumberOfTopLevelProxies;
    }

    NarrowPhaseStrategy World::getNarrowPhaseStrategySphereSphere() const {
        return mNarrowPhaseShapeStrategies.mStrategySphereSphere;
    }

    NarrowPhaseStrategy World::getNarrowPhaseStrategyBoxBox() const {
        return mNarrowPhaseShapeStrategies.mStrategyBoxBox;
    }

    NarrowPhaseStrategy World::getNarrowPhaseStrategyMeshMesh() const {
        return mNarrowPhaseShapeStrategies.mStrategyMeshMesh;
    }

    NarrowPhaseStrategy World::getNarrowPhaseStrategyBoxSphere() const {
        return mNarrowPhaseShapeStrategies.mStrategyBoxSphere;
    }
    
    NarrowPhaseStrategy World::getNarrowPhaseStrategySphereMesh() const {
        return mNarrowPhaseShapeStrategies.mStrategySphereMesh;
    }

    void World::setNarrowPhaseStrategySphereSphere(NarrowPhaseStrategy newStrategy) {
        mNarrowPhaseShapeStrategies.mStrategySphereSphere = newStrategy;
    }

    void World::setNarrowPhaseStrategyBoxBox(NarrowPhaseStrategy newStrategy) {
        mNarrowPhaseShapeStrategies.mStrategyBoxBox = newStrategy;
    }

    void World::setNarrowPhaseStrategyMeshMesh(NarrowPhaseStrategy newStrategy) {
        mNarrowPhaseShapeStrategies.mStrategyMeshMesh = newStrategy;
    }
    void World::setNarrowPhaseStrategyBoxSphere(NarrowPhaseStrategy newStrategy) {
        mNarrowPhaseShapeStrategies.mStrategyBoxSphere = newStrategy;
    }
    void World::setNarrowPhaseStrategySphereMesh(NarrowPhaseStrategy newStrategy) {
        mNarrowPhaseShapeStrategies.mStrategySphereMesh = newStrategy;
    }

    /*!
     * \return The \ref DebugLog object of the \ref World class. A new entry is
     * started in every \ref calculateAllCollisions call. This method is mainly used
     * for debugging and performance measuring.
     */
    inline DebugLog* World::getDebugLog() const {
        return mDebugLog;
    }

    /*!
     * \return The current \ref DebugLogEntry of the \ref DebugLog (see
     * \ref getDebugLog) or NULL if there is no "current" entry.
     * A current entry does only exist during a \ref calculateAllCollisions call (but
     * does not have to exist at all - e.g. for performance reasons none may be
     * created at all).
     */
    inline DebugLogEntry* World::getCurrentDebugLogEntry() const {
        return mCurrentDebugLogEntry;
    }
    
}

#endif // DCOLLIDE_WORLD_H
/*
 * vim: et sw=4 ts=4
 */
