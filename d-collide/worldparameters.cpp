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


#include "worldparameters.h"

#include "boundingvolumes/boundingvolume.h"
#include "dcollide-defines.h"
#include "meshsplitter.h"

namespace dcollide {
    /*!
     * \brief c'tor of class WorldParameters
     */
    WorldParameters::WorldParameters() {
        // AB: hardcoded default of 4 worker threads (i.e. 5 threads in total,
        // including the control thread).
        mWorkerThreadCount = 4;

        setWorldMinMax(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 1.0, 1.0));

        // currently we provide the spatial hash by default
        mDeformableAlgorithms.push_back(DEFORMABLE_TYPE_SPATIAL_HASH);

        mAllowLocalCoordinates = true;
        mBroadPhaseType = BROADPHASE_TYPE_HIERARCHICALGRID;
        mRigidMaxMeshPartTriangles = MAX_BVHNODE_LEAF_TRIANGLES;
#ifdef USE_AABB
        mRigidBoundingVolumeType = BV_TYPE_AABB;
#else
        mRigidBoundingVolumeType = BV_TYPE_KDOP;
#endif

        mMeshSplitterFactory = new MeshSplitterFactory();
    }

    /*!
     * \brief copy c'tor of class WorldParameters
     */
    WorldParameters::WorldParameters(const WorldParameters& worldParameters) {
        *this = worldParameters;
    }

    WorldParameters::~WorldParameters() {
        delete mMeshSplitterFactory;
    }

    /*!
     * \brief = operator to assign WorldParameters to each other
     */
    WorldParameters& WorldParameters::operator=(const WorldParameters& worldParameters) {
        mWorkerThreadCount = worldParameters.mWorkerThreadCount;
        mWorldDimension = worldParameters.mWorldDimension;
        mWorldMin = worldParameters.mWorldMin;
        mWorldMax = worldParameters.mWorldMax;
        mAllowLocalCoordinates = worldParameters.mAllowLocalCoordinates;
        mBroadPhaseType = worldParameters.mBroadPhaseType;
        mRigidBoundingVolumeType = worldParameters.mRigidBoundingVolumeType;
        mRigidMaxMeshPartTriangles = worldParameters.mRigidMaxMeshPartTriangles;
        mDeformableAlgorithms = worldParameters.mDeformableAlgorithms;
        delete mMeshSplitterFactory;
        mMeshSplitterFactory = worldParameters.mMeshSplitterFactory->clone();
        return *this;
    }

    /*!
     * \brief Set the world Min/Max to the given Min/Max values
     *
     * \return TRUE if the previously stored values have been changed
     * \param worldMin new Min Value
     * \param worldMax new Max Value
     * \param proxyPosition if given, calculate new Min/Max Values
     *
     * FIXME: work out the problem of a world with completely positive or
     * negative coordinates, by now a quickfix is used for that
     */
    bool WorldParameters::setWorldMinMax(const Vector3& worldMin,
            const Vector3& worldMax, const Vector3* proxyPosition) {

        Vector3 oldWorldDimension = mWorldDimension;
        Vector3 oldMin = mWorldMin;
        Vector3 oldMax = mWorldMax;

        Vector3 newWorldMin = worldMin;
        Vector3 newWorldMax = worldMax;


        // If proxyPosition is given, calculate new min/max values in such a way
        // that the proxy does fit into the world:
        // This also adjusts the new values to a power of 2
        if (proxyPosition) {

            // First adjust positive old World Mins:
            // FIXME: This should be handled more accurate!
            if (newWorldMin.getX() >= 0) {
              newWorldMin.setX(-2);
            }
            if (newWorldMin.getY() >= 0) {
              newWorldMin.setY(-2);
            }
            if (newWorldMin.getZ() >= 0) {
              newWorldMin.setZ(-2);
            }

           // grow the world Min in x,y and z direction until the proxy fits in
            while (proxyPosition->getX() <= newWorldMin.getX()) {
              newWorldMin.setX(newWorldMin.getX()*2);
            }
            while (proxyPosition->getY() <= newWorldMin.getY()) {
              newWorldMin.setY(newWorldMin.getY()*2);
            }
            while (proxyPosition->getZ() <= newWorldMin.getZ()) {
              newWorldMin.setZ(newWorldMin.getZ()*2);
            }

            // First adjust negative old World Max:
            // FIXME: This should be handled more accurate!
            if (newWorldMax.getX() <= 0) {
              newWorldMax.setX(2);
            }
            if (newWorldMax.getY() <= 0) {
              newWorldMax.setY(2);
            }
            if (newWorldMax.getZ() <= 0) {
              newWorldMax.setZ(2);
            }

           // grow the world Max in x,y and z direction until the proxy fits in
            while (proxyPosition->getX() >= newWorldMax.getX()) {
              newWorldMax.setX(newWorldMax.getX()*2);
            }
            while (proxyPosition->getY() >= newWorldMax.getY()) {
              newWorldMax.setY(newWorldMax.getY()*2);
            }
            while (proxyPosition->getZ() >= newWorldMax.getZ()) {
              newWorldMax.setZ(newWorldMax.getZ()*2);
            }

            // At last: Set the new values
            mWorldMin = newWorldMin;
            mWorldMax = newWorldMax;
        }

        // If not given, only adjust to power of 2
        else {
            mWorldMin = adjustToLastPower(worldMin);
            mWorldMax = adjustToNextPower(worldMax);
        }

        // recalculate World Dimension:
        mWorldDimension = (mWorldMax - mWorldMin);

        if (!mWorldMin.isEqual(oldMin) || !mWorldMax.isEqual(oldMax) || !mWorldDimension.isEqual(oldWorldDimension)) {
            return true;
        }
        return false;
    }

    /*!
     * Set the world size to the given dimension.
     * Min/Max will be calculated using half of dimension as origin
     *
     * The values provided should be something of pow(2,n), however if other
     * values are provided, they are automatically adjusted to the next power of 2.
     *
     * \return TRUE if the previously stored values have been changed, otherwise
     * FALSE. This may be useful to update internal data structures (such as
     * a hierarchical grid) that depend on the world size.
     */
    bool WorldParameters::setWorldDimension(const Vector3& worldDimension) {
        Vector3 oldWorldDimension = mWorldDimension;
        Vector3 oldMin = mWorldMin;
        Vector3 oldMax = mWorldMax;

        mWorldDimension = adjustToNextPower(worldDimension);

        Vector3 worldMax = mWorldDimension / 2.0;
        Vector3 worldMin = mWorldDimension / 2.0;
        worldMin.scale(-1.0);
        mWorldMin = worldMin;
        mWorldMax = worldMax;

        if (!mWorldMin.isEqual(worldMin) || !mWorldMax.isEqual(oldMax) || mWorldDimension.isEqual(oldWorldDimension)) {
            return true;
        }
        return false;
    }

    /*!
     * \brief calc. nearest next power of 2 of vector
     */
    Vector3 WorldParameters::adjustToNextPower(const Vector3& vec1) const {
        // Return Value
        Vector3 newVector;

        // First: Check the values, must be something ^2:
        real givenXvalue = vec1.getX();
        real givenYvalue = vec1.getY();
        real givenZvalue = vec1.getZ();


        real newValue = 2;
        while (newValue < givenXvalue) {
            newValue *= 2;
        }
        // if not equal, adjust to next higher value
        newVector.setX( newValue );

        newValue = 2;
        while (newValue < givenYvalue) {
            newValue *= 2;
        }
        newVector.setY( newValue );

        newValue = 2;
        while (newValue < givenZvalue) {
            newValue *= 2;
        }
        newVector.setZ( newValue );

        return newVector;
    }

    /*!
     * \brief calc. nearest last power of 2 of vector
     */
    Vector3 WorldParameters::adjustToLastPower(const Vector3& vec1) const {

        // Return Value
        Vector3 newVector;

        // First: Check the values, must be something ^2:
        real givenXvalue = vec1.getX();
        real givenYvalue = vec1.getY();
        real givenZvalue = vec1.getZ();

        real lastValue = 0;
        real newValue;

//        std::cout << "VOR:" << vec1 << std::endl;

        // If givenValue is >= 0:
        if (givenXvalue >= 0) {
            newValue = 2;
            while (newValue <= givenXvalue) {
                lastValue = newValue;
                newValue *= 2;
            }
        // If givenValue is < 0
        } else {
            newValue = -2;
            while (newValue >= givenXvalue) {
                lastValue = newValue;
                newValue *= 2;
            }
        }
        // if not equal, adjust to last value
        newVector.setX( lastValue );


        // If givenValue is >= 0:
        lastValue = 0;
        if (givenYvalue >= 0) {
            newValue = 2;
            while (newValue <= givenYvalue) {
                lastValue = newValue;
                newValue *= 2;
            }
        // If givenValue is < 0
        } else {
            newValue = -2;
            while (newValue >= givenYvalue) {
                lastValue = newValue;
                newValue *= 2;
            }
        }
        newVector.setY( lastValue );

        // If givenValue is >= 0:
        lastValue = 0;
        if (givenZvalue >= 0) {
            newValue = 2;
            while (newValue <= givenZvalue) {
                lastValue = newValue;
                newValue *= 2;
            }
        // If givenValue is < 0
        } else {
            newValue = -2;
            while (newValue >= givenZvalue) {
                lastValue = newValue;
                newValue *= 2;
            }
        }
        newVector.setZ( lastValue );

//       std::cout << "NACH:" << newVector << std::endl;


        return newVector;
    }

    /*!
     * \brief Sets the internal switch to allow or disallow local coordinates
     * 
     * When set to true any proxy in a proxy hierarchy can have his own
     * coordinates, this is the way many rendering systems see the world. But as
     * many physic engines don't support such hierarchies you can set this value
     * to false which would lead to global positioning of all leaf proxies, but
     * with the advantage of grouping support. In this case you could put your
     * proxies in a hierarchy but all shapeless proxies wont have any
     * coordinates and all shaped proxies willl be adressed in world coordinates
     * only.
     */
    void WorldParameters::setAllowLocalCoordinates(bool value) {
        mAllowLocalCoordinates = value;
    }
    
    /*!
     * \brief set Type of Broadpase
     * \param type The type
     */
    void WorldParameters::setBroadPhaseType(BroadPhaseType type) {
        mBroadPhaseType = type;
    }

    /*!
     * \returns the type of the BroadPhase
     */
    BroadPhaseType WorldParameters::getBroadPhaseType() const {
        return mBroadPhaseType;
    }

    /*!
     * Rigid bodies maintain a Bounding Volume Hierarchy with a certain bounding
     * volume type (one type for one World object). This value sets which
     * bounding volume type is meant to be used for rigid objects.
     *
     * Note that this value does not necessarily apply to deformable objects.
     *
     * \param type The bounding volume type to be used for rigid objects. See
     * \ref BoundingVolume::BoundingVolumeType for possible values.
     */
    void WorldParameters::setRigidBoundingVolumeType(BoundingVolumeType type) {
        mRigidBoundingVolumeType = type;
    }

    /*!
     * \return The bounding volume type that should be used for rigid proxies.
     * See \ref setRigidBoundingVolumeType
     */
    BoundingVolumeType WorldParameters::getRigidBoundingVolumeType() const {
        return mRigidBoundingVolumeType;
    }

    /*!
     * Parameter to the \ref MeshSplitter class: the \ref MeshSplitter will
     * continue to split a rigid mesh until at most \p max triangles are left.
     *
     * Therefore any rigid leaf \ref MeshPart will have \p max triangles or
     * less.
     */
    void WorldParameters::setRigidMaxMeshPartTriangles(unsigned int max) {
        mRigidMaxMeshPartTriangles = max;
    }

    /*!
     * See \ref setRigidMaxMeshPartTriangles
     */
    unsigned int WorldParameters::getRigidMaxMeshPartTriangles() const {
        return mRigidMaxMeshPartTriangles;
    }

    /*!
     * \param type Add this algorithms to the list of available algorithms for
     * deformable collision detection. The \ref DetectorDeformManager decides
     * which algorithms will be used for a spefic \ref Proxy pair.
     */
    void WorldParameters::addDeformableAlgorithm(DeformableAlgorithmType type) {
        mDeformableAlgorithms.push_back(type);
    }

    /*!
     * Like \ref addDeformableAlgorithm, but \em preprends the algorithm \p type
     * to the internal list, suggesting that is will be used as primary
     * algorithm for deformable collision detection.
     *
     * Note that this is only a hint to \ref DetectorDeformManager, which it
     * may or may not make use of.
     */
    void WorldParameters::addPrimaryDeformableAlgorithm(DeformableAlgorithmType type) {
        mDeformableAlgorithms.push_front(type);
    }

    /*!
     * Like \ref addDeformableAlgorithm but replaces the list of available
     * algorithms (i.e. can also be used to remove algorithms added by default
     * on construction)
     */
    void WorldParameters::setDeformableAlgorithms(const std::list<DeformableAlgorithmType>& list) {
        mDeformableAlgorithms = list;
    }

    /*!
     * \return The algorithm type that will be used for deformable collision
     * detection, see \ref setDeformableDetectorType
     */
    const std::list<DeformableAlgorithmType>& WorldParameters::getDeformableAlgorithms() const {
        return mDeformableAlgorithms;
    }

    /*!
     * Replace the currently set \ref MeshSplitterFactory by \p factory. You
     * need to call this only, if you want to use a different \ref
     * MeshSplitterFactory (and therefore a different \ref MeshSplitter) than
     * the default.
     *
     * OWNERSHIP NOTICE: ownership of the specified \p factory is taken. It is
     * deleted on destruction.
     */
    void WorldParameters::setMeshSplitterFactory(MeshSplitterFactory* factory) {
        delete mMeshSplitterFactory;
        mMeshSplitterFactory = factory;
    }

    /*!
     * Set the number of worker threads that will be created by \ref World for
     * used in (mainly) \ref World::calculateAllCollisions. Use 0 to disable
     * multithreading.
     *
     * Note that these threads are created in addition to the caller thread,
     * which is considered a "control" thread as opposed to the "worker" threads
     * that will be created.
     *
     * The default is to enable multithreading with a certain default number of
     * worker threads (4 at the time of this writing).
     *
     * See also \ref ThreadPool and in particular ThreadPool::ThreadPool for
     * details.
     */
    void WorldParameters::setWorkerThreadCount(unsigned int threadCount) {
        mWorkerThreadCount = threadCount;
    }

    /*!
     * \return The number of worker threads that should be created by \ref
     * World. 0 to disable multithreading. See also \ref setWorkerThreadCount.
     */
    unsigned int WorldParameters::getWorkerThreadCount() const {
        return mWorkerThreadCount;
    }
}
/*
 * vim: et sw=4 ts=4
 */
