/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
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

#include "proxy.h"

#include "math/matrix.h"
#include "bvhnodedefault.h"
#include "debug.h"
#include "collisioncache.h"
#include "timing.h"
#include "world.h"
#include "debugstream.h"
#include "meshsplitter.h"
#include "shapes/shape.h"
#include "shapes/mesh.h"
#include "boundingvolumes/aabb.h"
#include "broadphase/broadphase.h"
#include "detectordeform/detectordeformproxydata.h"
#include "detectordeform/detectordeformmanager.h"
#include "detectordeform/detectordeformalgorithm.h"
#include "exceptions/exception.h"
#include "broadphase/broadphaseproxydata.h"

namespace dcollide {
    /*!
     * \brief Helper class for to save the current state of \ref Proxy
     */
    class ProxyState {
        public:
            ProxyState(Proxy* proxy);

            void saveProxyState();
            void notifyProxyStateOfChildSaved();
            void restoreProxyState();
            void discardProxyStateAndResetMoveFlags();

            bool isSaved() const;
            bool getChildIsSaved() const;

            const Matrix& getMatrix() const;

        private:
            bool mIsSaved;
            bool mChildIsSaved; // isSaved() of at least one child is true
            Proxy* mProxy;
            Matrix mMatrix;
            int mType;
    };

    ProxyState::ProxyState(Proxy* proxy) {
        mProxy = proxy;
        mType = PROXYTYPE_RIGID;
        mIsSaved = false;
        mChildIsSaved = false;
    }

    /*!
     * \return TRUE if this object currently provides a previous state,
     * otherwise FALSE.
     */
    bool ProxyState::isSaved() const {
        return mIsSaved;
    }

    /*!
     * \return TRUE if \ref isSaved of at least one child proxy is true,
     * otherwise FALSE.
     */
    bool ProxyState::getChildIsSaved() const {
        return mChildIsSaved;
    }

    const Matrix& ProxyState::getMatrix() const {
        return mMatrix;
    }

    void ProxyState::saveProxyState() {
        if (isSaved()) {
            error() << dc_funcinfo << "already a state saved! overwriting...";
        }
        if (!mProxy->isInHierarchy()) {
            return;
        }
        if (mProxy->mMoveFlags != Proxy::MOVEFLAG_UNMOVED) {
            throw Exception("Proxy MoveFlags is not MOVEFLAG_UNMOVED! (ProxyState should have been saved before already)");
        }
        mIsSaved = true;
        mMatrix = *mProxy->mMatrix;
        mType = mProxy->mType;

        // TODO: deformations?
        //       open question: do we want to support un-doing of deformations?

        if (mProxy->getParent()) {
            mProxy->getParent()->mPreviousState->notifyProxyStateOfChildSaved();
        } else {
            mProxy->getWorld()->notifyProxyStateChangedSincePreviousStep(mProxy);
        }

        // note: we _explicitly_ do NOT save:
        // - children added since the previous frame
        // - children removed since the previous frame
        // - the "isInHierarchy" flag (i.e. whether the proxy is added to the
        //    world/to a parent)
        // --> if we saved these, we had e.g. to store all Shape objects of the
        //     proxies when they get removed. so either we had to make a copy,
        //     or disallow deleting them.
        //     both are really bad ideas (and making it dependable would be a
        //     lot of work).
        //
        // also note that some things dont need to be stored:
        // - bounding volume
        // - BroadPhaseProxyData
        // --> they are updated when setTransformation() is called on restoreProxyState()
        //     anyway.
    }

    void ProxyState::notifyProxyStateOfChildSaved() {
        if (mChildIsSaved) {
            return;
        }
        mChildIsSaved = true;

        if (mProxy->getParent()) {
            mProxy->getParent()->mPreviousState->notifyProxyStateOfChildSaved();
        } else {
            mProxy->getWorld()->notifyProxyStateChangedSincePreviousStep(mProxy);
        }
    }

    /*!
     * Restores the state when \ref saveProxyState was called the last time.
     *
     * In particular this method
     * \li Restores the previously saved state, if \ref isSaved is TRUE
     * \li Calls \ref restoreProxyState of all child proxies, if \ref
     *     getChildIsSaved is TRUE
     * \li Does nothing otherwise
     */
    void ProxyState::restoreProxyState() {
        if (!isSaved() && !getChildIsSaved()) {
            return;
        }
        if (getChildIsSaved()) {
            const std::list<Proxy*>& children = mProxy->getChildProxies();
            for (std::list<Proxy*>::const_iterator it = children.begin(); it != children.end(); ++it) {
                (*it)->mPreviousState->restoreProxyState();
            }
        }
        if (isSaved()) {
            mProxy->setTransformation(mMatrix);
            mProxy->mMoveFlags = Proxy::MOVEFLAG_UNMOVED;

            if (mProxy->mType != mType) {
                mProxy->setProxyType(mType);
            }
        }


        // AB: the caller is responsible for removing the Proxy from the
        //     World::mProxyStateChanged list!


        // state has been restored, this object does not store the previous
        // state anymore
        // note: we set these at the very end, so that restoreProxyState() won't
        //       trigger a call to saveProxyState()
        mIsSaved = false;
        mChildIsSaved = false;
    }

    void ProxyState::discardProxyStateAndResetMoveFlags() {
        if (getChildIsSaved()) {
            const std::list<Proxy*>& children = mProxy->getChildProxies();
            for (std::list<Proxy*>::const_iterator it = children.begin(); it != children.end(); ++it) {
                (*it)->mPreviousState->discardProxyStateAndResetMoveFlags();
            }
        }
        mIsSaved = false;
        mChildIsSaved = false;
        mProxy->mMoveFlags = Proxy::MOVEFLAG_UNMOVED;
    }

    /*!
     * \brief Creates a Proxy without shape (used as container for child-proxies)
     *
     * This method should not used directly. Use \ref World::createProxy
     * instead.
     *
     * The proxy is created at the base point of the world coordinates,
     * and starts unrotated - i.e. it uses an identity matrix in \ref
     * getState
     *
     * \param type The type (rigid, deformable, ...) of this proxy, see \ref
     * ProxyTypes. By default a rigid shape is created.
     */
    Proxy::Proxy(World* world, ProxyTypes type) {
        mWorld = world;
        mIsInHierarchy = false;
        mParent = 0;
        mShape = 0;
        mType = type;
        mPreviousState = new ProxyState(this);
        mMatrix = new Matrix();
        mMoveFlags = MOVEFLAG_UNMOVED;
        mBroadPhaseProxyData = 0;
        mBvHierarchyNode = 0;
        mBvHierarchyNodeOfShape = 0;
        mBvUpdatesEnabled = true; //Automatic update is enabled by default
        mUser1 = 0;
        mUser2 = 0;
        
        mDebugOutputPrefix = "";
        mPrintDebugOutput = false;

        if (mType & PROXYTYPE_FIXED) {
            mType = (ProxyTypes)(mType | PROXYTYPE_RIGID);
        }

        //if (mType & PROXYTYPE_RIGID) {
            mBvHierarchyNode = new BvhNodeDefault(getWorld(), this);
            mBvHierarchyNode->initializeNode();
        //}
    }

    /*!
     * \brief Creates a Proxy with a shape
     *
     * This method should not used directly. Use \ref World::createProxy
     * instead.
     *
     * The proxy is created at the base point of the world coordinates,
     * and starts unrotated - i.e. it uses an identity matrix in \ref
     * getState
     *
     * \param shape The shape (sphere, box, mesh, ...) of this proxy.
     * OWNERSHIP NOTICE: the proxy takes ownership of the shape
     *
     * \param type The type (rigid, deformable, ...) of this proxy, see \ref
     *              ProxyTypes. By default a rigid shape is created.
     *              If the shape is not a Mesh, the type-flags "closed hull" and
     *              "convex" are set automatically.
     *              If you create a Mesh, you need to set these flags manually. 
     */
    Proxy::Proxy(World* world, Shape* shape, ProxyTypes type) {
        if (shape != 0 && shape->getShapeType() == Shape::SHAPE_TYPE_MESH_PART) {
            throw MeshPartProxyException();
        }
        if (!world) {
            throw NullPointerException("Parameter world");
        }
        if (type & PROXYTYPE_DEFORMABLE) {
            if (shape) {
                if (   (shape->getShapeType() != Shape::SHAPE_TYPE_MESH)
                    && (shape->getShapeType() != Shape::SHAPE_TYPE_MESH_PART)) {
                    
                    throw Exception("Deformable proxies must not contain shapes other than meshes!");
                }
            }
        }


        mWorld = world;
        mIsInHierarchy = false;
        mParent = 0;
        mShape = shape;
        mType = type;
        mPreviousState = new ProxyState(this);
        mMatrix = new Matrix();
        mMoveFlags = MOVEFLAG_UNMOVED;
        mBroadPhaseProxyData = 0;
        mBvHierarchyNode = 0;
        mBvHierarchyNodeOfShape = 0;
        mBvUpdatesEnabled = true; //Automatic update is enabled by default
        mUser1 = 0;
        mUser2 = 0;
        
        mDebugOutputPrefix = "";
        mPrintDebugOutput = false;

        if (mType & PROXYTYPE_FIXED) {
            mType = (ProxyTypes)(mType | PROXYTYPE_RIGID);
        }

        //if (mType & PROXYTYPE_RIGID) {
            mBvHierarchyNode = new BvhNodeDefault(getWorld(), this);
            mBvHierarchyNode->initializeNode();
        //}

        if (mShape) {
            mShape->setProxy(this);
            mBvHierarchyNodeOfShape = new BvhNodeDefault(getWorld(), mShape);
            mBvHierarchyNodeOfShape->initializeNode();

            if (shape->getShapeType() != Shape::SHAPE_TYPE_MESH) {
                mType |= PROXYTYPE_CLOSEDHULL;
                mType |= PROXYTYPE_CONVEX;
            }

            if (mType & PROXYTYPE_RIGID) {

                if (mShape->getShapeType() == Shape::SHAPE_TYPE_MESH) {
                    MeshSplitter* splitter = getWorld()->createMeshSplitter(
                                static_cast<Mesh*>(mShape),
                                mBvHierarchyNodeOfShape);
                    splitter->startSplitting();
                }
            }

            mBvHierarchyNode->addChild(mBvHierarchyNodeOfShape);
        }
    }


    /*!
     * \brief proxy destructor
     */
    Proxy::~Proxy() {
        delete mBroadPhaseProxyData;
        for (std::vector<DetectorDeformProxyData*>::iterator it = mDetectorDeformProxyData.begin();
                it != mDetectorDeformProxyData.end(); ++it) {
            delete *it;
        }
        mDetectorDeformProxyData.clear();
        delete mPreviousState;
        delete mMatrix;
        while (!mChildProxies.empty()) {
            Proxy* child = mChildProxies.front();
            mChildProxies.pop_front();
            delete child;
        }
        if (mParent) {
            mParent->mChildProxies.remove(this);
        }
        //prevent double deletion: delete only if it was not already deleted
        if (mBvHierarchyNode != NULL ) {
            delete mBvHierarchyNode;
        }
        delete mShape;

    }

    /*!
     * \brief Add a child proxy to this Proxy.
     *
     * Creates or extends the proxy hierarchy by adding the \p child to
     * this proxy
     *
     * OWNERSHIP NOTICE: the proxy takes ownership of the child
     */
    void Proxy::addChild(Proxy* child) {
        if (!child) {
            throw NullPointerException("parameter \"Proxy* child\"");
        }
        if (child->mParent) {
            throw Exception("Moving a proxy to a new parent is not supported. Remove from parent first.");
        }
        if (child->getBvHierarchyNode()->getParent()) {
            throw Exception("BV Hierarchy Node of child already has a parent");
        }
        //The child must have the same type. Exception from this: 
        //the "convex" and "closed hull" flags, which are for shape-proxies only
        //it is ok if these two flags are different in child and proxy
        
        if (   (mType                  | PROXYTYPE_CLOSEDHULL | PROXYTYPE_CONVEX) 
            != (child->getProxyType()  | PROXYTYPE_CLOSEDHULL | PROXYTYPE_CONVEX)) {
            delete child;
            throw TypeMismatchException("Type of parent and child are not equal.");
        }
        if (getWorld() != child->getWorld()) {
            throw InvalidWorldException("Child Proxy must be from the same World as parent Proxy");
        }
        if (!getWorld()->getWorldParameters().getAllowLocalCoordinates()) {
            if (getShape()) {
                throw Exception("Adding a child to a proxy which got a shape isn't supported in combination with the WorldParameter allowLocalCoordinates set to false.");
            }
        }

        //BV of the new child is already calculated
        child->invalidateMeshWorldPositionsRecursive();

        mChildProxies.push_back(child);
        child->mParent = this;
        child->setIsInHierarchy(isInHierarchy());

        mBvHierarchyNode->addChild(child->mBvHierarchyNode);
        mBvHierarchyNode->recalculateBoundingVolumes();

        if (getProxyType() & PROXYTYPE_DEFORMABLE) {
            getWorld()->getDetectorDeformManager()->addChildProxy(this, child);
        }

        if (mIsInHierarchy && mWorld != 0 && mWorld->getUseCollisionCaching() &&
                        child->getBvHierarchyNode()->getBoundingVolume() != 0) {
            mWorld->getCollisionCache()->invalidate(this);
        }
    }

    /*!
     * \brief Remove a child proxy from this Proxy.
     *
     * OWNERSHIP NOTICE: the caller takes ownership of the child for later
     * re-use
     */
    void Proxy::removeChild(Proxy* child) {
        if (!child) {
            throw NullPointerException("parameter \"Proxy* child\"");
        }
        if (child == this) {
            std::cerr << dc_funcinfo
                      << "Deleting himself is not supported!"
                      << std::endl;
            return;
        }

        // Checking if child is a child of this:
        std::list<Proxy*>::iterator iter;
        bool childFound = false;
        for (iter = mChildProxies.begin(); iter!=mChildProxies.end(); iter++) {
            if (child == *iter) {
                mChildProxies.erase(iter);
                childFound = true;
                break;
            }
        }

        if (!childFound) {
            std::cerr << dc_funcinfo
                      << "Proxy is no child of this Proxy!"
                      << std::endl;
            return;
        }

        if (getProxyType() & PROXYTYPE_DEFORMABLE) {
            getWorld()
                ->getDetectorDeformManager()
                ->removeChildProxy(this, child);
        }

        // Update BVHNode:
        mBvHierarchyNode->removeChild(child->mBvHierarchyNode);
        child->setIsInHierarchy(false);
        child->mParent = 0;
        mBvHierarchyNode->recalculateBoundingVolumes();

        // Invalidate CollisionCache if necessary
        if (mIsInHierarchy && mWorld != 0 && mWorld->getUseCollisionCaching() &&
                        child->getBvHierarchyNode()->getBoundingVolume() != 0) {
            mWorld->getCollisionCache()->invalidate(this);
        }
    }

    /*!
     * \brief Translates (i.e. moves) the base point of this proxy
     * 
     * NOTE: Use the parameter respectOrientation to control the behaviour of
     *       the translation.
     * 
     *       Set it to 'true' to let the translation respects the current
     *       orientation of the proxy. The translation happens than in local
     *       (i.e. rotated) coordinates, and not in (unrotated) world
     *       coordinates.
     * 
     *       Otherwise set it to 'false' if you want to translate the proxy in
     *       (unrotated) world coordinates.
     */
    void Proxy::translate(const Vector3& translateBy, bool respectOrientation) {
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "Proxy::translate(translateBy: " << translateBy << ", respectOrientation: " << respectOrientation << ")" << std::endl;
        }

        // Check if we have local coordintes within the hierarchy

        if (!getWorld()->getWorldParameters().getAllowLocalCoordinates()) {

            // We notify all childs that they should translate, the actual
            // translation is only performed on leaf proxies

            if (!mChildProxies.empty()) {

                if (mPrintDebugOutput) {
                    std::cout << mDebugOutputPrefix << "Translation operation is handed to the child proxies!" << std::endl;
                }

                for (std::list<Proxy*>::iterator child = mChildProxies.begin();
                     child != mChildProxies.end();
                     ++child) {
                    
                    (*child)->translate(translateBy, respectOrientation);
                }
                return;

            }
        }

        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "This is a leaf Proxy, it will be translated!" << std::endl;
        }

        invalidateMeshWorldPositionsRecursive();

        if (!mPreviousState->isSaved()) {
            mPreviousState->saveProxyState();
        }

//#define DEBUG_TRANSLATE
#ifdef DEBUG_TRANSLATE
        Timing t;
#endif

        if (     getWorld()->getWorldParameters().getAllowLocalCoordinates()
            || !(getProxyType() & PROXYTYPE_DEFORMABLE)) {

            mMatrix->translate(translateBy, respectOrientation);

            if (mPrintDebugOutput) {
                std::cout << mDebugOutputPrefix << "- Simple state translation done!" << std::endl;
            }

        } else {

            // We haven't got local coordinates and as this is a deformable leaf
            // proxy, so we deform the mesh instead of moving the proxy

            for (unsigned int vertexIndex = 0;
                 vertexIndex < getShape()->getMesh()->getVertexCount();
                 ++vertexIndex) {

                addDeformVector(vertexIndex, translateBy);
            }
            deform();

            if (mPrintDebugOutput) {
                std::cout << mDebugOutputPrefix << "- Translated through a deform process!" << std::endl;
            }
        }


        if (isInHierarchy()) {
            mMoveFlags |= MOVEFLAG_TRANSLATED;
        }


#ifdef DEBUG_TRANSLATE
        std::cout << "transform: " << t.elapsedTimeSinceStart() << std::endl;
        t.restart();
#endif

        // Only move all BVs, we don't need to recalculate them

        Vector3 globalTranslation;

        if (respectOrientation) {
            Vector3 globalNull;
            Matrix m = getWorldTransformation();

            m.transform(&globalNull, Vector3(0,0,0));
            m.transform(&globalTranslation, translateBy);
            globalTranslation = globalTranslation - globalNull;

        } else {
            globalTranslation = translateBy;
        }

        mBvHierarchyNode->translate(globalTranslation);


        // Notify Deformable Algorithms about the translation
        
        if (getProxyType() & PROXYTYPE_DEFORMABLE) {
            getWorld()
                ->getDetectorDeformManager()
                ->translateProxy(this, translateBy, respectOrientation);
        }

#ifdef DEBUG_TRANSLATE
        std::cout << "BV translate: " << t.elapsedTimeSinceStart() << std::endl;
        t.restart();
#endif


        // Notify the BroadPhase and invalidate the cache if needed
        
        if (isInHierarchy() && mWorld != 0) {
            // update BroadPhase: ONLY IF we have a world and a running
            //                    simulation
            if (   mWorld->getBroadPhase() != 0
                && mWorld->prepareSimulationWasCalled()) {

                mWorld->getBroadPhase()->notifyProxyChanged(getToplevelProxy());
            }
            if (mWorld->getUseCollisionCaching()) {
                mWorld->getCollisionCache()->invalidate(this);
            }
        }
#ifdef DEBUG_TRANSLATE
        std::cout << "broadphase update: "
                  << t.elapsedTimeSinceStart()
                  << std::endl;
#endif
#undef DEBUG_TRANSLATE
    }

    /*!
     * \brief Rotates the proxy by a given 3x3 rotation matrix
     *
     * NOTE: The rotation is done around the local coordinate systems axes,
     *       not around the fixed world coordinate ones.
     *  
     * \param rotateBy The rotation matrix used for the rotation. This 4x4
     * matrix is interpreted as a 3x3 matrix, i.e. the last column and row are
     * ignored.
     */
    void Proxy::rotate(const Matrix& rotateBy, bool respectOrientation) {
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "Proxy::rotate(const Matrix& rotateBy)" << std::endl;
        }

        // Check if we have local coordintes within the hierarchy

        if (!getWorld()->getWorldParameters().getAllowLocalCoordinates()) {

            // We notify all childs that they should rotate, the actual
            // rotation is only performed on leaf proxies.

            if (!mChildProxies.empty()) {

                for (std::list<Proxy*>::iterator child = mChildProxies.begin();
                     child != mChildProxies.end();
                     ++child) {

                    (*child)->rotate(rotateBy, respectOrientation);
                }
                return;
            }
        }


        invalidateMeshWorldPositionsRecursive();

        if (!mPreviousState->isSaved()) {
            mPreviousState->saveProxyState();
        }

        if (     getWorld()->getWorldParameters().getAllowLocalCoordinates()
            || !(getProxyType() & PROXYTYPE_DEFORMABLE)) {

            if (respectOrientation) {
                //TODO: check if the 4x4 matrix is a valid rotation matrix
                // see http://www.fastgraph.com/makegames/3Drotation/
                //     -> "properties 1 and 2" might be useful here
                mMatrix->multiply(&rotateBy);
            } else {
                //OH: rotate without transforming the axis to local coordinates
                std::cerr << "Proxy::rotate without respect to Proxy "
                          << "orientation isn't implemented yet!"
                          << std::endl;
            }
        
        } else {
            // We are at a deformable leaf proxy, so we deform the mesh instead
            // of rotating the proxy.

            // Therefor we use the rotation matrix to transform all
            // vertex coordinates seperately.

            Vector3 newPosition;
            const Mesh* mesh = getShape()->getMesh();

            for (unsigned int i = 0; i < mesh->getVertexCount(); ++i) {

                const Vector3& oldPosition
                    = mesh->getVertices()[i]->getPosition();

                rotateBy.transform(&newPosition, oldPosition);
                setVertexPosition(i, newPosition);
            }
        }

        if (isInHierarchy()) {
            mMoveFlags |= MOVEFLAG_ROTATED;
        }

        /* TODO: on some BVs we may be able to make optimizations for rotations!
         * use them! -> no need to recalculate them.
         * (e.g. on AABBs we may actually rotate the AABB and create a new AABB
         * from the rotated one. this makes the new AABB larger than necessary,
         * but it'd be a LOT faster usually)
         * UPDATE: partially done: we have a rotate() function.
         *         however we still need to figure out what parameters to
         *         provide, probably the position of the proxy
         *         (Matrix::getPosition()) and the rotation matrix itself.
         *         -> the first value is the "reference point" of the rotation
         *         TODO 2: can this still work if other opertions but simple
         *                 rotations and translations have been applied?
         * UPDATE: rotation possible if OOBBs are used!
         */
        mBvHierarchyNode->rotate(rotateBy);


        // Notify Deformable Algorithms about the rotation

        if (getProxyType() & PROXYTYPE_DEFORMABLE) {
            getWorld()
                ->getDetectorDeformManager()
                ->rotateProxy(this, rotateBy, respectOrientation);
        }


        // Notify the BroadPhase and invalidate the cache if needed

        if (isInHierarchy() && mWorld != 0) {
            // update BroadPhase: ONLY IF we have a world and a running
            //                    simulation
            // FIXME: Perhaps use different method in broadphase?
            if (   mWorld->getBroadPhase() != 0
                && mWorld->prepareSimulationWasCalled()) {
                
                mWorld->getBroadPhase()->notifyProxyChanged(getToplevelProxy());
            }
            if (mWorld->getUseCollisionCaching()) {
                mWorld->getCollisionCache()->invalidate(this);
            }
        }
#ifdef DEBUG_TRANSLATE
        std::cout << "broadphase update: "
                  << t.elapsedTimeSinceStart()
                  << std::endl;
#endif
#undef DEBUG_TRANSLATE
    }

    /*!
     * \overload
     * \brief Rotates the proxy by a given angle around a given axis
     */
    void Proxy::rotate(real angle, real x, real y, real z, bool respectOrientation) {
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "Proxy::rotate(real angle, real x, real y, real z)" << std::endl;
        }
        //OH: should be the axis transformed in the loca coordinate system?
        Matrix rotationMatrix;
        rotationMatrix.rotate(angle, x, y, z);
        rotate(rotationMatrix, respectOrientation);
    }

    /*!
     * \brief Deforms the proxy
     * 
     * Applies the prior given deformations vectors.
     */
    void Proxy::deform() {
        deformInternal();
    }

    /*!
     * \brief Deforms the proxy
     *
     * Can only be applied to proxies containing meshes. The Proxy has to be
     * of type PROXYTYPE_DEFORMABLE. The n-th vertex of the mesh is translated
     * by vertexMoveArray[n-1].
     *
     * \p vertexMoveArray must be of the same size as the vertex-list, otherwise
     *    a MeshDeformException is thrown. See Mesh::deform for more details.
     */
    void Proxy::deform(const std::vector<Vector3>& vertexMoveArray) {
        if (!vertexMoveArray.empty()) {
            deformInternal(vertexMoveArray);
        } else {
            throw MeshDeformException();
        }
    }

    /*!
     * \brief Calls Mesh::addDeformVector(unsigned int, const Vector3&)
     * 
     * This method aplies only for Proxy of type PROXYTYPE_DEFORMABLE which
     * contains meshes. See Mesh::addDeformVector(unsigned int, const Vector3&)
     * for details on the parameters and usage.
     */
    void Proxy::addDeformVector(unsigned int vertexIndex, const Vector3& deformVector) {
        if (!getShape()) {
            return;
        }

        Shape* s = getShape();

        if (    (s->getShapeType() != Shape::SHAPE_TYPE_MESH)
            || !(getProxyType() & PROXYTYPE_DEFORMABLE)) {

            return;
        }

        Mesh* mesh = static_cast<Mesh*>(s);

        mesh->addDeformVector(vertexIndex, deformVector);
    }

    /*!
     * \brief Calls Mesh::discardDeformVectors()
     * 
     * This method aplies only for Proxy of type PROXYTYPE_DEFORMABLE which
     * contains meshes. See Mesh::discardDeformVectors() for details on the
     * parameters and usage.
     */
    void Proxy::discardDeformVectors() {
        if (!getShape()) {
            return;
        }
        
        Shape* s = getShape();
        
        if (    (s->getShapeType() != Shape::SHAPE_TYPE_MESH)
            || !(getProxyType() & PROXYTYPE_DEFORMABLE)) {

            return;
        }
        
        Mesh* mesh = static_cast<Mesh*>(s);
        
        mesh->discardDeformVectors();    
    }
    
    /*!
     * \internal
     * \brief Performs the actual Proxy deformation for Proxy::deform() and
     *        Proxy::deform(const std::vector<Vector3>&)
     * 
     * When an empty std::vector is given we call the Mesh::deform method
     * instead of Mesh::deform(const std::vector<Vector3>&).
     */
    void Proxy::deformInternal(const std::vector<Vector3>& vertexMoveArray) {
        if (!getShape()) {
            return;
        }
        Shape* s = getShape();
        if (    (s->getShapeType() != Shape::SHAPE_TYPE_MESH)
            || !(getProxyType() & PROXYTYPE_DEFORMABLE)) {

            return;
        }

        if (!mPreviousState->isSaved()) {
            mPreviousState->saveProxyState();
        }

        Mesh* mesh = static_cast<Mesh*>(s);

        if (isInHierarchy()) {
            mMoveFlags |= MOVEFLAG_DEFORMED;
        }
        if (!vertexMoveArray.empty()) {
            mesh->deform(vertexMoveArray);
        } else {
            mesh->deform();
        }

        //mBvHierarchyNode->deform();
        getWorld()
            ->getDetectorDeformManager()
            ->deformProxy(this, vertexMoveArray);

        if (isInHierarchy() && mWorld != 0) {
            // update BroadPhase: ONLY IF we have a world and a running
            //                    simulation
            // FIXME: Perhaps use different method in broadphase?
            if (   mWorld->getBroadPhase() != 0
                && mWorld->prepareSimulationWasCalled()) {
                
                mWorld->getBroadPhase()->notifyProxyChanged(getToplevelProxy());
            }
            if (mWorld->getUseCollisionCaching()) {
                mWorld->getCollisionCache()->invalidate(this);
            }
        }
    }
    

    /*!
     * \brief Apply an arbitrary transformation matrix to the proxy.
     *
     * Replaces the internal transformation matrix by \p state. Usually methods
     * like \ref translate, \ref rotate etc. should be preferred due to
     * performance reasons.
     */
    void Proxy::setTransformation(const Matrix& state) {
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "Proxy::setTransformation(const Matrix& state)" << std::endl;
        }

        if (!getWorld()->getWorldParameters().getAllowLocalCoordinates()) {
            if (   !mChildProxies.empty()
                || (getProxyType() & PROXYTYPE_DEFORMABLE)) {
                
                return;
            }
        }


        invalidateMeshWorldPositionsRecursive();

        if (!mPreviousState->isSaved()) {
            mPreviousState->saveProxyState();
        }

        *mMatrix = state;
        if (isInHierarchy()) {
            mMoveFlags |= MOVEFLAG_ROTATED | MOVEFLAG_TRANSLATED | MOVEFLAG_CUSTOM_TRANSFORMATION;
        }

        mBvHierarchyNode->changeMatrix();

        if (getProxyType() & PROXYTYPE_DEFORMABLE) {
            getWorld()->getDetectorDeformManager()->setProxyMatrix(this, state);
        }

        if (isInHierarchy() && mWorld != 0) {
            if (   mWorld->getBroadPhase() != 0
                && mWorld->prepareSimulationWasCalled()) {
                
                mWorld->getBroadPhase()->notifyProxyChanged(getToplevelProxy());
            }
            if (mWorld->getUseCollisionCaching()) {
                mWorld->getCollisionCache()->invalidate(this);
            }
        }
    }


    Vector3 Proxy::getPosition() const {
        return getTransformation().getPosition();
    }
    
    /*!
     * See Proxy::setPosition(real x, real y, real z) for details!
     */
    void Proxy::setPosition(const Vector3& position) {
        setPosition(position.getX(), position.getY(), position.getZ());
    }

    /*!
     * \brief Sets the position of this proxy in the "world" of the parent
     * 
     * This method puts this proxy to the given point in the coordinate system
     * of its parent. If it has no parent this is the same as if you set its
     * world coordinates. But please notice when you deactivate the
     * 'AllowLocalCoordinates' WorldParameter this method has no effect! 
     */
    void Proxy::setPosition(real x, real y, real z) {
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "Proxy::setPosition(x: " << x <<", y: " << y << ", z: " << z << ")" << std::endl;
            std::cout << mDebugOutputPrefix << "getAllowLocalCoordinates(): " << getWorld()->getWorldParameters().getAllowLocalCoordinates() << std::endl;
            std::cout << mDebugOutputPrefix << "ProxyType: " << getProxyType() << std::endl;
        }

        if (!getWorld()->getWorldParameters().getAllowLocalCoordinates()) {
            if (   !mChildProxies.empty()
                || (getProxyType() & PROXYTYPE_DEFORMABLE)) {

                if (mPrintDebugOutput) {
                    std::cout << mDebugOutputPrefix << "The setPosition command wasn't actually executed!" << std::endl;
                }

                return;
            }
        }

        Matrix matrix = getTransformation();

        matrix.setElement(0, 3, x);
        matrix.setElement(1, 3, y);
        matrix.setElement(2, 3, z);

        setTransformation(matrix);
    }


    Matrix Proxy::getRotation() const {
        return getTransformation().getRotationMatrix();
    }

    /*!
     * \brief Sets an absolute rotation for this proxy
     *
     * See Proxy::setTransformation more details.
     */
    void Proxy::setRotation(const Matrix& rotation) {
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "Proxy::setRotation(const Matrix& rotation)" << std::endl;
        }

        if (!getWorld()->getWorldParameters().getAllowLocalCoordinates()) {
            if (   !mChildProxies.empty()
                || (getProxyType() & PROXYTYPE_DEFORMABLE)) {
                
                return;
            }
        }

        Matrix state = getTransformation();
        Matrix matrix = rotation;

        matrix.setElement(0, 3, state.getElement(0, 3));
        matrix.setElement(1, 3, state.getElement(1, 3));
        matrix.setElement(2, 3, state.getElement(2, 3));

        setTransformation(matrix);
    }

    /*!
     * \brief Sets an absolute rotation for this proxy
     *
     * See Proxy::setRotation(const Matrix&) for more details.
     *  
     * NOTE: The rotation is handled in the coordinates of the parent proxy!
     */
    void Proxy::setRotation(real angle, real x, real y, real z){
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "Proxy::setRotation(real angle, real x, real y, real z)" << std::endl;
        }

        if (!getWorld()->getWorldParameters().getAllowLocalCoordinates()) {
            if (   !mChildProxies.empty()
                || (getProxyType() & PROXYTYPE_DEFORMABLE)) {
                
                return;
            }
        }


        Matrix state = getTransformation();
        Matrix matrix;

        matrix.rotate(angle, x, y, z);

        matrix.setElement(0, 3, state.getElement(0, 3));
        matrix.setElement(1, 3, state.getElement(1, 3));
        matrix.setElement(2, 3, state.getElement(2, 3));

        setTransformation(matrix);
    }

    /*!
     * \brief Sets the absolute position of a vertex within its mesh
     */
    void Proxy::setVertexPosition(unsigned int vertexIndex, const Vector3& position) {
        if (mPrintDebugOutput) {
            std::cout << mDebugOutputPrefix << "Proxy::setVertexPosition(vertexIndex: " << vertexIndex << ", position: " << position << ")" << std::endl;
        }
        setVertexPositionInternal(vertexIndex, position, std::vector<Vector3>());
    }

    /*!
     * \brief Sets all vertices to the given absolute positions within
     *        their mesh
     */
    void Proxy::setVertexPositions(const std::vector<Vector3>& vertexPositionArray) {
        if (mPrintDebugOutput) {
            std::cout << "Proxy::setVertexPositions(const std::vector<Vector3>& vertexPositionArray)" << std::endl;
        }
        if (!vertexPositionArray.empty()) {
            setVertexPositionInternal(0, Vector3(0,0,0), vertexPositionArray);
        } else {
            throw MeshDeformException();
        }
    }

    /*!
     * \internal
     * \brief The actual setVertexPosition(s) method
     * 
     * To avoid code duplication this method performs the actual positioning of
     * the vertices.
     * 
     * Please use setVertexPosition or setVertexPositions instead!
     */
    void Proxy::setVertexPositionInternal(unsigned int vertexIndex,
                                          const Vector3& position,
                                          const std::vector<Vector3>& vertexPositionArray) {

        if (!getShape()) {
            return;
        }

        Shape* shape = getShape();
        if (    (shape->getShapeType() != Shape::SHAPE_TYPE_MESH)
            || !(getProxyType() & PROXYTYPE_DEFORMABLE)) {

            return;
        }

        if (!mPreviousState->isSaved()) {
            mPreviousState->saveProxyState();
        }

        Mesh* mesh = static_cast<Mesh*>(shape);

        if (isInHierarchy()) {
            mMoveFlags |= MOVEFLAG_DEFORMED;
        }

        if (vertexPositionArray.empty()) {
            mesh->setVertexPosition(vertexIndex, position);
            getWorld()
                ->getDetectorDeformManager()
                ->deformProxy(this, vertexIndex, position);

        } else {
            mesh->setVertexPositions(vertexPositionArray);
            getWorld()
                ->getDetectorDeformManager()
                ->deformProxy(this, vertexPositionArray);

        }

        if (mBvUpdatesEnabled) {
        	mBvHierarchyNode->deform();
        }

        if (isInHierarchy() && mWorld != 0) {
            // update BroadPhase: ONLY IF we have a world and a running
            //                    simulation
            // FIXME: Perhaps use different method in broadphase?
            if (   mWorld->getBroadPhase() != 0
                && mWorld->prepareSimulationWasCalled()) {
                
                mWorld->getBroadPhase()->notifyProxyChanged(getToplevelProxy());
            }
            if (mWorld->getUseCollisionCaching()) {
                mWorld->getCollisionCache()->invalidate(this);
            }
        }
    }


    /*!
     * Internal method. Do \em not call directly! The \ref World may maintain a
     * list of previously saved proxies and therefore the \ref World must be the
     * only class to call this method.
     *
     * This method restores the state of this proxy back to the moment when \ref
     * World::startNextStep was called the last time. The state of the proxy
     * includes:
     * \li The \ref getMatrix
     * \li The \ref getProxyType
     * \li The \ref getMoveFlags
     *
     * It does \em not include (and therefore these are \em never restored):
     * \li Whether this Proxy is in the world (\ref World::addProxy and \ref
     *     World::removeProxy and \ref isInHierarchy)
     * \li Whether this Proxy is child of another Proxy (see \ref addChild, \ref
     *     removeChild and \ref getParent and \ref isInHierarchy)
     */
    void Proxy::restorePreviousState() {
        mPreviousState->restoreProxyState();
    }

    /*!
     * Internal method.
     *
     * Discard any stored changes since the last call to \ref
     * World::startNextStep. This is called by \ref World::startNextStep.
     */
    void Proxy::discardPreviousState() {
        mPreviousState->discardProxyStateAndResetMoveFlags();
    }

    /*!
     * \brief Calculates the world coordinates of this proxy
     *
     * Each proxy stores its position and orientation relative to its parents
     * The world coordinates are calculated by multiplying the state-matrices,
     * beginning with the topmost parent
     */
    Matrix Proxy::getWorldTransformation() const {

        if (!(getWorld()->getWorldParameters().getAllowLocalCoordinates())) {

            if (   !(mChildProxies.empty())
                ||  (getProxyType() & PROXYTYPE_DEFORMABLE)) {

                    // The user has disabled hierarchical coordinates and this
                    // is not a "leaf" proxy or a deformable one. So we return
                    // the identity matrix.

                    return Matrix();

            } else {

                // This is a non deformable leaf proxy so its world state is
                // its local/own state.

                return Matrix(*mMatrix);
            }
        }

        if (mParent == NULL) {
            return Matrix(*mMatrix);
        } else {
            Matrix mWorldState = mParent->getWorldTransformation();
            mWorldState.multiply(mMatrix);
            return mWorldState;
        }
    }

    /*!
     * \return The \ref getWorldState of the previous state, i.e. the value of
     * \ref getWorldState right after the last call to \ref
     * World::startNextStep.
     */
    Matrix Proxy::getPreviousWorldTransformation() const {
        if (mParent == NULL) {
            if (!mPreviousState->isSaved()) {
                return getWorldTransformation();
            }
            return mPreviousState->getMatrix();
        } else {
            Matrix previousWorldState = mParent->getPreviousWorldTransformation();
            if (mPreviousState->isSaved()) {
                previousWorldState.multiply(&mPreviousState->getMatrix());
            } else {
                previousWorldState.multiply(mMatrix);
            }
            return previousWorldState;
        }
    }

    /*!
     * Used internally to invalidate the cache of world positions. See also \ref
     * Shape::invalidateAllWorldPositions
     */
    void Proxy::invalidateMeshWorldPositionsRecursive() {

        // Notify the shape that its vertex coordinates aren't valid anymore
        if (mShape) {
            mShape->invalidateAllWorldPositions();
        }

        // invalidate all children as well
        for (std::list<Proxy*>::iterator iter = mChildProxies.begin();
                    iter != mChildProxies.end(); ++iter) {
            (*iter)->invalidateMeshWorldPositionsRecursive();
        }
    }

    /*!
     *\brief Convenience getter/shortcut to the top-level proxy
     */
    Proxy* Proxy::getToplevelProxy() {
        Proxy* currentProxy = this;
        while (currentProxy->getParent() != 0) {
            currentProxy = currentProxy->getParent();
        }
        return currentProxy;
    }

    /*!
     * Internal method. Do not call directly.
     *
     * See \ref isInHierarchy
     *
     * \internal
     *
     * Sets the "isInHierarchy" flag recursively to this and all child proxies
     * (or removes the flag).
     */
    void Proxy::setIsInHierarchy(bool is) {
        mIsInHierarchy = is;
        for (std::list<Proxy*>::const_iterator it = mChildProxies.begin(); it != mChildProxies.end(); ++it) {
            (*it)->setIsInHierarchy(is);
        }
    }

    /*!
     * \brief Change the type of this proxy and all of its children
     *
     * See also \ref ProxyTypes.
     *
     * \param type \ref ProxyTypes OR'ed together.
     */
    void Proxy::setProxyType(int type) {
        if (type & PROXYTYPE_FIXED) {
            type = (type | PROXYTYPE_RIGID);
        }

        // only if Proxy is Top-Level-Proxy, or if noParentCheck:
        if (getParent() == 0) {
            if (!mPreviousState->isSaved()) {
                mPreviousState->saveProxyState();
            }

            // First remove Proxy from world:
            mWorld->removeProxy(this);

            // Now setting new Type
            mType = type;
            for (std::list<Proxy*>::const_iterator it = mChildProxies.begin(); it != mChildProxies.end(); ++it) {
                // Call special private function, which does not check if parent
                (*it)->setProxyTypeNoParent(type);
            }

            // At last, re-add Proxy to the world:
            mWorld->addProxy(this);

            // removeProxy() has also removed the proxy from the undo list.
            // re-adding...
            mWorld->notifyProxyStateChangedSincePreviousStep(this);
        }
    }

    /*!
     * \brief change the type of this proxy and all of its children
     * Ignore if it is parent or not, also does not remove and re-add, as this
     * makes no sense for non-Parent Proxies!
     * this one should only be called from \ref setProxyType!
     * See also \ref ProxyTypes.
     */
    void Proxy::setProxyTypeNoParent(int type) {

        // Sanity Check: Don't use this method if is parent!
        if (getParent() != 0) {
            if (!mPreviousState->isSaved()) {
                mPreviousState->saveProxyState();
            }

            // Setting new Type
            mType = type;
            for (std::list<Proxy*>::const_iterator it = mChildProxies.begin(); it != mChildProxies.end(); ++it) {
                // Call special private function, which does not check if parent
                (*it)->setProxyTypeNoParent(type);
            }
        }
     }

    /*!
     * \brief get center of the according bounding volume
     */
    Vector3 Proxy::getBoundingVolumeCenter() const {
        if (mBvHierarchyNode->getBoundingVolume()) {
          return (
                  mBvHierarchyNode->getBoundingVolume()->getSurroundingAabbMax()
                 +mBvHierarchyNode->getBoundingVolume()->getSurroundingAabbMin()
                 )/2;
        } else {
            return Vector3(0,0,0);
        }
    }

    /*!
     * \brief get position of the center of the according bounding volume
     */
    Vector3 Proxy::getBoundingVolumeCenterPosition() const {
        if (mBvHierarchyNode->getBoundingVolume()) {
          return (mBvHierarchyNode->getBoundingVolume()->getSurroundingAabbMax()
            + mBvHierarchyNode->getBoundingVolume()->getSurroundingAabbMin())/2;
        } else {
            return Vector3(0,0,0);
        }
    }


    /*!
     * Internal method, used by \ref BroadPhase derived classes.
     *
     * This sets the data used by the \ref BroadPhase. Any previously set data
     * is deleted.
     *
     * The proxy takes ownership of the given data pointer and deletes it on
     * destruction.
     */
    void Proxy::setBroadPhaseProxyData(BroadPhaseProxyData* data) {
        delete mBroadPhaseProxyData;
        mBroadPhaseProxyData = data;
    }

    /*!
     * Internal method, used by \ref DetectorDeformManager only.
     *
     * This sets the data used by the various \ref DetectorDeformAlgorithm
     * derived classes. Any previously set data
     * are deleted.
     *
     * The proxy takes ownership of the given data pointers and deletes them on
     * destruction.
     */
    void Proxy::setDetectorDeformProxyData(const std::vector<DetectorDeformProxyData*>& data) {
        for (std::vector<DetectorDeformProxyData*>::iterator it = mDetectorDeformProxyData.begin();
                it != mDetectorDeformProxyData.end(); ++it) {
            delete *it;
        }
        mDetectorDeformProxyData = data;
    }

    /*!
     * The \ref DetectorDeformAlgorithm derived classes are meant to use
     * this, to retrieve data relevant for that particular algorithm.
     *
     * See also \ref DetectorDeformAlgorithm::createProxyData, which creates
     * these objects and \ref DetectorDeformAlgorithm::getProxyDataIndex
     * which specifies the \p index for the algorithm.
     *
     * \param index The internal index of the algorithm that the
     * DetectorDeformProxyData belongs to. See \ref
     * DetectorDeformAlgorithm::getProxyDataIndex().
     *
     * \return The DetectorDeformProxyData object associated with the \ref
     * DetectorDeformAlgorithm algorithm that is specified by \p index. This
     * object is allowed to be NULL, if the \ref DetectorDeformAlgorithm does
     * not use any \ref DetectorDeformProxyData objects (e.g. if this proxy is
     * not deformable).
     */
    DetectorDeformProxyData* Proxy::getDetectorDeformProxyData(unsigned int index) const {
        if (!(getProxyType() & PROXYTYPE_DEFORMABLE)) {
            return 0;
        }
        if (index >= mDetectorDeformProxyData.size()) {
            throw ArrayIndexOutOfBoundsException("mDetectorDeformProxyData", mDetectorDeformProxyData.size(), index);
        }
        return mDetectorDeformProxyData[index];
    }

    /*!
     * \overload
     *
     * This method retrieves the \ref DetectorDeformProxyData for the given \p
     * algorithm.
     *
     * See also \ref DetectorDeformAlgorithm::getProxyDataIndex()
     *
     * \return The \ref DetectorDeformAlgorithm object for \p algorithm, or NULL
     *         if no such object exists. By default an algorithm has no such
     *         object (see \ref DetectorDeformAlgorithm::createProxyData() to
     *         change that for an algorithm).
     */
    DetectorDeformProxyData* Proxy::getDetectorDeformProxyData(const DetectorDeformAlgorithm* algorithm) const {
        return getDetectorDeformProxyData(algorithm->getProxyDataIndex());
    }

}

/*
 * vim: et sw=4 ts=4
 */
