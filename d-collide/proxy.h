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

#ifndef DCOLLIDE_PROXY_H
#define DCOLLIDE_PROXY_H

#include "math/vector.h"
#include "dcollide-global.h"
#include "bvhnodedefault.h"

#include <list>
#include <vector>

namespace dcollide {

    class Shape;
    class World;
    class Matrix;
    class BroadPhaseProxyData;
    class DetectorDeformProxyData;
    class DetectorDeformAlgorithm;
    class ProxyState;

    /*!
     * \brief representation of user objects
     *
     *  Proxies represent objects which the user inserted into the world.
     *  They act as containers for a shape and are responsible for managing
     *  position and orientation data. Proxies can be nested.
     *
     * Attention: The Proxy constructors are protected. Use \ref World::createProxy
     * methods to create them for a given world.
     */
    class Proxy {
        public:
            /*!
             * \brief enum/Bitflags of possible Movements of a proxy
             *
             * This enum is used internally to keep track of movements of the
             * proxy. See \ref getMoveFlags.
             */
            enum MoveFlags {
                /*!
                 * The proxy is in the same state as in the last frame
                 */
                MOVEFLAG_UNMOVED    = 0,

                /*!
                 * The proxy was translated within the last step
                 */
                MOVEFLAG_TRANSLATED = 2,

                /*!
                 * The proxy was rotated within the last step
                 */
                MOVEFLAG_ROTATED    = 4,

                /*!
                 * A custom matrix operation has been performed on the Proxy
                 * (see \ref setTransformation). Note that if this is set, \ref
                 * MOVEFLAG_TRANSLATED and \ref MOVEFLAG_ROTATED will also be
                 * set for convenience.
                 */
                MOVEFLAG_CUSTOM_TRANSFORMATION = 8,

                /*!
                 * The shape (Mesh) of the proxy has been deformed, see \ref
                 * deform.
                 */
                MOVEFLAG_DEFORMED   = 16
            };


            /*!
             * \brief User pointer.
             *
             * This pointer is not used by the d-collide library, it is meant to
             * take user-defined data. It is initialized to 0 on construction,
             * after that this library will never change its value.
             */
            void* mUser1;

            /*!
             * \brief User pointer 2.
             *
             * This pointer is not used by the d-collide library, it is meant to
             * take user-defined data. It is initialized to 0 on construction,
             * after that this library will never change its value.
             */
            void* mUser2;

        private:
            /*!
             * The type (rigid, deformable, ...) of this proxy. See \ref
             * ProxyTypes.
             */
            int mType;

            /*!
             * \brief See \ref getMoveFlags
             */
            int mMoveFlags;

            /*! \brief Pointer to the shape of the proxy (can be NULL)
             */
            Shape* mShape;

            /*!
             * \brief 4x4 Matrix for current position and orientation
             */
            Matrix* mMatrix;

            /*!
             * \brief List of children in the proxy-hierarchy
             *
             * Proxies can be nested to describe a hierarchy. Each Proxy keeps
             * track of its children.
             */
            std::list<Proxy*> mChildProxies;

            /*!
             * \brief back-pointer to the parent of this proxy
             */
            Proxy* mParent;

            /*!
             * \brief Pointer to the corresponding Node in the BV hierarchy
             * Bounding volumes of Proxies are organized in a tree-structure
             */
            BvhNode* mBvHierarchyNode;

            BvhNode* mBvHierarchyNodeOfShape;

            World* mWorld;
            bool mIsInHierarchy;

            
            /*!
             * \brief Use this flag to indicate whether the bounding volumes of the bounding volumes of
             * the rigid collision detection should be updated automatically
             */
             bool mBvUpdatesEnabled;
            
            BroadPhaseProxyData* mBroadPhaseProxyData;
            std::vector<DetectorDeformProxyData*> mDetectorDeformProxyData;

            friend class ProxyState;
            ProxyState* mPreviousState;

        protected:
            // Constructor(s) and Destructor
            Proxy(World* world, ProxyTypes type = PROXYTYPE_RIGID);
            Proxy(World* world, Shape* shape, ProxyTypes type = PROXYTYPE_RIGID);
            friend class ProxyFactory;

        public:
            ~Proxy();

            Proxy* getToplevelProxy();
            inline World* getWorld() const;
            inline Shape* getShape() const;
            inline Proxy* getParent() const;

            inline int getMoveFlags() const;
            inline int getProxyType() const;

            void setProxyType(int type);


            void setIsInHierarchy(bool is);
            inline bool isInHierarchy() const;

            void addChild(Proxy* child);
            void removeChild(Proxy* child);

            inline const BvhNode* getBvHierarchyNode() const;
            inline const std::list<Proxy*>& getChildProxies() const;


            void restorePreviousState();
            void discardPreviousState();

            void setTransformation(const Matrix& state);
            inline const Matrix& getTransformation() const;

            Matrix getWorldTransformation() const;
            Matrix getPreviousWorldTransformation() const;


            Vector3 getPosition() const;
            void setPosition(const Vector3& position);
            void setPosition(real x, real y, real z);

            void translate(const Vector3& translateBy, bool respectOrientation = true);
            inline void translate(real x, real y, real z, bool respectOrientation = true);


            Matrix getRotation() const;
            void setRotation(const Matrix& rotation);
            void setRotation(real angle, real x, real y, real z);

            void rotate(const Matrix& rotateBy, bool respectOrientation = true);
            void rotate(real angle, real x, real y, real z, bool respectOrientation = true);


            void deform();
            void deform(const std::vector<Vector3>& vertexMoveArray);
            void addDeformVector(unsigned int vertexIndex, const Vector3& deformVector);
            void discardDeformVectors();

            void setVertexPosition(unsigned int vertexIndex, const Vector3& position);
            void setVertexPositions(const std::vector<Vector3>& vertexPositionArray);

            inline void setBvUpdatesEnabled(bool enabled);
            
            Vector3 getBoundingVolumeCenter() const;
            Vector3 getBoundingVolumeCenterPosition() const;

            void setBroadPhaseProxyData(BroadPhaseProxyData* data);
            inline BroadPhaseProxyData* getBroadPhaseProxyData() const;

            void setDetectorDeformProxyData(const std::vector<DetectorDeformProxyData*>& data);
            DetectorDeformProxyData* getDetectorDeformProxyData(unsigned int index) const;
            DetectorDeformProxyData* getDetectorDeformProxyData(const DetectorDeformAlgorithm* algorithm) const;


            bool mPrintDebugOutput;
            std::string mDebugOutputPrefix;
            inline void printDebugOutput(bool b, std::string prefix = "");

        private:
            void setProxyTypeNoParent(int type);
            void invalidateMeshWorldPositionsRecursive();

            void deformInternal(const std::vector<Vector3>& vertexMoveArray = std::vector<Vector3>());
            void setVertexPositionInternal(unsigned int vertexIndex,
                                           const Vector3& position,
                                           const std::vector<Vector3>& vertexPositionArray);
    };

    /*!
     * \return A 4x4 matrix describing the current state, i.e. the position,
     * rotation and scale of the proxy.
     */
    const Matrix& Proxy::getTransformation() const {
        return *mMatrix;
    }

    /*!
     * \return The \ref BvhNode of this proxy. This pointer is always non-NULL -
     * note however, that the Bounding Volume maintained by this node can be
     * NULL (if neither the node or its children contain a shape).
     */
    const BvhNode* Proxy::getBvHierarchyNode() const {
        return mBvHierarchyNode;
    }


    /*!
     * \return The parent of this proxy. See also \ref addChild.
     */
    inline Proxy* Proxy::getParent() const {
        return mParent;
    }

    /*!
     * \return All child proxies of this proxy, see \ref addChild
     */
    const std::list<Proxy*>& Proxy::getChildProxies() const {
        return mChildProxies;
    }

    /*!
     * \return The type (rigid, deformable, ...) of this proxy as given to the
     * constructor. See also \ref ProxyTypes.
     */
    int Proxy::getProxyType() const {
        return mType;
    }

    /*!
     * \return The \ref Shape object of this Proxy as given to the constructor.
     * NULL if this proxy has no shape.
     */
    Shape* Proxy::getShape() const {
        return mShape;
    }

    /*!
     * \overload
     */
    inline void Proxy::translate(real x, real y, real z, bool respectOrientation) {
        translate(Vector3(x, y, z), respectOrientation);
    }

    /*!
     * \return The world this proxy belongs to, i.e. which \ref World object
     * created this Proxy object.
     */
    inline World* Proxy::getWorld() const {
        return mWorld;
    }

    /*!
     * \return TRUE if the \ref Proxy has been added to the hierarchy already,
     * i.e. either it is a top level proxy and has been added to the world by
     * using \ref World::addProxy, or it is a child (grand-child, ...) of such a
     * Proxy.
     */
    inline bool Proxy::isInHierarchy() const {
        return mIsInHierarchy;
    }

    /*!
     * The moveflags describe how the proxy was moved (see \ref
     * setTransformation, \ref
     * translate, \ref rotate, \ref deform) since the last frame (more
     * precisely: since the last call to \ref World::startNextStep, i.e. since
     * the start of this frame == since the end of the last frame).
     *
     * \return Whether the transformation matrix was changed somehow or whether
     *         this proxy was deformed since the last frame. See \ref MoveFlags.
     *         This value is reset to the \ref MOVEFLAG_UNMOVED state whenever a
     *         new frame is started (see \ref World::startNextStep), or when a
     *         frame is "undone" (see \ref World::undoStep).
     */
    int Proxy::getMoveFlags() const {
        return mMoveFlags;
    }

    inline BroadPhaseProxyData* Proxy::getBroadPhaseProxyData() const {
        return mBroadPhaseProxyData;
    }
    
    inline void Proxy::printDebugOutput(bool b, std::string prefix) {
        mPrintDebugOutput = b;
        mDebugOutputPrefix = prefix;
    }
    
    /*!
     * \brief Enables or disables automatic rigid bounding volume hierarchy updates
     * \param boolean enabled
     * 
     * Note that a rigid proxy hierarchy update will be triggered by this method,
     * when automatic updates were disabled previously and are enabled by a call
     * to this method.
     */
    
    inline void Proxy::setBvUpdatesEnabled(bool enabled) {
    	if(enabled) {
    		if(mBvUpdatesEnabled == false) {
    			//Automatic updates should be enabled and were disabled before, thus
    			//we have to update the rigid hierarchy, because deformations could
    			//have happenend prior to this call
    			mBvHierarchyNode->deform();
    			mBvUpdatesEnabled = true;
    		} else {
    			//Automatic update was already enabled before - so there is nothing to do
    			return;
    		}
    	} else {
    		//Disable automatic updates
    		mBvUpdatesEnabled = false;
    	}
    }
}

#endif // DCOLLIDE_PROXY_H
/*
 * vim: et sw=4 ts=4
 */
