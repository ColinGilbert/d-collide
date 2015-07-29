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

#ifndef DCOLLIDE_BVHNODE_H
#define DCOLLIDE_BVHNODE_H

#include "thread/thread.h" // TODO: we need mutex only. split it out of that header?
#include "math/vector.h"
#include "math/matrix.h"

#include <list>

namespace dcollide {
    class Proxy;
    class BoundingVolume;
    class MeshPart;
    class Mesh;
    class Shape;
    class World;

    // TODO: make this class abstract, so that people are _forced_ to use
    // derived classes (usually BvhNodeDefault)

    /*!
     * \brief node in a BoundingVolume-hierarchy-tree
     * Proxies and their shapes are organised in a tree-structure to speed
     * up collision detection.
     *
     * The bounding volume hierarchy usually matches the \ref Proxy hierarchy
     * given by the user, that is:
     * \li A proxy has exactly one corresponding BvhNode
     * \li For each (proxy-)child of a proxy there is exactly one BvhNode
     *     child of the corresponding BvhNode
     * \li There is another child of the BvhNode corresponding to a proxy
     *     that represents the \ref Shape object of the \ref Proxy (if the proxy
     *     has such an object).
     *
     * The bounding volume of every BvhNode is large enough to include all
     * of its children (i.e. it actually is a bounding volume). Thus, if a
     * BvhNode corresponds to a \ref Proxy object, then the BvhNode
     * is large enough to include all proxy children \em and (!) its \ref Shape
     * object.
     *
     * To make this (again) abundantly clear: a \ref Shape object of a \Proxy is
     * treated by BvhNode node exactly as if it was a normal child of that
     * proxy.
     *
     * Note that while every \ref Proxy object has one corresponding
     * BvhNode, this is not always true in the opposite direction: the
     * library may decide to subdivide the hierarchy further to make collision
     * detection more efficient.
     *
     * \author Gregor Jochmann <gregor.jochmann@uni-dortmund.de>
     */
    class BvhNode {
        protected:
            /*! \brief the bounding volume big enough to contain all children
             * The bounding volume of a node is built from all of its 
             * child elements
             */
            BoundingVolume* mBoundingVolume;

            /*! \brief list with pointers to all children. can be empty
             * Each node of a tree stores references to its children.
             */
            std::list<BvhNode*> mChildren;

        private:
            /*! \brief pointer to the parent of this node in the tree
             * This is a back-reference to the parent node.
             * Can be NULL if the node is the root of the hierarchy
             */
            BvhNode* mParent;

        private:
            /*! \brief pointer to the corresponding proxy (if existent)
             * Nodes might have a directly corresponding proxy.
             */
            Proxy* mProxy;
            
            /*! \brief pointer to the corresponding shape (if existent)
             * If this node represents the bounding volume of a shape, then this
             * is the pointer to that shape, otherwise this pointer is NULL.
             */
            Shape* mShape;

            /*!
             * Used to NOT call \ref adjustToShape for \ref mShape if the shape
             * is already represented by child nodes (e.g. because the shape has
             * been split up into \ref MeshPart objects)
             */
            bool mIgnoreShape;

            /*! \brief translation accumulator since last BV calculation
             * each node stores its own translation only.
             * To get the total translation, we need to sum up from the top
             */
            Vector3 mTranslationAccu;
            bool mIsTranslationAccuEmpty;

            Mutex mMutex;

            World* mWorld;

            bool mInitializeCalled;

        public:
            /*! \brief User pointer.
             *
             * This pointer is not used by the d-collide library, it is meant to
             * take user-defined data. It is initialized to 0 on construction,
             * after that this library will never change its value.
             */
            void* mUser;



        public:
            // Constructor(s) and Destructor
            BvhNode(World* world);
            BvhNode(World* world, Proxy* proxy);
            BvhNode(World* world, Shape* shape, bool ignoreShape = false);

            virtual ~BvhNode();

            void initializeNode();

            virtual BoundingVolume* createBoundingVolume();
            virtual BoundingVolume* createBoundingVolume(const BoundingVolume* copy);

            virtual void addChild(BvhNode* newChild);
            virtual void removeChild(BvhNode* child);

            virtual void recalculateBoundingVolumes();
            virtual void translate(const Vector3& translateBy);
            virtual void rotate(const Matrix& rotateBy = 0);
            virtual void deform();
            virtual void changeMatrix();


            inline void setParent(BvhNode* parent);
            inline BvhNode* getParent() const;

            void setIgnoreShape(bool ignore);
            bool getIgnoreShape() const;

            inline const std::list<BvhNode*>& getChildren() const;
            inline const BoundingVolume* getBoundingVolume() const;
            inline bool isLeaf() const;
            inline World* getWorld() const;
            inline Proxy* getProxy() const;
            inline Shape* getShape() const;

            void printBvHierarchy() const;

        protected:
            void clearBoundingVolume();
            void mergeBoundingVolumeWith(const BoundingVolume* bv);
            void applyTranslationAccu();
            void applyTranslationAccuFromParents();
            void updateParents();

            inline virtual void calculateBoundingVolume();

            void recalculateBottomUp();

        private:
            void storeTranslation(const Vector3& translateBy);
    };



    /*!
     * \return The parent of this node (see \ref addChild) or NULL if this is a
     * toplevel node.
     */
    BvhNode* BvhNode::getParent() const {
        return mParent;
    }

    /*!
     * Set the parent node of this node. See also \ref getParent.
     *
     * This method is called automatically by \ref addChild. You do not normally
     * need to call it.
     */
    void BvhNode::setParent(BvhNode* parent) {
        mParent=parent;
    }

    /*!
     * \brief gets the BoundingVolume (PLEASE READ DETAILED DOCUMENTATION)
     *
     * WARNING: If you call this for a node BEFORE calling it for its parent,
     * the result will be inaccurate! We expect you to traverse trough the 
     * boundingvolume-hierarchy starting at the root/top-level node
     *
     * WARNING#2: BvhNodes might have a NULL-BoundingVolume if there is no shape
     *            or child with a shape that defines it. Be sure to make NULL-
     *            checks when using this!
     *
     * \return a pointer to the BoundingVolume of this node (can also be NULL!)
     */
    const BoundingVolume* BvhNode::getBoundingVolume() const{
        // AB: note: all methods called here MUST be thread safe!

        const_cast<BvhNode*>(this)->applyTranslationAccu(); // is thread safe
        const_cast<BvhNode*>(this)->calculateBoundingVolume();

        return mBoundingVolume;
    }

    /*!
     * \return All child nodes of this node, see \ref addChild
     */
    const std::list<BvhNode*>& BvhNode::getChildren() const {
        return mChildren;
    }

    /*!
     * \return TRUE if this node is a leaf, i.e. has no children, otherwise
     * FALSE
     */
    bool BvhNode::isLeaf() const {
        return mChildren.empty();
    }

    /*!
     * \return The \ref World this node belongs to, as given to the constructor.
     */
    World* BvhNode::getWorld() const {
        return mWorld;
    }

    /*!
     * \return The proxy assigned to this node (as specified in the
     * constructor), if this node belongs (directly!) to a \ref Proxy. NULL if
     * this node does not belong directly to a proxy (a \ref getParent may
     * however have a non-NULL proxy).
     */
    Proxy* BvhNode::getProxy() const {
        return mProxy;
    }

    /*!
     * \return The \ref Shape assigned to this node, as specified in the
     * constructor, or NULL if this node does not belong to a \ref Shape.
     */
    Shape* BvhNode::getShape() const {
        return mShape;
    }

    /*!
     * WARNING: this method MUST be thread-safe!
     *
     * Called to ensure the \ref BoundingVolume object stored here (see \ref
     * getBoundingVolume) is actually valid. The translation accu (see \ref
     * applyTranslationAccu) has already been applied at this point.
     *
     * This method is called by \ref getBoundingVolume and therefore a valid
     * (and "current") \ref BoundingVolume is absolutely required.
     *
     * The default implementation does nothing - instead \ref
     * recalculateBoundingVolumes is used to make sure that this \ref
     * BoundingVolume is never out of date.
     *
     * Derived classes may want to reimplement this e.g. to implement a top-down
     * update strategy (as opposed to the bottom-up strategy that \ref
     * recalculateBoundingVolumes uses).
     *
     *
     * WARNING: if you reimplement this mehtod you \em MUST ensure that it is
     * thread-safe!
     */
    inline void BvhNode::calculateBoundingVolume() {
    }

}

#endif // DCOLLIDE_HNODE_H
/*
 * vim: et sw=4 ts=4
 */
