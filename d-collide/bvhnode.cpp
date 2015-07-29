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

#include "bvhnode.h"
#include "boundingvolumes/boundingvolume.h"
#include "boundingvolumes/aabb.h"
#include "boundingvolumes/kdop.h"
#include "boundingvolumes/obb.h"
#include "debugstream.h"
#include "broadphase/broadphase.h"
#include "shapes/mesh.h"
#include "shapes/mesh/triangle.h"
#include "shapes/mesh/vertex.h"
#include "dcollide-defines.h"
#include "world.h"
#include "proxy.h"
#include "meshsplitter.h"

#include <iostream>
#include <stack>

namespace dcollide {
    // Constructor(s) and Destructor
    /*!
     * Create a new BvhNode object for use with objects created by \p world.
     *
     * Note that you must call \ref initializeNode right after constructing an
     * object of this class, before you can use it.
     *
     * \param world Pointer to the \ref World object that this node is being
     * used in.
     */
    BvhNode::BvhNode(World* world) {
        if (!world) {
            throw NullPointerException("Parameter world");
        }
        mWorld = world;
        mParent = 0;
        mBoundingVolume = 0;
        mProxy = 0;
        mShape = 0;
        mUser = 0;
        mIsTranslationAccuEmpty = true;
        mIgnoreShape = false;
        mInitializeCalled = false;
    }

    /*!
     * THIS IS NOT A USER FUNCTION
     *
     * \internal
     *
     * It will be called automatically when constructing a proxy
     *
     * Note that you must call \ref initializeNode right after constructing an
     * object of this class, before you can use it.
     *
     * \param world Pointer to the \ref World object that this node is being
     * used in.
     */
    BvhNode::BvhNode(World* world, Proxy* proxy) {
        if (!world) {
            throw NullPointerException("Parameter world");
        }
        mWorld = world;
        mParent = 0;
        mBoundingVolume = 0;
        mProxy = proxy;
        mShape = 0;
        mUser = 0;
        mIsTranslationAccuEmpty = true;
        mIgnoreShape = false;
        mInitializeCalled = false;
    }

    /*!
     * THIS IS NOT A USER FUNCTION
     *
     * \internal
     *
     * It will be called automatically when constructing a proxy
     *
     * Note that you must call \ref initializeNode right after constructing an
     * object of this class, before you can use it.
     *
     * \param world Pointer to the \ref World object that this node is being
     * used in.
     * \param ignoreShape See \ref setIgnoreShape. FALSE by default.
     */
    BvhNode::BvhNode(World* world, Shape* shape, bool ignoreShape) {
        if (!world) {
            throw NullPointerException("Parameter world");
        }
        mWorld = world;
        mParent = 0;
        mBoundingVolume = 0;
        mProxy = 0;
        mShape = shape;
        mUser = 0;
        mIsTranslationAccuEmpty = true;
        mIgnoreShape = ignoreShape;
        mInitializeCalled = false;
    }

    /*!
     * Removes itself out of the parents list,
     * then deletes all children and itself
     */
    BvhNode::~BvhNode() {
        if (mParent) {
            mParent->removeChild(this);
        }
        while (!mChildren.empty()) {
            BvhNode* child = mChildren.front();
            mChildren.pop_front();
            delete child;
        }
        delete mBoundingVolume;
        if (mShape!=0 && mShape->getShapeType() == Shape::SHAPE_TYPE_MESH_PART) {
            delete mShape;
        }
    }

    /*!
     * Must be called right after the constructor of this object.
     *
     * This method initializes the node and may create a \ref BoundingVolume
     * object, e.g. if it has a \ref Shape (see \ref getShape).
     *
     * This method makes use of virtual method and therefore must be called \em
     * after the constructor, not \em in the constructor, unless you know what
     * you are doing.
     */
    void BvhNode::initializeNode() {
        if (mInitializeCalled) {
            return;
        }
        mInitializeCalled = true;

        if (mShape) {
            if (!mIgnoreShape) {
                mBoundingVolume = createBoundingVolume();
                mBoundingVolume->adjustToShape(mShape);
            }
        }
    }

    /*!
     * Note: if you overwrite this method, you also have to \em both methods of
     * this name (i.e. also the one that creates a copy of a \ref BoundingVolume
     * object).
     *
     * \return A new \ref BoundingVolume object for use in this node. This
     * method honors \ref WorldParameters::getRigidBoundingVolumeType by
     * default, derived classes may override this.
     */
    BoundingVolume* BvhNode::createBoundingVolume() {
        if (!mInitializeCalled) {
            throw Exception("initializeNode() has not been called yet");
        }
        return BoundingVolume::createBoundingVolume(getWorld(), this);
    }

    /*!
     * \overload
     *
     * This version is like the above version, but the returned node will be a
     * copy of \p copy (i.e. the copy-constructor is used).
     */
    BoundingVolume* BvhNode::createBoundingVolume(const BoundingVolume* copy) {
        if (!mInitializeCalled) {
            throw Exception("initializeNode() has not been called yet");
        }
        return BoundingVolume::createBoundingVolume(getWorld(), copy, this);
    }

    /*!
     * Causes this BvhNode to ignore the \ref getShape object (if any) when
     * recalculating the \ref BoundingVolume of this node.
     *
     * This method is an internal method - you should not usually call this.
     *
     * This method is used by the \ref MeshSplitter: a \ref Mesh is split up into
     * multiple \ref MeshPart objects which form a partition of the \ref Mesh,
     * i.e. together they contain all vertices and triangles of the \ref Mesh.
     * Therefore it is sufficient to calculate the BV of the mesh by calculating
     * the union of the \ref MeshPart objects, so the \ref Mesh itself can be
     * ignored. Since the \ref MeshSplitter takes care of this, the user should
     * not usually call this method, however if you develop a new \ref
     * BoundingVolume based algorithm which works in a similar way, you may need
     * this method, too.
     */
    void BvhNode::setIgnoreShape(bool ignore) {
        mIgnoreShape = ignore;
        if (!mIgnoreShape && mShape && !mBoundingVolume) {
            mBoundingVolume = createBoundingVolume();
            mBoundingVolume->adjustToShape(mShape);
        }
    }

    /*!
     * \return Whether this node ignores the shape when recalculating the
     * bounding volume. See also \ref setIgnoreShape
     */
    bool BvhNode::getIgnoreShape() const {
        return mIgnoreShape;
    }

    /*!
     * Merge the internal \ref BoundingVolume with \p bv, so that \ref
     * getBoundingVolume is enlarged to include \p bv.
     *
     * If this node does not currently hold a \ref BoundingVolume, then after
     * this call it will hold a \ref BoundingVolume that matches \p bv.
     */
    void BvhNode::mergeBoundingVolumeWith(const BoundingVolume* bv) {
        if (mBoundingVolume == 0) {
            mBoundingVolume = createBoundingVolume(bv);
            //not neccesary, is already done by createBV
            //mBoundingVolume->setHierarchyNode(this);
        } else {
            mBoundingVolume->mergeWith(bv);
        }
    }

    /*!
     * Clear the internal \ref BoundingVolume, i.e. delete it. This will make
     * \ref getBoundingVolume return NULL until a new BV is created (usually by
     * a call to \ref mergeBoundingVolumeWith following a call to this method).
     */
    void BvhNode::clearBoundingVolume() {
        delete mBoundingVolume;
        mBoundingVolume = 0;
    }


    /*! \brief INTERNAL USE ONLY: adds a child to this bvhnode
     *
     * WARNING: THIS FUNCTION SHOULD NOT BE CALLED BY THE USER
     *
     * Adds \p newChild as a child of this node, grows the boundingvolumes
     *
     * OWNERSHIP NOTICE: the parent takes ownership of the child
     */
    void BvhNode::addChild(BvhNode* newChild) {
        if (!newChild) {
            throw NullPointerException("parameter \"BvhNode* newChild\"");
        }
        applyTranslationAccuFromParents();
        mChildren.push_back(newChild);
        newChild->setParent(this);

        //grow this nodes bounding volume to contain the new child.
        //promote this growing to all parents
        BvhNode* curNode = this;
        while (curNode->getParent()) {
            curNode->mergeBoundingVolumeWith(newChild->getBoundingVolume());
            curNode = curNode->getParent();
        }
        curNode->mergeBoundingVolumeWith(newChild->getBoundingVolume());

        // Now we can update the Broadphase (if Proxies already have a
        // world):
        if ((curNode->getProxy()) && (mWorld->prepareSimulationWasCalled())) {
            if (curNode->getProxy()->isInHierarchy() && curNode->getProxy()->getWorld()) {
                if (curNode->getBoundingVolume()) {
                    BroadPhase* b = curNode->getProxy()->getWorld()->getBroadPhase();
                    b->notifyProxyChanged(curNode->getProxy());
                }
            }
        }
    }

    /*!
     * Cuts \p child from the list of children
     * TODO check if we can reduce the hierarchy level, depending on
     * new childCount > MAX_CHILDREN_COUNT
     */
    void BvhNode::removeChild(BvhNode* child) {
        mChildren.remove(child);
        child->setParent(NULL);

    }

    /*! \brief updates all parents (up to toplevel) to contain this BV
     */
    void BvhNode::updateParents() {
        BvhNode* currentParentNode = mParent;
        while (currentParentNode != 0) {
            currentParentNode->clearBoundingVolume();
            //remerge this parent out of all of its children
            for (std::list<BvhNode*>::const_iterator iter =
                                currentParentNode->getChildren().begin();
                                iter != currentParentNode->getChildren().end();
                                ++iter) {
                BvhNode* child=*iter;
                currentParentNode->mergeBoundingVolumeWith(
                                                    child->getBoundingVolume());
            }
            currentParentNode=currentParentNode->getParent();
        }
    }

    /*!
     * This triggers full recalculation of the \ref BoundingVolume hierarchy,
     * starting at this node. Changes to this node are also propagated back to
     * its parents (however siblings of this node are not recalculated).
     *
     * The recalculation is done in a bottom-up way, i.e. leaf-nodes are updated
     * first and the \ref BoundingVolume objects of their parents are calculated
     * by merging the \ref BoundingVolume objects of their children.
     *
     * A toplevel approach, where a node would calculate its \ref BoundingVolume
     * directly from all \ref Shapes in the tree below it is \em not used.
     *
     * If you reimplement this method in derived classes you need at least to:
     * \li Flush the translation accu of all parent/grand-parents in the
     *     hierarchy (see \ref applyTranslationAccuFromParents) and this node
     *     (see \ref applyTranslationAccu).
     * \li Actually recalculate the \ref BoundingVolume in whichever way is
     *     appropriate for you.
     * \li Notify all parents (grand-parents, ...) about the updated \ref
     *     BoundingVolume (see \ref updateParents).
     */
    void BvhNode::recalculateBoundingVolumes() {
        applyTranslationAccuFromParents();
        recalculateBottomUp();
        updateParents();
    }

    /*!
     * Recalculate this node - this is used by \ref recalculateBoundingVolumes.
     *
     * The recalculation is done by:
     * \li Flushing the translation accu (see \ref applyTranslationAccu).
     * \li Clearing the old \ref BoundingVolume
     * \li Recalculate the \ref BoundingVolume of all children (i.e. recursive
     *     call)
     * \li Merge the \ref BoundingVolume objects of all children to get the new
     *     \ref BoundingVolume of this node.
     *
     * i.e. we use a bottom-up appraoch.
     *
     * Note that the \ref getParent is \em not notified about the updated
     * bounding volume. You need to manually call this after calling this
     * method, if you use this method directly. See \ref updateParents.
     *
     * Normally this should be called from \ref recalculateBoundingVolumes (and
     * recursively from this method) only.
     */
    void BvhNode::recalculateBottomUp() {
        applyTranslationAccu();

        //reset the BV
        clearBoundingVolume();

        //now, we can do the recalculation:
        //the bvs of this node and its children needs to be recalculated
        //recursive build up this nodes bv from all its children again,
        //then clear the bvs of all parents in the hierarchy and remerge them

        for (std::list<BvhNode*>::iterator iter = mChildren.begin();
                                            iter != mChildren.end();
                                            ++iter) {
            BvhNode* child=*iter;
            child->recalculateBottomUp();

            // TODO: some BV types may do merging more efficiently and/or more
            // precisely when they merge with all children at once.
            // i.e. provide mergeBoundingVolumeWith(mChildren) which by default
            // calls mergeBoundingVolumeWith() once for each child.
            //
            // -> the DeformableBvhNode may do someting different!
            mergeBoundingVolumeWith(child->getBoundingVolume());
        }

        if (mShape && !mIgnoreShape) {
            BoundingVolume* bv = createBoundingVolume();
            bv->adjustToShape(mShape);
            mergeBoundingVolumeWith(bv);
            delete bv;
        }
    }

    /*!
     * Apply the translation accu from all possible parents.
     *
     * The translation accu normally only stores translations "very high" in the
     * hierarchy, the children (grand-children, ...) do not know whether the
     * accu holds something or not. This method makes sure that all translation
     * accus above this node are flushed.
     *
     * You should call this from \ref recalculateBoundingVolumes before doing
     * anything else!
     */
    void BvhNode::applyTranslationAccuFromParents() {
        
        if (!getWorld()->getWorldParameters().getAllowLocalCoordinates()) {
            return;
        }
        
        //apply the accus, but the top-level one has to be applied first.
        //so, find the top level one and store all visited nodes in a stack
        //on the way back down, apply the accus and "push down" translations
        std::stack<BvhNode*> parentStack;
        BvhNode* curentParent = mParent;
        while (curentParent != 0) {
            parentStack.push(curentParent);
            curentParent = curentParent->getParent();
        }
        //now do the backtracking with the build stack
        while (!parentStack.empty()) {
            BvhNode* stackTop = parentStack.top();
            stackTop->applyTranslationAccu();
            parentStack.pop();
        }
        applyTranslationAccu();
    }


    /*!
     * \brief updates all childrens BV with their translation accus
     *
     * This method is thread safe.
     */
    void BvhNode::applyTranslationAccu() {
        if (!mIsTranslationAccuEmpty) {

            MutexLocker lock(&mMutex);
            //translate by current accu
            if (mBoundingVolume != 0) {
                mBoundingVolume->translate(mTranslationAccu);
            }
            //pass translation to all children
            for (std::list<BvhNode*>::iterator iter = mChildren.begin();
                                                iter != mChildren.end();
                                                ++iter) {
                BvhNode* child=*iter;
                child->storeTranslation(mTranslationAccu);
            }

            mTranslationAccu.reset();
            mIsTranslationAccuEmpty = true;
        }
    }


    /*! \brief adds to the translation accus of THIS node
     */
    void BvhNode::storeTranslation(const Vector3& translateBy) {
        mTranslationAccu+=translateBy;
        mIsTranslationAccuEmpty = false;
    }


    /*!
     * Translate this node by \p translateBy.
     *
     * Translations can be handled very efficiently by \ref BoundingVolume
     * objects, since they can be translated, too (no recalculation necessary).
     *
     * In addition, this method only "directly" translates the \ref
     * BoundingVolume of this node, and stores the translations of its children
     * in a "translation accu". These translations are then applied later when
     * needed only.
     *
     * If derived classes reimplement this they need at least:
     * \li Translate the bounding volume in this BvhNode (see \ref
     *     getBoundingVolume)
     * \li Translate the child-nodes of this node (see \ref getChildren)
     * \li Notify the parent of this note that this note changed (see \ref
     *     updateParents)
     *
     *
     * \param translateBy The vector by which the node should be translated.
     *        This vector is in \em global coordinates, not in the coordinate
     *        system of the \ref Proxy or parent node that this node belongs to.
     */
    void BvhNode::translate(const Vector3& translateBy) {
        
        if (getWorld()->getWorldParameters().getAllowLocalCoordinates()) {

            //children store accu
            for (std::list<BvhNode*>::iterator iter = mChildren.begin();
                                                iter != mChildren.end();
                                                ++iter) {
                BvhNode* child=*iter;
                child->storeTranslation(translateBy);
            }
            //translate own BV
            if (mBoundingVolume != 0) {
                mBoundingVolume->translate(translateBy);
            }

            updateParents();

        } else {
            recalculateBoundingVolumes();
        }
    }

    /*!
     * Called when the parent proxy has been rotated.
     *
     * The default implementation simply calls \ref recalculateBoundingVolumes.
     * If OOBBs are used, we can just rotate them, instead of recalculating!
     *
     * TODO: figure out what paremters to use.
     *       probably we need at least
     *       a) the rotation that was applied to the proxy
     *       b) the position of the proxy (i.e. the reference point of the
     *          rotation)
     *       I have NOT yet added these, because rotations are highly
     *       non-trivial for our main BVs (AABBs, k-DOPs), as we can implement
     *       them only by either recalculating the BVs or by using
     *       approximations
     */
    void BvhNode::rotate(const Matrix& rotateBy) {
        if (mBoundingVolume && (mBoundingVolume->getVolumeType() == BV_TYPE_OBB)) {
            applyTranslationAccuFromParents();
            static_cast<Obb*>(mBoundingVolume)->rotate(rotateBy);
            for (std::list<BvhNode*>::const_iterator it = mChildren.begin(); it != mChildren.end(); ++it) {
                (*it)->rotate(rotateBy);
            }
            updateParents();
        } else {
            recalculateBoundingVolumes();
        }
    }

    /*!
     * Called when the \ref Proxy::setTransformation was called for the parent proxy.
     *
     * The default implementation simply calls \ref recalculateBoundingVolumes.
     */
    // AB: I don't think we can do much smart things here, except for
    //     recalculating the BVs.
    //     if we ever want to make actual use of this method, we may need the
    //     old matrix and the new matrix as parameters to this method.
    void BvhNode::changeMatrix() {
        recalculateBoundingVolumes();
    }

    /*!
     * Called when the parent proxy has been deformed.
     *
     * The default implementation simply calls \ref recalculateBoundingVolumes.
     */
    // AB: TODO: maybe provide the deformations as parameter?
    void BvhNode::deform() {
        recalculateBoundingVolumes();
    }

    /*!
     * Print the bounding volume hierarchy as debug output.
     *
     * This method assumes that this node is the root node, i.e. that is has no
     * parent.
     */
    void BvhNode::printBvHierarchy() const {
        if (getParent()) {
            warning() << dc_funcinfo << "not a root node";
        }
        std::list< std::pair<const BvhNode*, std::string> > nodes;
        nodes.push_back(std::make_pair(this, ""));
        while (!nodes.empty()) {
            const BvhNode* node = nodes.front().first;
            std::string indentation = nodes.front().second;
            nodes.pop_front();
            for (std::list<BvhNode*>::const_reverse_iterator it = node->getChildren().rbegin(); it != node->getChildren().rend(); ++it) {
                nodes.push_front(std::make_pair(*it, indentation + std::string("  ")));
            }

            std::string shape = "";
            if (node->getShape()) {
                std::stringstream ss;
                ss << "have shape: ";
                ss << node->getShape();
                ss << " - ";
                Shape::ShapeType type = node->getShape()->getShapeType();
                ss << type;
                if (type == Shape::SHAPE_TYPE_MESH) {
                    const Mesh* m = (const Mesh*)node->getShape();
                    ss << " triangles: " << m->getTriangles().size();
                }
                if (type == Shape::SHAPE_TYPE_MESH_PART) {
                    const MeshPart* m = (const MeshPart*)node->getShape();
                    ss << " triangles: " << m->getTriangles().size();
                }
                shape = ss.str();
            }
            debug() << indentation << node << " children: " << node->getChildren().size() << " " << shape << " BV: " << node->getBoundingVolume();
        }
    }
}

/*
 * vim: et sw=4 ts=4
 */
