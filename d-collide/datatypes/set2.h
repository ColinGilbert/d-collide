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

#ifndef DCOLLIDE_SET2_H
#define DCOLLIDE_SET2_H

#define SET_DEBUG 1
//#undef SET_DEBUG

#include "debug.h"
#include "datatypes/list.h"
#include "collisionpair.h"

// Datei zum einschalten des set2.h: broadphasehierarchicalgridjob2.{h,cpp}

namespace dcollide {

    /*!
     * \brief The leafs of a \ref BinaryTree
     * Here we save the data
     */
    template <class T> class BinaryTreeLeaf {

        public:
            BinaryTreeLeaf(BinaryTreeLeaf* parent);
            BinaryTreeLeaf(BinaryTreeLeaf* parent,const T& data);
            ~BinaryTreeLeaf();

            BinaryTreeLeaf* mParent;
            BinaryTreeLeaf* mLeftChild;
            BinaryTreeLeaf* mRightChild;

            inline const T& getData() const;
            inline void setData(const T& data);
            inline bool insertData(const T& data);
            inline void clear();

        private:
            T mData;
    };


    /*!
     * \brief This is the data-container for the \ref Set
     * The tree consists of \ref BinaryTreeLeafs, in these the data will be
     * saved.
     * This class contains the whole functionality of a BinaryTree, the leafs
     * are data containers only!
     */
    template <class T> class BinaryTree {

        public:
            BinaryTree();
            ~BinaryTree();

            void clear();

            inline bool insertData(const T& data);
            inline bool contains(const T& data);
            inline bool remove(const T& data);


        private:

            // Copy c'tor
            BinaryTree(const BinaryTree& binaryTree);

            // Root of the Tree:
            BinaryTreeLeaf<T>* mRoot;
    };

    /*!
     * \brief This datatype saves unique values
     * A \ref BinaryTree is used to save the values, each value must be unique
     */
    template <class T> class Set2 {

        public:
            Set2();
            ~Set2();
            // unique insert
            inline bool insert(const T& data);

            inline void clear();

            inline bool empty();

            inline unsigned int size() const;

            inline bool contains(T& data) const;

            inline const T& getElement() ;

            inline bool remove(T& data);

            inline T& pop_back();

        private:

            // Copy c'tor
            Set2(const Set2& set);

            /*!
             * \brief Actual number of members in the Set
             */
            unsigned int mCount;

            /*!
             * \brief will be returned if mCount = 0;;
             */
            T mDefaultElement;

            /*!
             * \brief The Binary Tree
             */
            BinaryTree<T>* mTree;

    };
    /*!
     * \brief c'tor
     * \param parent The parent of this leaf
     */
    template <class T> BinaryTreeLeaf<T>::BinaryTreeLeaf(BinaryTreeLeaf* parent) {
        mParent = parent;
        mLeftChild = 0;
        mRightChild = 0;
    }
    /*!
     * \brief c'tor
     * \param parent The parent of this leaf
     * \param data The data to save
     */
    template <class T> BinaryTreeLeaf<T>::BinaryTreeLeaf(
            BinaryTreeLeaf* parent, const T& data) {
        mParent = parent;
        mLeftChild = 0;
        mRightChild = 0;
        mData = data;
    }

    /*!
     * \brief d'tor
     */
    template <class T> BinaryTreeLeaf<T>::~BinaryTreeLeaf() {
        // Don't delete parent, in the end this must be done by class BinaryTree
        //delete mLeftChild;
        //delete mRightChild;
    }

    /*!
     * \returns the data from the leaf
     */
    template <class T> const T& BinaryTreeLeaf<T>::getData() const {
        return mData;
    }

    /*!
     * \brief inserts data in the Tree
     * \returns true if success (means no duplicate) or false if already in
     */
    template <class T> bool BinaryTreeLeaf<T>::insertData(const T&
            data) {

        bool ret = false;

        /* Go through all the leafs until we reach the end of the tree:
         * Go left if new data < leaf-data
         * Go right if new data > leaf-data
         * exit if new data == leaf-data
         */
#ifdef SET_DEBUG
         std::cout << "At leaf " << this << ": ";
#endif
         // exit if new data == leaf-data
         if (data == mData) {
#ifdef SET_DEBUG
             std::cout << data << "==" << mData << " FALSE! "<< std::endl;
#endif
             return false;
         }
         // Go left if new data < leaf-data:
         else if (data < mData) {
#ifdef SET_DEBUG
            std::cout << " < -> " << mLeftChild << std::endl;
#endif
            if (mLeftChild) {
                ret = mLeftChild->insertData(data);
            } else {
                mLeftChild = new BinaryTreeLeaf<T>(this,data);
#ifdef SET_DEBUG
                std::cout <<  " Creating LC! " << mLeftChild << std::endl;
                std::cout << "\t" << data << " INSERT in "<< mLeftChild  << std::endl;
#endif
                return true;
            }
        }
        // Go right if new data > leaf-data
        // If not == and not < => >!
        else {
#ifdef SET_DEBUG
            std::cout << " > -> " << mRightChild << std::endl;
#endif
            if (mRightChild) {
                ret = mRightChild->insertData(data);
            } else {
                mRightChild = new BinaryTreeLeaf<T>(this,data);
#ifdef SET_DEBUG
                std::cout <<  " Creating RC! " << mRightChild << std::endl;
                std::cout << "\t" << data << " INSERT in "<< mRightChild << std::endl;
#endif
                return true;
            }
        }

        return ret;
    }

    /*!
     * \brief saves data in \ref mData in the leaf
     * Beware: Here we don't check if data does already exist! This is done by
     * \ref BinaryTree
     * \param data the data to save
     */
    template <class T> void BinaryTreeLeaf<T>::setData(const T& data) {
        mData = data;
    }


    /*!
     * \brief clears the leaf and all the children above
     */
    template <class T> void BinaryTreeLeaf<T>::clear() {
#ifdef SET_DEBUG
        std::cout << this << " Trying ... " << std::endl;
#endif
        if (mLeftChild) {
#ifdef SET_DEBUG
            std::cout << this << " BTLLeft: clear() "<< mLeftChild << std::endl;
#endif
            mLeftChild->clear();
        }
#ifdef SET_DEBUG
        std::cout << this << " BTLLeft: delete" << std::endl;
#endif
        delete mLeftChild;
        if (mRightChild) {
#ifdef SET_DEBUG
            std::cout << this << " BTLRight: clear() " << mRightChild <<std::endl;
#endif
            mRightChild->clear();
        }
#ifdef SET_DEBUG
        std::cout << this <<  " BTLRight: delete" << std::endl;
#endif
        delete mRightChild;
    }

    /*!
     * \brief c'tor
     */
    template <class T> BinaryTree<T>::BinaryTree() {
        // Don't initalize mRoot, this mus de done when the first data should be
        // inserted;
    }

    /*!
     * \brief d'tor
     */
    template <class T> BinaryTree<T>::~BinaryTree() {
        delete mRoot;
    }

    /*!
     * \brief inserts data in the Tree
     * \returns true if success (means no duplicate) or false if already in
     */
    template <class T> bool BinaryTree<T>::insertData(const T& data) {

        if (mRoot) {
#ifdef SET_DEBUG
            std::cout <<  " Root: " << mRoot;
#endif
            return mRoot->insertData(data);
        } else {
            mRoot = new BinaryTreeLeaf<T>(0,data);
#ifdef SET_DEBUG
            std::cout <<  " Creating Root! " << mRoot << std::endl;
#endif
            return true;
        }
    }


    /*!
     * \brief removes data from Tree
     * \param the data to remove
     * \returns true if success
     */
    template <class T> bool BinaryTree<T>::remove(const T& data) {
        bool ret = false;
        //TODO
        return ret;
    }

    /*!
     * \brief checks if data is already in the Tree
     * \returns true if data is in the Tree
     */
    template <class T> bool BinaryTree<T>::contains(const T& data) {
        bool ret = false;
        //TODO
        return ret;
    }


    /*!
     * \brief empties the tree
     * Runs in O(log N)
     */
    template <class T> void BinaryTree<T>::clear() {
        if (mRoot) {
#ifdef SET_DEBUG
            std::cout << this << " BT: clear()" << std::endl;
#endif
            mRoot->clear();
        }
#ifdef SET_DEBUG
        std::cout << this << " BT: delete mRoot " << mRoot << std::endl;
#endif
        delete mRoot;
        mRoot = 0;
    }

    /*!
     * \brief c'tor of Set
     *
     */
    template <class T> Set2<T>::Set2() {
        mCount = 0;
        mTree = new BinaryTree<T>();
    }

    /*!
     * \brief copy c'tor of Set2
     */
    template <class T> Set2<T>::Set2(const Set2& set) {

    }

    /*!
     * \brief d'tor of the Set
     */
    template <class T> Set2<T>::~Set2() {
        mTree->clear();
        delete mTree;
    }

    /*!
     * \brief inserts data into the set
     * \param the data to insert
     * \returns true if success, false if already in the set
     */
    template <class T> bool Set2<T>::insert(const T& data) {
        bool ret = false;
        ret = mTree->insertData(data);
        return ret;
    }

    /*!
     * \brief empties the set
     */
    template <class T> void Set2<T>::clear() {
        mCount = 0;
        mTree->clear();
    }

    /*!
     * \returns true if set is empty
     */
    template <class T> bool Set2<T>::empty() {
        if (mCount > 0) {
            return false;
        } else {
            return true;
        }
    }

    /*!
     * \returns true if data found, else false
     */
    template <class T> bool Set2<T>::contains(T& data) const {
        bool ret = false;
        ret = mTree->contains(data);
        return ret;
    }

    /*!
     * \brief removes Data from Set
     * \returns true if removed successfully, else false
     */
    template <class T> bool Set2<T>::remove(T& data) {
        bool ret = false;
        ret = mTree->remove(data);
        return ret;
    }

    /*!
     * \brief Get Element from the beginning of the set
     * FIFO
     * use this to get all values:
     *   while (!Set::endPosition()) {Set::getElement()}
     * \returns element from set
     */
    template <class T> const T& Set2<T>::getElement() {
        //TODO
    }

    /*!
     * \brief Get Element from the end of the set and remove it
     * LIFO
     * gets element from the back of the set and also remove it
     * use this to get all values: while (!Set::empty) {Set::pop_back()}
     * \returns element from set
     */
    template <class T> T& Set2<T>::pop_back() {
        if (mCount == 0) {
            // return default element:
            return mDefaultElement;
        }
        // decrease already at this point:
        --mCount;
        //TODO
    }

    /*!
     * \returns size of the Set
     */
    template <class T> inline unsigned int Set2<T>::size() const {
        return mCount;
    }

}
#endif // DCOLLIDE_SET_H

