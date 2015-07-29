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


#ifndef DCOLLIDE_LIST_H
#define DCOLLIDE_LIST_H

#include "dcollide-config.h"

#include <list>


// FIXME: remove?
#include "debug.h"
#include <iostream>

// #define DCOLLIDE_LIST_DEBUG 1

namespace dcollide {

    // AB: T must implement:
    // - copy c'tor
    // - default c'tor
    // - assignment(=) operator
    /*!
     * \brief Class that represents a node of \ref List
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    template <class T> class ListNode {
        public:
            inline ListNode();
            inline explicit ListNode(T data);
            inline ~ListNode();

            inline ListNode<T>* getNext() const;
            inline ListNode<T>* getPrevious() const;
            inline const T& getData() const;
            inline T& getData();

            ListNode<T>* mNext;
            ListNode<T>* mPrevious;
            T mData;
    };

    /*!
     * \brief Allocation pool for \ref List
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    template <class T> class ListNodePool {
        public:
            inline ListNodePool();
            inline ~ListNodePool();

            inline ListNode<T>* getNode();
            inline void releaseNode(ListNode<T>* node);
            inline void releaseFromTo(ListNode<T>* start, ListNode<T>* end, unsigned int count);

            inline void clearPool();

        private:
            ListNode<T>* mFirst;
#ifdef DCOLLIDE_LIST_DEBUG
            // AB: this is currently used for debugging only, but can also be
            // used to limit the maximum number of nodes (to prevent memory
            // leaks getting too large)
            unsigned int mCount;
#endif
    };

    /*!
     * \brief List class that uses an allocation pool
     *
     * This list class is meant as replacement of std::list in d-collide
     * whenever appropriate. The main feature of this class is that it provides
     * an allocation pool (see \ref ListNodePool):
     * \code
     * List<Foo> list;
     * list.push_back(Foo()); // new ListNode is created
     * list.pop_back();
     * list.push_back(Foo()); // new ListNode is NOT created
     * list.pop_back();
     * list.push_back(Foo()); // new ListNode is NOT created
     * \endcode
     * In the code above, only a single ListNode object is created. Every call
     * to \ref pop_back, \ref pop_front, \ref erase, \ref clear or similar
     * methods store the internal \ref ListNode objects in the \ref ListNodePool
     * and re-use them in the next \ref push_back call.
     *
     * This allocation pool makes this list in particular useful when items are
     * often added and removed to a list: in a std::list \em every item that is
     * added to the list also needs to create a node. In this \ref List however,
     * only the maximum value of \ref size of the list is also the maximum
     * number of \ref ListNode objects that need to be created.
     *
     * In particular this list can provide a O(1) implementation of \ref clear.
     * However as a disadvantage this list does \em not provide an
     * implementation of std::list::splice. So if you want to "move" one list
     * over to another list, std::list may be more efficient.
     *
     * All classes that should be used with this list need to implement:
     * \li the copy constructor
     * \li the default constructor
     * \li the assignment(=) operator
     *
     * Note that this is always the case if you construct a list of pointers,
     * the class that your pointers point to dont need to implement anything:
     * \code
     * // Foo has to implement additional functions:
     * List<Foo> list1;
     *
     * // Foo does NOT have to implement additional functions:
     * List<Foo*> list2;
     * \endcode
     *
     * \author Andreas Beckermann <b_mann@gmx.de>
     */
    template <class T> class List {
        public:
            inline List();
            inline List(const List<T>& otherList);
            inline explicit List(ListNodePool<T>* pool); // the pool is NOT thread safe! -> dont use the same pool in multiple threads
            inline ~List();

            inline List& operator=(const List<T>& otherList);

            inline void push_back(const T& data);
            inline void pop_front();
            inline void pop_back();
            inline T& front() const;
            inline T& back() const;

            inline void appendList(const List<T>& otherList);
            inline void appendStdList(const std::list<T>& otherList);

            inline void clear();
            inline void erase(ListNode<T>* node);
//            inline void splice(List<T>* list); // AB: may not get implemented (see below)
            inline void removeAll(const T& data);
            inline void removeFirst(const T& data);

            inline unsigned int size() const;
            inline unsigned int count() const;
            inline bool empty() const;

            inline ListNode<T>* getFirstNode() const;

            // note: pointer to the _last_ element, not beyond the last element
            // (like std::list::end())
            inline ListNode<T>* getLastNode() const;

        private:
            ListNodePool<T>* mPool;
            bool mDeletePool;

            unsigned int mCount;
            ListNode<T>* mFirst;
            ListNode<T>* mLast;
    };



    /*!
     * Construct a new ListNode object. For the \ref getData, the default
     * constructor is used.
     */
    template <class T> inline ListNode<T>::ListNode() :
            mNext(0),
            mPrevious(0) {
    }

    /*!
     * Construct a ListNode that stores \p data.
     *
     * The copy constructor of \p data is used to store the data in this node.
     * I.e. for example:
     * \code
     * Foo a;
     * ListNode<Foo> node(a); // Foo::Foo(a) is called
     *
     * // BUT:
     * ListNode<Foo*> node(&a); // Foo::Foo(&a) is NOT called, because Foo* is copied, not Foo!
     * \endcode
     */
    template <class T> inline ListNode<T>::ListNode(T data) :
            mNext(0),
            mPrevious(0),
            mData(data) {
    }

    /*!
     * Destruct this list node.
     *
     * This sets the "next" pointer of the "previous" node to 0 and also deletes
     * the "next" node.
     */
    template <class T> inline ListNode<T>::~ListNode() {
        if (mPrevious) {
            mPrevious->mNext = 0;
        }

#if 1
        // AB: we don't simply delete mNext here, because for very long lists this
        // may cause a stack overflow!
        // -> note however, that if a stack overflow is possible, there is
        //    probably a memory leak around...
        ListNode* n = mNext;
        while (n) {
            ListNode* n2 = n->mNext;
            n->mPrevious = 0;
            n->mNext = 0;
            delete n;
            n = n2;
        }
#else
        if (mNext) {
            mNext->mPrevious = 0;
            delete mNext;
        }
#endif
    }

    /*!
     * \return The next node (i.e. the node after this node in the list) or NULL
     * if this is the end of the list
     */
    template <class T> inline ListNode<T>* ListNode<T>::getNext() const {
        return mNext;
    }

    /*!
     * \return The previous node (i.e. the node before this node in the list) or NULL
     * if this is the beginning of the list
     */
    template <class T> inline ListNode<T>* ListNode<T>::getPrevious() const {
        return mPrevious;
    }

    /*!
     * \return A const reference to the data associated with (i.e. stored in) this node.
     */
    template <class T> inline const T& ListNode<T>::getData() const {
        return mData;
    }

    /*!
     * \return A reference to the data associated with (i.e. stored in) this node.
     */
    template <class T> inline T& ListNode<T>::getData() {
        return mData;
    }



    /*!
     * Construct an empty allocation pool.
     */
    template <class T> inline ListNodePool<T>::ListNodePool() :
            mFirst(0) {
#ifdef DCOLLIDE_LIST_DEBUG
        mCount = 0;
#endif
    }

    /*!
     * Destruct the allocation pool and delete all nodes stored in it. See also
     * \ref clearPool.
     */
    template <class T> inline ListNodePool<T>::~ListNodePool() {
        clearPool();
    }

    /*!
     * \return A \ref ListNode object for use in \ref List. If this allocation
     * pool is empty, a new \ref ListNode object is created, otherwise the first
     * available node is returned.
     */
    template <class T> inline ListNode<T>* ListNodePool<T>::getNode() {
        if (mFirst) {
            ListNode<T>* node = mFirst;
            mFirst = node->mNext;
            node->mNext = 0;
            node->mPrevious = 0; // AB: necessary, as releaseFromTo() keeps mPrevious intact (==dangling pointer)

#ifdef DCOLLIDE_LIST_DEBUG
            if (mCount == 0) {
                std::cerr << dc_funcinfo << "ERROR: mCount == 0" << std::endl;
            } else {
                mCount--;
            }
#endif
            return node;
        }
#ifdef DCOLLIDE_LIST_DEBUG
        if (mCount != 0) {
            std::cerr << dc_funcinfo << "ERROR: mFirst == 0, but mCount==" << mCount << std::endl;
        }
#endif
        return new ListNode<T>();
    }


    // TODO: do not save ALL nodes for later re-use!
    // -> if this lists holds 1000000 once for a short time, we'll never need
    //    all these objects again.
    //    maybe keep 20% or so of additional nodes
    /*!
     * Used by \ref List to release \p node from the list and give it to this
     * allocation pool for later usage.
     *
     * \p node is meant to be stored in this allocation pool, so that it can be
     * used in later calls to \ref getNode.
     */
    template <class T> inline void ListNodePool<T>::releaseNode(ListNode<T>* node) {
        // AB: note: if we ever delete nodes here: node->mNext MUST be set to
        // NULL! it may still point to a valid node
        node->mNext = mFirst;
        node->mPrevious = 0; // AB: we use the nodes as a single linked list in the pool.
        mFirst = node;

#ifdef DCOLLIDE_LIST_DEBUG
        mCount++;
#endif
    }

    // AB: we assume that start and end are actually connected with each other
    // (with start being before end)
    /*!
     * Like \ref releaseNode, but releases all nodes from \p start to \p end
     * (including both nodes).
     *
     * \param count The number of nodes in the list specified by \p start and \p
     * end.
     */
    template <class T> inline void ListNodePool<T>::releaseFromTo(ListNode<T>* start, ListNode<T>* end, unsigned int count) {
        end->mNext = mFirst;
        mFirst = start;

#ifdef DCOLLIDE_LIST_DEBUG
        mCount += count;
#else
        DCOLLIDE_UNUSED(count);
#endif

        // AB: we do NOT touch the mPrevious pointers here, for efficiency
        // reasons.
        // note that they are considered to be dangling pointers at this point!
    }

    /*!
     * Delete all internally stored nodes. Later calls to \ref getNode will need
     * to create new nodes.
     */
    template <class T> inline void ListNodePool<T>::clearPool() {
        // deletes mFirst and all "next" nodes.
        delete mFirst;
        mFirst = 0;
    }



    /*!
     * Construct a new empty list.
     */
    template <class T> inline List<T>::List() :
            mPool(new ListNodePool<T>()),
            mDeletePool(true),
            mCount(0),
            mFirst(0),
            mLast(0) {
    }

    /*!
     * Construct a new empty list and use \p pool as allocation pool.
     *
     * The ownership of \p pool is NOT taken.
     *
     * This method is meant to "share" an allocation pool among different lists.
     * Note that this also means that when using this list and another list that
     * uses \pool in different threads, you need to add mutex locks whenever you
     * add or remove item to or from one of the lists.
     */
    template <class T> inline List<T>::List(ListNodePool<T>* pool) :
            mPool(pool),
            mDeletePool(false),
            mCount(0),
            mFirst(0),
            mLast(0) {
    }

    /*!
     * Create a new list as a copy of \p otherList. All elements of \p otherList
     * are copied to this list, see also \ref appendList
     */
    template <class T> inline List<T>::List(const List<T>& otherList) :
            mPool(new ListNodePool<T>()),
            mDeletePool(true),
            mCount(0),
            mFirst(0),
            mLast(0) {
        appendList(otherList);
    }

    /*!
     * Destruct this list and delete the allocation pool.
     */
    template <class T> inline List<T>::~List() {
        clear();
        if (mDeletePool) {
            delete mPool;
        }
    }

    /*!
     * Assign \p otherList to this list. See also \ref clear and \ref appendList
     */
    template <class T> inline List<T>& List<T>::operator=(const List<T>& otherList) {
        clear();
        appendList(otherList);
        return *this;
    }

    // will crash if list is empty!
    /*!
     * \return The \ref ListNode::getData of the first list item. This method
     * assumes that \ref empty is false!
     */
    template <class T> inline T& List<T>::front() const {
        return mFirst->mData;
    }

    // will crash if list is empty!
    /*!
     * \return The \ref ListNode::getData of the last list item. This method
     * assumes that \ref empty is false!
     */
    template <class T> inline T& List<T>::back() const {
        return mLast->mData;
    }

    /*!
     * \return The number of items in this list. This method works in O(1)!
     */
    template <class T> inline unsigned int List<T>::size() const {
        return mCount;
    }

    /*!
     * \return The number of items in this list. This method works in O(1)!
     * See also \ref size
     */
    template <class T> inline unsigned int List<T>::count() const {
        return size();
    }

    /*!
     * \return TRUE if this list is empy, otherwise FALSE
     */
    template <class T> inline bool List<T>::empty() const {
#ifdef DCOLLIDE_LIST_DEBUG
        if ((mFirst == 0) != (mCount == 0)) {
            std::cerr << dc_funcinfo << "(mFirst == 0) != (mCount == 0)" << std::endl;
        }
#endif
        return (mFirst == 0);
    }

    /*!
     * Add \p data to the end of this list.
     */
    template <class T> inline void List<T>::push_back(const T& data) {
        ListNode<T>* node = mPool->getNode();
        node->mData = data;

        if (mLast) {
            mLast->mNext = node;
        } else {
            mFirst = node;
        }
        node->mPrevious = mLast;
        mLast = node;

        mCount++;
    }

    /*!
     * Remove the first entry from this list. Emits an error if \ref empty is
     * TRUE.
     */
    template <class T> inline void List<T>::pop_front() {
        if (mFirst) {
            ListNode<T>* node = mFirst;
            mFirst = mFirst->mNext;
            if (mFirst == 0) {
                mLast = 0;
            } else {
                mFirst->mPrevious = 0;
            }
            mPool->releaseNode(node);
            mCount--;
        } else {
            std::cerr << dc_funcinfo << "ERROR: list already empty" << std::endl;
        }
    }

    /*!
     * Remove the last entry from this list. Emits an error if \ref empty is
     * TRUE.
     */
    template <class T> inline void List<T>::pop_back() {
        if (mLast) {
            ListNode<T>* node = mLast;
            if (mLast->mPrevious == 0) {
                mFirst = 0;
            } else {
                mLast->mPrevious->mNext = 0;
            }
            mLast = mLast->mPrevious;
            mPool->releaseNode(node);
            mCount--;
        } else {
            std::cerr << dc_funcinfo << "ERROR: list already empty" << std::endl;
        }
    }

    /*!
     * Append all items from \p otherList to this list, by calling \ref
     * push_back on all items of \p otherList (see \ref ListNode::getData()).
     */
    template <class T> inline void List<T>::appendList(const List<T>& otherList) {
        for (ListNode<T>* node = otherList.getFirstNode(); node; node = node->mNext) {
            push_back(node->mData);
        }
    }

    /*!
     * Append all items from \p otherList to this list, by calling \ref
     * push_back on all items of \p otherList.
     */
    template <class T> inline void List<T>::appendStdList(const std::list<T>& otherList) {
        // AB: the keyword "typename" is required here, see e.g.
        // http://pages.cs.wisc.edu/~driscoll/typename.html
        for (typename std::list<T>::const_iterator it = otherList.begin(); it != otherList.end(); ++it) {
            push_back(*it);
        }
    }

    /*!
     * Make this list an empty list. This moves all internal nodes from this
     * list to the internal allocation pool.
     *
     * This method works in O(1).
     */
    template <class T> inline void List<T>::clear() {
        if (mFirst) {
            mPool->releaseFromTo(mFirst, mLast, mCount);
            mFirst = 0;
            mLast = 0;
            mCount = 0;
        }
    }

    /*!
     * This method is primarily meant for iterating the list. Example:
     * \code
     * List<Foo> list;
     * // ...
     * // add items to list
     * // ...
     * for (ListNode<Foo>* node = list.getFirstNode(); node; node = node->getNext()) {
     *     Foo data = list->getData();
     *     // do something
     * }
     * \endcode
     *
     * \return A pointer to the first \ref ListNode object of this list, or NULL
     * if this list is empty (see \ref empty).
     */
    template <class T> inline ListNode<T>* List<T>::getFirstNode() const {
        return mFirst;
    }

    /*!
     * \return A pointer to the last \ref ListNode object of this list, or NULL
     * if this list is empty (see \ref empty). See also \ref getFirstNode
     */
    template <class T> inline ListNode<T>* List<T>::getLastNode() const {
        return mLast;
    }

    // AB: note: if "node" does not belong to this list, the list will be
    // totally fucked up!
    // -> wrong mCount
    /*!
     * Remove \p node from this list.
     *
     * WARNING: this method assumes that \p node is actually part of this list,
     * but it does \em NOT check that! If \p node is not actually part of this
     * list, this list will be broken (in particular \ref size will return wrong
     * values)
     */
    template <class T> inline void List<T>::erase(ListNode<T>* node) {
        if (mCount == 0) {
            std::cerr << dc_funcinfo << "ERROR: list already empty" << std::endl;
            return;
        }
        if (node == mFirst) {
            pop_front();
            return;
        } else if (node == mLast) {
            pop_back();
            return;
        }

        ListNode<T>* prev = node->mPrevious;
        ListNode<T>* next = node->mNext;
        if (prev) {
            prev->mNext = next;
        }
        if (next) {
            next->mPrevious = prev;
        }
        mPool->releaseNode(node);
        mCount--;
    }

    /*!
     * Remove all occurances of \p data from the list
     */
    template <class T> inline void List<T>::removeAll(const T& data) {
        ListNode<T>* n = getFirstNode();
        while (n) {
            ListNode<T>* next = n->getNext();
            if (n->getData() == data) {
                erase(n);
            }
            n = next;
        }
    }

    /*!
     * Remove the first occurance of \p data from the list
     */
    template <class T> inline void List<T>::removeFirst(const T& data) {
        for (ListNode<T>* n = getFirstNode(); n; n = n->getNext()) {
            if (n->getData() == data) {
                erase(n);
                return;
            }
        }
    }


    // AB: I think we should not provide this method, otherwise people will
    //     (mis-)use it.
    //     if the other list remains in use, its nodes will not go to the pool
    //     and thus needs to reallocate sooner.
    //     on the other hand, this list will have MORE nodes and possibly never
    //     run out of them -> huge memory leak if you are not careful!
    //
    //     however this is an unexpected behaviour and thus I think we should
    //     disallow this completely.
#if 0
    /*!
     * \brief Moves all elements from \p list to the \em end of this list.
     *
     * After this call \p list is empty and all its elements are in this list.
     *
     * Implementation note: the ownership of all nodes of \p list is transferred
     * to this list, i.e. the \ref ListNodePool of this class will get the
     * nodes afterwards.
     */
    // WARNING: this moves over all nodes from the other list to this list.
    //          consequently, if the other list remains in use, push_back() has
    //          fewer nodes available in the pool to use and probably needs more
    //          allocations!
    //          avoid this method!
    template <class T> inline void List<T>::splice(List<T>* list) {
        mLast->mNext = list->mFirst;
        mLast = list->mLast;
        mCount += list->mCount;

        list->mFirst = 0;
        list->mLast = 0;
        list->mCount = 0;
    }
#endif

}


#endif
/*
 * vim: et sw=4 ts=4
 */
