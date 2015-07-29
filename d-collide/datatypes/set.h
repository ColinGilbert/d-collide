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

#ifndef DCOLLIDE_SET_H
#define DCOLLIDE_SET_H

#include "debug.h"
#include "datatypes/list.h"
#include "collisionpair.h"

#include <cstring>

namespace dcollide {

    // The Hash
    template <class T> class Hash {

        public:

            Hash(int hashValues,
                    T* theSetData);
            ~Hash();

            inline bool insertHashValue(
                   int value, int position, const T& data);

            inline void removeHashValue(int value, 
                    const T& data);

            inline void clear();

            inline int find(int value, T& data) const ;
            inline bool contains(T& data) const ;

            inline void setTheSetData(T* theSetData);

        private:
            // Copy c'tor
            Hash(const Hash& hash);

            const T* mTheSetData;

            inline bool hashValueExists(int value);

            int mNumberOfHashValues;

            // This array saves a list of integers as a fake 2-dim array,
            // which are the index of the according vector in class Set
            List<int>* mIndexes;
    };

    /* each data will be assigned to one hash-cell. The "adress" of this
       cell must be computed somehow from the "value" of the data. The easiest
       thing for this would be to use pointers only. If one address is already
       occupied, than this data already exists in the cell and we have to check
       wether the data is already inserted or not. If it is new data, we have 
       to add this data to the list in this hashcell

       To get unique "ids" of a data, we could force the user of this set
       to implement a method getHashValue() for the type of data he wants to 
       save in the set

       The unique id aka HashValue is 1 integer, representing pos. in
       hash

       Use modulo operation to get hashvalue 

       Idea: to get an idea how big the hash and the array of the data must be
       we could allow the user to give a max number of elemenst to save. 
    */
    template <class T> class Set {

        public:
            Set(int maxNumber = 1021);
            ~Set();
            // unique insert
            inline bool insert(const T& data);

            inline void clear();

            inline bool empty();

            inline unsigned int size() const;

            inline bool contains(T& data) const;
            inline int find(T& data) const ;

            inline int getMaxNumber() const;
            inline int getNumberOfHashValues() const;

           // inline int calculateHashValue(void* dataPointer);

            inline const T& getElement() ;
            inline const T& getElement(int position) const;

            inline bool remove(T& data);

            inline T& pop_back();

            inline void resetPosition();
            inline unsigned int getPosition();
            inline bool endPosition() ;
            inline int getEndPosition() const;

        private:

            // Copy c'tor
            Set(const Set& hash);

            /*!
             * \brief Actual number of members in the Set
             */
            unsigned int mCount;

            /*!
             * \brief position counter set by \ref getElement
             * this counter is increased each time getElement is called,
             * can be reseted via resetPosition()
             * FIXME: How do we reset this counter? Perhaps automatically
             *        if endPosition was reached?
             */
            unsigned int mPosition;

            /*!
             * \brief will be returned if mCount = 0;;
             */
            T mDefaultElement;

            /*!
             * \brief The Hash of this Set
             */
            Hash<T>* mHash;

            /*!
             * \brief Array of the data
             */
            T* mData;

            /*!
             * \brief Number of HashValues:
             */
            int mNumberOfHashValues;

            /*!
             * \brief Maximum number of elements in this set:
             */
            int mMaxNumber;

            void increaseDataContainer();
    };

    /*!
     * \brief c'tor of Hash
     * The number of rows is by default 8
     * \param Number of Lines of the Hash
     */
    template <class T> Hash<T>::Hash(int hashValues, T* theSetData) {
        mIndexes = new List<int>[hashValues];
        mNumberOfHashValues = hashValues;
        mTheSetData = theSetData;

        /* Fill every hashcell with a -1, so that it does contain anything
        for (int i = 0; i<(mHashLines*mHashRows); i++) {
            mIndexes[i] = -1;
        }*/
    }

    /*!
     * \brief copy c'tor of Hash
     */
    template <class T> Hash<T>::Hash(const Hash& hash) {

    }

    /*!
     * \brief d'tor of Hash
     */
    template <class T> Hash<T>::~Hash() {
        delete[] mIndexes;
    }

    /*!
     * \brief empty the hash
     * runs in O(n)
     */
    template <class T> void Hash<T>::clear() {
        for (int i = 0; i<mNumberOfHashValues; i++) {
            if (!mIndexes[i].empty()) {
                mIndexes[i].clear();
            }
        }
    }

    /*
     * \brief insert HashValue into Hash
     * \param The HashValue
     * \returns true if success, else false
     */
    template <class T> bool Hash<T>::insertHashValue(
            int value, int position, const T& data) {
        if (!hashValueExists(value)) {
            mIndexes[value].push_back(position);
            return true;
        } else {

            // Test if value that exists is representing the same data:
            for (ListNode<int>* it =
                    mIndexes[value].getFirstNode();
                    it != 0; 
                    it = it->getNext()) {
                if (mTheSetData[it->getData()] == data) {
                    return false;
                }
            }

            // if we reach this point, we can add it to the hash, because we 
            // did not find the same data in the hash:
            mIndexes[value].push_back(position);
            return true;
        }
    }

    /*!
     * \brief Removes HashValue from Hash
     * \param The HashValue to remove
     */
    template <class T> void Hash<T>::removeHashValue(
            int value, const T& data) {

        for (ListNode<int>* it =
            mIndexes[value].getFirstNode();
            it != 0;
            it = it->getNext()) {
            if (mTheSetData[it->getData()] == data) {
                mIndexes[value].erase(it);
                return;
            }
        }
    }

    /*
     * \brief checks if HashValue is already in the Hash
     * \param value The HashValue
     * \returns true if Value exists, else false
     */
    template <class T> bool Hash<T>::hashValueExists(int value) {
        if (mIndexes[value].empty()) {
            return false;
        } else {
            return true;
        }
    }
    
    /*
     * \brief finds data in Hash
     * \param value The HashValue
     * \param data The Data
     * \returns position in Set if exists, else -1
     */
    template <class T> int Hash<T>::find(int value, T& data) const {
        // Test if value that exists is representing the same data:
        for (ListNode<int>* it =
                mIndexes[value].getFirstNode();
                it != 0; 
                it = it->getNext()) {
            if (mTheSetData[it->getData()] == data) {
                return it->getData();
            }
        }
        return -1;
    }

    /*
     * \brief finds data in Hash
     * \param value The HashValue
     * \param data The Data
     * \returns pointer to position in Set if exists, else NULL
     */
    template <class T> bool Hash<T>::contains(T& data) const {
        const int value = getHashValue(data,mNumberOfHashValues);
        // Test if value that exists is representing the same data:
        for (ListNode<int>* it =
                mIndexes[value].getFirstNode();
                it != 0; 
                it = it->getNext()) {
//            std::cout << mTheSetData[it->getData()] << " vs. " << data;
            if (mTheSetData[it->getData()] == data) {
//            std::cout << "TRUE" << std::endl;
                return true;
            }
        }
//        std::cout << "FALSE" << std::endl;
        return false;
    }
    
    /*!
     * \brief sets the new pointer bto the SetData
     */
    template <class T> void Hash<T>::setTheSetData(T* theSetData) {
        mTheSetData = theSetData;
    }

    /*!
     * \brief c'tor of Set
     * number of Set Elements should be a prime number
     * \param maximum number of elements in the set, default is 1021
     *
     */
    template <class T> Set<T>::Set(int maxNumber) {
        mMaxNumber = maxNumber;
        mNumberOfHashValues = maxNumber;
        mCount = 0;
        mPosition = 0;

        mData = new T[mMaxNumber];
        mHash = new Hash<T>(mNumberOfHashValues,this->mData);
    }

    /*!
     * \brief copy c'tor of Set
     */
    template <class T> Set<T>::Set(const Set& set) {

    }

    /*!
     * \brief d'tor of the Set
     */
    template <class T> Set<T>::~Set() {
        delete mHash;
        delete[] mData;
    }

    /*!
     * \brief inserts data into the set
     * \param the data to insert
     * \return true if success, false if already in the set
     */
    template <class T> bool Set<T>::insert(const T& data) {
        const int hv = getHashValue(data,mNumberOfHashValues);

        bool ret = false;
        // try to insert, position = mCount as array starts at 0!:
        int nextPosition = mCount;
        // Only insert if we haven't reached end of the set:
        if (nextPosition < mMaxNumber) {
            ret = mHash->insertHashValue(hv,nextPosition, data);

            // if success, add to data container mData:
            if (ret) {
                mData[nextPosition] = data;
                ++mCount;
            }

        // end of set reached, increase mData: 
        } else {
            increaseDataContainer();

            // now try again to add this:
            ret = mHash->insertHashValue(hv,nextPosition, data);

            // if success, add to data container mData:
            if (ret) {
                mData[nextPosition] = data;
                ++mCount;
            }
        }
        return ret;
    }

    /*!
     * \brief empties the set
     */
    template <class T> void Set<T>::clear() {
        mHash->clear();
        // FIXME: Is it necessary to delete them? If we clear the hash, this
        // data can no longer be accessed and will be overwritten, if we set
        // mCount = 0;
        mCount = 0;
        mPosition = 0;

    } 

    /*!
     * \returns true if set is empty 
     */
    template <class T> bool Set<T>::empty() {
        if (mCount > 0) {
            return false;
        } else {
            return true;
        }
    }

    /*!
     * \returns true if data found, else false 
     */
    template <class T> bool Set<T>::contains(T& data) const {
        if (mHash->contains(data)) {
            return true;
        }
        return false;
    }

    /*!
     * \returns position of data if data found, else -1
     */
    template <class T> int Set<T>::find(T& data) const {
        int pos = mHash->find(getHashValue(data,mNumberOfHashValues),data);
        return pos;
    }

    /*!
     * \brief removes Data from Set
     * FIXME Doesn't work
     * \returns true if removed successfully, else false
     */
    template <class T> bool Set<T>::remove(T& data) {
        bool success = false;
        // removing value from Hash and getting position in DataContainer:
        mHash->removeHashValue(getHashValue(data,mNumberOfHashValues),data);

        // Now we must delete this data from DataContainer:

        /*T* oldData = new T[mMaxNumber];// = mData;
        memcpy(oldData,mData,(mMaxNumber*sizeof(T)));
        delete[] mData;
        mData = new T[mMaxNumber+mNumberOfHashValues];
        memcpy(mData,oldData,(mMaxNumber*sizeof(T)));
        mMaxNumber += mNumberOfHashValues;
        delete[] oldData;
*/
        // Setting new Pointer in Hash:
        mHash->setTheSetData(mData);


        return success;
    }

    /*!
     * \brief resets position counter \ref mPosition 
     */
    template <class T> void Set<T>::resetPosition() {
        mPosition = 0;
    }
    /*!
     * \returns actual value of mPosition 
     */
    template <class T> unsigned int Set<T>::getPosition() {
        return mPosition;
    }

    /*!
     * \returns Last possible value of Position
     * you can use this to iterate through the set without
     * changing it.
     */
    template <class T> int Set<T>::getEndPosition() const {
        return (mCount-1);
    }

    /*!
     * \returns true if position is at the end of the list.
     */
    template <class T> bool Set<T>::endPosition() {
        if (mPosition == mCount) {
            // resetting Position Counter:
            resetPosition();
            return true;
        }
        return false;
    }

    /*!
     * \brief Get Element from the beginning of the set
     * FIFO
     * use this to get all values: 
     *   while (!Set::endPosition()) {Set::getElement()}
     * \returns element from set
     */
    template <class T> const T& Set<T>::getElement() {
        if (endPosition()) {
            // return default element: 
            return mDefaultElement;
        }
        // setting next Position:
       ++mPosition; 
       // here we return the reference saved in mData, indexed by actual
       // position:
       return mData[(mPosition-1)]; 
    }

    /*!
     * \brief Get Element from Position i
     * this method can be used to iterate through the Set-Data,
     * to get all values: 
     *    (for int i=0; i<= Set::getEndPosition(); ++i) {Set::getElement(i)}
     * \param the position of the element in the set
     * \returns element i from set
     */
    template <class T> const T& Set<T>::getElement(int position) const {
        // don't go behind the end!
        if (position > getEndPosition()) {
            // return default element: 
            return mDefaultElement;
        }
       // here we return the reference saved in mData at position i:
       return mData[position]; 
    }

    /*!
     * \brief Get Element from the end of the set and remove it
     * LIFO
     * gets element from the back of the set and also remove it
     * use this to get all values: while (!Set::empty) {Set::pop_back()}
     * \returns element from set
     */
    template <class T> T& Set<T>::pop_back() {
        if (mCount == 0) {
            // return default element: 
            return mDefaultElement;
        }
        // decrease already at this point:
        --mCount; 
        mHash->removeHashValue(getHashValue(mData[mCount],mNumberOfHashValues),
                mData[mCount]);
       // here we return the reference saved in mData:
       return mData[mCount]; 
    }

    /*!
     * \returns size of the Set
     */
    template <class T> inline unsigned int Set<T>::size() const {
        return mCount;
    }

    /*!
     * \returns mMaxNumber
     */
    template <class T> int Set<T>::getMaxNumber() const {
        return mMaxNumber;
    }

    /*!
     * \returns mNumberOfHashValues
     */
    template <class T> int Set<T>::getNumberOfHashValues() const {
        return mNumberOfHashValues;
    }

    /*!
     * \returns hashValue of given pointer
     * Uses simple modulo operation to get a hash value
     *
    template <class T> int Set<T>::calculateHashValue(void* dataPointer) {

        int hashValue = 0;

        // Getting address of bvol's and saving it as long int
        // FIXME: This is not platform-independant!
        unsigned long address = (unsigned long) dataPointer;

        hashValue = address % mNumberOfHashValues;

        return hashValue;
    }*/
    /*!
     * \brief increases mData by mNumberOfHashValues
     */
    template <class T> void Set<T>::increaseDataContainer() {
  //      std::cout << this << "III" << mNumberOfHashValues << std::endl;
        T* oldData = new T[mMaxNumber];// = mData;
        memcpy(oldData,mData,(mMaxNumber*sizeof(T)));
        delete[] mData;
        mData = new T[mMaxNumber+mNumberOfHashValues];
        memcpy(mData,oldData,(mMaxNumber*sizeof(T)));
        mMaxNumber += mNumberOfHashValues;
        delete[] oldData;

   //     std::cout << this << "IIII" << mNumberOfHashValues << std::endl;
        // Setting new Pointer in Hash:
        mHash->setTheSetData(mData);
    }
}
  /*!
   * \brief
   * \param dataPointer The Pointer to save
   * \param NumberOfHashValues The actual maximum Number of Hash Values
   * \returns hashValue of given pointer
   * Uses simple modulo operation to get a hash value
   */
  inline int getHashValue(const void* dataPointer,int NumberOfHashValues) {
  
    int hashValue = 0;

    // Getting address of bvol's and saving it as long int
    // FIXME: This is not platform-independant!
    unsigned long address = (unsigned long) dataPointer;

    hashValue = address % NumberOfHashValues;
        
    return hashValue;
  }

  /*!
   * \brief
   * \param dataPointer The Pointer to save
   * \param NumberOfHashValues The actual maximum Number of Hash Values
   * \returns hashValue of given pointer
   * Uses simple modulo operation to get a hash value
   */
  inline int getHashValue(const dcollide::CollisionPair& colPair,int NumberOfHashValues) {

    int hashValue = 0;

    // Getting address of bvol's and saving it as long int
    // FIXME: This is not platform-independant!
    unsigned long addressBVol1 = (unsigned long) colPair.bvol1; 
    unsigned long addressBVol2 = (unsigned long) colPair.bvol2;


    hashValue = (addressBVol1 + addressBVol2) % NumberOfHashValues;

    return hashValue;
  }

#endif // DCOLLIDE_SET_H 

