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


#ifndef DCOLLIDE_MULTIMAPELEMENT_H
#define DCOLLIDE_MULTIMAPELEMENT_H

namespace dcollide {

    /*!
     * \brief Implements a multi-map element for sets
     * 
     * This class is a implementation of a multi-map element for normal sets.
     * With this class you can use two different keys which maps any data
     * element. Thereby are two MultiMapElements equal if the both keys are
     * equal and espescially it doesn't matter in which order you give the key
     * elements. See the example below:
     * MultiMapElement("abc", "def") == MultiMapElement("def", "abc")
     * 
     * Please notice, that the implementation of this class requires, that your
     * key elements must support the operators '==' and '<'.     
     */
    template <typename Key, typename Data> class MultiMapElement {
         public:
             inline MultiMapElement(const Key* k1, const Key* k2);
             inline MultiMapElement(const Key* k1, const Key* k2, Data& data);
             inline ~MultiMapElement();

             inline const Key* getFirstKey() const;
             inline const Key* getSecondKey() const;

             inline const Data& getData() const;
             inline Data& getData();
             inline void setData(Data& data);

             inline bool operator<(const MultiMapElement& other) const;
             inline bool operator<=(const MultiMapElement& other) const;
             inline bool operator==(const MultiMapElement& other) const;

         private:
             Data mData;

             const Key* mFirstKey;
             const Key* mSecondKey;

             inline const Key* getFirstCompareKey() const;
             inline const Key* getSecondCompareKey() const;
     };


     /*!
      * \brief Constructs a MultiMapElement and assigns the given keys to the
      *        internal member variables 
      */
     template <class Key, class Data> inline MultiMapElement<Key, Data>::MultiMapElement(const Key* k1, const Key* k2) {
         mFirstKey = k1;
         mSecondKey = k2;
     }

     /*!
      * \brief Constructs a MultiMapElement and assigns the given keys and the
      *        given data to the internal member variables 
      */
     template <class Key, class Data> inline MultiMapElement<Key, Data>::MultiMapElement(const Key* k1, const Key* k2, Data& data) {
         mFirstKey = k1;
         mSecondKey = k2;
         mData = data;
     }

     /*!
      * \brief Destroys a MultiMapElement.
      * 
      * It does nothing in concrete at the moment!
      */
     template <class Key, class Data> inline MultiMapElement<Key, Data>::~MultiMapElement() {
         /* nothing todo at the moment */
     }

     /*
      * \brief Returns the first key of the both saved keys
      */
     template <class Key, class Data> inline const Key* MultiMapElement<Key, Data>::getFirstKey() const {
         return mFirstKey;
     }

     /*!
      * \brief Returns the second key of the both saved keys
      */
     template <class Key, class Data> inline const Key* MultiMapElement<Key, Data>::getSecondKey() const {
         return mSecondKey;
     }


     /*
      * \brief Returns the key which is greater of the both saved keys, if both
      *        keys are equal the one which is saved as first one is returned
      */
     template <class Key, class Data> inline const Key* MultiMapElement<Key, Data>::getFirstCompareKey() const {
         return (mFirstKey >= mSecondKey) ? mFirstKey : mSecondKey;
     }

     /*!
      * \brief Returns the key which is the smaller one of the both saved keys,
      *        if both keys are equal the one which is saved as first one is
      *        returned
      */
     template <class Key, class Data> inline const Key* MultiMapElement<Key, Data>::getSecondCompareKey() const {
         return (mSecondKey <= mFirstKey) ? mSecondKey : mFirstKey;
     }


     /*!
      * \brief Returns the content of the internal mData variable as a const
      *        pointer
      * 
      * A const reference is returned, so that you can use this class even in
      * const sets.
      */     
     template <class Key, class Data> inline const Data& MultiMapElement<Key, Data>::getData() const {
         return mData;
     }

     /*!
      * \brief Returns the content of the internal mData variable
      * 
      * A non const reference is returned, so that it is possible to change the
      * data later on.
      */
     template <class Key, class Data> inline Data& MultiMapElement<Key, Data>::getData() {
         return mData;
     }

     /*!
      * \brief Simply sets the internal mData variable to the given pointer
      * 
      * OWNERSHIP NOTICE:
      * The ownership isn't taken and relies to the callee. But please notice
      * that it is possible to change the data from here. This class wont do
      * this on its own, but the user could possibly do it.
      */
     template <class Key, class Data> inline void MultiMapElement<Key, Data>::setData(Data& data) {
         mData = data;
     }


     /*!
      * \brief Implementation of the < operator for MultiMapElement class
      * 
      * Compares mFirstKey and mSecondKey of this and the \p other
      * MultiMapElement with each other.
      */
     template <class Key, class Data> inline bool MultiMapElement<Key, Data>::operator<(const MultiMapElement<Key, Data>& other) const {
         if (getFirstCompareKey() < other.getFirstCompareKey()) {
             return true;
         }
         if ((getFirstCompareKey() == other.getFirstCompareKey()) && (getSecondCompareKey() < other.getSecondCompareKey())) {
             return true;
         }
         return false;
     }

     /*!
      * \brief Implementation of the <= operator for MultiMapElement class
      * 
      * Compares this and the other MultiMapElement with each other. It uses
      * the MultiMapElement::operator< and MultiMapElement::operator==
      * methods.
      */
     template <class Key, class Data> inline bool MultiMapElement<Key, Data>::operator<=(const MultiMapElement<Key, Data>& other) const {
         return ((*this < other) || (*this == other));
     }

     /*!
      * \brief Implementation of the == operator for MultiMapElement class
      * 
      * Compares mFirstKey and mSecondKey of this and the \p other
      * MultiMapElement with each other.
      */
     template <class Key, class Data> inline bool MultiMapElement<Key, Data>::operator==(const MultiMapElement<Key, Data>& other) const {
         return ((other.getFirstCompareKey() == getFirstCompareKey())  && (other.getSecondCompareKey() == getSecondCompareKey()));
     }
}

#endif /* DCOLLIDE_MULTIMAPELEMENT_H */
