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


#ifndef DCOLLIDE_ARRAY_H
#define DCOLLIDE_ARRAY_H

//For the size_t definition
#include <cstddef>
#include <iterator>

namespace dcollide {
    /*!
     * \brief Implementation of c-array functionality, which provides additional access via iterators
     * \params myType element-type of container
     * \params size   element count
     * 
     * Usage example: \n
     * dcollide::array<Vertex*,3> vertices; \n
     * This defines array of vertex pointers with three elements. \n
     * You may access the elements by array semantic: \n
     * vertices[1] = v; \n
     * \n
     * Additionally you may access its elements by iterators also, like this: \n
     * \n
     *    for(array<Vertex*,3>::iterator i = vertices.begin(); i !=vertices.end(); ++i ) {
     *        (*i).performAnything();
     *    }
     * \n
     * This implementation does no bound checking of iterators and indices,
     * so the user is responsible to use reasonable values. \n
     * \n
     * The implementation is also not comprehensive, some methods are not implemented
     * yet; there is no postfix ++ operator, so use ++i no i++. No 'insert' method
     * but the user may use operator[] instead.
     */
    template<typename myType, std::size_t size> class array {
        public:
                
            /*!
             * \brief This allows stl-like iterator access to the array class
             * Essentially this class is a wrapper around a pointer.
             * It is an inner class of array so 'myType' will be substituted by
             * the same type as in the array class
             */
            class iterator {
                protected:
                    myType* mPtr;
                public:

                    iterator() : mPtr(0) {
                    }

                    iterator(myType* ptr) : mPtr(ptr) {
                    }

                    inline iterator& operator=(iterator i) {
                        mPtr = i.mPtr;

                        return (*this);
                    }

                    inline bool operator==(iterator i) const {
                        return (mPtr == i.mPtr);
                    }

                    inline bool operator!=(iterator i) {
                        return (mPtr != i.mPtr);
                    }
                    inline myType operator*() const {
                        return (*mPtr);
                    }

                    inline iterator& operator++() {
                          mPtr++;

                          return (*this);
                    }

                    inline iterator& operator--() {
                      mPtr--;

                      return (*this);
                    }
            };
            
            //This is a hack. Don´t know how to deal with const_iterators
            //proberbly
            
            typedef iterator const_iterator;

        private:
            myType* mArray;
       
        public:
            inline array() {
                mArray = new myType[size];
            }
            inline ~array() {
                delete[] mArray;
            }

            inline array(const array<myType, size>& a) {
                mArray = new myType[size];
                *this = a;
            }

            inline array<myType, size>& operator=(const array<myType, size>& a) {
                for (std::size_t i = 0; i < size; i++) {
                    mArray[i] = a[i];
                }
                return *this;
            }

            inline myType& operator[](std::size_t index) {
                return mArray[index];
            }

            inline const myType& operator[](std::size_t index) const {
                return mArray[index];
            }

            inline iterator begin() const {
                return iterator(mArray);
            }

            inline iterator end() const {
                return iterator(mArray + size);
            }
    };
}
#endif
/*
 * vim: et sw=4 ts=4
 */
