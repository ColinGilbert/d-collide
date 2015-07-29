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

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "listtest.h"

using namespace std;
using namespace dcollide;

CPPUNIT_TEST_SUITE_REGISTRATION(ListTest);

namespace dcollide {
    void ListTest::setUp(void) {
    }

    void ListTest::tearDown(void) {
    }

    void ListTest::testList() {
        List<int> list;
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);

        list.clear();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);

        list.push_back(1);
        CPPUNIT_ASSERT_EQUAL((unsigned int)1, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(false, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() != 0);
        CPPUNIT_ASSERT(list.getLastNode() != 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getFirstNode() == list.getLastNode());
        CPPUNIT_ASSERT_EQUAL(1, list.front());

        list.clear();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);

        list.push_back(1);
        CPPUNIT_ASSERT_EQUAL((unsigned int)1, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(false, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() != 0);
        CPPUNIT_ASSERT(list.getLastNode() != 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getFirstNode() == list.getLastNode());
        CPPUNIT_ASSERT_EQUAL(1, list.front());

        list.push_back(2);
        CPPUNIT_ASSERT_EQUAL((unsigned int)2, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(false, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() != 0);
        CPPUNIT_ASSERT(list.getLastNode() != 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getFirstNode() != list.getLastNode());
        CPPUNIT_ASSERT_EQUAL(1, list.front());

        list.clear();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);

        for (int i = 1; i <= 20; i++) {
            list.push_back(i);
            CPPUNIT_ASSERT_EQUAL((unsigned int)i, list.size());
            CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
            CPPUNIT_ASSERT_EQUAL(false, list.empty());
            CPPUNIT_ASSERT(list.getFirstNode() != 0);
            CPPUNIT_ASSERT(list.getLastNode() != 0);
            CPPUNIT_ASSERT(list.getFirstNode()->mPrevious == 0);
            CPPUNIT_ASSERT(list.getLastNode()->mNext == 0);
        }

        ListNode<int>* node = list.getFirstNode();
        ListNode<int>* prev = 0;
        int number = 1;
        for (; node; node = node->mNext) {
            if (node != list.getFirstNode()) {
                CPPUNIT_ASSERT(node->mPrevious != 0);
            }
            if (node != list.getLastNode()) {
                CPPUNIT_ASSERT(node->mNext != 0);
            }
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            CPPUNIT_ASSERT_EQUAL(number, node->mData);
            if (prev) {
                CPPUNIT_ASSERT_EQUAL(prev->mNext, node);
            }

            prev = node;
            number++;
        }


        list.clear();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);

        list.push_back(42);
        list.push_back(42 * 2);
        list.push_back(1);
        list.push_back(2);
        list.push_back(3);
        list.push_back(4);
        list.push_back(5);
        list.push_back(6);
        CPPUNIT_ASSERT_EQUAL((unsigned int)8, list.size());

        // erase of the very first node (== pop_front())
        list.erase(list.getFirstNode());
        CPPUNIT_ASSERT_EQUAL((unsigned int)7, list.size());
        {
            ListNode<int>* node = list.getFirstNode();
            ListNode<int>* prev = 0;
            CPPUNIT_ASSERT_EQUAL(42 * 2, node->mData);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(1, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(2, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(3, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(4, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(5, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(6, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            node = node->mNext;
            CPPUNIT_ASSERT(node == 0);
        }

        // erase of an "inner" node
        list.erase(list.getFirstNode()->mNext);
        {
            ListNode<int>* node = list.getFirstNode();
            ListNode<int>* prev = 0;
            CPPUNIT_ASSERT_EQUAL(42 * 2, node->mData);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(2, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(3, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(4, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(5, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(6, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            node = node->mNext;
            CPPUNIT_ASSERT(node == 0);
        }
        CPPUNIT_ASSERT_EQUAL((unsigned int)6, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(false, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() != 0);
        CPPUNIT_ASSERT(list.getLastNode() != 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getFirstNode() != list.getLastNode());

        // erase of the last node (== pop_back())
        list.erase(list.getLastNode());
        {
            ListNode<int>* node = list.getFirstNode();
            ListNode<int>* prev = 0;
            CPPUNIT_ASSERT_EQUAL(42 * 2, node->mData);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(2, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(3, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(4, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(5, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            node = node->mNext;
            CPPUNIT_ASSERT(node == 0);
        }
        CPPUNIT_ASSERT_EQUAL((unsigned int)5, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(false, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() != 0);
        CPPUNIT_ASSERT(list.getLastNode() != 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getFirstNode() != list.getLastNode());

        // actual pop_back()
        list.pop_back();
        {
            ListNode<int>* node = list.getFirstNode();
            ListNode<int>* prev = 0;
            CPPUNIT_ASSERT_EQUAL(42 * 2, node->mData);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(2, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(3, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(4, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            node = node->mNext;
            CPPUNIT_ASSERT(node == 0);
        }
        CPPUNIT_ASSERT_EQUAL((unsigned int)4, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(false, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() != 0);
        CPPUNIT_ASSERT(list.getLastNode() != 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getFirstNode() != list.getLastNode());

        // actual pop_front()
        list.pop_front();
        {
            ListNode<int>* node = list.getFirstNode();
            ListNode<int>* prev = 0;
            CPPUNIT_ASSERT_EQUAL(2, node->mData);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(3, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            prev = node;
            node = node->mNext;
            CPPUNIT_ASSERT_EQUAL(4, node->mData);
            CPPUNIT_ASSERT_EQUAL(prev, node->mPrevious);
            node = node->mNext;
            CPPUNIT_ASSERT(node == 0);
        }
        CPPUNIT_ASSERT_EQUAL((unsigned int)3, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(false, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() != 0);
        CPPUNIT_ASSERT(list.getLastNode() != 0);
        CPPUNIT_ASSERT(list.getFirstNode()->mPrevious == 0);
        CPPUNIT_ASSERT(list.getLastNode()->mNext == 0);
        CPPUNIT_ASSERT(list.getFirstNode() != list.getLastNode());


        list.clear();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);


        list.push_back(1);
        // pop_back() when first==last
        list.pop_back();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);


        list.clear();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);

        list.push_back(1);
        // pop_back() when first==last
        list.pop_front();
        CPPUNIT_ASSERT_EQUAL((unsigned int)0, list.size());
        CPPUNIT_ASSERT_EQUAL(list.size(), list.count());
        CPPUNIT_ASSERT_EQUAL(true, list.empty());
        CPPUNIT_ASSERT(list.getFirstNode() == 0);
        CPPUNIT_ASSERT(list.getLastNode() == 0);
    }
}
/*
 * vim: et sw=4 ts=4
 */
