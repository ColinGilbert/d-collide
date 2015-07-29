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

#include "multimapelementtest.h"

#include <d-collide/math/vector.h>
#include <d-collide/datatypes/multimapelement.h>

#include <set>

using namespace dcollide;

CPPUNIT_TEST_SUITE_REGISTRATION (MultiMapElementTest);

namespace dcollide {

    /*!
     *  \brief resource allocation for unit test
     */
     void MultiMapElementTest::setUp(void) {
        //Constructor for Unit Tests
        //   *not needed here*
     }

    /*!
     *  \brief resource deallocation for unit test
     */
     void MultiMapElementTest::tearDown(void) {
        //Destructor for Unit Tests
        //   *not needed here*
     }

    void MultiMapElementTest::basicTest() {
    
        Vector3 readData;
        std::set<MultiMapElement<Vector3, Vector3> > testSet;
        std::set<MultiMapElement<Vector3, Vector3> >::iterator position;
        
        Vector3 data(5, 5, 5);
        Vector3 a(0, 1, 3);
        Vector3 b(3, 0, 1);
        
        
        /* test if the compare functions work correctly with the std::set */
        testSet.insert(MultiMapElement<Vector3, Vector3>(&a, &b, data));
        
        position = testSet.end();
        position = testSet.find(MultiMapElement<Vector3, Vector3>(&b, &a));
        CPPUNIT_ASSERT_NOT_EQUAL(testSet.end(), position);
        
        testSet.insert(MultiMapElement<Vector3, Vector3>(&b, &a));
        CPPUNIT_ASSERT_EQUAL((int) 1, (int) testSet.size());
        
        readData = (*position).getData();
        CPPUNIT_ASSERT_EQUAL(data, readData);
    }

}
