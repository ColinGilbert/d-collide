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

//include directives
#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <math.h>

#include "math/vector.h"
#include "boundingvolumetest.h"
#include "boundingvolumes/aabb.h"
/* TODO: Testing Suite for kdops, but as far as they are not supported yet
 *       there is no need (nor the possibility) to test them.
 */
#include "boundingvolumes/kdop.h"
#include "boundingvolumes/boundingsphere.h"

//-------------------------------------
//-------using directives--------------
using namespace std;
using namespace dcollide;

/*
add the class test to a global list of tests
if this line is forgotten - nothing will happen
the test methods will never get called

IMPORTANT:
This registration MUST happen in the .cpp file
If it is done in the header, which is also possible,
the registration gets run every time the header is
included. The Unit Test may be registered several times.
*/

CPPUNIT_TEST_SUITE_REGISTRATION (BoundingVolumeTest);


namespace dcollide {
    /*!
     *  \brief resource allocation for unit test
     */
     void BoundingVolumeTest::setUp(void) {
        //Constructor for Unit Tests
        //not needed here
        //use this to allocate resources as memory, files etc.
     }

    /*!
     *  \brief resource deallocation for unit test
     */
     void BoundingVolumeTest::tearDown(void) {
        //Destructor for Unit Tests
        //not needed here
        //use this to free the resources allocated in setUp()
     }

     /*!
      *  \brief checks if BoundingVolumes return their correct type
      */
     void BoundingVolumeTest::ReturnTypeTest() {
         Aabb testAabb;
         Kdop testKdop;
         BoundingSphere testSphere;
         CPPUNIT_ASSERT(
                testAabb.getVolumeType() == BV_TYPE_AABB);
         CPPUNIT_ASSERT(
                testKdop.getVolumeType() == BV_TYPE_KDOP);
         CPPUNIT_ASSERT(
                testSphere.getVolumeType() == BV_TYPE_SPHERE);
     }

}
/*
 * vim: et sw=4 ts=4
 */
