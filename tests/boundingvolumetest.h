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

/*******************************************************************************
 * What this testsuite is for and what it is not for                           *
 * -------------------------------------------------                           *
 * This testfile tests the interaction of _different_ boundingvolumes.         *
 * Interaction of equal boundingvolumes is already tested by the according     *
 * testsuite. But what is not sure is, wether different boundingvolumes        *
 * interact correctly or not.                                                  *
 * What is also tested are boundingvolume-specific commands                    *
 *
 * Actually, as there are no kops
 ******************************************************************************/
        
#ifndef DCOLLIDE_BOUNDINGVOLUMETEST
#define DCOLLIDE_BOUNDINGVOLUMETEST

//-------------------------------------
//-------#include directives-----------
#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "math/vector.h"
#include "boundingvolumes/boundingvolume.h"
#include "boundingvolumes/aabb.h"
/* TODO: Interaction with Kdops, but as far as they are not fully implemented
 *       yet, there is the need (nor the possibility) to test them.
 *       But as soon as they are implemented they should be included into
 *       testsuite.
 */
#include "boundingvolumes/kdop.h"

//-------------------------------------
//-------using directives--------------

using namespace CPPUNIT_NS;

namespace dcollide {

    /*!
     * \brief Test Class for BoundingVolume
     */
    class BoundingVolumeTest : public TestFixture {
        CPPUNIT_TEST_SUITE (BoundingVolumeTest);
            CPPUNIT_TEST ( ReturnTypeTest );
        CPPUNIT_TEST_SUITE_END();
    public:
        void setUp();
        void tearDown();

    protected:
        void ReturnTypeTest();
    };

    //------------ Implementation of short methods -------------

}


#endif // DCOLLIDE_BoundingVolumeTEST
/*
 * vim: et sw=4 ts=4
 */
