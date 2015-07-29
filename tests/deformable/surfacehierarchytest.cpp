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


#include "surfacehierarchytest.h"

#include "shapes/mesh/vertex.h"
#include "detectordeform/surfacehierarchy/deformablebvhnode.h"
#include "world.h"
#include "debugstream.h"

#include <modelloader/loaderdummy.h>
#include <modelloader/loaderply.h>


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

CPPUNIT_TEST_SUITE_REGISTRATION (SurfaceHierarchyTest);


namespace dcollide {

/*!
     *  \brief resource allocation for unit test
 */
    void SurfaceHierarchyTest::setUp(void) {
        //Constructor for Unit Tests

        /*try {
            world = new World();
        } catch (Exception e) {
            world = 0;
            throw (e);
        }
*/

        WorldParameters parameters;
        parameters.addPrimaryDeformableAlgorithm(DEFORMABLE_TYPE_SURFACE_HIERARCHY);
        parameters.setWorldMinMax(Vector3(-100.0, -100.0, -100.0), Vector3(100.0, 100.0, 100.0));
        mWorld = new World(parameters);
        
        ModelLoader::LoaderDummy loaderDummy;
        real width = 50.0;
        real height = 50.0;
        mClothMesh = loaderDummy.createRectangleSurface(width, height, 1.0, 1.0);
        //debug() << "created mesh with " << mClothMesh->getTriangles().size() << " triangles";
        Proxy* cloth = mWorld->createProxy(mClothMesh, PROXYTYPE_DEFORMABLE);
        cloth->translate(-width / 2.0, -height / 2.0, 0.0);
        mWorld->addProxy(cloth);


        mWorld->prepareSimulation();
    }

    /*!
     *  \brief resource deallocation for unit test
     */
    void SurfaceHierarchyTest::tearDown(void) {
        
        // TODO: delete the other objects created by setUp()!
    }

    /*!
    * \brief checks if the hierarchy is right
    *
    */
    void SurfaceHierarchyTest::surfaceHierarchyCreationTest() {
        std::vector<Triangle*> triangles = mClothMesh->getTriangles();
        std::vector<Triangle*>::const_iterator iter = triangles.begin();
        
        //calculates the root node from the first triangle
        const BvhNode* currentBvhNode = (*iter)->getBvHierarchyNodes().front();
        while(currentBvhNode->getParent() != 0) {
            currentBvhNode = currentBvhNode->getParent();
        }
        const BvhNode* rootBvhBode = currentBvhNode;
        
        //compares the root node with all other triangle root nodes
        for (iter = triangles.begin(); iter != triangles.end(); iter++) {
            
            currentBvhNode = (*iter)->getBvHierarchyNodes().front();
            while(currentBvhNode->getParent() != 0) {
                currentBvhNode = currentBvhNode->getParent();
            }
            CPPUNIT_ASSERT (rootBvhBode == currentBvhNode);
        }
    }
}
/*
 * vim: et sw=4 ts=4
 */

