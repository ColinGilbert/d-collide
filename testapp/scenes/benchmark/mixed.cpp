/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *                                                                             *
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

#include "mixed.h"
#include "myobjectnode.h"

#include <d-collide/world.h>
#include <d-collide/math/vector.h>
#include <d-collide/math/matrix.h>
#include <d-collide/shapes/mesh/triangle.h>
#include <d-collide/shapes/mesh/vertex.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/detectordeform/surfacehierarchy/surfacehierarchy.h>
#include <d-collide/detectordeform/spatialhash/spatialhash.h>
#include <d-collide/detectordeform/detectordeformmanager.h>

#include <modelloader/loader3ds.h>


dcollide::Mesh* MixedBenchmark::createTetraeder() {
        //Mesh-Tetraeder
/*
        3 \
       / \ \
      /  \  \
     /  --\---2
    0----  \ /
       -----1
*/
    using namespace dcollide;
    Vertex* v0 = new Vertex(Vector3(0, 0, 0));
    Vertex* v1 = new Vertex(Vector3(75, 0, 0));
    Vertex* v2 = new Vertex(Vector3(50, 50, 0));
    Vertex* v3 = new Vertex(Vector3(50, 50, 75));

    Triangle* t0 = new Triangle(v0, v2, v1);
    Triangle* t1 = new Triangle(v0, v1, v3);
    Triangle* t2 = new Triangle(v1, v2, v3);
    Triangle* t3 = new Triangle(v2, v0, v3);

    std::vector<Vertex*> vertices;
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);

    std::vector<Triangle*> triangles;
    triangles.push_back(t0);
    triangles.push_back(t1);
    triangles.push_back(t2);
    triangles.push_back(t3);

    return new Mesh(vertices, triangles);
}

MixedBenchmark::MixedBenchmark(Ogre::Root* root) : SceneBase(root) {
    
}
MixedBenchmark::~MixedBenchmark() {
    
}

std::string MixedBenchmark::getSceneDescription() const {
    return "Test of mixed rigid and deformable object collisions without physic simulation";
}

dcollide::Vector3 MixedBenchmark::initialWorldDimension() const {
    return dcollide::Vector3(2000.0f,2000.0f,2000.0f);    
}

void MixedBenchmark::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;
    //algorithm.push_back(DEFORMABLE_TYPE_BV_HIERARCHY);
    //algorithm.push_back(DEFORMABLE_TYPE_SPATIAL_HASH);
    algorithm.push_back(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);

    parameters.setDeformableAlgorithms(algorithm);
    parameters.setWorkerThreadCount(4);
}

bool MixedBenchmark::initializeScene() {
    //ModelLoader::Loader3ds loader;

    ModelLoader::Loader3ds loader;
    
    //initialize pseudo-random-number generator with reproducable seed
    int seed = 12345;
    srand(seed);
    //usage: 
    //int randomValue = rand() % 100 + 1; //Generates value between 1 and 100


    std::cout << "generating objects" << std::endl;
    //generate 200 objects of each type
    for (int i = 0; i<100; i++) {
        //Box 10x5x15
        MyObjectNode* boxNode = new MyObjectNode(getCollisionWorld(),
                                                 new dcollide::Box(50, 25, 75),
                                                 dcollide::PROXYTYPE_RIGID);
        mAllElements.push_back(boxNode);

        //Sphere radius 8
        MyObjectNode* sphereNode = new MyObjectNode(getCollisionWorld(),
                                                    new dcollide::Sphere(40),
                                                    dcollide::PROXYTYPE_RIGID);
        mAllElements.push_back(sphereNode);

        //Mesh (tetraeder)
        MyObjectNode* meshNode = new MyObjectNode(getCollisionWorld(),
                                                  createTetraeder(),
                                                  dcollide::PROXYTYPE_RIGID);
        mAllElements.push_back(meshNode);
    }

    for (int i = 0; i<10; i++) {
        //Helicopter Model
        dcollide::Mesh* helicopterMesh = loader.loadFromFileToOneMesh("puma.3ds");
        if (helicopterMesh) {
            dcollide::Proxy* helicopterProxy = getCollisionWorld()->createProxy(helicopterMesh, dcollide::PROXYTYPE_RIGID);
            MyObjectNode* helicopterNode = new MyObjectNode(helicopterProxy);
            mAllElements.push_back(helicopterNode);
        } else {
            std::cerr << "Couldn't load helicopter model!" << std::endl;
        }
    }

    for (int i = 0; i<2; i++) {
        //Helicopter Model
        dcollide::Mesh* helicopterMesh = loader.loadFromFileToOneMesh("puma.3ds");
        if (helicopterMesh) {
            dcollide::Proxy* helicopterProxy = getCollisionWorld()->createProxy(helicopterMesh, (dcollide::ProxyTypes)(dcollide::PROXYTYPE_DEFORMABLE | dcollide::PROXYTYPE_CLOSEDHULL | dcollide::PROXYTYPE_SELFCOLLIDABLE));
            MyObjectNode* helicopterNode = new MyObjectNode(helicopterProxy);
            mAllElements.push_back(helicopterNode);
        } else {
            std::cerr << "Couldn't load helicopter model!" << std::endl;
        }
    }
    
    
    std::cout << "placing objects in the world" << std::endl;
    //random initial placement and orientation of the proxies
    for (std::vector<MyObjectNode*>::iterator proxyIter = mAllElements.begin();
         proxyIter != mAllElements.end();
         proxyIter++) {

        MyObjectNode* element = *proxyIter;

        dcollide::Matrix rotation;
        //random rotation around x,y,and z
        rotation.rotate(rand() % 360, 1, 0, 0);
        rotation.rotate(rand() % 360, 0, 1, 0);
        rotation.rotate(rand() % 360, 0, 0, 1);
        element->setRotation(rotation);

        element->translate(rand() % 1000, rand() % 1000, rand() % 1000);

        addTopLevelObject(element);
    }

    return true;
}

void MixedBenchmark::startNextSceneFrame() {
    //perform some steps: move/rotate all objects and get all collisions
    int maxRotation = 10; //max rotation per axis
    int maxTranslation = 10; //max translation per axis
    for (std::vector<MyObjectNode*>::iterator proxyIter = mAllElements.begin();
         proxyIter != mAllElements.end();
         proxyIter++) {

        MyObjectNode* element = *proxyIter;

        dcollide::Matrix rotation = element->getRotation();
        //random rotation around x,y,and z
        rotation.rotate(rand() % maxRotation - maxRotation/2, 1, 0, 0);
        rotation.rotate(rand() % maxRotation - maxRotation/2, 0, 1, 0);
        rotation.rotate(rand() % maxRotation - maxRotation/2, 0, 0, 1);
        element->setRotation(rotation);

        element->translate( rand() % maxTranslation - maxTranslation/2,
                            rand() % maxTranslation - maxTranslation/2,
                            rand() % maxTranslation - maxTranslation/2);
    }
}
