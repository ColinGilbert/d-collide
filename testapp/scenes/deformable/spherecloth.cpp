/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,         *
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

#include "spherecloth.h"
#include "dcollide-config_testapp.h"
#include "myobjectnode.h"

#include <modelloader/loaderdummy.h>

#include <d-collide/debug.h>
#include <d-collide/shapes/mesh.h>
#include <d-collide/shapes/sphere.h>
#include <d-collide/shapes/box.h>

using namespace dcollide;

SphereCloth::SphereCloth(Ogre::Root* root)
        : PhysicsSceneBase(root) {
}

SphereCloth::~SphereCloth() {
}

dcollide::Vector3 SphereCloth::initialWorldDimension() const {
    return dcollide::Vector3(2048.0, 2048.0, 2048.0);
}

void SphereCloth::addSceneSpecificWorldParameters(dcollide::WorldParameters& parameters) const {
    std::list<dcollide::DeformableAlgorithmType> algorithm;
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_BV_HIERARCHY);
    //algorithm.push_back(dcollide::DEFORMABLE_TYPE_SPATIAL_HASH);
    algorithm.push_back(dcollide::DEFORMABLE_TYPE_SURFACE_HIERARCHY);
    parameters.setDeformableAlgorithms(algorithm);
}

Mesh* SphereCloth::createClothMesh(real sidelength, int xtiles, int ytiles) {

    //Step one: Build vertices of the cloth
    //Vertices will be arrangend in a regular grid with a total sidelength
    //given by the parameter.

    std::vector<dcollide::Vertex*>   vertices;

    //Center the cloth in local coordinates
    real xoffset = -(xtiles * sidelength / 2 );
    real yoffset = -(ytiles * sidelength / 2 );
    Vector3 position(xoffset,yoffset,0);

    real step = sidelength;

    for(int y = 0; y < ytiles; y++ ) {
        for(int x = 0; x < xtiles; x++) {
            //Add vertice
            position.set(x*step + xoffset,y*step + yoffset,0);
            vertices.push_back(new dcollide::Vertex(position));
        }
    }

    //Step two: build triangles of the cloth
    std::vector<dcollide::Triangle*> triangles;

    for(int y = 0; y < ytiles - 1; y++ ) {
        for(int x = 0; x < xtiles - 1; x++) {
            //Build first triangle
            triangles.push_back(
                new Triangle(vertices[ gridCoord(x+1,y  ,xtiles) ],
                             vertices[ gridCoord(x  ,y  ,xtiles) ],
                             vertices[ gridCoord(x  ,y+1,xtiles) ]));

            //Build second triangle
            triangles.push_back(
                new Triangle(vertices[ gridCoord(x+1,y+1,xtiles) ],
                             vertices[ gridCoord(x+1,y  ,xtiles) ],
                             vertices[ gridCoord(x  ,y+1,xtiles) ]));
        }
    }

    return new Mesh(vertices, triangles);
}

int SphereCloth::gridCoord(int x, int y, int resolution) {
    return y * resolution + x;
}
bool SphereCloth::initializeScene() {
    //Define the physic attributes of our scene
    dWorldSetGravity(getOdeWorld(), 0, 0, -7.5);
   mPhysicsSimStepsize = 0.03;
   mLinearDampingFactor = 0.001;
   mAngularDampingFactor = 0.005;

    mCloth = new MyDeformableObjectNode(getCollisionWorld(),
                                                   createClothMesh(3,15,40),
                                                   dcollide::PROXYTYPE_DEFORMABLE |
                                                   dcollide::PROXYTYPE_SELFCOLLIDABLE,
                                                   false);

    mCloth->createPhysicsBody(mOdeWorld, 25);
    //rotate cloth so that the faces look down
    //FIXME this causes the cloth to crumble (See ticket #368)
    //      GJ: as a workaround, i directly changed the triangles.
    //mCloth->rotate(180, 1,0,0);
    addTopLevelObject(mCloth);
    mCloth->translate(0, -20, 0);

    MyObjectNode* environment = new MyObjectNode(getCollisionWorld(), 0, dcollide::PROXYTYPE_FIXED);
    for(int x = 0; x < 15; x++) {
            int y = 0;
            MyObjectNode* box = new MyObjectNode (getCollisionWorld(),
                        new dcollide::Box(6.0, 6.0, 6.0),
                        //new dcollide::Sphere(3),
                        dcollide::PROXYTYPE_FIXED,true, false);
            box->setPosition(x*6.0-25,y*6.0-25,-100);
            environment->addChild(box);
    }
    addTopLevelObject(environment);

    const_cast<ModelLoader::TextureInformation&> (mCloth
                        ->getTextureInformation()).setTransparent(0.5);
    return true;
}

void SphereCloth::startNextSceneFrame() {
}

void SphereCloth::restart() {
    removeObject(mCloth);

    mCloth = new MyDeformableObjectNode(getCollisionWorld(),
                                                   createClothMesh(3,15,40),
                                                   dcollide::PROXYTYPE_DEFORMABLE |
                                                   dcollide::PROXYTYPE_SELFCOLLIDABLE,
                                                   false);

    mCloth->createPhysicsBody(mOdeWorld, 25);

    addTopLevelObject(mCloth);
    deleteCollisions();
}

std::string SphereCloth::getSceneDescription() const {
      return "SelfCollision Test";
}

