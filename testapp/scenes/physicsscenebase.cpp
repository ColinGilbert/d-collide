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

#include "physicsscenebase.h"

#include "odewrapper.h"
#include "myobjectnode.h"
#include "mydeformableobjectnode.h"

#include <ode/odewrapper.h>
#include <ode/objects.h>

#include <d-collide/worldcollisions.h>
#include <d-collide/worldparameters.h>
#include <d-collide/proxy.h>
#include <d-collide/dcollide-global.h>
#include <d-collide/debugstream.h>
#include <d-collide/shapes/shape.h>
#include <d-collide/shapes/mesh.h>
#include <d-collide/shapes/mesh/triangle.h>
#include <d-collide/shapes/mesh/vertex.h>

#include <iostream>


/*!
 * Construct a new d-collide scene with ODE physics integrated
 *
 * The scene itself is not yet created, only the necessary data
 * structures. Call \ref initializeScene to actually create a scene.
 *
 * \param root A pointer to the ogre root object. Ownership is NOT
 * taken, the pointer will not be deleted by this class.
 */
PhysicsSceneBase::PhysicsSceneBase(Ogre::Root* root) : SceneBase(root){
    mOdeWorld = 0;
    mContactJoints = 0;

    mPhysicsSimStepsize = 0.05f;
    mWorldCollisions = 0;

    mRemoveContactJoins = true;
    mAddContactJoins = true;
    
}

PhysicsSceneBase::~PhysicsSceneBase() {
    removeAllObjects();
    dJointGroupEmpty(mContactJoints);
    dWorldDestroy(mOdeWorld);
}


bool PhysicsSceneBase::initScene(){
    std::cout << dc_funcinfo << std::endl;
    mOdeWorld = dWorldCreate();
    mContactJoints = dJointGroupCreate(0);
    //use default gravity in negative z-direction
    dWorldSetGravity(getOdeWorld(), 0, 0, -10);
    
    SceneBase::initScene();
    
    return true;
}

void PhysicsSceneBase::addInitialWorldParameters(dcollide::WorldParameters& parameters) const {
    parameters.setAllowLocalCoordinates(false);
    //parameters.setRigidBoundingVolumeType(dcollide::BV_TYPE_OBB);
    //parameters.setRigidBoundingVolumeType(dcollide::BV_TYPE_SPHERE);
}

/*!
 * \brief slows down all objects in \ref mDampedBodyNodes
 * 
 * The linear and angular velocities of the objects are reduced
 * by a given factor (see \ref mLinearDampingFactor and mAngularDampingFactor)
 * 
 * FIXME: GJ this does not work as intendet
 */
void PhysicsSceneBase::dampMotions() {
    for (std::list<MyObjectNode*>::iterator iter = mDampedBodyNodes.begin();
                iter != mDampedBodyNodes.end(); ++iter) {
        dBodyID body = (*iter)->getPhysicsBody();
        //Linear damping
        const dReal* linearVelocity = dBodyGetLinearVel(body);
        dReal linLength = sqrt(linearVelocity[0] * linearVelocity[0] 
                               + linearVelocity[1]*linearVelocity[1]
                               + linearVelocity[2]*linearVelocity[2]);
        //Null velocity if very small
        if (linLength<0.001) {
            dBodySetLinearVel(body, 0,0,0);
            //std::cout << "zeroing linear velocity of body " << body << std::endl;
        }else{
            //relative velocity reduction
            dBodySetLinearVel(body,     linearVelocity[0] * (1-mLinearDampingFactor),
                                        linearVelocity[1] * (1-mLinearDampingFactor),
                                        linearVelocity[2] * (1-mLinearDampingFactor));
        }

        //Rotational damping (based on angular velocity)
        //TODO only apply this damping if the body collides with some other element
        
        const dReal* angularVelocity = dBodyGetAngularVel(body);
                    
        //compute length of angvel-vector
        dReal anglength = sqrt(angularVelocity[0] * angularVelocity[0] 
                               + angularVelocity[1]*angularVelocity[1]
                               + angularVelocity[2]*angularVelocity[2]);
        //if lenght is close to zero, null it
        //std::cout << "angular lenght absolute" << anglength <<std::endl;
        if (anglength<0.001) {
            dBodySetAngularVel(body, 0,0,0);
            //std::cout << "zeroing angular velocity of body " << body << std::endl;
        }else{
            //relative velocity reduction
            dBodySetAngularVel(body,    angularVelocity[0] * (1-mAngularDampingFactor),
                                        angularVelocity[1] * (1-mAngularDampingFactor),
                                        angularVelocity[2] * (1-mAngularDampingFactor));
        }
        
    }
}


/*!
 * \brief calculates a physics-simulation step in ODE
 *
 *
 */
void PhysicsSceneBase::doPhysicsStep() {
    //dcollide::debug() << "physics step";
    //GJ: disabled as it does not work as intended for now
    dampMotions();
    if (mRemoveContactJoins) {
        dJointGroupEmpty(mContactJoints);
    }

    if (mAddContactJoins) {
        convertDCollideResultsToOde();
    }

    //Disable automatic bv-hierarchy updates for deformable objects
    std::list<dcollide::Proxy*> deformableObjects = mCollisionWorld->getDeformableProxies();
    
    for(std::list<dcollide::Proxy*>::iterator i = deformableObjects.begin(); i != deformableObjects.end(); ++i) {
    	(*i)->setBvUpdatesEnabled(false);
    }
    
    //dWorldStep() : the second parameter is the number of seconds in simulation time
    //NOTE: the stepsize must not change while the simulation is running
    //std::cout << "stepping the physics world. \n---------"<< std::endl;
    dWorldQuickStep (mOdeWorld, mPhysicsSimStepsize);

    //Disable automatic bv-hierarchy updates for deformable objects
       
    //Reenable automatic updates. A rigid-bv-hierarchy update will be triggered by this call
    for(std::list<dcollide::Proxy*>::iterator i = deformableObjects.begin(); i != deformableObjects.end(); ++i) {
    	(*i)->setBvUpdatesEnabled(true);
    }
    
    /*
    for (std::list<dcollide::Proxy*>::const_iterator iter = mCollisionWorld->getTopLevelProxies().begin();
                                            iter != mCollisionWorld->getTopLevelProxies().end(); ++iter){
        //Print AABBs
        const dcollide::BoundingVolume* bv = (*iter)->getBvHierarchyNode()->getBoundingVolume();
        if (bv) {
            std::cout << "BV size of proxy* " << (*iter) << " is " << bv->getSurroundingAabbExtents() <<std::endl;
        }
    }
    */
}

void PhysicsSceneBase::convertDCollideResultsToOde() {
    //detection of the collisions in the current state has already been done
    //now we need to feed ODE with these collisions and take a step

    if (mWorldCollisions) {

        // only when there are collisions we need to go on
        // read the collsionInfo
        for (std::list<dcollide::CollisionInfo>::const_iterator collIter = mWorldCollisions->getNarrowPhaseCollisions().begin();
             collIter != mWorldCollisions->getNarrowPhaseCollisions().end();
             collIter++) {

            dcollide::CollisionInfo collision = *collIter;

            dcollide::Vertex*   penetratingVertex   = collision.penetratingVertex;

            dcollide::Proxy* penetratedProxy  = collision.penetratedProxy;
            dcollide::Proxy* penetratingProxy = collision.penetratingProxy;

            MyObjectNode* penetratedNode  = (MyObjectNode*) penetratedProxy->mUser1;
            MyObjectNode* penetratingNode = (MyObjectNode*) penetratingProxy->mUser1;

            JointTemplate joint(collision);
            
            //Set penetrating Body
            //   rigid     : simply use body given by the penetrating node
            //   deformable: use the mass particle of the penetrating vertex
            //               and overwrite penetrating geom
            if (penetratingProxy->getProxyType() & dcollide::PROXYTYPE_DEFORMABLE) {
                //penetrating is deformable, check types
                if (! (dynamic_cast<MyDeformableObjectNode*>(penetratingNode))){
                    dcollide::error() << "collision contained a deformable proxy with no corresponding MyDeformableObjectNode!";
                }
                if (!penetratingVertex) {
                    throw dcollide::NullPointerException("Vertex* penetratingVertex");
                }
                joint.penetratingGeom
                    = ((MyDeformableObjectNode*) penetratingNode)
                          ->getOdeGeom(penetratingVertex->getVertexIndex());

                // Get penetrated/penetrating bodies and apply them
                joint.penetratingBody
                        = ((MyDeformableObjectNode*) penetratingNode)
                           ->getPhysicsBody(penetratingVertex->getVertexIndex());
            } else {
                joint.penetratingBody = penetratingNode->getPhysicsBody();
            }
            
            //Set penetrated Body
            //   rigid     : simply use body given by the penetrated node
            //   deformable: create collision with static environment instead
            if (penetratedProxy->getProxyType() & dcollide::PROXYTYPE_DEFORMABLE) {
                //create collision with static environment
                //(as we have no penetratedTriangle)
                joint.penetratedBody = 0;
                //joint.penetratedGeom = 0;
            } else {
                joint.penetratedBody = penetratedNode->getPhysicsBody();
            }
            // Dispose the actual creation of the contact joints
            createOdeContactJoint(joint);
        }
    }
}

void PhysicsSceneBase::createOdeContactJoint(JointTemplate& jointTemplate) {

    // Create empty contact joint
    dContact* contact = new dContact();

    // Fills in the generic information about the collision
    contact->geom = convertCollisionInfo(jointTemplate.collInfo);

    // Add damping and friction model
    addDampingAndFrictionParameters(contact);

    // Overwrite ODE geoms if claimed by the template
    if (jointTemplate.penetratingGeom) {
        contact->geom.g1 = jointTemplate.penetratingGeom;
    }
    if (jointTemplate.penetratedGeom) {
        contact->geom.g2 = jointTemplate.penetratedGeom;
    }

    // Create the actual joint and apply it to the physics world
    dJointID joint = dJointCreateContact(mOdeWorld,
                                         mContactJoints,
                                         contact);
    dJointAttach(joint, jointTemplate.penetratingBody,
                        jointTemplate.penetratedBody);

}

void PhysicsSceneBase::addDampingAndFrictionParameters(dContact* contact) {

    /* -------------- specify joint properties -------------- */
    // See http://opende.sourceforge.net/wiki/index.php/Manual_%28Joint_Types_and_Functions%29
    // TODO: Each scene might want to set these values
    //       differently, maybe object-dependent.
    
    // Use an advanced friction model
    contact->surface.mode |= dContactApprox1;
    
    // Set friction coefficient
    // GJ: This value is guessed! Doc says "use some appropriate
    //     value between 0 and infinity".
    contact->surface.mu = 1;
    
    // Make all contacts somewhat bouncy
    contact->surface.mode |= dContactBounce;
    
    //surface.bounce must be between 0 and 1
    contact->surface.bounce = (dReal) 0.2;
    
    //minimum velocity for bouncing
    contact->surface.bounce_vel = (dReal) 0.1;
}


struct OdeCollisionData {
    dWorldID mOdeWorld;
    dJointGroupID mContactJoints;
};

void PhysicsSceneBase::calculateCollisions(unsigned int flags) {
    SceneBase::calculateCollisions(flags);
}

/*
 * vim: et sw=4 ts=4
 */
