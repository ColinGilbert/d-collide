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

#ifndef DCOLLIDE_PHYSICSSCENEBASE_H
#define DCOLLIDE_PHYSICSSCENEBASE_H

#include "scenes/scenebase.h"

#include <ode/ode.h>

class OgreSceneObjectFactory;
class MyObjectNode;

/*!
 * \brief base class for physics scenes
 * 
 * provides some additional functions and members needed
 * for the simulation with ODE
 */
class PhysicsSceneBase : public SceneBase {
public:
    PhysicsSceneBase(Ogre::Root* root);
    virtual ~PhysicsSceneBase();

    bool initScene();
    virtual inline bool getUsePhysics() const;
    
    inline dWorldID getOdeWorld() const;
    void doPhysicsStep();

    virtual void calculateCollisions(unsigned int flags);

protected:
    virtual void addInitialWorldParameters(dcollide::WorldParameters& parameters) const;

private:
    struct JointTemplate {
        dcollide::CollisionInfo& collInfo;
        dGeomID penetratingGeom;
        dGeomID penetratedGeom;
        dBodyID penetratingBody;
        dBodyID penetratedBody;
        
        JointTemplate(dcollide::CollisionInfo& infoObj) : collInfo(infoObj) {
            penetratingGeom = 0;
            penetratedGeom = 0;
            penetratingBody = 0;
            penetratedBody = 0;
        };
    };
    
    void dampMotions();

    void convertDCollideResultsToOde();
    void createOdeContactJoint(JointTemplate& jointTemplate);
    void addDampingAndFrictionParameters(dContact* contact);

protected:
    dJointGroupID mContactJoints;
    dWorldID mOdeWorld;

    /*!
     * \brief stepsize used for stepping the physics world. DO NOT MODIFY
     * 
     * 
     * The stepsize should be set once in the Scene constructor. As ODE
     * recommends to use fixed stepsizes, you should not modify that value
     * afterwards.
     */
    float mPhysicsSimStepsize;
    
    /*!
     * \brief damping factor for linear-velocity-based friction simulation
     * 
     * should be between -1 (instant damping) and 0 (no damping)
     * Used for ODE only
     */
    dReal mLinearDampingFactor;
    
    /*!
     * \brief damping factor for angular-velocity-based friction simulation
     * 
     * Should be greater between 0 (no damping) and 1(stop every rotation instantly)
     * Used for ODE only
     */
    dReal mAngularDampingFactor;

    /*!
     * \brief list of nodes with physics bodies whose movements will be damped
     * 
     * Used for ODE only
     */
    std::list<MyObjectNode*> mDampedBodyNodes;

    /*!
     * \brief Indicates whenether all contact joints should be removed from
     *        the ODE world when beginning a new physics-step
     * 
     * This flag is useful for scenes which want to test the reaction of the
     * physics simulation to pre defined collision responses.
     * 
     * Defaults to true.
     */
    bool mRemoveContactJoins;
    
    /*!
     * \brief Indicates whenether the detected collision should be filled
     *        into ODD as contact joins or not
     * 
     * This flag is useful for scenes which want to test the reaction of the
     * physics simulation to pre defined collision responses.
     * 
     * Defaults to true.
     */
    bool mAddContactJoins;
};

inline dWorldID PhysicsSceneBase::getOdeWorld() const {
    return mOdeWorld;
}

inline bool PhysicsSceneBase::getUsePhysics() const{
    return true;
}

#endif
/*
 * vim: et sw=4 ts=4
 */
