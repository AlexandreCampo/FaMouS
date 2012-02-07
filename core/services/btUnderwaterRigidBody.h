/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2012 Alexandre Campo                                 */
/*                                                                            */
/*    This file is part of FaMouS  (a fast, modular and simple simulator).    */
/*                                                                            */
/*    FaMouS is free software: you can redistribute it and/or modify          */
/*    it under the terms of the GNU General Public License as published by    */
/*    the Free Software Foundation, either version 3 of the License, or       */
/*    (at your option) any later version.                                     */
/*                                                                            */
/*    FaMouS is distributed in the hope that it will be useful,               */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           */
/*    GNU General Public License for more details.                            */
/*                                                                            */
/*    You should have received a copy of the GNU General Public License       */
/*    along with FaMouS.  If not, see <http://www.gnu.org/licenses/>.         */
/*----------------------------------------------------------------------------*/

// IMPORTANT INFORMATION 
//
// To speed things up we make compromises and assumptions. Make sure you read the following :
//
// this extends btRigidBody of bullet, at the expense of virtual methods.
// added mass is not implemented at all
// momentum is damped (same coefficient in all dimensions), 
// drag is applied to linear velocity (3 different coefficients for 3 dimensions)
// quadratic drag is not implemented to speed things up
// drag calculation is timestep size dependent...
// timestep must be <= 1.0 (drag calculation, no check ...)


#ifndef BT_UNDERWATER_RIGID_BODY_H
#define BT_UNDERWATER_RIGID_BODY_H

#include "BulletDynamics/Dynamics/btRigidBody.h"

#include <iostream>

// Derivation for underwater bodies
struct	btUnderwaterRigidBodyConstructionInfo : public btRigidBody::btRigidBodyConstructionInfo
{
    btScalar m_fluidDensity;
    btVector3 m_centerOfVolume;
    btScalar m_volume;
    btVector3 m_gravity;

    btUnderwaterRigidBodyConstructionInfo (btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia=btVector3(0,0,0), btVector3 centerOfVolume = btVector3 (0,0,0), btScalar fluidDensity = btScalar(1000), btScalar volume = btScalar(1.0)) 
	: btRigidBody::btRigidBodyConstructionInfo (mass, motionState, collisionShape, localInertia)
    {
	
	m_fluidDensity = fluidDensity;
	m_centerOfVolume = centerOfVolume;
	m_volume = volume;
    }
};


class btUnderwaterRigidBody : public btRigidBody
{
public:
    
    // be careful ... lin vel and ang vel are expressed in world coords. uvw pqr are the 6 dof in local coords
    btScalar u, v, w, p, q, r;

    btVector3 m_buoyancyCenter;
    btVector3 m_buoyancy;
    btScalar m_fluidDensity;
    btScalar m_volume;
    
    btVector3 m_linearDrag;
    btVector3 m_angularDrag;
    btVector3 m_linearQuadraticDrag;
    btVector3 m_angularQuadraticDrag;

    btVector3 m_centerOfVolume;

    btVector3 m_linearAddedMass;
    btVector3 m_angularAddedMass;
    
    btVector3 m_inertia;
    /* btMatrix3x3 m_invInertiaTensorWorldLinear; */
    /* btMatrix3x3 m_invInertiaTensorWorldAngular; */
    

    btUnderwaterRigidBody (const btUnderwaterRigidBodyConstructionInfo& constructionInfo);
    void setAddedMass (const btVector3& linear, const btVector3& angular);
    void setCenterOfVolume(const btVector3& position);
    void setDragCoefficients (const btVector3& linear, const btVector3& angular);
    void setDragQuadraticCoefficients (const btVector3& qlinear, const btVector3& qangular);
    void setVolume (float v);
    void updateInertiaTensor ();
    void applyGravity ();
    void integrateVelocities (btScalar step);
    void applyDamping (btScalar timeStep);
};

#endif
