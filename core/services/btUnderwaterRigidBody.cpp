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

#include "btUnderwaterRigidBody.h"

#define BTUNDERWATERRIGIDBODY_3D_DRAG

btUnderwaterRigidBody::btUnderwaterRigidBody (const btUnderwaterRigidBodyConstructionInfo& constructionInfo) :
    btRigidBody (constructionInfo)
    {
	m_fluidDensity = constructionInfo.m_fluidDensity;
	m_inertia = constructionInfo.m_localInertia;
	m_centerOfVolume = constructionInfo.m_centerOfVolume;
	setGravity (constructionInfo.m_gravity);
	setVolume (constructionInfo.m_volume);
	setDragCoefficients (btVector3 (0,0,0), btVector3 (0,0,0));
	setAddedMass (btVector3 (0,0,0), btVector3 (0,0,0));
	updateInertiaTensor();
    }

// the tensor is symmetric
// if the body has 3 planes of symmetry, the tensor becomes diagonal
// since this assumption is made with inertia tensor, we also use it here...
void btUnderwaterRigidBody::setAddedMass (const btVector3& linear, const btVector3& angular)
{
    m_linearAddedMass = linear;
    m_angularAddedMass = angular;
    updateInertiaTensor ();
}

// compund shape is the other way, but I want to make faster calculations
void btUnderwaterRigidBody::setCenterOfVolume(const btVector3& position) 
{
    m_centerOfVolume = position;
    updateInertiaTensor ();
}

// drag coefficients must be set as 
// linear part : area exposed * coefficient
// angular part : coefficient only
void btUnderwaterRigidBody::setDragCoefficients (const btVector3& linear, const btVector3& angular)
{
    m_linearDrag = btScalar (-1.0) * linear;
    m_angularDrag = btScalar (-1.0) * angular;
    updateInertiaTensor ();
}

void btUnderwaterRigidBody::setDragQuadraticCoefficients (const btVector3& qlinear, const btVector3& qangular)
{
    m_linearQuadraticDrag = btScalar (-0.5) * m_fluidDensity * qlinear;
    m_angularQuadraticDrag = btScalar (-0.5) * m_fluidDensity * qangular;
    updateInertiaTensor ();
}

void btUnderwaterRigidBody::setVolume (btScalar volume)
{
    m_buoyancy =  m_gravity_acceleration * volume * m_fluidDensity * -1.0;
    m_volume = volume;
}

void btUnderwaterRigidBody::updateInertiaTensor() 
{
    m_invInertiaTensorWorld = m_worldTransform.getBasis().scaled(m_invInertiaLocal) * m_worldTransform.getBasis().transpose();
}

// void btUnderwaterRigidBody::updateInertiaTensor() 
// {
//     // calculate linear part : m + added mass
//     btScalar m = btScalar(1.0) / m_inverseMass;
    
//     btVector3 mlin (
// 	1.0 / (m_linearAddedMass.x() + m),
// 	1.0 / (m_linearAddedMass.y() + m),
// 	1.0 / (m_linearAddedMass.z() + m));

//     btMatrix3x3 inertia (
// 	m_inertia.x() + m_angularAddedMass.x(),  0,                                          0,
// 	0,                                       m_inertia.y() + m_angularAddedMass.y(),     0,
// 	0,                                       0,                                          m_inertia.z() + m_angularAddedMass.z());
    
//     // TODO : not dealing correctly with added mass here ...
//     const btVector3& v = m_centerOfVolume;
//     btMatrix3x3 inertiaTranslator (
// 	m * (v.y() * v.y() + v.z() * v.z()),        -m * v.x() * v.y(),                          -m * v.x() * v.z(),
// 	-m * v.x() * v.y(),                           m * (v.x() * v.x() + v.z() * v.z()),        -m * v.y() * v.z(),
// 	-m * v.x() * v.z(),                          -m * v.y() * v.z(),                           m * (v.x() * v.x() + v.y() * v.y())
// 	);
    
//     btMatrix3x3 i = inertia + inertiaTranslator;	
//     btMatrix3x3 invInertiaLocal;
    
//     if (i.getColumn(0).x() != btScalar(0.0)) invInertiaLocal.getColumn(0).setX (1.0 / i.getColumn(0).x());
//     if (i.getColumn(0).y() != btScalar(0.0)) invInertiaLocal.getColumn(0).setY (1.0 / i.getColumn(0).y());
//     if (i.getColumn(0).z() != btScalar(0.0)) invInertiaLocal.getColumn(0).setZ (1.0 / i.getColumn(0).z());
//     if (i.getColumn(1).x() != btScalar(0.0)) invInertiaLocal.getColumn(1).setX (1.0 / i.getColumn(1).x());
//     if (i.getColumn(1).y() != btScalar(0.0)) invInertiaLocal.getColumn(1).setY (1.0 / i.getColumn(1).y());
//     if (i.getColumn(1).z() != btScalar(0.0)) invInertiaLocal.getColumn(1).setZ (1.0 / i.getColumn(1).z());
//     if (i.getColumn(2).x() != btScalar(0.0)) invInertiaLocal.getColumn(2).setX (1.0 / i.getColumn(2).x());
//     if (i.getColumn(2).y() != btScalar(0.0)) invInertiaLocal.getColumn(2).setY (1.0 / i.getColumn(2).y());
//     if (i.getColumn(2).z() != btScalar(0.0)) invInertiaLocal.getColumn(2).setZ (1.0 / i.getColumn(2).z());
    
    
// //	m_invInertiaTensorWorldLinear = m_worldTransform.getBasis().scaled(mlin) * m_worldTransform.getBasis().transpose();
// //	m_invInertiaTensorWorldAngular = m_worldTransform.getBasis() * invInertiaLocal * m_worldTransform.getBasis().transpose();
//     m_invInertiaTensorWorld = m_worldTransform.getBasis() * invInertiaLocal * m_worldTransform.getBasis().transpose();
// }

// gravity is now applied with all forces, for clarity and correctness, I prefer to centralize computation of motion
void btUnderwaterRigidBody::applyGravity()
{
}

void btUnderwaterRigidBody::integrateVelocities(btScalar step) 
{
    if (isStaticOrKinematicObject())
	return;
    
    // TODO : this model is far from accurate
    // TODO : drag forces are not realistically computed (missing quadratic forces), 
    //        but I just want smth fast, and the approximation is valid for slow speeds...
    // TODO : in general added mass is not considered here
    // TODO : transformations from local to global are costly, tried to avoid them, but did some for 3d drag
    // TODO : drag on angular velocity is a single same coefficient (damping) for all dimensions, cause transfo is costly
    
        
    const btMatrix3x3& worldBasis = m_worldTransform.getBasis();
    btVector3 localBuoyancyForce = worldBasis * m_buoyancy;
    btVector3 buoyancyTorque = m_centerOfVolume.cross(localBuoyancyForce * m_linearFactor);
    
#ifdef BTUNDERWATERRIGIDBODY_3D_DRAG
    /* btMatrix3x3 worldToLocal ( */
    /*     1,     0,                            -sin(m_angularVelocity.y()),  */
    /*     0,     cos(m_angularVelocity.x()),    sin(m_angularVelocity.x()) * cos(m_angularVelocity.y()),  */
    /*     0,    -sin(m_angularVelocity.x()),    cos(m_angularVelocity.x()) * cos(m_angularVelocity.y()) */
    /*     ); */
    /* btVector3 localAngularVelocity = worldToLocal * m_angularVelocity; */
    
    // WARNING : 
    // ok this is not implemented cause it sucks and consumes too much cpu.
    // the trick here is to get the euler angles of the body, so that we can translate the angular velocity
    // in local frame. it is costly to calculate these angles then local velocity, only to apply some damping, 
    // then bring it back to global coords. For the moment I leave it, and use damping.
    // btVector3 localAngularVelocity (
    //     m_angularVelocity.x() - m_angularVelocity.z() * sin (euler),
    //     m_angularVelocity.y() * cos (euler) 
    // 	    + m_angularVelocity.z() * sin (euler) * cos (euler),
    //     - m_angularVelocity.y() * sin (euler) 
    // 	    + m_angularVelocity.z() * cos (euler) * cos (euler)
    //     );

    // calculate projected area on the plane perpendicular to velocity
    // WARNING : to go faster, do not use drag computations in local frame, rather rely on damping from bullet.
    // WARNING : local frame calculations allow different drag coefficients in 3d
    btVector3 localLinearVelocity = worldBasis.transpose() * m_linearVelocity;
    // btVector3 d = m_linearQuadraticDrag * localLinearVelocity.absolute();
    btVector3 dragForce = localLinearVelocity * m_linearDrag; // + (d.x() + d.y() + d.z()) * localLinearVelocity ; // removed quadratic terms: useless unless high speed
    // btVector3 dragTorque = m_angularVelocity * m_angularDrag; // removed quadratic terms: useless unless high rotational speed
        
    // transform drag force to apply it against motion
    dragForce = worldBasis * dragForce;
    
    btVector3 forces = m_totalForce + m_buoyancy + m_gravity + dragForce; 
    btVector3 torques = m_totalTorque + buoyancyTorque; // + dragTorque;

#else
    btVector3 forces = m_totalForce + m_buoyancy + m_gravity; 
    btVector3 torques = m_totalTorque + buoyancyTorque;
#endif
    
    // TODO added mass is not considered here
    m_linearVelocity +=  forces * m_inverseMass * step;
    m_angularVelocity += m_invInertiaTensorWorld * torques * step;
}


// for underwater, we apply drag here 
// as explained in bullet forums, this is not accurate, and makes simulation depend on timestep size....
// applyDamping damps the velocity, using the given m_linearDamping and m_angularDamping
void btUnderwaterRigidBody::applyDamping(btScalar timeStep)
{
    // assume timestep is less than 1...
    // either use drag in local coords, or damping in global coords
#ifndef BTUNDERWATERRIGIDBODY_3D_DRAG
    m_linearVelocity -= m_linearVelocity * m_linearDamping * timeStep;
    m_angularVelocity -= m_angularVelocity * m_angularDamping * timeStep;
#else
    m_angularVelocity -= m_angularVelocity * m_angularDamping * timeStep;
#endif
}
