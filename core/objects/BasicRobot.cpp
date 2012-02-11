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

#include "BasicRobot.h"
#include "Simulator.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

#include <GL/glut.h> 
#include <GL/gl.h>   
#include <GL/glu.h>  

#include <iostream>


BasicRobot::BasicRobot() :
    Object(),
    PhysicsBulletInterface(),
    RenderOpenGLInterface()
{
    // define collision type
    SetCollisionType (1 << 2);
    SetCollisionFilter (0x7FFFFFFF);
} 


BasicRobot::~BasicRobot()
{

}

void BasicRobot::AddDevices (PhysicsBullet* p) 
{
   magicForce = new DeviceMagicForce(body);
   Add (magicForce);
   
   // btTransform t;
   // t.setIdentity();    
   // int cf = this->collisionType;
   // int ct = (1 << 3);
   // proximitySensor = new DeviceProximitySensor(p, body, t, cf, ct);
   // Add (proximitySensor);
}

void BasicRobot::Draw (RenderOpenGL* r)
{
    // drawing depends on existence in physics bullet... 
    if (body != 0)
    {
	btScalar m[16];
	btMatrix3x3 rot;
	rot.setIdentity();
	
	btTransform t = body->getCenterOfMassTransform();
	t.getOpenGLMatrix(m);
        
	glPushMatrix(); 
	glMultMatrixf(m);
	
	dsRotationMatrix rotmat;
	float pos[3];	
	pos[0] = 0;
	pos[1] = 0;
	pos[2] = 0;
       
	dRotation2dsRotationMatrix(0, rotmat);

	r->dsSetColor (1, 0, 0);
	r->dsSetTexture (0);
	r->dsDrawCylinder((float*) &pos, (float*) &rotmat, height, radius);

	glPopMatrix();
    }
}


void BasicRobot::Register (PhysicsBullet* p)
{
    height = 0.02 * p->scalingFactor;
    radius = 0.035 * p->scalingFactor;

    //create a few dynamic rigidbodies
    // Re-using the same collision is better for memory usage and performance    
    shape = new btCylinderShapeZ(btVector3(radius, radius, height / 2.0));
    p->m_collisionShapes.push_back(shape);
    
    btScalar mass(0.1);    
    btVector3 localInertia(0.0, 0.0, 0.0);
    shape->calculateLocalInertia(mass,localInertia);    
 
    btTransform startTransform;
    startTransform.setIdentity();
		
    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    rbInfo.m_friction = 0.05;    
    rbInfo.m_linearDamping = 0.05;
    rbInfo.m_angularDamping = 0.05;
    body = new btRigidBody(rbInfo);    
    body->setGravity(btVector3 (0, 0, -9.81  * p->scalingFactor));
    body->setUserPointer((void*)(this));
    
    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);

    AddDevices(p);
}

void BasicRobot::Unregister (PhysicsBullet* p)
{
}

// very unfortunate... I consider this as a c++ bug : 
// overloading inherited methods hides base methods even though signature is different
void BasicRobot::Register (RenderOpenGL* r)
{
    RenderOpenGLInterface::Register(r);
}

void BasicRobot::Unregister (RenderOpenGL* r)
{
    RenderOpenGLInterface::Unregister(r);
}


void BasicRobot::SetPosition (float x, float y, float z)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setOrigin(btVector3(x, y, z));
    body->setCenterOfMassTransform(t);
}

void BasicRobot::SetRotation (float qx, float qy, float qz, float qa)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setRotation(btQuaternion(btVector3 (qx, qy, qz), qa));
    body->setCenterOfMassTransform(t);
}
