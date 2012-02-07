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

#include "Floor.h"
#include "Simulator.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btDefaultMotionState.h"

#include <GL/glut.h> 
#include <GL/gl.h>   
#include <GL/glu.h>  

#include <iostream>


Floor::Floor() :
    Object(),
    PhysicsBulletInterface(),
    RenderOpenGLInterface()
{
}


Floor::~Floor()
{

}

void Floor::Draw (RenderOpenGL* r)
{
    // drawing depends on existence in physics bullet... 
    if (body != 0)
    {
	dsRotationMatrix rotmat;
	dRotation2dsRotationMatrix(0, rotmat);
        
	btScalar m[16];
	btMatrix3x3 rot;
	rot.setIdentity();

	btCollisionObject* colObj = static_cast<btCollisionObject*>(body);
	if(body->getMotionState())
	{
	    btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
	    myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
	}
	else
	{
	    colObj->getWorldTransform().getOpenGLMatrix(m);
	}
		
	glPushMatrix(); 

#if defined(BT_USE_DOUBLE_PRECISION)
	glMultMatrixd(m);
#else
	glMultMatrixf(m);
#endif

	r->dsSetColor (0.3, 0.3, 0.3);
	r->dsSetTexture (0);
	float position [3];
	position[0] = 0;
	position[1] = 0;
	position[2] = 0;

	r->dsDrawBox((float*) &position, (float*) &rotmat, (float*) &dimensions);

	glPopMatrix();
    }    
}


void Floor::Register(PhysicsBullet* p)
{
    dimensions[0] = 1.0 * p->scalingFactor;
    dimensions[1] = 1.0 * p->scalingFactor;
    dimensions[2] = 0.05 * p->scalingFactor;

    // define collision type
    SetCollisionType (1<<1);

    ///create a few basic rigid bodies
    shape = new btBoxShape(btVector3(
			       dimensions[0] / 2.0,
			       dimensions[1] / 2.0,
			       dimensions[2] / 2.0
			       ));
    p->m_collisionShapes.push_back(shape);
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0.0, 0.0, -dimensions[2] / 2.0));
    
    btScalar mass(0.0);
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0,0,0);
    if (isDynamic)
    {
	shape->calculateLocalInertia(mass,localInertia);
    }
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    body = new btRigidBody(rbInfo);
    body->setUserPointer((void*)(this));
    
    //add the body to the dynamics world
    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);
}

void Floor::Unregister(PhysicsBullet* p)
{
    
}
