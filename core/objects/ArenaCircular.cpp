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

#include "ArenaCircular.h"
#include "Simulator.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btDefaultMotionState.h"

#include <GL/glut.h> 
#include <GL/gl.h>   
#include <GL/glu.h>  
#include <iostream>


ArenaCircular::ArenaCircular() :
    Object(),
    PhysicsBulletInterface(),
    RenderOpenGLInterface()
{
}


ArenaCircular::~ArenaCircular()
{

}

void ArenaCircular::Draw (RenderOpenGL* r)
{
    // drawing depends on existence in physics bullet... 
    if (bodies.size() > 0)
    {
	// go through all shapes and draw them
	int i = 0;
	vector<btRigidBody*>::iterator it;
	for (it = bodies.begin(); it != bodies.end(); it++)
	{
	    btRigidBody* body = *it;

	    dsRotationMatrix rotmat;
	    dRotation2dsRotationMatrix(0, rotmat);
        
	    btScalar m[16];
	    btMatrix3x3 rot;
	    rot.setIdentity();
	    
	    btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
	    myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
	    
	    glPushMatrix(); 	
	    glMultMatrixf(m);
	
	    float position [3];
	    position[0] = 0;
	    position[1] = 0;
	    position[2] = 0;

	    if (i == 0)
	    {
		r->dsSetColor (0.3, 0.3, 0.3);
		r->dsDrawBox((float*) &position, (float*) &rotmat, (float*) &dimGround.m_floats);
		i++;
	    }
	    else
	    {
		r->dsSetColorAlpha (0.3, 0.3, 0.3, 0.3);
		r->dsDrawBox((float*) &position, (float*) &rotmat, (float*) &dimBorder.m_floats);
		i++;
	    }
	    glPopMatrix();
	}
    }    
}


void ArenaCircular::Register(PhysicsBullet* p)
{
    radius = 1.0 * p->scalingFactor;
    borderHeight = 0.2  * p->scalingFactor;
    width = 0.05  * p->scalingFactor;
    borderResolution = 40.0  * p->scalingFactor;

    dimGround.setX (radius * 2.0);
    dimGround.setY (radius * 2.0);
    dimGround.setZ (width);

    // border component length is related to border perimeter
    float perimeter = 2.0 * radius * M_PI;
    dimBorder.setX (perimeter / borderResolution);
    dimBorder.setY (width);
    dimBorder.setZ (borderHeight);

    // define collision type
    SetCollisionType (1<<1);

    // create the ground floor
    btBoxShape* groundShape = new btBoxShape(dimGround / 2.0);
    p->m_collisionShapes.push_back(groundShape);
    shapes.push_back(groundShape);
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0.0, 0.0, -dimGround.z() / 2.0));
    
    btScalar mass(0.0);
    btVector3 localInertia(0,0,0);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);    
    body->setUserPointer((void*)(this));
    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);
    bodies.push_back(body);

    // now create the arena as a set of boxes
    float angle = 0.0;
    float angleStep = 2.0 * M_PI / borderResolution;
    for (int i = 0; i < (int)borderResolution; i++)
    {
	// find out the correct x size...
	btBoxShape* shape = new btBoxShape(dimBorder / 2.0);
	p->m_collisionShapes.push_back(shape);
	shapes.push_back(shape);

	// move the box in its right place...
	btTransform transform;
	transform.setIdentity();

	float x = cos(angle) * radius;
	float y = sin(angle) * radius;
	float z = borderHeight / 2.0;

	transform.setOrigin(btVector3(x, y, z));
	btQuaternion q(btVector3(0, 0, 1), angle + M_PI / 2.0);
	transform.setRotation(q);
	
	btScalar mass(0.0);
	btVector3 localInertia(0,0,0);
    
	btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* b = new btRigidBody(rbInfo);    
	b->setUserPointer((void*)(this));
	p->m_dynamicsWorld->addRigidBody(b, collisionType, collisionFilter);
	bodies.push_back(b);

	angle += angleStep;
    }
}

void ArenaCircular::Unregister(PhysicsBullet* p)
{
    
}
