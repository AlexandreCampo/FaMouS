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

#include <iostream>
#include <GL/glut.h> 
#include <GL/gl.h>   
#include <GL/glu.h>  


ArenaCircular::ArenaCircular (float radius, float height, float thickness, float resolution) :
    Object(),
    PhysicsBulletInterface(),
    RenderOpenGLInterface()
{
    this->radius = radius;
    this->height = height;
    this->width = thickness;
    this->borderResolution = resolution;

    dimGround.setX (radius * 2.0 + width);
    dimGround.setY (radius * 2.0 + width);
    dimGround.setZ (width);

    // border component length is related to border perimeter
    float perimeter = 2.0 * radius * M_PI;
    float angle = 2.0 * M_PI / borderResolution;
    dimBorder.setX (tan(angle / 2.0) * radius * 3.0);
    dimBorder.setY (width);
    dimBorder.setZ (height);

    // define collision type
    SetCollisionType (1<<1);
    SetCollisionFilter (0x7FFFFFFF);
}


ArenaCircular::~ArenaCircular()
{

}

void ArenaCircular::Draw (RenderOpenGL* r)
{
    // go through all shapes and draw them
    int i = 0;
    vector<btRigidBody*>::iterator it;
//    for (it = bodies.begin(); it != bodies.end(); it++)
    it = bodies.begin();
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
	position[2] = width/2.0 - 0.1;
	
	if (i == 0)
	{
	    r->dsSetColor (0.3, 0.3, 0.3);
	    btVector3 d (radius*2.0, radius*2.0, 0.1); 
	    r->dsDrawBox((float*) &position, (float*) &rotmat, (float*) &d.m_floats);
	    i++;
	}
	else
	{
	    // dont draw the real boxes, it is unaesthetic
	    // r->dsSetColor (0.7, 0.7, 0.7);
	    // btVector3 d = dimBorder; 
	    // r->dsDrawBox((float*) &position, (float*) &rotmat, (float*) &d.m_floats);
//	    r->dsDrawCylinder((float*) &position, (float*) &rotmat, d.z(), d.x());
	    i++;
	}
	glPopMatrix();
	glPushMatrix(); 	

	// draw the borders using wireframe
	glEnable(GL_COLOR_MATERIAL);	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(1.0);
	glDisable(GL_CULL_FACE);
	glColor4f(0.1, 0.1, 0.1, 1);

	float l = height;
	float r = radius;
	int i;
	float tmp,ny,nz,a,ca,sa;
	const int n = borderResolution;	// number of sides to the cylinder (divisible by 4)
	
	a = float(M_PI*2.0)/float(n);
	sa = (float) sin(a);
	ca = (float) cos(a);
	
	int ringsCount = 3;
	float zstep = l / float(ringsCount);
	float zoffset = l + 0.05;
	for (int zs = 0; zs <= ringsCount; zs++)
	{
	    // draw cylinder body
	    ny=1; nz=0;		  // normal vector = (0,ny,nz)
	    glBegin (GL_LINE_STRIP);
	    for (i=0; i<=n; i++) 
	    {
		glNormal3d (ny,nz,zoffset);
		glVertex3d (ny*r,nz*r,zoffset);
		
		// rotate ny,nz
		tmp = ca*ny - sa*nz;
		nz = sa*ny + ca*nz;
		ny = tmp;
	    }
	    glEnd();
	    zoffset -= zstep;
	}

	glEnable(GL_CULL_FACE);	
	glPolygonMode(GL_FRONT, GL_FILL);
	glDisable(GL_COLOR_MATERIAL);

	glPopMatrix();
    }    
}


void ArenaCircular::Register(PhysicsBullet* p)
{
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

	float x = cos(angle) * (radius + width / 2.0);
	float y = sin(angle) * (radius + width / 2.0);
	float z = height / 2.0;

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

// very unfortunate... I consider this as a c++ bug : 
// overloading inherited methods hides base methods even though signature is different
void ArenaCircular::Register (RenderOpenGL* r)
{
    RenderOpenGLInterface::Register(r);
}

void ArenaCircular::Unregister (RenderOpenGL* r)
{
    RenderOpenGLInterface::Unregister(r);
}
