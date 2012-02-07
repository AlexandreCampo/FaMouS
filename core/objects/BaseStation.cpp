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

#include "BaseStation.h"
#include "Simulator.h"

#include <iostream>


BaseStation::BaseStation() :
    Object(),
    PhysicsBulletInterface(),
    RenderOpenGLInterface()
{
} 


BaseStation::~BaseStation()
{

}

void BaseStation::AddDevices (PhysicsBullet* p) 
{
   btTransform t;
   t.setIdentity();    
   int cf = (1 << 2); // TODO we should probably not include walls ....
   int ct = (1 << 3);
   acoustic = new DeviceAcousticTransceiver (p, body, t, cf, ct, 0.7 * p->scalingFactor);
   Add (acoustic);
}

void BaseStation::Draw (RenderOpenGL* r)
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
    
    r->dsSetColor (1.0, 0.75, 0.0);
    r->dsSetTexture (0);
    r->dsDrawCylinder((float*) &pos, (float*) &rotmat, height, radius);
    
    glPopMatrix();
    
//    proximitySensor->Draw(r);
//    acoustic->Draw (r);
}


void BaseStation::Register (PhysicsBullet* p)
{
    height = 0.05 * p->scalingFactor;
    radius = 0.2 * p->scalingFactor;

    // define collision type and ensure there is no collisions on this object
    SetCollisionType (1 << 2);
    SetCollisionFilter (0);
    
    btCylinderShape* shape = new btCylinderShapeZ(btVector3(radius, radius, height / 2.0));
    p->m_collisionShapes.push_back(shape);
    
    // no dynamics
    btScalar mass(0.0);    
    btVector3 localInertia(0.0, 0.0, 0.0);
 
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0.5, 0.5, 2));
		
    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    rbInfo.m_friction = 0.05;
    body = new btRigidBody(rbInfo);    
    body->setUserPointer((void*)(this));
    
    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);

    AddDevices(p);
}

void BaseStation::Unregister (PhysicsBullet* p)
{
}


void BaseStation::SetPosition (float x, float y, float z)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setOrigin(btVector3(x, y, z));
    body->setCenterOfMassTransform(t);
}

void BaseStation::GetPosition (float& x, float& y, float& z)
{
    btTransform t = body->getCenterOfMassTransform();
    btVector3 pos = t.getOrigin();
    x = pos.x();
    y = pos.y();
    z = pos.z();
}

// very unfortunate... I consider this as a c++ bug : 
// overloading inherited methods hides base methods even though signature is different
void BaseStation::Register (RenderOpenGL* r)
{
    RenderOpenGLInterface::Register(r);
}

void BaseStation::Unregister (RenderOpenGL* r)
{
    RenderOpenGLInterface::Unregister(r);
}
