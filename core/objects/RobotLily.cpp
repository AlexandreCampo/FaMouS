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

#include "RobotLily.h"
#include "Simulator.h"

#include <iostream>



RobotLily::RobotLily() :
    Object(),
    PhysicsBulletInterface(),
    RenderOpenGLInterface()
{

    colr = 1.0;
    colg = 0.0;
    colb = 0.0;
    cola = 1.0;

    // define collision type
    SetCollisionType (1 << 2);
    SetCollisionFilter (0x7FFFFFFF);
}
 


RobotLily::~RobotLily()
{
}

void RobotLily::AddDevices(PhysicsBullet* p)
{
    btVector3 leftPos(0, -dimensions[1] * 1.0, 0);
    btVector3 rightPos(0, +dimensions[1] * 1.0, 0);
    propellers = new DevicePropellers(body, leftPos, rightPos, 0.1 * scalingFactor);
    Add (propellers);

    float neutralVolume = (1.0 / body->getInvMass()) * body->m_fluidDensity;
    float ballastVolume = (0.02 * 0.02 * 0.15 * scalingFactor * scalingFactor * scalingFactor);
    ballast = new DeviceBallast (body, neutralVolume - ballastVolume / 2.0, neutralVolume + ballastVolume / 2.0);
    
    btTransform t2;
    t2.setIdentity();    
    int cf2 = this->collisionType;
    int ct2 = (1 << 3);
    acoustic = new DeviceAcousticTransceiver (p, body, t2, cf2, ct2, 0.9 * p->scalingFactor);    
    Add (acoustic);
    
    int cf3 = 0x7FFFFFFF; 
    int ct3 = this->collisionType; 
    float a = 15.0 * M_PI / 180.0;
    float px = cos (a) * cos (a);
    float pyz = sin (a);
    
    // ray sensors may not detect objects that go inside the other object... hence I add some distance.
    float addedrange = 0.005 * p->scalingFactor;
    
    float range = 0.3 * p->scalingFactor + addedrange;
    
    btVector3 pos (0.1 * p->scalingFactor - addedrange, -0.005  * p->scalingFactor, 0.005  * p->scalingFactor);
    btVector3 dir (px, -pyz, pyz);   
    rayFrontLU = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);
    Add (rayFrontLU);
    
    pos = btVector3 (0.1 * p->scalingFactor - addedrange, -0.005  * p->scalingFactor, -0.005  * p->scalingFactor);
    dir = btVector3 (px, -pyz, -pyz);   
    rayFrontLD = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);
    Add (rayFrontLD);

    pos = btVector3 (0.1 * p->scalingFactor - addedrange, 0.005  * p->scalingFactor, 0.005  * p->scalingFactor);
    dir = btVector3 (px, pyz, pyz);   
    rayFrontRU = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);
    Add (rayFrontRU);

    pos = btVector3 (0.1 * p->scalingFactor - addedrange, 0.005  * p->scalingFactor, -0.005  * p->scalingFactor);
    dir = btVector3 (px, pyz, -pyz);   
    rayFrontRD = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);
    Add (rayFrontRD);
    
    pos = btVector3 (0.0, -0.025 * p->scalingFactor + addedrange, 0.0);
    dir = btVector3 (0.0, -1.0, 0.0);   
    rayLeft = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);  
    Add (rayLeft);
    
    pos = btVector3 (0.0, 0.025 * p->scalingFactor - addedrange, 0.0);
    dir = btVector3 (0.0, 1.0, 0.0);   
    rayRight = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);  
    Add (rayRight);
    
    pos = btVector3 (0.0, 0.0, 0.05 * p->scalingFactor - addedrange);
    dir = btVector3 (0.0, 0.0, 1.0);   
    rayTop = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);  
    Add (rayTop);
    
    pos = btVector3 (0.0, 0.0, -0.05 * p->scalingFactor + addedrange);
    dir = btVector3 (0.0, 0.0, -1.0);   
    rayBottom = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);  
    Add (rayBottom);
    
    pos = btVector3 (-0.1 + addedrange, 0.0, 0);
    dir = btVector3 (-1.0, 0.0, 0.0);   
    rayBack = new DeviceRayCast (p, body, pos, dir, range, cf3, ct3);  
    Add (rayBack);
}

void RobotLily::Draw (RenderOpenGL* r)
{
    btScalar m[16];
    btMatrix3x3 rot;
    rot.setIdentity();
    
    btTransform t = body->getCenterOfMassTransform() * principalTransform.inverse();
    t.getOpenGLMatrix(m);
        
    glPushMatrix(); 
    glMultMatrixf(m);
    
    dsRotationMatrix rotmat;

    float dims[3]; 
    memcpy (dims, dimensions, sizeof(dims));

    float pos[3];	
    pos[0] = -dimensions[0] / 5.0 / 2.0;
    pos[1] = 0;
    pos[2] = 0;
    
    dRotation2dsRotationMatrix(0, rotmat);
    
    r->dsSetColor (colr, colg, colb);
    r->dsSetTexture (0);

    dims[0] *= 4.0 / 5.0;
    r->dsDrawBox((float*) &pos, (float*) &rotmat, (float*) &dims);

    r->dsSetColor (1, 1, 1);
    r->dsSetTexture (0);
    pos[0] = dimensions[0] * 4.0 / 5.0 / 2.0;
    dims[0] *= 1.0 / 5.0;
    r->dsDrawBox((float*) &pos, (float*) &rotmat, (float*) &dims);

    glPopMatrix();
    
    // draw devices ...
    // proximitySensor->Draw(r);
    // propellers->Draw(r);
    // rayFrontLU->Draw(r);
    // rayFrontLD->Draw(r);
    // rayFrontRU->Draw(r);
    // rayFrontRD->Draw(r);
    // rayBack->Draw(r);
    // rayLeft->Draw(r);
    // rayRight->Draw(r);
    // rayTop->Draw(r);
    // rayBottom->Draw(r);
}





void RobotLily::Register (PhysicsBullet* p)
{
    this->scalingFactor =  p->scalingFactor;

    dimensions[0] = 0.20 * p->scalingFactor;
    dimensions[1] = 0.05 * p->scalingFactor;
    dimensions[2] = 0.10 * p->scalingFactor;

    btScalar mass(0.5);
    btScalar waterDensity (1000.0 / (p->scalingFactor * p->scalingFactor * p->scalingFactor));
    btScalar volume = dimensions[0] * dimensions[1] * dimensions[2];
    btVector3 localInertia(0.0, 0.0, 0.0);
    btVector3 linearDrag (0.25, 0.3, 0.9);
    btVector3 angularDrag (0, 0, 0);
    btScalar linearDamping = 0;
    btScalar angularDamping = 0.7;

    // we create a first, detailed compound shape in order to calculate the correct inertia tensor
    btCompoundShape* cshape = new btCompoundShape(false);

    btBoxShape* boxShapeA = new btBoxShape(btVector3(dimensions[0], dimensions[1], dimensions[2]) / 2.0);
    btTransform localTransform;
    localTransform.setIdentity();
    cshape->addChildShape(localTransform,boxShapeA);

    btBoxShape* boxShapeB = new btBoxShape(btVector3(dimensions[0], dimensions[1], dimensions[2] / 10.0) / 2.0);
    localTransform.setOrigin (btVector3(0, 0, - dimensions[2] / 2.0 + dimensions[2] / 10.0 / 2.0));
    cshape->addChildShape(localTransform,boxShapeB);

    btScalar* masses = new btScalar [2];
    masses[0] = 0.2 * mass;
    masses[1] = 0.8 * mass;

    btVector3 inertia;
    CalculateInertia (cshape, masses, principalTransform, inertia);

    // we can delete previous shapes
    delete cshape;
    delete boxShapeA;
    delete boxShapeB;
    delete [] masses;

    // now the real shape used used, a simplified compound shape that will be used for collisions
    // create a new compound with world transform/center of mass properly aligned with the principal axis   
    cshape = new btCompoundShape(false); 
    btBoxShape* boxShape = new btBoxShape(btVector3(dimensions[0], dimensions[1], dimensions[2]) / 2.0);
    
    localTransform.setIdentity();
    btTransform newLocalTransform = principalTransform.inverse() * localTransform;
    cshape->addChildShape(newLocalTransform,boxShape);
       
    cshape->recalculateLocalAabb();
    p->m_collisionShapes.push_back(cshape);
    shape = cshape;
	
    btTransform startTransform;
    startTransform.setIdentity();

		
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform * principalTransform);
    btUnderwaterRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,inertia, principalTransform.getOrigin(), waterDensity, volume);
    rbInfo.m_gravity = btVector3 (0, 0, -9.81  * p->scalingFactor);
    rbInfo.m_friction = 0.05;    
    rbInfo.m_linearDamping = linearDamping; // was 0.1
    rbInfo.m_angularDamping = angularDamping; //was :  0.25; but creates problem to control motion... trying other values

    body = new btUnderwaterRigidBody(rbInfo);           
    body->setDragCoefficients (linearDrag, angularDrag);
    body->setUserPointer((void*)(this));
    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);

    AddDevices(p);
}

void RobotLily::Unregister (PhysicsBullet* p)
{
}

// very unfortunate... I consider this as a c++ bug : 
// overloading inherited methods hides base methods even though signature is different
void RobotLily::Register (RenderOpenGL* r)
{
    RenderOpenGLInterface::Register(r);
}

void RobotLily::Unregister (RenderOpenGL* r)
{
    RenderOpenGLInterface::Unregister(r);
}


void RobotLily::SetPosition (float x, float y, float z)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setOrigin(btVector3(x, y, z));
    body->setCenterOfMassTransform(t);
}

void RobotLily::GetPosition (float& x, float& y, float& z)
{
    btTransform t = body->getCenterOfMassTransform();
    btVector3 pos = t.getOrigin();
    x = pos.x();
    y = pos.y();
    z = pos.z();
}


void RobotLily::SetRotation (float qx, float qy, float qz, float qa)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setRotation(btQuaternion(btVector3 (qx, qy, qz), qa));
    body->setCenterOfMassTransform(t);
}


void RobotLily::SetColor (float r, float g, float b, float a)
{
    this->colr = r;
    this->colg = g;
    this->colb = b;
    this->cola = a;
}


// I had to reimplement this method because I just want a single shape but a shifted COM...
void RobotLily::CalculateInertia(btCompoundShape* s, btScalar* masses, btTransform& principal, btVector3& inertia) const
{
    int n = s->getNumChildShapes();
    
    btScalar totalMass = 0;
    btVector3 center(btVector3(0,0,0));
    int k;
    
    for (k = 0; k < n; k++)
    {
	btAssert(masses[k]>0);
	center += s->getChildTransform(k).getOrigin() * masses[k];
	totalMass += masses[k];
    }
    
    btAssert(totalMass>0);
    
    center /= totalMass;
    principal.setOrigin(center);
    
    btMatrix3x3 tensor(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for ( k = 0; k < n; k++)
    {
	btVector3 i;
	s->getChildShape(k)->calculateLocalInertia(masses[k], i);
	
	const btTransform& t = s->getChildTransform(k);
	btVector3 o = t.getOrigin() - center;
	
	//compute inertia tensor in coordinate system of compound shape
	btMatrix3x3 j = t.getBasis().transpose();
	j[0] *= i[0];
	j[1] *= i[1];
	j[2] *= i[2];
	j = t.getBasis() * j;
	
	//add inertia tensor
	tensor[0] += j[0];
	tensor[1] += j[1];
	tensor[2] += j[2];
	
	//compute inertia tensor of pointmass at o
	btScalar o2 = o.length2();
	j[0].setValue(o2, 0, 0);
	j[1].setValue(0, o2, 0);
	j[2].setValue(0, 0, o2);
	j[0] += o * -o.x(); 
	j[1] += o * -o.y(); 
	j[2] += o * -o.z();
	
	//add inertia tensor of pointmass
	tensor[0] += masses[k] * j[0];
	tensor[1] += masses[k] * j[1];
	tensor[2] += masses[k] * j[2];
    }
    
    tensor.diagonalize(principal.getBasis(), btScalar(0.00001), 20);
    inertia.setValue(tensor[0][0], tensor[1][1], tensor[2][2]);
}
