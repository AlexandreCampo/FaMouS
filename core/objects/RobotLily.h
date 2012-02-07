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

#ifndef ROBOT_LILY2_H
#define ROBOT_LILY2_H

#include "Object.h"
#include "RenderOpenGLInterface.h"
#include "PhysicsBulletInterface.h"

#include "DeviceMagicForce.h"
#include "DevicePropellers.h"
#include "DeviceBallast.h"
#include "DeviceAcousticTransceiver.h"
#include "DeviceRayCast.h"


class RobotLily : public Object, public RenderOpenGLInterface, public PhysicsBulletInterface
{
public:

    float dimensions[3];
    float scalingFactor;

    btCollisionShape* shape;
    btUnderwaterRigidBody* body;

    btTransform principalTransform;

    DeviceMagicForce* magicForce;
    DevicePropellers* propellers;
    DeviceBallast* ballast;
    DeviceRayCast* rayFrontLU; // left up
    DeviceRayCast* rayFrontLD; // left down
    DeviceRayCast* rayFrontRU; // right up
    DeviceRayCast* rayFrontRD; // right down
    DeviceRayCast* rayTop;
    DeviceRayCast* rayBottom;
    DeviceRayCast* rayLeft;
    DeviceRayCast* rayRight;
    DeviceRayCast* rayBack;
    DeviceAcousticTransceiver* acoustic;

    float colr, colg, colb, cola;

    RobotLily ();
    ~RobotLily ();

    void AddDevices(PhysicsBullet* p);

    void Draw (RenderOpenGL* r);

    void Register (PhysicsBullet* p);
    void Unregister (PhysicsBullet* p);

    void Register (RenderOpenGL* r);
    void Unregister (RenderOpenGL* r);

    void SetPosition (float x, float y, float z);
    void SetRotation (float qx, float qy, float qz, float qa);
    void GetPosition (float& x, float& y, float& z);

    void SetColor (float r = 1, float g = 0, float b = 0, float a = 1);
    void CalculateInertia (btCompoundShape* s, btScalar* masses, btTransform& principal, btVector3& inertia) const;
};

#endif
