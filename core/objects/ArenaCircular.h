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

#ifndef ARENA_CIRCULAR_H
#define ARENA_CIRCULAR_H

#include "Object.h"
#include "RenderOpenGLInterface.h"
#include "PhysicsBulletInterface.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#include <vector>

class ArenaCircular : public Object, public RenderOpenGLInterface, public PhysicsBulletInterface
{
public:

    float radius;
    float borderHeight;
    float width;
    float borderResolution;
    btVector3 dimGround;
    btVector3 dimBorder;

    // the arena is made of several boxes (ground, then walls around)
    std::vector<btCollisionShape*> shapes;
    std::vector<btRigidBody*> bodies;

    ArenaCircular();
    ~ArenaCircular();

    void Draw (RenderOpenGL* r);

    void Register (PhysicsBullet* p);
    void Unregister (PhysicsBullet* p);
};

#endif
