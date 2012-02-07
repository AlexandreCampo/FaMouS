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

#include "DeviceMagicForce.h"

#include <iostream>

DeviceMagicForce::DeviceMagicForce(btRigidBody* body)
{
    this->body = body;
}


DeviceMagicForce::~DeviceMagicForce()
{

}

// not nice ... should take place in ActionStep
void DeviceMagicForce::ApplyForce(btVector3 f)
{
    body->activate();

    // btTransform t = body->getCenterOfMassTransform();
    // btVector3 f2 = t.getBasis().getColumn(0) * 0.5;
    // btVector3 flocal = btVector3(1,0,0) * 0.5;

    //  btVector3 pos (0, -0.1, 0);
//    body->applyTorque(pos.cross(f));
    body->applyCentralForce(f);
}

void DeviceMagicForce::PerceptionStep(float time, float timestep)
{
}

void DeviceMagicForce::ActionStep(float time, float timestep)
{
}

void DeviceMagicForce::Reset()
{
}
