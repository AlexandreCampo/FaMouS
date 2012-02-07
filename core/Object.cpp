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

#include "Object.h"

#include <cstddef>
#include <iostream>

Object::~Object ()
{
    controller = NULL;
}

void Object::Add (Device* d)
{
    devices.push_back(d);
    d->object = this;
}

void Object::Set (Controller* c)
{
    controller = c;
}

void Object::Step (float time, float timestep)
{
    // std::vector<Device*>::iterator it;
    // for (it = devices.begin(); it != devices.end(); it++)
    // {
    // 	(*it)->Step(delta);
    // }

    if (controller)
    {
	controller->Step(time, timestep);
    }
}

void Object::Reset()
{
    if (controller != NULL) controller->Reset();
    
    std::vector<Device*>::iterator it (devices.begin());
    for (; it != devices.end(); ++it)
    {
    	(*it)->Reset();
    }
}
