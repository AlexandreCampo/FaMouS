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

#include "DeviceBallast.h"

#include <iostream>


DeviceBallast::DeviceBallast(btUnderwaterRigidBody* body, float minVolume, float maxVolume) :
    Device()
{
    this->body = body;    
    
    this->minVolume = minVolume;
    this->maxVolume = maxVolume;
    
    CalculateNeutralBuoyancy ();

    SetBuoyancyFactor (0.0);
}


DeviceBallast::~DeviceBallast()
{
}

void DeviceBallast::CalculateNeutralBuoyancy ()
{
    // calculate volume of fluid to make up for the mass of the body
    float m = 1.0 / body->getInvMass();
    float d = body->m_fluidDensity;

    neutralVolume = m / d;
}

// the factor is in [-1; 1] , 0 is neutral buoyancy
void DeviceBallast::SetBuoyancyFactor (float f)
{
    buoyancyFactor = f;
    
    if (f >= 0.0)
    {
	float v = neutralVolume + (maxVolume - neutralVolume) * f;
	body->setVolume (v);
    }
    else
    {
	float v = neutralVolume + (neutralVolume - minVolume) * f;
	body->setVolume (v);	
    }
}


void DeviceBallast::PerceptionStep(float time, float timestep)
{
}

void DeviceBallast::ActionStep(float time, float timestep)
{
}

void DeviceBallast::Reset()
{
    SetBuoyancyFactor (0.0);    
}
   
void DeviceBallast::SetMinVolume (float v)
{
    minVolume = v;
}
void DeviceBallast::SetMaxVolume (float v)
{
    maxVolume = v;
}
float DeviceBallast::GetMinVolume ()
{
    return (minVolume);
}
float DeviceBallast::GetMaxVolume ()
{
    return (maxVolume);
}
float DeviceBallast::GetBuoyancyFactor ()
{
    return (buoyancyFactor);
}

