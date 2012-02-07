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


#ifndef DEVICE_BALLAST_H
#define DEVICE_BALLAST_H

#include "Device.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btUnderwaterRigidBody.h"


class DeviceBallast : public Device
{
public : 
    btUnderwaterRigidBody* body;    

    float minVolume;
    float maxVolume;
    float neutralVolume;

    float buoyancyFactor;


    DeviceBallast(btUnderwaterRigidBody* body, float minVolume, float maxVolume);
    ~DeviceBallast ();

    void PerceptionStep(float time, float timestep);
    void ActionStep(float time, float timestep);
    void Reset();

    void CalculateNeutralBuoyancy ();

    void SetMinVolume (float v);
    void SetMaxVolume (float v);
    float GetMinVolume ();
    float GetMaxVolume ();
    
    void SetBuoyancyFactor (float f);    
    float GetBuoyancyFactor ();
};



#endif
