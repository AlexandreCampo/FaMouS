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

#include "Simulator.h"
#include "RenderOpenGLInterface.h"

#include <iostream>

Simulator::Simulator() 
{
    // the simulator is created : display an advertising banner 
    std::cout << "                                                                               " << std::endl;
    std::cout << "                                                                               " << std::endl;
    std::cout << "                                                                               " << std::endl;
    std::cout << "88888888888          88b           d88                             ad88888ba   " << std::endl;
    std::cout << "88                   888b         d888                            d8\"     \"8b  " << std::endl;
    std::cout << "88                   88`8b       d8'88                            Y8,          " << std::endl;
    std::cout << "88aaaaa  ,adPPYYba,  88 `8b     d8' 88   ,adPPYba,   88       88  `Y8aaaaa,    " << std::endl;
    std::cout << "88\"\"\"\"\"  \"\"     `Y8  88  `8b   d8'  88  a8\"     \"8a  88       88    `\"\"\"\"\"8b,  " << std::endl;
    std::cout << "88       ,adPPPPP88  88   `8b d8'   88  8b       d8  88       88          `8b  " << std::endl;
    std::cout << "88       88,    ,88  88    `888'    88  \"8a,   ,a8\"  \"8a,   ,a88  Y8a     a8P  " << std::endl;
    std::cout << "88       `\"8bbdP\"Y8  88     `8'     88   `\"YbbdP\"'    `\"YbbdP'Y8   \"Y88888P\"   " << std::endl;
    std::cout << "                                                                               " << std::endl;
    std::cout << "                                        A Fast, Modular, and Simple simulator  " << std::endl;         
    std::cout << "                                                                               " << std::endl;
//    std::cout << "                                                                               " << std::endl;

    time = 0.0;
    timestep = 0.05;
    controlTimestep = 0.1;
    controlTime = 1.0;
}

Simulator::~Simulator()
{
    for (unsigned int i = 0; i < objects.size(); i++)
    {
	delete objects[i];
    }

    for (unsigned int i = 0; i < services.size(); i++)
    {
	delete services[i];
    }
}

void Simulator::Add (Service* s)
{
    services.push_back(s);
}

void Simulator::Add (Object* o)
{
    objects.push_back(o);
}

void Simulator::Step ()
{
    // for each service
    // this call must happen here, because services may provide additional information to devices 
    // (eg bullet uses callback methods for collision detection)
    for (unsigned int i = 0; i < services.size(); i++)
    {
	services[i]->Step(time, timestep);
    }

    // for each object
    // first perceive the environment, 
    for (unsigned int i = 0; i < objects.size(); i++)
    {
	Object* o = objects[i];
	for (unsigned int j = 0; j < o->devices.size(); j++)
	{
	    o->devices[j]->PerceptionStep(time, timestep);
	}
    }
    
    // then objects process information
    if (controlTime > controlTimestep)
    {
	for (unsigned int i = 0; i < objects.size(); i++)
	{
	    objects[i]->Step(time, timestep);
	}
	controlTime = 0.0;
    }
    
    // then execute some actions in the environment    
    for (unsigned int i = 0; i < objects.size(); i++)
    {
	Object* o = objects[i];
	for (unsigned int j = 0; j < o->devices.size(); j++)
	{
	    o->devices[j]->ActionStep(time, timestep);
	}
    }

    time += timestep;
    controlTime += timestep;
}


void Simulator::SetTimestep (float ts)
{
    timestep = ts;
}

float Simulator::GetTimestep ()
{
    return timestep;
}

void Simulator::SetControlTimestep (float cts)
{
    controlTimestep = cts;
}

float Simulator::GetControlTimestep ()
{
    return controlTimestep;
}


void Simulator::SetTime (float t)
{
    time = t;
}

float Simulator::GetTime ()
{
    return time;
}

void Simulator::Reset ()
{
    time = 0.0;
    controlTime = 1.0;

    // it makes sense to reset in this central point that will anyway
    // also call the experiment. Hence any service can ask for a reset, without
    // knowing who does what, so this leaves freedom in what happens.

    vector<Service*>::iterator s (services.begin());
    for (; s != services.end(); ++s)
    {
    	(*s)->Reset();
    }

    vector<Object*>::iterator o (objects.begin());
    for (; o != objects.end(); ++o)
    {
    	(*o)->Reset();
    }
}
