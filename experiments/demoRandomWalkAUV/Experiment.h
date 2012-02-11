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

#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "Simulator.h"
#include "Service.h"
#include "Gsl.h"

#include <vector>


class PhysicsBullet;
class RenderOpenGL;
class RobotLily;
class ControllerRandomWalkParameters;

struct ExperimentParameters
{
    int robotsCount;
    float aquariumRadius;
    ExperimentParameters ()
	{
	    robotsCount = 30;
	    aquariumRadius = 2.0;
	}    
};


class Experiment : public Service
{
public:

    PhysicsBullet* physics;
    RenderOpenGL* render;
    unsigned int ticks;

    std::vector<RobotLily*> robots;
		
    ExperimentParameters* paramsExp;
    ControllerRandomWalkParameters* paramsCtrl;

    Experiment (Simulator* s);
    ~Experiment ();
    
    void Reset ();
    void Step (float time, float timestep);
    void Run();
};


#endif
