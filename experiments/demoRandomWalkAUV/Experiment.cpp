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

#include "Experiment.h"

// services used in the simulator
#include "RenderOpenGL.h"
#include "PhysicsBullet.h"

// objects used in this experiment
#include "Floor.h"
#include "AquariumCircular.h"
#include "RobotLily.h"

// controllers used
#include "ControllerRandomWalk.h"

// headers for random number generation
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

// additional tools
#include <iostream>

extern gsl_rng* rng;
extern long int rngSeed;

Experiment::Experiment (Simulator* simulator) :
    Service (simulator)
{
    // read params from file if available
    paramsExp = new ExperimentParameters;
    paramsCtrl = new ControllerRandomWalkParameters;

    // random number generation
    init_rng(&rng);        

    // add services 
    physics = new PhysicsBullet(simulator, 10.0);
    simulator->Add (physics);
    
    render = new RenderOpenGL(simulator);
    simulator->Add (render);

    // set view point in the opengl render
    float lookat[3] = {0,0,0.7 * physics->scalingFactor};
    float dab[3] = {4.0 * physics->scalingFactor, 90.0, 20.0}; // distance to lookat point, angle around, elevation angle
    render->dsSetViewpoint2 (lookat, dab);

    // add the experiment so that we can step regularly
    simulator->Add (this);
    

    // add robots
    for (int i = 0; i < paramsExp->robotsCount; i++)
    {
	RobotLily* r = new RobotLily ();
	r->Register(physics);
	if (render) r->Register(render);
	ControllerRandomWalk* c = new ControllerRandomWalk (r);
	c->SetParameters(paramsCtrl);
	robots.push_back(r);
	simulator->Add(r);   	
	
	// position is set in reset
    }

    AquariumCircular* aquarium = new AquariumCircular(paramsExp->aquariumRadius * physics->scalingFactor,  1.0 * physics->scalingFactor, 1.0 * physics->scalingFactor, 40.0);
    aquarium->Register(physics);
    aquarium->Register(render);
    simulator->Add(aquarium);

    // set last stuff, position of robots mainly
    Reset();
}

Experiment::~Experiment()
{
}

void Experiment::Reset ()
{
    ticks = 0;

    // reset robots
    for (unsigned int i = 0; i < robots.size(); i++)
    {
	RobotLily* r = robots[i];

	// reset robot's position
	float distance = gsl_rng_uniform(rng) * 0.8 * paramsExp->aquariumRadius * physics->scalingFactor;
	float angle = (gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI);
	float x = cos(angle) * distance;
	float y = sin(angle) * distance;
//	float z = (gsl_rng_uniform(rng) * 1.8 * physics->scalingFactor) + r->dimensions[2] / 2.0;
	float z = 0.7 * physics->scalingFactor + r->dimensions[2] / 2.0;	

	float qx = 0.0 ; 
	float qy = 0.0;
	float qz = 1.0;
	float qa = gsl_rng_uniform(rng) * M_PI * 2.0 - M_PI;

	r->SetPosition (x, y, z);
	r->SetRotation (qx, qy, qz, qa);
    }
}

void Experiment::Step (float time, float timestep)
{       
    // assume service time step is 0.05 sec
    if (ticks % 200 == 0)
    {
	// 10 simulated seconds elapsed since last call
    }
    ticks++;
}

void Experiment::Run()
{
    // with render, give up control and stepping is done by render
    render->SetPaused (true);
    render->Run();
}
