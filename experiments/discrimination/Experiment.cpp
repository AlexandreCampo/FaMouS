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

#include "RenderOpenGL.h"
#include "PhysicsBullet.h"

#include "Floor.h"
#include "ArenaCircular.h"
#include "AquariumCircular.h"
#include "BasicRobot.h"
#include "RobotLily.h"
#include "BaseStation.h"

#include "ControllerBaseStation.h"
#include "ControllerDiscrimination.h"
#include "Experiment.h"

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <tinyxml.h>

extern gsl_rng* rng;
extern long int rngSeed;

Experiment::Experiment (Simulator* simulator, bool graphics, string outputFilename, int replications, int replicationShift, string paramsFilename) :
    Service (simulator)
{
    // read params from file if available
    paramsExp = new ExperimentParameters;
    paramsCtrl = new ControllerDiscriminationParameters;
    if (!paramsFilename.empty()) LoadParameters (paramsFilename);

    // record replications count
    if (replications > 0) paramsExp->replications = replications;
    if (replicationShift > 0) paramsExp->replicationShift = replicationShift;
    currentReplication = replicationShift;

    // output file 
    if (!outputFilename.empty()) paramsExp->outputFilename = outputFilename;

    // random number generation
    init_rng(&rng);        

    // add services 
    physics = new PhysicsBullet(simulator, 10.0);
    simulator->Add (physics);
    
    render = NULL;
    if (graphics)
    {
	render = new RenderOpenGL(simulator);
	simulator->Add (render);

	float lookat[3] = {0,0,0.7 * physics->scalingFactor};
	float dab[3] = {4.0 * physics->scalingFactor, 90.0, 20.0};
	render->dsSetViewpoint2 (lookat, dab);
    }

    // add the experiment so that we can step regularly
    simulator->Add (this);
    

    // add robots
    for (int i = 0; i < paramsExp->robotsCount; i++)
    {
	RobotLily* r = new RobotLily ();
	r->Register(physics);
	if (render) r->Register(render);
	ControllerDiscrimination* c = new ControllerDiscrimination (r);
	c->SetParameters(paramsCtrl);
	robots.push_back(r);
	simulator->Add(r);   	
	
	// position is set in reset
    }

    // add base stations
    float a = 0.0;
    for (int i = 0; i < paramsExp->basestationsCount; i++)
    {
	BaseStation* b = new BaseStation ();
	b->Register (physics);
	if (render) b->Register (render);
	ControllerBaseStation* c = new ControllerBaseStation (b, i);	
	basestations.push_back(b);
	simulator->Add (b);
	
	float distance =  1.0 * physics->scalingFactor;
	float angle = a;
	float x = cos(angle) * distance;
	float y = sin(angle) * distance;
	float z = 1.0 * physics->scalingFactor + b->height / 2.0;
	b->SetPosition(x, y, z);

	a += 2.0 * M_PI / float (paramsExp->basestationsCount);
    }

    AquariumCircular* aquarium = new AquariumCircular(paramsExp->aquariumRadius * physics->scalingFactor,  1.0 * physics->scalingFactor, 1.0 * physics->scalingFactor, 40.0);
    aquarium->Register(physics);
    if (render) aquarium->Register(render);
    simulator->Add(aquarium);

    // set last stuff, position of robots mainly
    Reset();
}

Experiment::~Experiment()
{
    // if requested, dump results to a file
    if (!paramsExp->outputFilename.empty())
    {
	std::fstream output (paramsExp->outputFilename.c_str(), std::ios::out);

	if (output)
	{
	    for (unsigned int i = 0; i < results.size(); i++)
	    {
		output << results[i].replication << "\t";
		output << results[i].time << "\t";
		for (unsigned int j = 0; j < basestations.size(); j++)
		{
		    output << results[i].base[j] << "\t";
		}
		output << std::endl;
	    }
	    output.close();	
	}
    }
}

void Experiment::LoadParameters (string filename)
{
    TiXmlDocument xml(filename.c_str());
    bool loadOkay = xml.LoadFile();
    
    if (!loadOkay)
    {
	std::cout << "Could not load parameters file " << filename << std::endl;
	exit(1);
    }
    
    TiXmlNode* node = NULL;
    TiXmlElement* el = NULL;

    node = xml.FirstChild( "ControllerDiscrimination" );
    if (node) 
    {
	el = node->ToElement();
	paramsCtrl->LoadXML (el);
    }
    
    node = xml.FirstChild( "Experiment" );
    if (node) 
    {
	el = node->ToElement();
	paramsExp->LoadXML (el);
    }
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

	// // todo testing ray cast device
	if (i == 0)
	{
	    x = 0.45 * physics->scalingFactor; y = 0.03 * physics->scalingFactor; 
	    qa = M_PI;
	}
	if (i == 1)
	{
	    x = -0.45 * physics->scalingFactor; y = -0.03 * physics->scalingFactor;
	    qa = 0.0;
	}

	r->SetPosition (x, y, z);
	r->SetRotation (qx, qy, qz, qa);
    }

    // basestations
    // don't change anything    
}

void Experiment::Step (float time, float timestep)
{       
    // assume service time step is 0.05 sec
    if (ticks % 200 == 0)
    {
	if (!paramsExp->outputFilename.empty())
	{
	    // ok here I want to check how many robots are below each base
	    // for each base, check how many robots fit in a given radius	
	    int robotsAtBase[10];
	    for (unsigned int i = 0; i < basestations.size(); i++)
	    {
		robotsAtBase[i] = 0;
		float x, y, z;
		basestations[i]->GetPosition(x, y, z);
		
		// now cycle through all robots to find out the closest ones
		for (unsigned int j = 0; j < robots.size(); j++)
		{
		    float rx, ry, rz;
		    robots[j]->GetPosition(rx, ry, rz);
		    
		    float dx = rx - x; 
		    float dy = ry - y;
		    float dz = rz - z; 
		    
		    float d = sqrt (dx * dx + dy * dy + dz * dz);
		    if (d < basestations[i]->acoustic->maxRange)
		    {
			// the robot belongs to the base !
			robotsAtBase[i]++;
		    }
		}
	    }
	    Results r;
	    r.replication = currentReplication;
	    r.time = time;
	    for (unsigned int i = 0; i < basestations.size(); i++)
	    {
		r.base[i] = robotsAtBase[i];
	    }
	    results.push_back(r);
	}
    }
    ticks++;
}

void Experiment::Run()
{
    // with render, give up control and stepping is done by render
    if (render)
    {
	render->SetPaused (true);
	render->Run();
    }
    else
    {
	for (currentReplication = paramsExp->replicationShift; currentReplication < paramsExp->replications + paramsExp->replicationShift; currentReplication++)
	{	    
	    for (float t = 0.0; t < paramsExp->maxTime; t+=0.05)
	    {
		simulator->Step();
	    }

	    // reset the simulator, and also services, objects etc...
	    simulator->Reset();    
	}
    }
}
