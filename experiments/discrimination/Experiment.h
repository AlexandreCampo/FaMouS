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
#include <fstream>
#include <tinyxml.h>


class PhysicsBullet;
class RenderOpenGL;
class RobotLily;
class BaseStation;
class ControllerDiscriminationParameters;

struct ExperimentParameters
{
    string outputFilename;
    int replications;
    int replicationShift;	
    int robotsCount;
    int basestationsCount;	
    float maxTime;
    float aquariumRadius;
    ExperimentParameters ()
	{
	    replications = 1;
	    replicationShift = 0;
	    robotsCount = 15;
	    basestationsCount = 2;
	    maxTime = 60.0;
	    aquariumRadius = 2.0;
	}
    
    void LoadXML (TiXmlElement* el)
	{
	    const char* res;
	    double dvalue;
	    int ivalue;
	    res = el->Attribute ("replications", &ivalue); if (res) replications = ivalue;
	    res = el->Attribute ("replicationShift", &ivalue); if (res) replicationShift = ivalue;
	    res = el->Attribute ("robotsCount", &ivalue); if (res) robotsCount = ivalue;
	    res = el->Attribute ("basestationsCount", &ivalue); if (res) basestationsCount = ivalue;
	    res = el->Attribute ("maxTime", &dvalue); if (res) maxTime = dvalue;
	    res = el->Attribute ("aquariumRadius", &dvalue); if (res) aquariumRadius = dvalue;
	    res = el->Attribute ("outputFilename"); if (res) outputFilename = std::string(res);
	}
};


class Experiment : public Service
{
public:

    PhysicsBullet* physics;
    RenderOpenGL* render;
    unsigned int ticks;
    int currentReplication;

    std::vector<RobotLily*> robots;
    std::vector<BaseStation*> basestations;
		
    ExperimentParameters* paramsExp;
    ControllerDiscriminationParameters* paramsCtrl;

    struct Results
    {
	int replication;
	float time;
	int base[10]; // assume max 10 basestations ...
    };
    std::vector<Results> results;
    

    Experiment (Simulator* s, bool graphics, string outputFilename, int replications = -1, int replicationShift = -1, string paramsFilename = "");
    ~Experiment ();

    void LoadParameters (string filename);
    
    void Reset ();
    void Step (float time, float timestep);
    void Run();
};


#endif
