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

#ifndef CONTROLLER_DISCRIMINATION_H
#define CONTROLLER_DISCRIMINATION_H

#include "Controller.h"
#include "RobotLily.h"
#include <tinyxml.h>


#define EXPLORE 1
#define TURN 2 
#define REST 3
#define BACK_TO_SHELTER 4

struct ControllerDiscriminationParameters
{
    float theta;
    float rho;
    float decisionDelay;
    float obstacleAvoidanceThreshold;
    float maxProximitySensing;
    float obstacleAvoidanceSpeed;
    float exploreMeanDuration;
    float exploreSpeed;    
    float turnSpeed;
    float breakSpeed;
    float restSpeed;
    float collisionsDecisionDelay;

    ControllerDiscriminationParameters ()
	{
	    theta = 0.75;
	    rho = 1000.0;
	    decisionDelay = 30.0;
	    obstacleAvoidanceThreshold = 0.1;
	    maxProximitySensing = 0.12;
	    obstacleAvoidanceSpeed = 0.99;
	    exploreMeanDuration = 15.0;
	    exploreSpeed = 0.5;
	    turnSpeed = 0.1;
	    breakSpeed = 0.2;
	    restSpeed = 0.5;
	    collisionsDecisionDelay = 5.0;
	}

    void LoadXML (TiXmlElement* el)
	{
	    // extract all params one by one ...
	    const char* res;
	    double value;
	    int ivalue;
	    
	    res = el->Attribute("theta", &value); if (res) theta = value;
	    res = el->Attribute("rho", &value); if (res) rho = value;
	    res = el->Attribute("decisionDelay", &value); if (res) decisionDelay = value;
	    res = el->Attribute("obstacleAvoidanceThreshold", &value); if (res) obstacleAvoidanceThreshold = value;
	    res = el->Attribute("maxProximitySensing", &value); if (res) maxProximitySensing = value;
	    res = el->Attribute("obstacleAvoidanceSpeed", &value); if (res) obstacleAvoidanceSpeed = value;
	    res = el->Attribute("exploreMeanDuration", &value); if (res) exploreMeanDuration = value;
	    res = el->Attribute("exploreSpeed", &value); if (res) exploreSpeed = value;
	    res = el->Attribute("turnSpeed", &value); if (res) turnSpeed = value;
	    res = el->Attribute("breakSpeed", &value); if (res) breakSpeed = value;
	    res = el->Attribute("restSpeed", &value); if (res) restSpeed = value;
	    res = el->Attribute("collisionsDecisionDelay", &value); if (res) collisionsDecisionDelay = value;
	}
};

class ControllerDiscrimination : public Controller
{
public : 
    RobotLily* lily;
    
    ControllerDiscrimination (RobotLily* l);
    ~ControllerDiscrimination ();

    void SetParameters (ControllerDiscriminationParameters* p);

    void Step (float time, float timestep);

    void StateExploreInit ();
    void StateExplore ();
    void StateRestInit ();
    void StateRest ();
    void StateTurnInit (int previousState, float angle);
    void StateTurn ();
    bool IsAtBase ();
    void Reset ();
    bool ObstacleAvoidance ();
    /* void StateBackToShelterInit (); */
    /* void StateBackToShelter (); */

    
private:

    // state handling
    int state;

    // time
    float time;
    float timestep;

    ControllerDiscriminationParameters params;

    // state working variables
    float exploreDuration;
    float exploreStartTime;

    int turnPreviousState;
    float turnDuration;
    float turnStartTime;
    float turnSign;
    
    /* float restMeanDuration; */
    float restDuration;
    float restStartTime;
    bool restEscapeFromShelter;
    float restLastSignalTime;
    
    float collisionsDecisionLastTime;

    float breakSpeed;
};


#endif
