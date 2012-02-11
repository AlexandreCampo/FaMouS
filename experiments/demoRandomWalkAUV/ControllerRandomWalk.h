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

#ifndef CONTROLLER_RANDOMWALK_H
#define CONTROLLER_RANDOMWALK_H

#include "Controller.h"
#include "RobotLily.h"


#define EXPLORE 1
#define TURN 2 

struct ControllerRandomWalkParameters
{
    float obstacleAvoidanceThreshold;
    float maxProximitySensing;
    float exploreMeanDuration;
    float speed;

    ControllerRandomWalkParameters ()
	{
	    obstacleAvoidanceThreshold = 0.1;
	    maxProximitySensing = 0.12;
	    exploreMeanDuration = 15.0;
	    speed = 0.5;
	}
};

class ControllerRandomWalk : public Controller
{
public : 
    RobotLily* lily;
    
    ControllerRandomWalk (RobotLily* l);
    ~ControllerRandomWalk ();

    void SetParameters (ControllerRandomWalkParameters* p);

    void Step (float time, float timestep);

    void StateExploreInit ();
    void StateExplore ();
    void StateTurnInit (int previousState, float angle);
    void StateTurn ();
    void Reset ();
    bool ObstacleAvoidance ();

    
private:

    // state handling
    int state;

    // time
    float time;
    float timestep;

    ControllerRandomWalkParameters params;

    // state working variables
    float exploreDuration;
    float exploreStartTime;

    int turnPreviousState;
    float turnDuration;
    float turnStartTime;
    float turnSign;    
};


#endif
