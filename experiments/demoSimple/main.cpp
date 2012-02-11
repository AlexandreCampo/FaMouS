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

// This example is the most simple possible with underwater vehicles.
// A group robots make random walks and avoid obstacles in an aquarium. 


#include "Simulator.h"
#include "Experiment.h"

#include <iostream>

	
int main(int argc,char** argv)
{
    Simulator* simulator = new Simulator ();
    simulator->SetTimestep (0.005); // the default at 0.05s is not accurate enough for cylinders...
    Experiment* exp = new Experiment (simulator);

    exp->Run();    

    delete simulator;
}

