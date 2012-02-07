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
#include "Experiment.h"

#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;
	
int main(int argc,char** argv)
{
    // Parse command line with lib boost
    po::options_description desc("\nAllowed options");
    desc.add_options()
	("help,h", "produce help message")
        ("graphics,g", "activate graphic output")
        ("output,o", po::value<string>(), "output results to a file")
        ("replications,r", po::value<int>(), "replicate the experiment n times")
        ("shift,s", po::value<int>(), "replication count shift (or count start)")
        ("parameters,p", po::value<string>(), "read parameters from an xml file")
	;
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    
    
    if (vm.count("help")) 
    {
	std::cout << desc << "\n";
	return 1;
    }

    string paramsFilename;
    if (vm.count("parameters")) 
    {
	paramsFilename = vm["parameters"].as<string>();
	std::cout << "Reading parameters from file " << paramsFilename << std::endl;
    } 

    bool graphics = false;
    if (vm.count("graphics")) 
    {
	std::cout << "Graphic output enabled " << std::endl;
	graphics = true;	
    } 

    string outputFilename;
    if (vm.count("output")) 
    {
	outputFilename = vm["output"].as<string>();
	std::cout << "Output results to the file " << outputFilename << std::endl;
    } 

    int replications = -1;
    if (vm.count("replications")) 
    {
	replications = vm["replications"].as<int>();
	std::cout << "Making " << replications << " replications" << std::endl;
    } 

    int replicationShift = -1;
    if (vm.count("shift")) 
    {
	replicationShift = vm["shift"].as<int>();
    } 

    // setup and run simulated experiment
    Simulator* simulator = new Simulator ();
    std::cout << "Check possible command line switches with --help !" << std::endl << std::endl;

    Experiment* exp = new Experiment (simulator, graphics, outputFilename, replications, replicationShift, paramsFilename);
    exp->Run();    

    delete simulator;

    return 0;
}

