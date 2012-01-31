// Copyright (C) 2011, 2012 by Antonio El Khoury.
//
// This file is part of the orb-planner.
//
// orb-planner is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// orb-planner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with orb-planner.  If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE plan

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoUtility/kitParameterMap.h>
#include <KineoModel/KineoModel.h>
#include <KineoController/KineoController.h>
#include <KineoModuleManager/KineoModuleManager.h>
#include <kprParserXML/KineoParserXML.h>
#include <KineoWorks2/kwsDiffusingRdmBuilder.h>

#include "parser.cc"

using boost::test_tools::output_test_stream;

// Define where the device loading libraries are. Make sure you load
// the correct ones (depending on whether you're using the release or
// debug libraries).
#define KINEO_INSTALL_DIR "/home/aelkhour/profiles/kitelab-2.06-i686-linux-ubuntu-10.04/install/stable/kineo-2.06"

#define KINEODEVICEPARSING_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviceParsingd.so"
#define KINEODEVICEBASE_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviceBased.so"
#define KINEODEVICE_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviced.so"

// This is the main program.
BOOST_AUTO_TEST_CASE (plan)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // ----------------------------------------------------------------

  // Initialize module manager to allow parsing device.
  CkppModuleManagerShPtr moduleManager = CkppModuleManager::create ();
  moduleManager->addModuleFile (KINEODEVICEPARSING_SO);
  moduleManager->addModuleFile (KINEODEVICEBASE_SO);
  moduleManager->addModuleFile (KINEODEVICE_SO);

  CkprParserManager::defaultManager ()->moduleManager (moduleManager);

  moduleManager->initializeModules ();

  if (moduleManager->countModules () == 0)
    std::cout << "No module loaded. "
      "Are you sure you the modules paths are correct?" << std::endl;
  else
    for (unsigned int i=0; i < moduleManager->countModules (); i++)
      std::cout << "Module " << i << ": "
		<< moduleManager->module (i)->name () << std::endl;
  
  // Create a parser instance.
  CkprParserManagerShPtr parser = CkprParserManager::defaultManager ();

  CkppDocumentShPtr document =
    CkppDocument::create (CkprParserManager::defaultManager()
			  ->moduleManager ());
  CkppComponentFactoryRegistryShPtr registry
    = document->componentFactoryRegistry ();

  // ----------------------------------------------------------------

  // Load the environment.
  std::string obstacleFilename ("./obstacle.kxml");

  CkppModelTreeShPtr modelTree = CkppModelTree::create ();
  
  parseFile (obstacleFilename,
  	     parser,
  	     registry,
  	     modelTree);

  // ----------------------------------------------------------------

  // Load a robot in the same scene.
  CkppComponentShPtr robotModelTreeComponent;
  std::string robotFilename("./robot.kxml");
  
  parseFile (robotFilename,
  	     parser,
  	     registry,
  	     modelTree);

  printComponent (modelTree);
  
  // ----------------------------------------------------------------

  // Set initial pose of the robot.

  // Set target pose of the robot.

  // ----------------------------------------------------------------

  // Plan a path.
  // CkwsRoadmapShPtr roadmap = CkwsRoadmap->create (robot->penetration ());
  // CkwsRoadmapBuilderShPtr roadmapBuilder = CkwsDiffusingRdmBuilder::create (roadmap);
  // roadmapBuilder->diffuseFromProblemGoal (true);
  
  // CkwsPathShPtr solutionPath = CkwsPath::createWithDirectPath (robot);
  // roadmapBuilder->solveProblem (startConfig, goalConfig, solutionPath);

  // ----------------------------------------------------------------

  // Read path from MUSCOD Data and create kineo path.

  // ----------------------------------------------------------------

  // Check new path for collisions.
}
