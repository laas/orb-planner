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

using boost::test_tools::output_test_stream;

// Define where the device loading libraries are. Make sure you load
// the correct ones (depending on whether you're using the release or
// debug libraries).
#define KINEO_INSTALL_DIR "/home/shouzang/profiles/ubuntu-10.04/install/stable/kineo-2.06"

#define KINEODEVICEPARSING_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviceParsingd.so"
#define KINEODEVICEBASE_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviceBased.so"
#define KINEODEVICE_SO KINEO_INSTALL_DIR"/bin/modulesd/KineoDeviced.so"

// Define function that prints a component tree hierarchy starting
// from a root component.
void printComponent (const CkppComponentShPtr& i_component,
		     std::set<int> lastChildDepths = std::set<int> (),
		     int depth = 0);

void printComponent (const CkppComponentShPtr& i_component,
		     std::set<int> lastChildDepths,
		     int depth)
{
  std::cout << i_component->name () << std::endl;
  for (unsigned i = 0;
       i < i_component->countChildComponents ();
       ++i)
    {
      for (int j = 0; j < depth; ++j)
	if (lastChildDepths.find (j) != lastChildDepths.end ())
	  std::cout << "    ";
	else
	  std::cout << "|   ";
      std::cout << "|-> ";

      if (i == i_component->countChildComponents () - 1)
	lastChildDepths.insert (depth);

      printComponent (i_component->childComponent (i),
		      lastChildDepths,
		      depth + 1);
      lastChildDepths.erase (depth);
    }
}

void parseFile (const std::string& fileName,
		const CkprParserManagerShPtr& parser,
		const CkppComponentFactoryRegistryShPtr& registry,
		CkppModelTreeShPtr& modelTree);

void parseFile (const std::string& fileName,
		const CkprParserManagerShPtr& parser,
		const CkppComponentFactoryRegistryShPtr& registry,
		CkppModelTreeShPtr& modelTree)
{
  assert (!!modelTree && "Null pointer to modelTree.");

  CkppComponentShPtr parsedModelTreeComponent;

  std::cout << "Parsing file..." << std::endl;
  if (parser->loadComponentFromFile (fileName,
				     parsedModelTreeComponent,
				     registry,
				     CkitParameterMap::create ()) != KD_OK)
    {
      CkprParser::Error error = parser->lastError();
      std::string message = "failed to read " + fileName + ".\n"
	+ std::string(error.errorMessage());
      std::cout << message << std::endl;
      return;
    }

  CkppModelTreeShPtr parsedModelTree =
    KIT_DYNAMIC_PTR_CAST(CkppModelTree, parsedModelTreeComponent);
  if (!parsedModelTree)
    {
      std::cout << "Main node is not of type CkppModelTree." << std::endl;
      return;
    }

  std::cout << "Retrieving geometry node..." << std::endl;
  if (parsedModelTree->geometryNode ())
    {
      for(unsigned int i = 0;
	  i < parsedModelTree->geometryNode ()->countChildComponents ();
	  i++)
	{
	  CkppGeometryComponentShPtr geometryComponent =
	    KIT_DYNAMIC_PTR_CAST(CkppGeometryComponent,
				 parsedModelTree->geometryNode ()
				 ->childComponent (i));
	  CkppGeometryComponentWkPtr wkptr;
	  geometryComponent->parent (wkptr);
	  modelTree->geometryNode ()->addChildComponent (geometryComponent);
	}
    }

  std::cout << "Retrieving group node..." << std::endl;
  if (parsedModelTree->groupNode ())
    {
      for(unsigned int i = 0;
	  i < parsedModelTree->groupNode()->countChildComponents();
	  i++)
	{
	  CkppGroupComponentShPtr groupComponent =
	    KIT_DYNAMIC_PTR_CAST(CkppGroupComponent,
				 parsedModelTree->groupNode ()
				 ->childComponent (i));
	  CkppGroupComponentWkPtr wkptr;
	  groupComponent->parent (wkptr);
	  modelTree->groupNode ()->addChildComponent (groupComponent);
	}
    }

  std::cout << "Retrieving device node..." << std::endl;
  if (parsedModelTree->deviceNode ())
    {
      for(unsigned int i = 0;
	  i < parsedModelTree->deviceNode()->countChildComponents();
	  i++)
	{
	  CkppDeviceComponentShPtr deviceComponent =
	    KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,
				 parsedModelTree->deviceNode ()
				 ->childComponent (i));
	  CkppDeviceComponentWkPtr wkptr;
	  deviceComponent->parent (wkptr);
	  modelTree->deviceNode ()->addChildComponent (deviceComponent);
	}
    }
  
  std::cout << "Retrieving path node..." << std::endl;
  if (parsedModelTree->pathNode ())
    {
      for(unsigned int i = 0;
	  i < parsedModelTree->pathNode ()->countChildComponents ();
	  i++)
	{
	  CkppPathComponentShPtr pathComponent =
	    KIT_DYNAMIC_PTR_CAST (CkppPathComponent,
				  parsedModelTree->pathNode ()
				  ->childComponent (i));
	  CkppPathComponentWkPtr wkptr;
	  pathComponent->parent (wkptr);
	  modelTree->pathNode ()->addChildComponent (pathComponent);
	}
    }

  std::cout << "Retrieving clipping plane node..." << std::endl;
  if (parsedModelTree->clippingPlaneNode ())
    {
      for(unsigned int i = 0;
	  i < parsedModelTree->clippingPlaneNode ()->countChildComponents ();
	  i++)
	{
	  CkppClippingPlaneShPtr clippingPlaneComponent =
	    KIT_DYNAMIC_PTR_CAST (CkppClippingPlane,
				  parsedModelTree->clippingPlaneNode ()
				  ->childComponent (i));
	  CkppClippingPlaneWkPtr wkptr;
	  clippingPlaneComponent->parent (wkptr);
	  modelTree->clippingPlaneNode ()
	    ->addChildComponent (clippingPlaneComponent);
	}
    }

  std::cout << "Retrieving camera node..." << std::endl;
  if (parsedModelTree->cameraNode ())
    {
      for(unsigned int i = 0;
	  i < parsedModelTree->cameraNode ()->countChildComponents ();
	  i++)
	{
	  CkppCameraShPtr cameraComponent =
	    KIT_DYNAMIC_PTR_CAST (CkppCamera,
				  parsedModelTree->cameraNode ()
				  ->childComponent (i));
	  CkppCameraWkPtr wkptr;
	  cameraComponent->parent (wkptr);
	  modelTree->cameraNode ()
	    ->addChildComponent (cameraComponent);
	}
    }
}

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
  CkppComponentShPtr obstacleModelTreeComponent;
  std::string
    obstacleFilename(KINEO_INSTALL_DIR"/share/demos/KXML/labyrinth.kxml");

  // if (parser->loadComponentFromFile (obstacleFilename,
  // 				     obstacleModelTreeComponent,
  // 				     registry,
  // 				     CkitParameterMap::create ()) != KD_OK) {
  //   CkprParser::Error error = parser->lastError();
  //   std::string message = "failed to read " + obstacleFilename + ".\n"
  //     + std::string(error.errorMessage());
  //   std::cout << message << std::endl;
  //   return;
  // }

  // CkppModelTreeShPtr modelTree =
  //   KIT_DYNAMIC_PTR_CAST(CkppModelTree, obstacleModelTreeComponent);
  // if (!modelTree)
  //   {
  //     std::cout << "Main node is not of type CkppModelTree." << std::endl;
  //     return;
  //   }

  // ----------------------------------------------------------------

  // Load a robot in the scene.
  CkppComponentShPtr robotModelTreeComponent;
  std::string
    robotFilename("./BridgeDevice.kxml");

  // if (parser->loadComponentFromFile (robotFilename,
  // 				     robotModelTreeComponent,
  // 				     registry,
  // 				     CkitParameterMap::create ()) != KD_OK) {
  //   CkprParser::Error error = parser->lastError();
  //   std::string message = "failed to read " + robotFilename + ".\n"
  //     + std::string(error.errorMessage());
  //   std::cout << message << std::endl;
  //   return;
  // }

  // for (unsigned i = 0;
  //      i < robotModelTreeComponent->countChildComponents ();
  //      ++i)
  //   {
  //     modelTree->addChild
  // 	robotModelTreeComponent->childComponent (i);
  //   }

  CkppModelTreeShPtr modelTree = CkppModelTree::create ();
  
  parseFile (obstacleFilename,
  	     parser,
  	     registry,
  	     modelTree);

  printComponent (modelTree);

  std::cout << std::endl;
  
  parseFile (robotFilename,
  	     parser,
  	     registry,
  	     modelTree);

  printComponent (modelTree);
  
  // ----------------------------------------------------------------

  // Set initial pose of the robot using (simple) IK.

  // Set target pose of the robot using (simple) IK.

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
