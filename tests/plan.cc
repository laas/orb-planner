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

#include <iomanip>

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoUtility/kitParameterMap.h>
#include <KineoModel/KineoModel.h>
#include <KineoController/KineoController.h>
#include <KineoModuleManager/KineoModuleManager.h>
#include <kprParserXML/KineoParserXML.h>
#include <KineoWorks2/kwsDiffusingRdmBuilder.h>
#include <kwsKcd2/kwsKCDBody.h>

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
  std::string robotFilename("./robot.kxml");

  parseFile (robotFilename,
  	     parser,
  	     registry,
  	     modelTree);

  printComponent (modelTree);

  // ----------------------------------------------------------------

  // Assuming there is one robot in the model tree, retrieve it.
  assert (modelTree->deviceNode ()->countChildComponents () == 1
	  && "Wrong number of devices in model tree, expected 1.");
  CkppDeviceComponentShPtr robot
    = KIT_DYNAMIC_PTR_CAST (CkppDeviceComponent,
			    modelTree->deviceNode ()->childComponent (0));
  assert (!!robot && "Null pointer to robot.");

  // ----------------------------------------------------------------

  // A body of the robot is said to be in collision if any out the
  // outer objects collide with any of the inner objects. Usually you
  // have only one inner objet, and multiple outer objects, such as
  // objects from the environment, or other bodies of the same robot.

  // Store all geometry component references of device to avoid adding
  // them as outer objects. This will allows us later on to add only
  // obstacles. Self-collision pairs can be then defined separately.
  std::vector<CkppSolidComponentRefShPtr> solidComponentRefVector;
  robot->getSolidComponentRefVector (solidComponentRefVector);

  // Add all activated obstacles in model tree as outer objects of
  // each joint body.
  for (unsigned i = 0;
       i < modelTree->geometryNode ()->countChildComponents ();
       ++i)
    {
      CkppSolidComponentShPtr solidComponent
	= KIT_DYNAMIC_PTR_CAST (CkppSolidComponent,
				modelTree->geometryNode ()->childComponent (i));
      assert (!!solidComponent && "Null pointer to solidComponent.");

      unsigned j = 0;
      bool isSolidComponentInRobot = false;
      while (j < solidComponentRefVector.size ()
	     && isSolidComponentInRobot == false)
	{
	  CkppSolidComponentShPtr robotSolidComponent
	    = solidComponentRefVector[j]->referencedSolidComponent ();
	  assert (!!robotSolidComponent
		  && "Null pointer to robot solid component.");
	  if (robotSolidComponent == solidComponent)
	    {
	      std::cout << "solid component " << i
			<< " matches body in joint " << j << std::endl;
	      isSolidComponentInRobot = true;
	    }
	  ++j;
	}

      if (!isSolidComponentInRobot && solidComponent->isActivated ())
	for (unsigned j = 0; j < solidComponentRefVector.size (); ++j)
	  {
	    CkcdObjectShPtr object
	      = KIT_DYNAMIC_PTR_CAST (CkcdObject,
				      solidComponent);

	    CkwsDevice::TJointVector jointVector;
	    robot->getJointVector (jointVector);

	    CkwsKCDBodyShPtr body
	      = KIT_DYNAMIC_PTR_CAST (CkwsKCDBody,
				      jointVector[j]->attachedBody ());
	    std::vector<CkcdObjectShPtr> outerObjects = body->outerObjects ();
	    outerObjects.push_back (object);
	    body->outerObjects (outerObjects);
	}
    }

  // To avoid self-collision, add robot bodies as outer bodies. Here
  // we add all collision pairs. This is optional, and it may be nice
  // to find automatically which collision pairs should be taken into
  // consideration.
  for (unsigned i = 0; i < solidComponentRefVector.size (); ++i)
    {
      CkppSolidComponentShPtr robotSolidComponent1
	= solidComponentRefVector[i]->referencedSolidComponent ();
      assert (!!robotSolidComponent1
	      && "Null pointer to robot solid component 1.");
      if (robotSolidComponent1->isActivated ())
	{
	  CkwsDevice::TJointVector jointVector;
	  robot->getJointVector (jointVector);
	  CkwsKCDBodyShPtr body
	    = KIT_DYNAMIC_PTR_CAST (CkwsKCDBody,
				    jointVector[i]->attachedBody ());
	  std::vector<CkcdObjectShPtr> outerObjects = body->outerObjects ();

	  for (unsigned j = i + 2; j < solidComponentRefVector.size (); ++j)
	    {
	      CkppSolidComponentShPtr robotSolidComponent
		= solidComponentRefVector[j]->referencedSolidComponent ();
	      assert (!!robotSolidComponent
		      && "Null pointer to robot solid component.");
	      if (robotSolidComponent->isActivated ())
		{
		  CkcdObjectShPtr robotObject
		    = KIT_DYNAMIC_PTR_CAST (CkcdObject,
					    robotSolidComponent);
		  assert (!!robotObject
			  && "Null pointer to robot object.");
		  outerObjects.push_back (robotObject);
		}
	    }

	  body->outerObjects (outerObjects);
	}
    }

  // Print inner and outer objects for each joint of the robot.
  CkwsDevice::TJointVector jointVector;
  robot->getJointVector (jointVector);
  for (unsigned i = 0; i < jointVector.size (); ++i)
    {
      std::cout << "Joint " << i << std::endl;
      CkwsKCDBodyShPtr body
	= KIT_DYNAMIC_PTR_CAST (CkwsKCDBody,
				jointVector[i]->attachedBody ());

      std::vector<CkcdObjectShPtr> innerObjects = body->innerObjects ();
      std::vector<CkcdObjectShPtr> outerObjects = body->outerObjects ();

      std::cout << "Number of inner objects: "
		<< innerObjects.size () << std::endl;
      std::cout << "Number of outer objects: "
		<< outerObjects.size () << std::endl;
    }

  // ----------------------------------------------------------------

  // Set initial pose of the robot.
  CkwsConfig startConfig (robot);
  assert (robot->countDofs () == 6 && "Incorrect number of dofs, expected 6.");
  std::vector<double> startDofValues (6);
  startDofValues[0] = 0.01;
  startDofValues[1] = - M_PI / 2;
  startDofValues[2] = 0.;
  startDofValues[3] = 0.;
  startDofValues[4] = 0.;
  startDofValues[5] = 0.;
  startConfig.setDofValues (startDofValues);
  assert (startConfig.isValid () == true
	  && "Start configuration is not collision free, should be.");

  // Set target pose of the robot.
  CkwsConfig goalConfig (robot);
  std::vector<double> goalDofValues (6);
  goalDofValues[0] = M_PI;
  goalDofValues[1] = - M_PI / 2;
  goalDofValues[2] = 0.;
  goalDofValues[3] = 0.;
  goalDofValues[4] = 0.;
  goalDofValues[5] = 0.;
  goalConfig.setDofValues (goalDofValues);
  assert (goalConfig.isValid () == true
	  && "Goal configuration is not collision free, should be.");

  // ----------------------------------------------------------------

  // Create linear steering method. A direct path created with this
  // steering method uses linear interpolation to compute a
  // configuration between the direct path start and end
  // configuration.
  CkwsSteeringMethodShPtr steeringMethod = CkwsSMLinear::create ();
  robot->steeringMethod (steeringMethod);

  // Create roadmap builder, i.e. the motion planning algorithm.
  CkwsRoadmapShPtr roadmap = CkwsRoadmap::create (robot);
  CkwsDiffusingRdmBuilderShPtr roadmapBuilder
    = CkwsDiffusingRdmBuilder::create (roadmap);
  roadmapBuilder->penetration (10.);
  roadmapBuilder->diffuseFromProblemGoal (true);

  // Create initial path from start and goal configurations.
  CkwsPathShPtr initPath = CkwsPath::create (robot);
  CkwsConfigShPtr startConfigShPtr = CkwsConfig::create (startConfig);
  CkwsConfigShPtr goalConfigShPtr = CkwsConfig::create (goalConfig);
  assert (!!startConfigShPtr && "Null pointer to start config.");
  assert (!!goalConfigShPtr && "Null pointer to goal config.");
  assert (!goalConfig.isEquivalent (startConfig)
	  && "Goal and start config are equivalent, must be different.");
  initPath->appendDirectPath (startConfigShPtr, goalConfigShPtr);

  assert (!!initPath && "Null pointer to initial path.");
  assert (initPath->countDirectPaths () == 1
  	  && "Wrong number of direct paths in initial path, expected 1.");
  assert (initPath->validateWithPenetration (10.) == false
	  && "Init path is not collliding, no point in planning.");

  // Plan collision-free path from start and goal configurations.
  std::cout << "Solving... " << std::endl;
  CkwsPathShPtr solutionPath = CkwsPath::createCopy (initPath);
  if (KD_OK == roadmapBuilder
      ->solveProblem (startConfig, goalConfig, solutionPath))
    std::cout << "OK" << std::endl;
  else
    std::cout << "ERROR" << std::endl;

  // Print solution path.
  std::cout << "solution path: " << std::endl;
  for (unsigned i = 0; i < solutionPath->countConfigurations (); ++i)
    {
      CkwsConfig config (robot);
      solutionPath->getConfiguration (i, config);
      std::cout << std::setw (5) << "q(" << i << "):";
      for (unsigned j = 0; j < config.size (); ++j)
	std::cout << std::setw (13) << config.dofValue (j);
      std::cout << std::endl;
    }

  // Check for collisions on solution path. Penetration is a kind of
  // tolerance when checking for collisions. The lowest it is, the
  // safer the result, but the slower the validation.
  assert (solutionPath->validateWithPenetration (10.) == true
	  && "Solution path is collliding, there is a problem.");

  // ----------------------------------------------------------------

  // Optimize path using Kineo random optimizer to shorten path
  // length. It is a good idea to use this optimizer on the solution
  // path to have a nicer initial guess for MUSCOD (even if the
  // objective function does not take path length into consideration).
  CkwsPathShPtr optimizedPath = CkwsPath::createCopy (solutionPath);
  CkwsRandomOptimizerShPtr optimizer = CkwsRandomOptimizer::create ();
  std::cout << "Optimizing..." << std::endl;
  if (KD_OK == optimizer->optimizePath (optimizedPath))
    std::cout << "OK" << std::endl;
  else
    std::cout << "ERROR" << std::endl;

  // Print solution path.
  std::cout << "optimized path: " << std::endl;
  for (unsigned i = 0; i < optimizedPath->countConfigurations (); ++i)
    {
      CkwsConfig config (robot);
      optimizedPath->getConfiguration (i, config);
      std::cout << std::setw (5) << "q(" << i << "):";
      for (unsigned j = 0; j < config.size (); ++j)
	std::cout << std::setw (13) << config.dofValue (j);
      std::cout << std::endl;
    }

  assert (optimizedPath->validateWithPenetration (10.) == true
	  && "Solution path is collliding, there is a problem.");

  // ----------------------------------------------------------------

  // Read path from MUSCOD Data and create kineo path.

  // You should in this case parse MUSCOD data, create
  // configurations, create direct paths with each pair of
  // configurations, and append each direct path to a path starting
  // with an empty one.
}
