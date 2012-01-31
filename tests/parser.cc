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

#include <KineoUtility/kitParameterMap.h>
#include <KineoModel/KineoModel.h>
#include <KineoController/KineoController.h>
#include <KineoModuleManager/KineoModuleManager.h>
#include <kprParserXML/KineoParserXML.h>
#include <KineoWorks2/kwsDiffusingRdmBuilder.h>

/// \brief Parse a file and add contents to model tree.
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
	  CkppSolidComponentShPtr solidComponent =
	    KIT_DYNAMIC_PTR_CAST(CkppSolidComponent,
				 parsedModelTree->geometryNode ()
				 ->childComponent (i));
	  CkppSolidComponentWkPtr wkptr;
	  solidComponent->parent (wkptr);
	  modelTree->geometryNode ()->addChildComponent (solidComponent);
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

/// \brief Print a component tree hierarchy starting from a root
/// component.
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
