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


/**
 * \brief Declaration of Capsule.
 */

#ifndef KCD_CAPSULE_HH_
# define KCD_CAPSULE_HH_

# include <kcd2/kcdInterface.h>

# include "hpp/geometry/collision/fwd.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      class Capsule : public CkcdGeometrySubElement
      {
      public:
	/// Create a new capsule.
	///
	/// \param tree Tree to which the element belongs
	///	\param index Sphere index within the tree
	/// \param endPoint1 First end point of the capsule axis
	///	\param endPoint2 Second end point of the capsule axis
	///	\param radius Radius of the capsule
	///
	/// \return New capsule
	static CapsuleShPtr create (const TestTreeCapsuleShPtr testTree,
				    unsigned int index,
				    const CkcdPoint& endPoint1,
				    const CkcdPoint& endPoint2,
				    kcdReal radius);
    
	/// Destructor.
	virtual ~Capsule ();

	/// Get the parent geometry.
	virtual CkcdGeometryConstShPtr geometry () const;

	/// Get index in test tree.
	unsigned int index () const;

	/// Get the coordinates of the capsule's axis first end point
	/// relative to the root of the tree.
	///
	/// \return center coordinate
	CkcdPoint endPoint1 () const;

	/// Get the coordinates of the capsule's axis second end point
	/// relative to the root of the tree.
	///
	/// \return center coordinate
	CkcdPoint endPoint2 () const;

	/// Retrieves the radius of the capsule.
	///
	/// \return radius
	kcdReal radius () const;

      protected:
	/// Constructor.
	Capsule (TestTreeCapsuleShPtr testTree);

	/// Initialize.
	ktStatus init (const CapsuleWkPtr& weakPtr,
		       unsigned int index,
		       const CkcdPoint& endPoint1,
		       const CkcdPoint& endPoint2,
		       kcdReal radius);

	/// Get capsule test tree.
	TestTreeCapsuleShPtr testTreeCapsule () const;

      private:
	CapsuleWkPtr weakPtr_;

	unsigned int index_;
	CkcdPoint endPoint1_;
	CkcdPoint endPoint2_;
	kcdReal radius_;
      };

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.

#endif //! KCD_CAPSULE_HH_
