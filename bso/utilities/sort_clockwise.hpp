#ifndef SORT_CLOCKWISE_HPP
#define SORT_CLOCKWISE_HPP


#include <bso/utilities/geometry.hpp>
#include <iostream>
#include <vector>

namespace bso { namespace utilities {

	/*
	 * The functions in this file are used to sort points in a plane in a clockwise manner
	 */
	
	std::vector<bso::spatial_design::conformal::cf_vertex> sortClockwisePlanar(std::vector<bso::spatial_design::conformal::cf_vertex>& pPtr);
	std::vector<bso::utilities::geometry::vertex> sortClockwisePlanar(std::vector<bso::utilities::geometry::vertex>& pPtr);
	
	
} // namespace utilities
} // namespace bso

#include <bso/utilities/sort_clockwise.cpp>

#endif // SORT_CLOCKWISE_HPP

