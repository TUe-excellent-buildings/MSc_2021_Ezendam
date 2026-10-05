#ifndef SORT_CLOCKWISE_CPP
#define SORT_CLOCKWISE_CPP

#include <algorithm>
#include <iostream>
#include <vector>
#include <math.h>

namespace bso { namespace utilities {

	
	std::vector<bso::spatial_design::conformal::cf_vertex> sortClockwisePlanar(std::vector<bso::spatial_design::conformal::cf_vertex>& pPtr)
	{
		// background information: std::vector<cf_vertex>& intsecV
		//https://stackoverflow.com/questions/47949485/sorting-a-list-of-3d-points-in-clockwise-order
		//https://stackoverflow.com/questions/14370636/sorting-a-list-of-3d-coplanar-points-to-be-clockwise-or-counterclockwise
		
		
		// find centroid of points
		bso::spatial_design::conformal::cf_vertex centroid;
		bso::spatial_design::conformal::cf_vertex temp;
				
		for (int i = 0; i<(pPtr.size()+1); i++)
		{
			temp = temp+pPtr[i];
		}
		std::cout << " " << std::endl;
	
		temp = temp/pPtr.size();
		centroid = temp; //  calculation errors should be removed (zerro is no zerro)
		
		
		// Determine the normal of plane
		const double& tol = 1e-3; //mGeometryModel->tolerance(); //
		bso::utilities::geometry::vector normal;
		bool allVerticesOnOneLine = true;
		for (unsigned int i = 1; i < pPtr.size()-1; i++)
		{
			bso::utilities::geometry::vector v1 = {pPtr[i]-pPtr[0]};
			bso::utilities::geometry::vector v2 = {pPtr[i+1]-pPtr[0]};
			if (v1.isZero(tol) || v2.isZero(tol)) continue;
			if (!v1.isParallel(v2,tol))
			{
				allVerticesOnOneLine = false;
				normal = v1.cross(v2).normalized();
				break;
			}
		}
		
		if(allVerticesOnOneLine == true)
		{
			std::stringstream errorMessage;
						errorMessage << "\nError, can't find the normal when sort_clockwise of points \n"
												 << "(bso/utilities/sort_clockwise)." << std::endl;
						throw std::runtime_error(errorMessage.str());
		}
		
		
		// angle calculation, defining sorting mechanics
		std::vector<double> theta;
		std::vector<double> thetaSorted;
		std::vector<double> clockorder;
		std::vector <bso::utilities::geometry::vector> centroidVector;
		
		for (int i=0; i< pPtr.size(); i++)
		{
			centroidVector.push_back(pPtr[i]-centroid);
		}
		
		for (int i=0; i< centroidVector.size(); i++)
		{			
			clockorder.push_back(normal.dot(centroidVector[0].cross(centroidVector[i])));
			
			if(centroidVector[0].isParallel(centroidVector[i]))
			{
				if((centroidVector[0]).isCodirectional(centroidVector[i]))
				{
					theta.push_back(0);
					thetaSorted.push_back(0);
				}
				else if (!((centroidVector[0]).isCodirectional(centroidVector[i])))
				{
					theta.push_back(3.1416); //180 graden
					thetaSorted.push_back(3.1416);
				}
			}
			else
			{
				if(normal.dot(centroidVector[0].cross(centroidVector[i])) < 0) // clockwise side
				{
					theta.push_back(acos(centroidVector[0].dot((centroidVector[i])/(centroidVector[0].norm()*centroidVector[i].norm()))));
					thetaSorted.push_back(acos(centroidVector[0].dot((centroidVector[i])/(centroidVector[0].norm()*centroidVector[i].norm()))));
				}
				else if (normal.dot(centroidVector[0].cross(centroidVector[i])) > 0) // non-clockwise side
				{
					theta.push_back(6.2832-(acos(centroidVector[0].dot((centroidVector[i])/(centroidVector[0].norm()*centroidVector[i].norm())))));
					thetaSorted.push_back(6.2832-(acos(centroidVector[0].dot((centroidVector[i])/(centroidVector[0].norm()*centroidVector[i].norm())))));
				}
				else
				{
					std::stringstream errorMessage;
						errorMessage << "\nError, can't determine the sorting angle when sort_clockwise of points \n"
												 << "(bso/utilities/sort_clockwise)." << std::endl;
						throw std::runtime_error(errorMessage.str());
				}
			}
		}
		std::sort(thetaSorted.begin(), thetaSorted.end());
		
		// fale safe for dubplicated angles, therefore wrong point assignets to the ordered points
		// https://www.geeksforgeeks.org/stdunique-in-cpp/
		int size1 = thetaSorted.size();
		std::vector<double>::iterator ip; 
		ip = std::unique(thetaSorted.begin(), thetaSorted.end()); 
		thetaSorted.resize(std::distance(thetaSorted.begin(), ip)); 
		int size2 = thetaSorted.size();
		int sizediference = size1-size2;
		if (size1 != size2)
		{
			std::stringstream errorMessage;
						errorMessage << "\nError, There are at least 2 points with same angle to first vector when sort_clockwise of points \n"
												 << "after removing duplicates is the difference: " << sizediference << "\n" 
												 << "(bso/utilities/sort_clockwise)." << std::endl;
						throw std::runtime_error(errorMessage.str());
		}
		
		
		
		// generate the new sorted vector
		std::vector<bso::spatial_design::conformal::cf_vertex> sortedpPtr;
		for(int i=0; i<thetaSorted.size(); i++)
		{
			std::vector<double>::iterator itr = std::find(theta.begin(), theta.end(), thetaSorted[i]);
			if (itr != theta.cend()) 
			{
				sortedpPtr.push_back(pPtr[std::distance(theta.begin(), itr)]);
			}
			else 
			{
				std::stringstream errorMessage;
						errorMessage << "\nError, can't find the item in theta from the thetaSorted vector when sort_clockwise of points \n"
												 << "(bso/utilities/sort_clockwise)." << std::endl;
						throw std::runtime_error(errorMessage.str());
			}
		}

		return sortedpPtr;
	}
	
	
	std::vector<bso::utilities::geometry::vertex> sortClockwisePlanar(std::vector<bso::utilities::geometry::vertex>& pPtr)
	{
		// background information: std::vector<cf_vertex>& intsecV
		//https://stackoverflow.com/questions/47949485/sorting-a-list-of-3d-points-in-clockwise-order
		//https://stackoverflow.com/questions/14370636/sorting-a-list-of-3d-coplanar-points-to-be-clockwise-or-counterclockwise
				
		
		// find centroid of points
		bso::utilities::geometry::vertex centroid;
		bso::utilities::geometry::vertex temp;
				
		for (int i = 0; i<(pPtr.size()+1); i++)
		{
			temp = temp+pPtr[i];
		}
		std::cout << " " << std::endl;
	
		temp = temp/pPtr.size();
		centroid = temp; //  calculation errors should be removed (zerro is no zerro)
		
		
		// Determine the normal of plane
		const double& tol = 1e-3; //mGeometryModel->tolerance(); //
		bso::utilities::geometry::vector normal;
		bool allVerticesOnOneLine = true;
		for (unsigned int i = 1; i < pPtr.size()-1; i++)
		{
			bso::utilities::geometry::vector v1 = {pPtr[i]-pPtr[0]};
			bso::utilities::geometry::vector v2 = {pPtr[i+1]-pPtr[0]};
			if (v1.isZero(tol) || v2.isZero(tol)) continue;
			if (!v1.isParallel(v2,tol))
			{
				allVerticesOnOneLine = false;
				normal = v1.cross(v2).normalized();
				break;
			}
		}
		
		if(allVerticesOnOneLine == true)
		{
			std::stringstream errorMessage;
						errorMessage << "\nError, can't find the normal when sort_clockwise of points \n"
												 << "(bso/utilities/sort_clockwise)." << std::endl;
						throw std::runtime_error(errorMessage.str());
		}		
		
		// angle calculation, defining sorting mechanics
		std::vector<double> theta;
		std::vector<double> thetaSorted;
		std::vector<double> clockorder;
		std::vector <bso::utilities::geometry::vector> centroidVector;
		
		for (int i=0; i< pPtr.size(); i++)
		{
			centroidVector.push_back(pPtr[i]-centroid);
		}
		
		for (int i=0; i< centroidVector.size(); i++)
		{			
			clockorder.push_back(normal.dot(centroidVector[0].cross(centroidVector[i])));
			
			if(centroidVector[0].isParallel(centroidVector[i]))
			{
				if((centroidVector[0]).isCodirectional(centroidVector[i]))
				{
					theta.push_back(0);
					thetaSorted.push_back(0);
				}
				else if (!((centroidVector[0]).isCodirectional(centroidVector[i])))
				{
					theta.push_back(3.1416); //180 graden
					thetaSorted.push_back(3.1416);
				}
			}
			else
			{
				if(normal.dot(centroidVector[0].cross(centroidVector[i])) < 0) // clockwise side
				{
					theta.push_back(acos(centroidVector[0].dot((centroidVector[i])/(centroidVector[0].norm()*centroidVector[i].norm()))));
					thetaSorted.push_back(acos(centroidVector[0].dot((centroidVector[i])/(centroidVector[0].norm()*centroidVector[i].norm()))));
				}
				else if (normal.dot(centroidVector[0].cross(centroidVector[i])) > 0) // non-clockwise side
				{
					theta.push_back(6.2832-(acos(centroidVector[0].dot((centroidVector[i])/(centroidVector[0].norm()*centroidVector[i].norm())))));
					thetaSorted.push_back(6.2832-(acos(centroidVector[0].dot((centroidVector[i])/(centroidVector[0].norm()*centroidVector[i].norm())))));
				}
				else
				{
					std::stringstream errorMessage;
						errorMessage << "\nError, can't determine the sorting angle when sort_clockwise of points \n"
												 << "(bso/utilities/sort_clockwise)." << std::endl;
						throw std::runtime_error(errorMessage.str());
				}
			}
		}
		std::sort(thetaSorted.begin(), thetaSorted.end());
		
		// fale safe for dubplicated angles, therefore wrong point assignets to the ordered points
		// https://www.geeksforgeeks.org/stdunique-in-cpp/
		int size1 = thetaSorted.size();
		std::vector<double>::iterator ip; 
		ip = std::unique(thetaSorted.begin(), thetaSorted.end()); 
		thetaSorted.resize(std::distance(thetaSorted.begin(), ip)); 
		int size2 = thetaSorted.size();
		int sizediference = size1-size2;
		if (size1 != size2)
		{
			std::stringstream errorMessage;
						errorMessage << "\nError, There are at least 2 points with same angle to first vector when sort_clockwise of points \n"
												 << "after removing duplicates is the difference: " << sizediference << "\n" 
												 << "(bso/utilities/sort_clockwise)." << std::endl;
						throw std::runtime_error(errorMessage.str());
		}
		
		
		
		// generate the new sorted vector
		std::vector<bso::utilities::geometry::vertex> sortedpPtr;
		for(int i=0; i<thetaSorted.size(); i++)
		{
			std::vector<double>::iterator itr = std::find(theta.begin(), theta.end(), thetaSorted[i]);
			if (itr != theta.cend()) 
			{
				sortedpPtr.push_back(pPtr[std::distance(theta.begin(), itr)]);
			}
			else 
			{
				std::stringstream errorMessage;
						errorMessage << "\nError, can't find the item in theta from the thetaSorted vector when sort_clockwise of points \n"
												 << "(bso/utilities/sort_clockwise)." << std::endl;
						throw std::runtime_error(errorMessage.str());
			}
		}
		return sortedpPtr;
	}
	
	
} // namespace utilities
} // namespace bso


#endif // SORT_CLOCKWISE_CPP

