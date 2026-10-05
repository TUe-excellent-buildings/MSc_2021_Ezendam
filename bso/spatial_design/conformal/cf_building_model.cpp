#ifndef CF_BUILDING_MODEL_CPP
#define CF_BUILDING_MODEL_CPP

#include <bitset>
#include <Fade_2D.h>
#include <FadeExport.h>
#include <fstream>
#include <iomanip> // std::setprecision()

namespace bso { namespace spatial_design { namespace conformal {
	
	void cf_building_model::addSpace(const ms_space& msSpace) // initialize initial quad-hexahedorn geometry conformal model and final building conformal model
	{ // 
		mCFSpaces.push_back(new cf_space(msSpace.getGeometry(), this));
		mCFSpaces.back()->setSpaceID(msSpace.getID());
		std::string possibleSpaceType;
		if (msSpace.getSpaceType(possibleSpaceType))
		{
			mCFSpaces.back()->setSpaceType(possibleSpaceType);
		}

		auto spPtr = mCFSpaces.back();
		for (const auto& i : *spPtr)
		{
			mCFPoints.push_back(new cf_point(i, this));
			mCFPoints.back()->addSpace(spPtr);
			spPtr->addPoint(mCFPoints.back());
		}
		
		for (const auto& i : spPtr->getLines())
		{
			mCFEdges.push_back(new cf_edge(i, this));
			mCFEdges.back()->addSpace(spPtr);
			spPtr->addEdge(mCFEdges.back());
		}
		
		std::vector<std::string> possibleSurfaceTypes;
		bool surfaceTypesAvailable = msSpace.getSurfaceTypes(possibleSurfaceTypes);
		if (surfaceTypesAvailable)
		{
			std::swap(possibleSurfaceTypes[0],possibleSurfaceTypes[2]);
			std::swap(possibleSurfaceTypes[4],possibleSurfaceTypes[5]);
		}
		auto typeIte = possibleSurfaceTypes.begin();
		
		for (const auto& i : spPtr->getPolygons())
		{
			mCFSurfaces.push_back(new cf_surface(*i, this));
			mCFSurfaces.back()->addSpace(spPtr);
			spPtr->addSurface(mCFSurfaces.back());
			if (surfaceTypesAvailable)
			{
				mCFSurfaces.back()->setSurfaceType(*typeIte);
				++typeIte;
			}
		}	
	} // addSpace
	
	void cf_building_model::addSpaceT(const ms_space& msSpace) // initialize tetrahedron geometry conformal model
	{
		bso::utilities::geometry::quad_hexahedron temp(msSpace.getGeometry());
		mCFSpaces.push_back(new cf_space(msSpace.getGeometry(), this , (temp.getTetrahedrons())));
		
		
		// Add space types
		mCFSpaces.back()->setSpaceID(msSpace.getID());
		std::string possibleSpaceType;
		if (msSpace.getSpaceType(possibleSpaceType))
		{
			mCFSpaces.back()->setSpaceType(possibleSpaceType);
		}


		// generate the triangle entities of the tetrahedron, since otherwise the polygon does not convert
		std::vector<utilities::geometry::triangle*> tri;
		for(const auto i: ((mCFSpaces.back() -> getTetrahedrons())))
		{
			for(const auto j: i.getPolygons())
			{
				std::vector <utilities::geometry::vertex> triVertex;
				for(const auto k: j->getVertices())
				{
					triVertex.push_back(k);
				}
				tri.push_back(new utilities::geometry::triangle(triVertex));
			}
		}
		
		
		// Add the geometry entities to cf_geometry_model and cf_building_model
		auto spPtr = mCFSpaces.back();
		for (const auto& i : *spPtr)
		{
			mCFPoints.push_back(new cf_point(i, this)); // add points of space spPtr to mCFSpaces and initate constoctor, which adds the points to cf_geometry_model mCFVertices
			mCFPoints.back()->addSpace(spPtr); // store the space to which the points belongs in the mCFSpaces of cf_point
			spPtr->addPoint(mCFPoints.back()); // add the cf_point to the mCFSpaces of the cf_space to which it belongs point belongs
		}
		
		for (const auto& i : spPtr->getLines())
		{
			mCFEdges.push_back(new cf_edge(i, this));
			mCFEdges.back()->addSpace(spPtr);
			spPtr->addEdge(mCFEdges.back());
		}
		
		std::vector<std::string> possibleSurfaceTypes;
		bool surfaceTypesAvailable = msSpace.getSurfaceTypes(possibleSurfaceTypes);
		if (surfaceTypesAvailable)
		{
			std::swap(possibleSurfaceTypes[0],possibleSurfaceTypes[2]);
			std::swap(possibleSurfaceTypes[4],possibleSurfaceTypes[5]);
		}
		auto typeIte = possibleSurfaceTypes.begin();
		
		for (const auto& i : spPtr->getPolygons())
		{
			std::vector<utilities::geometry::triangle*> triOnPoly;
			
			//find triangles from tetrahedron which are on the space polygon to link to this surface
			for(const auto l: tri)
			{
				bool onFace = true;
				for(const auto k: l->getVertices()) //Note: tolerance is not acounted for, seems not to be needed. Therefore is this kept the origonal way
				{
					if ((std::find((i->getVertices()).begin(),(i->getVertices()).end(), k) == (i->getVertices()).end()) == false)
					{
						onFace = false;
					}
				}
				
				if(onFace == true)
				{
					triOnPoly.push_back(l);
				}
			}
			
			// add surface and triangles
			
			mCFSurfaces.push_back(new cf_surface(*i, this, triOnPoly));
			mCFSurfaces.back()->addSpace(spPtr);
			spPtr->addSurface(mCFSurfaces.back());
			if (surfaceTypesAvailable)
			{
				mCFSurfaces.back()->setSurfaceType(*typeIte);
				++typeIte;
			}
		}		
	} //addSpaceT
	
	void cf_building_model::addSpaceD(const ms_space& msSpace, std::vector<utilities::geometry::triangular_prism>& triPrismPtr)
	{
		mCFSpaces.push_back(new cf_space(msSpace.getGeometry(), this, triPrismPtr));
		
		mCFSpaces.back()->setSpaceID(msSpace.getID());
		std::string possibleSpaceType;
		if (msSpace.getSpaceType(possibleSpaceType))
		{
			mCFSpaces.back()->setSpaceType(possibleSpaceType);
		}
		
		// Store all unique polygon and line entities from the to insert triPrism for consideration of insertion into the geometry conformal model	
		std::vector<utilities::geometry::line_segment> prismLine;
		for(const auto i: triPrismPtr)
		{			
			for(auto j: i.getLines())
			{								
				bool isSameAs = false;
				for(const auto k: prismLine)
				{
					if(k.isSameAs(j, mTol))
					{
						isSameAs = true;
					}
				}
				if(isSameAs == false)
				{
					prismLine.push_back(j);
				}
			}
		}
			
		// Add the geometry entities to cf_geometry_model and the corosponding cf_building_model geometry, i.e. a line from a space may exist fom multiple lines in the cf_geometry_model
		auto spPtr = mCFSpaces.back();
		for (const auto& i : *spPtr)
		{
			mCFPoints.push_back(new cf_point(i, this)); // add points of space spPtr to mCFSpaces and initate constoctor, which adds the points to cf_geometry_model mCFVertices
			mCFPoints.back()->addSpace(spPtr); // store the space to which the points belongs in the mCFSpaces of cf_point
			spPtr->addPoint(mCFPoints.back()); // add the cf_point to the mCFSpaces of the cf_space to which it belongs point belongs
		}
		
					
		for (const auto& i : spPtr->getLines())
		{ 
			//Determine which lines are coresponding with which edges in the building conformal model
			std::vector<utilities::geometry::line_segment*> linesOnEdge;
			
			for(const auto j: prismLine)
			{
				
				if(i.isOnLine(j[0], mTol) && i.isOnLine(j[1], mTol))
				{
					linesOnEdge.push_back(new utilities::geometry::line_segment(j));
				}
				else if(i.isOnLine(j[0], mTol))
				{
					if((i[0]).isSameAs(j[1], mTol) || (i[1]).isSameAs(j[1], mTol))
					{
						linesOnEdge.push_back(new utilities::geometry::line_segment(j));
					}
				}
				else if(i.isOnLine(j[1], mTol))
				{
					if((i[0]).isSameAs(j[0], mTol) || (i[1]).isSameAs(j[0], mTol))
					{
						linesOnEdge.push_back(new utilities::geometry::line_segment(j));
					}
				}
				else if((i[1]).isSameAs(j[0], mTol) && (i[0]).isSameAs(j[1], mTol))
				{
					linesOnEdge.push_back(new utilities::geometry::line_segment(j));
				}
				else if((i[0]).isSameAs(j[0], mTol) && (i[1]).isSameAs(j[1], mTol))
				{
					linesOnEdge.push_back(new utilities::geometry::line_segment(j));
				}				
			}

			// Add edges acordingly
			mCFEdges.push_back(new cf_edge(i, this, linesOnEdge));
			mCFEdges.back()->addSpace(spPtr);
			spPtr->addEdge(mCFEdges.back());
		}
		
		std::vector<std::string> possibleSurfaceTypes;
		bool surfaceTypesAvailable = msSpace.getSurfaceTypes(possibleSurfaceTypes);
		if (surfaceTypesAvailable)
		{
			std::swap(possibleSurfaceTypes[0],possibleSurfaceTypes[2]);
			std::swap(possibleSurfaceTypes[4],possibleSurfaceTypes[5]);
		}
		auto typeIte = possibleSurfaceTypes.begin();
		
		
		for (const auto& i : spPtr->getPolygons())
		{
			//Determine which lines are coresponding with with edges
			std::vector<utilities::geometry::polygon*> PolyOnSurface;
			std::vector<utilities::geometry::triangle*> triOnSurface;
			std::vector<utilities::geometry::quadrilateral*> quadOnSurface;
			
			for(const auto j: triPrismPtr)
			{
				for(const auto& k: j.getTriangles())
				{
					bool allVOnSurface = true;
				
					for(const auto l: k)
					{
						if(i->isInsideOrOn(l, mTol) == false){allVOnSurface = false;}
					}
					
					if(allVOnSurface == true)
					{
						triOnSurface.push_back(new utilities::geometry::triangle(k)); // should the new key word be used here, do need a new memory adress since a const storing isn't posible
					}
				}
				for(const auto& k: j.getQuadrilateral())
				{
					bool allVOnSurface = true;
				
					for(const auto l: k)
					{
						if(i->isInsideOrOn(l, mTol) == false){allVOnSurface = false;}
					}
					
					if(allVOnSurface == true)
					{
						quadOnSurface.push_back(new utilities::geometry::quadrilateral(k)); // should the new key word be used here, do need a new memory adress since a const storing isn't posible
					}
				}
			}

			// Add surface and triangles
			mCFSurfaces.push_back(new cf_surface(*i, this, triOnSurface, quadOnSurface));
			mCFSurfaces.back()->addSpace(spPtr);
			spPtr->addSurface(mCFSurfaces.back());
			if (surfaceTypesAvailable)
			{
				mCFSurfaces.back()->setSurfaceType(*typeIte);
				++typeIte;
			}
		}

		//Check if the space volume is considered fully filled with the inserted triangular prism cells
		double spaceVolume = (mCFSpaces.back())->getVolume();
		double cellVolume = 0;
		for(const auto i: triPrismPtr)
		{
			cellVolume = cellVolume + i.getVolume();
		}
		
		if((spaceVolume-cellVolume) < (0-10) || (spaceVolume-cellVolume) > (0+10)) // The standart tolerance is too small and trikkers the error unrightfully fast
		{
			std::stringstream errorMessage;
			errorMessage << "When initializing multiple triangular prism cells in a space in the addSpaceD() function.\n"
									 << "The space was not fully filled with the inserted cells. \n"
									 << "Meaning that there are cells missing. \n"
									 << "The diference between the space volume and the sum of all the cells is: " << (spaceVolume-cellVolume) << "\n"
									 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
			throw std::invalid_argument(errorMessage.str());
		}
	}// addSpaceD
	
	void cf_building_model::makeConformalN() // Step one iterative partition method
	{ //
		// main variable declaration
		std::vector <cf_vertex> pIntersectionV;
		std::vector <utilities::geometry::line_segment> pIntersectionL;
		utilities::geometry::vertex pIntersection;
		bool usecheckVertexN = true; // or usecheckVertexNO() for the duration of the method (see decribtion of the diference in cf_space.hpp file)		
		// the recomended and standard method is checkVertexN() function.
	
		// Check variable decleration
		std::vector <cf_vertex> pIntersectionVCheck;
		
		
		interspaceCount = 1;
		int t=1;
		for (int i=0; i<t; i++)
		{		
			bool isSplitDone = false;
			//std::cout << "start interspace" << std::endl;
			for (const auto& i: mCFSpaces)
			{			
				for (const auto& j: mCFSpaces)
				{
					if(i == j) 
					{
						continue;
					}

					for(const auto& o: (i->cfCuboids()))
					{						
						for(const auto& p: (j->cfCuboids()))
						{
							if(o == p)
							{
								continue;
							}
							// check for points inside space
							
						
							// check for line - rectangle intersections, and add the found vertex to the geometry model and pIntersection
							for (const auto k: o->cfRectangles())
							{
								for (const auto l: p->getLines())
								{
									if (k->intersectsWith(l, pIntersection, mTol))
									{
										this->addVertex(pIntersection);
										//std::cout << "Add polyintersection: " << pIntersection << std::endl;
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
										}
									}
									else if(k->isInside(l[0], mTol) && k->isInside(l[1], mTol))
									{
										pIntersectionL.push_back(l);
										
									}
								}
							}
							
							// check for line line intersections, and add the found vertex to the geometry model and pIntersection
							for (const auto k: o->getLines())
							{
								for (const auto l: p->getLines())
								{
									if(k == l)
									{
										continue;
									}
									else if (k.intersectsWith(l, pIntersection, mTol))
									{
										this->addVertex(pIntersection);
										pIntersectionL.push_back(l);
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
											//std::cout << "Add Line intersection: " << pIntersection << "the intersecting lines are: " << "safed " << l << " unsafed " << k << std::endl;
										}
									}
								}
							}
						}
					}
				}
				
				if(pIntersectionV.size() >= 1)
				{
					// remove ducplicates from pIntersectionL
					auto pIntersectionL_end = pIntersectionL.end();
					for(auto it = pIntersectionL.begin(); it != pIntersectionL_end; ++it)
					{
						pIntersectionL_end = std::remove(it + 1, pIntersectionL_end, *it);
					}
					pIntersectionL.erase(pIntersectionL_end, pIntersectionL.end());
					
					//send to step two
					if(usecheckVertexN)
					{
						i->checkVertexN(pIntersectionV, pIntersectionL, methodUsed, this);
						//storeCfStages(this);
					}
					else
					{
						i->checkVertexNO(pIntersectionV, pIntersectionL, methodUsed);
					}
					
					if(isSplitDone == false)
					{
						t++;
						interspaceCount++;
						isSplitDone = true;
					}
				}
				pIntersectionV.clear();
				pIntersectionL.clear();
			}
			
			//std::cout << "start intercell" << std::endl;
			for (const auto& h: mCFSpaces) // intercell
			{		
				std::vector <cf_vertex> pIntersectionV;
				std::vector <utilities::geometry::line_segment> pIntersectionL;
				utilities::geometry::vertex pIntersection;	
				
				
				int s = 1;
				intercellCount = 1;
				for (int i=0; i<s; i++) //Repeat two
				{
					bool isSplitDone = false;
					
					for(int o=0; o < (h->cfCuboids()).size(); o++)
					{						
						for(int p=0; p <(h->cfCuboids()).size(); p++)
						{
							// check for line - rectangle intersections, and add the found vertex to the geometry model and pIntersection
							for (const auto k: (h->cfCuboids())[o]->getPolygons())
							{				
								for (const auto l: (h->cfCuboids())[p]->getLines())
								{									
									if (k->intersectsWith(l, pIntersection, mTol))
									{					
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
										}
									}
									
									else if(k->isInside(l[0], mTol) && k->isInside(l[1], mTol))
									{
										pIntersectionL.push_back(l);
									}
								}
							}
							
							// check for line line intersections, and add the found vertex to the geometry model and pIntersection
							for (const auto k: (h->cfCuboids())[o]->getLines())
							{
								for (const auto l: (h->cfCuboids())[p]->getLines())
								{
									if (k.intersectsWith(l, pIntersection, mTol))
									{
										pIntersectionL.push_back(l);
																				
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);								
										}
									}
								}
							}
						}

						if(pIntersectionV.size() >= 1)
						{
							// remove ducplicates from pIntersectionL
							auto pIntersectionL_end = pIntersectionL.end();
							for(auto it = pIntersectionL.begin(); it != pIntersectionL_end; ++it)
							{
								pIntersectionL_end = std::remove(it + 1, pIntersectionL_end, *it);
							}
							pIntersectionL.erase(pIntersectionL_end, pIntersectionL.end());
							
							int cuboidSize = (h->cfCuboids()).size();
							std::vector<cf_cuboid*> intersectedCell;
							intersectedCell.push_back((h->cfCuboids())[o]);
							
							//send to step two
							if(usecheckVertexN)
							{
								h->checkVertexN(pIntersectionV, pIntersectionL, methodUsed, intersectedCell, this);
								//storeCfStages(this);
							}
							else
							{
								h->checkVertexNO(pIntersectionV, pIntersectionL, methodUsed, intersectedCell);
							}
							
							if(isSplitDone == false)
							{
								s++;
								intercellCount++;
								isSplitDone = true;
							}
						}
						pIntersectionV.clear();
						pIntersectionL.clear();
					}
					
					// delete the lines, rectangles, and cuboids that were tagged for deletion
					mCFLines.erase(std::remove_if(mCFLines.begin(), mCFLines.end(), [](const auto& i){ return i->deletion(); }), mCFLines.end());
					mCFRectangles.erase(std::remove_if(mCFRectangles.begin(), mCFRectangles.end(), [](const auto& i){ return i->deletion(); }), mCFRectangles.end());
					mCFCuboids.erase(std::remove_if(mCFCuboids.begin(), mCFCuboids.end(), [](const auto& i){ return i->deletion(); }), mCFCuboids.end());
				}
			}
			
			// delete the lines, rectangles, and cuboids that were tagged for deletion
			mCFLines.erase(std::remove_if(mCFLines.begin(), mCFLines.end(), [](const auto& i){ return i->deletion(); }), mCFLines.end());
			mCFRectangles.erase(std::remove_if(mCFRectangles.begin(), mCFRectangles.end(), [](const auto& i){ return i->deletion(); }), mCFRectangles.end());
			mCFCuboids.erase(std::remove_if(mCFCuboids.begin(), mCFCuboids.end(), [](const auto& i){ return i->deletion(); }), mCFCuboids.end());
		}
	} //makeConformalN()
	
	
	void cf_building_model::makeConformalN2D() // Step one quad-hexahedron method
	{ //
		std::vector <cf_vertex> pIntersectionV;
		std::vector <utilities::geometry::line_segment> pIntersectionL;
		utilities::geometry::vertex pIntersection;	
		
		//Check all groundfloors of the cells
		int t = 1;
		for (int i=0; i<t; i++)
		{		
			bool isSplitDone = false;
			//std::cout << "start interspace" << std::endl;
			for (const auto& i: mCFSpaces)
			{			
				//std::cout << "  " << std::endl;
				//std::cout << "step one space: " << i->getSpaceID() << " containing " << *i << std::endl;
				for (const auto& j: mCFSpaces)
				{
					if(i == j) 
					{
						continue;
					}

					for(const auto& o: (i->cfCuboids()))
					{						
						for(const auto& p: (j->cfCuboids()))
						{
							if(o == p)
							{
								continue;
							}					
							
							// check for points inside space

							
							// check for line - rectangle intersections, and add the found vertex to the geometry model and pIntersection							
							for (const auto l: (p->getCuboidAtGround()).getLines())
							{								
								if ((o->getRectangleAtGround()).intersectsWith(l, pIntersection, mTol))
								{
									if(p->isInsideOrOn(pIntersection, mTol))
									{	
										//utilities::geometry::vertex shift = {0,0,o->getGroundDiference()};
										//this->addVertex(pIntersection+shift);
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
											//std::cout << "Add polyintersection: " << pIntersection << std::endl;
										}
									}
								}
								else if((o->getRectangleAtGround()).isInsideOrOn(l[0], mTol) || (o->getRectangleAtGround()).isInsideOrOn(l[1], mTol))
								{
									pIntersectionL.push_back(l);
									//std::cout << "add poly guideLine: " << l << std::endl;
								}
							}
							
							
							// check for line line intersections, and add the found vertex to the geometry model and pIntersection
							for (const auto k: (o->getRectangleAtGround()).getLines())
							{
								
								for (const auto l: (p->getCuboidAtGround()).getLines())
								{
									if(k.isSameAs(l, mTol))
									{
										continue;
									}
									else if (k.intersectsWith(l, pIntersection, mTol))
									{
										//utilities::geometry::vertex shift = {0,0,o->getGroundDiference()};
										//this->addVertex(pIntersection+shift);

										pIntersectionL.push_back(l);
										//std::cout << "add line guideLine: " << l << std::endl;
										
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
											//std::cout << "Add lineintersection: " << pIntersection << std::endl;
										}
									}
								}
							}
						}
					}
				}
				
				if(pIntersectionV.size() >= 1)
				{
					// remove ducplicates from pIntersectionL
					auto pIntersectionL_end = pIntersectionL.end();
					for(auto it = pIntersectionL.begin(); it != pIntersectionL_end; ++it)
					{
						pIntersectionL_end = std::remove(it + 1, pIntersectionL_end, *it);
					}
					pIntersectionL.erase(pIntersectionL_end, pIntersectionL.end());			
					
					if(i->checkVertexN2D(pIntersectionV, pIntersectionL, methodUsed))
					{
						if(isSplitDone == false)
						{
							t++;
							isSplitDone = true;
						}
					}
				}
				pIntersectionV.clear();
				pIntersectionL.clear();
			}
			
			
			//std::cout << "start intercell" << std::endl;
			for (const auto& h: mCFSpaces) // intercell
			{		
				std::vector <cf_vertex> pIntersectionV;
				std::vector <utilities::geometry::line_segment> pIntersectionL;
				utilities::geometry::vertex pIntersection;	
				
				
				int s = 1;
				for (int i=0; i<s; i++) //Repeat two
				{
					bool isSplitDone = false;
					
					for(int o=0; o < (h->cfCuboids()).size(); o++)
					{						
						for(int p=0; p <(h->cfCuboids()).size(); p++)
						{
						
							
							// check for points inside space

							
							// check for line - rectangle intersections, and add the found vertex to the geometry model and pIntersection							
							for (const auto l: ((h->cfCuboids())[p]->getCuboidAtGround()).getLines())
							{								
								if (((h->cfCuboids())[o]->getRectangleAtGround()).intersectsWith(l, pIntersection, mTol))
								{
									//utilities::geometry::vertex shift = {0,0,o->getGroundDiference()};
									//this->addVertex(pIntersection+shift);
									
									bool doubleInsert = false;
									for (const auto& m : pIntersectionV)
									{// check if pIntersection already exist in pIntersectionV
										if (m.isSameAs(pIntersection, mTol))
										{
											doubleInsert = true;
										}
									}
									
									if (doubleInsert == false)
									{
										pIntersectionV.push_back(pIntersection);
										//std::cout << "Add polyintersection: " << pIntersection << std::endl;
									}
								}
								else if(((h->cfCuboids())[o]->getRectangleAtGround()).isInsideOrOn(l[0], mTol) && ((h->cfCuboids())[o]->getRectangleAtGround()).isInsideOrOn(l[1], mTol))
								{
									pIntersectionL.push_back(l);
									//std::cout << "add poly guideLine: " << l << std::endl;
								}
							}
							
							
							// check for line line intersections, and add the found vertex to the geometry model and pIntersection
							for (const auto k: ((h->cfCuboids())[o]->getRectangleAtGround()).getLines())
							{
								
								for (const auto l: ((h->cfCuboids())[p]->getCuboidAtGround()).getLines())
								{
									if(k.isSameAs(l, mTol))
									{
										continue;
									}
									else if (k.intersectsWith(l, pIntersection, mTol))
									{
										//utilities::geometry::vertex shift = {0,0,o->getGroundDiference()};
										//this->addVertex(pIntersection+shift);

										//pIntersectionL.push_back(l);
										//std::cout << "add line guideLine: " << l << std::endl;
										
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
										}
									}
								}
							}							
						}

						if(pIntersectionV.size() >= 1)
						{
							// remove ducplicates from pIntersectionL
							auto pIntersectionL_end = pIntersectionL.end();
							for(auto it = pIntersectionL.begin(); it != pIntersectionL_end; ++it)
							{
								pIntersectionL_end = std::remove(it + 1, pIntersectionL_end, *it);
							}
							pIntersectionL.erase(pIntersectionL_end, pIntersectionL.end());
							
							int cuboidSize = (h->cfCuboids()).size();
							std::vector<cf_cuboid*> intersectedCell;
							intersectedCell.push_back((h->cfCuboids())[o]);
							if(h->checkVertexN2D(pIntersectionV, pIntersectionL, methodUsed, intersectedCell))
							{
								if(isSplitDone == false)
								{
									s++;
									isSplitDone = true;
								}
							}
						}
						pIntersectionV.clear();
						pIntersectionL.clear();
					}
					
					// delete the lines, rectangles, and cuboids that were tagged for deletion
					mCFLines.erase(std::remove_if(mCFLines.begin(), mCFLines.end(), [](const auto& i){ return i->deletion(); }), mCFLines.end());
					mCFRectangles.erase(std::remove_if(mCFRectangles.begin(), mCFRectangles.end(), [](const auto& i){ return i->deletion(); }), mCFRectangles.end());
					mCFCuboids.erase(std::remove_if(mCFCuboids.begin(), mCFCuboids.end(), [](const auto& i){ return i->deletion(); }), mCFCuboids.end());
				}
			}
			
			
			// delete the lines, rectangles, and cuboids that were tagged for deletion
			mCFLines.erase(std::remove_if(mCFLines.begin(), mCFLines.end(), [](const auto& i){ return i->deletion(); }), mCFLines.end());
			mCFRectangles.erase(std::remove_if(mCFRectangles.begin(), mCFRectangles.end(), [](const auto& i){ return i->deletion(); }), mCFRectangles.end());
			mCFCuboids.erase(std::remove_if(mCFCuboids.begin(), mCFCuboids.end(), [](const auto& i){ return i->deletion(); }), mCFCuboids.end());
		}
		
		
		//Check all walls of the cells
		t = 1;
		for (int i=0; i<t; i++)
		{		
			bool isSplitDone = false;
			//std::cout << "start interspace" << std::endl;
			for (const auto& i: mCFSpaces)
			{			
				//std::cout << "  " << std::endl;
				//std::cout << "step one space: " << i->getSpaceID() << " containing " << *i << std::endl;
				for (const auto& j: mCFSpaces)
				{
					if(i == j) 
					{
						continue;
					}

					for(const auto& o: (i->cfCuboids()))
					{						
						for(const auto& p: (j->cfCuboids()))
						{
							if(o == p)
							{
								continue;
							}					
							
							// check for points inside space

							
							// check for line - rectangle intersections, and add the found vertex to the geometry model and pIntersection							
							for (const auto k: o->getCellWalls())
							{
								for (const auto l: (p->getCuboidAtGround()).getLines())
								{								
									if (k->intersectsWith(l, pIntersection, mTol))
									{
										//utilities::geometry::vertex shift = {0,0,o->getGroundDiference()};
										//this->addVertex(pIntersection+shift);
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
											//std::cout << "Add polyintersection: " << pIntersection << std::endl;
										}
									}
									else if(k->isInsideOrOn(l[0], mTol) && k->isInsideOrOn(l[1], mTol))
									{
										pIntersectionL.push_back(l);
										//std::cout << "add poly guideLine: " << l << std::endl;
									}
								}
							}							
							
							// check for line line intersections, and add the found vertex to the geometry model and pIntersection
							for (const auto k: (o->getLines()))
							{
								for (const auto l: (p->getLines()))
								{
									if(k.isSameAs(l, mTol))
									{
										continue;
									}
									else if (k.intersectsWith(l, pIntersection, mTol))
									{
										//utilities::geometry::vertex shift = {0,0,o->getGroundDiference()};
										//this->addVertex(pIntersection+shift);

										//pIntersectionL.push_back(l);
										//std::cout << "add line guideLine: " << l << std::endl;
										
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
										}
									}
								}
							}
						}
					}
				}
				
				if(pIntersectionV.size() >= 1)
				{
					// remove ducplicates from pIntersectionL
					auto pIntersectionL_end = pIntersectionL.end();
					for(auto it = pIntersectionL.begin(); it != pIntersectionL_end; ++it)
					{
						pIntersectionL_end = std::remove(it + 1, pIntersectionL_end, *it);
					}
					pIntersectionL.erase(pIntersectionL_end, pIntersectionL.end());			
					
					if(i->checkVertexN2D(pIntersectionV, pIntersectionL, methodUsed))
					{
						if(isSplitDone == false)
						{
							t++;
							isSplitDone = true;
						}
					}
				}
				pIntersectionV.clear();
				pIntersectionL.clear();
			}
			
			//std::cout << "start intercell" << std::endl;
			for (const auto& h: mCFSpaces) // intercell
			{		
				std::vector <cf_vertex> pIntersectionV;
				std::vector <utilities::geometry::line_segment> pIntersectionL;
				utilities::geometry::vertex pIntersection;	
				
				
				int s = 1;
				for (int i=0; i<s; i++) //Repeat two
				{
					bool isSplitDone = false;
					
					for(int o=0; o < (h->cfCuboids()).size(); o++)
					{						
						for(int p=0; p <(h->cfCuboids()).size(); p++)
						{
						
							
							// check for points inside space

							
							// check for line - rectangle intersections, and add the found vertex to the geometry model and pIntersection							
							for (const auto k: (h->cfCuboids())[o]->getCellWalls())
							{
								for (const auto l: ((h->cfCuboids())[p])->getLines())
								{								
									if (k->intersectsWith(l, pIntersection, mTol))
									{
										//utilities::geometry::vertex shift = {0,0,o->getGroundDiference()};
										//this->addVertex(pIntersection+shift);
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
											//std::cout << "Add polyintersection: " << pIntersection << std::endl;
										}
									}
									else if(k->isInsideOrOn(l[0], mTol) && k->isInsideOrOn(l[1], mTol))
									{
										pIntersectionL.push_back(l);
										//std::cout << "add poly guideLine: " << l << std::endl;
									}
								}
							}
							
							// check for line line intersections, and add the found vertex to the geometry model and pIntersection
							for (const auto k: ((h->cfCuboids())[o])->getLines())
							{
								for (const auto l: ((h->cfCuboids())[p])->getLines())
								{
									if(k.isSameAs(l, mTol))
									{
										continue;
									}
									else if (k.intersectsWith(l, pIntersection, mTol))
									{
										//utilities::geometry::vertex shift = {0,0,o->getGroundDiference()};
										//this->addVertex(pIntersection+shift);

										//pIntersectionL.push_back(l);
										//std::cout << "add line guideLine: " << l << std::endl;
										
										
										bool doubleInsert = false;
										for (const auto& m : pIntersectionV)
										{// check if pIntersection already exist in pIntersectionV
											if (m.isSameAs(pIntersection, mTol))
											{
												doubleInsert = true;
											}
										}
										
										if (doubleInsert == false)
										{
											pIntersectionV.push_back(pIntersection);
										}
									}
								}
							}							
						}

						if(pIntersectionV.size() >= 1)
						{
							// remove ducplicates from pIntersectionL
							auto pIntersectionL_end = pIntersectionL.end();
							for(auto it = pIntersectionL.begin(); it != pIntersectionL_end; ++it)
							{
								pIntersectionL_end = std::remove(it + 1, pIntersectionL_end, *it);
							}
							pIntersectionL.erase(pIntersectionL_end, pIntersectionL.end());
							
							int cuboidSize = (h->cfCuboids()).size();
							std::vector<cf_cuboid*> intersectedCell;
							intersectedCell.push_back((h->cfCuboids())[o]);
							if(h->checkVertexN2D(pIntersectionV, pIntersectionL, methodUsed, intersectedCell))
							{
								if(isSplitDone == false)
								{
									s++;
									isSplitDone = true;
								}
							}
						}
						pIntersectionV.clear();
						pIntersectionL.clear();
					}
					
					// delete the lines, rectangles, and cuboids that were tagged for deletion
					mCFLines.erase(std::remove_if(mCFLines.begin(), mCFLines.end(), [](const auto& i){ return i->deletion(); }), mCFLines.end());
					mCFRectangles.erase(std::remove_if(mCFRectangles.begin(), mCFRectangles.end(), [](const auto& i){ return i->deletion(); }), mCFRectangles.end());
					mCFCuboids.erase(std::remove_if(mCFCuboids.begin(), mCFCuboids.end(), [](const auto& i){ return i->deletion(); }), mCFCuboids.end());
				}
			}
			
			
			// delete the lines, rectangles, and cuboids that were tagged for deletion
			mCFLines.erase(std::remove_if(mCFLines.begin(), mCFLines.end(), [](const auto& i){ return i->deletion(); }), mCFLines.end());
			mCFRectangles.erase(std::remove_if(mCFRectangles.begin(), mCFRectangles.end(), [](const auto& i){ return i->deletion(); }), mCFRectangles.end());
			mCFCuboids.erase(std::remove_if(mCFCuboids.begin(), mCFCuboids.end(), [](const auto& i){ return i->deletion(); }), mCFCuboids.end());
		}
	} //makeConformalN2D()
	
	
	void cf_building_model::makeConformalT()
	{
		// Check conditions for using this method
		if(methodUsed != 'T')
		{

		}
		
		unsigned int t = 0;
		while (t < mCFVertices.size()) //Repeat
		{
			t = mCFVertices.size();			
			
			// check for line line intersections, and add the found vertex to the geometry model
			utilities::geometry::vertex pIntersection;
			for (unsigned int i = 0; i < mCFLines.size(); ++i)
			{
				for (unsigned int j = i+1; j < mCFLines.size(); ++j)
				{
					if (mCFLines[i]->intersectsWith(*(mCFLines[j]), pIntersection, mTol))
					{
						this->addVertex(pIntersection);
					}
				}
			}
			
			// check for line - rectangle intersections, and add the found vertex to the geometry model
			for (unsigned int i = 0; i < mCFTriangles.size(); ++i)
			{
				for (unsigned int j = 0; j < mCFLines.size(); ++j)
				{
					if (mCFTriangles[i]->intersectsWith(*(mCFLines[j]), pIntersection, mTol))
					{
						this->addVertex(pIntersection);
					}
				}
			}

			// Send to step 2
			unsigned int i = 0;
			while (i < mCFVertices.size()) 
			{
				for (auto& j : mCFSpaces)
				{
					j->checkVertexT(mCFVertices[i]);
				}
				++i;
			}
			
			// delete the lines, rectangles, and cuboids that were tagged for deletion
			mCFLines.erase(std::remove_if(mCFLines.begin(), mCFLines.end(), [](const auto& i){ return i->deletion(); }), mCFLines.end());
			mCFTriangles.erase(std::remove_if(mCFTriangles.begin(), mCFTriangles.end(), [](const auto& i){ return i->deletion(); }), mCFTriangles.end());
			mCFTetrahedrons.erase(std::remove_if(mCFTetrahedrons.begin(), mCFTetrahedrons.end(), [](const auto& i){ return i->deletion(); }), mCFTetrahedrons.end());
		}
	} //makeConformalT()
	
	void cf_building_model::makeConformalND(const ms_building& msModel) // Delauney method
	{ //
		// Declerations for needed information per space
		std::vector<utilities::geometry::quad_hexahedron> spacesAtLevel;
		//std::vector<utilities::geometry::quad_hexahedron> spacesAtGround; // Declared in private section class
		std::vector<utilities::geometry::vertex> highV;
		std::vector<utilities::geometry::vertex> LowV;
		
		// Declerations for needed information per building spatial design
		std::vector<double> allLevels; // stores all the floor levels
		std::vector<std::vector<double>> levelPerSpace;
		int count =0; //counts the amound of spaces
		
		//Get the above information
		for (const auto& i : mMSModel)
		{
			//Add origonal space geometry
			spacesAtLevel.push_back(i->getGeometry());
			
			//Find the highest and lowest vertex
			utilities::geometry::vertex lowVertex = (spacesAtLevel.back())[0];
			utilities::geometry::vertex highVertex = (spacesAtLevel.back())[0];
			for (const auto j: spacesAtLevel.back())
			{
				if(lowVertex.z() > j.z())
				{//find the lowest vertex of SpaceGeomTemp_1 for the shifting of the space geometry
					lowVertex = j;
				}
				else if(highVertex.z() < j.z())
				{
					highVertex = j;
				}
			}
			highV.push_back(highVertex);
			LowV.push_back(lowVertex);
			
			//Generate the at ground level space geometry
			std::vector<utilities::geometry::vertex> vertices_1;
			utilities::geometry::vertex shift = {0,0,(LowV.back()).z()};
			for(auto k: (spacesAtLevel.back())){vertices_1.push_back(k-shift);}
			spacesAtGround.push_back(utilities::geometry::quad_hexahedron(vertices_1, mTol));
			
			// Determine all the floor levels in the building spatial designs
			bool highVUnique = true;
			bool lowVUnique = true;
			for(double j: allLevels)
			{
				if(highVertex.z()<=(j+mTol) && highVertex.z()>=(j-mTol))
				{
					highVUnique = false;
				}
				if(lowVertex.z()<=(j+mTol) && lowVertex.z()>=(j-mTol))
				{
					lowVUnique = false;
				}
			}
			if(highVUnique){allLevels.push_back(highVertex.z());}
			if(lowVUnique){allLevels.push_back(lowVertex.z());}
			
			count++;
		}
		
		//Sort allLevels for later use in triangular prism cell definition
		sort(allLevels.begin(), allLevels.end());
		
		
		// Declaration delauney variables
		// std::vector <utilities::geometry::vertex> delauneyPoints; // Points to be inserted into the delauney triangulation; Declared in class
		// std::vector <utilities::geometry::line_segment> delauneyLines; //Lines where the delauney triangulation should constrain to; Declared in class
		
		//Find the apropriate information for the delauney triangulation
		for(int i=0; i<count; i++)
		{
			//Add all unique cornerpoints at z=0 of spaceAtGround to the delauneyPoints
			for(const auto j: spacesAtGround[i])
			{
				utilities::geometry::vertex temp = {j.x(),j.y(),0};

				bool doubleInsert = false;
				for (const auto& m : delauneyPoints)
				{// check if temp already exist in delauneyPoints
					if (m.isSameAs(temp, mTol))
					{
						doubleInsert = true;
					}
				}
				if(doubleInsert == false)
				{//if not, then add to delaunayPoints
					delauneyPoints.push_back(temp);
				}
			}
		}
			
		for(int i=0; i<count; i++)
		{
			for (const auto& k : spacesAtGround[i].getLines())
			{
				
				//std::cout << "a vertex of ltemp to be a delaunayLine: " << k[0].x() << " ; " << k[0].y() << std::endl;
				// Add the unique lines of each space to delauneyLines
				utilities::geometry::vertex Ltemp1 = {k[0].x(),k[0].y(),0};
				utilities::geometry::vertex Ltemp2 = {k[1].x(),k[1].y(),0};
				utilities::geometry::line_segment Ltemp {{Ltemp1},{Ltemp2}}; //Note the line is newly defined for the case that some round of error occur during the shifting of the lines of the spacesAtGround[i]
				
				bool doubleInsert = false;
				for (const auto& m : delauneyLines)
				{// check if line k already exist in delauneyPoints
					if (m.isSameAs(Ltemp, mTol))
					{
						doubleInsert = true;
					}
				}
				if(Ltemp1.isSameAs(Ltemp2, mTol))
				{//Check if line has diferent end points
					doubleInsert = true;
				}
				if(doubleInsert == false)
				{//if both agree, then add to delaunayLines
					delauneyLines.push_back(Ltemp);
				}
				
				//Find line-line intersections, note: no line-rectangle intersection check needed since these are acounted for by the addition of the cornerpoints
				utilities::geometry::vertex pIntersection;
				for(int j=i+1; j<count; j++)
				{
					if(i == j){continue;}
					for (const auto& l : spacesAtGround[j].getLines())
					{
						if (k.intersectsWith(l, pIntersection, mTol))
						{
							//consider the translated to ground floor vertex of the space
							utilities::geometry::vertex temp = {pIntersection.x(),pIntersection.y(),0};
							
							bool doubleInsert = false;
							for (const auto& m : delauneyPoints)
							{// check if temp already exist in delauneyPoints
								if (m.isSameAs(temp, mTol))
								{
									doubleInsert = true;
								}
							}
							
							if(doubleInsert == false)
							{//if not, then add to delaunayPoints
								delauneyPoints.push_back(temp);
							}
						}
					}
				}
			}
		}
		
		// Split lines at intersection vertices and remove duplicated lines
		splitLines(delauneyPoints, delauneyLines);// this prevent the genration of traingles on one line or triangles containing duplicate vertices

		// Send all vertices and constrain lines to the delauney triangulation and receive all triangles
		std::vector<utilities::geometry::triangle> delaunayTri = delaunay(delauneyPoints, delauneyLines);
		
		
		// identify which triangles should be used to generate a triangular prism cell in the space and generate that space and acompaning geometric entities
		int i = 0;
		for(const auto& t : mMSModel)
		{
			// Find the triangles in the space
			std::vector<utilities::geometry::triangle> triCell;
			for(const auto j : delaunayTri)
			{
				bool allNodesInside = true;
				for(const auto k: j)
				{
					if(!spacesAtGround[i].isInsideOrOn(k, mTol))
					{
						//additional check for prevention malfuntioning, which has been previously observed
						bool annyTrue = false;
						for(const auto l: spacesAtGround[i].getLines())
						{
							if(l.isOnLine(k, mTol))
							{
								annyTrue = true;
							}
						}
						if(!annyTrue)
						{
							allNodesInside = false;
						}
					}
				}
				if(allNodesInside)
				{
					triCell.push_back(j);
				}
			}
			
			//Are there anny midLevels and if yes, which are they?
			std::vector<double> levelToConsider;			
			
			for(const double j: allLevels)
			{
				if(j>LowV[i].z() && j< highV[i].z())
				{
					levelToConsider.push_back(j);
				}
			}

			// Define all sixs vertices of the triangular prism and store in triPrism
			std::vector<utilities::geometry::triangular_prism> triPrism;
			for(const auto j: triCell)
			{
				std::vector<utilities::geometry::vertex> triPrismV;
				triPrismV.reserve(6);
				if(levelToConsider.size()>0)
				{
					//Generate triangular prism
					triPrismV.clear();
					for(const auto k: j)
					{// botum row prisms
						utilities::geometry::vertex low = {0,0, LowV[i].z()};
						utilities::geometry::vertex high = {0,0, levelToConsider[0]};
						triPrismV.push_back(k+low);
						triPrismV.push_back(k+high);
					}
					triPrism.push_back(utilities::geometry::triangular_prism(triPrismV, mTol));
					triPrismV.clear();
					for(const auto k: j)
					{// top row prisms
						utilities::geometry::vertex low = {0,0, levelToConsider[levelToConsider.size()-1]};
						utilities::geometry::vertex high = {0,0, highV[i].z()};
						triPrismV.push_back(k+low);
						triPrismV.push_back(k+high);
					}
					triPrism.push_back(utilities::geometry::triangular_prism(triPrismV, mTol));
					if(levelToConsider.size()>1)
					{
						//std::cout << "There are inbetween levels. " << std::endl;
						for(int l=0; l<(levelToConsider.size()-1); l++)
						{// inbetween row prisms
							triPrismV.clear();
							for(const auto k: j)
							{
								utilities::geometry::vertex low = {0,0, levelToConsider[l]};
								utilities::geometry::vertex high = {0,0, levelToConsider[l+1]};
								triPrismV.push_back(k+low);
								triPrismV.push_back(k+high);
							}
							triPrism.push_back(utilities::geometry::triangular_prism(triPrismV, mTol));
						}
					}
				}
				else
				{
					for(const auto k: j)
					{
						utilities::geometry::vertex low = {0,0, LowV[i].z()};
						utilities::geometry::vertex high = {0,0, highV[i].z()};
						triPrismV.push_back(k+low);
						triPrismV.push_back(k+high);
						
					}
					triPrism.push_back(utilities::geometry::triangular_prism(triPrismV, mTol));
				}
			}
			
			// Send triPrismPtr to the AddSpace function to add the cells and spaces to the conformal model
			this->addSpaceD(*t, triPrism);
			i++;
		}		
	} //makeConformalND()
	
	void cf_building_model::splitLines(const std::vector<utilities::geometry::vertex>& intsecV, std::vector<utilities::geometry::line_segment>& intsecL)
	{					
		// split lines in acordance to intersection points
		for(int k=0; k<intsecL.size(); k++)
		{
			for(const auto l :intsecV)
			{
				if(intsecL[k].isOnLine(l, mTol))
				{
					utilities::geometry::line_segment one = {{(intsecL[k])[0]},{l}};
					utilities::geometry::line_segment two = {{(intsecL[k])[1]},{l}};
					
					intsecL.push_back(one);
					intsecL.push_back(two);

					intsecL.erase(intsecL.begin() + k);

					k--; // Makes sure the for loop isn't snipping a constrain lines when one is ereased.
				}
			}
		}
		
		// remove ducplicates from intsecL
		auto intsecL_end = intsecL.end();
		for(auto it = intsecL.begin(); it != intsecL_end; ++it)
		{
			intsecL_end = std::remove(it + 1, intsecL_end, *it);
		}
		intsecL.erase(intsecL_end, intsecL.end());
		
		
		// remove ducplicates which has switched vertexes in intsecL		
		for(int k; k<intsecL.size(); k++)
		{
			for(auto m : intsecL)
			{
				if (m == intsecL[k]){continue;} //No tolerance needed. Just opperator to make sure the same entities are not checked with each other.
				if (m.isSameAs(intsecL[k], mTol)){intsecL.erase(intsecL.begin() + k);}
			}
		}		
	}//cf_space::splitLines
	
	std::vector<utilities::geometry::triangle> cf_building_model::delaunay(const std::vector <utilities::geometry::vertex> delauneyPoints, const std::vector <utilities::geometry::line_segment> delauneyLines)
	{
		// Input checks
		if(delauneyPoints.size() <= 2)
		{
			std::stringstream errorMessage;
				errorMessage << "\nError, the inserted points in the delaunay traingulation are less then three. \n"
					 << "Meaning that a delaunay triangulation can not be generated. \n"
					 << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
				throw std::runtime_error(errorMessage.str());
		}
		else if(delauneyLines.size() == 0)
		{
			std::stringstream errorMessage;
				errorMessage << "\nError, there are no constrain lines added to the delauney triangulation. \n"
					 << "Meaning that a delaunay triangulation can not be generated. \n"
					 << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
				throw std::runtime_error(errorMessage.str());
		}
		//Check for duplicate vertices in the delaunay point variable
		bool duplicates = false; 
		std::vector<utilities::geometry::vertex> duplicatePoints;
		for(int t =0; t< delauneyPoints.size(); t++)
		{ 
			for(int s=0; s<delauneyPoints.size(); s++)
			{
				if(s==t){continue;}
				else if(delauneyPoints[t].isSameAs(delauneyPoints[s], mTol))
				{
					duplicates = true;
					duplicatePoints.push_back(delauneyPoints[s]);
				}
			}
		}
		if(duplicates)
		{
			std::stringstream errorMessage;
			errorMessage << "There are duplicate vertices found in the DelauneyPoints inserted in the delaunay method. \n" 
						 << "The point(s) which has a duplicate are: " << std::endl;
			for(const auto t: duplicatePoints){errorMessage << t;}
			errorMessage << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		//Check the constrain lines
		std::vector<utilities::geometry::vertex> noDelauneyPoint;
		std::vector<utilities::geometry::vertex> pointOnConstraintLine;
		std::vector<utilities::geometry::line_segment> contrainLines;
		for(auto t: delauneyLines)
		{
			for(const auto k: t)
			{
				if (std::find(delauneyPoints.begin(),delauneyPoints.end(),k) == delauneyPoints.end())
				{
					noDelauneyPoint.push_back(k);
				}
				if(t.isOnLine(k))
				{
					pointOnConstraintLine.push_back(k);
				}
			}
			for(auto k: delauneyLines)
			{
				if(k.isSameAs(t, mTol)){continue;}
				for(const auto s: k)
				{
					if(t.isOnLine(s, mTol))
					{
						pointOnConstraintLine.push_back(s);
						contrainLines.push_back(t);
					}
				}
			}
		}
		if(noDelauneyPoint.size() > 0)
		{
			std::stringstream errorMessage;
			errorMessage << "The following vertices are end vertices of the constraint lines, but not a delauney points (Checked with tolerance): " << std::endl;
			for(const auto t: noDelauneyPoint){errorMessage << t;}
			errorMessage << std::endl;
			errorMessage << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		if(pointOnConstraintLine.size() > 0)
		{
			std::stringstream errorMessage;
			int counter=0;
			errorMessage << "The following vertices are vertices on the constrain line, this is not allowed and should have been solved in the splitLines function. See the following vertices: " << std::endl;
			for(const auto t: pointOnConstraintLine)
			{
				errorMessage << "vertex: " << t << " has been found on: " << contrainLines[counter] << std::endl;
				counter++;
			}
			errorMessage << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		

		// Start Delaunay triangulation process
		
		// std::vector<utilities::geometry::triangle> delaunayTri; Declared in class
		
		// start adding the points/constrainLines to delaunay triangulation algorithm Fade_2D
		GEOM_FADE2D::Fade_2D dt;
		
		for(const auto i: delauneyPoints)
		{
			dt.insert(GEOM_FADE2D::Point2(i.x(),i.y()));
		}
		std::vector<GEOM_FADE2D::Segment2> vSegments;
		for(const auto i: delauneyLines)
		{
			vSegments.push_back(GEOM_FADE2D::Segment2(GEOM_FADE2D::Point2(i[0].x(),i[0].y()),GEOM_FADE2D::Point2(i[1].x(),i[1].y())));
		}
		GEOM_FADE2D::ConstraintGraph2* pCG=dt.createConstraint(vSegments,GEOM_FADE2D::CIS_CONSTRAINED_DELAUNAY);
		
		
		// Makes the constrained delauney a delauney respecting the boundaries by adding the intersection point found on the boundary after delaunay
		bool aplyConstrainedDelaunay = true;
		if(!aplyConstrainedDelaunay)
		{
			double minLen(0.1);
			pCG->makeDelaunay(minLen);
		}
		// Draw; generate a Design.ps file for visualization in for example illustrator 
		//dt.show("Design.ps",true);
				
		
		// Export fade2D data to Toolbox information format
		GEOM_FADE2D::FadeExport fadeExport;
		bool bCustomIndices(false); // To retrieve custom indices also
		bool bClear(false); // Clear memory in $dt
		dt.exportTriangulation(fadeExport,bCustomIndices,bClear);
		
		for(int triIdx=0;triIdx<fadeExport.numTriangles;++triIdx)
		{ // Iterate through each triangle in Fade_2D and store triangle in Toolbox entities
			int vtxIdx0,vtxIdx1,vtxIdx2;
			fadeExport.getCornerIndices(triIdx,vtxIdx0,vtxIdx1,vtxIdx2);
			
			double x0,y0;
			double x1,y1;
			double x2,y2;
			fadeExport.getCoordinates(vtxIdx0,x0,y0);
			fadeExport.getCoordinates(vtxIdx1,x1,y1);
			fadeExport.getCoordinates(vtxIdx2,x2,y2);
			
			// Define triangle in Toolbox entities
			utilities::geometry::vertex C1 = {x0,y0,0};
			utilities::geometry::vertex C2 = {x1,y1,0};
			utilities::geometry::vertex C3 = {x2,y2,0};
			
			if(C1.isSameAs(C2, mTol) || C2.isSameAs(C3, mTol) || C3.isSameAs(C1, mTol))
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, the delaunay traingulation (Fade_2D) generated a triangle with two the same vertices. \n"
					 << "The specific coördinates inserted are: " <<  C1 << " ; " << C2 << " ; " << C3 << "\n"
					 << "All the inserted vertices into the delaunay triangulation are: " << std::endl;
					 for(const auto t: delauneyPoints){errorMessage << t;}
					 errorMessage << std::endl;
					 duplicates = false; 
					 for(int t =0; t< delauneyPoints.size(); t++)
					 {
						for(int s=0; s<delauneyPoints.size(); s++)
						{
							if(s==t){continue;}
							else if(delauneyPoints[t].isSameAs(delauneyPoints[s], 0.1))
							{
								duplicates = true;
							}
						}
					 }
					 errorMessage << "There are " << duplicates << " duplicate vertices (0 means false and 1 means true). \n" 
							 << "The following lines are the constrain lines used during delauney: " << std::endl;
							 for(const auto t: delauneyLines){errorMessage << t << std::endl;}
				
					 std::vector<utilities::geometry::vertex> noDelauneyPoint;
					 for(auto t: delauneyLines)
					 {
						for(const auto k: t)
						{
							if (std::find(delauneyPoints.begin(),delauneyPoints.end(),k) == delauneyPoints.end())
							{
								noDelauneyPoint.push_back(k);
							}
						}
					 }
					 errorMessage << "The following vertices are end vertices of the constraint lines, but not a delauney points: " << std::endl;
					 for(const auto t: noDelauneyPoint){errorMessage << t;}
					 errorMessage << "\n (bso/spatial_design/conformal/cf_building_model)." << std::endl;
					 throw std::runtime_error(errorMessage.str());
			}
			
			// Skip trianlges wherefrom all vertices fall on one line acording to BSO Toolbox tolerences
				// This is needed since FADE_2D calculates with a finer tolerance
				// and may therefore create unlogical triangles from the BSO Toolbox perspective
				// See paper automatic partitioning methods for more information
			utilities::geometry::line_segment L1 = {{C1},{C2}};
			utilities::geometry::line_segment L2 = {{C2},{C3}};
			utilities::geometry::line_segment L3 = {{C3},{C1}};
			if(L1.isOnLine(C3, mTol) || L2.isOnLine(C1, mTol) || L3.isOnLine(C2, mTol))
			{
				//std::cout << "This triangle is not added: {"<<x0<<","<<y0<<"; "<<x1<<","<<y1<<"; "<<x2<<","<<y2<< "}"<<std::endl;
				continue;
			}
			
			try
			{
				utilities::geometry::triangle tri = {{C1},{C2},{C3}};
				delaunayTri.push_back(tri);
			}
			catch(std::exception& e)
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, The triangle resulting from the Delauney triangulation Fade_2D was unviable for the Toolbox. \n"
							 << "The specific coördinates inserted are: " <<  C1 << " ; " << C2 << " ; " << C3 << "\n"
							 << "The following error message was provided: \n"
							 << e.what() << "\n"
							 
							 << "The following information was profided to the Delauney triangulation: \n"
							 << "All the inserted vertices into the delaunay triangulation are: " << std::endl;
							 for(const auto t: delauneyPoints){errorMessage << t;}
							 errorMessage << std::endl;
							 
							duplicates = false; 
							duplicatePoints.clear();
							for(int t =0; t< delauneyPoints.size(); t++)
							{ //Check for duplicate vertices in the delaunay point variable
								for(int s=0; s<delauneyPoints.size(); s++)
								{
									if(s==t){continue;}
									else if(delauneyPoints[t].isSameAs(delauneyPoints[s], mTol))
									{
										duplicates = true;
										duplicatePoints.push_back(delauneyPoints[s]);
									}
								}
							}
							
							if(duplicates)
							{
								errorMessage << "There are duplicate vertices found in the DelauneyPoints inserted in the delaunay method. \n" 
											 << "The point(s) which has a duplicate are: " << std::endl;
								for(const auto t: duplicatePoints){errorMessage << t;}
								errorMessage << std::endl;
							}
							else
							{
								errorMessage << "There are no duplicate vertices found in the delaunayPoints with a check containing a tolerance. " << std::endl;
							}
							
							errorMessage << "The following lines are the constrain lines used during delauney: " << std::endl;
							for(const auto t: delauneyLines){errorMessage << t << std::endl;}
							errorMessage << std::endl;
							noDelauneyPoint.clear();
							pointOnConstraintLine.clear();
							contrainLines.clear();
							for(auto t: delauneyLines)
							{
								for(const auto k: t)
								{
									if (std::find(delauneyPoints.begin(),delauneyPoints.end(),k) == delauneyPoints.end())
									{
										noDelauneyPoint.push_back(k);
									}
									if(t.isOnLine(k))
									{
										pointOnConstraintLine.push_back(k);
									}
								}
								for(auto k: delauneyLines)
								{
									if(k.isSameAs(t, mTol)){continue;}
									for(const auto s: k)
									{
										if(t.isOnLine(s, mTol))
										{
											pointOnConstraintLine.push_back(s);
											contrainLines.push_back(t);
										}
									}
								}
							}
							
							if(noDelauneyPoint.size() > 0)
							{
								errorMessage << "The following vertices are end vertices of the constraint lines, but not a delauney points (Checked with tolerance): " << std::endl;
								for(const auto t: noDelauneyPoint){errorMessage << t;}
								errorMessage << std::endl;
							}
							else
							{
								errorMessage << "All the end points of the constrain lines are provided in the DelauneyPoints." << std::endl;
							}
							
							if(pointOnConstraintLine.size() > 0)
							{
								int counter=0;
								errorMessage << "The following vertices are vertices on the constrain line, this is not allowed and should have been solved in the splitLines function. See the following vertices: " << std::endl;
								for(const auto t: pointOnConstraintLine)
								{
									errorMessage << "vertex: " << t << " has been found on: " << contrainLines[counter] << std::endl;
									counter++;
								}
							}
							else
							{
								errorMessage << "There are no end vertices of the contrains lines on another constrain lines. " << std::endl;
							}						
							errorMessage << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
		
		if(delaunayTri.size() == 0)
		{
			std::stringstream errorMessage;
				errorMessage << "\nError, there are no triangle generated during the delaunay triangulation. \n"
					 << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
				throw std::runtime_error(errorMessage.str());
		}
		
		return delaunayTri;
	} //delaunay()
	
	void cf_building_model::makeConformal() // Origonal orthogonal rectangular geometry conformal method
	{ // 
		// check for line line intersections, and add the found vertex to the geometry model
		utilities::geometry::vertex pIntersection;
		for (unsigned int i = 0; i < mCFLines.size(); ++i)
		{
			for (unsigned int j = i+1; j < mCFLines.size(); ++j)
			{
				if (mCFLines[i]->intersectsWith(*(mCFLines[j]), pIntersection, mTol))
				{
					this->addVertex(pIntersection);
				}
			}
		}


		// check for line - rectangle intersections, and add the found vertex to the geometry model
		for (unsigned int i = 0; i < mCFRectangles.size(); ++i)
		{
			for (unsigned int j = 0; j < mCFLines.size(); ++j)
			{
				if (mCFRectangles[i]->intersectsWith(*(mCFLines[j]), pIntersection, mTol))
				{
					this->addVertex(pIntersection);
				}
			}
		}


		// check each vertex if it intersects with any space	
		unsigned int i = 0;
		while (i < mCFVertices.size())
		{
			for (auto& j : mCFSpaces)
			{
				j->checkVertex(mCFVertices[i]);
			}
			++i;
		}

		
		// delete the lines, rectangles, and cuboids that were tagged for deletion
		mCFLines.erase(std::remove_if(mCFLines.begin(), mCFLines.end(), [](const auto& i){ return i->deletion(); }), mCFLines.end());
		mCFRectangles.erase(std::remove_if(mCFRectangles.begin(), mCFRectangles.end(), [](const auto& i){ return i->deletion(); }), mCFRectangles.end());
		mCFCuboids.erase(std::remove_if(mCFCuboids.begin(), mCFCuboids.end(), [](const auto& i){ return i->deletion(); }), mCFCuboids.end());
			
	} //  makeConformal()
	
	
	void cf_building_model::checkEntities()
	{// Additional checks not provided intrincically in the partition methods
		std::cout << "start checkEntities()" << std::endl;		
		
		// Checking for overlapping geometry in the geometry conformal model
		for(const auto i: mCFCuboids)
		{
			for(const auto j: mCFCuboids)
			{
				if(i == j)
				{
					continue;
				}
				
				for(const auto k: (*j))
				{
					if(i->isInside(k, mTol))
					{
						std::stringstream errorMessage;
						errorMessage << "The checkEntities() function result in the following conclusion:\n"
												 << "There is overlapping geometry detected in the mCFCuboids. \n"
												 << "Meaning that the geometry conformal model is incorrect. \n"
												 << "The cuboids considered are:  \n"
												 << *i  << "\n"
												 << *j  << "\n"
												 << "This check is performed by checking if one of the cornerpoints, "
												 << "of the second cuboid are Inside the first cuboid\n"
												 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
						throw std::invalid_argument(errorMessage.str());
					}
				}
			}
		}
		
		for(const auto i: mCFTetrahedrons)
		{
			for(const auto j: mCFTetrahedrons)
			{
				if(i == j)
				{
					continue;
				}
				
				for(const auto k: (*j))
				{
					if(i->isInside(k, mTol))
					{
						std::stringstream errorMessage;
						errorMessage << "The checkEntities() function result in the following conclusion:\n"
												 << "There is overlapping geometry detected in the mCFTetrahedrons. \n"
												 << "Meaning that the geometry conformal model is incorrect. \n"
												 << "The tetrahedrons considered are:  \n"
												 << *i  << "\n"
												 << *j  << "\n"
												 << "This check is performed by checking if each of the cornerpoints, "
												 << "of the second tetrahedrons are Inside the first tetrahedron\n"
												 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
						throw std::invalid_argument(errorMessage.str());
					}
				}
			}
		}
		
		for(const auto i: mCFTriPrisms)
		{
			for(const auto j: mCFTriPrisms)
			{
				if(i == j)
				{
					continue;
				}
				for(const auto k: (*j))
				{
					if(i->isInside(k, mTol))
					{
						std::stringstream errorMessage;
						errorMessage << "The checkEntities() function result in the following conclusion:\n"
												 << "There is overlapping geometry detected in the mCFTriPrisms. \n"
												 << "Meaning that the geometry conformal model is incorrect. \n"
												 << "The triPrisms considered are:  \n"
												 << *i  << "\n"
												 << *j  << "\n"
												 << "This check is performed by checking if each of the cornerpoints, "
												 << "of the second triangular_prism are Inside the first triangular_prism\n"
												 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
						throw std::invalid_argument(errorMessage.str());
					}
				}
			}
		}
		
		for(const auto i: mCFRectangles)
		{
			for(const auto j: mCFRectangles)
			{
				if(i == j)
				{
					continue;
				}
				for(const auto k: (*j))
				{
					if(i->isInside(k, mTol))
					{
						std::stringstream errorMessage;
						errorMessage << "The checkEntities() function result in the following conclusion:\n"
												 << "There is overlapping geometry detected in the mCFRectangles. \n"
												 << "Meaning that the geometry conformal model is incorrect. \n"
												 << "The rectangles considered are:  \n"
												 << *i  << "\n"
												 << *j  << "\n"
												 << "This check is performed by checking if each of the cornerpoints, "
												 << "of the second rectangle are Inside or on the first rectangle\n"
												 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
						throw std::invalid_argument(errorMessage.str());
					}
				}
			}
		}
		
		for(const auto i: mCFTriangles)
		{
			for(const auto j: mCFTriangles)
			{
				if(i == j)
				{
					continue;
				}
				for(const auto k: (*j))
				{
					if(i->isInside(k, mTol))
					{
						std::stringstream errorMessage;
						errorMessage << "The checkEntities() function result in the following conclusion:\n"
												 << "There is overlapping geometry detected in the mCFTriangles. \n"
												 << "Meaning that the geometry conformal model is incorrect. \n"
												 << "The triangles considered are: \n"
												 << *i  << "\n"
												 << *j  << "\n"
												 << "This check is performed by checking if each of the cornerpoints, "
												 << "of the second triangle are Inside or on the first triangle\n"
												 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
						throw std::invalid_argument(errorMessage.str());
					}
				}
			}
		}
		
		for(const auto i: mCFRectangles)
		{
			for(const auto j: mCFTriangles)
			{
				for(const auto k: (*j))
				{
					if(i->isInside(k, mTol))
					{
						std::stringstream errorMessage;
						errorMessage << "The checkEntities() function result in the following conclusion:\n"
												 << "There is overlapping geometry detected between a mCFRectangles and a mCFTriangles. \n"
												 << "Meaning that the geometry conformal model is incorrect. \n"
												 << "The rectangle and triangle considered are:  \n"
												 << *i  << "\n"
												 << *j  << "\n"
												 << "This check is performed by checking if each of the cornerpoints, "
												 << "of the triangle are Inside or on the first rectangle\n"
												 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
						throw std::invalid_argument(errorMessage.str());
					}
				}
			}
		}
		
		for(const auto i: mCFLines)
		{
			for(const auto j: mCFLines)
			{
				if(i == j)
				{
					continue;
				}
				if((i->getVector()).isCodirectional((j->getVector()), mTol))
				{
					for(const auto k: (*j))
					{
						if(i->isOnLine(k, mTol))
						{
							if((k.isSameAs((*i)[0], mTol)) || (k.isSameAs((*i)[1], mTol)))
							{
								continue;
							}
							
							std::stringstream errorMessage;
							errorMessage << "The checkEntities() function result in the following conclusion:\n"
													 << "There is overlapping geometry detected in the mCFLines. \n"
													 << "Meaning that the geometry conformal model is incorrect. \n"
													 << "The lines considered are:  \n"
													 << *i  << "\n"
													 << *j  << "\n"
													 << "This check is performed by first checking if the lines are codirectional, "
													 << "if so, then it is checked if the cornerpoint of the second line is on line one. \n"
													 << "if so, then it is checked if the cornerpoint of the second line is the same as either one of the cornerpoints of the first line. \n"
													 << "if so, then there is no overlap. If not, then there is overlap. \n"
													 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
							throw std::invalid_argument(errorMessage.str());
						}
					}
				}
			}
		}
		//Please note that there can not be duplicate vertices duo to the way the addVertex() function in cf_geometry_model is defined.
		//Therefore, it is not needed to check this again in the checkEntities() funciton
		
		
		// Check if the full space volume is decribed with cells
		for(const auto i: mCFSpaces)
		{
			double spaceVolume = i->getVolume();
			double cellVolume = 0;
			if(methodUsed == 'I' || methodUsed == 'O' || methodUsed == 'M')
			{
				for(const auto j: (i->cfCuboids()))
				{
					cellVolume = cellVolume + j->getVolume();
				}
			}
			else if(methodUsed == 'T')
			{
				for(const auto j: (i->cfTetrahedrons()))
				{
					cellVolume = cellVolume + j->getVolume();
				}
			}
			else if(methodUsed == 'D')
			{
				for(const auto j: (i->cfTriPrism()))
				{
					cellVolume = cellVolume + j->getVolume();
				}
			}
			else
			{
				std::stringstream errorMessage;
				errorMessage << "The checkEntities() function result in the following conclusions:\n"
										 << "The methods inserted is not supported in the checkEntities() function. \n"
										 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
				throw std::invalid_argument(errorMessage.str());
			}
			
			if((spaceVolume-cellVolume) < (0-10) || (spaceVolume-cellVolume) > (0+10)) // The standart tolerance is too small and trikkers the error unrightfully fast
			{
				std::stringstream errorMessage;
				errorMessage << "The checkEntities() function result in the following conclusions:\n"
										 << "The volume of the space is not fully decribed with the cells. \n"
										 << "Meaning that there are cells missing. \n"
										 << "This has been concluded on the basis of the diference between the space volume, \n"
										 << "and the sum of all the cells volume, which is: " << (spaceVolume-cellVolume) << "\n"
										 << "the spaceVolume is: " << spaceVolume << " and the cellVolume is: " << cellVolume << "\n"
										 << "the space considered is: " << *i << "\n"
										 << "the cells within the space are: " << std::endl;
										 if(methodUsed == 'D'){for(const auto j: (i->cfTriPrism())){errorMessage << *j << std::endl;}}
										 else if(methodUsed == 'T'){for(const auto j: (i->cfTetrahedrons())){errorMessage << *j << std::endl;}}
										 else if(methodUsed == 'I' || methodUsed == 'O' || methodUsed == 'M'){for(const auto j: (i->cfCuboids())){errorMessage << *j << std::endl;}}
							errorMessage << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
				throw std::invalid_argument(errorMessage.str());
			}
		}
		
		// Check if the full surface is decribed with rectangles/triangles
		for(const auto i: mCFSurfaces)
		{
			double surfaceArea = i->getArea();
			double recttrisurface = 0;
			if(methodUsed == 'I' || methodUsed == 'O' || methodUsed == 'M')
			{
				for(const auto j: (i->cfRectangles()))
				{
					recttrisurface = recttrisurface + j->getArea();
				}
			}
			else if(methodUsed == 'T')
			{
				for(const auto j: (i->cfTriangles()))
				{
					recttrisurface = recttrisurface + j->getArea();
				}
			}
			else if(methodUsed == 'D')
			{
				for(const auto j: (i->cfTriangles()))
				{
					recttrisurface = recttrisurface + j->getArea();
				}
				for(const auto j: (i->cfRectangles()))
				{
					recttrisurface = recttrisurface + j->getArea();
				}
			}
			else
			{
				std::stringstream errorMessage;
				errorMessage << "The checkEntities() function result in the following conclusions:\n"
										 << "The methods inserted is not supported in the checkEntities() function. \n"
										 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
				throw std::invalid_argument(errorMessage.str());
			}
			
			if((surfaceArea-recttrisurface) < (0-10) || (surfaceArea-recttrisurface) > (0+10)) // The standart tolerance is too small and trikkers the error unrightfully fast
			{
				std::stringstream errorMessage;
				errorMessage << "The checkEntities() function result in the following conclusions:\n"
										 << "The area of the surface is not fully decribed with rectangles/triangles. \n"
										 << "Meaning that there are rectangles/triangles missing. \n"
										 << "This has been concluded on the basis of the diference between the surface area, \n"
										 << "and the sum of all the rectangles or triangles area, which is: " << (surfaceArea-recttrisurface) << "\n"
										 << "the surface area is: " << surfaceArea << " and the rectangle/triangle area is: " << recttrisurface << "\n"
										 << "the surface considered is: " << *i << "\n"
										 << "the rectangle or triangle within the surface are: " << std::endl;
										 if(methodUsed == 'D'){for(const auto j: (i->cfTriangles())){errorMessage << *j << std::endl;}
										 for(const auto j: (i->cfRectangles())){errorMessage << *j << std::endl;}}
										 else if(methodUsed == 'T'){for(const auto j: (i->cfTriangles())){errorMessage << *j << std::endl;}}
										 else if(methodUsed == 'I' || methodUsed == 'O' || methodUsed == 'M'){for(const auto j: (i->cfRectangles())){errorMessage << *j << std::endl;}}
							errorMessage << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
				throw std::invalid_argument(errorMessage.str());
			}
		}
		
		
		// Check if the full edge is decribed with lines
		for(const auto i: mCFEdges)
		{
			double edgeLength = i->getLength();
			double lineLength = 0;
			
			for(const auto j: (i->cfLines()))
			{
				lineLength = lineLength + j->getLength();
			}
			
			if((edgeLength-lineLength) < (0-10) || (edgeLength-lineLength) > (0+10)) // The standart tolerance is too small and trikkers the error unrightfully fast
			{
				std::stringstream errorMessage;
				errorMessage << "The checkEntities() function result in the following conclusions:\n"
										 << "The length of the edge is not fully decribed with lines. \n"
										 << "Meaning that there are lines missing. \n"
										 << "This has been concluded on the basis of the diference between the edge length, \n"
										 << "and the sum of all the lines length, which is: " << (edgeLength-lineLength) << "\n"
										 << "the edge length is: " << edgeLength << " and the combined line length is: " << lineLength << "\n"
										 << "the edge considered is: " << *i << "\n"
										 << "the lines within the edge are: " << std::endl;
										 for(const auto j: (i->cfLines())){errorMessage << *j << std::endl;}
							errorMessage << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
				throw std::invalid_argument(errorMessage.str());
			}
			
		}
		
		
		//Check for intersection between 2 lines or a polygon and a line.
		utilities::geometry::vertex pIntersection;
		for (unsigned int i = 0; i < mCFLines.size(); ++i)
		{// check for line line intersections
			for (unsigned int j = i+1; j < mCFLines.size(); ++j)
			{
				if (mCFLines[i]->intersectsWith(*(mCFLines[j]), pIntersection, mTol))
				{
					std::stringstream errorMessage;
					errorMessage << "The checkEntities() function result in the following conclusion:\n"
											 << "There is an intersection found between two lines while this should have been resolved in the automatic partitioning method. \n"
											 << "Meaning that the geometry conformal model is incorrect. \n"
											 << "The lines considered are:  \n"
											 << *(mCFLines[i])  << "\n"
											 << *(mCFLines[j])  << "\n"
											 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
					throw std::invalid_argument(errorMessage.str());
				}
			}
		}
		for (unsigned int j = 0; j < mCFLines.size(); ++j)
		{// check for line - polygon intersections
			for (unsigned int i = 0; i < mCFRectangles.size(); ++i)
			{
				if (mCFRectangles[i]->intersectsWith(*(mCFLines[j]), pIntersection, mTol))
				{
					std::stringstream errorMessage;
					errorMessage << "The checkEntities() function result in the following conclusion:\n"
											 << "There is an intersection found between a polygon and a lines while this should have been resolved in the automatic partitioning method. \n"
											 << "Meaning that the geometry conformal model is incorrect. \n"
											 << "The polygon and lines considered are:  \n"
											 << *(mCFRectangles[i])  << "\n"
											 << *(mCFLines[j])  << "\n"
											 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
					throw std::invalid_argument(errorMessage.str());
				}
			}
			for (unsigned int i = 0; i < mCFTriangles.size(); ++i)
			{
				if (mCFTriangles[i]->intersectsWith(*(mCFLines[j]), pIntersection, mTol))
				{
					std::stringstream errorMessage;
					errorMessage << "The checkEntities() function result in the following conclusion:\n"
											 << "There is an intersection found between a polygon and a lines while this should have been resolved in the automatic partitioning method. \n"
											 << "Meaning that the geometry conformal model is incorrect. \n"
											 << "The polygon and lines considered are:  \n"
											 << *(mCFTriangles[i])  << "\n"
											 << *(mCFLines[j])  << "\n"
											 << "(bso/utilities/geomtry/cf_building_model.cpp)" << std::endl;
					throw std::invalid_argument(errorMessage.str());
				}
			}
		}
		
		
		
		
		
		/*
		//Check if the space is generated with the acording geometry conformal model information, a.k.a. cells, rectangles, and lines.
		// entities in cf_building_model
		bool cfSpacesCorect = true;
		for (const auto i: mCFSpaces)
		{
			if((i->cfSpaces()).size() > 1){cfSpacesCorect = false;}
			if(!(i->cfSurfaces()).size() == 6){cfSpacesCorect = false;}
			if(!(i->cfEdges()).size() == 12){cfSpacesCorect = false;}
			if(!(i->cfPoints()).size() == 8){cfSpacesCorect = false;}
			
			
			for(const auto j: i->cfCuboids())
			{
				if (std::find(mCFCuboids.begin(),mCFCuboids.end(),j) == mCFCuboids.end()){cfSpacesCorect = false;}
				for(const auto k: j->cfRectangles())
				{
					if (std::find(mCFRectangles.begin(),mCFRectangles.end(),k) == mCFRectangles.end()){cfSpacesCorect = false;}
					for(const auto l: j->cfLines())
					{
						if (std::find(mCFLines.begin(),mCFLines.end(),l) == mCFLines.end()){cfSpacesCorect = false;}
						for(const auto m: l->cfVertices())
						{
							if (std::find(mCFVertices.begin(),mCFVertices.end(),m) == mCFVertices.end()){cfSpacesCorect = false;}
						}
					}
				}
			}
			
			for(const auto j: i->cfRectangles())
			{
				if (std::find(mCFRectangles.begin(),mCFRectangles.end(),j) == mCFRectangles.end()){cfSpacesCorect = false;}
				for(const auto l: j->cfLines())
				{
					if (std::find(mCFLines.begin(),mCFLines.end(),l) == mCFLines.end()){cfSpacesCorect = false;}
					for(const auto m: l->cfVertices())
					{
						if (std::find(mCFVertices.begin(),mCFVertices.end(),m) == mCFVertices.end()){cfSpacesCorect = false;}
					}
				}
			}
			
			for(const auto j: i->cfLines())
			{
				if (std::find(mCFLines.begin(),mCFLines.end(),j) == mCFLines.end()){cfSpacesCorect = false;}
				for(const auto m: j->cfVertices())
				{
					if (std::find(mCFVertices.begin(),mCFVertices.end(),m) == mCFVertices.end()){cfSpacesCorect = false;}
				}
			}
			
			for(const auto j: i->cfVertices())
			{
				if (std::find(mCFVertices.begin(),mCFVertices.end(),j) == mCFVertices.end()){cfSpacesCorect = false;}
			}
		}
		
		if(cfSpacesCorect == false)
		{
			std::stringstream errorMessage;
			errorMessage << "\nError, the entities in the cf_building_model and cf_geometry_model do not match\n"
									 << "with the individual entity storage\n"
									 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		else
		{
			//std::cout << "all entities are correct!" << std::endl;
		}
		*/
	} // checkEntities()
	
	
	void cf_building_model::checkPartitionSize()
	{
		//Data over full geometry conformal model
		double amountOfPoly =0;
		double totalArea = 0;
		double lowestArea = ((cfRectangles())[0])->getArea();
		double highestArea = ((cfRectangles())[0])->getArea();
		
		
		//Data considering only triangles
		double amountOfPolyTri =0;
		double totalAreaTri = 0;
		double lowestAreaTri;
		double highestAreaTri;
		cf_triangle* highestAreaT;
		
		
		//Data considering only polygons
		double amountOfPolyQuad =0;
		double totalAreaQuad = 0;
		double lowestAreaQuad;
		double highestAreaQuad;
		cf_rectangle* highestAreaRec; 	
		
		
		//Initiate write to file for all designs
		std::ofstream allFile;
		allFile.open ("AreaAllDesigns", std::fstream::in | std::fstream::out | std::fstream::app);
		allFile << std::fixed << std::setprecision(0) << std::endl;
		
		std::ofstream allFileBL;
		allFileBL.open ("AreaAllDesigns", std::fstream::in | std::fstream::out | std::fstream::app);
		allFileBL << std::fixed << std::setprecision(0) << std::endl;
		
		
		//Initiate write to file per design
		std::string base = "Area_";
		std::string fileName = mMSModel.getInsertFileName();
		std::string newFileName = base + fileName;
		std::ofstream file;
		file.open (newFileName, std::fstream::out);
		
		
		//Insert file name
		allFile << fileName << "\t";
		allFileBL << fileName << "\t";
		file << fileName << std::endl;
		file << std::endl;		
		
		
		//Determine the surface area of rectangles and triangles
		file << "cf_rectangle" << std::endl;
		allFile << "cf_rectangle" << "\t";
		allFileBL << "cf_rectangle" << "\t";
		bool initiated = false;
		for(const auto i: cfRectangles())
		{
			if((i->cfSurfaces()).size() > 0)
			{ // only consider rectangles which belonging to an surface
				if(!initiated)
				{ // initiate the first value to be changed untill the final value is reached
					lowestAreaQuad = i->getArea();
					highestAreaQuad = i->getArea();
					highestAreaRec = i;
					highestAreaRec->markInVisualization = true;
					initiated = true;
				}
				
				amountOfPoly++;
				amountOfPolyQuad++;
				double temp = i->getArea();
				
				// full geometry conformal model
				totalArea = totalArea + temp;
				if(lowestArea > temp){lowestArea = temp;}
				if(highestArea < temp){highestArea = temp;}
				
				// all quads from geometry conformal model
				totalAreaQuad = totalAreaQuad + temp;
				if(lowestAreaQuad > temp){lowestAreaQuad = temp;}
				if(highestAreaQuad < temp)
				{
					highestAreaQuad = temp;
					highestAreaRec->markInVisualization = false;
					i->markInVisualization = true;
					highestAreaRec = i;
				}
				
				// Write to file
				file << temp << std::endl;
				allFile << temp << "\t";
			}
		}
		
		file << std::endl;
		file << "cf_triangle" << std::endl;
		allFile << "cf_triangle" << "\t";
		initiated = false;
		if(cfTriangles().size() > 0)
		{
			for(const auto i: cfTriangles())
			{
				if((i->cfSurfaces()).size() > 0)
				{ // only consider triangles which belonging to an surface
					if(!initiated)
					{ // initiate the first value to be changed untill the final value is reached
						lowestAreaTri = i->getArea();
						highestAreaTri = i->getArea();
						highestAreaT = i;
						highestAreaT->markInVisualization = true;
						initiated = true;
					}
					
					amountOfPoly++;
					amountOfPolyTri++;
					double temp = i->getArea();
					
					// full geometry conformal model
					totalArea = totalArea + temp;
					if(lowestArea > temp){lowestArea = temp;}
					if(highestArea < temp){highestArea = temp;}
					
					// all Triangles from geometry conformal model
					totalAreaTri = totalAreaTri + temp;
					if(lowestAreaTri > temp){lowestAreaTri = temp;}
					if(highestAreaTri < temp)
					{
						highestAreaTri = temp;
						highestAreaT->markInVisualization = false;
						i->markInVisualization = true;
						highestAreaT = i;
					}
					
					// Write to file
					file << temp << std::endl;
					allFile << temp << "\t";
				}
			}
		}
		
		
		// Determine the skewness of rectangles and triangles
		// https://www.engmorph.com/skewness-finite-elemnt
		/*
		for(const auto i: cfRectangles())
		{
			if((i->cfSurfaces()).size() > 0)
			{ // only consider rectangles which belonging to an surface
				std::vector<bso::utilities::geometry::vertex> mVertices
				for(auto j = i.begin(); j != i.end(); j++)
				{
					mVertices.push_back(j);
				}
				
				utilities::geometry::vertex midPoint_1 = mVertices[0]-mVertices[1];
				utilities::geometry::vertex midPoint_2 = mVertices[1]-mVertices[2];
				utilities::geometry::vertex midPoint_3 = mVertices[2]-mVertices[3];
				utilities::geometry::vertex midPoint_4 = mVertices[3]-mVertices[0];
				
				
				
				
				
				
			}
		}
		
		
		if(cfTriangles().size() > 0)
		{
			for(const auto i: cfTriangles())
			{ // only consider triangles which belonging to an surface
				
				
				
			}
		}
		
		
		
		
		// Determine the aspect ratio of rectangles and triangles
		for(const auto i: cfRectangles())
		{
			if((i->cfSurfaces()).size() > 0)
			{ // only consider rectangles which belonging to an surface
				
				
				
				
			}
		}
		
		
		if(cfTriangles().size() > 0)
		{
			for(const auto i: cfTriangles())
			{ // only consider triangles which belonging to an surface
				
				
				
			}
		}
		*/
		
		
		// Close all files
		file.close();
		allFile.close();
		
		
		// Cout algemene informatie naar de console
		double avarageArea = totalArea/amountOfPoly;
		
		//std::cout << "The lowest Area is: " << lowestArea << std::endl;
		//std::cout << "The highest Area is: " << highestArea << std::endl;
		//std::cout << "The avarage Area is: " << avarageArea << std::endl;
		
		if(cfTriangles().size() > 0)
		{
			double avarageAreaTri = totalAreaTri/amountOfPolyTri;
			//std::cout << "The lowest AreaTri is: " << lowestAreaTri << std::endl;
			//std::cout << "The highest AreaTri is: " << highestAreaTri << std::endl;
			//std::cout << "The avarage AreaTri is: " << avarageAreaTri << std::endl;
		}
		
		double avarageAreaQuad = totalAreaQuad/amountOfPolyQuad;
		//std::cout << "The lowest AreaQuad is: " << lowestAreaQuad << std::endl;
		//std::cout << "The highest AreaQuad is: " << highestAreaQuad << std::endl;
		//std::cout << "The avarage AreaQuad is: " << avarageAreaQuad << std::endl;
		
		
	} // checkPartitionSize()
	
	int cf_building_model::getNrInitInters() const
	{
		std::vector <cf_vertex> pIntersectionV;
		utilities::geometry::vertex pIntersection;	

		for (const auto& i: mCFSpaces) //interSpace
		{			
			for (const auto& j: mCFSpaces)
			{
				if(i == j) continue;

				for(const auto& o: (i->cfCuboids()))
				{						
					for(const auto& p: (j->cfCuboids()))
					{
						if(o == p) continue;
						// check for line - rectangle intersections, and add the found vertex to the geometry model and pIntersection
						for (const auto k: o->cfRectangles())
						{
							for (const auto l: p->getLines())
							{
								if (k->intersectsWith(l, pIntersection, mTol))
								{									
									bool doubleInsert = false;
									for (const auto& m : pIntersectionV)
									{// check if pIntersection already exist in pIntersectionV
										if (m.isSameAs(pIntersection, mTol))
										{
											doubleInsert = true;
										}
									}
									
									if (doubleInsert == false)
									{
										pIntersectionV.push_back(pIntersection);
									}
								}
							}
						}
						
						// check for line line intersections, and add the found vertex to the geometry model and pIntersection
						for (const auto k: o->getLines())
						{
							for (const auto l: p->getLines())
							{
								if(k == l)
								{
									continue;
								}
								else if (k.intersectsWith(l, pIntersection, mTol))
								{									
									bool doubleInsert = false;
									for (const auto& m : pIntersectionV)
									{// check if pIntersection already exist in pIntersectionV
										if (m.isSameAs(pIntersection, mTol))
										{
											doubleInsert = true;
										}
									}
									
									if (doubleInsert == false)
									{
										pIntersectionV.push_back(pIntersection);
									}
								}
							}
						}
					}
				}
			}
		}

		for (const auto& h: mCFSpaces) // intercell
		{		
			for(int o=0; o < (h->cfCuboids()).size(); o++)
			{						
				for(int p=0; p <(h->cfCuboids()).size(); p++)
				{
					// check for line - rectangle intersections, and add the found vertex to the geometry model and pIntersection							
					for (const auto k: (h->cfCuboids())[o]->getCellWalls())
					{
						for (const auto l: ((h->cfCuboids())[p])->getLines())
						{								
							if (k->intersectsWith(l, pIntersection, mTol))
							{
								bool doubleInsert = false;
								for (const auto& m : pIntersectionV)
								{// check if pIntersection already exist in pIntersectionV
									if (m.isSameAs(pIntersection, mTol))
									{
										doubleInsert = true;
									}
								}
								
								if (doubleInsert == false)
								{
									pIntersectionV.push_back(pIntersection);
								}
							}
						}
					}
					
					// check for line line intersections, and add the found vertex to the geometry model and pIntersection
					for (const auto k: ((h->cfCuboids())[o])->getLines())
					{
						for (const auto l: ((h->cfCuboids())[p])->getLines())
						{
							if(k.isSameAs(l, mTol))
							{
								continue;
							}
							else if (k.intersectsWith(l, pIntersection, mTol))
							{										
								bool doubleInsert = false;
								for (const auto& m : pIntersectionV)
								{// check if pIntersection already exist in pIntersectionV
									if (m.isSameAs(pIntersection, mTol))
									{
										doubleInsert = true;
									}
								}
								
								if (doubleInsert == false)
								{
									pIntersectionV.push_back(pIntersection);
								}
							}
						}
					}							
				}
			}
		}

		return pIntersectionV.size();
	} // getNrInitInters()

	int cf_building_model::getNrIterationsinterSpace() const
	{
		return interspaceCount;
	} //getNrIterationsinterSpace()

	int cf_building_model::getNrIterationsinterCell() const
	{
		return intercellCount;
	} //getNrIterationsinterCell()
	
	int cf_building_model::getNrCells() const
	{
		if(methodUsed == 'D') return cfTriPrism().size();
		else return cfCuboids().size();
	} //getNrCells()

	cf_building_model::cf_building_model(const cf_building_model& rhs, bool solyForVisualization /*=false*/)
	{
		if(solyForVisualization)
		{
			mCFPoints = rhs.cfPoints();
			mCFEdges = rhs.cfEdges();
			mCFSurfaces = rhs.cfSurfaces();
			mCFSpaces = rhs.cfSpaces();
			mCFVertices = rhs.cfVertices();
			mCFLines = rhs.cfLines();
			mCFRectangles = rhs.cfRectangles();
			mCFTriangles = rhs.cfTriangles();
			mCFCuboids = rhs.cfCuboids();
			mCFTetrahedrons = rhs.cfTetrahedrons();
			mCFTriPrisms = rhs.cfTriPrism();
			mTol = tolerance();
		}
		else
		{
			auto newPtr = new cf_building_model(rhs.mMSModel, rhs.methodUsed, rhs.mTol);
			*this = *newPtr;
		}
	} // copy ctor()
	
	
	cf_building_model::cf_building_model(const ms_building& msModel, const double& tol , bool alreadyConformal)
	:	cf_geometry_model(tol), mMSModel(msModel), mTol(tol), methodUsed('I') //constructor using standart geometry conformal method
	{ // 		
		// Methods:
		// I = quad-hexahedron; sub-method: perpendicular to intersection side
		// O = quad-hexahedorn; sub-method: perpendicular to oposide side
		// M = quad-hexahedorn; sub-method: middle ratio method (not finisched)
		// T = tetrahedron method
		
		if (methodUsed == 'I' || methodUsed == 'O' || methodUsed == 'M')
		{
			for (const auto& i : mMSModel)
			{
				addSpace(*i);
			}
			if(alreadyConformal == true)
			{
				makeConformalN();
			}
		}
		else if (methodUsed == 'T')
		{
			for (const auto& i : mMSModel)
			{
				addSpaceT(*i);
			}
			if(alreadyConformal == true)
			{
				makeConformalT();
			}
		}
		else if (methodUsed == 'D')
		{
			if(alreadyConformal == true)
			{
				makeConformalND(msModel);
			}
			else if(alreadyConformal == false)
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, with split method D is no initial conformal model generated. \n"
					 << "The final conformal model is directly generated. \n"
					 << "Therefore, it is not allowed to saporately asked for the makeConformalND() function. \n"
					 << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
	} // 

	cf_building_model::cf_building_model(const ms_building& msModel, char method, const double& tol, bool alreadyConformal)
	:	cf_geometry_model(tol), mMSModel(msModel), mTol(tol), methodUsed(method) //constructor for user defined geometry conformal method
	{ // 	
		// Methods:
		// I = perpendicular to intersection side
		// O = perpendicular to oposide side
		// M = middle ratio method
		// T = tetrahedron method
		
		if (methodUsed == 'I' || methodUsed == 'O' || methodUsed == 'M')
		{
			for (const auto& i : mMSModel)
			{
				addSpace(*i);
			}
			if(alreadyConformal == true)
			{
				makeConformalN();
			}
		}
		else if (methodUsed == 'T')
		{
			for (const auto& i : mMSModel)
			{
				addSpaceT(*i);
			}
			if(alreadyConformal == true)
			{
				makeConformalT();
			}
		}
		else if (methodUsed == 'D')
		{
			if(alreadyConformal == true)
			{
				makeConformalND(msModel);
			}
			else if(alreadyConformal == false)
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, with split method D is no initial conformal model generated. \n"
					 << "The final conformal model is directly generated. \n"
					 << "Therefore, it is not allowed to saporately asked for the makeConformalND() function. \n"
					 << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
		else
		{
			std::stringstream errorMessage;
			errorMessage << "\nError, the choosen split method is non-existing \n"
					 << "Choose either an 'I', 'O', 'M', 'D' or 'T' method, see \n"
					 << "code cf_building_model.cpp for explaination of methods kinds"
					 << "(bso/spatial_design/conformal/cf_building_model)." << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
	} // 
	
	
	

	cf_building_model::~cf_building_model()
	{ // 
		for (auto& i : mCFSpaces) delete i;
		for (auto& i : mCFSurfaces) delete i;
		for (auto& i : mCFEdges) delete i;
		for (auto& i : mCFPoints) delete i;
	} // 
	
} // conformal
} // spatial_design
} // bso

#endif // CF_BUILDING_MODEL_CPP