#ifndef CF_SPACE_CPP
#define CF_SPACE_CPP

#include <bso/utilities/sort_clockwise.hpp>
//#include <memory>

namespace bso { namespace spatial_design { namespace conformal { 
	
	cf_space::cf_space(const utilities::geometry::quad_hexahedron& rhs, cf_building_model* buildingModel)
	:  utilities::geometry::quad_hexahedron(rhs,buildingModel->tolerance())
	{ // 
		mBuildingModel = buildingModel;
		mCFCuboids.push_back(mBuildingModel->addCuboid(*this));
		mCFCuboids.back()->addSpace(this);
	} // 
	
	cf_space::cf_space(const utilities::geometry::quad_hexahedron& rhs, cf_building_model* buildingModel, std::vector<utilities::geometry::tetrahedron> tet)
	:  utilities::geometry::quad_hexahedron(rhs,buildingModel->tolerance())
	{ // 
		mBuildingModel = buildingModel;
		for(const auto i : tet)
		{
			mCFTetrahedrons.push_back(mBuildingModel->addTetrahedron(i));
			mCFTetrahedrons.back()->addSpace(this);
		}
	} // 
	
	cf_space::cf_space(const utilities::geometry::quad_hexahedron& rhs, cf_building_model* buildingModel, std::vector<utilities::geometry::triangular_prism> triPrism)
	:  utilities::geometry::quad_hexahedron(rhs,buildingModel->tolerance())
	{ // 
		mBuildingModel = buildingModel;
		for(const auto i : triPrism)
		{
			mCFTriPrisms.push_back(mBuildingModel->addTriPrism(i));
			mCFTriPrisms.back()->addSpace(this);
		}
	} // 
	
	void cf_space::checkVertex(cf_vertex* pPtr) // orthogonal_rectangular step 2 of geometry conformal method
	{ // 
		// check if the vertex is in or on the space's geometry
		bool onGeometry = false;

		if ((this->isInside(*pPtr, mBuildingModel->tolerance()))) onGeometry = true;
		
		for (const auto& i : mPolygons)
		{
			if (onGeometry) break;
			if (i->isInside(*pPtr, mBuildingModel->tolerance()))
			{
				onGeometry = true;
				break;
			}
		}

		for (const auto& i : mLineSegments)
		{
			if (onGeometry) break;
			if (i.isOnLine(*pPtr, mBuildingModel->tolerance()))
			{
				onGeometry = true;
				break;
			}
		}
		
		if (!onGeometry) return;  // if it isn't return the function immediately

		// check each cuboid if it can be split by the vertex
		for (unsigned int i = 0; i < mCFCuboids.size(); ++i)
		{
			mCFCuboids[i]->split(pPtr);
		}
	} // checkVertex
	
	
	void cf_space::checkVertexNO(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method) // quad-hexahedron method step 2
	{
		std::vector<cf_cuboid*> c;
		for(const auto h: mCFCuboids)
		{
			c.push_back(h);
		}
		checkVertexNO(intsecV, intsecL, method, c);
	}
	
	
	void cf_space::checkVertexNO(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method, std::vector<cf_cuboid*> c) // quad-hexahedron method step 2
	{ // 
		// split intsecL on the hand of the found intsecV
		cf_space::splitLines(intsecV, intsecL);
		
		// Define cells to check
		std::vector<cf_cuboid*> checkCells = c;
		
		// check if there are vertexes on a polygon
		for (int i=0; i < intsecV.size(); i++)
		{
			bool onGeometry = false;
			int h = 0;
			while(h < checkCells.size())
			//for (unsigned int k = 0; k < mCFCuboids.size(); ++k)
			{
				onGeometry = false;
				
				for (const auto& j : checkCells[h]->cfRectangles())
				{
					if (onGeometry) {break;}
					if (j->isInside(intsecV[i], mBuildingModel->tolerance()))
					{
						onGeometry = true;
						std::vector <utilities::geometry::line_segment> linesToAccound;
						
						// find the conected lines to onPolyIntersection to accound for
						for(const auto l : intsecL)
						{
							if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								if (j->isInsideOrOn(l[0], mBuildingModel->tolerance()) && j->isInsideOrOn(l[1], mBuildingModel->tolerance()))
								{
									bool isCornerpoint = false;
									for(const auto m : j->getVertices())
									{
										if(m.isSameAs(l[0], mBuildingModel->tolerance()) || m.isSameAs(l[1], mBuildingModel->tolerance()))
										{
											isCornerpoint = true;
										}
									}
									if(!isCornerpoint)
									{
										linesToAccound.push_back(l);
									}
								}
							}
						}
						
						// generate the vector of vertices to split with
						std::vector<cf_vertex> splitVertexen; 
						splitVertexen.push_back(intsecV[i]);
						
						for(const auto k : linesToAccound)
						{
							if(k[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen.push_back(k[1]);
							}
							if(k[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen.push_back(k[0]);
							}
						}
						
						
						// remove ducplicates from splitVertexen
						auto splitVertexen_end = splitVertexen.end();
						for(auto it = splitVertexen.begin(); it != splitVertexen_end; ++it)
						{
							splitVertexen_end = std::remove(it + 1, splitVertexen_end, *it);
						}
						splitVertexen.erase(splitVertexen_end, splitVertexen.end());
						
						// pars split vertexes to split function
						std::vector<cf_cuboid*> newCells;
						if(checkCells[h]->splitN(&splitVertexen, method, newCells))
						{
							// adjust checkCells
							checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[h]), checkCells.end());
							for(const auto t: newCells)
							{
								checkCells.push_back(t);
							}
							// find the newly generated intersection points and split the acording lines
							splitLinesTwo(intsecV, intsecL);
							// make sure no Cell is skipped
							h--;
							break;
						}
					}
				}
				h++;
			}
		}
		
		
		// check if there are vertexes on the line_segments		
		int k = 0;
		while(k < checkCells.size())
		{
			bool onLFGeometry = false;
			std::vector<std::vector<cf_vertex>*> splitVLineF;
			std::vector<cf_vertex>* splitVertexen;
			for (int i=0; i < intsecV.size(); i++)
			{
				for (const auto& j : checkCells[k]->cfRectangles())
				{
					for(const auto h: j->cfLines())
					{
						if (h->isOnLine(intsecV[i], mBuildingModel->tolerance()))
						{
							//std::cout << "Vertex isOnLine " << intsecV[i] << std::endl;
							onLFGeometry = true;
							std::vector <utilities::geometry::line_segment> linesToAccound;
							
							// find the conected lines to onLineIntersection to accound for
							for(const auto l : intsecL)
							{
								if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									if (j->isInsideOrOn(l[0], mBuildingModel->tolerance()) && j->isInsideOrOn(l[1], mBuildingModel->tolerance()))
									{
										bool isCornerpoint = false;
										if(h->isOnLine(l[0], mBuildingModel->tolerance()) && h->isOnLine(l[1], mBuildingModel->tolerance()))
										{
											isCornerpoint = true;
										}
										for(const auto m : j->getVertices())
										{
											if(m.isSameAs(l[0], mBuildingModel->tolerance()) || m.isSameAs(l[1], mBuildingModel->tolerance()))
											{
												isCornerpoint = true;
											}
										}
										if(!isCornerpoint)
										{
											linesToAccound.push_back(l);
										}
									}
								}
							}
							
							// generate the vector of vertices to split with
							//std::unique_ptr<std::vector<cf_vertex>> splitVertexen (new std::vector<cf_vertex>);
							splitVertexen = new std::vector<cf_vertex>; // Datalek!!!
							splitVertexen->push_back(intsecV[i]);
							for(const auto l : linesToAccound)
							{
								if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									splitVertexen->push_back(l[1]);
								}
								if(l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									splitVertexen->push_back(l[0]);
								}
							}
							
							// Add splitVertexen for this intersection case for step 3 split to splitVLineF
							splitVLineF.push_back(splitVertexen);
							linesToAccound.clear();
						}
					}
				}
			}
			if(onLFGeometry)
			{
				//std::cout << "The size of splitVLineF.size(): " << splitVLineF.size() << std::endl;

				std::sort (splitVLineF.begin(), splitVLineF.end(), this->sortSplitV);
				int mCFCuboidsSize = checkCells.size();
				for(int j=0; j < splitVLineF.size(); j++)
				{
					// Send splitVlineF to split function
					std::vector<cf_cuboid*> newCells;
					if(checkCells[k]->splitN(splitVLineF[j], method, newCells))
					{
						checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[k]), checkCells.end());
						for(const auto t: newCells)
						{
							checkCells.push_back(t);
						}
						k--;
						splitLinesTwo(intsecV, intsecL);							
						break;
					}
				}
				for (auto& i : splitVLineF) delete i; //Datalek fixen
			}
			k++;
		}
	} // checkVertexNO
	
	
	void cf_space::checkVertexN(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method, cf_building_model* buildingModel) // quad-hexahedron method step 2
	{
		std::vector<cf_cuboid*> c;
		for(const auto h: mCFCuboids)
		{
			c.push_back(h);
		}
		checkVertexN(intsecV, intsecL, method, c, buildingModel);
	}
	
	
	void cf_space::checkVertexN(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method, std::vector<cf_cuboid*> c, cf_building_model* buildingModel) // quad-hexahedron method step 2
	{ // 
		//std::cout << "The intersection lines are: " << intsecL.size() << std::endl;
		// split intsecL on the hand of the found intsecV
		cf_space::splitLines(intsecV, intsecL);
		
		// Define cells to check
		std::vector<cf_cuboid*> checkCells = c;
		
		//std::cout << " " << std::endl;
		//std::cout << "Check Space: " << getSpaceID() << " consisting of: " << *this << std::endl;
		//std::cout << "The cells considered are: " << std::endl;
		//for(const auto i: checkCells)
		//{
			//std::cout << *i << std::endl;
			//std::cout << "    with floors: " << std::endl;
			//for (const auto& j : i->getCellFloors())
			//{
				//std::cout  << "    " << *j << std::endl;
			//}
			//std::cout << "    with walls: " << std::endl;
			//for (const auto& j : i->getCellWalls())
			//{
				//std::cout << "    " << *j << std::endl;
			//}
		//}
		//std::cout << "The intersection vertices are: " << intsecV.size() << std::endl;
		//for(const auto i : intsecV)
		//{
			//std::cout << i ;
		//}
		//std::cout << " " << std::endl;
		
		
		//std::cout << " " << std::endl;
		//std::cout << "consider floors : " << std::endl;
		// check if there are vertexes on a polygon of the space's geometry, for the floors of the cell.
		for (int i=0; i < intsecV.size(); i++)
		{
			//std::cout << "Check intsecV.size()" << intsecV.size() << " intsecV[i]: " << intsecV[i] << std::endl;
			bool onGeometry = false;
			int h = 0;
			while(h < checkCells.size())
			{
				int mCFCuboidsSize = checkCells.size();
				onGeometry = false;
				for (const auto& j : checkCells[h]->getCellFloors())
				{
					if (onGeometry) {break;}
					if (j->isInside(intsecV[i], mBuildingModel->tolerance()))
					{						
						onGeometry = true;
						
						std::vector <utilities::geometry::line_segment> linesToAccound;
						
						// find the conected lines to onPolyIntersection to accound for
						for(const auto l : intsecL)
						{
							if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								if (j->isInsideOrOn(l[0], mBuildingModel->tolerance()) && j->isInsideOrOn(l[1], mBuildingModel->tolerance()))
								{
									bool isCornerpoint = false;
									for(const auto m : j->getVertices())
									{
										if(m.isSameAs(l[0], mBuildingModel->tolerance()) || m.isSameAs(l[1], mBuildingModel->tolerance()))
										{
											isCornerpoint = true;
										}
									}
									if(!isCornerpoint)
									{
										linesToAccound.push_back(l);
									}
								}
							}
						}
						
						// generate the vector of vertices to split with
						std::vector<cf_vertex> splitVertexen; 
						splitVertexen.push_back(intsecV[i]);
						
						for(const auto k : linesToAccound)
						{
							if(k[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen.push_back(k[1]);
							}
							if(k[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen.push_back(k[0]);
							}
						}
						
						
						// remove ducplicates from splitVertexen
						auto splitVertexen_end = splitVertexen.end();
						for(auto it = splitVertexen.begin(); it != splitVertexen_end; ++it) //Note: deze iteratie methode neemt geen tolerantie mee, mischien hierdoor fouten?
						{
							splitVertexen_end = std::remove(it + 1, splitVertexen_end, *it);
						}
						splitVertexen.erase(splitVertexen_end, splitVertexen.end());
						
						// pars split vertexes to split function
						std::vector<cf_cuboid*> newCells;
						if(checkCells[h]->splitN(&splitVertexen, method, newCells))
						{
							// adjust checkCells
							checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[h]), checkCells.end());
							for(const auto t: newCells)
							{
								checkCells.push_back(t);
							}
							// store the inbetween stage
							buildingModel->storeCfStages(buildingModel);
							//std::cout << "The cells considered in iteration " << (buildingModel->getCfStages()).size() << " are: " << std::endl;
							//for(const auto i: checkCells)
							//{
								//std::cout << *i << std::endl;
							//}
							// find the newly generated intersection points and split the acording lines
							splitLinesTwo(intsecV, intsecL);
							// make sure no Cell is skipped
							h--;
							break;
						}
					}
				}
				h++;
			}
		}
		//std::cout << "Check lines: " << std::endl;
		// check if there are vertexes on the line_segments of the space's geometry Floors		
		int k = 0;
		while(k < checkCells.size())
		//for (int k = 0; k < mCFCuboids.size(); ++k)
		{
			bool onLFGeometry = false;
			std::vector<std::vector<cf_vertex>*> splitVLineF;
			std::vector<cf_vertex>* splitVertexen;
			for (int i=0; i < intsecV.size(); i++)
			{
				for (const auto& j : checkCells[k]->getCellFloors())
				{
					for(const auto h: j->cfLines())
					{
						if (h->isOnLine(intsecV[i], mBuildingModel->tolerance()))
						{
							//std::cout << "Vertex isOnLine " << intsecV[i] << std::endl;
							onLFGeometry = true;
							std::vector <utilities::geometry::line_segment> linesToAccound;
							
							// find the conected lines to onLineIntersection to accound for
							for(const auto l : intsecL)
							{
								if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									if (j->isInsideOrOn(l[0], mBuildingModel->tolerance()) && j->isInsideOrOn(l[1], mBuildingModel->tolerance()))
									{
										bool isCornerpoint = false;
										if(h->isOnLine(l[0], mBuildingModel->tolerance()) && h->isOnLine(l[1], mBuildingModel->tolerance()))
										{
											isCornerpoint = true;
										}
										for(const auto m : j->getVertices())
										{
											if(m.isSameAs(l[0], mBuildingModel->tolerance()) || m.isSameAs(l[1], mBuildingModel->tolerance()))
											{
												isCornerpoint = true;
											}
										}
										if(!isCornerpoint)
										{
											linesToAccound.push_back(l);
										}
									}
								}
							}
							
							// generate the vector of vertices to split with
							//std::unique_ptr<std::vector<cf_vertex>> splitVertexen (new std::vector<cf_vertex>);
							splitVertexen = new std::vector<cf_vertex>; // Datalek!!!
							splitVertexen->push_back(intsecV[i]);
							for(const auto l : linesToAccound)
							{
								if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									splitVertexen->push_back(l[1]);
								}
								if(l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									splitVertexen->push_back(l[0]);
								}
							}
							
							// Add splitVertexen for this intersection case for step 3 split to splitVLineF
							splitVLineF.push_back(splitVertexen);
							linesToAccound.clear();
						}
					}
				}
			}
			if(onLFGeometry)
			{
				//std::cout << "The size of splitVLineF.size(): " << splitVLineF.size() << std::endl;

				std::sort (splitVLineF.begin(), splitVLineF.end(), this->sortSplitV);
				int mCFCuboidsSize = checkCells.size();
				for(int j=0; j < splitVLineF.size(); j++)
				{
					if((splitVLineF[j])->size() > 1)
					{
						// Send splitVlineF to split function
						std::vector<cf_cuboid*> newCells;
						if(checkCells[k]->splitN(splitVLineF[j], method, newCells))
						{
							// adjust checkCells
							checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[k]), checkCells.end());
							for(const auto t: newCells)
							{
								checkCells.push_back(t);
							}
							// store the inbetween stage
							buildingModel->storeCfStages(buildingModel);
							//std::cout << "The cells considered in iteration " << (buildingModel->getCfStages()).size() << " are: " << std::endl;
							//for(const auto i: checkCells)
							//{
								//std::cout << *i << std::endl;
							//}
							// make sure no Cell is skipped
							k--;
							// find the newly generated intersection points and split the acording lines
							splitLinesTwo(intsecV, intsecL);							
							break;
						}
					}
				}
				for (auto& i : splitVLineF) delete i; //Datalek fixen
			}
			k++;
		}
		
		//std::cout << " " << std::endl;
		//std::cout << "consider walls: " << std::endl;
		
		
		// check if there are vertexes on a polygon of the space's geometry, for the walls of the cell.
		for (int i=0; i < intsecV.size(); i++)
		{
			bool onGeometry = false;
			int h = 0;
			while(h < checkCells.size())
			//for (unsigned int h = 0; h < mCFCuboids.size(); ++h)
			{				
				int mCFCuboidsSize = checkCells.size();
				onGeometry = false;
				for (const auto& j : checkCells[h]->getCellWalls())
				{
					if (onGeometry) {break;}
					if (j->isInside(intsecV[i], mBuildingModel->tolerance()))
					{
						//std::cout << "intersection is on polygon: " << intsecV[i] << std::endl;
						onGeometry = true;
						
						std::vector <utilities::geometry::line_segment> linesToAccound;
						
						// find the conected lines to onPolyIntersection to accound for
						for(const auto l : intsecL)
						{
							if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								linesToAccound.push_back(l);
							}
						}
						
						// generate the vector of vertices to split with
						std::vector<cf_vertex> splitVertexen; 
						splitVertexen.push_back(intsecV[i]);
						
						for(const auto g : linesToAccound)
						{
							if(g[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen.push_back(g[1]);
							}
							if(g[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen.push_back(g[0]);
							}
						}
						
						
						// remove ducplicates from splitVertexen
						auto splitVertexen_end = splitVertexen.end();
						for(auto it = splitVertexen.begin(); it != splitVertexen_end; ++it) //Note: deze iteratie methode neemt geen tolerantie mee, mischien hierdoor fouten?
						{
							splitVertexen_end = std::remove(it + 1, splitVertexen_end, *it);
						}
						splitVertexen.erase(splitVertexen_end, splitVertexen.end());
						
						// pars split vertexes to split function
						std::vector<cf_cuboid*> newCells;
						if(checkCells[h]->splitN(&splitVertexen, method, newCells))
						{
							// adjust checkCells
							checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[h]), checkCells.end());
							for(const auto t: newCells)
							{
								checkCells.push_back(t);
							}
							// store the inbetween stage
							buildingModel->storeCfStages(buildingModel);
							//std::cout << "The cells considered in iteration " << (buildingModel->getCfStages()).size() << " are: " << std::endl;
							//for(const auto i: checkCells)
							//{
								//std::cout << *i << std::endl;
							//}
							// make sure no Cell is skipped
							h--;
							// find the newly generated intersection points and split the acording lines
							splitLinesTwo(intsecV, intsecL);
							break;
						}
					}
				}
				h++;
			}
		}
		
		

		// check if there are vertexes on the line_segments of the space's geometry walls
		k = 0;
		while(k < checkCells.size())
		{			
			bool onLFGeometry = false;
			std::vector<std::vector<cf_vertex>*> splitVLineF;
			std::vector<cf_vertex>* splitVertexen;
			for (int i=0; i < intsecV.size(); i++)
			{
				for (const auto& j : checkCells[k]->getCellWalls())
				{
					for(const auto h: j->cfLines())
					{
						if (h->isOnLine(intsecV[i], mBuildingModel->tolerance()))
						{						
							onLFGeometry = true;
							std::vector <utilities::geometry::line_segment> linesToAccound;
							
							// find the conected lines to onPolyIntersection to accound for
							for(const auto l : intsecL)
							{
								if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									if (j->isInsideOrOn(l[0], mBuildingModel->tolerance()) && j->isInsideOrOn(l[1], mBuildingModel->tolerance()))
									{
										bool isCornerpoint = false;
										if(h->isOnLine(l[0], mBuildingModel->tolerance()) && h->isOnLine(l[1], mBuildingModel->tolerance()))
										{
											isCornerpoint = true;
										}
										for(const auto m : j->getVertices())
										{
											if(m.isSameAs(l[0], mBuildingModel->tolerance()) || m.isSameAs(l[1], mBuildingModel->tolerance()))
											{
												isCornerpoint = true;
											}
										}
										if(!isCornerpoint)
										{
											linesToAccound.push_back(l);
										}
									}
								}
							}
							
							// generate the vector of vertices to split with
							//std::unique_ptr<std::vector<cf_vertex>> splitVertexen (new std::vector<cf_vertex>);
							splitVertexen = new std::vector<cf_vertex>; // Datalek!!!
							splitVertexen->push_back(intsecV[i]);
							for(const auto l : linesToAccound)
							{
								if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									splitVertexen->push_back(l[1]);
								}
								if(l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									splitVertexen->push_back(l[0]);
								}
							}
							
							// Add splitVertexen for this intersection case for step 3 split to splitVLineF
							splitVLineF.push_back(splitVertexen);
							linesToAccound.clear();
						}
					}
				}
			}
			
			if(onLFGeometry)
			{
				std::sort (splitVLineF.begin(), splitVLineF.end(), this->sortSplitV);
				//std::cout << "Before: mCFCuboids[k] is: " << *mCFCuboids[k] << std::endl;
				int mCFCuboidsSize = checkCells.size();
				for(int j=0; j < splitVLineF.size(); j++)
				{
					// Send splitVlineF to split function
					std::vector<cf_cuboid*> newCells;
					if(checkCells[k]->splitN(splitVLineF[j], method, newCells))
					{
						// adjust checkCells
						checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[k]), checkCells.end());
						for(const auto t: newCells)
						{
							checkCells.push_back(t);
						}
						// store the inbetween stage
						buildingModel->storeCfStages(buildingModel);
						//std::cout << "The cells considered in iteration " << (buildingModel->getCfStages()).size() << " are: " << std::endl;
						//for(const auto i: checkCells)
						//{
							//std::cout << *i << std::endl;
						//}
						// make sure no Cell is skipped
						k--;
						// find the newly generated intersection points and split the acording lines
						splitLinesTwo(intsecV, intsecL);
						break;
					}
				}
				for (auto& i : splitVLineF) delete i; //Datalek fixen
			}
			k++;
		}		
	} // checkVertexN
	
	bool cf_space::checkVertexN2D(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method) // quad-hexahedron method step 2
	{
		std::vector<cf_cuboid*> c;
		for(const auto h: mCFCuboids)
		{
			c.push_back(h);
		}
		if(checkVertexN2D(intsecV, intsecL, method, c))
		{
			return true;
		}
		return false;
	}
	
	bool cf_space::checkVertexN2D(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method, std::vector<cf_cuboid*> c) // quad-hexahedron method step 2
	{ // 
		
		// std::cout << "The intersection lines are: " << intsecL.size() << std::endl;
		// split intsecL on the hand of the found intsecV
		cf_space::splitLines(intsecV, intsecL);
		
		// Define cells to check
		std::vector<cf_cuboid*> checkCells = c;
		//checkCells.push_back(cPtr);
		
		//std::cout << " " << std::endl;
		//std::cout << "Check Space: " << getSpaceID() << " consisting of: " << *this << std::endl;
		//std::cout << "The intersection vertices are: " << intsecV.size() << std::endl;
		//for(const auto i : intsecV)
		//{
			//std::cout << i ;
		//}
		//std::cout << " " << std::endl;
		
		
		std::cout << " " << std::endl;
		std::cout << "consider ground floors: " << std::endl;
		
		bool split = false;
		// check if there are vertexes on a polygon of the space's geometry, for the floors of the cell.
		for (int i=0; i < intsecV.size(); i++)
		{
			//std::cout << "Check intsecV.size()" << intsecV.size() << " intsecV[i]: " << intsecV[i] << std::endl;
			bool onGeometry = false;
			int h = 0;
			while(h < checkCells.size())
			{
				int mCFCuboidsSize = checkCells.size();
				onGeometry = false;
				utilities::geometry::vertex shift = {0,0,checkCells[h]->getGroundDiference()};
				
				//if (onGeometry) {break;}
				if ((checkCells[h]->getRectangleAtGround()).isInside(intsecV[i], mBuildingModel->tolerance()))
				{
					onGeometry = true;
					
					std::vector <utilities::geometry::line_segment> linesToAccound;
					// find the conected lines to onPolyIntersection to accound for
					for(const auto l : intsecL)
					{
						if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
						{
							if ((checkCells[h]->getRectangleAtGround()).isInsideOrOn(l[0], mBuildingModel->tolerance()) && (checkCells[h]->getRectangleAtGround()).isInsideOrOn(l[1], mBuildingModel->tolerance()))
							{
								bool isCornerpoint = false;
								for(const auto m : (checkCells[h]->getRectangleAtGround()).getVertices())
								{
									if(m.isSameAs(l[0], mBuildingModel->tolerance()) || m.isSameAs(l[1], mBuildingModel->tolerance()))
									{
										isCornerpoint = true;
									}
								}
								if(!isCornerpoint)
								{
									linesToAccound.push_back(l);
								}
							}
						}
					}
					
					// generate the vector of vertices to split with
					std::vector<cf_vertex> splitVertexen; 
					
					splitVertexen.push_back(intsecV[i]+shift);
					
					for(const auto k : linesToAccound)
					{
						if(k[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
						{
							splitVertexen.push_back(k[1]+shift);
						}
						if(k[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
						{
							splitVertexen.push_back(k[0]+shift);
						}
					}
					
					
					// remove ducplicates from splitVertexen
					auto splitVertexen_end = splitVertexen.end();
					for(auto it = splitVertexen.begin(); it != splitVertexen_end; ++it) //Note: deze iteratie methode neemt geen tolerantie mee, mischien hierdoor fouten?
					{
						splitVertexen_end = std::remove(it + 1, splitVertexen_end, *it);
					}
					splitVertexen.erase(splitVertexen_end, splitVertexen.end());
					
					std::cout << "The split vertices for the line split are: " << std::endl;
					for(int p = 0; p<splitVertexen.size(); p++)
					{
						std::cout << splitVertexen[p];
					}
					std::cout << " " << std::endl;
					
					// pars split vertexes to split function
					std::vector<cf_cuboid*> newCells;
					if(checkCells[h]->splitN(&splitVertexen, method, newCells))
					{
						// adjust checkCells
						checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[h]), checkCells.end());
						for(const auto t: newCells)
						{
							checkCells.push_back(t);
						}
						// find the newly generated intersection points and split the acording lines
						splitLinesTwo2D(intsecV, intsecL);
						// make sure no Cell is skipped
						h--;
						split = true;
						break;
					}
				}
				h++;
			}
		}
		
		// check if there are vertexes on the line_segments of the space's geometry Floors		
		int k = 0;
		while(k < checkCells.size())
		//for (int k = 0; k < mCFCuboids.size(); ++k)
		{
			bool onLFGeometry = false;
			std::vector<std::vector<cf_vertex>*> splitVLineF;
			std::vector<cf_vertex>* splitVertexen;
			for (int i=0; i < intsecV.size(); i++)
			{
				for(const auto h: (checkCells[k]->getRectangleAtGround()).getLines())
				{
					if (h.isOnLine(intsecV[i], mBuildingModel->tolerance()))
					{
						//std::cout << "Vertex isOnLine " << intsecV[i] << std::endl;
						onLFGeometry = true;
						std::vector <utilities::geometry::line_segment> linesToAccound;
						
						// find the conected lines to onLineIntersection to accound for
						for(const auto l : intsecL)
						{
							if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								if ((checkCells[k]->getRectangleAtGround()).isInsideOrOn(l[0], mBuildingModel->tolerance()) && (checkCells[k]->getRectangleAtGround()).isInsideOrOn(l[1], mBuildingModel->tolerance()))
								{
									bool isCornerpoint = false;
									if(h.isOnLine(l[0], mBuildingModel->tolerance()) && h.isOnLine(l[1], mBuildingModel->tolerance()))
									{
										isCornerpoint = true;
									}
									for(const auto m : (checkCells[k]->getRectangleAtGround()).getVertices())
									{
										if(m.isSameAs(l[0], mBuildingModel->tolerance()) || m.isSameAs(l[1], mBuildingModel->tolerance()))
										{
											isCornerpoint = true;
										}
									}
									if(!isCornerpoint)
									{
										linesToAccound.push_back(l);
									}
								}
							}
						}
						
						// generate the vector of vertices to split with
						//std::unique_ptr<std::vector<cf_vertex>> splitVertexen (new std::vector<cf_vertex>);
						splitVertexen = new std::vector<cf_vertex>; // Datalek!!!
						utilities::geometry::vertex shift = {0,0,checkCells[k]->getGroundDiference()};
						splitVertexen->push_back(intsecV[i]+shift);
						for(const auto l : linesToAccound)
						{
							if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen->push_back(l[1]+shift);
							}
							if(l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen->push_back(l[0]+shift);
							}
						}
						
						// Add splitVertexen for this intersection case for step 3 split to splitVLineF
						splitVLineF.push_back(splitVertexen);
						linesToAccound.clear();
						
						std::cout << "The split vertices for the line split are: " << std::endl;
						for(int p = 0; p<splitVertexen->size(); p++)
						{
							std::cout << (*splitVertexen)[p];
						}
						std::cout << " " << std::endl;
					}
				}
			}
			if(onLFGeometry)
			{
				std::cout << "The size of splitVLineF.size(): " << splitVLineF.size() << std::endl;

				std::sort (splitVLineF.begin(), splitVLineF.end(), this->sortSplitV);
				int mCFCuboidsSize = checkCells.size();
				for(int j=0; j < splitVLineF.size(); j++)
				{
					if((splitVLineF[j])->size() > 1)
					{
						// Send splitVlineF to split function
						std::vector<cf_cuboid*> newCells;
						if(checkCells[k]->splitN(splitVLineF[j], method, newCells))
						{
							checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[k]), checkCells.end());
							for(const auto t: newCells)
							{
								checkCells.push_back(t);
							}
							k--;
							split = true;
							splitLinesTwo2D(intsecV, intsecL);							
							break;
						}
					}
				}
				for (auto& i : splitVLineF) delete i; //Datalek fixen
			}
			k++;
		}
		
		
		
		// check if there are vertexes on a polygon of the space's geometry, for the walls of the cell.
		for (int i=0; i < intsecV.size(); i++)
		{
			bool onGeometry = false;
			int h = 0;
			while(h < checkCells.size())
			//for (unsigned int h = 0; h < mCFCuboids.size(); ++h)
			{				
				int mCFCuboidsSize = checkCells.size();
				onGeometry = false;
				for (const auto& j : checkCells[h]->getCellWalls())
				{
					if (onGeometry) {break;}
					if (j->isInside(intsecV[i], mBuildingModel->tolerance()))
					{
						//std::cout << "intersection is on polygon: " << intsecV[i] << std::endl;
						onGeometry = true;
						
						std::vector <utilities::geometry::line_segment> linesToAccound;
						
						// find the conected lines to onPolyIntersection to accound for
						for(const auto l : intsecL)
						{
							if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								linesToAccound.push_back(l);
							}
						}
						
						// generate the vector of vertices to split with
						std::vector<cf_vertex> splitVertexen; 
						splitVertexen.push_back(intsecV[i]);
						
						for(const auto g : linesToAccound)
						{
							if(g[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen.push_back(g[1]);
							}
							if(g[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
							{
								splitVertexen.push_back(g[0]);
							}
						}
						
						
						// remove ducplicates from splitVertexen
						auto splitVertexen_end = splitVertexen.end();
						for(auto it = splitVertexen.begin(); it != splitVertexen_end; ++it) //Note: deze iteratie methode neemt geen tolerantie mee, mischien hierdoor fouten?
						{
							splitVertexen_end = std::remove(it + 1, splitVertexen_end, *it);
						}
						splitVertexen.erase(splitVertexen_end, splitVertexen.end());
						
						// pars split vertexes to split function
						std::vector<cf_cuboid*> newCells;
						if(checkCells[h]->splitN(&splitVertexen, method, newCells))
						{
							checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[h]), checkCells.end());
							for(const auto t: newCells)
							{
								checkCells.push_back(t);
							}
							h--;
							splitLinesTwo(intsecV, intsecL);
							break;
						}
					}
				}
				h++;
			}
		}
		
		

		// check if there are vertexes on the line_segments of the space's geometry walls
		k = 0;
		while(k < checkCells.size())
		{			
			bool onLFGeometry = false;
			std::vector<std::vector<cf_vertex>*> splitVLineF;
			std::vector<cf_vertex>* splitVertexen;
			for (int i=0; i < intsecV.size(); i++)
			{
				for (const auto& j : checkCells[k]->getCellWalls())
				{
					for(const auto h: j->cfLines())
					{
						if (h->isOnLine(intsecV[i], mBuildingModel->tolerance()))
						{						
							onLFGeometry = true;
							std::vector <utilities::geometry::line_segment> linesToAccound;
							
							// find the conected lines to onPolyIntersection to accound for
							for(const auto l : intsecL)
							{
								if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()) || l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									if (j->isInsideOrOn(l[0], mBuildingModel->tolerance()) && j->isInsideOrOn(l[1], mBuildingModel->tolerance()))
									{
										bool isCornerpoint = false;
										if(h->isOnLine(l[0], mBuildingModel->tolerance()) && h->isOnLine(l[1], mBuildingModel->tolerance()))
										{
											isCornerpoint = true;
										}
										for(const auto m : j->getVertices())
										{
											if(m.isSameAs(l[0], mBuildingModel->tolerance()) || m.isSameAs(l[1], mBuildingModel->tolerance()))
											{
												isCornerpoint = true;
											}
										}
										if(!isCornerpoint)
										{
											linesToAccound.push_back(l);
										}
									}
								}
							}
							
							// generate the vector of vertices to split with
							//std::unique_ptr<std::vector<cf_vertex>> splitVertexen (new std::vector<cf_vertex>);
							splitVertexen = new std::vector<cf_vertex>; // Datalek!!!
							splitVertexen->push_back(intsecV[i]);
							for(const auto l : linesToAccound)
							{
								if(l[0].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									splitVertexen->push_back(l[1]);
								}
								if(l[1].isSameAs(intsecV[i], mBuildingModel->tolerance()))
								{
									splitVertexen->push_back(l[0]);
								}
							}
							
							// Add splitVertexen for this intersection case for step 3 split to splitVLineF
							splitVLineF.push_back(splitVertexen);
							linesToAccound.clear();
						}
					}
				}
			}
			
			if(onLFGeometry)
			{
				std::sort (splitVLineF.begin(), splitVLineF.end(), this->sortSplitV);
				//std::cout << "Before: mCFCuboids[k] is: " << *mCFCuboids[k] << std::endl;
				int mCFCuboidsSize = checkCells.size();
				for(int j=0; j < splitVLineF.size(); j++)
				{
					// Send splitVlineF to split function
					std::vector<cf_cuboid*> newCells;
					if(checkCells[k]->splitN(splitVLineF[j], method, newCells))
					{
						checkCells.erase(std::remove(checkCells.begin(),checkCells.end(),checkCells[k]), checkCells.end());
						for(const auto t: newCells)
						{
							checkCells.push_back(t);
						}
						k--;
						splitLinesTwo(intsecV, intsecL);
						break;
					}
				}
				for (auto& i : splitVLineF) delete i; //Datalek fixen
			}
			k++;
		}		
		return split;
	} // checkVertexN2D
	
	void cf_space::checkVertexT(cf_vertex* pPtr)
	{ // 
		// check if the vertex is in or on the space's geometry
		bool onGeometry = false;

		if ((this->isInside(*pPtr, mBuildingModel->tolerance()))) onGeometry = true;
		
		for (const auto& i : mPolygons)
		{
			if (onGeometry) break;
			if (i->isInside(*pPtr, mBuildingModel->tolerance()))
			{
				onGeometry = true;
				break;
			}
		}

		for (const auto& i : mLineSegments)
		{
			if (onGeometry) break;
			if (i.isOnLine(*pPtr, mBuildingModel->tolerance()))
			{
				onGeometry = true;
				break;
			}
		}

		
		if (!onGeometry) return;  // if it isn't return the function immediately

		// check each cuboid if it can be split by the vertex
		
		for (unsigned int i = 0; i < mCFTetrahedrons.size(); ++i)
		{
			int preSplit = mCFTetrahedrons.size();
			mCFTetrahedrons[i]->split(pPtr);
			if (preSplit < mCFTetrahedrons.size()) // check if the split function changed the tetrahedrons
			{
				// if it does, then i-- prevents the skipping of an tetrahedron entity within the mCFTetrahedrons container
				// duo to the removing and adding of entities in the split function
				i--;
			}
		}
		
		/*
		//oude referentie methode waarbij als er entities aangepast worden in de mCFTetrahedrons in stap 3 een tetrahedron check word overgeslagen
		for (unsigned int i = 0; i < mCFTetrahedrons.size(); ++i)
		{
			mCFTetrahedrons[i]->split(pPtr);
		}
		*/
		
	} // checkVertexT
	
	
	void cf_space::splitLines(const std::vector<cf_vertex>& intsecV, std::vector<utilities::geometry::line_segment>& intsecL)
	{			
		// split lines in acordance to intersection points
		for(int k=0; k<intsecL.size(); k++)
		{
			for(const auto l :intsecV)
			{
				if(intsecL[k].isOnLine(l, mBuildingModel->tolerance())) // split lines: Als hier tolerantie word meegenomen dan kan er 
				{//mogelijk eerder een lijn gesplits worden en ontstaat er een kink in de orginele lijn.
					utilities::geometry::line_segment one = {{(intsecL[k])[0]},{l}};
					utilities::geometry::line_segment two = {{(intsecL[k])[1]},{l}};
					
					intsecL.push_back(one);
					intsecL.push_back(two);
					
					intsecL.erase(intsecL.begin() + k);
					k--; // makes sure loop 1 does not skip a line segment when one is removed
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
				if (m == intsecL[k]){continue;} //No tolerance needed. Just opperator to make sure the same entities are not checked with eachother.
				if (m.isSameAs(intsecL[k], mBuildingModel->tolerance())){intsecL.erase(intsecL.begin() + k);}
			}
		}		
	}//cf_space::splitLines
	
	void cf_space::splitLinesTwo(std::vector<cf_vertex>& intsecV, std::vector<utilities::geometry::line_segment>& intsecL)
	{		
		// Search for new intsecV points as a result of a previous split, these should be taken into account in the next iteration
		//std::cout << "splitLinesTwo: " << std::endl;
		cf_vertex pIntersection;	
		for(const auto o: mCFCuboids)
		{			
			// check for line - rectangle intersections, and add the found vertex to the intsecV
			for (const auto k: o->getPolygons())
			{
				for (const auto l: intsecL)
				{
					if (k->intersectsWith(l, pIntersection, mBuildingModel->tolerance()))
					{						
						bool doubleInsert = false;
						for (const auto& m : intsecV)
						{// check if pIntersection already exist in intsecV
							if (m.isSameAs(pIntersection, mBuildingModel->tolerance()))
							{
								doubleInsert = true;
								//std::cout << "already existing intsecV " << std::endl;
							}
						}
						
						if (doubleInsert == false)
						{
							intsecV.push_back(pIntersection);
							//std::cout << "additional guide vertex " << pIntersection <<  std::endl;
						}
					}
				}
			}
			// check for line line intersections, and add the found vertex to pIntersection intsecV
			for (const auto k: o->getLines())
			{
				for (const auto l: intsecL)
				{
					if (k.intersectsWith(l, pIntersection, mBuildingModel->tolerance()))
					{						
						bool doubleInsert = false;
						for (const auto& m : intsecV)
						{// check if pIntersection already exist in intsecV
							if (m.isSameAs(pIntersection, mBuildingModel->tolerance()))
							{
								doubleInsert = true;
							}
						}
						
						if (doubleInsert == false)
						{
							intsecV.push_back(pIntersection);
							//std::cout << "additional guide vertex "<< pIntersection << std::endl;
						}
					}
				}
			}
		}
		
		
		// split lines in acordance to intersection points
		for(int k; k<intsecL.size(); k++)
		{
			for(const auto l :intsecV)
			{
				if(intsecL[k].isOnLine(l, mBuildingModel->tolerance())) // split lines: Als hier tolerantie word meegenomen dan kan er 
				{//mogelijk eerder een lijn gesplits worden en ontstaat er een kink in de orginele lijn.
					utilities::geometry::line_segment one = {{(intsecL[k])[0]},{l}};
					utilities::geometry::line_segment two = {{(intsecL[k])[1]},{l}};
					
					intsecL.push_back(one);
					intsecL.push_back(two);
					
					intsecL.erase(intsecL.begin() + k);
					k--; // prevent the skipping of an intsecL when the previous is erased
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
				if (m == intsecL[k]){continue;} //No tolerance needed. Just opperator to make sure the same entities are not checked with eachother.
				if (m.isSameAs(intsecL[k], mBuildingModel->tolerance())){intsecL.erase(intsecL.begin() + k);}
			}
		}		
	}//cf_space::splitLinesTwo
	
	
	void cf_space::splitLinesTwo2D(std::vector<cf_vertex>& intsecV, std::vector<utilities::geometry::line_segment>& intsecL)
	{		
		// Search for new intsecV points as a result of a previous split, these should be taken into account in the next iteration
		//std::cout << "splitLinesTwo: " << std::endl;
		cf_vertex pIntersection;	
		for(const auto o: mCFCuboids)
		{			
			// check for line - rectangle intersections, and add the found vertex to the intsecV
			for (const auto l: intsecL)
			{
				if ((o->getRectangleAtGround()).intersectsWith(l, pIntersection, mBuildingModel->tolerance()))
				{						
					bool doubleInsert = false;
					for (const auto& m : intsecV)
					{// check if pIntersection already exist in intsecV
						if (m.isSameAs(pIntersection, mBuildingModel->tolerance()))
						{
							doubleInsert = true;
							//std::cout << "already existing intsecV " << std::endl;
						}
					}
					
					if (doubleInsert == false)
					{
						intsecV.push_back(pIntersection);
						//std::cout << "additional guide vertex " << pIntersection <<  std::endl;
					}
				}
			}
			// check for line line intersections, and add the found vertex to pIntersection intsecV
			for (const auto k: (o->getRectangleAtGround()).getLines())
			{
				for (const auto l: intsecL)
				{
					if (k.intersectsWith(l, pIntersection, mBuildingModel->tolerance()))
					{						
						bool doubleInsert = false;
						for (const auto& m : intsecV)
						{// check if pIntersection already exist in intsecV
							if (m.isSameAs(pIntersection, mBuildingModel->tolerance()))
							{
								doubleInsert = true;
							}
						}
						
						if (doubleInsert == false)
						{
							intsecV.push_back(pIntersection);
							//std::cout << "additional guide vertex "<< pIntersection << std::endl;
						}
					}
				}
			}
		}
		
		
		// split lines in acordance to intersection points
		for(int k; k<intsecL.size(); k++)
		{
			for(const auto l :intsecV)
			{
				if(intsecL[k].isOnLine(l, mBuildingModel->tolerance())) // split lines: Als hier tolerantie word meegenomen dan kan er 
				{//mogelijk eerder een lijn gesplits worden en ontstaat er een kink in de orginele lijn.
					utilities::geometry::line_segment one = {{(intsecL[k])[0]},{l}};
					utilities::geometry::line_segment two = {{(intsecL[k])[1]},{l}};
					
					intsecL.push_back(one);
					intsecL.push_back(two);
					
					intsecL.erase(intsecL.begin() + k);
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
				if (m == intsecL[k]){continue;} //No tolerance needed. Just opperator to make sure the same entities are not checked with eachother.
				if (m.isSameAs(intsecL[k], mBuildingModel->tolerance())){intsecL.erase(intsecL.begin() + k);}
			}
		}		
	}//cf_space::splitLinesTwo
	
	
	
	
	
	
} // conformal
} // spatial_design
} // bso

#endif // CF_SPACE_CPP