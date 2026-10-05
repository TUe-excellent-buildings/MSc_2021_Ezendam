#ifndef CF_CUBOID_CPP
#define CF_CUBOID_CPP

namespace bso { namespace spatial_design { namespace conformal {
	
	cf_cuboid::cf_cuboid(const utilities::geometry::quad_hexahedron& rhs, cf_geometry_model* geometryModel)
	: utilities::geometry::quad_hexahedron(rhs, geometryModel->tolerance())
	{
		mGeometryModel = geometryModel;
		for (const auto& i : mVertices)
		{
			mCFVertices.push_back(mGeometryModel->addVertex(i));
			mCFVertices.back()->addCuboid(this);
		}
		for (const auto& i : mLineSegments)
		{
			mCFLines.push_back(mGeometryModel->addLine(i));
			mCFLines.back()->addCuboid(this);
		}
		utilities::geometry::vector normalFloor = {0,0,1};
		for (const auto& i : mPolygons)
		{
			//std::cout << "line_1" << std::endl; // These couts are inserted to resolve a unknown cause of a segmentation fault when running design 370 in directory vh_designs
			mGeometryModel;
			//std::cout << "line_1a" << std::endl;
			*i;
			//std::cout << "line_1b" << std::endl;
			mCFRectangles.push_back(mGeometryModel->addRectangle(*i));
			//std::cout << "line_2" << std::endl;
			mCFRectangles.back()->addCuboid(this);
			//std::cout << "line_3" << std::endl;
			if(abs((mCFRectangles.back()->getNormal()).getHeading().second) > 45){floors.push_back(mCFRectangles.back());} // abs((rectangle->getNormal().normalized()).getHeading().second) > 45 //used in the grammar to distinquese floors and walls
			else {walls.push_back(mCFRectangles.back());}
			
		}
		//std::cout << "Mid-s performing Cuboid" << std::endl;
		
		if(floors.size() != 2)
		{// not true for cells with slant floors/roofs.
			std::stringstream errorMessage;
			errorMessage << "\nError, expected to find 2 floors per cell during the \n"
									 << "generation of a cuboid\n"
									 << "however, this is not found\n"
									 << "in case a building spatial design is inserted with non horizontal floors is this error also initiated\n"
									 << "It is then the responsibility of the toolbox user to verify it it is indead the intension. \n"
									 << "If so, then it this error message can be made inactive. \n"
									 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		
		//std::cout << "Mid performing Cuboid" << std::endl;
		if(getCellFloors().size() == 2)
		{
			if((((getCellFloors()[0])->getVertices())[0]).z() < (((getCellFloors()[1])->getVertices())[0]).z())
			{
				groundDiference = (((getCellFloors()[0])->getVertices())[0]).z();
				
				std::vector<utilities::geometry::vertex> vertices;
				utilities::geometry::vertex shift = {0,0,groundDiference};
				for(auto k: (getCellFloors()[0])->getVertices())
				{
					vertices.push_back(k-shift);
				}
				rectangleAtGround = utilities::geometry::quadrilateral(vertices, mGeometryModel->tolerance());

				
				std::vector<utilities::geometry::vertex> cornerPoints;
				for (const auto& i : mVertices)
				{
					cornerPoints.push_back(i-shift);
				}
				
				cuboidAtGround = (utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance()));
			}
			else
			{
				groundDiference = (((getCellFloors()[1])->getVertices())[0]).z();
				std::vector<utilities::geometry::vertex> vertices;
				utilities::geometry::vertex shift = {0,0,groundDiference};
				for(auto k: (getCellFloors()[1])->getVertices())
				{
					vertices.push_back(k-shift);
				}

				rectangleAtGround = utilities::geometry::quadrilateral(vertices, mGeometryModel->tolerance());
				std::vector<utilities::geometry::vertex> cornerPoints;
				for (const auto& i : mVertices)
				{
					cornerPoints.push_back(i-shift);
				}
				
				cuboidAtGround = (utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance()));
			}
		}
		//std::cout << "End performing Cuboid" << std::endl;
		/*
		else
		{
			groundDiference = (((getCellFloors()[0])->getVertices())[0]).z();
			
			utilities::geometry::vertex shift = {0,0,groundDiference};
			std::vector<utilities::geometry::vertex> vertices;
			//std::cout << "vertices of rectangleAtGround: " << std::endl;
			for(auto k: (getCellFloors()[0])->getVertices())
			{
				vertices.push_back(k-shift);
				//std::cout << vertices.back();
			}
			std::cout << "  " << std::endl;
			rectangleAtGround = utilities::geometry::quadrilateral(vertices, mGeometryModel->tolerance());
				
			std::cout << " " << std::endl;
			std::cout << "Waning!!!" << std::endl;
			std::cout << " " << std::endl;
		}
		
		
		std::vector<utilities::geometry::vertex> cornerPoints;
		utilities::geometry::vertex shift = {0,0,groundDiference};
		//std::cout << "vertices of cuboidAtGround: " << std::endl;
		for (const auto& i : mVertices)
		{
			cornerPoints.push_back(i-shift);
		}
		
		cuboidAtGround = (utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance()));
		*/		
	} // ctor
	
	cf_cuboid::~cf_cuboid()
	{
		for (const auto& i : mCFVertices) i->removeCuboid(this);
		for (const auto& i : mCFLines) i->removeCuboid(this);
		for (const auto& i : mCFRectangles) i->removeCuboid(this);
	} // dtor
	
	
	
	
	bool cf_cuboid::splitN(std::vector<cf_vertex>* intsecV, char method, std::vector<cf_cuboid*>& newCells) // step three quad-hexahedron split method
	{ // 
		//Declerations
		std::vector <bool> isInside;
		std::vector <bool> onPolygon;
		std::vector <bool> onLine;
		
		std::vector <utilities::geometry::vertex> isInsideV;
		std::vector <utilities::geometry::vertex> onPolygonV;
		std::vector <utilities::geometry::vertex> onLineV;
		
		std::vector <utilities::geometry::polygon*> intersecP;
		std::vector <utilities::geometry::line_segment> intersecL;
		
		int counter = 0;

		//Collect information for diferentiation between split methods for later use
		
		for (const auto i: *intsecV)
		{
			if (this->isInside(i, mGeometryModel->tolerance()))
			{
				isInsideV.push_back(i);
			}
			else
			{
				isInside.push_back(false);
			}
		}	
		
		bool firstVertexOnPoly = false; // only if the first vertex is onPoly then the poly-split may continue
		counter = 0;
		for (const auto i: *intsecV)
		{			
			unsigned int index = 0;
			for (const auto& j : mPolygons)
			{	
				utilities::geometry::vertex temp = j->getPointClosestTo(i);
				
				if (j->isInside(i, mGeometryModel->tolerance()) && i.isSameAs(temp, mGeometryModel->tolerance())) // temp is needed when the polygon consist of a (0,0,0) vertex, otherwise its finds a intersection when there is none
				{
					onPolygonV.push_back(i);
					intersecP.push_back(j);
					onPolygon.push_back(true);
					if (counter == 0)
					{
						firstVertexOnPoly = true;
					}
					index++;
				}
				else
				{
					onPolygon.push_back(false);
				}	
			}
			counter++;
		}	
		
		bool firstVertexOnLine = false; // only if the first vertex is onLine then the line-split may continue
		counter = 0;
		for (const auto i: *intsecV)
		{
			for (const auto& j : mLineSegments)
			{
				if (j.isOnLine(i,mGeometryModel->tolerance()))
				{
					onLineV.push_back(i);
					intersecL.push_back(j);
					onLine.push_back(true);
					if (counter == 0)
					{
						firstVertexOnLine = true;
					}
				}
				else
				{
					onLine.push_back(false);
				}
				
			}
			counter++;
		}

		//std::cout << "All the vertices in onPolygonV are: " << std::endl;
		//for(const auto i: onPolygonV)
		//{
			//std::cout << i ;
		//}
		//std::cout << " " << std::endl;
		//std::cout << "All the vertices in onLineV are: " << std::endl;
		//for(const auto i: onLineV)
		//{
			//std::cout << i ;
		//}
		//std::cout << " " << std::endl;
		
		//split methods execution
		
		// I = perpendicular to intersection side
		// O = perpendicular to oposide side
		// M = middle ratio method
		
		std::vector<cf_vertex*> newVertices;
		std::vector<cf_cuboid*> newCuboids;
		bool split = false;
		
		// Not written yet
		// split method for intersection points inside space which defide space into 8 cells
		if (isInsideV.size() >= 1)
		{ 
			//split = true;
			if (isInsideV.size() >= 2)
			{

			}
			else 
			{
				
			}
		} // split inside space			
		

		// split method for intersection point on polygon, split space into 4 "cuboids"
		if(!split)
		{ 
			if (onPolygonV.size() >= 1 && firstVertexOnPoly == true)
			{
				split = true;
				//std::cout << "Split acording to poly intersection: " << onPolygonV[0] << " with guide vertices: " ;
				//for(int j = 1; j < intsecV->size(); j++){std::cout << (*intsecV)[j];}
				//std::cout << " " << std::endl;

				// find the opposite polygon to the intersection polygon
				std::vector <bool> sameCornpoint;
				unsigned int opposite = 6;
				
								
				for (unsigned int j = 0; j < mPolygons.size(); j++)
				{
					bool sameCornp;
					
					for (auto m = (*mPolygons[j]).begin(); m != (*mPolygons[j]).end(); m++)
					{
						for (auto l = (*intersecP[0]).begin(); l != (*intersecP[0]).end(); l++)
						{
							if (m->isSameAs(*l, mGeometryModel->tolerance()))
							{
								sameCornpoint.push_back(true);
							}
							else 
							{
								sameCornpoint.push_back(false);
							}
						}
					}
			
					bool isOposite = true;
					for (int a=0; a < sameCornpoint.size(); a++)
					{
						if (sameCornpoint[a] == true)
						{
							isOposite = false;
						}
					}
					
					if (isOposite == true)
					{
						opposite = j;
					}
					sameCornpoint.clear();
				}
				
				if (opposite == 6)
				{
					markInVisualization = true;
					
					std::stringstream errorMessage;
					errorMessage << "\nError, could not find opposite surface when\n"
											 << "splitting a cuboid from a point on a surface\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
								 			 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
					throw std::runtime_error(errorMessage.str());
				}

				// Find the relevant line_segments to generate points on and store the first 2 known cornerpoints, namely the intersection point and the intersected side cornerpoint.
				for (const auto j : *(intersecP[0]))
				{
					// Declerations
					std::vector<utilities::geometry::vertex> cornerPoints;
					std::vector<utilities::geometry::line_segment> intersPoly_line;
					std::vector<utilities::geometry::line_segment> opositePoly_line;
																
					// Add first known cornerpoints
					cornerPoints.push_back(onPolygonV[0]);
					cornerPoints.push_back(j);
					
					
					
					// Find the lines within the polygon which are attached to the cornerPoints of the cuboid, and the attached line to vertex j
					for (const auto& k : mLineSegments)
					{
						for (auto l = (k).begin(); l != (k).end(); l++)
						{
													
							if(!(l->isSameAs(cornerPoints[1], mGeometryModel->tolerance()))) // (*l != cornerPoints[1])
							{ // only allow the continueation of line_segments with are attached to the relevant vertex
								continue;
							}
							
							std::vector <bool> isPartOf;
							isPartOf.clear();
							
							for (const auto& m : intersecP[0]-> getLines())
							{// Determinde if line_segments is the same of the polygon line_segments							
								if (k.isSameAs(m, mGeometryModel->tolerance()))
								{ //if so then add closest point to intersection point to cornerpoints
									intersPoly_line.push_back(k);
									isPartOf.push_back(true);
									continue;
								} 
								else
								{
									isPartOf.push_back(false);
								}
							}
							
							
							bool isPartOfPoly = false;
							for (int a=0; a < isPartOf.size(); a++)
							{ 
								if (isPartOf[a] == true)
								{
									isPartOfPoly = true;
								}
							}
							if (isPartOfPoly == true)
							{ // line_segment is attacht to vertex j, but is not in polygon
								continue;
							}
							
							
							
							// find vertix on the oposide side conected to vertex j
							
							
							// determine which of the point of non polygon line is not the same as the vertex space, so vertex of origonal cuboid on oposide side
							if(!(k[0].isSameAs(cornerPoints[1], mGeometryModel->tolerance()))) //(k[0] != cornerPoints[1])
							{
								cornerPoints.push_back(k[0]);
								for (const auto& m : mPolygons[opposite]-> getLines())
								{
									for (auto n = (m).begin(); n != (m).end(); n++)
									{
										if(!(k[0].isSameAs(*n, mGeometryModel->tolerance())))//(k[0] != *n)
										{
											continue;
										}
										opositePoly_line.push_back(m);							
									}
								}
							}
							else if(!(k[1].isSameAs(cornerPoints[1], mGeometryModel->tolerance()))) // (k[1] != cornerPoints[1])
							{
								cornerPoints.push_back(k[1]);
								
								for (const auto& m : mPolygons[opposite]-> getLines())
								{
									for (auto n = (m).begin(); n != (m).end(); n++)
									{
										if(!(k[1].isSameAs(*n, mGeometryModel->tolerance()))) //(k[1] != *n)
										{
											continue;
										}
										opositePoly_line.push_back(m);
									}
								}
							}
						}
					}
					
				

					
					// Intersection Poly
					
					//if onLineVertex are on the intersPoly_line then add the intersectionPoly_line_segments points to the cornerPoints value
					//else if there are no intersection points on the intersectionPoly_linesegments, add the perpendicular points to intersection point
					std::vector<bool> spaceInsecVInsert;
					for (int b = 0; b < intersPoly_line.size(); b++)
					{
						bool isFound = false;
						if (onPolygonV.size() > 1) // check for guidevertices on the polygon and project these points to the opposide side
						{
							for (int l=1; l<onPolygonV.size(); l++)
							{
								bool ProjectionIntersect = false;
								bso::utilities::geometry::vertex polyIntersectP;
								utilities::geometry::line_segment polyIntersectLine = {{cornerPoints[0]},{onPolygonV[l]}};
																
								ProjectionIntersect = polyIntersectLine.vIntersects(intersPoly_line[b],polyIntersectP);
								
								utilities::geometry::line_segment cuboidLine = {{cornerPoints[0]},{polyIntersectP}};
																
								if(ProjectionIntersect == true && intersPoly_line[b].isOnLine(polyIntersectP, mGeometryModel->tolerance()) && cuboidLine.isOnLine(onPolygonV[l], mGeometryModel->tolerance()))
								{
									cornerPoints.push_back(polyIntersectP);
									isFound = true;
									spaceInsecVInsert.push_back(true);
								}
							}
						}
						
						for (const auto l: onLineV) // check for guidevertices on the lines of the polygon and if yes add acordingly to cornerPoints
						{
							if (intersPoly_line[b].isOnLine(l, mGeometryModel->tolerance()))
							{
								cornerPoints.push_back(l);
								isFound = true;
								spaceInsecVInsert.push_back(true);
								continue;
							}
						}
						if(isFound == false) // Generate the vertices on the intersection polygon in case there are no guide vertices
						{
							if((intersPoly_line[b]).isOnLine((intersPoly_line[b].getPointClosestTo(onPolygonV[0])), mGeometryModel->tolerance()))
							{
								cornerPoints.push_back(intersPoly_line[b].getPointClosestTo(onPolygonV[0]));
							}
							else
							{
								markInVisualization = true;
					
								std::stringstream errorMessage;
								errorMessage << "\nError, the found intersection point is not on the considered Line of the intersection polygon \n"
										 << "during the intersected polygon split process\n"
										 << "The cuboid to replace is: " << *this << " \n"
										 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
										 << "The cornerpoints for the new cuboid generation are: " << std::endl;
										 for(const auto p: cornerPoints) {errorMessage << p;}
							errorMessage << " \n"
										 << "see section Intersection Poly of\n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
							
							spaceInsecVInsert.push_back(false);
						}								
					}

					
					//Opposide Poly, perpendicular to oposite line
					if (method == 'O')
					{
						if(mPolygons[opposite]-> isInside(mPolygons[opposite]->getPointClosestTo(onPolygonV[0]), mGeometryModel->tolerance()))
						{
							cornerPoints.push_back(mPolygons[opposite]->getPointClosestTo(onPolygonV[0])); // middle point on opposide polygon
						}
						else
						{
							markInVisualization = true;
					
							std::stringstream errorMessage;
							errorMessage << "\nError, the found intersection point is not on the considered oposite polygon \n"
									 << "The cuboid to replace is: " << *this << " \n"
									 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
									 << "The cornerpoints for the new cuboid generation are: " << std::endl;
									 for(const auto p: cornerPoints) {errorMessage << p;}
						errorMessage << " \n"
									 << "see section Opposide Poly, perpendicular to oposite line method == 'O'\n"
									 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
							throw std::runtime_error(errorMessage.str());
						}
						
						for (int b = 0; b < intersPoly_line.size(); b++)
						{							
							for(int c = 0; c < mPolygons.size(); c++)
							{
								bool isBothOnPoly1 = false;
																
								if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[0], mGeometryModel->tolerance())) 
								{
									if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[1], mGeometryModel->tolerance()))
									{
										isBothOnPoly1 = true;
									}
								}
								if (isBothOnPoly1 == false){continue;}
								
								for (int d = 0; d < opositePoly_line.size(); d++)
								{
									bool isBothOnPoly2 = false;
									if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[0], mGeometryModel->tolerance())) 
									{
										if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[1], mGeometryModel->tolerance()))
										{
											isBothOnPoly2 = true;
										}
									}
									if (isBothOnPoly2 == false){continue;}
									
									// found opposite line to intersPoly_line, therefore add the acording conerpoint (guidepoint or not):
									if (spaceInsecVInsert[b] == false) // no guidevertex to take into account
									{
										if((opositePoly_line[d]).isOnLine((opositePoly_line[d].getPointClosestTo(onPolygonV[0])), mGeometryModel->tolerance()))
										{
											cornerPoints.push_back(opositePoly_line[d].getPointClosestTo(onPolygonV[0]));
										}
										else
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, the found intersection point is not on the considered line of the oposite polygon \n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "see section Opposide Poly, perpendicular to oposite line method == 'O'/ spaceInsecVInsert[b] == false\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
									}
									if (spaceInsecVInsert[b] == true) // guidevertex is taken into account
									{
										//determine where the line intersects with the plane
										
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = mPolygons[opposite]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side \n"
														 << "during the polygon split process perpendicular to intersection\n"
														 << "The cuboid to replace is: " << *this << " \n"
														 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
														 << "The cornerpoints for the new cuboid generation are: " << std::endl;
														 for(const auto p: cornerPoints) {errorMessage << p;}
											errorMessage << " \n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
										
										
										
										/*
										if((opositePoly_line[d]).isOnLine((opositePoly_line[d].getPointClosestTo(cornerPoints[3+b])), mGeometryModel->tolerance()))
										{
											cornerPoints.push_back(opositePoly_line[d].getPointClosestTo(cornerPoints[3+b]));
											std::cout << "identified correct section" << opositePoly_line[d].getPointClosestTo(cornerPoints[3+b]) << std::endl;
										}
										else
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, the found intersection point is not on the considered line of the oposite polygon \n"
													 << "The cuboid to split is: " << *this << "\n"
													 << "see section Opposide Poly, perpendicular to oposite line method == 'O'/ spaceInsecVInsert[b] == true\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										*/
										
										
										
										
									}
								}
							}
						} //Opposide Poly, perpendicular to oposite line
					} // if (method == 'O')
					
					
					//Opposide Poly, perpendicular to intersection polygon.
					if (method == 'I')
					{
						for (int b = 0; b < intersPoly_line.size(); b++)
						{
							for(int c = 0; c < mPolygons.size(); c++)
							{
								bool isBothOnPoly1 = false;
								
								
								if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[0], mGeometryModel->tolerance())) 
								{
									if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[1], mGeometryModel->tolerance()))
									{
										isBothOnPoly1 = true;
									}
								}
								if (isBothOnPoly1 == false){continue;}
								
								for (int d = 0; d < opositePoly_line.size(); d++)
								{
									bool isBothOnPoly2 = false;
									if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[0], mGeometryModel->tolerance())) 
									{
										if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[1], mGeometryModel->tolerance()))
										{
											
											isBothOnPoly2 = true;
										}
									}
									if (isBothOnPoly2 == false){continue;}
									
									
									// found opposite line to intersPoly_line, therefore add conerpoint acordingly:
									if (spaceInsecVInsert[b] == false)
									{
										//determine where the line intersects with the plane
										// (http://geomalgorithms.com/a05-_intersect-1.html) background information intersection point line-plane
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = intersecP[0]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "see section Opposide_Poly/perpendicular to intersection polygon/(spaceInsecVInsert[b] == false)\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side during the polygon split process \n"
														 << "The cuboid to replace is: " << *this << " \n"
														 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
														 << "The cornerpoints for the new cuboid generation are: " << std::endl;
														 for(const auto p: cornerPoints) {errorMessage << p;}
											errorMessage << " \n"
														 << "see section Opposide_Poly/perpendicular to intersection polygon/(spaceInsecVInsert[b] == false)\n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
										
										
									}
									if (spaceInsecVInsert[b] == true)
									{
										//determine where the line intersects with the plane
										
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = intersecP[0]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side \n"
														 << "during the polygon split process perpendicular to intersection\n"
														 << "The cuboid to replace is: " << *this << " \n"
														 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
														 << "The cornerpoints for the new cuboid generation are: " << std::endl;
														 for(const auto p: cornerPoints) {errorMessage << p;}
											errorMessage << " \n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
									}
								}
							}
							
							
							if(b == 0) // Deze bepaling kan warschijnlijk makelijker met de functie: intersectsWith, aangezien de polygon al bestaat.
							{// intersectionpoint on oposite polygon
								utilities::geometry::vertex pInt;
								double tol = mGeometryModel->tolerance();
								
								utilities::geometry::vector normalIntsect = intersecP[0]->normal();
								utilities::geometry::vector normalOposite = mPolygons[opposite]->normal();
								
								utilities::geometry::vector v3 = cornerPoints[3] -cornerPoints[0];
								utilities::geometry::vector v4 = cornerPoints[4] -cornerPoints[0];
								
								
								
								if (normalOposite.isPerpendicular(v3, mGeometryModel->tolerance()) && normalOposite.isPerpendicular(v4, mGeometryModel->tolerance()))
								{//check if normals oposite plane is Perpendicular to the 2 vector within intersection plane
									if(mPolygons[opposite]->isInside((mPolygons[opposite]->getPointClosestTo(onPolygonV[0])), mGeometryModel->tolerance()))
									{
										cornerPoints.push_back(mPolygons[opposite]->getPointClosestTo(onPolygonV[0])); // point on opposide polygon	
									}
									else
									{
										markInVisualization = true;
					
										std::stringstream errorMessage;
										errorMessage << "\nError, could not find the intersection point on the opposite side \n"
												 << "during the polygon split process perpendicular to intersection\n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "See section polygon split/perpedicular to intersection/if(b == 0)\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}							
								}
								else
								{//determine where the line intersects with the plane
									
									utilities::geometry::line_segment l1 = {(cornerPoints[0]+normalIntsect),cornerPoints[0]};
									utilities::geometry::vector mNormal = mPolygons[opposite]->normal();
									
									
									if (l1.getVector().isPerpendicular(mNormal, tol))
									{
										markInVisualization = true;
					
										std::stringstream errorMessage;
										errorMessage << "\nError, could not find the intersection point on the opposite side \n"
												 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "See section polygon split/perpedicular to intersection/the else section of if(b == 0)\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
									else
									{									
										utilities::geometry::vector v1 = l1[1]-l1[0];
										utilities::geometry::vector v2 = ((mPolygons[opposite])->getVertices())[0]-l1[0]; 
										

										//check if the line intersects with the plane
										double r = (mNormal.dot(v2))/(mNormal.dot(v1));

										if (r > -tol /*&& r < (1 + tol)*/)
										{ // if it does, compute the point
											pInt = l1[0] + r * v1;
											cornerPoints.push_back(pInt);
										}
										else
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line does not intersect with the considered plane\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "See section polygon split/perpedicular to intersection/the else section of if(b == 0)\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
									}
								}
							}
						} //Opposide Poly, perpendicular to intersection polygon.
					} // if (method == 'I')
					
				
					// Opposide Poly, middle ratio method
					if (method == 'M')
					{
						markInVisualization = true;
					
						std::stringstream errorMessage;
							errorMessage << "\nError, the M split method polygon is not finished yet.  \n"
									 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
							throw std::runtime_error(errorMessage.str());
						
						
						// std::vector<utilities::geometry::line_segment> intersPoly_line;
						// std::vector<utilities::geometry::line_segment> opositePoly_line;
						// mPolygons[opposite];
						// *intersecP[0]
						

						/*
						virtual double baseArea = intersecP[0]->getArea()
						
						
						for(for (auto l = (intersecP[0]).begin(); l != (intersecP[0]).end(); l++))
						{
							vertixesAll.push_back(l);
						}
						
						
						// find ratio middle intersection point on opposite side
						utilities::geometry::vertex oppositePCenter = mPolygons[opposite]->getCenter();
						utilities::geometry::vertex intersectionPCenter = intersecP[0]->getCenter();
						utilities::geometry::vector oppositeNormal = mPolygons[opposite]->getNormal();
						utilities::geometry::vector intersectionNormal = intersecP[0]->getNormal();
						
						std::vector<utilities::geometry::vector> xyzUnit;
						xyzUnit.push_back({1,0,0});
						xyzUnit.push_back({0,1,0});
						xyzUnit.push_back({0,0,1});
						utilities::geometry::vector unitz = {0,0,1};
						utilities::geometry::vector unitx = {1,0,0};
						utilities::geometry::vector unity = {0,1,0};
						
						std::vector<utilities::geometry::vector> xyzo;
						xyzo.push_back(unitz.cross(oppositeNormal).normalized());
						xyzo.push_back(unitz.cross(oppositeNormal).normalized());
						xyzo.push_back(unitz.cross(oppositeNormal).normalized());
						utilities::geometry::vector xo = unitz.cross(oppositeNormal).normalized();
						utilities::geometry::vector yo = unity.cross(oppositeNormal).normalized();
						utilities::geometry::vector zo = unitx.cross(oppositeNormal).normalized();
						
						std::vector<utilities::geometry::vector> xyzi;
						utilities::geometry::vector xi = unitz.cross(intersectionNormal).normalized();
						utilities::geometry::vector yi = unitz.cross(intersectionNormal).normalized();
						utilities::geometry::vector zi = unitx.cross(intersectionNormal).normalized();
						
						utilities::geometry::vector translationVI = onPolygonV[0]-intersectionPCenter;
						
						// Add Vertex
						
						
						
						//Find area ratio's
						std::vector<double> ratio;
						std::vector<bool> usedRatio;
						// x=0
						// y=1
						// z=2
						
						for(int i = 0; i< 2; i++)
						{
							std::vector<utilities::geometry::vertex> vertixesAll;
							for(for (auto l = (intersecP[0]).begin(); l != (intersecP[0]).end(); l++))
							{
								vertixesAll.push_back(l);
							}
							
							if(xyzi[i]==xyzUnit[i])
							{
								i--;
								usedRatio.push_back(false);
								continue;
							}
							
							usedRatio.push_back(true);

						}
						
						*/
						
						
						
						//utilities::geometry::vector translationVO =  -oppositePCenter;
						
						/*
						for (int b = 0; b < intersPoly_line.size(); b++)
						{
							for(int c = 0; c < mPolygons.size(); c++)
							{
								bool isBothOnPoly1 = false;
								
								
								if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[0], mGeometryModel->tolerance())) 
								{
									if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[1], mGeometryModel->tolerance()))
									{
										isBothOnPoly1 = true;
									}
								}
								if (isBothOnPoly1 == false){continue;}
								
								for (int d = 0; d < opositePoly_line.size(); d++)
								{
									bool isBothOnPoly2 = false;
									if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[0], mGeometryModel->tolerance())) 
									{
										if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[1], mGeometryModel->tolerance()))
										{
											
											isBothOnPoly2 = true;
										}
									}
									if (isBothOnPoly2 == false){continue;}
									
									
									// found opposite line to intersPoly_line, therefore add conerpoint acordingly:
									if (spaceInsecVInsert[b] == false)
									{
										//determine where the line intersects with the plane
										// (http://geomalgorithms.com/a05-_intersect-1.html) background information intersection point line-plane
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = intersecP[0]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side \n"
														 << "during the polygon split process perpendicular to intersection plane\n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
										
										
									}
									if (spaceInsecVInsert[b] == true)
									{
										//determine where the line intersects with the plane
										
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = intersecP[0]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side \n"
														 << "during the polygon split process perpendicular to intersection plane\n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
										
										//std::cout << "spaceInsecVinsert[b] == true, with the following opositePoly_line[d]: " << opositePoly_line[d]<< "Whith the following normal: "<< mNormal << " add the following cornerpoints: " << pInt << std::endl;
									}
								}
							}
						
							*/
					} // if (method == 'M')
					
					
					
					// safety checks
					// Check for duplicates
					bool duplicate = false;
					for(int t=0; t<cornerPoints.size(); t++)
					{
						for(int s=0; s<cornerPoints.size(); s++)
						{
							if(t == s)
							{
								continue;
							}
							if(cornerPoints[t].isSameAs(cornerPoints[s], mGeometryModel->tolerance()))
							{
								markInVisualization = true;
					
								std::stringstream errorMessage;
								errorMessage << "\nError, There are duplicates inside the to generate quad-hexahedron cornerPoints.\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "Is the newly generated cell a triangular prims?\n"
											 << "Research error direction segestion: In case the inserted tolerance is 0.1 mm, then the polygon intersection check can mistake a line \n"
											 << "for a polygon intersection. Therefore performing the wrong split case resulting in duplicate value's"
											 << "Solution, make tolerance smaller\n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
						}
					}
					
					if (cornerPoints.size() < 8)
					{
						markInVisualization = true;
					
						std::stringstream errorMessage;
						errorMessage << "\nError, There are not enough value's in cornerpoints to\n"
								 << "generate the cuboids to split the cell during the split polygon precedure\n"
								 << "The cuboid to replace is: " << *this << " \n"
								 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
								 << "The cornerpoints for the new cuboid generation are: " << std::endl;
								 for(const auto p: cornerPoints) {errorMessage << p;}
					errorMessage << " \n"
								 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					if (cornerPoints.size() > 8)
					{
						markInVisualization = true;
					
						std::stringstream errorMessage;
						errorMessage << "\nError, There are too manny value's in cornerpoints to\n"
								 << "generate the cuboids to split the cell during the split polygon precedure\n"
								 << "The cuboid to replace is: " << *this << " \n"
								 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
								 << "The cornerpoints for the new cuboid generation are: " << std::endl;
								 for(const auto p: cornerPoints) {errorMessage << p;}
					errorMessage << " \n"
								 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					
					// The new cuboids generator
					newCuboids.push_back(mGeometryModel->addCuboid(
							utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance())));
				}
			}
		} // split polygon
		
		
		
		// split method for intersection points on line_segments which defide the cell into 2 cells
		if(!split)
		{ 
			if (onLineV.size() >= 1 && firstVertexOnLine == true)
			{
				split = true;
				//std::cout << "Split acording to line intersection: " << onLineV[0] << " with guide vertices: " ;
				//for(int j = 1; j < intsecV->size(); j++){std::cout << (*intsecV)[j];}
				//std::cout << " " << std::endl;
				
				/*
				if (onLineV.size() > 3)
				{// check if there are to manny guide intersection points
					markInVisualization = true;
					
					std::stringstream errorMessage;
						errorMessage << "\nError, can't split space when there are more than 2 guide vertices \n"
												 << "There are the following amound of points intersecting: " << onLineV.size() << " \n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
				}
				*/
				
				// if check is fine, then preceed on 
								
				for (auto i = (intersecL[0]).begin(); i != (intersecL[0]).end(); i++)
				{
					// Declerations
					std::vector<utilities::geometry::vertex> cornerPoints;
					std::vector<utilities::geometry::polygon*> polyInCuboid; //PolyInCuboid are the polygons which are the polygons to the side which does not need splitting
					std::vector<utilities::geometry::line_segment> lineToSplit; //Lines which need to be split for the generation of a new cuboid
					
					// Add first known cornerPoints
					cornerPoints.push_back(onLineV[0]);
					
					// Find the PolyInCuboid geometry
					for (const auto& j : mPolygons)
					{				
						for (auto k = (*j).begin(); k != (*j).end(); k++)
						{
							if(!(i->isSameAs(*k, mGeometryModel->tolerance()))) //(*i != *k)
							{
								continue;
							}
							
							bool polyWanted = true;
							for (const auto l: j->getLines())
							{
								if((l.isSameAs(intersecL[0], mGeometryModel->tolerance()))) // (l == intersecL[0])
								{
									polyWanted = false;
									continue;
								}	
								
							}
							
							if(polyWanted == true)
							{
								polyInCuboid.push_back(j);
							}
						}
					}
					
					if (polyInCuboid.size() >= 2)
					{
						markInVisualization = true;
					
						std::stringstream errorMessage;
						errorMessage << "\nError, found multiple PolyInCuboid, therefore having to manny vertexen to initiate a cuboid with \n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					// find lineToSplit
					for (auto j = (*polyInCuboid[0]).begin(); j != (*polyInCuboid[0]).end(); j++)
					{
						std::vector<utilities::geometry::line_segment> Temp;
						for (auto k : mLineSegments)
						{
							for (auto l = k.begin(); l != k.end(); l++)
							{
								if(!(l->isSameAs(*j, mGeometryModel->tolerance()))) // (*l != *j)
								{
									continue;
								}
								
								Temp.push_back(k);
							}
						}
						
						for (auto k: Temp)
						{
							bool partOfPoly = false;
							for (auto j: polyInCuboid[0]->getLines())
							{
								if(j.isSameAs(k, mGeometryModel->tolerance())) //(j == k)
								{
									partOfPoly = true;
								}
							}

							if (partOfPoly == false)
							{
								lineToSplit.push_back(k);
							}
						}
						Temp.clear();
					}
								
					
					
					// Add all the vertexes of the cuboid to variable cornerPoints
					
					// perpendicular to opposite side method
					if (method == 'O')
					{
						// add the already known in new cuboids vertexes
						for (auto j = (*polyInCuboid[0]).begin(); j != (*polyInCuboid[0]).end(); j++)
						{//polygon of the new cuboid which goes unchanged into the new cuboid
							cornerPoints.push_back(*j);
						}
						
						// determine if there is a guidance line
						utilities::geometry::vertex guidanceV; 
						utilities::geometry::line_segment guidanceL;
						bool guidanceVertex = false;
						for (auto j : lineToSplit)
						{//the new vertixen in the middle of the to split cuboids
							if (j == intersecL[0])
							{
								continue;
							}
							// check if line already has an intersection point
							for(int k=1; k < onLineV.size(); k++)
							{				
								if(j.isOnLine(onLineV[k], mGeometryModel->tolerance()))
								{
									guidanceVertex = true;
									cornerPoints.push_back(onLineV[k]);
									guidanceV = onLineV[k];
									guidanceL = j;
									continue;
								}
							}
						}
						
						// Check if anny of the (projected) guide vertices are on the polyInCuboid, since then this results in a triangular prism 
						for (const auto j: polyInCuboid[0]->getLines())
						{
							for(int k=1; k < onLineV.size(); k++)
							{				
								utilities::geometry::vertex dummy;
								utilities::geometry::line_segment guideVProject = {{onLineV[0]},{onLineV[k]}};
								
								if(j.vIntersects(guideVProject, dummy, mGeometryModel->tolerance()))
								{
									if(polyInCuboid[0]->isInsideOrOn(dummy, mGeometryModel->tolerance()))
									{
										bool isCornerpoint = false;
										for(const auto l: this->getVertices())
										{
											if(dummy.isSameAs(l,mGeometryModel->tolerance()))
											{
												isCornerpoint = true;
											}
										}
										if(!isCornerpoint)
										{										
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, there is a guide vertex on the line of the polyInCuboid of the newly to generate cell \n"
													 << "during the line split process, therefore generating a triangular prism. \n"
													 << "This is currently unsuported in the toolbox\n"
													 << "The guide vertex considered is: " << onLineV[k] << "\n"
													 << "The calculated interseciton point is: " << dummy << "\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onLineV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
									}
								}
							}
						}
						
						// find intersection point on rest of LineToSplit
						for (auto j : lineToSplit)
						{
							if (j == intersecL[0]) // no tolerance needed during this check
							{
								continue;
							}
							
							if (guidanceL == j) // no tolerance needed during this check
							{
								continue;
							}
							
							
							//define intersection variables
							utilities::geometry::vertex pInt;
							double tol = mGeometryModel->tolerance();
							utilities::geometry::line_segment l1 = j;
							utilities::geometry::vector mNormal;
							// assign mNormal
							if (guidanceVertex == true)
							{
								utilities::geometry::polygon* intersectionPolygon;
								utilities::geometry::polygon* opositePolygon;
								
								// Find intersectionPolygon with guideline on it
								for(int c = 0; c < mPolygons.size(); c++)
								{
									bool isBothOnPoly1 = false;
																	
									if (mPolygons[c]->isCoplanarN((onLineV[0]), mGeometryModel->tolerance())) 
									{
										if (mPolygons[c]->isCoplanarN(guidanceV, mGeometryModel->tolerance()))
										{
											isBothOnPoly1 = true;
										}
									}
									if (isBothOnPoly1 == false){continue;}
									
									intersectionPolygon = mPolygons[c];
									break;
								}
								
								// Find oppositePolygon to intersectionPolygon
								std::vector <bool> sameCornpoint;
								unsigned int opposite = 6;
								
								for (unsigned int j = 0; j < mPolygons.size(); j++)
								{
									bool sameCornp;
									
									for (auto m = (*mPolygons[j]).begin(); m != (*mPolygons[j]).end(); m++)
									{
										for (auto l = (*intersectionPolygon).begin(); l != (*intersectionPolygon).end(); l++)
										{
											if (*m == *l) // geen tolerantie nodig aangezien deze waardes identiek horen te zijn omdat ze van dezelfde quad-hexahedron zijn
											{
												sameCornpoint.push_back(true);
											}
											else 
											{
												sameCornpoint.push_back(false);
											}
										}
									}
							
									bool isOposite = true;
									for (int a=0; a < sameCornpoint.size(); a++)
									{
										if (sameCornpoint[a] == true)
										{
											isOposite = false;
										}
									}
									
									if (isOposite == true)
									{
										opposite = j;
										opositePolygon = mPolygons[j];
									}
									sameCornpoint.clear();
								}
								
								if (opposite == 6)
								{
									markInVisualization = true;
					
									std::stringstream errorMessage;
									errorMessage << "\nError, could not find opposite surface when\n"
															 << "splitting a cuboid from a point on a surface\n"
															 << "The cuboid to replace is: " << *this << " \n"
															 << "The intersection point in consideration is: " << onLineV[0] << " \n"
															 << "The cornerpoints for the new cuboid generation are: " << std::endl;
															 for(const auto p: cornerPoints) {errorMessage << p;}
												errorMessage << " \n"
															 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
								
								// Use the normal of the oppositePolygon to define the normal of the cell plane for line intersections
								utilities::geometry::line_segment guidanceLine = {{onLineV[0]},{guidanceV}};
								utilities::geometry::vector guidanceLVector = guidanceLine.getVector();
								utilities::geometry::vector intersectionPlaneV = opositePolygon->getNormal();
								mNormal = (intersectionPlaneV.cross(guidanceLVector));	
								
								
								// compute rest of intersection points by plane intersection with the to split lines
								if (l1.getVector().isPerpendicular(mNormal, tol))
								{
									markInVisualization = true;
					
									std::stringstream errorMessage;
									errorMessage << "\nError, could not find the intersection point on the opposite side \n"
											 << "during the line split process, since the line to intersect is perpendicular to the plane\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
								else
								{
									utilities::geometry::vector v1 = l1[1]-l1[0];
									utilities::geometry::vector v2 = onLineV[0]-l1[0];

									//check if the line intersects with the plane
									double r = (mNormal.dot(v2))/(mNormal.dot(v1));
									if (r > -tol && r < (1 + tol))
									{ // if it does, compute the point
										pInt = l1[0] + r * v1;
										cornerPoints.push_back(pInt);
									}
									else
									{
										markInVisualization = true;
					
										std::stringstream errorMessage;
										errorMessage << "\nError, the vertex is not within the oposite lines borders \n"
												 << "during the line to split process of the perpendicular to opposite method (guidanceVertex == true)\n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
								}
							}
							else 
							{
								// find opositePolygon, there are 2 polygon opposites
								utilities::geometry::polygon* opositePolygon1;
								utilities::geometry::polygon* opositePolygon2;
								int k = 0;
								for(int c = 0; c < mPolygons.size(); c++)
								{
									bool isBothNotOnPoly1 = true;
																		
									if ((mPolygons[c]->isCoplanarN((intersecL[0])[0], mGeometryModel->tolerance()))) 
									{
										isBothNotOnPoly1 = false;
									}
									else if ((mPolygons[c]->isCoplanarN((intersecL[0])[1], mGeometryModel->tolerance())))
									{
										isBothNotOnPoly1 = false;
									}
									if (isBothNotOnPoly1 == false)
									{
										continue;
									}
									
									
									if(k == 0)
									{
										opositePolygon1 = mPolygons[c];
										k++;
									}
									else if (k == 1)
									{
										opositePolygon2 = mPolygons[c];
										k++;
									}
									else
									{
										markInVisualization = true;
					
										std::stringstream errorMessage;
										errorMessage << "\nError, found to manny opposite polygons \n"
												 << "during the line split process of split to opposite \n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
								}
								
								// Use the normal of the two oppositePolygon to define the normal of the cell plane for line intersections
								utilities::geometry::vector opositePolygon2N = opositePolygon2->getNormal();
								utilities::geometry::vector opositePolygon1N = opositePolygon1->getNormal();
								mNormal = (opositePolygon1N.cross(opositePolygon2N));	
								
								
								// compute rest of intersection points by plane intersection with the to split lines
								if (l1.getVector().isPerpendicular(mNormal, tol))
								{
									markInVisualization = true;
					
									std::stringstream errorMessage;
									errorMessage << "\nError, could not find the intersection point on the opposite side \n"
											 << "during the line split process, since the line to intersect is perpendicular to the plane\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
								else
								{
									utilities::geometry::vector v1 = l1[1]-l1[0];
									utilities::geometry::vector v2 = onLineV[0]-l1[0];

									//check if the line intersects with the plane
									double r = (mNormal.dot(v2))/(mNormal.dot(v1));
									//std::cout << "r is: " << r << std::endl;
									if (r > -tol && r < (1 + tol))
									{ // if it does, compute the point
										pInt = l1[0] + r * v1;
										cornerPoints.push_back(pInt);
									}
									else
									{
										markInVisualization = true;
					
										std::stringstream errorMessage;
										errorMessage << "\nError, the vertex is not within the oposite lines borders \n"
												 << "during the line to split process of the perpendicular to opposite method (guidanceVertex == false)\n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
								}
							}
						}
					} // if (method == 'O')
					
					
					// perpendicular to intersection side method
					if (method == 'I')
					{
						// add the already known in cornerPoints
						for (auto j = (*polyInCuboid[0]).begin(); j != (*polyInCuboid[0]).end(); j++)
						{//polygon of the new cuboid which goes unchanged into the new cuboid
							cornerPoints.push_back(*j);
						}
						
						// determine if there is a guidance line
						utilities::geometry::vertex guidanceV; 
						utilities::geometry::line_segment guidanceL;
						bool guidanceVertex = false;
						for (const auto j : lineToSplit)
						{//the new vertixen in the middle of the to split cuboids
							if (j == intersecL[0])
							{
								continue;
							}
							
							// check if line already has an intersection point
							for(int k=1; k < onLineV.size(); k++)
							{				
								if(j.isOnLine(onLineV[k], mGeometryModel->tolerance()))
								{
									guidanceVertex = true;
									cornerPoints.push_back(onLineV[k]);
									guidanceV = onLineV[k];
									guidanceL = j;
								}
							}
						}
						
						// Check if anny of the (projected) guide vertices are on the polyInCuboid, since then this results in a triangular prism 
						for (const auto j: polyInCuboid[0]->getLines())
						{
							for(int k=1; k < onLineV.size(); k++)
							{				
								utilities::geometry::vertex dummy;
								utilities::geometry::line_segment guideVProject = {{onLineV[0]},{onLineV[k]}};
								
								if(j.vIntersects(guideVProject, dummy, mGeometryModel->tolerance()))
								{
									if(polyInCuboid[0]->isInsideOrOn(dummy, mGeometryModel->tolerance()))
									{
										bool isCornerpoint = false;
										for(const auto l: this->getVertices())
										{
											if(dummy.isSameAs(l,mGeometryModel->tolerance()))
											{
												isCornerpoint = true;
											}
										}
										if(!isCornerpoint)
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, there is a guide vertex on the line of the polyInCuboid of the newly to generate cell \n"
													 << "during the line split process, therefore generating a triangular prism. \n"
													 << "This is currently unsuported in the toolbox\n"
													 << "The guide vertex considered is: " << onLineV[k] << "\n"
													 << "The calculated interseciton point is: " << dummy << "\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onLineV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
									}
								}
							}
						}
						
						// find intersection point on rest of LineToSplit
						for (auto j : lineToSplit)
						{
							if (j == intersecL[0])// no tolerance needed during this check
							{
								continue;
							}
							
							if (guidanceL == j)// no tolerance needed during this check
							{
								continue;
							}
							
							
							//define intersection variables
							utilities::geometry::vertex pInt;
							double tol = mGeometryModel->tolerance();
							utilities::geometry::line_segment l1 = j;
							utilities::geometry::vector mNormal;
							// assign mNormal
							if (guidanceVertex == true)
							{
								utilities::geometry::line_segment guidanceLine = {{onLineV[0]},{guidanceV}};
								utilities::geometry::vector intersecLVector = (intersecL[0]).getVector();
								utilities::geometry::vector guidanceLVector = guidanceLine.getVector();

								utilities::geometry::vector intersectionPlaneV = (intersecLVector.cross(guidanceLVector)).normalized();
								mNormal = (intersectionPlaneV.cross(guidanceLVector));								
							}
							else 
							{
								mNormal = (intersecL[0]).getVector();
							}

							// compute rest of intersection points
							
							
							if (l1.getVector().isPerpendicular(mNormal, tol))
							{
								markInVisualization = true;
					
								std::stringstream errorMessage;
								errorMessage << "\nError, could not find the intersection point on the opposite side \n"
										 << "during the line split process, since the line to intersect is perpendicular to the plane\n"
										 << "The cuboid to replace is: " << *this << " \n"
										 << "The intersection point in consideration is: " << onLineV[0] << " \n"
										 << "The cornerpoints for the new cuboid generation are: " << std::endl;
										 for(const auto p: cornerPoints) {errorMessage << p;}
							errorMessage << " \n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
							else
							{
								utilities::geometry::vector v1 = l1[1]-l1[0];
								utilities::geometry::vector v2 = onLineV[0]-l1[0];

								//check if the line intersects with the plane
								double r = (mNormal.dot(v2))/(mNormal.dot(v1));
								if (r > -tol && r < (1 + tol))
								{ // if it does, compute the point
									pInt = l1[0] + r * v1;
									cornerPoints.push_back(pInt);
								}
								else
								{
									markInVisualization = true;
					
									std::stringstream errorMessage;
									errorMessage << "\nError, the vertex is not within the oposite lines borders \n"
											 << "during the line to split process of the perpendicular to intersected method\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "the line found to intersect is: " << intersecL[0] << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
							}
						}
					} // if (method == 'I')
					
					
					// not fully finished yet
					// Ratio method
					if (method == 'M')
					{
						// add the already known in new cuboids vertexes
						for (auto j = (*polyInCuboid[0]).begin(); j != (*polyInCuboid[0]).end(); j++)
						{//polygon of the new cuboid which goes unchanged into the new cuboid
							cornerPoints.push_back(*j);
						}
						
						// determine if there is a guidance line
						utilities::geometry::vertex guidanceV; 
						utilities::geometry::line_segment guidanceL;
						bool guidanceVertex = false;
						for (auto j : lineToSplit)
						{//the new vertixen in the middle of the to split cuboids
							if (j == intersecL[0])
							{
								continue;
							}
							// check if line already has an intersection point
							for(int k=1; k < onLineV.size(); k++)
							{				
								if(j.isOnLine(onLineV[k]))
								{
									guidanceVertex = true;
									cornerPoints.push_back(onLineV[k]);
									guidanceV = onLineV[k];
									guidanceL = j;
									continue;
								}
							}
						}
						
						// Define the middle ratio point on the shortest line so that through the intersection plane the second one can be found
						utilities::geometry::line_segment lToAccound;
						double lengthlToAccound = 0;
						for (auto j : lineToSplit)
						{
							if (j == intersecL[0])
							{
								continue;
							}
							
							if (guidanceL == j)
							{
								continue;
							}
							
							if(j.getLength() > lengthlToAccound)
							{
								lToAccound = j;
								lengthlToAccound = j.getLength();
							}
						}
						
						// find intersection point on rest of LineToSplit
						for (auto j : lineToSplit)
						{
							if (j == intersecL[0])
							{
								continue;
							}
							
							if (guidanceL == j)
							{
								continue;
							}
							
							
							//define intersection variables
							utilities::geometry::vertex pInt;
							double tol = mGeometryModel->tolerance();
							utilities::geometry::line_segment l1 = j;
							utilities::geometry::vector mNormal;
							
							// assign mNormal
							if (guidanceVertex == true)
							{
								utilities::geometry::line_segment guidanceLine = {{onLineV[0]},{guidanceV}};
								utilities::geometry::vector guidanceLVector = guidanceLine.getVector();
								
								// define intersectionPlaneV
								utilities::geometry::vector IntersecVector = intersecL[0].getVector();
								utilities::geometry::line_segment partLine1 = {{(intersecL[0])[0]},{cornerPoints[0]}};
								
								double IntersecLength = (intersecL[0]).getLength();
								double IntersecLength1 = partLine1.getLength();
								double ratio = IntersecLength1/IntersecLength;

								utilities::geometry::vector interlineVector = lToAccound[1]-lToAccound[0];
								utilities::geometry::vertex jIntersection = (lToAccound[0]+interlineVector*ratio);
								utilities::geometry::vector intersectionPlaneV = jIntersection-cornerPoints[0];
								intersectionPlaneV.normalized();

								mNormal = (intersectionPlaneV.cross(guidanceLVector));								
							}
							else 
							{
								// mNormal = (intersecL[0]).getVector();
							}

							
							// compute rest of intersection points
							
							
							if (l1.getVector().isPerpendicular(mNormal, tol))
							{
								markInVisualization = true;
					
								std::stringstream errorMessage;
								errorMessage << "\nError, could not find the intersection point on the opposite side \n"
										 << "during the line split process, since the line to intersect is perpendicular to the plane\n"
										 << "The cuboid to replace is: " << *this << " \n"
										 << "The intersection point in consideration is: " << onLineV[0] << " \n"
										 << "The cornerpoints for the new cuboid generation are: " << std::endl;
										 for(const auto p: cornerPoints) {errorMessage << p;}
							errorMessage << " \n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
							else
							{
								utilities::geometry::vector v1 = l1[1]-l1[0];
								utilities::geometry::vector v2 = onLineV[0]-l1[0];

								//check if the line intersects with the plane
								double r = (mNormal.dot(v2))/(mNormal.dot(v1));
								if (r > -tol && r < (1 + tol))
								{ // if it does, compute the point
									pInt = l1[0] + r * v1;
									cornerPoints.push_back(pInt);
								}
								else
								{
									markInVisualization = true;
					
									std::stringstream errorMessage;
									errorMessage << "\nError, could not find the intersection point of the \n"
											 << "perpendicular line with the oposide side line, \n"
											 << "since the vertex is not within the oposite line borders \n"
											 << "during the middle-ratio method\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
							}
						}
					} // if (method == 'M')
					
					
					// safety checks
					// Check for duplicates
					bool duplicate = false;
					for(int t=0; t<cornerPoints.size(); t++)
					{
						for(int s=0; s<cornerPoints.size(); s++)
						{
							if(t == s)
							{
								continue;
							}
							if(cornerPoints[t].isSameAs(cornerPoints[s], mGeometryModel->tolerance()))
							{								
								markInVisualization = true;
					
								std::stringstream errorMessage;
								errorMessage << "\nError, There are duplicates inside the to generate quad-hexahedron cornerPoints.\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "Is the newly generated cell a triangular prims?\n"
											 << "Research error direction segestion: In case the inserted tolerance is 0.1 mm, then the polygon intersection check can mistake a line \n"
											 << "for a polygon intersection. Therefore performing the wrong split case resulting in duplicate value's"
											 << "Solution, make tolerance smaller\n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								
								throw std::runtime_error(errorMessage.str());
							}
						}
					}
					
					
					// safety checks
					if (cornerPoints.size() < 8)
					{					
						markInVisualization = true;
					
						std::stringstream errorMessage;
						errorMessage << "\nError, There are not enough value's in cornerpoints to\n"
								 << "make a cuboids to split space during the split line precedure\n"
								 << "The cuboid to replace is: " << *this << " \n"
								 << "The intersection point in consideration is: " << onLineV[0] << " \n"
								 << "The cornerpoints for the new cuboid generation are: " << std::endl;
								 for(const auto p: cornerPoints) {errorMessage << p;}
					errorMessage << " \n"
								 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					if (cornerPoints.size() > 8)
					{
						markInVisualization = true;
					
						std::stringstream errorMessage;
						errorMessage << "\nError, There are too manny value's in cornerpoints to\n"
								 << "make a cuboids to split space during the split line precedure\n"
								 << "The cuboid to replace is: " << *this << " \n"
								 << "The intersection point in consideration is: " << onLineV[0] << " \n"
								 << "The cornerpoints for the new cuboid generation are: " << std::endl;
								 for(const auto p: cornerPoints) {errorMessage << p;}
					errorMessage << " \n"
								 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					// The new generator for the cuboids should be placed here
					newCuboids.push_back(mGeometryModel->addCuboid(
							utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance())));
				} // cornerpoint line_segment
			}
		} // split line_segment
		
		
		
		if (split)
		{
			mDeletion = true;

			for (auto& i : mCFSpaces)
			{
				//std::cout << "the following cell is substituded: " << *this << std::endl;
				i->removeCuboid(this);
				for (auto& j : newCuboids)
				{
					//std::cout << "The new cell is: " << *j << std::endl;
					
					i->addCuboid(j);
					j->addSpace(i);
				}
				checkAssociatedN(newCuboids);
			}
				
			mCFSpaces.clear();
			newCells = newCuboids;
			return split; 
		}
		else
		{
			return split;
		}
	} // splitN
	
	
	bool cf_cuboid::splitNtry2split(std::vector<cf_vertex>* intsecV, char method, std::vector<cf_cuboid*>& newCells) // step three quad-hexahedron split method
	{ // 
		//Declerations
		std::vector <bool> isInside;
		std::vector <bool> onPolygon;
		std::vector <bool> onLine;
		
		std::vector <utilities::geometry::vertex> isInsideV;
		std::vector <utilities::geometry::vertex> onPolygonV;
		std::vector <utilities::geometry::vertex> onLineV;
		
		std::vector <utilities::geometry::polygon*> intersecP;
		std::vector <utilities::geometry::line_segment> intersecL;
		
		int counter = 0;

		//Collect information for diferentiation between split methods for later use
		
		for (const auto i: *intsecV)
		{
			if (this->isInside(i, mGeometryModel->tolerance()))
			{
				isInsideV.push_back(i);
			}
			else
			{
				isInside.push_back(false);
			}
		}	
		
		bool firstVertexOnPoly = false; // only if the first vertex is onPoly then the poly-split may continue
		counter = 0;
		for (const auto i: *intsecV)
		{			
			unsigned int index = 0;
			for (const auto& j : mPolygons)
			{	
				utilities::geometry::vertex temp = j->getPointClosestTo(i);
				
				if (j->isInside(i, mGeometryModel->tolerance()) && i.isSameAs(temp, mGeometryModel->tolerance())) // temp is needed when the polygon consist of a (0,0,0) vertex, otherwise its finds a intersection when there is none
				{
					onPolygonV.push_back(i);
					intersecP.push_back(j);
					onPolygon.push_back(true);
					if (counter == 0)
					{
						firstVertexOnPoly = true;
					}
					index++;
				}
				else
				{
					onPolygon.push_back(false);
				}	
			}
			counter++;
		}	
		
		bool firstVertexOnLine = false; // only if the first vertex is onLine then the line-split may continue
		counter = 0;
		for (const auto i: *intsecV)
		{
			for (const auto& j : mLineSegments)
			{
				if (j.isOnLine(i,mGeometryModel->tolerance()))
				{
					onLineV.push_back(i);
					intersecL.push_back(j);
					onLine.push_back(true);
					if (counter == 0)
					{
						firstVertexOnLine = true;
					}
				}
				else
				{
					onLine.push_back(false);
				}
				
			}
			counter++;
		}

	
		//split methods execution
		
		// I = perpendicular to intersection side
		// O = perpendicular to oposide side
		// M = middle ratio method
		
		std::vector<cf_vertex*> newVertices;
		std::vector<cf_cuboid*> newCuboids;
		bool split = false;
		
		// Not written yet
		// split method for intersection points inside space which defide space into 8 cells
		if (isInsideV.size() >= 1)
		{ 
			//split = true;
			if (isInsideV.size() >= 2)
			{

			}
			else 
			{
				
			}
		} // split inside space			
		

		// split method for intersection point on polygon, split space into 4 "cuboids"
		if(!split)
		{ 
			if (onPolygonV.size() >= 1 && firstVertexOnPoly == true)
			{
				split = true;
				//std::cout << "Split acording to poly intersection: " << onPolygonV[0] << " with guide vertices: " ;
				//for(int j = 1; j < intsecV->size(); j++){std::cout << (*intsecV)[j];}
				//std::cout << " " << std::endl;

				// find the opposite polygon to the intersection polygon
				std::vector <bool> sameCornpoint;
				unsigned int opposite = 6;
				
								
				for (unsigned int j = 0; j < mPolygons.size(); j++)
				{
					bool sameCornp;
					
					for (auto m = (*mPolygons[j]).begin(); m != (*mPolygons[j]).end(); m++)
					{
						for (auto l = (*intersecP[0]).begin(); l != (*intersecP[0]).end(); l++)
						{
							if (m->isSameAs(*l, mGeometryModel->tolerance()))
							{
								sameCornpoint.push_back(true);
							}
							else 
							{
								sameCornpoint.push_back(false);
							}
						}
					}
			
					bool isOposite = true;
					for (int a=0; a < sameCornpoint.size(); a++)
					{
						if (sameCornpoint[a] == true)
						{
							isOposite = false;
						}
					}
					
					if (isOposite == true)
					{
						opposite = j;
					}
					sameCornpoint.clear();
				}
				
				if (opposite == 6)
				{
					markInVisualization = true;
					
					std::stringstream errorMessage;
					errorMessage << "\nError, could not find opposite surface when\n"
											 << "splitting a cuboid from a point on a surface\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
								 			 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
					throw std::runtime_error(errorMessage.str());
				}

				// Find the relevant line_segments to generate points on and store the first 2 known cornerpoints, namely the intersection point and the intersected side cornerpoint.
				for (const auto j : *(intersecP[0]))
				{
					// Declerations
					std::vector<utilities::geometry::vertex> cornerPoints;
					std::vector<utilities::geometry::line_segment> intersPoly_line;
					std::vector<utilities::geometry::line_segment> opositePoly_line;
																
					// Add first known cornerpoints
					cornerPoints.push_back(onPolygonV[0]);
					cornerPoints.push_back(j);
					
					
					
					// Find the lines within the polygon which are attached to the cornerPoints of the cuboid, and the attached line to vertex j
					for (const auto& k : mLineSegments)
					{
						for (auto l = (k).begin(); l != (k).end(); l++)
						{
													
							if(!(l->isSameAs(cornerPoints[1], mGeometryModel->tolerance()))) // (*l != cornerPoints[1])
							{ // only allow the continueation of line_segments with are attached to the relevant vertex
								continue;
							}
							
							std::vector <bool> isPartOf;
							isPartOf.clear();
							
							for (const auto& m : intersecP[0]-> getLines())
							{// Determinde if line_segments is the same of the polygon line_segments							
								if (k.isSameAs(m, mGeometryModel->tolerance()))
								{ //if so then add closest point to intersection point to cornerpoints
									intersPoly_line.push_back(k);
									isPartOf.push_back(true);
									continue;
								} 
								else
								{
									isPartOf.push_back(false);
								}
							}
							
							
							bool isPartOfPoly = false;
							for (int a=0; a < isPartOf.size(); a++)
							{ 
								if (isPartOf[a] == true)
								{
									isPartOfPoly = true;
								}
							}
							if (isPartOfPoly == true)
							{ // line_segment is attacht to vertex j, but is not in polygon
								continue;
							}
							
							
							
							// find vertix on the oposide side conected to vertex j
							
							
							// determine which of the point of non polygon line is not the same as the vertex space, so vertex of origonal cuboid on oposide side
							if(!(k[0].isSameAs(cornerPoints[1], mGeometryModel->tolerance()))) //(k[0] != cornerPoints[1])
							{
								cornerPoints.push_back(k[0]);
								for (const auto& m : mPolygons[opposite]-> getLines())
								{
									for (auto n = (m).begin(); n != (m).end(); n++)
									{
										if(!(k[0].isSameAs(*n, mGeometryModel->tolerance())))//(k[0] != *n)
										{
											continue;
										}
										opositePoly_line.push_back(m);							
									}
								}
							}
							else if(!(k[1].isSameAs(cornerPoints[1], mGeometryModel->tolerance()))) // (k[1] != cornerPoints[1])
							{
								cornerPoints.push_back(k[1]);
								
								for (const auto& m : mPolygons[opposite]-> getLines())
								{
									for (auto n = (m).begin(); n != (m).end(); n++)
									{
										if(!(k[1].isSameAs(*n, mGeometryModel->tolerance()))) //(k[1] != *n)
										{
											continue;
										}
										opositePoly_line.push_back(m);
									}
								}
							}
						}
					}
					
				

					
					// Intersection Poly
					
					//if onLineVertex are on the intersPoly_line then add the intersectionPoly_line_segments points to the cornerPoints value
					//else if there are no intersection points on the intersectionPoly_linesegments, add the perpendicular points to intersection point
					std::vector<bool> spaceInsecVInsert;
					for (int b = 0; b < intersPoly_line.size(); b++)
					{
						bool isFound = false;
						if (onPolygonV.size() > 1) // check for guidepoints on polygon and project these points to the opposide side if needed
						{
							for (int l=1; l<onPolygonV.size(); l++)
							{
								bool ProjectionIntersect = false;
								bso::utilities::geometry::vertex polyIntersectP;
								utilities::geometry::line_segment polyIntersectLine = {{cornerPoints[0]},{onPolygonV[l]}};
																
								ProjectionIntersect = polyIntersectLine.vIntersects(intersPoly_line[b],polyIntersectP);
								
								utilities::geometry::line_segment cuboidLine = {{cornerPoints[0]},{polyIntersectP}};
																
								if(ProjectionIntersect == true && intersPoly_line[b].isOnLine(polyIntersectP, mGeometryModel->tolerance()) && cuboidLine.isOnLine(onPolygonV[l], mGeometryModel->tolerance()))
								{
									cornerPoints.push_back(polyIntersectP);
									isFound = true;
									spaceInsecVInsert.push_back(true);
								}
							}
						}
						
						for (const auto l: onLineV)
						{
							if (intersPoly_line[b].isOnLine(l, mGeometryModel->tolerance()))
							{
								cornerPoints.push_back(l);
								isFound = true;
								spaceInsecVInsert.push_back(true);
								continue;
							}
						}
						if(isFound == false) //
						{
							if((intersPoly_line[b]).isOnLine((intersPoly_line[b].getPointClosestTo(onPolygonV[0])), mGeometryModel->tolerance()))
							{
								cornerPoints.push_back(intersPoly_line[b].getPointClosestTo(onPolygonV[0]));
							}
							else
							{
								markInVisualization = true;
					
								std::stringstream errorMessage;
								errorMessage << "\nError, the found intersection point is not on the considered Line of the intersection polygon \n"
										 << "during the intersected polygon split process\n"
										 << "The cuboid to replace is: " << *this << " \n"
										 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
										 << "The cornerpoints for the new cuboid generation are: " << std::endl;
										 for(const auto p: cornerPoints) {errorMessage << p;}
							errorMessage << " \n"
										 << "see section Intersection Poly of\n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
							
							spaceInsecVInsert.push_back(false);
						}								
					}

					
					//Opposide Poly, perpendicular to oposite line
					if (method == 'O')
					{
						if(mPolygons[opposite]-> isInside(mPolygons[opposite]->getPointClosestTo(onPolygonV[0]), mGeometryModel->tolerance()))
						{
							cornerPoints.push_back(mPolygons[opposite]->getPointClosestTo(onPolygonV[0])); // middle point on opposide polygon
						}
						else
						{
							markInVisualization = true;
					
							std::stringstream errorMessage;
							errorMessage << "\nError, the found intersection point is not on the considered oposite polygon \n"
									 << "The cuboid to replace is: " << *this << " \n"
									 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
									 << "The cornerpoints for the new cuboid generation are: " << std::endl;
									 for(const auto p: cornerPoints) {errorMessage << p;}
						errorMessage << " \n"
									 << "see section Opposide Poly, perpendicular to oposite line method == 'O'\n"
									 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
							throw std::runtime_error(errorMessage.str());
						}
						
						for (int b = 0; b < intersPoly_line.size(); b++)
						{							
							for(int c = 0; c < mPolygons.size(); c++)
							{
								bool isBothOnPoly1 = false;
																
								if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[0], mGeometryModel->tolerance())) 
								{
									if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[1], mGeometryModel->tolerance()))
									{
										isBothOnPoly1 = true;
									}
								}
								if (isBothOnPoly1 == false){continue;}
								
								for (int d = 0; d < opositePoly_line.size(); d++)
								{
									bool isBothOnPoly2 = false;
									if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[0], mGeometryModel->tolerance())) 
									{
										if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[1], mGeometryModel->tolerance()))
										{
											isBothOnPoly2 = true;
										}
									}
									if (isBothOnPoly2 == false){continue;}
									
									// found opposite line to intersPoly_line, therefore add the acording conerpoint (guidepoint or not):
									if (spaceInsecVInsert[b] == false) // no guidevertex to take into account
									{
										if((opositePoly_line[d]).isOnLine((opositePoly_line[d].getPointClosestTo(onPolygonV[0])), mGeometryModel->tolerance()))
										{
											cornerPoints.push_back(opositePoly_line[d].getPointClosestTo(onPolygonV[0]));
										}
										else
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, the found intersection point is not on the considered line of the oposite polygon \n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "see section Opposide Poly, perpendicular to oposite line method == 'O'/ spaceInsecVInsert[b] == false\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
									}
									if (spaceInsecVInsert[b] == true) // guidevertex is taken into account
									{
										//determine where the line intersects with the plane
										
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = mPolygons[opposite]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side \n"
														 << "during the polygon split process perpendicular to intersection\n"
														 << "The cuboid to replace is: " << *this << " \n"
														 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
														 << "The cornerpoints for the new cuboid generation are: " << std::endl;
														 for(const auto p: cornerPoints) {errorMessage << p;}
											errorMessage << " \n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
										
										
										
										/*
										if((opositePoly_line[d]).isOnLine((opositePoly_line[d].getPointClosestTo(cornerPoints[3+b])), mGeometryModel->tolerance()))
										{
											cornerPoints.push_back(opositePoly_line[d].getPointClosestTo(cornerPoints[3+b]));
											std::cout << "identified correct section" << opositePoly_line[d].getPointClosestTo(cornerPoints[3+b]) << std::endl;
										}
										else
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, the found intersection point is not on the considered line of the oposite polygon \n"
													 << "The cuboid to split is: " << *this << "\n"
													 << "see section Opposide Poly, perpendicular to oposite line method == 'O'/ spaceInsecVInsert[b] == true\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										*/
										
										
										
										
									}
								}
							}
						} //Opposide Poly, perpendicular to oposite line
					} // if (method == 'O')
					
					
					//Opposide Poly, perpendicular to intersection polygon.
					if (method == 'I')
					{
						for (int b = 0; b < intersPoly_line.size(); b++)
						{
							for(int c = 0; c < mPolygons.size(); c++)
							{
								bool isBothOnPoly1 = false;
								
								
								if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[0], mGeometryModel->tolerance())) 
								{
									if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[1], mGeometryModel->tolerance()))
									{
										isBothOnPoly1 = true;
									}
								}
								if (isBothOnPoly1 == false){continue;}
								
								for (int d = 0; d < opositePoly_line.size(); d++)
								{
									bool isBothOnPoly2 = false;
									if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[0], mGeometryModel->tolerance())) 
									{
										if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[1], mGeometryModel->tolerance()))
										{
											
											isBothOnPoly2 = true;
										}
									}
									if (isBothOnPoly2 == false){continue;}
									
									
									// found opposite line to intersPoly_line, therefore add conerpoint acordingly:
									if (spaceInsecVInsert[b] == false)
									{
										//determine where the line intersects with the plane
										// (http://geomalgorithms.com/a05-_intersect-1.html) background information intersection point line-plane
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = intersecP[0]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "see section Opposide_Poly/perpendicular to intersection polygon/(spaceInsecVInsert[b] == false)\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side during the polygon split process \n"
														 << "The cuboid to replace is: " << *this << " \n"
														 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
														 << "The cornerpoints for the new cuboid generation are: " << std::endl;
														 for(const auto p: cornerPoints) {errorMessage << p;}
											errorMessage << " \n"
														 << "see section Opposide_Poly/perpendicular to intersection polygon/(spaceInsecVInsert[b] == false)\n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
										
										
									}
									if (spaceInsecVInsert[b] == true)
									{
										//determine where the line intersects with the plane
										
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = intersecP[0]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side \n"
														 << "during the polygon split process perpendicular to intersection\n"
														 << "The cuboid to replace is: " << *this << " \n"
														 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
														 << "The cornerpoints for the new cuboid generation are: " << std::endl;
														 for(const auto p: cornerPoints) {errorMessage << p;}
											errorMessage << " \n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
									}
								}
							}
							
							
							if(b == 0) // Deze bepaling kan warschijnlijk makelijker met de functie: intersectsWith, aangezien de polygon al bestaat.
							{// intersectionpoint on oposite polygon
								utilities::geometry::vertex pInt;
								double tol = mGeometryModel->tolerance();
								
								utilities::geometry::vector normalIntsect = intersecP[0]->normal();
								utilities::geometry::vector normalOposite = mPolygons[opposite]->normal();
								
								utilities::geometry::vector v3 = cornerPoints[3] -cornerPoints[0];
								utilities::geometry::vector v4 = cornerPoints[4] -cornerPoints[0];
								
								
								
								if (normalOposite.isPerpendicular(v3, mGeometryModel->tolerance()) && normalOposite.isPerpendicular(v4, mGeometryModel->tolerance()))
								{//check if normals oposite plane is Perpendicular to the 2 vector within intersection plane
									if(mPolygons[opposite]->isInside((mPolygons[opposite]->getPointClosestTo(onPolygonV[0])), mGeometryModel->tolerance()))
									{
										cornerPoints.push_back(mPolygons[opposite]->getPointClosestTo(onPolygonV[0])); // point on opposide polygon	
									}
									else
									{
										markInVisualization = true;
					
										std::stringstream errorMessage;
										errorMessage << "\nError, could not find the intersection point on the opposite side \n"
												 << "during the polygon split process perpendicular to intersection\n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "See section polygon split/perpedicular to intersection/if(b == 0)\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}							
								}
								else
								{//determine where the line intersects with the plane
									
									utilities::geometry::line_segment l1 = {(cornerPoints[0]+normalIntsect),cornerPoints[0]};
									utilities::geometry::vector mNormal = mPolygons[opposite]->normal();
									
									
									if (l1.getVector().isPerpendicular(mNormal, tol))
									{
										markInVisualization = true;
					
										std::stringstream errorMessage;
										errorMessage << "\nError, could not find the intersection point on the opposite side \n"
												 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "See section polygon split/perpedicular to intersection/the else section of if(b == 0)\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
									else
									{									
										utilities::geometry::vector v1 = l1[1]-l1[0];
										utilities::geometry::vector v2 = ((mPolygons[opposite])->getVertices())[0]-l1[0]; 
										

										//check if the line intersects with the plane
										double r = (mNormal.dot(v2))/(mNormal.dot(v1));

										if (r > -tol /*&& r < (1 + tol)*/)
										{ // if it does, compute the point
											pInt = l1[0] + r * v1;
											cornerPoints.push_back(pInt);
										}
										else
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line does not intersect with the considered plane\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "See section polygon split/perpedicular to intersection/the else section of if(b == 0)\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
									}
								}
							}
						} //Opposide Poly, perpendicular to intersection polygon.
					} // if (method == 'I')
					
				
					// Opposide Poly, middle ratio method
					if (method == 'M')
					{
						markInVisualization = true;
					
						std::stringstream errorMessage;
							errorMessage << "\nError, the M split method polygon is not finished yet.  \n"
									 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
							throw std::runtime_error(errorMessage.str());
						
						
						// std::vector<utilities::geometry::line_segment> intersPoly_line;
						// std::vector<utilities::geometry::line_segment> opositePoly_line;
						// mPolygons[opposite];
						// *intersecP[0]
						

						/*
						virtual double baseArea = intersecP[0]->getArea()
						
						
						for(for (auto l = (intersecP[0]).begin(); l != (intersecP[0]).end(); l++))
						{
							vertixesAll.push_back(l);
						}
						
						
						// find ratio middle intersection point on opposite side
						utilities::geometry::vertex oppositePCenter = mPolygons[opposite]->getCenter();
						utilities::geometry::vertex intersectionPCenter = intersecP[0]->getCenter();
						utilities::geometry::vector oppositeNormal = mPolygons[opposite]->getNormal();
						utilities::geometry::vector intersectionNormal = intersecP[0]->getNormal();
						
						std::vector<utilities::geometry::vector> xyzUnit;
						xyzUnit.push_back({1,0,0});
						xyzUnit.push_back({0,1,0});
						xyzUnit.push_back({0,0,1});
						utilities::geometry::vector unitz = {0,0,1};
						utilities::geometry::vector unitx = {1,0,0};
						utilities::geometry::vector unity = {0,1,0};
						
						std::vector<utilities::geometry::vector> xyzo;
						xyzo.push_back(unitz.cross(oppositeNormal).normalized());
						xyzo.push_back(unitz.cross(oppositeNormal).normalized());
						xyzo.push_back(unitz.cross(oppositeNormal).normalized());
						utilities::geometry::vector xo = unitz.cross(oppositeNormal).normalized();
						utilities::geometry::vector yo = unity.cross(oppositeNormal).normalized();
						utilities::geometry::vector zo = unitx.cross(oppositeNormal).normalized();
						
						std::vector<utilities::geometry::vector> xyzi;
						utilities::geometry::vector xi = unitz.cross(intersectionNormal).normalized();
						utilities::geometry::vector yi = unitz.cross(intersectionNormal).normalized();
						utilities::geometry::vector zi = unitx.cross(intersectionNormal).normalized();
						
						utilities::geometry::vector translationVI = onPolygonV[0]-intersectionPCenter;
						
						// Add Vertex
						
						
						
						//Find area ratio's
						std::vector<double> ratio;
						std::vector<bool> usedRatio;
						// x=0
						// y=1
						// z=2
						
						for(int i = 0; i< 2; i++)
						{
							std::vector<utilities::geometry::vertex> vertixesAll;
							for(for (auto l = (intersecP[0]).begin(); l != (intersecP[0]).end(); l++))
							{
								vertixesAll.push_back(l);
							}
							
							if(xyzi[i]==xyzUnit[i])
							{
								i--;
								usedRatio.push_back(false);
								continue;
							}
							
							usedRatio.push_back(true);

						}
						
						*/
						
						
						
						//utilities::geometry::vector translationVO =  -oppositePCenter;
						
						/*
						for (int b = 0; b < intersPoly_line.size(); b++)
						{
							for(int c = 0; c < mPolygons.size(); c++)
							{
								bool isBothOnPoly1 = false;
								
								
								if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[0], mGeometryModel->tolerance())) 
								{
									if (mPolygons[c]->isCoplanarN((intersPoly_line[b])[1], mGeometryModel->tolerance()))
									{
										isBothOnPoly1 = true;
									}
								}
								if (isBothOnPoly1 == false){continue;}
								
								for (int d = 0; d < opositePoly_line.size(); d++)
								{
									bool isBothOnPoly2 = false;
									if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[0], mGeometryModel->tolerance())) 
									{
										if (mPolygons[c]->isCoplanarN((opositePoly_line[d])[1], mGeometryModel->tolerance()))
										{
											
											isBothOnPoly2 = true;
										}
									}
									if (isBothOnPoly2 == false){continue;}
									
									
									// found opposite line to intersPoly_line, therefore add conerpoint acordingly:
									if (spaceInsecVInsert[b] == false)
									{
										//determine where the line intersects with the plane
										// (http://geomalgorithms.com/a05-_intersect-1.html) background information intersection point line-plane
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = intersecP[0]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side \n"
														 << "during the polygon split process perpendicular to intersection plane\n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
										
										
									}
									if (spaceInsecVInsert[b] == true)
									{
										//determine where the line intersects with the plane
										
										// defines plane to be intersected
										utilities::geometry::vector v3 = cornerPoints[3+b]-cornerPoints[0];							
										utilities::geometry::vector v4 = intersecP[0]->normal().normalized();	
										
										//define intersection variables
										utilities::geometry::vertex pInt;
										double tol = mGeometryModel->tolerance();
										utilities::geometry::line_segment l1 = opositePoly_line[d];
										utilities::geometry::vector mNormal = v3.cross(v4).normalized();
										
										if (l1.getVector().isPerpendicular(mNormal, tol))
										{
											markInVisualization = true;
					
											std::stringstream errorMessage;
											errorMessage << "\nError, could not find the intersection point on the opposite side \n"
													 << "during the polygon split process, since the line to intersect is perpendicular to the plane\n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
										else
										{
											utilities::geometry::vector v1 = l1[1]-l1[0];
											utilities::geometry::vector v2 = cornerPoints[3+b]-l1[0];

											//check if the line intersects with the plane
											double r = (mNormal.dot(v2))/(mNormal.dot(v1));
											
											if (r > -tol && r < (1 + tol))
											{ // if it does, compute the point
												pInt = l1[0] + r * v1;
												cornerPoints.push_back(pInt);
											}
											else
											{
												markInVisualization = true;
					
												std::stringstream errorMessage;
												errorMessage << "\nError, could not find the intersection point on the opposite side \n"
														 << "during the polygon split process perpendicular to intersection plane\n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
												throw std::runtime_error(errorMessage.str());
											}
										}
										
										//std::cout << "spaceInsecVinsert[b] == true, with the following opositePoly_line[d]: " << opositePoly_line[d]<< "Whith the following normal: "<< mNormal << " add the following cornerpoints: " << pInt << std::endl;
									}
								}
							}
						
							*/
					} // if (method == 'M')
					
					
					
					// safety checks
					// Check for duplicates
					bool duplicate = false;
					for(int t=0; t<cornerPoints.size(); t++)
					{
						for(int s=0; s<cornerPoints.size(); s++)
						{
							if(t == s)
							{
								continue;
							}
							if(cornerPoints[t].isSameAs(cornerPoints[s], mGeometryModel->tolerance()))
							{
								markInVisualization = true;
					
								std::stringstream errorMessage;
								errorMessage << "\nError, There are duplicates inside the to generate quad-hexahedron cornerPoints.\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "Is the newly generated cell a triangular prims?\n"
											 << "Research error direction segestion: In case the inserted tolerance is 0.1 mm, then the polygon intersection check can mistake a line \n"
											 << "for a polygon intersection. Therefore performing the wrong split case resulting in duplicate value's"
											 << "Solution, make tolerance smaller\n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
						}
					}
					
					if (cornerPoints.size() < 8)
					{
						markInVisualization = true;
					
						std::stringstream errorMessage;
						errorMessage << "\nError, There are not enough value's in cornerpoints to\n"
								 << "generate the cuboids to split the cell during the split polygon precedure\n"
								 << "The cuboid to replace is: " << *this << " \n"
								 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
								 << "The cornerpoints for the new cuboid generation are: " << std::endl;
								 for(const auto p: cornerPoints) {errorMessage << p;}
					errorMessage << " \n"
								 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					if (cornerPoints.size() > 8)
					{
						markInVisualization = true;
					
						std::stringstream errorMessage;
						errorMessage << "\nError, There are too manny value's in cornerpoints to\n"
								 << "generate the cuboids to split the cell during the split polygon precedure\n"
								 << "The cuboid to replace is: " << *this << " \n"
								 << "The intersection point in consideration is: " << onPolygonV[0] << " \n"
								 << "The cornerpoints for the new cuboid generation are: " << std::endl;
								 for(const auto p: cornerPoints) {errorMessage << p;}
					errorMessage << " \n"
								 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					
					// The new cuboids generator
					newCuboids.push_back(mGeometryModel->addCuboid(
							utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance())));
				}
			}
		} // split polygon
		
		
		
		// split method for intersection points on line_segments which defide the cell into 2 cells
		if(!split)
		{ 
			if (onLineV.size() >= 1 && firstVertexOnLine == true)
			{
				split = true;
				//std::cout << "Split acording to line intersection: " << onLineV[0] << " with guide vertices: " ;
				//for(int j = 1; j < intsecV->size(); j++){std::cout << (*intsecV)[j];}
				//std::cout << " " << std::endl;
				
				/*
				if (onLineV.size() > 3)
				{// check if there are to manny guide intersection points
					std::stringstream errorMessage;
						errorMessage << "\nError, can't split space when there are more than 2 guide vertices \n"
												 << "There are the following amound of points intersecting: " << onLineV.size() << " \n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
				}
				*/
				
				// if check is fine, then preceed on 
								
				for (auto i = (intersecL[0]).begin(); i != (intersecL[0]).end(); i++)
				{
					// Declerations
					std::vector<utilities::geometry::vertex> cornerPoints;
					std::vector<utilities::geometry::polygon*> polyInCuboid; //PolyInCuboid are the polygons which are the polygons to the side which does not need splitting
					std::vector<utilities::geometry::line_segment> lineToSplit; //Lines which need to be split for the generation of a new cuboid
					
					// Add first known cornerPoints
					cornerPoints.push_back(onLineV[0]);
					
					// Find the PolyInCuboid geometry
					for (const auto& j : mPolygons)
					{				
						for (auto k = (*j).begin(); k != (*j).end(); k++)
						{
							if(!(i->isSameAs(*k, mGeometryModel->tolerance()))) //(*i != *k)
							{
								continue;
							}
							
							bool polyWanted = true;
							for (const auto l: j->getLines())
							{
								if((l.isSameAs(intersecL[0], mGeometryModel->tolerance()))) // (l == intersecL[0])
								{
									polyWanted = false;
									continue;
								}	
								
							}
							
							if(polyWanted == true)
							{
								polyInCuboid.push_back(j);
							}
						}
					}
					
					if (polyInCuboid.size() >= 2)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, found multiple PolyInCuboid, therefore having to manny vertexen to initiate a cuboid with \n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					// find lineToSplit
					for (auto j = (*polyInCuboid[0]).begin(); j != (*polyInCuboid[0]).end(); j++)
					{
						std::vector<utilities::geometry::line_segment> Temp;
						for (auto k : mLineSegments)
						{
							for (auto l = k.begin(); l != k.end(); l++)
							{
								if(!(l->isSameAs(*j, mGeometryModel->tolerance()))) // (*l != *j)
								{
									continue;
								}
								
								Temp.push_back(k);
							}
						}
						
						for (auto k: Temp)
						{
							bool partOfPoly = false;
							for (auto j: polyInCuboid[0]->getLines())
							{
								if(j.isSameAs(k, mGeometryModel->tolerance())) //(j == k)
								{
									partOfPoly = true;
								}
							}

							if (partOfPoly == false)
							{
								lineToSplit.push_back(k);
							}
						}
						Temp.clear();
					}
								
					
					
					// Add all the vertexes of the cuboid to variable cornerPoints
					
					// perpendicular to opposite side method
					if (method == 'O')
					{
						// add the already known in new cuboids vertexes
						for (auto j = (*polyInCuboid[0]).begin(); j != (*polyInCuboid[0]).end(); j++)
						{//polygon of the new cuboid which goes unchanged into the new cuboid
							cornerPoints.push_back(*j);
						}
						
						// determine if there is a guidance line
						utilities::geometry::vertex guidanceV; 
						utilities::geometry::line_segment guidanceL;
						bool guidanceVertex = false;
						for (auto j : lineToSplit)
						{//the new vertixen in the middle of the to split cuboids
							if (j == intersecL[0])
							{
								continue;
							}
							// check if line already has an intersection point
							for(int k=1; k < onLineV.size(); k++)
							{				
								if(j.isOnLine(onLineV[k], mGeometryModel->tolerance()))
								{
									guidanceVertex = true;
									cornerPoints.push_back(onLineV[k]);
									guidanceV = onLineV[k];
									guidanceL = j;
									continue;
								}
							}
						}
						
						// Check if anny of the (projected) guide vertices are on the polyInCuboid, since then this results in a triangular prism 
						for (const auto j: polyInCuboid[0]->getLines())
						{
							for(int k=1; k < onLineV.size(); k++)
							{				
								utilities::geometry::vertex dummy;
								utilities::geometry::line_segment guideVProject = {{onLineV[0]},{onLineV[k]}};
								
								if(j.vIntersects(guideVProject, dummy, mGeometryModel->tolerance()))
								{
									if(polyInCuboid[0]->isInsideOrOn(dummy, mGeometryModel->tolerance()))
									{
										bool isCornerpoint = false;
										for(const auto l: this->getVertices())
										{
											if(dummy.isSameAs(l,mGeometryModel->tolerance()))
											{
												isCornerpoint = true;
											}
										}
										if(!isCornerpoint)
										{										
											std::stringstream errorMessage;
											errorMessage << "\nError, there is a guide vertex on the line of the polyInCuboid of the newly to generate cell \n"
													 << "during the line split process, therefore generating a triangular prism. \n"
													 << "This is currently unsuported in the toolbox\n"
													 << "The guide vertex considered is: " << onLineV[k] << "\n"
													 << "The calculated interseciton point is: " << dummy << "\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onLineV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
									}
								}
							}
						}
						
						// find intersection point on rest of LineToSplit
						for (auto j : lineToSplit)
						{
							if (j == intersecL[0]) // no tolerance needed during this check
							{
								continue;
							}
							
							if (guidanceL == j) // no tolerance needed during this check
							{
								continue;
							}
							
							
							//define intersection variables
							utilities::geometry::vertex pInt;
							double tol = mGeometryModel->tolerance();
							utilities::geometry::line_segment l1 = j;
							utilities::geometry::vector mNormal;
							// assign mNormal
							if (guidanceVertex == true)
							{
								utilities::geometry::polygon* intersectionPolygon;
								utilities::geometry::polygon* opositePolygon;
								
								// Find intersectionPolygon with guideline on it
								for(int c = 0; c < mPolygons.size(); c++)
								{
									bool isBothOnPoly1 = false;
																	
									if (mPolygons[c]->isCoplanarN((onLineV[0]), mGeometryModel->tolerance())) 
									{
										if (mPolygons[c]->isCoplanarN(guidanceV, mGeometryModel->tolerance()))
										{
											isBothOnPoly1 = true;
										}
									}
									if (isBothOnPoly1 == false){continue;}
									
									intersectionPolygon = mPolygons[c];
									break;
								}
								
								// Find oppositePolygon to intersectionPolygon
								std::vector <bool> sameCornpoint;
								unsigned int opposite = 6;
								
								for (unsigned int j = 0; j < mPolygons.size(); j++)
								{
									bool sameCornp;
									
									for (auto m = (*mPolygons[j]).begin(); m != (*mPolygons[j]).end(); m++)
									{
										for (auto l = (*intersectionPolygon).begin(); l != (*intersectionPolygon).end(); l++)
										{
											if (*m == *l) // geen tolerantie nodig aangezien deze waardes identiek horen te zijn omdat ze van dezelfde quad-hexahedron zijn
											{
												sameCornpoint.push_back(true);
											}
											else 
											{
												sameCornpoint.push_back(false);
											}
										}
									}
							
									bool isOposite = true;
									for (int a=0; a < sameCornpoint.size(); a++)
									{
										if (sameCornpoint[a] == true)
										{
											isOposite = false;
										}
									}
									
									if (isOposite == true)
									{
										opposite = j;
										opositePolygon = mPolygons[j];
									}
									sameCornpoint.clear();
								}
								
								if (opposite == 6)
								{
									std::stringstream errorMessage;
									errorMessage << "\nError, could not find opposite surface when\n"
															 << "splitting a cuboid from a point on a surface\n"
															 << "The cuboid to replace is: " << *this << " \n"
															 << "The intersection point in consideration is: " << onLineV[0] << " \n"
															 << "The cornerpoints for the new cuboid generation are: " << std::endl;
															 for(const auto p: cornerPoints) {errorMessage << p;}
												errorMessage << " \n"
															 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
								
								// Use the normal of the oppositePolygon to define the normal of the cell plane for line intersections
								utilities::geometry::line_segment guidanceLine = {{onLineV[0]},{guidanceV}};
								utilities::geometry::vector guidanceLVector = guidanceLine.getVector();
								utilities::geometry::vector intersectionPlaneV = opositePolygon->getNormal();
								mNormal = (intersectionPlaneV.cross(guidanceLVector));	
								
								
								// compute rest of intersection points by plane intersection with the to split lines
								if (l1.getVector().isPerpendicular(mNormal, tol))
								{
									std::stringstream errorMessage;
									errorMessage << "\nError, could not find the intersection point on the opposite side \n"
											 << "during the line split process, since the line to intersect is perpendicular to the plane\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
								else
								{
									utilities::geometry::vector v1 = l1[1]-l1[0];
									utilities::geometry::vector v2 = onLineV[0]-l1[0];

									//check if the line intersects with the plane
									double r = (mNormal.dot(v2))/(mNormal.dot(v1));
									if (r > -tol && r < (1 + tol))
									{ // if it does, compute the point
										pInt = l1[0] + r * v1;
										cornerPoints.push_back(pInt);
									}
									else
									{
										std::stringstream errorMessage;
										errorMessage << "\nError, the vertex is not within the oposite lines borders \n"
												 << "during the line to split process of the perpendicular to opposite method (guidanceVertex == true)\n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
								}
							}
							else 
							{
								// find opositePolygon, there are 2 polygon opposites
								utilities::geometry::polygon* opositePolygon1;
								utilities::geometry::polygon* opositePolygon2;
								int k = 0;
								for(int c = 0; c < mPolygons.size(); c++)
								{
									bool isBothNotOnPoly1 = true;
																		
									if ((mPolygons[c]->isCoplanarN((intersecL[0])[0], mGeometryModel->tolerance()))) 
									{
										isBothNotOnPoly1 = false;
									}
									else if ((mPolygons[c]->isCoplanarN((intersecL[0])[1], mGeometryModel->tolerance())))
									{
										isBothNotOnPoly1 = false;
									}
									if (isBothNotOnPoly1 == false)
									{
										continue;
									}
									
									
									if(k == 0)
									{
										opositePolygon1 = mPolygons[c];
										k++;
									}
									else if (k == 1)
									{
										opositePolygon2 = mPolygons[c];
										k++;
									}
									else
									{
										std::stringstream errorMessage;
										errorMessage << "\nError, found to manny opposite polygons \n"
												 << "during the line split process of split to opposite \n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
								}
								
								// Use the normal of the two oppositePolygon to define the normal of the cell plane for line intersections
								utilities::geometry::vector opositePolygon2N = opositePolygon2->getNormal();
								utilities::geometry::vector opositePolygon1N = opositePolygon1->getNormal();
								mNormal = (opositePolygon1N.cross(opositePolygon2N));	
								
								
								// compute rest of intersection points by plane intersection with the to split lines
								if (l1.getVector().isPerpendicular(mNormal, tol))
								{
									std::stringstream errorMessage;
									errorMessage << "\nError, could not find the intersection point on the opposite side \n"
											 << "during the line split process, since the line to intersect is perpendicular to the plane\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
								else
								{
									utilities::geometry::vector v1 = l1[1]-l1[0];
									utilities::geometry::vector v2 = onLineV[0]-l1[0];

									//check if the line intersects with the plane
									double r = (mNormal.dot(v2))/(mNormal.dot(v1));
									//std::cout << "r is: " << r << std::endl;
									if (r > -tol && r < (1 + tol))
									{ // if it does, compute the point
										pInt = l1[0] + r * v1;
										cornerPoints.push_back(pInt);
									}
									else
									{
										std::stringstream errorMessage;
										errorMessage << "\nError, the vertex is not within the oposite lines borders \n"
												 << "during the line to split process of the perpendicular to opposite method (guidanceVertex == false)\n"
												 << "The cuboid to replace is: " << *this << " \n"
												 << "The intersection point in consideration is: " << onLineV[0] << " \n"
												 << "The cornerpoints for the new cuboid generation are: " << std::endl;
												 for(const auto p: cornerPoints) {errorMessage << p;}
									errorMessage << " \n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
								}
							}
						}
					} // if (method == 'O')
					
					
					// perpendicular to intersection side method
					if (method == 'I')
					{
						// add the already known in cornerPoints
						for (auto j = (*polyInCuboid[0]).begin(); j != (*polyInCuboid[0]).end(); j++)
						{//polygon of the new cuboid which goes unchanged into the new cuboid
							cornerPoints.push_back(*j);
						}
						
						// determine if there is a guidance line
						utilities::geometry::vertex guidanceV; 
						utilities::geometry::line_segment guidanceL;
						bool guidanceVertex = false;
						for (const auto j : lineToSplit)
						{//the new vertixen in the middle of the to split cuboids
							if (j == intersecL[0])
							{
								continue;
							}
							
							// check if line already has an intersection point
							for(int k=1; k < onLineV.size(); k++)
							{				
								if(j.isOnLine(onLineV[k], mGeometryModel->tolerance()))
								{
									guidanceVertex = true;
									cornerPoints.push_back(onLineV[k]);
									guidanceV = onLineV[k];
									guidanceL = j;
								}
							}
						}
						
						// Check if anny of the (projected) guide vertices are on the polyInCuboid, since then this results in a triangular prism 
						for (const auto j: polyInCuboid[0]->getLines())
						{
							for(int k=1; k < onLineV.size(); k++)
							{				
								utilities::geometry::vertex dummy;
								utilities::geometry::line_segment guideVProject = {{onLineV[0]},{onLineV[k]}};
								
								if(j.vIntersects(guideVProject, dummy, mGeometryModel->tolerance()))
								{
									if(polyInCuboid[0]->isInsideOrOn(dummy, mGeometryModel->tolerance()))
									{
										bool isCornerpoint = false;
										for(const auto l: this->getVertices())
										{
											if(dummy.isSameAs(l,mGeometryModel->tolerance()))
											{
												isCornerpoint = true;
											}
										}
										if(!isCornerpoint)
										{
											std::stringstream errorMessage;
											errorMessage << "\nError, there is a guide vertex on the line of the polyInCuboid of the newly to generate cell \n"
													 << "during the line split process, therefore generating a triangular prism. \n"
													 << "This is currently unsuported in the toolbox\n"
													 << "The guide vertex considered is: " << onLineV[k] << "\n"
													 << "The calculated interseciton point is: " << dummy << "\n"
													 << "The cuboid to replace is: " << *this << " \n"
													 << "The intersection point in consideration is: " << onLineV[0] << " \n"
													 << "The cornerpoints for the new cuboid generation are: " << std::endl;
													 for(const auto p: cornerPoints) {errorMessage << p;}
										errorMessage << " \n"
													 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
											throw std::runtime_error(errorMessage.str());
										}
									}
								}
							}
						}
						
						// find intersection point on rest of LineToSplit
						for (auto j : lineToSplit)
						{
							if (j == intersecL[0])// no tolerance needed during this check
							{
								continue;
							}
							
							if (guidanceL == j)// no tolerance needed during this check
							{
								continue;
							}
							
							
							//define intersection variables
							utilities::geometry::vertex pInt;
							double tol = mGeometryModel->tolerance();
							utilities::geometry::line_segment l1 = j;
							utilities::geometry::vector mNormal;
							// assign mNormal
							if (guidanceVertex == true)
							{
								utilities::geometry::line_segment guidanceLine = {{onLineV[0]},{guidanceV}};
								utilities::geometry::vector intersecLVector = (intersecL[0]).getVector();
								utilities::geometry::vector guidanceLVector = guidanceLine.getVector();

								utilities::geometry::vector intersectionPlaneV = (intersecLVector.cross(guidanceLVector)).normalized();
								mNormal = (intersectionPlaneV.cross(guidanceLVector));								
							}
							else 
							{
								mNormal = (intersecL[0]).getVector();
							}

							// compute rest of intersection points
							
							
							if (l1.getVector().isPerpendicular(mNormal, tol))
							{
								std::stringstream errorMessage;
								errorMessage << "\nError, could not find the intersection point on the opposite side \n"
										 << "during the line split process, since the line to intersect is perpendicular to the plane\n"
										 << "The cuboid to replace is: " << *this << " \n"
										 << "The intersection point in consideration is: " << onLineV[0] << " \n"
										 << "The cornerpoints for the new cuboid generation are: " << std::endl;
										 for(const auto p: cornerPoints) {errorMessage << p;}
							errorMessage << " \n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
							else
							{
								utilities::geometry::vector v1 = l1[1]-l1[0];
								utilities::geometry::vector v2 = onLineV[0]-l1[0];

								//check if the line intersects with the plane
								double r = (mNormal.dot(v2))/(mNormal.dot(v1));
								if (r > -tol && r < (1 + tol))
								{ // if it does, compute the point
									pInt = l1[0] + r * v1;
									cornerPoints.push_back(pInt);
								}
								else
								{
									std::stringstream errorMessage;
									errorMessage << "\nError, the vertex is not within the oposite lines borders \n"
											 << "during the line to split process of the perpendicular to intersected method\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
							}
						}
					} // if (method == 'I')
					
					
					// not fully finished yet
					// Ratio method
					if (method == 'M')
					{
						// add the already known in new cuboids vertexes
						for (auto j = (*polyInCuboid[0]).begin(); j != (*polyInCuboid[0]).end(); j++)
						{//polygon of the new cuboid which goes unchanged into the new cuboid
							cornerPoints.push_back(*j);
						}
						
						// determine if there is a guidance line
						utilities::geometry::vertex guidanceV; 
						utilities::geometry::line_segment guidanceL;
						bool guidanceVertex = false;
						for (auto j : lineToSplit)
						{//the new vertixen in the middle of the to split cuboids
							if (j == intersecL[0])
							{
								continue;
							}
							// check if line already has an intersection point
							for(int k=1; k < onLineV.size(); k++)
							{				
								if(j.isOnLine(onLineV[k]))
								{
									guidanceVertex = true;
									cornerPoints.push_back(onLineV[k]);
									guidanceV = onLineV[k];
									guidanceL = j;
									continue;
								}
							}
						}
						
						// Define the middle ratio point on the shortest line so that through the intersection plane the second one can be found
						utilities::geometry::line_segment lToAccound;
						double lengthlToAccound = 0;
						for (auto j : lineToSplit)
						{
							if (j == intersecL[0])
							{
								continue;
							}
							
							if (guidanceL == j)
							{
								continue;
							}
							
							if(j.getLength() > lengthlToAccound)
							{
								lToAccound = j;
								lengthlToAccound = j.getLength();
							}
						}
						
						// find intersection point on rest of LineToSplit
						for (auto j : lineToSplit)
						{
							if (j == intersecL[0])
							{
								continue;
							}
							
							if (guidanceL == j)
							{
								continue;
							}
							
							
							//define intersection variables
							utilities::geometry::vertex pInt;
							double tol = mGeometryModel->tolerance();
							utilities::geometry::line_segment l1 = j;
							utilities::geometry::vector mNormal;
							
							// assign mNormal
							if (guidanceVertex == true)
							{
								utilities::geometry::line_segment guidanceLine = {{onLineV[0]},{guidanceV}};
								utilities::geometry::vector guidanceLVector = guidanceLine.getVector();
								
								// define intersectionPlaneV
								utilities::geometry::vector IntersecVector = intersecL[0].getVector();
								utilities::geometry::line_segment partLine1 = {{(intersecL[0])[0]},{cornerPoints[0]}};
								
								double IntersecLength = (intersecL[0]).getLength();
								double IntersecLength1 = partLine1.getLength();
								double ratio = IntersecLength1/IntersecLength;

								utilities::geometry::vector interlineVector = lToAccound[1]-lToAccound[0];
								utilities::geometry::vertex jIntersection = (lToAccound[0]+interlineVector*ratio);
								utilities::geometry::vector intersectionPlaneV = jIntersection-cornerPoints[0];
								intersectionPlaneV.normalized();

								mNormal = (intersectionPlaneV.cross(guidanceLVector));								
							}
							else 
							{
								// mNormal = (intersecL[0]).getVector();
							}

							
							// compute rest of intersection points
							
							
							if (l1.getVector().isPerpendicular(mNormal, tol))
							{
								std::stringstream errorMessage;
								errorMessage << "\nError, could not find the intersection point on the opposite side \n"
										 << "during the line split process, since the line to intersect is perpendicular to the plane\n"
										 << "The cuboid to replace is: " << *this << " \n"
										 << "The intersection point in consideration is: " << onLineV[0] << " \n"
										 << "The cornerpoints for the new cuboid generation are: " << std::endl;
										 for(const auto p: cornerPoints) {errorMessage << p;}
							errorMessage << " \n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
							else
							{
								utilities::geometry::vector v1 = l1[1]-l1[0];
								utilities::geometry::vector v2 = onLineV[0]-l1[0];

								//check if the line intersects with the plane
								double r = (mNormal.dot(v2))/(mNormal.dot(v1));
								if (r > -tol && r < (1 + tol))
								{ // if it does, compute the point
									pInt = l1[0] + r * v1;
									cornerPoints.push_back(pInt);
								}
								else
								{
									std::stringstream errorMessage;
									errorMessage << "\nError, could not find the intersection point of the \n"
											 << "perpendicular line with the oposide side line, \n"
											 << "since the vertex is not within the oposite line borders \n"
											 << "during the middle-ratio method\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
									throw std::runtime_error(errorMessage.str());
								}
							}
						}
					} // if (method == 'M')
					
					
					// safety checks
					// Check for duplicates
					bool duplicate = false;
					for(int t=0; t<cornerPoints.size(); t++)
					{
						for(int s=0; s<cornerPoints.size(); s++)
						{
							if(t == s)
							{
								continue;
							}
							if(cornerPoints[t].isSameAs(cornerPoints[s], mGeometryModel->tolerance()))
							{								
								std::stringstream errorMessage;
								errorMessage << "\nError, There are duplicates inside the to generate quad-hexahedron cornerPoints.\n"
											 << "The cuboid to replace is: " << *this << " \n"
											 << "The intersection point in consideration is: " << onLineV[0] << " \n"
											 << "The cornerpoints for the new cuboid generation are: " << std::endl;
											 for(const auto p: cornerPoints) {errorMessage << p;}
								errorMessage << " \n"
											 << "Is the newly generated cell a triangular prims?\n"
											 << "Research error direction segestion: In case the inserted tolerance is 0.1 mm, then the polygon intersection check can mistake a line \n"
											 << "for a polygon intersection. Therefore performing the wrong split case resulting in duplicate value's"
											 << "Solution, make tolerance smaller\n"
										 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								
								throw std::runtime_error(errorMessage.str());
							}
						}
					}
					
					
					// safety checks
					if (cornerPoints.size() < 8)
					{					
						std::stringstream errorMessage;
						errorMessage << "\nError, There are not enough value's in cornerpoints to\n"
								 << "make a cuboids to split space during the split line precedure\n"
								 << "The cuboid to replace is: " << *this << " \n"
								 << "The intersection point in consideration is: " << onLineV[0] << " \n"
								 << "The cornerpoints for the new cuboid generation are: " << std::endl;
								 for(const auto p: cornerPoints) {errorMessage << p;}
					errorMessage << " \n"
								 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					if (cornerPoints.size() > 8)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, There are too manny value's in cornerpoints to\n"
								 << "make a cuboids to split space during the split line precedure\n"
								 << "The cuboid to replace is: " << *this << " \n"
								 << "The intersection point in consideration is: " << onLineV[0] << " \n"
								 << "The cornerpoints for the new cuboid generation are: " << std::endl;
								 for(const auto p: cornerPoints) {errorMessage << p;}
					errorMessage << " \n"
								 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					// The new generator for the cuboids should be placed here
					newCuboids.push_back(mGeometryModel->addCuboid(
							utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance())));
				} // cornerpoint line_segment
			}
		} // split line_segment
		

		if (split)
		{
			newCells = newCuboids;
			return split; 
		}
		else
		{
			return split;
		}
	} // splitNtry2split
	
	bool cf_cuboid::splitN(std::vector<cf_vertex>* intsecV, char method) // step three quad-hexahedron split method
	{
		std::vector<cf_cuboid*> dummy;
		return this->splitN(intsecV, method, dummy);
	}
	
	bool cf_cuboid::splitNtry(std::vector<cf_vertex>* intsecV, char method) // step three quad-hexahedron split method
	{
		// sub-method diferentiation_ 1; try and catch methods
		std::vector<cf_cuboid*> dummy;
		bool splitN = false;
		try
		{
			if(this->splitN(intsecV, 'O', dummy))
			{
				splitN = true;
			}
		}
		catch(std::exception& e)
		{
			try
			{
				if(this->splitN(intsecV, 'I', dummy))
				{
					splitN = true;
				}
			}
			catch(std::exception& e)
			{
				try
				{
					if(this->splitN(intsecV, 'M', dummy))
					{
						splitN = true;
					}
				}
				catch(std::exception& e)
				{
					markInVisualization = true;
					
					std::stringstream errorMessage;
						errorMessage << "\nError, could not split the cell since all split cell methods were unable to split the cell \n"
									 << e.what() << "\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
				}
			}
		}
		return splitN;
	}
	
	bool cf_cuboid::splitNtry(std::vector<cf_vertex>* intsecV, char method, std::vector<cf_cuboid*>& newCells)
	{
		bool splitN = false;
		try
		{
			if(this->splitN(intsecV, 'O', newCells))
			{
				splitN = true;
			}
		}
		catch(std::exception& e)
		{
			try
			{
				if(this->splitN(intsecV, 'I', newCells))
				{
					splitN = true;
				}
			}
			catch(std::exception& e)
			{
				try
				{
					if(this->splitN(intsecV, 'M', newCells))
					{
						splitN = true;
					}
				}
				catch(std::exception& e)
				{
					markInVisualization = true;
					
					std::stringstream errorMessage;
						errorMessage << "\nError, could not split the cell since all split cell methods were unable to split the cell \n"
									 << e.what() << "\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
				}
			}
		}
		return splitN;
	}
	
	bool cf_cuboid::splitNtry2(std::vector<cf_vertex>* intsecV, char method) // step three quad-hexahedron split method
	{
		// sub-method diferentiation_ 1; try and catch methods
		std::vector<cf_cuboid*> dummy;
		return this->splitNtry2(intsecV, method, dummy);
	} //splitNtry2
	
	bool cf_cuboid::splitNtry2(std::vector<cf_vertex>* intsecV, char method, std::vector<cf_cuboid*>& newCells)
	{
		bool splitN = false;
		
		std::vector<cf_cuboid*> methodI;
		std::vector<cf_cuboid*> methodO;
		std::vector<cf_cuboid*> methodM;
		
		//get all posible newCells configurations
		try
		{
			if(this->splitNtry2split(intsecV, 'I', methodI))
			{
				splitN = true;
			}
		}
		catch(std::exception& e)
		{
			methodI.clear();
		}
		
		try
		{
			if(this->splitNtry2split(intsecV, 'O', methodO))
			{
				splitN = true;
			}
		}
		catch(std::exception& e)
		{
			methodO.clear();
		}
		
		try
		{
			if(this->splitNtry2split(intsecV, 'M', methodM))
			{
				splitN = true;
			}
		}
		catch(std::exception& e)
		{
			methodM.clear();
		}
		
		
		//Determine which is the bestaat
		double biggestVolumeDifI = 0;
		double biggestVolumeDifO = 0;
		double biggestVolumeDifM = 0;
		
		for(const auto i: methodI)
		{
			for(const auto j: methodI)
			{
				if(i == j){continue;}
				if(biggestVolumeDifI < i->getVolume()-j->getVolume())
				{
					biggestVolumeDifI = i->getVolume()-j->getVolume();
				}
				else if(biggestVolumeDifI < j->getVolume()-i->getVolume())
				{
					biggestVolumeDifI = j->getVolume()-i->getVolume();
				}
			}
		}
		for(const auto i: methodO)
		{
			for(const auto j: methodO)
			{
				if(i == j){continue;}
				if(biggestVolumeDifO < i->getVolume()-j->getVolume())
				{
					biggestVolumeDifO = i->getVolume()-j->getVolume();
				}
				else if(biggestVolumeDifO < j->getVolume()-i->getVolume())
				{
					biggestVolumeDifO = j->getVolume()-i->getVolume();
				}
			}
		}
		for(const auto i: methodM)
		{
			for(const auto j: methodM)
			{
				if(i == j){continue;}
				if(biggestVolumeDifM < i->getVolume()-j->getVolume())
				{
					biggestVolumeDifM = i->getVolume()-j->getVolume();
				}
				else if(biggestVolumeDifM < j->getVolume()-i->getVolume())
				{
					biggestVolumeDifM = j->getVolume()-i->getVolume();
				}
			}
		}
		
		
		std::vector<cf_cuboid*> newCuboids;
		if(biggestVolumeDifI < biggestVolumeDifO && biggestVolumeDifI < biggestVolumeDifM)
		{
			newCuboids = methodI;
		}
		else if(biggestVolumeDifO < biggestVolumeDifI && biggestVolumeDifO < biggestVolumeDifM)
		{
			newCuboids = methodO;
		}
		else if(biggestVolumeDifM < biggestVolumeDifI && biggestVolumeDifM < biggestVolumeDifO)
		{
			newCuboids = methodM;
		}
		
		
		if(splitN)
		{
			mDeletion = true;

			for (auto& i : mCFSpaces)
			{
				//std::cout << "the following cell is substituded: " << *this << std::endl;
				i->removeCuboid(this);
				for (auto& j : newCuboids)
				{
					//std::cout << "The new cell is: " << *j << std::endl;
					
					i->addCuboid(j);
					j->addSpace(i);
				}
				checkAssociatedN(newCuboids);
			}
				
			mCFSpaces.clear();
			newCells = newCuboids;
			return splitN;
		}
		else
		{
			return splitN;
		}
	} //splitNtry2	
	
	
	void cf_cuboid::split(cf_vertex* pPtr)
	{ // 
		for (auto& i : mCFVertices)
		{
			if (pPtr == i) return;
		}

		std::vector<cf_vertex*> newVertices;
		std::vector<cf_cuboid*> newCuboids;
		bool split = false;
		
		std::vector<utilities::geometry::vector> normals; // the three normals of the cuboid
					
		for (auto& i : mPolygons)
		{
			bool normalFound = false;
			for (const auto& j : normals)
			{
				if (i->normal().isParallel(j, mGeometryModel->tolerance())) normalFound = true;
			}
			if (!normalFound) normals.push_back(i->normal().normalized());
			if (normals.size() == 3) break;
		}
		if (normals.size() != 3) 
		{
			std::stringstream errorMessage;
			errorMessage << "\nError, expected to find 3 normals of a cuboid\n"
									 << "found: " << normals.size() << ".\n"
									 << "(bso/spatial_design/conformal/cf_cuboid.cpp)" << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		
		if (this->isInside(*pPtr, mGeometryModel->tolerance()))
		{
			//std::cout << "The cuboid split section has been used" << std::endl;
			
			split = true;
			newVertices.push_back(pPtr);
			
			for (const auto& i : mPolygons)
			{ // find the point closest on each surface
				newVertices.push_back(mGeometryModel->addVertex(i->getPointClosestTo(*pPtr)));
				if (!(i->isInside(*(newVertices.back()), mGeometryModel->tolerance())))
				{
					std::stringstream errorMessage;
					errorMessage << "\nError, found a point closest to a rectangle,\n"
											 << "but that point is not inside the rectangle.\n"
											 << "While splitting a cuboid at a point inside that cuboid.\n"
											 << "Rectangle: " << *i << "\n"
											 << "Split point: " << pPtr->transpose() << "\n"
											 << "Found point: " << newVertices.back()->transpose() << "\n"
											 << "(bso/spatial_design/conformal/cf_cuboid.cpp)" << std::endl;
					throw std::runtime_error(errorMessage.str());
				}
			}
			
			for (const auto& i : mLineSegments)
			{ // ifnd the point closest on each line
				newVertices.push_back(mGeometryModel->addVertex(i.getPointClosestTo(*pPtr)));
				if (!(i.isOnLine(*(newVertices.back()), mGeometryModel->tolerance())))
				{
					std::stringstream errorMessage;
					errorMessage << "\nError, found a point closest to a line segment,\n"
											 << "but that point is not on the line segment.\n"
											 << "While splitting a cuboid at a point inside that cuboid.\n"
											 << "Line: " << i  << "\n"
											 << "Split point: " << pPtr->transpose() << "\n"
											 << "Found point: " << newVertices.back()->transpose() << "\n"
											 << "(bso/spatial_design/conformal/cf_cuboid.cpp)" << std::endl;
					throw std::runtime_error(errorMessage.str());
				}
			}
			
			for (const auto& i: mVertices)
			{
				std::vector<utilities::geometry::vertex> cornerPoints;
				cornerPoints.push_back(*pPtr);
				cornerPoints.push_back(i);
				
				for (const auto& j : normals)
				{
					double projectedDistance = j.dot(cornerPoints[1] - cornerPoints[0]);
					cornerPoints.push_back(cornerPoints[0] + projectedDistance * j);
					cornerPoints.push_back(cornerPoints[1] - projectedDistance * j);
				}
					
				newCuboids.push_back(mGeometryModel->addCuboid(
					utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance())));
			}
		}
		
		if (!split)
		{
			for (unsigned int i = 0; i < 6; ++i)
			{ // check if the point is on any of the rectangles of this cuboid
								
				if (mPolygons[i]->isInside(*pPtr, mGeometryModel->tolerance()))
				{
				 // if it is, lets split the cuboid into four new ones
					
					split = true;
					newVertices.push_back(pPtr);
					unsigned int opposite = 6;
					// first find the rectangle opposite to rectangle i
					for (unsigned int j = 0; j < 6; ++j)
					{
						if (j != i && mPolygons[i]->normal().isParallel(mPolygons[j]->normal(), mGeometryModel->tolerance()))
						{
							opposite = j;
							break;
						}
					}
					if (opposite == 6)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, could not find opposite surface when\n"
												 << "splitting a cuboid from a point on a surface\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					newVertices.push_back(mGeometryModel->addVertex(mPolygons[opposite]->getPointClosestTo(*pPtr)));
					
					if (!(mPolygons[opposite]->isInside(*(newVertices.back())), mGeometryModel->tolerance()))
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, found a point closest to a surface,\n"
												 << "but that point is not inside the surface.\n"
												 << "When splitting cuboid at surface.\n"
												 << "Surface:\n" << *mPolygons[opposite] << "\n"
												 << "Split point: " << pPtr->transpose() << "\n"
												 << "Found point: " << (newVertices.back())->transpose() << "\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					unsigned int splitSurfaces[2] = {i,opposite};
					
					// next find the vertices on each of the lines of the surface
					for (unsigned int j = 0; j < 2; ++j)
					{
						for (const auto& k : mPolygons[splitSurfaces[j]]->getLines())
						{
							newVertices.push_back(mGeometryModel->addVertex(k.getPointClosestTo(*(newVertices[j]))));
							
							if (!k.isOnLine(*(newVertices.back()), mGeometryModel->tolerance()))
							{
								std::stringstream errorMessage;
								errorMessage << "\nError, found a point closest to a line segment,\n"
														 << "but that point is not on the line segment.\n"
														 << "When splitting cuboid at surface.\n"
														 << "Surface:\n" << *mPolygons[splitSurfaces[j]] << "\n"
														 << "Line:\n" << k << "\n"
														 << "Split point: " << pPtr->transpose() << "\n"
														 << "Found point: " << newVertices.back()->transpose() << "\n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
							throw std::runtime_error(errorMessage.str());
							}
						}
					}
					
					for (const auto j : *(mPolygons[opposite]))
					{
						std::vector<utilities::geometry::vertex> cornerPoints;
						cornerPoints.push_back(*pPtr);
						cornerPoints.push_back(j);
						for (const auto& k : normals)
						{
							double projectedDistance = k.dot(cornerPoints[1] - cornerPoints[0]);
							cornerPoints.push_back(cornerPoints[0] + projectedDistance * k);
							cornerPoints.push_back(cornerPoints[1] - projectedDistance * k);
						}
						newCuboids.push_back(mGeometryModel->addCuboid(
							utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance())));
					}
					break;
				}
			}
		}
		
		if (!split)
		{		
			for (unsigned int i = 0; i < 12; ++i)
			{
				if (mLineSegments[i].isOnLine(*pPtr,mGeometryModel->tolerance()))
				{
					split = true;
					newVertices.push_back(pPtr);
					utilities::geometry::vector v1 = mLineSegments[i].getVector();
					
					for (unsigned int j = 0; j < 12; ++j)
					{
						if (j == i) continue;
						
						if (mLineSegments[j].getVector().isParallel(v1, mGeometryModel->tolerance()))
						{
							newVertices.push_back(mGeometryModel->addVertex(mLineSegments[j].getPointClosestTo(*pPtr)));
							if (!mLineSegments[j].isOnLine(*(newVertices.back()), mGeometryModel->tolerance()))
							{
								std::stringstream errorMessage;
								errorMessage << "\nError, found a point closest to a line segment,\n"
														 << "but that point is not on the line segment.\n"
														 << "When splitting cuboid at a line segment.\n"
														 << "Line segment: " << mLineSegments[j] << "\n"
														 << "Split point: " << pPtr->transpose() << "\n"
														 << "Found point: " << newVertices.back()->transpose() << "\n"
														 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
								throw std::runtime_error(errorMessage.str());
							}
						}
					}
					
					if (newVertices.size() != 4)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, expected to find 4 new vertices,\n"
												 << "when dividing a cuboid at a line segment.\n"
												 << "Found: " << newVertices.size() << "\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					for (const auto& j : mPolygons)
					{
						if (v1.isParallel(j->normal(), mGeometryModel->tolerance()))
						{
							std::vector<utilities::geometry::vertex> cornerPoints;
							for (unsigned int k = 0; k < 4; ++k)
							{
								cornerPoints.push_back(*newVertices[k]);
							}
							cornerPoints.insert(cornerPoints.end(),j->begin(), j->end());
							
							newCuboids.push_back(mGeometryModel->addCuboid(
								utilities::geometry::quad_hexahedron(cornerPoints, mGeometryModel->tolerance())));
						}
					}
					if (newCuboids.size() != 2)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, expected to find 2 new cuboids,\n"
												 << "when dividing a cuboid at a line segment.\n"
												 << "Found: " << newCuboids.size() << "\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					break;
				}
			}
		}

		if (split)
		{
			mDeletion = true;
			for (auto& i : mCFSpaces)
			{
				i->removeCuboid(this);
				for (auto& j : newCuboids)
				{
					i->addCuboid(j);
					j->addSpace(i);
				}
			}
			

			for (const auto& i : newVertices) this->checkAssociated(i);
			mCFSpaces.clear();
		}
	} // split


	
	
	
	
	void cf_cuboid::checkAssociated(cf_vertex* pPtr)
	{ // 
		for (auto& i : mCFSpaces)
		{
			i->checkVertex(pPtr);
			for (auto& j : i->cfEdges())
			{
				j->checkVertex(pPtr);
			}
			for (auto& j : i->cfSurfaces())
			{
				j->checkVertex(pPtr);
			}
		}
	} // checkAssociated
	
	
	void cf_cuboid::checkAssociatedN(std::vector<cf_cuboid*> newCuboids)
	{
		// insert the found polygons in the regtangular split function to split the origonal polygons
		for (const auto& j : cfRectangles()) // origonal polygons
		{
			std::vector<cf_rectangle*> newRect;

			
			for (auto i : newCuboids)
			{
				std::vector <bool> origonalPolycheck;
				
				for (auto m : (i->cfRectangles())) // new polygons of new cuboids
				{				
					for (auto l = m->begin(); l != m->end(); l++) // vertixes of new polygons
					{
						
						if((j->isInsideOrOn(*l, mGeometryModel->tolerance()))) // isInsideOrOn does not reconize a common vertex {0,0,0}
						{
							origonalPolycheck.push_back(true);
						}
						
						else
						{
							for (auto n = j->begin(); n != j->end(); n++)
							{
								if(l->isSameAs(*n, mGeometryModel->tolerance()))
								{
									origonalPolycheck.push_back(true);
								}
							}
							
						}
					}
					
					if(origonalPolycheck.size() == 4) // all vertixes of new polygons are on the orgigonal polygon
					{
						newRect.push_back(m);
					}
					else if (origonalPolycheck.size() > 4)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, something went wrong when searching for the new polygons within the old one,\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					origonalPolycheck.clear();
				}
			}
			
			// Insert new polygons to rectangular split function
			if(newRect.size() > 1)
			{
				j -> splitN(newRect);
			}
		}
		
		
		
		// insert the found line segments in the line split function to split the origonal lines
		for (const auto& j : cfLines()) // origonal line segments
		{
			std::vector<cf_line*> newLine;			
			
			for (auto i : newCuboids)
			{
				std::vector <bool> origonalLinecheck;
							
				for (auto m : (i->cfLines())) // lines of the new cuboids
				{
					for (auto n = m->begin(); n != m->end(); n++)
					{
						if(j->isOnLine(*n, mGeometryModel->tolerance()))
						{
							origonalLinecheck.push_back(true);
						}
						else
						{
							for (auto k = j->begin(); k != j->end(); k++)
							{
								if(k->isSameAs(*n, mGeometryModel->tolerance()))
								{
									origonalLinecheck.push_back(true);
								}
							}

						}
					}
					
					if(origonalLinecheck.size() == 2) // all vertixes of new polygons are on the orgigonal polygon
					{
						newLine.push_back(m);
					}
					else if (origonalLinecheck.size() > 2)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, something went wrong when searching for the new line within the old one,\n"
												  << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					origonalLinecheck.clear();
				}	
			}
			
			
			// Insert new lines to line split function
			if(newLine.size() > 1)
			{
				j -> splitN(newLine);
			}
		}
		
		
	}//checkAssociatedN

} // conformal
} // spatial_design
} // bso

#endif // CF_CUBOID_CPP