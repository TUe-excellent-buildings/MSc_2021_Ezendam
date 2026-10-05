#ifndef CF_POLYGON_CPP
#define CF_POLYGON_CPP

namespace bso { namespace spatial_design { namespace conformal {
	
	void cf_polygon::sortPoints(const double& tol /*= 1e-3*/)
	{ // 
		/*
		//Sort points
		std::vector<vertex> sortV;
		if (mVertices.size() < 4)	{sortV = mVertices;}
		else {sortV = bso::utilities::sortClockwise(intsecV);}
		*/
	
		/*
		line_segment l01 = {mVertices[1], mVertices[0]};
		line_segment l23 = {mVertices[3], mVertices[2]};
		line_segment l03 = {mVertices[3], mVertices[0]};
		line_segment l12 = {mVertices[2], mVertices[1]};
		
		if (l01.intersectsWith(l23,tol))
		{
			std::iter_swap(mVertices.begin() + 1, mVertices.begin() + 2);
		}
		else if (l03.intersectsWith(l12, tol))
		{
			std::iter_swap(mVertices.begin() + 2, mVertices.begin() + 3);
		}
		
		line_segment l02 = {mVertices[2], mVertices[0]};
		line_segment l13 = {mVertices[3], mVertices[1]};
		
		if (!l02.intersectsWith(l13, tol))
		{
			std::stringstream errorMessage;
			errorMessage << "When sorting the vertices of a quadrilateral,\n"
									 << "the quadrilateral is not convex.\n"
									 << "\n(bso/utilities/geometry/quadrilateral.cpp)" << std::endl;
			throw std::invalid_argument(errorMessage.str());
		}
		*/
		
		polygon::sortPoints(tol);
	} //  sortPoints()
	
	cf_polygon::cf_polygon(const utilities::geometry::polygon& rhs, cf_geometry_model* geometryModel)
	: utilities::geometry::polygon(rhs, geometryModel->tolerance())
	{
		mGeometryModel = geometryModel;
		for (const auto& i : mVertices)
		{
			mCFVertices.push_back(mGeometryModel->addVertex(i));
			mCFVertices.back()->addPolygon(this);
		}
		for (const auto& i : mLineSegments)
		{
			mCFLines.push_back(mGeometryModel->addLine(i));
			mCFLines.back()->addPolygon(this);
		}
		
		try 
		{
			this->sortPoints(geometryModel->tolerance());
		}
		catch (std::exception& e)
		{
			std::stringstream errorMessage;
			errorMessage << "Could not sort points when initializing polygon from container.\n"
									 << "Received the following error: \n"
									 << e.what()
									 << "\n(bso/spatial_design/conformal.cpp)" << std::endl;
			throw std::invalid_argument(errorMessage.str());
		}
	} // ctor
	
	cf_polygon::~cf_polygon()
	{
		for (const auto& i : mCFVertices) i->removePolygon(this);
		for (const auto& i : mCFLines) i->removePolygon(this);
	} // dtor
	
	
	void cf_polygon::split(std::vector<cf_polygon*> newpol)
	{
		/*
		//std::cout << "regtangular splitN function started: " << std::endl;
		
		//Declarations
		// std::vector<cf_vertex*> newVertices;
		std::vector<cf_rectangle*> newRectangles;
		bool split = false;
		
		//Definition of new rectangles
		int k = 0;
		for (const auto i : newRect)
		{
			split = true;
			std::vector<utilities::geometry::vertex> cornerVertices;
			for(auto j = i->begin(); j != i->end(); j++)
			{
				cornerVertices.push_back(*j);
			}
			
			newRectangles.push_back(mGeometryModel->addRectangle(
					utilities::geometry::quadrilateral(cornerVertices, mGeometryModel->tolerance())));
			//std::cout << "The rectangle added is: " << *newRectangles[k] << std::endl;
			k++;
		}
		
		//Removing origonal rectangle and replace it with new rectangles
		if (split)
		{
			mDeletion = true;
			for (auto& i : mCFSurfaces)
			{
				i->removeRectangle(this);
				for (const auto& j : newRectangles)
				{
					i->addRectangle(j);
					j->addSurface(i);
				}
			}
			
			// Sjonnies additional code for error visual fix
			//for (auto& i : mCFVertices) i->removeRectangle(this);
			//for (auto& i : mCFLines) i->removeRectangle(this);
			
			//for (const auto& i : newVertices) this->checkAssociated(i);	
			mCFSurfaces.clear();
		}
		*/
	} //splitN
	
	void split(cf_vertex* pPtr)
	{
		
	}
	
	void checkAssociated(cf_vertex* pPtr)
	{
		
	}
	
	
} // conformal
} // spatial_design
} // bso

#endif // CF_POLYGON_CPP