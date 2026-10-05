#ifndef CF_TRIANGLE_CPP
#define CF_TRIANGLE_CPP

namespace bso { namespace spatial_design { namespace conformal {
	
	cf_triangle::cf_triangle(const utilities::geometry::triangle& rhs, cf_geometry_model* geometryModel)
	: utilities::geometry::triangle(rhs, geometryModel->tolerance())
	{
		mGeometryModel = geometryModel;
		for (const auto& i : mVertices)
		{
			mCFVertices.push_back(mGeometryModel->addVertex(i));
			mCFVertices.back()->addTriangle(this);
		}
		for (const auto& i : mLineSegments)
		{
			mCFLines.push_back(mGeometryModel->addLine(i));
			mCFLines.back()->addTriangle(this);
		}
	} // ctor
	
	cf_triangle::~cf_triangle()
	{
		for (const auto& i : mCFVertices) i->removeTriangle(this);
		for (const auto& i : mCFLines) i->removeTriangle(this);
	} // dtor
	
	
	void cf_triangle::splitT(std::vector<cf_triangle*> newTri)
	{		
		//Declarations
		std::vector<cf_triangle*> newTriangles;
		bool split = false;
		
		//Definition of new triangles
		int k = 0;
		for (const auto i : newTri)
		{
			split = true;
			std::vector<utilities::geometry::vertex> cornerVertices;
			for(auto j = i->begin(); j != i->end(); j++)
			{
				cornerVertices.push_back(*j);
			}
			
			newTriangles.push_back(mGeometryModel->addTriangle(
					utilities::geometry::triangle(cornerVertices, mGeometryModel->tolerance())));
			k++;
		}
		
		//Removing origonal rectangle and replace it with new rectangles
		if (split)
		{
			mDeletion = true;
			for (auto& i : mCFSurfaces)
			{
				i->removeTriangle(this);
				for (const auto& j : newTriangles)
				{
					i->addTriangle(j);
					j->addSurface(i);
				}
			}
			
			mCFSurfaces.clear();
		}
	}
	
	
} // conformal
} // spatial_design
} // bso

#endif // CF_TRIANGLE_CPP