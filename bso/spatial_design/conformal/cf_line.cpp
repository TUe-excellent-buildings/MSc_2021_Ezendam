#ifndef CF_LINE_CPP
#define CF_LINE_CPP

namespace bso { namespace spatial_design { namespace conformal {
	
	cf_line::cf_line(const utilities::geometry::line_segment& l, cf_geometry_model* geometryModel)
	: bso::utilities::geometry::line_segment(l)
	{
		mGeometryModel = geometryModel;
		for (const auto& i : mVertices)
		{
			mCFVertices.push_back(mGeometryModel->addVertex(i));
			mCFVertices.back()->addLine(this);
		}
	}
	
	cf_line::~cf_line()
	{
		for (auto& i : mCFVertices) i->removeLine(this);
	} // dtor
	
	
	void cf_line::splitN(std::vector<cf_line*> newLine)
	{
		bool split = false;
		if(newLine.size() > 1)
		{
			split = true;
		}
		
		
		if(split == true)
		{
			mDeletion = true;
			
			for(const auto j: newLine)
			{
				mGeometryModel->addLine(*j);
			}
			
			for (auto& i : mCFEdges)
			{
				i->removeLine(this);
				
				for(const auto j: newLine)
				{
					j->addEdge(i);
					i->addLine(j);
				}
			}
			
			mCFEdges.clear();
		}
	} // splitN()
	
	void cf_line::splitT(std::vector<cf_line*> newLine)
	{
		bool split = false;
		if(newLine.size() > 1)
		{
			split = true;
		}
		
		
		if(split == true)
		{
			mDeletion = true;
			
			for(const auto j: newLine)
			{
				mGeometryModel->addLine(*j);
			}
			
			for (auto& i : mCFEdges)
			{
				i->removeLine(this);
				
				for(const auto j: newLine)
				{
					j->addEdge(i);
					i->addLine(j);
				}
			}
			
			mCFEdges.clear();
		}		
	} // splitT()
	
	void cf_line::split(cf_vertex* pPtr)
	{
		for (auto& i : mCFVertices)
		{
			if (pPtr == i) return;
		}

		if (this->isOnLine(*pPtr, mGeometryModel->tolerance()))
		{
			mDeletion = true;
			cf_line* l1 = mGeometryModel->addLine({mVertices[0], *pPtr});
			cf_line* l2 = mGeometryModel->addLine({mVertices[1], *pPtr});
			
			for (auto& i : mCFEdges)
			{
				l1->addEdge(i);
				l2->addEdge(i);
				i->addLine(l1);
				i->addLine(l2);
				i->removeLine(this);
			}
			
			this->checkAssociated(pPtr);
			mCFEdges.clear();
		}
	} // split()
	
	void cf_line::checkAssociated(cf_vertex* pPtr)
	{
		for (auto& i : mCFEdges)
		{
			i->checkVertex(pPtr);
			for (auto& j : i->cfSurfaces())
			{
				j->checkVertex(pPtr);
			}
			for (auto& j : i->cfSpaces())
			{
				j->checkVertex(pPtr);
			}
		}
	} // checkAssociated()
	
	void cf_line::checkAssociatedT(cf_vertex* pPtr)
	{
		for (auto& i : mCFEdges)
		{
			i->checkVertex(pPtr);
			for (auto& j : i->cfSurfaces())
			{
				j->checkVertex(pPtr);
			}
			for (auto& j : i->cfSpaces())
			{
				j->checkVertex(pPtr);
			}
		}
	} // checkAssociated()
	
} // conformal
} // spatial_design
} // bso

#endif // CF_LINE_CPP