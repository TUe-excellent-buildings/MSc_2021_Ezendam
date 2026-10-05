#ifndef CF_SPACE_HPP
#define CF_SPACE_HPP

namespace bso { namespace spatial_design { namespace conformal {
	
	/*----------------------------------------------------------------------------
	The cf_space class
	
	Purpose: store the space information.   
	Furthermore, this class constains the facilaties of generating a conformal model with the checkVertex...() functions.
	This function represent step two in the iterative partitions methods for which multiple variants where developed:
	
	Existing step two iterative partitions methods:
	Method for soly orthogonal rectangular BSDs:
		Always combine with makeConformal() of cf_building_model:
			checkVertex() --> is the orthogonal rectangular partition methods (or conformation methods) 
							  as decribed in Sjonnies Boonstra paper of 2018.
	
	Method for both orthogonal rectangular BSDs and all shapes of the non-orthogonal BSD:
		Always combine with makeConformalN() of cf_building_model:
			checkVertexN() --> quad-hexahedron method/iterative partition method first considering all more then two intersection on the floors 
							   (first polygon intersection then line intersection). 
							   Thereafter it considers all intersection on the walls (first polygon intersection then line intersection).
			checkVertexNO() --> quad-hexahedron method/iterative partition method (as decribed in ) first considered all polygon intersections 
							    whereafter all line intersection are considered.
							   
		Always combine with makeConformalT() of cf_building_model:
			checkVertexT() --> tetrahedron method as decribed in 
	
	Method for both orthogonal rectangular BSDs and non-orthogonal BSD soly consisting of vertical walls and horizontal floors:
		Note: This methods is not finished and neither published and is therefore not recomended to be used.
		Always combine with makeConformalN2D() of cf_building_model:
			checkVertexN2D() --> consideres the NOBSD as a 2D intersection case in the z=0 plane. 
								 This to prevent iteration order problems in the vertical stacked space consideration
	------------------------------------------------------------------------------*/ 
	
	
	class cf_space : public utilities::geometry::quad_hexahedron,
									 public cf_building_entity
	{
	private:
		std::string mSpaceType = "";
		unsigned int mSpaceID = 0;
	public:
		cf_space(const utilities::geometry::quad_hexahedron& rhs, cf_building_model* buildingModel);
		cf_space(const utilities::geometry::quad_hexahedron& rhs, cf_building_model* buildingModel, std::vector<utilities::geometry::tetrahedron> tet);
		cf_space(const utilities::geometry::quad_hexahedron& rhs, cf_building_model* buildingModel, std::vector<utilities::geometry::triangular_prism> triPrism);
		
		//All step two function or call step two diferently
		void checkVertex(cf_vertex* pPtr);
		void checkVertexNO(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method, std::vector<cf_cuboid*> c); // only checks the inserted cells for splitting
		void checkVertexNO(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method); // Checks all the cells in the space for splitting
		void checkVertexN(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method, std::vector<cf_cuboid*> c, cf_building_model* buildingModel); // only checks the inserted cells for splitting
		void checkVertexN(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method, cf_building_model* buildingModel); // Checks all the cells in the space for splitting
		bool checkVertexN2D(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method, std::vector<cf_cuboid*> c); // Find intersection at ground plane of all cells
		bool checkVertexN2D(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment> intsecL, char method); // Find intersection at ground plane of all cells
		void checkVertexT(cf_vertex* pPtr);
		
		void splitLines(const std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment>& intsecL);
		void splitLinesTwo(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment>& intsecL);
		void splitLinesTwo2D(std::vector<cf_vertex>& intsecV, std::vector <utilities::geometry::line_segment>& intsecL);
		
		void addLine					(cf_line* 			lPtr	)	= delete;
		void addRectangle				(cf_rectangle* 	recPtr) 		= delete;
		void addTriangle				(cf_triangle* 		triPtr) 	= delete;
		void removeLine					(cf_line* 			lPtr	)	= delete;
		void removeRectangle			(cf_rectangle* 	recPtr) 		= delete;
		void removeTriangle				(cf_triangle* 		triPtr) 	= delete;
		void addSpace					(cf_space*			spPtr	) 	= delete;
		
		void setSpaceID(const unsigned int& spaceID) {mSpaceID = spaceID;}
		void setSpaceType(const std::string& spaceType) {mSpaceType = spaceType;}

		// It seems that these four function bellow are useless:
		const std::vector<cf_vertex*		>& cfVertices() 	const { return mCFVertices;}
		const std::vector<cf_line*			>& cfLines() 		const { return mCFLines;}
		const std::vector<cf_rectangle*		>& cfRectangles() 	const { return mCFRectangles;}
		const std::vector<cf_space*			>& cfSpaces() 		const { return mCFSpaces;}
		
		const unsigned int getSpaceID() const {return mSpaceID;}
		const std::string getSpaceType() const {return mSpaceType;}
		
		static bool sortSplitV (std::vector<cf_vertex>* ptr1, std::vector<cf_vertex>* ptr2) {return (ptr1->size() > ptr2->size());} // Kan dit wel static benoemt worden
	};
	
} // conformal
} // spatial_design
} // bso

#endif // CF_SPACE_HPP