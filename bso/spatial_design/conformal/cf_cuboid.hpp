#ifndef CF_CUBOID_HPP
#define CF_CUBOID_HPP

namespace bso { namespace spatial_design { namespace conformal {
	
	class cf_cuboid : public bso::utilities::geometry::quad_hexahedron,
										public cf_geometry_entity
	{
	private:
		std::vector<cf_rectangle*> floors; // This is neccecary for the geometry conformal method and need not be the same as in the grammer
		std::vector<cf_rectangle*> walls; // This is neccecary for the geometry conformal method and need not be the same as in the grammer
		double groundDiference;
		utilities::geometry::quad_hexahedron cuboidAtGround;
		utilities::geometry::quadrilateral rectangleAtGround;
	public:
		cf_cuboid(const utilities::geometry::quad_hexahedron& rhs, cf_geometry_model* geomModel);
		~cf_cuboid();
		
		void split(cf_vertex* pPtr); // orthogonal rectangular split function
		
		// These split function only consideres a single kind of sub-method
		bool splitN(std::vector<cf_vertex>* intsecV, char method, std::vector<cf_cuboid*>& newCells); // non-orthogonal split function
		bool splitN(std::vector<cf_vertex>* intsecV, char method); // Diferent call to non-orthogonal split function
		
		// These split functions alternate the sub-methods considered
		bool splitNtry(std::vector<cf_vertex>* intsecV, char method, std::vector<cf_cuboid*>& newCells); // non-orthogonal split function
		bool splitNtry(std::vector<cf_vertex>* intsecV, char method); // Diferent call to non-orthogonal split function
		bool splitNtry2(std::vector<cf_vertex>* intsecV, char method, std::vector<cf_cuboid*>& newCells); // non-orthogonal split function
		bool splitNtry2split(std::vector<cf_vertex>* intsecV, char method, std::vector<cf_cuboid*>& newCells); // non-orthogonal split function
		bool splitNtry2(std::vector<cf_vertex>* intsecV, char method); // Diferent call to non-orthogonal split function
		
		
		void checkAssociated(cf_vertex* pPtr); // orthogonal rectangular split function to split the other cf entities to conformal entities
		void checkAssociatedN(std::vector<cf_cuboid*> newCuboids); // non-orthogonal split function; send geometry entities to other cf_geometry for splitting
		
		const std::vector<cf_rectangle*>& getCellWalls() const{return walls;}
		const std::vector<cf_rectangle*>& getCellFloors() const{return floors;}
		const double getGroundDiference() const{return groundDiference;}
		const utilities::geometry::quad_hexahedron& getCuboidAtGround() {return cuboidAtGround;}
		const utilities::geometry::quadrilateral& getRectangleAtGround() const {return rectangleAtGround;}
		
		void addLine					(cf_line* 			lPtr) 		= delete ;
		void addRectangle				(cf_rectangle* 	recPtr) 		= delete ;
		void addTriangle				(cf_triangle* 		triPtr) 	= delete ;
		void addCuboid					(cf_cuboid* 		cubPtr) 	= delete ;
		void addTetrahedron				(cf_tetrahedron* 	tetPtr)		= delete ;
		void addTriPrism				(cf_triPrism*		priPtr)		= delete ;
		void removeLine					(cf_line* 			lPtr) 		= delete ;
		void removeRectangle			(cf_rectangle* 	recPtr) 		= delete ;
		void removeTriangle				(cf_triangle* 		triPtr) 	= delete ;
		void removeCuboid				(cf_cuboid* 		cubPtr) 	= delete ;
		void removeTetrahedron			(cf_tetrahedron* 	tetPtr)		= delete ;
		void removeTriPrism				(cf_triPrism*		priPtr)		= delete ;
		void addPoint					(cf_point*			pPtr) 		= delete ;
		void addEdge					(cf_edge*				ePtr) 	= delete ;
		void addSurface					(cf_surface*		srfPtr) 	= delete ;
		
		const std::vector<cf_triangle*		>& cfTriangles() 	= delete;
		const std::vector<cf_cuboid*		>& cfCuboids() 		= delete;
		const std::vector<cf_tetrahedron*	>& cfTetrahedrons()	= delete;
		const std::vector<cf_triPrism*		>& cfTriPrism()		= delete;
		const std::vector<cf_point*			>& cfPoints() 		= delete;
		const std::vector<cf_edge*			>& cfEdges() 		= delete;
		const std::vector<cf_surface*		>& cfSurfaces() 	= delete;
		
		bool markInVisualization = false;
	};
	
} // conformal
} // spatial_design
} // bso

#endif // CF_CUBOID_HPP