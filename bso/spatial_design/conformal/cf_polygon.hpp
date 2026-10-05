#ifndef CF_POLYGON_HPP
#define CF_POLYGON_HPP

namespace bso { namespace spatial_design { namespace conformal {
	
	class cf_polygon : public bso::utilities::geometry::polygon,
											 public cf_geometry_entity
	{
	private:
		
	public:
		cf_polygon(const utilities::geometry::polygon& rhs, cf_geometry_model* geometryModel);
		~cf_polygon();
		
		void sortPoints(const double& tol /*= 1e-3*/);
		void split(std::vector<cf_polygon*> newRect);
		void split(cf_vertex* pPtr);
		void checkAssociated(cf_vertex* pPtr);
		
		void addLine					(cf_line* 			lPtr) 	= delete;
		void addRectangle				(cf_rectangle* 		recPtr) = delete;
		void addPolygon					(cf_polygon* 		polPtr) = delete;
		void addCuboid					(cf_cuboid* 		cubPtr)	= delete;
		void addTetrahedron				(cf_tetrahedron* 	tetPtr) = delete; // should have been added in the tetrahedron phase
		void addTriPrism				(cf_triPrism*		triPtr) = delete;
		void removeLine					(cf_line* 			lPtr) 	= delete;
		void removeRectangle			(cf_rectangle* 		recPtr) = delete;
		void removePolygon				(cf_polygon* 		polPtr) = delete;
		void removeCuboid				(cf_cuboid* 		cubPtr)	= delete;
		void removeTriPrism				(cf_triPrism*		triPtr)	= delete;
		void addPoint					(cf_point*			pPtr) 	= delete;
		void addEdge					(cf_edge*			ePtr) 	= delete;
		void addSpace					(cf_space*			spPtr) 	= delete;
		
		const std::vector<cf_rectangle*		>& cfRectangles() 	= delete;
		const std::vector<cf_polygon*		>& cfPolygons()		= delete;
		const std::vector<cf_cuboid*		>& cfCuboids()		= delete;
		const std::vector<cf_tetrahedron*	>& cfTetrahedrons() = delete; // should have been added in the tetrahedron phase
		const std::vector<cf_triPrism*		>& cfTriPrism()		= delete;
		const std::vector<cf_point*			>& cfPoints() 		= delete;
		const std::vector<cf_edge*			>& cfEdges() 		= delete;
		const std::vector<cf_space*			>& cfSpaces() 		= delete;
	};
	
} // conformal
} // spatial_design
} // bso

#endif // CF_POLYGON_HPP