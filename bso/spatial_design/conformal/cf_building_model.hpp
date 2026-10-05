#ifndef CF_BUILDING_MODEL_HPP
#define CF_BUILDING_MODEL_HPP

#include <bso/spatial_design/ms_building.hpp>

namespace bso { namespace spatial_design { namespace conformal {
	
	/*----------------------------------------------------------------------------
	The cf_building_model class

	Purpose: stores both the building conformal model and the geometry conformal model (as a public inheritance of class cf_geometry_model) representing the full 'conformal model'. 
	Both these models provide the storage of each of the separate geometry entities required for discipline specific models definition.
	This advantage of such a definition of each of the separate geometry entities is that: 
	(1) it does not need to be generated or search for in each individual discipline specific models definition part of the toolbox. 
	Hence, allowing for a faster toolbox since the generation of each individual geometry only has to be performed ones.
	(2) the exact geometry will therefore be standardized thus reducing the risk of round of error and slightly different geometry 
	entities used during discipline specific model definitions and analysis.
	
	Please note: the UML class diagram (describing all the links between the geometry entities used in the building conformal model and the geometry conformal model) 
	and the explanations of each individual type of automatic partitioning methods to reach a conformal	geometry can be found in the following literature: 
	(1)
	(2)
	
	Existing methods to make the geometry conformal.
	Use the following char to asked for any of these method:
	makeConformalN() --> 'I'/'O'/'M', the multiple letters are used for the different sub-methods to split a quad-hexahedron cell
	makeConformalN2D() --> ''
	makeConformalT() --> 'T', 
	makeConformalND() --> 'D', Delaunay triangulation is performed in a 2D plane to reach a conformal model represented by triangular prism.
			
	The following makeConformal...() function are some of the provided methods to make the geometry conformal
	These function are placed in the public section to allow for an interim visualization in case of an error or for the visualization of the initial geometry conformal model.
	To make a visualization of the interim conformal model perform the following:
	(1) set alreadyConformal = false; while asking for the constructor
	(2) ask for one of the makeConformal...() functions within a try{}catch(){} system with makeConformal...() in the try section
	(3) initiate visualization in the catch section for this cf_building_model entity
	To make a visualization of the initial conformal model perform the following:
	(1) set alreadyConformal = false; while asking for the constructor
	(2) initiate visualization for this cf_building_model entity
	------------------------------------------------------------------------------*/ 
		
	class cf_building_model : public cf_geometry_model
	{
	private:
		//Stores the building conformal model entities
		std::vector<cf_point*> mCFPoints;
		std::vector<cf_edge*> mCFEdges;
		std::vector<cf_surface*> mCFSurfaces;
		std::vector<cf_space*> mCFSpaces;

		ms_building mMSModel; // safe it, in case copy consttructor is called
		char methodUsed; // safe it, in case copy constructor is called
		double mTol; //store the tolerance used during conformal model generation
		std::vector<cf_building_model*> cfStages; // used to store a visualization of each indivudual split action performed in sequence
		int interspaceCount;
		int intercellCount;

		std::vector<utilities::geometry::quad_hexahedron> spacesAtGround; //safe it, in case of the visualization of 'space_at_ground' from the Delaunay method
		std::vector<utilities::geometry::line_segment> delauneyLines; //Lines where the delauney triangulation should constrain to; safe it, in case of visualization of the 'delaunay' from the Delaunay method
		std::vector<utilities::geometry::triangle> delaunayTri; //resulting delaunay triangles at z=0, when using the delaunay triangulation method; safe it, in case of visualization of the 'delaunay' from the Delaunay method
		std::vector <utilities::geometry::vertex> delauneyPoints; //Store the amount of delauney points to get the amount later on in the process

		void addSpace(const ms_space& msSpace); // initializer buiding- and geometry-conformal model quad-hexahedron method; add the space geometry as the initial cells (note: the geometry in the geometry-conformal model is not yet conformal, please ensure that the makeConformalN() or makeConformalN2D() is performed to reach the required conformal geometry)
		void addSpaceT(const ms_space& msSpace); // (note: the geometry in the geometry-conformal model is not yet conformal, please ensure that the makeConformalN() or makeConformalN2D() is performed to reach the required conformal geometry)
		void addSpaceD(const ms_space& msSpace, std::vector<utilities::geometry::triangular_prism>& triPrismPtr); // Generates the buiding- and geometry-conformal model after the makeConformalND() and is automatically initiated in case the Delaunay method is choosen.
		
		void makeConformal(); // makeConformal function for rectangular and orthogonal building spatial designs
		void makeConformalND(const ms_building& msModel); // NOBSD delaunay method
		
		void splitLines(const std::vector<utilities::geometry::vertex>& intsecV, std::vector<utilities::geometry::line_segment>& intsecL); // used during the delaunay method to avoid the inserting of overlapping and duplicate constrain lines
		std::vector<utilities::geometry::triangle> delaunay(const std::vector <utilities::geometry::vertex> delauneyPoints, const std::vector <utilities::geometry::line_segment> delauneyLines); // Delaunay triangulation performed with the help of the FADE_2D libary
		
		cf_building_model& operator = (cf_building_model& rhs) = default;
		friend class cf_geometry_entity;
	public:
		cf_building_model(const cf_building_model& rhs , bool solyForVisualization = false); // copy constructor(), this functions makes a copy of this entity so that the following stages works with a copy of this cf_model.
		
		cf_building_model(const ms_building& msModel, const double& tol = 1e-3, bool alreadyConformal = true); // makes use of standart method 'I'
		cf_building_model(const ms_building& msModel, char method, const double& tol = 1e-3, bool alreadyConformal = true); // enable user to define the a method to make the geometry conformal, see bellow for an explenation on the existing methods
		~cf_building_model();
		
		void makeConformalN(); // NOBSD quad-hexahedorn geometry conformal method (method I,O,M)
		void makeConformalN2D(); // NOBSD quad-hexahedorn geometry conformal method (method I,O,M) solved with the consideration of the intersections in 2D plane
		void makeConformalT(); // NOBSD tetrahedron geometry conformal method
		
		void checkEntities(); // this check is currently not an extensive check on all the links which should be correct.
		void checkPartitionSize(); // used for analysing the suitablity of the conformal model for a quality mesh acording to the multi-block method principles
		void storeCfStages(cf_building_model* cf) {cfStages.push_back(new cf_building_model(*cf, true));}
		const std::vector<cf_building_model*> getCfStages() const {return cfStages;}
		const std::vector <utilities::geometry::vertex> getDelauneyPoints() const {return delauneyPoints;}
		int getNrInitInters() const;
		int getNrIterationsinterSpace() const;
		int getNrIterationsinterCell() const;
		int getNrCells() const;
		
		const std::vector<cf_point*			>& cfPoints() 		const { return mCFPoints;}
		const std::vector<cf_edge*			>& cfEdges() 		const { return mCFEdges;}
		const std::vector<cf_surface*		>& cfSurfaces() 	const { return mCFSurfaces;}
		const std::vector<cf_space*			>& cfSpaces() 		const { return mCFSpaces;}
		
		const char getMethodUsed() const {return methodUsed;}	
		
		//Functions used to allow for visualization of the interim stages of the Delaunay method
		const std::vector<utilities::geometry::quad_hexahedron> getSpacesAtGround() const {return spacesAtGround;} //Used for sending to spaces at ground level to the visualization
		const std::vector<utilities::geometry::line_segment> getDelauneyLines() const {return delauneyLines;} //Used for sending to constrain lines inserted in the Delaunay method to the visualization
		const std::vector<utilities::geometry::triangle> getDelaunayTri() const {return delaunayTri;} //Used for sending to delaunay triangles at ground to the visualization
	};
	
} // conformal
} // spatial_design
} // bso

#endif // CF_BUILDING_MODEL_HPP