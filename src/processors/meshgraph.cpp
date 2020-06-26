/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2020 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/cyclonevis/processors/meshgraph.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/core/network/networklock.h>
#include <inviwo/cyclonevis/util/coordinatetransformations.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MeshGraph::processorInfo_{
    "org.inviwo.MeshGraph",      // Class identifier
    "Mesh Graph",                // Display name
    "CycloneVis",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo MeshGraph::getProcessorInfo() const { return processorInfo_; }

MeshGraph::MeshGraph()
    : Processor()
    , meshInport_("meshInport")
    , outport_("outport")
    , filterVertices_("filterVertices", "Filter Vertices", false)
    , filterEdges_("filterEdges", "Filter Edges", false)
    , filterXMin_("filterXMin", "Filter X Min Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterYMin_("filterYMin", "Filter Y Min Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterZMin_("filterZMin", "Filter Z Min Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterXMax_("filterXMax", "Filter X Max Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterYMax_("filterYMax", "Filter Y Max Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterZMax_("filterZMax", "Filter Z Max Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
	, distanceMethod_("distanceMethod", "Distance Method",
		{
		  {"euclidean", "Euclidean", DistanceMethod::Euclidean},
		  {"greatCircleHaversine", "Great Circle Distance - Haversine", DistanceMethod::GreatCircleHaversine}
		}, 1)
    , filterEdgeLength_("filterEdgeLength", "Filter Edge Length", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
	, basis_("Basis", "Basis and offset")
    , graphCreated_(false) {
    
    addPort(meshInport_);
    addPort(outport_);
    addProperties(filterVertices_, filterXMin_, filterXMax_, filterYMin_, filterYMax_, filterZMin_, filterZMax_, filterEdges_, distanceMethod_, filterEdgeLength_, basis_);
    
    // Serialize as filtering positions depend on mesh input
    filterXMin_.setSerializationMode(PropertySerializationMode::All);
    filterYMin_.setSerializationMode(PropertySerializationMode::All);
    filterZMin_.setSerializationMode(PropertySerializationMode::All);
    filterXMax_.setSerializationMode(PropertySerializationMode::All);
    filterYMax_.setSerializationMode(PropertySerializationMode::All);
    filterZMax_.setSerializationMode(PropertySerializationMode::All);
    filterEdgeLength_.setSerializationMode(PropertySerializationMode::All);

    meshInport_.onChange([this]() {
        meshInportOnChange();
    });

}

void MeshGraph::meshInportOnChange() {
    graphCreated_ = false;
    filterVertices_ = false;
    filterEdges_ = false;
    const auto mesh = meshInport_.getData();
    auto posbuffer = mesh->findBuffer(BufferType::PositionAttrib);
    if (posbuffer.first) {
        NetworkLock lock(this);
        auto minmax = util::bufferMinMax(posbuffer.first);
        
        // Calculate midpoints
        double xMidpoint = (minmax.second.x + minmax.first.x)/2.0;
        double yMidpoint = (minmax.second.y + minmax.first.y)/2.0;
        double zMidpoint = (minmax.second.z + minmax.first.z)/2.0;
        
        // Define first range [min, midpoint]
        vec2 xRangeMin{minmax.first.x, xMidpoint};
        vec2 yRangeMin{minmax.first.y, yMidpoint};
        vec2 zRangeMin{minmax.first.z, zMidpoint};
        
        // Define second range [midpoint, max]
        vec2 xRangeMax{xMidpoint, minmax.second.x};
        vec2 yRangeMax{yMidpoint, minmax.second.y};
        vec2 zRangeMax{zMidpoint, minmax.second.z};
        
        filterXMin_.set(xRangeMin, xRangeMin, increment_, minSep_);
        filterYMin_.set(yRangeMin, yRangeMin, increment_, minSep_);
        filterZMin_.set(zRangeMin, zRangeMin, increment_, minSep_);
        filterXMax_.set(xRangeMax, xRangeMax, increment_, minSep_);
        filterYMax_.set(yRangeMax, yRangeMax, increment_, minSep_);
        filterZMax_.set(zRangeMax, zRangeMax, increment_, minSep_);
        
        filterXMin_.setCurrentStateAsDefault();
        filterYMin_.setCurrentStateAsDefault();
        filterZMin_.setCurrentStateAsDefault();
        filterXMax_.setCurrentStateAsDefault();
        filterYMax_.setCurrentStateAsDefault();
        filterZMax_.setCurrentStateAsDefault();
        
    } else {
        filterXMin_.resetToDefaultState();
        filterYMin_.resetToDefaultState();
        filterZMin_.resetToDefaultState();
        filterXMax_.resetToDefaultState();
        filterYMax_.resetToDefaultState();
        filterZMax_.resetToDefaultState();
        filterEdgeLength_.resetToDefaultState();
    }
}

bool MeshGraph::filterPos(const vec3& v) {
    // Bools for each dim
    bool inXRange = false;
    bool inYRange = false;
    bool inZRange = false;
    
    // Check if x value is within either range
    if ((std::isgreaterequal(v.x, filterXMin_->x) && std::islessequal(v.x, filterXMin_->y)) ||
        (std::isgreaterequal(v.x, filterXMax_->x) && std::islessequal(v.x, filterXMax_->y))) {
        inXRange = true;
    }
    // Check if y value is within either range
    if ((std::isgreaterequal(v.y, filterYMin_->x) && std::islessequal(v.y, filterYMin_->y)) ||
        (std::isgreaterequal(v.y, filterYMax_->x) && std::islessequal(v.y, filterYMax_->y))) {
        inYRange = true;
    }
    // Check if z value is within either range
    if ((std::isgreaterequal(v.z, filterZMin_->x) && std::islessequal(v.z, filterZMin_->y)) ||
        (std::isgreaterequal(v.z, filterZMax_->x) && std::islessequal(v.z, filterZMax_->y))) {
        inZRange = true;
    }
    
    // Return true if all dims are within a range
    if (inXRange && inYRange && inZRange) {
        return true;
    }
    else {
        return false;
    }
}

bool MeshGraph::filterLength(const double &d) {
	// Check if edge distance is within allowed ranged
    return std::isgreaterequal(d, filterEdgeLength_->x) && std::islessequal(d, filterEdgeLength_->y);
}

void MeshGraph::createGraph() {
    // Clear graph
    graph_.clear();
    
    // Get input mesh to get data for the graph
    std::shared_ptr<Mesh> mesh(meshInport_.getData()->clone());

	mat4 haha = mesh->getModelMatrix();
	mat3 hmsa = mesh->getBasis();
    
    // Buffers, only works when there is one position buffer and one color buffer
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    
    // Get data from the buffer
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    positions_ = positions;
    
    // Get index buffer
    auto indBuffer = mesh->getIndices(0);
    auto indices = indBuffer->getEditableRAMRepresentation()->getDataContainer();
    
    // Create vector of edges where [int, int] are the vertices connected to that edge
    std::vector<std::pair<int, int>> edges;
    for (unsigned long i = 0; i < indices.size() - 1; i += 2) {
        edges.push_back({indices[i], indices[i+1]});
    }
    
    // Data for saving min and max length of edge
    double minLength = std::numeric_limits<double>::max();
    double maxLength = std::numeric_limits<double>::min();
    
    // Create graph by inserting edges
    for (unsigned long i = 0; i < edges.size(); ++i) {
		// Get edge
        std::pair<int, int> edge = edges[i];

		switch (distanceMethod_) {
			case DistanceMethod::Euclidean: {
				// Calculate edge length
				double edgeLength = glm::distance(positions[edge.second], positions[edge.first]);

				// Set min and max length for filtering
				minLength = std::min(edgeLength, minLength);
				maxLength = std::max(edgeLength, maxLength);

				// Insert edge and set vertex data as vertex position
				// and set edge data as edge length
				graph_.insert_edge({ edge.first, positions[edge.first] }, { edge.second, positions[edge.second] }, edgeLength);

				break;
			}

			case DistanceMethod::GreatCircleHaversine: {
				// Transform edge points to latitude longitude
				vec2 dimX = vec2(basis_.offset_.get().x, basis_.offset_.get().x + basis_.a_.get().x);
				vec2 dimY = vec2(basis_.offset_.get().y, basis_.offset_.get().y + basis_.b_.get().y);
				vec2 pos1LatLong = coordTransform::cartesianToLatLong(vec2(positions[edge.first][0], positions[edge.first][1]), dimX, dimY);
				vec2 pos2LatLong = coordTransform::cartesianToLatLong(vec2(positions[edge.second][0], positions[edge.second][1]), dimX, dimY);

				std::cout << pos1LatLong << std::endl;
				std::cout << pos2LatLong << std::endl;

				// Distance is returned in meters, but show in km instead for clarity
				double edgeLengthGreatCircle = coordTransform::distanceHaversine(pos1LatLong, pos2LatLong) / 1000;

				// Set min and max length for filtering
				minLength = std::min(edgeLengthGreatCircle, minLength);
				maxLength = std::max(edgeLengthGreatCircle, maxLength);

				// Insert edge and set vertex data as vertex position
				// and set edge data as edge length
				graph_.insert_edge({ edge.first, positions[edge.first] }, { edge.second, positions[edge.second] }, edgeLengthGreatCircle);

				break;
			}

			default:
				break;
		}
    }

    // Set property for filtering edge length
    filterEdgeLength_.set({minLength, maxLength}, {minLength, maxLength}, increment_, minSep_);

    graphCreated_ = true;
}


// TODO: actual tests instead of strange debugging setup?
void MeshGraph::testsCoordinateTransformations() {
	// ------ TESTNG COORD TRANSFORMS ------ //

	/* CYLINDER TRANSFORMS */
	{
		// Cylinder to cartesian -- Works!

		//// Test 1
		//vec3 cyl = vec3(4, (2 * M_PI) / 3, -2);
		//vec3 expected_res = vec3(-2, 2 * sqrt(3), -2);
		//vec3 res = coordTransform::cylindricalToCartesian(cyl);

		//// Test2
		//vec3 cyl = vec3(5, M_PI / 6, 4);
		//vec3 expected_res = vec3(5 * sqrt(3) / 2, 5.0/2, 4);
		//vec3 res = coordTransform::cylindricalToCartesian(cyl);

		//// Used for debugging bc. assert what
		//std::cout << cyl << std::endl;
		//std::cout << expected_res << std::endl;
		//std::cout << res << std::endl;
	}
	{
		// Cartesian to cylinder -- Works!

		//// Test 1
		//vec3 cart = vec3(1, -3, 5);
		//vec3 res_expected = vec3(sqrt(10), 5.03, 5); // or theta = -1.249..
		//vec3 res = coordTransform::cartesianToCylindrical(cart);

		//// Test 2
		//vec3 cart = vec3(-8, 8, -7);
		//vec3 res_expected = vec3(8*sqrt(2), (3 * M_PI)/ 4, -7);
		//vec3 res = coordTransform::cartesianToCylindrical(cart);

		//std::cout << cart << std::endl;
		//std::cout << res_expected << std::endl;
		//std::cout << res << std::endl;
	}

	/* SPHERE TRANSFORMS */
	{
		// Spherical to cartesian -- Works!

		//// Test 1
		//vec3 spher = vec3(8, M_PI / 3, M_PI / 6);
		//vec3 res_expected = vec3(2, 2 * sqrt(3), 4 * sqrt(3));
		//vec3 res = coordTransform::sphericalToCartesian(spher);

		//// Test 2
		//vec3 spher = vec3(2, (- 5 * M_PI) / 6, M_PI / 6);
		//vec3 res_expected = vec3(- sqrt(3) / 2, -0.5, sqrt(3));
		//vec3 res = coordTransform::sphericalToCartesian(spher);

		//std::cout << spher << std::endl;
		//std::cout << res_expected << std::endl;
		//std::cout << res << std::endl;
	}
	{
		// Cartesian to spherical -- Works!

		//// Test 1
		//vec3 cart = vec3(-1, 1, sqrt(6));
		//vec3 res_expected = vec3(2 * sqrt(2), (3 * M_PI) / 4, M_PI / 6);
		//vec3 res = coordTransform::cartesianToSpherical(cart);

		//std::cout << cart << std::endl;
		//std::cout << res_expected << std::endl;
		//std::cout << res << std::endl;

	}

	/* LATITUDE-LONGITUDE */
	{
		// Cartesian to latitude longitude

		// following may be simpler using extent instead of basis...
		vec2 dimX = vec2(basis_.offset_.get().x, basis_.offset_.get().x + basis_.a_.get().x);
		vec2 dimY = vec2(basis_.offset_.get().y, basis_.offset_.get().y + basis_.b_.get().y);

		//// Test 1
		//vec2 cart = vec2(0, 0);
		//vec2 res_exp = vec2(0, 0);
		//vec2 res = coordTransform::cartesianToLatLong(cart, dimX, dimY);

		//std::cout << cart << std::endl;
		//std::cout << res_exp << std::endl;
		//std::cout << res << std::endl;

		//// Test 2
		//vec2 cart = vec2(16, 0);
		//vec2 res_exp = vec2(0, 180); // (lat, long) --> x is long, y is lat.
		//vec2 res = coordTransform::cartesianToLatLong(cart, dimX, dimY);

		//std::cout << cart << std::endl;
		//std::cout << res_exp << std::endl;
		//std::cout << res << std::endl;

		//// Test 3
		//vec2 cart = vec2(16, 8);
		//vec2 res_exp = vec2(90, 180); // (lat, long) --> x is long, y is lat.
		//vec2 res = coordTransform::cartesianToLatLong(cart, dimX, dimY);

		//std::cout << cart << std::endl;
		//std::cout << res_exp << std::endl;
		//std::cout << res << std::endl;

		//// Test 4
		//vec2 cart = vec2(-16, -8);
		//vec2 res_exp = vec2(-90, -180); // (lat, long) --> x is long, y is lat.
		//vec2 res = coordTransform::cartesianToLatLong(cart, dimX, dimY);

		//std::cout << cart << std::endl;
		//std::cout << res_exp << std::endl;
		//std::cout << res << std::endl;

		//// Test 5
		//vec2 cart = vec2(8, 4);
		//vec2 res_exp = vec2(45, 90); // (lat, long) --> x is long, y is lat.
		//vec2 res = coordTransform::cartesianToLatLong(cart, dimX, dimY);

		//std::cout << cart << std::endl;
		//std::cout << res_exp << std::endl;
		//std::cout << res << std::endl;

		// Test 6
		//vec2 cart = vec2(-8, -4);
		//vec2 res_exp = vec2(-45, -90); // (lat, long) --> x is long, y is lat.
		//vec2 res = coordTransform::cartesianToLatLong(cart, dimX, dimY);

		//std::cout << cart << std::endl;
		//std::cout << res_exp << std::endl;
		//std::cout << res << std::endl;
	}

	// ------------------------------------ //
}

void MeshGraph::process() {
    // If there is no input yet
    if (!meshInport_.isReady())
        return;
    
    // Create graph if not created
    if (!graphCreated_ || distanceMethod_.isModified())
        createGraph();

	testsCoordinateTransformations();

    // Filtered graph
    NGraph::tGraph<int, vec3, double> graphFiltered;
    
    // Return unfiltered mesh if no filtering is activated
    if (!filterVertices_ && !filterEdges_) {
        outport_.setData(meshInport_.getData()->clone());

        return;
    }
    
    // Filter both vertices and edges
    if (filterVertices_ && filterEdges_) {
        // Bind filter functions to search condition, then do BFS on graph
        graph_.search_condition_vertex = std::bind(&MeshGraph::filterPos, this, std::placeholders::_1);
        graph_.search_condition_edge = std::bind(&MeshGraph::filterLength, this, std::placeholders::_1);
        
        NGraph::tGraph<int, vec3, double> graphVertexFiltered = graph_.BFS_vertex();
        NGraph::tGraph<int, vec3, double> graphEdgeFiltered = graph_.BFS_edge();
        
        // Combine results
        graphFiltered = graphVertexFiltered.intersect(graphEdgeFiltered);
    
    }
    // Only filter vertices
    else if (filterVertices_ && !filterEdges_) {
        // Bind filter function, then do BFS
        graph_.search_condition_vertex = std::bind(&MeshGraph::filterPos, this, std::placeholders::_1);
        
        graphFiltered = graph_.BFS_vertex();
    }
    // Only filter edges
    else if (!filterVertices_ && filterEdges_) {
        // Bind filter function, then do BFS
        graph_.search_condition_edge = std::bind(&MeshGraph::filterLength, this, std::placeholders::_1);
        graphFiltered = graph_.BFS_edge();
    }
    
    // Get indices from the filtered graph
    std::vector<int> graphIndices = graphFiltered.get_graph_as_vertex_list();

    // Create mapping of indices and vector of positions not filtered away
    std::set<int> uniqueIndices(graphIndices.begin(), graphIndices.end());
    std::map<int, int> rangedIndices;
    std::vector<vec3> filterPositions;
    int counter = 0;
    for (std::set<int>::iterator i = uniqueIndices.begin(); i != uniqueIndices.end(); i++) {
        rangedIndices[*i] = counter;
        filterPositions.push_back(positions_[*i]);
        ++counter;
    }
    
    // Map all graph indices to the new mapping
    std::vector<std::uint32_t> indexMeshData;
    for (unsigned long i = 0; i < graphIndices.size(); i++) {
        indexMeshData.push_back(static_cast<std::uint32_t>(rangedIndices[graphIndices[i]]));
    }
    
    auto indexBuff = util::makeIndexBuffer(std::move(indexMeshData));
    auto posBuff = util::makeBuffer(std::move(filterPositions));

    // Create mesh from buffers
    inviwo::Mesh* result = new Mesh(DrawType::Lines, ConnectivityType::None);
    result->addBuffer(BufferType::PositionAttrib, posBuff);
    result->addIndicies(Mesh::MeshInfo(DrawType::Lines, ConnectivityType::None), indexBuff);

    outport_.setData(result);
}

}  // namespace inviwo
