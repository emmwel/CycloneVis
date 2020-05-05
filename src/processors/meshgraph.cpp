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
    , inport_("inport")
    , outport_("outport")
    , filterVertices_("filterVertices", "Filter Vertices", false)
    , filterEdges_("filterEdges", "Filter Edges", false)
    , filterXMin_("filterXMin", "Filter X Min Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterYMin_("filterYMin", "Filter Y Min Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterZMin_("filterZMin", "Filter Z Min Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterXMax_("filterXMax", "Filter X Max Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterYMax_("filterYMax", "Filter Y Max Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterZMax_("filterZMax", "Filter Z Max Range", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , filterEdgeLength_("filterEdgeLength", "Filter Edge Length", 0.0f, 1.0f, -1.0e1f, 1.0e1f)
    , graphCreated_(false) {
    
    addPort(inport_);
    addPort(outport_);
    addProperties(filterVertices_, filterXMin_, filterXMax_, filterYMin_, filterYMax_, filterZMin_, filterZMax_, filterEdges_, filterEdgeLength_);
    
    // Serialize as filtering positions depend on mesh input
    filterXMin_.setSerializationMode(PropertySerializationMode::All);
    filterYMin_.setSerializationMode(PropertySerializationMode::All);
    filterZMin_.setSerializationMode(PropertySerializationMode::All);
    filterXMax_.setSerializationMode(PropertySerializationMode::All);
    filterYMax_.setSerializationMode(PropertySerializationMode::All);
    filterZMax_.setSerializationMode(PropertySerializationMode::All);
    filterEdgeLength_.setSerializationMode(PropertySerializationMode::All);
        
    inport_.onChange([this]() {
        graphCreated_ = false;
        filterVertices_ = false;
        filterEdges_ = false;
        const auto mesh = inport_.getData();
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
    });
        
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
    return std::isgreaterequal(d, filterEdgeLength_->x) && std::islessequal(d, filterEdgeLength_->y);
}

void MeshGraph::createGraph() {
    // Clear graph
    graph_.clear();
    
    // Get input mesh to get data for the graph
    std::shared_ptr<Mesh> mesh(inport_.getData()->clone());
    
    // Buffers, only works when there is one position buffer and one color buffer
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto colorBuffer = static_cast<Buffer<vec4>*>(mesh->getBuffer(BufferType::ColorAttrib));
    
    // Get data from the buffers
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    positions_ = positions;
    auto colors = colorBuffer->getEditableRAMRepresentation()->getDataContainer();
    
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
        std::pair<int, int> edge = edges[i];
        
        // Calculate edge length
        double edgeLength = glm::distance(positions[edge.second], positions[edge.first]);
       
        // Set min and max length
        minLength = std::min(edgeLength, minLength);
        maxLength = std::max(edgeLength, maxLength);
        
        // Insert edge and set vertex data as vertex position
        // and set edge data as edge length
        graph_.insert_edge({edge.first, positions[edge.first]}, {edge.second, positions[edge.second]}, edgeLength);
    }
    // Set property for filtering edge length
    filterEdgeLength_.set({minLength, maxLength}, {minLength, maxLength}, increment_, minSep_);

    graphCreated_ = true;
}

void MeshGraph::process() {
    // If there is no input yet
    if (!inport_.isReady())
        return;
    
    // Create graph if not created
    if (!graphCreated_)
        createGraph();
    
    // Filtered graph
    NGraph::tGraph<int, vec3, double> graphFiltered;
    
    // Return unfiltered mesh if no filtering is activated
    if (!filterVertices_ && !filterEdges_) {
        outport_.setData(inport_.getData()->clone());

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
    
    // Create index buffer from the filtered graph
    std::vector<std::uint32_t> indexMeshData;
    std::vector<int> newIndices = graphFiltered.get_graph_as_vertex_list();

    for (auto& ni : newIndices) {
        indexMeshData.push_back(static_cast<std::uint32_t>(ni));
    }
    auto indexBuff = util::makeIndexBuffer(std::move(indexMeshData));

    // Create position buffer by copying and moving old positions
    std::vector<vec3> positions(positions_);
    auto posBuff = util::makeBuffer(std::move(positions));


    // Create mesh from buffers
    inviwo::Mesh* result = new Mesh(DrawType::Lines, ConnectivityType::None);
    result->addBuffer(BufferType::PositionAttrib, posBuff);
    result->addIndicies(Mesh::MeshInfo(DrawType::Lines, ConnectivityType::None), indexBuff);
    
    outport_.setData(result);
    
    
}

}  // namespace inviwo
