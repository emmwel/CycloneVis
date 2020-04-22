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
    , filterX_("filterX", "Filter X Pos", 0, 256, 0, 256, 1, 1)
    , filterY_("filterY", "Filter Y Pos", 0, 256, 0, 256, 1, 1)
    , filterZ_("filterZ", "Filter Z Pos", 0, 256, 0, 256, 1, 1)
    , graphCreated_(false) {
    
    addPort(inport_);
    addPort(outport_);
    addProperties(filterVertices_, filterX_, filterY_, filterZ_, filterEdges_);
    
    // Serialize as filtering positions depend on mesh input
    filterX_.setSerializationMode(PropertySerializationMode::All);
    filterY_.setSerializationMode(PropertySerializationMode::All);
    filterZ_.setSerializationMode(PropertySerializationMode::All);
        
    inport_.onChange([this]() {
        graphCreated_ = false;
        
        const auto mesh = inport_.getData();
        auto posbuffer = mesh->findBuffer(BufferType::PositionAttrib);
        if (posbuffer.first) {
            NetworkLock lock(this);
            auto minmax = util::bufferMinMax(posbuffer.first);
            
            // Define ranges
            vec2 x_range{minmax.first.x, minmax.second.x};
            vec2 y_range{minmax.first.y, minmax.second.y};
            vec2 z_range{minmax.first.z, minmax.second.z};
            
            filterX_.setRangeNormalized(x_range);
            filterY_.setRangeNormalized(y_range);
            filterZ_.setRangeNormalized(z_range);
        } else {
            filterX_.resetToDefaultState();
            filterY_.resetToDefaultState();
            filterZ_.resetToDefaultState();
        }
    });
        
}

bool MeshGraph::filterPos(const vec3& v) {
    // Get min and max positions
    vec3 filterMin(filterX_->x, filterY_->x, filterZ_->x);
    vec3 filterMax(filterX_->y, filterY_->y, filterZ_->y);
    
    // Check if the vertex is within the min and max positions
    if (glm::all(glm::greaterThan(v, filterMin)) && glm::all(glm::lessThan(v, filterMax))) {
            return true;
    }
    
    return false;
}

void MeshGraph::createGraph() {
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
    
    // Create graph by inserting edges
    for (unsigned long i = 0; i < edges.size(); ++i) {
        std::pair<int, int> edge = edges[i];
        
        // Calculate edge length
        double edge_length = glm::distance(positions[edge.second], positions[edge.first]);
        
        // Insert edge and set vertex data as vertex position
        // set edge data as edge length
        graph_.insert_edge({edge.first, positions[edge.first]}, {edge.second, positions[edge.second]}, edge_length);
    }
    
    graphCreated_ = true;
}

void MeshGraph::process() {
    // Create graph if not created
    if (!graphCreated_) {
        createGraph();
    }
    
    // Set outport data in case no filtering should be done
    if (!filterVertices_ && !filterEdges_) {
        outport_.setData(inport_.getData()->clone());
    }
    
    if (filterVertices_) {
        
        // Bind filter function to search condition, then do BFS on graph
        graph_.search_condition = std::bind(&MeshGraph::filterPos, this, std::placeholders::_1);
        NGraph::tGraph<int, vec3, double> B = graph_.BFS_vertex();
        
        // Create index buffer from new graph
        std::vector<std::uint32_t> indexMeshData;
        std::vector<int> newIndices = B.get_graph_as_vertex_list();
        
        for (auto& ni : newIndices) {
            indexMeshData.push_back(static_cast<std::uint32_t>(ni));
        }
        auto indexBuff = util::makeIndexBuffer(std::move(indexMeshData));
        
        // Create position buffer (vertex data)
        std::vector<vec3> positions(positions_);
        auto posBuff = util::makeBuffer(std::move(positions));

        
        // Create mesh from buffers
        // NOTE: should be moved when edges can be filtered
        inviwo::Mesh* result = new Mesh(DrawType::Lines, ConnectivityType::None);
        result->addBuffer(BufferType::PositionAttrib, posBuff);
        result->addIndicies(Mesh::MeshInfo(DrawType::Lines, ConnectivityType::None), indexBuff);
        
        outport_.setData(result);
    }
    
    if (filterEdges_) {
        // TODO
    }
    
    
}

}  // namespace inviwo
