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
    , filterVertices_("filter_vertices", "Filter Vertices", false)
    , filterEdges_("filter_edges", "Filter Edges", false)
    , graphCreated_(false){
    
    addPort(inport_);
    addPort(outport_);
    addProperties(filterVertices_, filterEdges_);
        
    inport_.onChange([this]() {
        graphCreated_ = false;
    });
        
}

bool isXLessThan0(const vec3& v) {
    return v.x > 0.0;
}

void MeshGraph::createGraph() {
    // Get input mesh to get data for graph
    std::shared_ptr<Mesh> mesh(inport_.getData()->clone());
    
    // Buffers, only works when there is one position buffer and one color buffer
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto colorBuffer = static_cast<Buffer<vec4>*>(mesh->getBuffer(BufferType::ColorAttrib));
    
    // Get data from the buffers
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    positions_ = positions;
    auto colors = colorBuffer->getEditableRAMRepresentation()->getDataContainer();
    
    // Index buffer
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
        // Insert edge and set vertex data as vertex position
        graph_.insert_edge({edge.first, positions[edge.first]}, {edge.second, positions[edge.second]}, 0.1 * i);
    }
    
    // print graph
    //graph_.print();
    
    graphCreated_ = true;
}

void MeshGraph::process() {
    // Create graph if not created
    if (!graphCreated_) {
        createGraph();
    }
    
    // Set outport data in case
    if (!filterVertices_ && !filterEdges_) {
        outport_.setData(inport_.getData()->clone());
    }
    
    if (filterVertices_) {
        // test bfs on vertices
        std::cout << "h" << std::endl;
        NGraph::tGraph<int, vec3, double> B = graph_.BFS_vertex(0, isXLessThan0);
        //B.print();
        
        // create index buffer from new graph
        std::vector<std::uint32_t> indexMeshData;
        std::vector<int> newIndices = B.get_graph_as_vertex_list();
        
        for (auto& ni : newIndices) {
            indexMeshData.push_back(static_cast<std::uint32_t>(ni));
        }
        auto indexBuff = util::makeIndexBuffer(std::move(indexMeshData));
        
        // create position buffer (vertex data)
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
