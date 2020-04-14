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
    , position_("position", "Position", vec3(0.0f), vec3(-100.0f), vec3(100.0f)) {
    
        addPort(inport_);
    addPort(outport_);
    addProperty(position_);
}

void MeshGraph::process() {
    std::shared_ptr<Mesh> mesh(inport_.getData()->clone());
    
    // Buffers, only works when there is one position buffer and one color buffer
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto colorBuffer = static_cast<Buffer<vec4>*>(mesh->getBuffer(BufferType::ColorAttrib));
    
    // Get data from the buffers
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    auto colors = colorBuffer->getEditableRAMRepresentation()->getDataContainer();
    std::cout << positions.size() << std::endl;
    for(auto& p : positions) {
        std::cout << p << std::endl;
    }
    
    std::cout << colors.size() << std::endl;
    for(auto& c : colors) {
        std::cout << c << std::endl;
    }
    
    // Index buffer
    auto indBuffer = mesh->getIndices(0);
    auto indices = indBuffer->getEditableRAMRepresentation()->getDataContainer();
    
    std::cout << indices.size() << std::endl;
    for(auto& i : indices) {
        std::cout << i << std::endl;
    }
    
    // vector of edges where [int, int] are the vertices connected to that edge
    std::vector<std::pair<int, int>> edges;
    for (int i = 0; i < indices.size() - 1; i += 2) {
        edges.push_back({indices[i], indices[i+1]});
    }
    
    // print for check
    for (auto& e : edges) {
        std::cout << "[" << e.first << ", " << e.second << "]" << std::endl;
    }
    
    
    outport_.setData(mesh);
}

}  // namespace inviwo
