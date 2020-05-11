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

#include <inviwo/cyclonevis/processors/floodfillvolume.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo FloodFillVolume::processorInfo_{
    "org.inviwo.FloodFillVolume",      // Class identifier
    "Flood Fill Volume",                // Display name
    "CycloneVis",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo FloodFillVolume::getProcessorInfo() const { return processorInfo_; }

FloodFillVolume::FloodFillVolume()
    : Processor()
    , meshInport_("meshInport")
    , volumeInport_("volumeInport")
    , volumeOutport_("volumeOutport")
    {

    addPort(meshInport_);
    addPort(volumeInport_);
    addPort(volumeOutport_);

}

size3_t getVoxelIndexFromPosition(const dvec3& position, const dvec3& dims) {
    ivec3 voxelPos = ivec3(glm::floor(position * dims));
    ivec3 dimsi = ivec3(dims);
    
    auto modInt = [](int a, int b) {
        int result = a % b;
        //int result = a - b * (a / b);
        // take care of negative values
        //return result + (result < 0 ? b : 0);
        return result + ((result >> 31) & b);
    };
    
    size3_t indexPos;
    
    for (int i = 0; i < 3; i++) {
        indexPos[i] = modInt(voxelPos[i], dimsi[i]);
    }
    
    return indexPos;
}

void FloodFillVolume::floodFill(ivec3 index, double offset, ivec3 dims) {
    
    std::vector<bool> visited(dims[0] * dims[1] * dims[2], false);
    
    std::list<ivec3> queue;
    auto inVolumeDataAccesser = inVolume_->getRepresentation<VolumeRAM>();
    
    queue.push_back(index);
    
    double isoValue = inVolumeDataAccesser->getAsDouble(index);
    std::cout << isoValue << std::endl;
    
    while(!queue.empty()) {
        ivec3 curIndex = queue.front();
        double voxelVal = 0;
        int indexInt = 0;
        queue.pop_front();
        
        // Check nearby voxels in X-dim
        ivec3 indexMinusXDim = {curIndex - ivec3(1, 0, 0)};
        indexInt = indexMinusXDim[0] + (dims[0] * indexMinusXDim[1]) + (dims[0] * dims[1] * indexMinusXDim[2]);
        voxelVal = inVolumeDataAccesser->getAsDouble(indexMinusXDim);
        
        if (!visited[indexInt] && indexMinusXDim[0] > -1 && (voxelVal - isoValue) < offset ) {
            visited[indexInt] = true;
            std::cout << voxelVal << std::endl;
            std::cout << indexMinusXDim << std::endl;
        }
        
        ivec3 indexAddXDim = {curIndex + ivec3(1, 0, 0)};
        voxelVal = inVolumeDataAccesser->getAsDouble(indexAddXDim);
        
        if (indexAddXDim[0] < (dims[0] - 1) && (voxelVal - isoValue) < offset ) {
            std::cout << voxelVal << std::endl;
            std::cout << indexAddXDim << std::endl;
        }

        ivec3 indexMinusYDim = {curIndex - ivec3(0, 1, 0)};
        voxelVal = inVolumeDataAccesser->getAsDouble(indexMinusYDim);
        
        if (indexMinusYDim[1] > -1 && (voxelVal - isoValue) < offset) {
            std::cout << voxelVal << std::endl;
            std::cout << indexMinusYDim << std::endl;
        }
        
        ivec3 indexAddYDim = {curIndex + ivec3(0, 1, 0)};
        voxelVal = inVolumeDataAccesser->getAsDouble(indexAddYDim);
        
        if (indexAddYDim[1] < (dims[1] - 1) && (voxelVal - isoValue) < offset ) {
            std::cout << voxelVal << std::endl;
            std::cout << indexAddYDim << std::endl;
        }

        ivec3 indexMinusZDim = {curIndex - ivec3(0, 0, 1)};
        voxelVal = inVolumeDataAccesser->getAsDouble(indexMinusZDim);
        
        if (indexMinusZDim[2] > -1 && (voxelVal - isoValue) < offset ) {
            std::cout << voxelVal << std::endl;
            std::cout << indexMinusZDim << std::endl;
        }
        
        ivec3 indexAddZDim = {curIndex + ivec3(0, 0, 1)};
        voxelVal = inVolumeDataAccesser->getAsDouble(indexAddZDim);
        
        if (indexAddZDim[2] < (dims[2] - 1) && (voxelVal - isoValue) < offset ) {
            std::cout << voxelVal << std::endl;
            std::cout << indexAddZDim << std::endl;
        }
//
//        std::cout << indexMinusXDim << std::endl;
//        std::cout << indexAddXDim << std::endl;
//        std::cout << indexMinusYDim << std::endl;
//        std::cout << indexAddYDim << std::endl;
//        std::cout << indexMinusZDim << std::endl;
//        std::cout << indexAddZDim << std::endl;
        break;
    }
}

void FloodFillVolume::process() {
    
    if (!meshInport_.isReady() || !volumeInport_.isReady())
        return;
    
    // Get inport volume size
    inVolume_ = volumeInport_.getData()->clone();
    size3_t dims = inVolume_->getDimensions();
    
    // Create output volume and set size to inport volume
    outVolume_ = std::make_shared<Volume>(Volume(dims));
    outVolume_->setBasis(inVolume_->getBasis());
    outVolume_->setOffset(inVolume_->getOffset());
    
    // Get inport mesh positions
    std::shared_ptr<Mesh> mesh(meshInport_.getData()->clone());
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    
//    for(auto& p : positions)
//        std::cout << getVoxelIndexFromPosition(p, dims) << std::endl;
    
    floodFill(getVoxelIndexFromPosition(positions[0], dims), 1000, dims);
    
    // Get volume data
//    const util::IndexMapper<3, int> im(dims);
//    auto vrprecision = inVolume_->getRepresentation<VolumeRAM>()->getAsDouble(size3_t(0, 0, 0));
//    std::cout << vrprecision << std::endl;
    volumeOutport_.setData(outVolume_);
}

}  // namespace inviwo
