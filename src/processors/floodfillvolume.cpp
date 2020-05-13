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

size3_t FloodFillVolume::getVoxelIndexFromPosition(const dvec3& position, const dvec3& dims) {
    
    // go from world to model coordinates
    dvec3 modelCoords = dvec3(glm::inverse(inVolume_->getModelMatrix())* dvec4(position, 1));
    
    // From BatchVolumeSampler.h //
    ivec3 voxelPos = ivec3(glm::floor(modelCoords * dims));
    ivec3 dimsi = ivec3(dims);
    
    // Function which takes modulus within bounds
    auto modInt = [](int a, int b) {
        int result = a % b;
        return result + ((result >> 31) & b);
    };
    
    size3_t indexPos;
    
    // Make sure index is within dimensions using modInt
    for (int i = 0; i < 3; i++) {
        indexPos[i] = modInt(voxelPos[i], dimsi[i]);
    }
    
    return indexPos;
}

void FloodFillVolume::floodFill(ivec3 index, double boundary, ivec3 dims) {
    
    // std::vector<bool> visited(dims[0] * dims[1] * dims[2], false);
    
    // Get acces to data
    auto inVolumeDataAccesser = inVolume_->getRepresentation<VolumeRAM>();
    auto outVolumeDataAccesser = outVolume_->getEditableRepresentation<VolumeRAM>();
    
    // Create queue
    std::list<ivec3> queue;
    queue.push_back(index);
    
    // Get isoValue
    double isoValue = inVolumeDataAccesser->getAsDouble(index);
    outVolumeDataAccesser->setFromDouble(index, 0.0);
    
    // Get offset used to get indices of nearby voxels
    const std::array<ivec3, 8> offsets = {{{-1, 0, 0},
                                           {1, 0, 0},
                                           {0, -1, 0},
                                           {0, 1, 0},
                                           {-1, -1, 0},
                                           {1, -1, 0},
                                           {1, 1, 0},
                                           {-1, 1, 0}}};
    
    // Dimension check
    ivec3 vsse = voxelSearchSpaceExtent_;
    auto withinSearchSpace = [vsse, index] (ivec3 i) {
        ivec3 lowerBound{index - vsse};
        ivec3 upperBound{index + vsse};
        return glm::all(glm::greaterThan(i, lowerBound)) && glm::all(glm::lessThan(i, upperBound));
    };
    
    while(!queue.empty()) {
        ivec3 curIndex = queue.front();
        queue.pop_front();
//        std::cout << "current: " << curIndex << std::endl;
        for (unsigned long i = 0; i < offsets.size(); i++) {
            ivec3 neighborIndex{curIndex + offsets[i]};
//            std::cout << neighborIndex << std::endl;
            // Check that neighbor voxel is within dimensions
            if (withinSearchSpace(neighborIndex)) {
                double neighborVal = inVolumeDataAccesser->getAsDouble(neighborIndex);
                double compValue = neighborVal - isoValue;
                
                // Check value is within boundary
                if ( compValue < boundary && compValue > 0.0) {
                    
                    // Only add to queue and change value if voxel has not been accessed before
                    if (outVolumeDataAccesser->getAsDouble(neighborIndex) > boundary) {

                        // Set voxel value of output index as diff with iso value
                        outVolumeDataAccesser->setFromDouble(neighborIndex, compValue);
//                        std::cout << outVolumeDataAccesser->getAsDouble(neighborIndex) << std::endl;
                        // Add to queue
                        queue.push_back(neighborIndex);
                    }
                }
            }
            
        }

    }
//    std::cout << "adios" << std::endl;
}

void FloodFillVolume::process() {
    
    if (!meshInport_.isReady() || !volumeInport_.isReady())
        return;
    
    // Get inport volume size
    inVolume_ = volumeInport_.getData()->clone();
    size3_t dims = inVolume_->getDimensions();
    
    // Create output volume and set size to inport volume
    outVolume_ = std::make_shared<Volume>(Volume(dims, inVolume_->getDataFormat()));
    outVolume_->setBasis(inVolume_->getBasis());
    outVolume_->setOffset(inVolume_->getOffset());
    
    // Set all output voxel values to max possible number
    auto outVolumeDataAccesser = outVolume_->getEditableRepresentation<VolumeRAM>();
    double maxDouble = std::numeric_limits<double>::max();
    
    // Loop through all voxels and set to max value
    for (unsigned long i = 0; i < dims.x; i++) {
        for (unsigned long j = 0; j < dims.y; j++) {
            for (unsigned long k = 0; k < dims.z; k++) {
                outVolumeDataAccesser->setFromDouble(size3_t(i, j, k), maxDouble);
            }
        }
    }

    // Get inport mesh positions
    std::shared_ptr<Mesh> mesh(meshInport_.getData()->clone());
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    
//    for(auto& p : positions)
//        std::cout << getVoxelIndexFromPosition(p, dims) << std::endl;
    
    for(auto& pos : positions) {
        floodFill(getVoxelIndexFromPosition(pos, dims), 500.0, dims);
    }
    
    volumeOutport_.setData(outVolume_);
}

}  // namespace inviwo
