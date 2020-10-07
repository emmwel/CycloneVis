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

#include <inviwo/cyclonevis/processors/meshaddmetabuffer.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MeshAddMetaBuffer::processorInfo_{
    "org.inviwo.MeshAddMetaBuffer",      // Class identifier
    "Mesh Add Meta Buffer",                // Display name
    "Undefined",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo MeshAddMetaBuffer::getProcessorInfo() const { return processorInfo_; }

MeshAddMetaBuffer::MeshAddMetaBuffer()
    : Processor()
    , meshInport_("meshInport")
    , volumeInport_("volumeInport")
    , meshOutport_("meshOutport") {

    addPort(meshInport_);
    addPort(volumeInport_);
    addPort(meshOutport_);
}

ivec3 MeshAddMetaBuffer::getVoxelIndexFromPosition(const dvec3& position) {

    // go from world to model coordinates
    dvec3 modelCoords = dvec3(glm::inverse(inVolume_->getModelMatrix()) * dvec4(position, 1));

    // From BatchVolumeSampler.h //
    ivec3 voxelPos = ivec3(glm::floor(modelCoords * dvec3(dims_)));

    // Function which takes modulus within bounds (from batchvolumesampler by Martin)
    auto modInt = [](int a, int b) {
        int result = a % b;
        return result + ((result >> 31) & b);
    };

    size3_t indexPos;

    // Make sure index is within dimensions using modInt
    for (int i = 0; i < 3; i++) {
        indexPos[i] = modInt(voxelPos[i], dims_[i]);
    }

    return indexPos;

}


void MeshAddMetaBuffer::process() {
    if (!meshInport_.isReady() || !volumeInport_.isReady())
        return;

    // Get inport volume size
    inVolume_ = volumeInport_.getData()->clone();
    dims_ = ivec3(inVolume_->getDimensions());

    // Get inport mesh
    std::shared_ptr<Mesh> mesh(meshInport_.getData()->clone());

    // Get position buffer
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();

    // Create volume accesser
    auto volumeDataAccesser = inVolume_->getEditableRepresentation<VolumeRAM>();

    // Get range of data
    dvec2 dataRange = inVolume_->dataMap_.dataRange;

    std::vector<float> metaVals;

    // Get min and max values
    for (unsigned long i = 0; i < positions.size(); i++) {
        vec3 p = positions[i];
        if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
            // do nothing
            metaVals.push_back(0.f);
            continue;
        }
        else {
            // Get voxel index from mesh position
            ivec3 voxelIndex = getVoxelIndexFromPosition(p);

            // Map the voxel value to the mesh texCoord scalar's u(s?)-value
            double voxelVal = volumeDataAccesser->getAsDouble(voxelIndex);

            // map value to [0, 1] u-coords range
            double mappedValue = (voxelVal - dataRange[0]) / (dataRange[1] - dataRange[0]);
            metaVals.push_back(mappedValue);

        }
    }

    mesh->addBuffer(BufferType::ScalarMetaAttrib, util::makeBuffer(std::move(metaVals)));


    meshOutport_.setData(mesh);
}

}  // namespace inviwo
