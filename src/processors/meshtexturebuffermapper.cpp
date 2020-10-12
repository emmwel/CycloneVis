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

#include <inviwo/cyclonevis/processors/meshtexturebuffermapper.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MeshTextureBufferMapper::processorInfo_{
    "org.inviwo.MeshTextureBufferMapper",      // Class identifier
    "Mesh Texture Buffer Mapper",                // Display name
    "CycloneVis",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo MeshTextureBufferMapper::getProcessorInfo() const { return processorInfo_; }

MeshTextureBufferMapper::MeshTextureBufferMapper()
    : Processor()
    , meshInport_("meshInport")
    , volInport_("volumeInport")
    , meshOutport_("meshOutport")
    , mapBasedOnVolume_("mapBasedOnVolume", "Map Based On Volume", true)
    , inVolumeDataRange_("inVolumeDataRange_", "Input Volume Data Range", vec2{0.0f}, vec2{std::numeric_limits<float>::lowest()}, vec2{std::numeric_limits<float>::max()}, vec2{0.01}, InvalidationLevel::Valid, PropertySemantics::Text)
	, surfaceValueRange_("surfaceValueRange_", "Surface Value Range", vec2{ 0.0f }, vec2{ std::numeric_limits<float>::lowest() }, vec2{ std::numeric_limits<float>::max() }, vec2{ 0.01 }, InvalidationLevel::Valid, PropertySemantics::Text)
{

    addPort(meshInport_);
    addPort(volInport_);
    addPort(meshOutport_);
    
    addProperties(mapBasedOnVolume_, inVolumeDataRange_, surfaceValueRange_);
    
    // Do not allow value range to be changed in properties menu
	inVolumeDataRange_.setReadOnly(true);
	surfaceValueRange_.setReadOnly(true);
    
    volInport_.onChange([this](){
		inVolumeDataRange_.set(volInport_.getData()->dataMap_.dataRange);
    });
}

ivec3 MeshTextureBufferMapper::getVoxelIndexFromPosition(const dvec3& position) {
    
    // go from world to model coordinates -- NO, already in volume model coordinates
    //dvec3 modelCoords = dvec3(glm::inverse(inVolume_->getModelMatrix())* dvec4(position, 1.0));
	dvec3 modelCoords = dvec3(position);
    
    // From BatchVolumeSampler.h //
    ivec3 voxelPos = ivec3(glm::floor(modelCoords * dvec3(dims_)));
    
    size3_t indexPos;
    
    // Make sure index is within dimensions using modInt
    for (int i = 0; i < 3; i++) {
        indexPos[i] = voxelPos[i] % dims_[i];
    }
    
    return indexPos;
}

void MeshTextureBufferMapper::process() {
    if (!meshInport_.isReady() || !volInport_.isReady())
        return;
    
    // Get inport volume size
    inVolume_ = volInport_.getData()->clone();
    dims_ = ivec3(inVolume_->getDimensions());
    
    // Get inport mesh
    std::shared_ptr<Mesh> mesh(meshInport_.getData()->clone());

    // Get buffers
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto texBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::TexcoordAttrib));
    
    auto texCoords = texBuffer->getEditableRAMRepresentation()->getDataContainer();
    auto changeTexCoordsAcceser = texBuffer->getEditableRAMRepresentation();
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    
    // Create volume accesser
    auto volumeDataAccesser = inVolume_->getEditableRepresentation<VolumeRAM>();

    // Get range of data
    dvec2 dataRange = inVolume_->dataMap_.dataRange;
    double minVal = std::numeric_limits<double>::max();
    double maxVal = std::numeric_limits<double>::lowest();
    
    // Get min and max values
    for (unsigned long i = 0; i < positions.size(); i++) {
        vec3 p = positions[i];
        if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
            // do nothing
            continue;
        }
        else {
            // Get voxel index from mesh position
            ivec3 voxelIndex = getVoxelIndexFromPosition(p);
            
            // Check if voxel value is either min or max of the surface
            double voxelVal = volumeDataAccesser->getAsDouble(voxelIndex);
            minVal = std::min(minVal, voxelVal);
            maxVal = std::max(maxVal, voxelVal);
        }
    }

	surfaceValueRange_.set(vec2(minVal, maxVal));

    // Assign texture values
    for (unsigned long i = 0; i < positions.size(); i++) {
        vec3 p = positions[i];
        if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
            changeTexCoordsAcceser->setFromDVec3(i, util::glm_convert<vec3>(vec3(0, 0, 0)));
            continue;
        }
        else {
            // Get voxel index from mesh position
            ivec3 voxelIndex = getVoxelIndexFromPosition(p);
            double voxelVal = volumeDataAccesser->getAsDouble(voxelIndex);

            // Map u-coord to [0, 1] on either volume or surface data range
            double mappedValue = 0.0;
            if (mapBasedOnVolume_.get()) {
                mappedValue = (voxelVal - dataRange[0]) / (dataRange[1] - dataRange[0]);
            }
            else {
                mappedValue = (voxelVal - minVal) / (maxVal - minVal);
            }

            texCoords[i] = util::glm_convert<vec3>(vec3(mappedValue, texCoords[i].y, texCoords[i].z));
            changeTexCoordsAcceser->setFromDVec3(i, texCoords[i]);
        }
    }

    
    meshOutport_.setData(mesh);
}

}  // namespace inviwo
