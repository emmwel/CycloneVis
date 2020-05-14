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

#include <inviwo/cyclonevis/processors/cyclonecontourlines.h>

#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/datastructures/image/imageram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo CycloneContourLines::processorInfo_{
    "org.inviwo.CycloneContourLines",      // Class identifier
    "Cyclone Contour Lines",                // Display name
    "CycloneVis",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo CycloneContourLines::getProcessorInfo() const { return processorInfo_; }

CycloneContourLines::CycloneContourLines()
    : Processor()
    , meshInport_("meshInport")
    , volumeInport_("volumeInport")
    , meshOutport_("meshOutport"){

    addPort(meshInport_);
    addPort(volumeInport_);
    addPort(meshOutport_);
}

ivec3 CycloneContourLines::getVoxelIndexFromPosition(const dvec3& position, const dvec3& dims) {
    
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
    
    ivec3 indexPos;
    
    // Make sure index is within dimensions using modInt
    for (int i = 0; i < 3; i++) {
        indexPos[i] = modInt(voxelPos[i], dimsi[i]);
    }
    
    return indexPos;
}

void CycloneContourLines::process() {
    if (!meshInport_.isReady() || !volumeInport_.isReady())
        return;
    
    // Get inport volume and its' size
    inVolume_ = volumeInport_.getData()->clone();
    ivec3 dims = inVolume_->getDimensions();
    
    // Get inport mesh positions
    std::shared_ptr<Mesh> mesh(meshInport_.getData()->clone());
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    
    // Get index from position
    ivec3 index = getVoxelIndexFromPosition(positions[0], dims);
    
    // Get time value (slice)
    size_t timeSlice = index.z;
    
    // Use time value to slice volume and get an image
    auto image =
    volumeInport_.getData()->getRepresentation<VolumeRAM>()
        ->dispatch<std::shared_ptr<Image>, dispatching::filter::All>(
            [slice = timeSlice,
             &cache = imageCache_](const auto vrprecision) {
                using T = util::PrecisionValueType<decltype(vrprecision)>;

                const T* voldata = vrprecision->getDataTyped();
                const auto voldim = vrprecision->getDimensions();

                const auto imgdim = size2_t(voldim.x, voldim.y);

                auto res = cache.getTypedUnused<T>(imgdim);
                auto sliceImage = res.first;
                auto layerrep = res.second;
                auto layerdata = layerrep->getDataTyped();

                switch (util::extent<T, 0>::value) {
                    case 0:  // util::extent<T, 0>::value returns zero for non-glm types
                    case 1:
                        layerrep->setSwizzleMask({{ImageChannel::Red, ImageChannel::Red,
                                                   ImageChannel::Red, ImageChannel::One}});
                        break;
                    case 2:
                        layerrep->setSwizzleMask({{ImageChannel::Red, ImageChannel::Green,
                                                   ImageChannel::Zero, ImageChannel::One}});
                        break;
                    case 3:
                        layerrep->setSwizzleMask({{ImageChannel::Red, ImageChannel::Green,
                                                   ImageChannel::Blue, ImageChannel::One}});
                        break;
                    default:
                    case 4:
                        layerrep->setSwizzleMask({{ImageChannel::Red, ImageChannel::Green,
                                                   ImageChannel::Blue, ImageChannel::Alpha}});
                }
  
                auto z = glm::clamp(slice, size_t{0}, voldim.z - 1);
                const size_t dataSize = voldim.x * voldim.y;
                const size_t initialStartPos = z * voldim.x * voldim.y;

                std::copy(voldata + initialStartPos,
                          voldata + initialStartPos + dataSize, layerdata);

                cache.add(sliceImage);
                return sliceImage;
            });
    
    // Get iso-value and create a boundary around it
    auto inVolumeDataAccesser = inVolume_->getRepresentation<VolumeRAM>();
    double isoValue = inVolumeDataAccesser->getAsDouble(index);
    double boundary = isoValue + 250.0;
    
    // Use image contour algorithm with iso-value on image
    std::shared_ptr<Mesh> meshOut;
    meshOut = ImageContour::apply(image->getColorLayer()->getRepresentation<LayerRAM>(),
                                  0, boundary, vec4(1));
    
    // Transform mesh into volume basis
    auto posBufferOut = static_cast<Buffer<vec3>*>(meshOut->getBuffer(BufferType::PositionAttrib));
    auto positionsAccesser = posBufferOut->getEditableRAMRepresentation();
    auto positionsOut = positionsAccesser->getDataContainer();
    
    std::vector<dvec3> transformedPosOut(positionsOut.size());
    mat3 basis = inVolume_->getBasis();
    vec3 offset = inVolume_->getOffset();
    for (unsigned long i = 0; i < transformedPosOut.size(); i++) {
        transformedPosOut[i] = basis * positionsOut[i] + offset;
        positionsAccesser->setFromDVec3(i, transformedPosOut[i]);
    }
    
    meshOutport_.setData(meshOut);
}

}  // namespace inviwo
