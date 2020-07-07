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

#include <inviwo/cyclonevis/processors/vectorfieldonspheretransformation.h>
#include <inviwo/cyclonevis/util/coordinatetransformations.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VectorFieldOnSphereTransformation::processorInfo_{
    "org.inviwo.VectorFieldOnSphereTransformation",      // Class identifier
    "Vector Field On Sphere Transformation",                // Display name
    "CycloneVis",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo VectorFieldOnSphereTransformation::getProcessorInfo() const { return processorInfo_; }

VectorFieldOnSphereTransformation::VectorFieldOnSphereTransformation()
    : Processor()
	, inport_("inport")
    , outport_("outport") {

	addPort(inport_);
    addPort(outport_);
}

void VectorFieldOnSphereTransformation::process() {
    // outport_.setData(myImage);

	// Load input volume
	Volume* inVolume = inport_.getData()->clone();
	ivec3 dims = ivec3(inVolume->getDimensions());

	// Create output volume and set size to inport volume
	std::shared_ptr<Volume> outVolume = std::make_shared<Volume>(Volume(dims, inVolume->getDataFormat()));
	outVolume->setBasis(inVolume->getBasis());
	outVolume->setOffset(inVolume->getOffset());

	// Create data accessers
	auto inVolumeDataAccesser = inVolume->getRepresentation<VolumeRAM>();
	auto outVolumeDataAccesser = outVolume->getEditableRepresentation<VolumeRAM>();

	// Set dimensions for coord transform
	mat3 basis = inVolume->getBasis();
	vec3 offset = inVolume->getOffset();
	vec2 dimX = vec2(offset.x, offset.x + std::abs(basis[0].x));
	vec2 dimY = vec2(offset.y, offset.y + std::abs(basis[1].y));
	vec2 dimZ = vec2(offset.z, offset.z + std::abs(basis[2].z));

	// Used to define data range
	

	// For each voxel in the volume:
	for (int i = 0; i < dims.x; i++) {
		for (int j = 0; j < dims.y; j++) {
			for (int k = 0; k < dims.z; k++) {
				ivec3 currentVoxel = ivec3(i, j, k);

				// Get vector field in current voxel
				// The vector field has [u, v] coords which are [eastward, northward] aka [long, lat] aka [theta, -phi] direction
				vec2 vectorFieldVals = inVolumeDataAccesser->getAsDVec2(currentVoxel);

				// Calculate the latitude and longitude of the voxel position
				vec2 latLongCoords = coordTransform::cartesianToLatLong(vec2(currentVoxel.x, currentVoxel.y), dimX, dimY);

				// Scale x-component with the latitude
				vec2 vectorFieldMapped = vec2(std::cos(coordTransform::TO_RAD * latLongCoords.x) * vectorFieldVals.x, vectorFieldVals.y);

				outVolumeDataAccesser->setFromDVec2(currentVoxel, vectorFieldMapped);

			}
		}
	}
	// Set data range and value range
	auto minMax = util::volumeMinMax(outVolumeDataAccesser);
	dvec2 channel1(minMax.first.x, minMax.second.x);
	dvec2 channel2(minMax.first.y, minMax.second.y);
	outVolume->dataMap_.dataRange = dvec2(std::min(channel1.x, channel2.x), std::max(channel1.y, channel2.y));
	outVolume->dataMap_.valueRange = inVolume->dataMap_.valueRange;

	outport_.setData(outVolume);
}

}  // namespace inviwo
