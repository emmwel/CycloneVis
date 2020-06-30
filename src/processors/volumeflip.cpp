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

#include <inviwo/cyclonevis/processors/volumeflip.h>

#include <inviwo/core/util/volumeramutils.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/templatesampler.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeFlip::processorInfo_{
    "org.inviwo.VolumeFlip",      // Class identifier
    "Volume Flip",                // Display name
    "CycloneVis",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo VolumeFlip::getProcessorInfo() const { return processorInfo_; }

VolumeFlip::VolumeFlip()
    : Processor()
	, inport_("inport")
    , outport_("outport")
	, axislist_("axislist", "Flip Axis",
		{
		  {"xAxis", "Flip X Values", ValuesToFlip::XVals},
		  {"yAxis", "Flip Y Values", ValuesToFlip::YVals},
		  {"zAxis", "Flip Z Values", ValuesToFlip::ZVals}
		}, 0) {

	addPort(inport_);
    addPort(outport_);
	addProperty(axislist_);
}

void VolumeFlip::process() {
	// Get input volume data
	auto inVolume = inport_.getData();

	// Create output volume
	// Function returns a volume
	// dispatching::filter::All covers all possible data types
	auto outVolume =
		inVolume->getRepresentation<VolumeRAM>()
		->dispatch<std::shared_ptr<Volume>, dispatching::filter::All>([&](const auto vrprecision) {
			// Get value type, ex. double, vec2, vec3 etc
			using ValueType = util::PrecisionValueType<decltype(vrprecision)>;

			// Get data
			const ValueType* inVolumeData = vrprecision->getDataTyped();
			const auto dims = vrprecision->getDimensions();

			// Define volume to return, Rep is used for accessing the data
			auto newVolumeRep = std::make_shared<VolumeRAMPrecision<ValueType>>(dims);
			std::shared_ptr<Volume> newVolume = std::make_shared<Volume>(newVolumeRep);
			auto outVolumeData = newVolumeRep->getDataTyped();

			// Since getDataTyped() is an array of the volume data,
			// Use index mapper to go from voxel index to array index
			util::IndexMapper3D index(dims);

			// Go through all voxels
			util::forEachVoxel(*vrprecision, [&](const size3_t& inVoxel) {
				size3_t outVoxel = inVoxel;

				// Check along which axis values are switched, then switch voxel index for that axis
				switch (axislist_) {
					case ValuesToFlip::XVals: {
						outVoxel = size3_t(dims.x - 1 - inVoxel.x, inVoxel.y, inVoxel.z);
						break;
					}
					case ValuesToFlip::YVals: {
						outVoxel = size3_t(inVoxel.x, dims.y - 1 - inVoxel.y, inVoxel.z);
						break;
					}
					case ValuesToFlip::ZVals: {
						outVoxel = size3_t(inVoxel.x, inVoxel.y, dims.z - 1 - inVoxel.z);
						break;
					}
					default:
						break;
				}
				
				// Set output data 
				outVolumeData[index(outVoxel)] = inVolumeData[index(inVoxel)];
			});

			return newVolume;
	});

	outport_.setData(outVolume);
}

}  // namespace inviwo
