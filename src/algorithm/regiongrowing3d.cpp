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

#include <inviwo/cyclonevis/algorithm/regiongrowing3d.h>

namespace inviwo {

namespace regiongrowing3D {

	bool withinDimensions(const ivec3& index, const ivec3& dims) {
		return glm::all(glm::greaterThanEqual(index, ivec3(0))) && glm::all(glm::lessThan(index, dims));
        //return true;
    }

    std::pair<double, double> standardDeviationAroundSeed(
        Volume* inVolume,
        Volume* gradientMagVolume,
        const ivec3& seedVoxel
    ) {
        auto gradientVolumeDataAccesser = gradientMagVolume->getEditableRepresentation<VolumeRAM>();

        return inVolume->getRepresentation<VolumeRAM>()->dispatch<std::pair<double, double>, dispatching::filter::Scalars>(
        [&gradientVolumeDataAccesser, seedVoxel](auto inputVolumeRAM) {
            // ValueType is the concrete type of the volume, e.g. int, float
            using ValueType = util::PrecisionValueType<decltype(inputVolumeRAM)>;
            const size3_t dims = inputVolumeRAM->getDimensions();

            // Indexmapper mapping 3D index to 1D
            inviwo::util::IndexMapper3D map3DIndexto1D(dims);

            // Get input data
            const ValueType* valuesData = inputVolumeRAM->getDataTyped();

            auto gradientMagRamPrecision = static_cast<VolumeRAMPrecision<typename ValueType>*>(gradientVolumeDataAccesser);
            ValueType* gradientMagData = gradientMagRamPrecision->getDataTyped();

            // Get all neighbor values
            std::vector<ValueType> neighborsValues;
            std::vector<ValueType> neighborsGradientMagnitude;

            for (unsigned long i = 0; i < offsets.size(); i++) {
                ivec3 neighborVoxel{ seedVoxel + offsets[i] };

                if (withinDimensions(neighborVoxel, dims)) {
                    size_t neighborIndex1D = map3DIndexto1D(neighborVoxel);
                    neighborsValues.push_back(valuesData[neighborIndex1D]);
                    neighborsGradientMagnitude.push_back(gradientMagData[neighborIndex1D]);
                }
            }

            // Calculate mean
            ValueType meanValue = std::accumulate(neighborsValues.begin(), neighborsValues.end(), 0.0) / neighborsValues.size();
            ValueType meanGradMag = std::accumulate(neighborsGradientMagnitude.begin(), neighborsGradientMagnitude.end(), 0.0) / neighborsGradientMagnitude.size();

            // Calculate difference
            std::vector<ValueType> diffValue(neighborsValues.size());
            std::vector<ValueType> diffGradMag(neighborsGradientMagnitude.size());

            std::transform(neighborsValues.begin(), neighborsValues.end(), diffValue.begin(), [meanValue](double x) { return x - meanValue; });
            std::transform(neighborsGradientMagnitude.begin(), neighborsGradientMagnitude.end(), diffGradMag.begin(), [meanGradMag](double x) { return x - meanGradMag; });

            // Sum differences, divide by the size and take square root to get the standard deviation
            ValueType sqSumValues = std::inner_product(diffValue.begin(), diffValue.end(), diffValue.begin(), 0.0);
            ValueType sqSumGradMag = std::inner_product(diffGradMag.begin(), diffGradMag.end(), diffGradMag.begin(), 0.0);

            ValueType stdevValues = std::sqrt(sqSumValues / neighborsValues.size());
            ValueType stdevGradMag = std::sqrt(sqSumGradMag / neighborsGradientMagnitude.size());

            std::pair<double, double> output = std::make_pair(stdevValues, stdevGradMag);
            return output;
        });
    }

	void floodFill(
		Volume* inVolume,
		std::shared_ptr<Volume> outVolume,
		const ivec3& seedVoxel,
		double boundary,
		double& minVal,
		double& maxVal
	) {
        auto outVolumeDataAccesser = outVolume->getEditableRepresentation<VolumeRAM>();

        // Prep to set value range of output volume
        minVal = std::min(minVal, std::numeric_limits<double>::max());
        maxVal = std::max(maxVal, std::numeric_limits<double>::lowest());


        // Generate the output volume using dispatch for generalized data types
        inVolume->getRepresentation<VolumeRAM>()->dispatch<  void, dispatching::filter::Scalars>(
            [&outVolumeDataAccesser, seedVoxel, boundary, &minVal, &maxVal]
        (auto inputVolumeRAM) {
                // ValueType is the concrete type of the volume, e.g. int, float
                using ValueType = util::PrecisionValueType<decltype(inputVolumeRAM)>;
                const size3_t dims = inputVolumeRAM->getDimensions();

                // Indexmapper mapping 3D index to 1D
                inviwo::util::IndexMapper3D map3DIndexto1D(dims);

                // Get input data
                const ValueType* inputVolumeData = inputVolumeRAM->getDataTyped();

                // Create a representation of same type as the input volume
                auto outputRamPrecision = static_cast<VolumeRAMPrecision<typename ValueType>*>(outVolumeDataAccesser);
                ValueType* outVolumeData = outputRamPrecision->getDataTyped();

                // Create queue
                std::list<ivec3> queue;
                queue.push_back(seedVoxel);

                // Get isoValue
                size_t isoIndex1D = map3DIndexto1D(seedVoxel);
                ValueType isoValue = inputVolumeData[isoIndex1D];
                outVolumeData[isoIndex1D] = 0.0;

                while (!queue.empty()) {
                    ivec3 curIndex = queue.front();
                    queue.pop_front();

                    for (unsigned long i = 0; i < offsets.size(); i++) {
                        ivec3 neighborIndex{ curIndex + offsets[i] };

                        // Grow across x-boundary since they are the same points on the world
                        neighborIndex.x = neighborIndex.x % dims.x;

                        // Check that neighbor voxel is within dimensions
                        if (withinDimensions(neighborIndex, dims)) {
                            //double neighborVal = inVolumeDataAccesser->getAsDouble(neighborIndex);
                            size_t neighborIndex1D = map3DIndexto1D(neighborIndex);
                            ValueType neighborVal = inputVolumeData[neighborIndex1D];
                            ValueType compValue = std::abs(static_cast<double>(neighborVal) - isoValue);

                            // Check value is within boundary
                            if (compValue < boundary) {

                                // Only add to queue and change value if voxel has not been accessed before
                                if (outVolumeData[neighborIndex1D] > boundary) {
                                    // Check if the valueRange has been changed
                                    minVal = std::min(minVal, static_cast<double>(neighborVal));
                                    maxVal = std::max(maxVal, static_cast<double>(neighborVal));

                                    // Set voxel value of output index as diff with iso value
                                    outVolumeData[neighborIndex1D] = compValue;

                                    // Add to queue
                                    queue.push_back(neighborIndex);
                                }
                            }

                            else {
                                // Set values for boundary points
                                outVolumeData[neighborIndex1D] = compValue;
                            }
                        }
                    }
                }

            }
        );
	}

}

}  // namespace inviwo
