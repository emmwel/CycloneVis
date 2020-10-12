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
    Tags::CPU,               // Tags
};
const ProcessorInfo FloodFillVolume::getProcessorInfo() const { return processorInfo_; }

FloodFillVolume::FloodFillVolume()
    : Processor()
    , meshInport_("meshInport")
    , volumeInport_("volumeInport")
    , gradientMagnitudeVolInport_("gradientMagnitudeVolInport_")
    , volumeOutport_("volumeOutport")
    , method_("method", "Method",
              {
                {"floodfill", "Flood Fill", Method::FloodFill},
                {"regionGrowingValuesBased", "Region Growing Values Based", Method::RegionGrowingValuesBased},
                {"regionGrowingBoundaryBased", "Region Growing Boundary Based", Method::RegionGrowingBoundaryBased},
                {"regionGrowingCombined", "Region Growing Combined", Method::RegionGrowingCombined}
              }, 0)
    , boundary_("boundary", "Boundary", 250.0f, 0.0f, 5000.0f, 0.01f)
	, suggestedIsoVal_("suggestedIsoVal", "Suggested ISO", 250.0f, 0.0f, 5000.0f, 0.01f)
    , k_("k_", "Weighting coefficient k", 1.0f, 0.0f, 10.0f, 0.01f)
    , p_("p_", "Mix coefficient p", 1.0f, 0.0f, 1.0f, 0.01f)
    , valueRange_("valueRange_", "Value Range", vec2{0.0f}, vec2{std::numeric_limits<float>::lowest()}, vec2{std::numeric_limits<float>::max()}, vec2{0.01}, InvalidationLevel::Valid, PropertySemantics::Text)
    {

    addPort(meshInport_);
    addPort(volumeInport_);
    addPort(gradientMagnitudeVolInport_);
    addPort(volumeOutport_);
        
    addProperties(method_, boundary_, k_, p_, valueRange_, suggestedIsoVal_);
    boundary_.setVisible(true);
	suggestedIsoVal_.setReadOnly(true);
    k_.setVisible(false);
    p_.setVisible(false);
    valueRange_.setReadOnly(true);
        
    method_.onChange([this](){
        switch (method_) {
            case Method::FloodFill:
                boundary_.setVisible(true);
                k_.setVisible(false);
                p_.setVisible(false);
                break;
                
            case Method::RegionGrowingValuesBased:
                boundary_.setVisible(false);
                k_.setVisible(true);
                p_.setVisible(false);
                break;
                
            case Method::RegionGrowingBoundaryBased:
                boundary_.setVisible(false);
                k_.setVisible(true);
                p_.setVisible(false);
                break;
            
            case Method::RegionGrowingCombined:
                boundary_.setVisible(false);
                k_.setVisible(true);
                p_.setVisible(true);
                break;
                
            default:
                break;
        }
    });
        
    volumeInport_.onChange([this](){
        valueRange_.set(volumeInport_.getData()->dataMap_.valueRange);
    });
        
    // Create offset indices used for floodfill
    offsets_ = {{
        {-1, 0, 0}, // x
        {1, 0, 0},
        {0, -1, 0}, // y
        {0, 1, 0},
        {0, 0, -1}, // z
        {0, 0, 1},
        {-1, -1, 0}, // middle corners
        {1, -1, 0},
        {1, 1, 0},
        {-1, 1, 0},
        {-1, 0, -1}, // lower midpoints
        {1, 0, -1},
        {0, -1, -1},
        {0, 1, -1},
        {-1, 0, 1}, // upper midpoints
        {1, 0, 1},
        {0, -1, 1},
        {0, 1, 1},
        {-1, -1, -1}, // lower corners
        {1, -1, -1},
        {1, 1, -1},
        {-1, 1, -1},
        {-1, -1, 1}, // upper corners
        {1, -1, 1},
        {1, 1, 1},
        {-1, 1, 1}
    }};
    
    dims_ = ivec3(0, 0, 0);

}

size3_t FloodFillVolume::getVoxelIndexFromPosition(const dvec3& position) {
    
    // go from world to model coordinates
    dvec3 modelCoords = dvec3(glm::inverse(inVolume_->getModelMatrix())* dvec4(position, 1));
    
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

bool FloodFillVolume::withinDimensions(const ivec3& index) const {
    return glm::all(glm::greaterThanEqual(index, ivec3(0))) && glm::all(glm::lessThan(index, dims_));
}

std::pair<double, double> FloodFillVolume::standardDeviationAroundSeed(const ivec3& seedVoxel) {
    // Get all neighbor values
    std::vector<double> neighborsValues;
    std::vector<double> neighborsGradientMagnitude;
    auto inVolumeDataAccesser = inVolume_->getRepresentation<VolumeRAM>();
    auto gradientVolumeDataAccesser = gradientMagnitudeVolInport_.getData()->getRepresentation<VolumeRAM>();
    
    for (unsigned long i = 0; i < offsets_.size(); i++) {
        ivec3 neighborVoxel{seedVoxel + offsets_[i]};
        
        if (withinDimensions(neighborVoxel)) {
            neighborsValues.push_back(inVolumeDataAccesser->getAsDouble(neighborVoxel));
            neighborsGradientMagnitude.push_back(gradientVolumeDataAccesser->getAsDouble(neighborVoxel));
        }
    }
    
    // Calculate mean
    double meanValue = std::accumulate(neighborsValues.begin(), neighborsValues.end(), 0.0) / neighborsValues.size();
    double meanGradMag = std::accumulate(neighborsGradientMagnitude.begin(), neighborsGradientMagnitude.end(), 0.0) / neighborsGradientMagnitude.size();
    
    // Calculate difference
    std::vector<double> diffValue(neighborsValues.size());
    std::vector<double> diffGradMag(neighborsGradientMagnitude.size());
    
    std::transform(neighborsValues.begin(), neighborsValues.end(), diffValue.begin(), [meanValue](double x){ return x - meanValue;});
    std::transform(neighborsGradientMagnitude.begin(), neighborsGradientMagnitude.end(), diffGradMag.begin(), [meanGradMag](double x){ return x - meanGradMag;});
    
    // Sum differences, divide by the size and take square root to get the standard deviation
    double sqSumValues = std::inner_product(diffValue.begin(), diffValue.end(), diffValue.begin(), 0.0);
    double sqSumGradMag = std::inner_product(diffGradMag.begin(), diffGradMag.end(), diffGradMag.begin(), 0.0);
    
    double stdevValues = std::sqrt(sqSumValues / neighborsValues.size());
    double stdevGradMag = std::sqrt(sqSumGradMag / neighborsGradientMagnitude.size());
    
    return {stdevValues, stdevGradMag};
}

void FloodFillVolume::floodFill(ivec3 seedVoxel) {
    // Get acces to data
    auto inVolumeDataAccesser = inVolume_->getRepresentation<VolumeRAM>();
    auto outVolumeDataAccesser = outVolume_->getEditableRepresentation<VolumeRAM>();
    
    // Create queue
    std::list<ivec3> queue;
    queue.push_back(seedVoxel);
    
    // Get isoValue
    double isoValue = inVolumeDataAccesser->getAsDouble(seedVoxel);
    outVolumeDataAccesser->setFromDouble(seedVoxel, 0.0);
    
    // Get user-input boundary
    double boundary = boundary_.get();
    
    // Prep to set value range of output volume
    dvec2 valR = valueRange_.get();
    double maxVal = std::max(valR[1], std::numeric_limits<double>::lowest());
    double minVal = std::min(valR[0], std::numeric_limits<double>::max());
    
    while(!queue.empty()) {
        ivec3 curIndex = queue.front();
        queue.pop_front();

        for (unsigned long i = 0; i < offsets_.size(); i++) {
            ivec3 neighborIndex{curIndex + offsets_[i]};

			// Grow across x-boundary since they are the same points on the world
			neighborIndex.x = neighborIndex.x % dims_.x;
            
            // Check that neighbor voxel is within dimensions
            if (withinDimensions(neighborIndex)) {
                double neighborVal = inVolumeDataAccesser->getAsDouble(neighborIndex);
                double compValue = std::abs(neighborVal - isoValue);

                // Check value is within boundary
                if ( compValue < boundary) {

                    // Only add to queue and change value if voxel has not been accessed before
                    if (outVolumeDataAccesser->getAsDouble(neighborIndex) > boundary) {
                        // Check if the valueRange has been changed
                        maxVal = std::max(maxVal, neighborVal);
                        minVal = std::min(minVal, neighborVal);
                        
                        // Set voxel value of output index as diff with iso value
                        outVolumeDataAccesser->setFromDouble(neighborIndex, compValue);

                        // Add to queue
                        queue.push_back(neighborIndex);
                    }
                }

				else {
					outVolumeDataAccesser->setFromDouble(neighborIndex, compValue);
				}
            }
        }
    }
    
    // Set new value range
    valueRange_.set({minVal, maxVal});
}

void FloodFillVolume::regionGrowingValuesBased(ivec3 seedVoxel) {
    // Get acces to data
    auto inVolumeDataAccesser = inVolume_->getRepresentation<VolumeRAM>();
    auto outVolumeDataAccesser = outVolume_->getEditableRepresentation<VolumeRAM>();
    
    // Create queue
    std::list<ivec3> queue;
    queue.push_back(seedVoxel);
    
    // Get seed value
    double seedValue = inVolumeDataAccesser->getAsDouble(seedVoxel);
    outVolumeDataAccesser->setFromDouble(seedVoxel, 0.0);
    
    // Get standard deviation based on seed voxel neighbors
    std::pair<double, double> stdevAll = standardDeviationAroundSeed(seedVoxel);
    double stdevValue = stdevAll.first;
    
    // Prep to set value range of output volume
    dvec2 valR = valueRange_.get();
    double maxVal = std::max(valR[1], std::numeric_limits<double>::lowest());
    double minVal = std::min(valR[0], std::numeric_limits<double>::max());
    
    while(!queue.empty()) {
        ivec3 curIndex = queue.front();
        queue.pop_front();

        for (unsigned long i = 0; i < offsets_.size(); i++) {
            ivec3 neighborIndex{curIndex + offsets_[i]};

			// Grow across x-boundary since they are the same points on the world
			neighborIndex.x = neighborIndex.x % dims_.x;
            
            // Check that neighbor voxel is within dimensions
            if (withinDimensions(neighborIndex)) {
                double neighborVal = inVolumeDataAccesser->getAsDouble(neighborIndex);
                double fca = (std::abs(neighborVal - seedValue) / (k_.get() * stdevValue));

                if ( fca < 1) {
                    // Only add to queue and change value if voxel has not been accessed before
                    if (outVolumeDataAccesser->getAsDouble(neighborIndex) > 1) {
                        // Check if the valueRange has been changed
                        maxVal = std::max(maxVal, neighborVal);
                        minVal = std::min(minVal, neighborVal);

                        // Set voxel value of output index as diff with iso value
                        outVolumeDataAccesser->setFromDouble(neighborIndex, fca);

                        // Add to queue
                        queue.push_back(neighborIndex);
                    }
                }

				else {
					outVolumeDataAccesser->setFromDouble(neighborIndex, fca);
				}
            }
        }
    }
    
    // Set new value range
    valueRange_.set({minVal, maxVal});
}

void FloodFillVolume::regionGrowingBoundaryBased(ivec3 seedVoxel) {
    // Get acces to data
    auto inVolumeDataAccesser = inVolume_->getRepresentation<VolumeRAM>();
    auto gradientVolumeDataAccesser = gradientMagnitudeVolInport_.getData()->getRepresentation<VolumeRAM>();
    auto outVolumeDataAccesser = outVolume_->getEditableRepresentation<VolumeRAM>();
    
    // Create queue
    std::list<ivec3> queue;
    queue.push_back(seedVoxel);
    
    // Get isoValue
    double gradientValue = gradientVolumeDataAccesser->getAsDouble(seedVoxel);
    outVolumeDataAccesser->setFromDouble(seedVoxel, 0.0);
    
    // Get standard deviation based on seed voxel neighbors
    std::pair<double, double> stdevAll = standardDeviationAroundSeed(seedVoxel);
    double stdevMagGrad = stdevAll.second;
    
    // Prep to set value range of output volume
    dvec2 valR = valueRange_.get();
    double maxVal = std::max(valR[1], std::numeric_limits<double>::lowest());
    double minVal = std::min(valR[0], std::numeric_limits<double>::max());
    
    while(!queue.empty()) {
        ivec3 curIndex = queue.front();
        queue.pop_front();

        for (unsigned long i = 0; i < offsets_.size(); i++) {
            ivec3 neighborIndex{curIndex + offsets_[i]};

			// Grow across x-boundary since they are the same points on the world
			neighborIndex.x = neighborIndex.x % dims_.x;
            
            // Check that neighbor voxel is within dimensions
            if (withinDimensions(neighborIndex)) {
                double neighborVal = inVolumeDataAccesser->getAsDouble(neighborIndex);
                double neighborGradientVal = gradientVolumeDataAccesser->getAsDouble(neighborIndex);
                double fcb = (std::abs(neighborGradientVal - gradientValue) / (k_.get() * stdevMagGrad));

                if ( fcb < 1) {
                    // Only add to queue and change value if voxel has not been accessed before
                    if (outVolumeDataAccesser->getAsDouble(neighborIndex) > 1) {
                        
                        // Check if the valueRange has been changed
                        maxVal = std::max(maxVal, neighborVal);
                        minVal = std::min(minVal, neighborVal);

                        // Set voxel value of output index as diff with iso value
                        outVolumeDataAccesser->setFromDouble(neighborIndex, fcb);

                        // Add to queue
                        queue.push_back(neighborIndex);
                    }
                }
				else {
					outVolumeDataAccesser->setFromDouble(neighborIndex, fcb);
				}

            }
        }
    }
    
    // Set new value range
    valueRange_.set({minVal, maxVal});
}

void FloodFillVolume::regionGrowingCombined(ivec3 seedVoxel) {
    // Get acces to data
    auto inVolumeDataAccesser = inVolume_->getRepresentation<VolumeRAM>();
    auto gradientVolumeDataAccesser = gradientMagnitudeVolInport_.getData()->getRepresentation<VolumeRAM>();
    auto outVolumeDataAccesser = outVolume_->getEditableRepresentation<VolumeRAM>();
    
    // Create queue
    std::list<ivec3> queue;
    queue.push_back(seedVoxel);
    
    // Get isoValue
    double isoValue = inVolumeDataAccesser->getAsDouble(seedVoxel);
    double gradientValue = gradientVolumeDataAccesser->getAsDouble(seedVoxel);
    outVolumeDataAccesser->setFromDouble(seedVoxel, gradientValue);
    
    // Get standard deviation based on seed voxel neighbors
    std::pair<double, double> stdevAll = standardDeviationAroundSeed(seedVoxel);
    double stdevValue = stdevAll.first;
    double stdevMagGrad = stdevAll.second;
    
    // Prep to set value range of output volume
    dvec2 valR = valueRange_.get();
    double maxVal = std::max(valR[1], std::numeric_limits<double>::lowest());
    double minVal = std::min(valR[0], std::numeric_limits<double>::max());
    
    while(!queue.empty()) {
        ivec3 curIndex = queue.front();
        queue.pop_front();

        for (unsigned long i = 0; i < offsets_.size(); i++) {
            ivec3 neighborIndex{curIndex + offsets_[i]};

			// Grow across x-boundary since they are the same points on the world
			neighborIndex.x = neighborIndex.x % dims_.x;
            
            // Check that neighbor voxel is within dimensions
            if (withinDimensions(neighborIndex)) {
                double neighborVal = inVolumeDataAccesser->getAsDouble(neighborIndex);
                double neighborGradientVal = gradientVolumeDataAccesser->getAsDouble(neighborIndex);
                
                // Calculate cost functions
                double fca = (std::abs(neighborVal - isoValue) / (k_.get() * stdevValue));
                double fcb = (std::abs(neighborGradientVal - gradientValue) / (k_.get() * stdevMagGrad));
                
                //double p = stdevMagGrad / (stdevValue + stdevMagGrad);
                double p = p_.get();
                double fcc = fca * p + fcb * (1 - p);

                if ( fcc < 1) {
                    // Only add to queue and change value if voxel has not been accessed before
                    if (outVolumeDataAccesser->getAsDouble(neighborIndex) > 1) {
                        
                        // Check if the valueRange has been changed
                        maxVal = std::max(maxVal, neighborVal);
                        minVal = std::min(minVal, neighborVal);

                        // Set voxel value of output index as diff with iso value
                        outVolumeDataAccesser->setFromDouble(neighborIndex, fcc);

                        // Add to queue
                        queue.push_back(neighborIndex);
                    }
                }
				else {
					outVolumeDataAccesser->setFromDouble(neighborIndex, fcc);
				}
            }
        }
    }
    
    // Set new value range
    valueRange_.set({minVal, maxVal});
}

void FloodFillVolume::process() {
    
    if (!meshInport_.isReady() || !volumeInport_.isReady() || !gradientMagnitudeVolInport_.isReady())
        return;
    
    // Get inport volume size
    inVolume_ = volumeInport_.getData()->clone();
    dims_ = ivec3(inVolume_->getDimensions());
    
    // Create output volume and set size to inport volume
    outVolume_ = std::make_shared<Volume>(Volume(dims_, inVolume_->getDataFormat()));
    outVolume_->setBasis(inVolume_->getBasis());
    outVolume_->setOffset(inVolume_->getOffset());
    
    // Set all output voxel values to max possible number
    auto outVolumeDataAccesser = outVolume_->getEditableRepresentation<VolumeRAM>();
	double maxValInputVolume = inVolume_->dataMap_.dataRange[1];
    
    // Loop through all voxels and set to max value
    for (int i = 0; i < dims_.x; i++) {
        for (int j = 0; j < dims_.y; j++) {
            for (int k = 0; k < dims_.z; k++) {
                outVolumeDataAccesser->setFromDouble(size3_t(i, j, k), maxValInputVolume);
            }
        }
    }

    // Get inport mesh positions
    std::shared_ptr<Mesh> mesh(meshInport_.getData()->clone());
    auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
    auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();
    
    switch (method_) {
        case Method::FloodFill:
			// Set suggested iso
			suggestedIsoVal_.set(boundary_.get());

            // Reset valueRange_
            valueRange_.set({std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()});
            for(auto& pos : positions) {
                floodFill(getVoxelIndexFromPosition(pos));
            }
            break;
            
        case Method::RegionGrowingValuesBased:
			// Set suggested iso
			suggestedIsoVal_.set(1.f);

            // Reset valueRange_
            valueRange_.set({std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()});
            for(auto& pos : positions) {
                regionGrowingValuesBased(getVoxelIndexFromPosition(pos));
            }
            break;
            
        case Method::RegionGrowingBoundaryBased:
			// Set suggested iso
			suggestedIsoVal_.set(1.f);

            // Reset valueRange_
            valueRange_.set({std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()});
            for(auto& pos : positions) {
                regionGrowingBoundaryBased(getVoxelIndexFromPosition(pos));
            }
            break;
            
        case Method::RegionGrowingCombined:
			// Set suggested iso
			suggestedIsoVal_.set(1.f);

            // Reset valueRange_
            valueRange_.set({std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()});
            for(auto& pos : positions) {
                regionGrowingCombined(getVoxelIndexFromPosition(pos));
            }
            break;
            
        default:
            break;
    }
    
    volumeOutport_.setData(outVolume_);
}

}  // namespace inviwo
