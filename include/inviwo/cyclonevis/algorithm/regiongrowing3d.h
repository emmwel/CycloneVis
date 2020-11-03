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
#pragma once

#include <inviwo/cyclonevis/cyclonevismoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/util/indexmapper.h>

namespace inviwo {

/**
 * \brief VERY_BRIEFLY_DESCRIBE_THE_CLASS
 * DESCRIBE_THE_CLASS_FROM_A_DEVELOPER_PERSPECTIVE
 */
//class IVW_MODULE_CYCLONEVIS_API regiongrowing3D {
//public:
//    regiongrowing3D();
//    virtual ~regiongrowing3D() = default;
//};

namespace regiongrowing3D {
	// Variables
    static std::array<ivec3, 26> offsets = { {
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
    } };

	// Functions
	IVW_MODULE_CYCLONEVIS_API bool withinDimensions(const ivec3& index, const ivec3& dims);
	
    IVW_MODULE_CYCLONEVIS_API std::pair<double, double> standardDeviationAroundSeed(
        Volume* inVolume,
        Volume* gradientMagVolume,
        const ivec3& seedVoxel
    );

    IVW_MODULE_CYCLONEVIS_API void floodFill(
		Volume* inVolume,
		std::shared_ptr<Volume> outVolume,
		const ivec3& seedVoxel,
		double boundary,
		double& minVal,
		double& maxVal
	);
}

}  // namespace inviwo
