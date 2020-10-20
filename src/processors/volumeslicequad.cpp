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

#include <inviwo/cyclonevis/processors/volumeslicequad.h>
#include <inviwo/core/datastructures/geometry/typedmesh.h>

namespace inviwo {

	// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
	const ProcessorInfo VolumeSliceQuad::processorInfo_{
		"org.inviwo.VolumeSliceQuad",      // Class identifier
		"Volume Slice Quad",                // Display name
		"CycloneVis",              // Category
		CodeState::Experimental,  // Code state
		Tags::None,               // Tags
	};
	const ProcessorInfo VolumeSliceQuad::getProcessorInfo() const { return processorInfo_; }

	VolumeSliceQuad::VolumeSliceQuad()
		: Processor()
		, outport_("outport")
        , sliceNumber_("sliceNumber", "Slice Number", 4, 1, 8)
        , mapZToSlice_("mapZToSlice", "Map Z to slice", true)
        , basis_("Basis", "Basis and offset") {

		addPort(outport_);
		addProperties(sliceNumber_, mapZToSlice_, basis_);
	}

	void VolumeSliceQuad::process() {
		inviwo::Mesh* mesh = new Mesh;

		// Get linked basis and offset, set as model matrix
		mat4 basisandoffset = mat4{
			vec4(basis_.a_.get(), 0),
			vec4(basis_.b_.get(), 0),
			vec4(basis_.c_.get(), 0),
			vec4(basis_.offset_.get(), 1)
		};

		mesh->setModelMatrix(basisandoffset);

		// Define z-val and check if z-val should be based on slice or not
        float z_val = 0.f;
        if (mapZToSlice_.get()) {
            z_val = float(sliceNumber_.get() - 1) / (sliceNumber_.getMaxValue() - sliceNumber_.getMinValue());
        }

		// Create positions
		vec3 lowerLeft = vec3(0.0f, 0.0f, z_val);
		vec3 lowerRight = vec3(0.0f, 1.0f, z_val);
		vec3 upperLeft = vec3(1.0f, 0.0f, z_val);
		vec3 upperRight = vec3(1.0f, 1.0f, z_val);
		std::vector<vec3> positions = { lowerLeft, lowerRight, upperLeft, upperRight};

		// Create texture coordinates
		vec3 lowerLeftTexCoord = vec3(0.0f, 0.0f, 0.0f);
		vec3 lowerRightTexCoord = vec3(0.0f, 1.0f, 0.0f);
		vec3 UpperLeftTexCoord = vec3(1.0f, 0.0f, 0.0f);
		vec3 UpperRightTexCoord = vec3(1.0f, 1.0f, 0.0f);
		std::vector<vec3> textureCoords = { lowerLeftTexCoord, lowerRightTexCoord, UpperLeftTexCoord, UpperRightTexCoord };

		// Normals, colors and indices for triangle index buffer
		std::vector<vec3> normals;
		std::vector<vec4> colors;
		std::vector<std::uint32_t> indexMeshData;
		for (int i = 0; i < 4; i++) {
			normals.push_back(vec3(0, 0, 1));
			colors.push_back(vec4(1, 1, 1, 1));
			indexMeshData.push_back(i);
		}

		// Indices for line index buffer
		//std::vector<std::uint32_t> indexMeshData2 = { 0, 1, 0, 2, 1, 3, 2, 3 };
		std::vector<std::uint32_t> indexMeshData2;
		int ROWS = 1;
		int COLS = 1;
		for (int row = 0; row < ROWS; row++) {
			for (int col = 0; col < COLS; col++) {

				int curRowIndex = row * (ROWS + 1);
				int upRowIndex = (row + 1) * (ROWS + 1);

				//2------3
				//|      |
				//|      |
				//0------1

				// (0 - 1)
				indexMeshData2.push_back(curRowIndex + col);
				indexMeshData2.push_back(curRowIndex + col + 1);

				// (1 - 3)
				indexMeshData2.push_back(curRowIndex + col + 1);
				indexMeshData2.push_back(upRowIndex + col + 1);

				// (0 - 2)
				indexMeshData2.push_back(curRowIndex + col);
				indexMeshData2.push_back(upRowIndex + col);

				// (2 - 3)
				indexMeshData2.push_back(upRowIndex + col);
				indexMeshData2.push_back(upRowIndex + col + 1);
				
			}
		}

		// Make buffers
		auto posBuff = util::makeBuffer(std::move(positions));
		auto normalBuff = util::makeBuffer(std::move(normals));
		auto texBuff = util::makeBuffer(std::move(textureCoords));
		auto colBuff = util::makeBuffer(std::move(colors));
		auto indexBuff = util::makeIndexBuffer(std::move(indexMeshData));
		auto indexBuffLines = util::makeIndexBuffer(std::move(indexMeshData2));

		mesh->addBuffer(BufferType::PositionAttrib, posBuff);
		mesh->addBuffer(BufferType::NormalAttrib, normalBuff);
		mesh->addBuffer(BufferType::TexcoordAttrib, texBuff);
		mesh->addBuffer(BufferType::ColorAttrib, colBuff);
		mesh->addIndicies(Mesh::MeshInfo(DrawType::Triangles, ConnectivityType::Strip), indexBuff);
		mesh->addIndicies(Mesh::MeshInfo(DrawType::Lines, ConnectivityType::None), indexBuffLines);
        
		outport_.setData(mesh);
	}

}  // namespace inviwo
