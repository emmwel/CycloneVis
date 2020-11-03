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

		// Create positions based on a grid of the domain
		std::vector<vec3> positions;
		std::vector<vec3> textureCoords;
		int ROWS = basisandoffset[1][1];
		int COLS = basisandoffset[0][0];

		for (int row = 0; row <= ROWS; row++) {
			for (int col = 0; col <= COLS; col++) {
				float x = (float)col / (float)COLS;
				float y = (float)row / (float)ROWS;
				float u = x;
				float v = y;

				positions.push_back(vec3(x, y, z_val));
				textureCoords.push_back(vec3(u, v, 0));
			}
		}

		// Normals and colors
		std::vector<vec3> normals;
		std::vector<vec4> colors;
		for (int i = 0; i < positions.size(); i++) {
			normals.push_back(vec3(0, 0, 1));
			colors.push_back(vec4(1, 1, 1, 1));
		}

		// Indices for line triangle buffer
		std::vector<std::uint32_t> indexMeshDataTriangles;
		for (int row = 0; row < ROWS; row++) {
			for (int col = 0; col < COLS; col++) {

				int curRowIndex = row * (COLS + 1);
				int upRowIndex = (row + 1) * (COLS + 1);

				// TRIANGLES
				//2------3
				//| \    |
				//|   \  |
				//0------1

				// 0 - 1 - 2
				indexMeshDataTriangles.push_back(curRowIndex + col);
				indexMeshDataTriangles.push_back(curRowIndex + col + 1);
				indexMeshDataTriangles.push_back(upRowIndex + col);

				// 1 - 3 - 2
				indexMeshDataTriangles.push_back(curRowIndex + col + 1);
				indexMeshDataTriangles.push_back(upRowIndex + col + 1);
				indexMeshDataTriangles.push_back(upRowIndex + col);
				
			}
		}

		// Make buffers
		auto posBuff = util::makeBuffer(std::move(positions));
		auto normalBuff = util::makeBuffer(std::move(normals));
		auto texBuff = util::makeBuffer(std::move(textureCoords));
		auto colBuff = util::makeBuffer(std::move(colors));
		auto indexBuffTriangles = util::makeIndexBuffer(std::move(indexMeshDataTriangles));

		mesh->addBuffer(BufferType::PositionAttrib, posBuff);
		mesh->addBuffer(BufferType::NormalAttrib, normalBuff);
		mesh->addBuffer(BufferType::TexcoordAttrib, texBuff);
		mesh->addBuffer(BufferType::ColorAttrib, colBuff);
		mesh->addIndicies(Mesh::MeshInfo(DrawType::Triangles, ConnectivityType::Strip), indexBuffTriangles);
        
		outport_.setData(mesh);
	}

}  // namespace inviwo
