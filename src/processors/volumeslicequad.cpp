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
		, position_("position", "Position", vec3(0.0f), vec3(-100.0f), vec3(100.0f))
        , basis_("Basis", "Basis and offset"){

		addPort(outport_);
		addProperties(position_, basis_);
	}

	void VolumeSliceQuad::process() {
		using MyMesh = TypedMesh<buffertraits::PositionsBuffer,
            buffertraits::NormalBuffer,
            buffertraits::TexcoordBuffer<3>,
			buffertraits::ColorsBuffer>;
		auto mesh = std::make_shared<MyMesh>();
        
        // Create vertices according to basis property
        mat3 basis = mat3{basis_.a_, basis_.b_, basis_.c_};
        vec3 offset = basis_.offset_.get();
        vec3 vert0 = basis * vec3(0.0f, 0.0f, 0.0f) + offset;
        vec3 vert1 = basis * vec3(0.0f, 1.0f, 0.0f) + offset;
        vec3 vert2 = basis * vec3(1.0f, 0.0f, 0.0f) + offset;
        vec3 vert3 = basis * vec3(1.0f, 1.0f, 0.0f) + offset;
        
        vec4 color = vec4(1, 1, 1, 1);
        vec3 normal = vec3(0, 0, 1);
        
		mesh->addVertex(vert0, normal, vec3(0.0f), color);
        mesh->addVertex(vert1, normal, vec3(0.0f, 1.0f, 0.0f), color);
        mesh->addVertex(vert2, normal, vec3(1.0f, 0.0f, 0.0f), color);
        mesh->addVertex(vert3, normal, vec3(1.0f, 1.0f, 0.0f), color);

		auto indexbuffer = mesh->addIndexBuffer(DrawType::Triangles, ConnectivityType::Strip);
		indexbuffer->add(0);
		indexbuffer->add(1);
		indexbuffer->add(2);
		indexbuffer->add(3);
        
		outport_.setData(mesh);
	}

}  // namespace inviwo
