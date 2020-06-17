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

#include <inviwo/cyclonevis/processors/meshwrappertosphere.h>
#include <inviwo/cyclonevis/util/coordinatetransformations.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MeshWrapperToSphere::processorInfo_{
    "org.inviwo.MeshWrapperToSphere",      // Class identifier
    "Mesh Wrapper To Sphere",                // Display name
    "CycloneVis",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo MeshWrapperToSphere::getProcessorInfo() const { return processorInfo_; }

MeshWrapperToSphere::MeshWrapperToSphere()
    : Processor()
    , outport_("outport")
	, meshInport_("meshInport")
	, sphereRadius_("sphereRadius", "Sphere Radius", 5.f, 1.f, 1000.f, 1.f){

	addPort(meshInport_);
    addPort(outport_);

	addProperty(sphereRadius_);
}

void MeshWrapperToSphere::process() {
    // Check inport is ready
	if (!meshInport_.isReady() )
		return;

	// Load mesh data
	std::shared_ptr<Mesh> mesh(meshInport_.getData()->clone());

	// Load position buffer, only works when there is one position buffer
	auto posBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::PositionAttrib));
	auto positions = posBuffer->getEditableRAMRepresentation()->getDataContainer();

	//auto texBuffer = static_cast<Buffer<vec3>*>(mesh->getBuffer(BufferType::TexcoordAttrib));
	//auto texCoords = texBuffer->getEditableRAMRepresentation()->getDataContainer();

	// Get index buffer
	auto indBuffer = mesh->getIndices(0);
	auto indices = indBuffer->getEditableRAMRepresentation()->getDataContainer();

	mat3 basis = mesh->getBasis();
	vec3 offset = mesh->getOffset();
	vec2 dimX = vec2(-basis[0][0]/2, basis[0][0]/2);
	vec2 dimY = vec2(-basis[1][1]/2, basis[1][1]/2);
	mat4 haha = mesh->getModelMatrix();
	std::vector<vec3> newPositions(positions.size());

	for (unsigned long i = 0; i < positions.size(); i++) {
		vec3 latLongAlt = coordTransform::mapToLatLongAlt(positions[i], dimX, dimY, sphereRadius_.get());
		vec3 cartesian = coordTransform::sphericalToCartesian(coordTransform::latLongAltToSpherical(latLongAlt));
		newPositions[i] = cartesian;

		std::cout << positions[i] << std::endl;
		std::cout << latLongAlt << std::endl;
	}

	auto outputPosBuffer = util::makeBuffer(std::move(newPositions));
	//auto outputTexBuffer = util::makeBuffer(std::move(texCoords));
	auto outputIndexBuffer = util::makeIndexBuffer(std::move(indices));

	// Create mesh from buffers
	DrawType hm = mesh->getDefaultMeshInfo().dt;


	inviwo::Mesh* result = new Mesh(DrawType::Lines, ConnectivityType::None);
	result->addBuffer(BufferType::PositionAttrib, outputPosBuffer);
	//result->addBuffer(BufferType::TexcoordAttrib, outputTexBuffer);
	result->addIndicies(Mesh::MeshInfo(DrawType::Lines, ConnectivityType::None), outputIndexBuffer);

	outport_.setData(result);
}

}  // namespace inviwo
