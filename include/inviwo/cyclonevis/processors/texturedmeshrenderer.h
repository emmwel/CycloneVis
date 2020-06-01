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
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/interaction/cameratrackball.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo/core/properties/cameraproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/simplelightingproperty.h>
#include <inviwo/core/rendering/meshdrawer.h>

#include <modules/opengl/shader/shader.h>

#include <vector>

namespace inviwo {

/** \docpage{org.inviwo.TexturedMeshRenderer, Textured Mesh Renderer}
 * ![](org.inviwo.TexturedMeshRenderer.png?classIdentifier=org.inviwo.TexturedMeshRenderer)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __<Inport1>__ <description>.
 *
 * ### Outports
 *   * __<Outport1>__ <description>.
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */
class IVW_MODULE_CYCLONEVIS_API TexturedMeshRenderer : public Processor {
public:
    TexturedMeshRenderer();

	// delete copy and assigment
	TexturedMeshRenderer(const TexturedMeshRenderer&) = delete;
	TexturedMeshRenderer& operator=(const TexturedMeshRenderer&) = delete;

    virtual ~TexturedMeshRenderer() = default;

	virtual void initializeResources() override;
    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    MeshInport inport_;
	ImageInport textureInport_;
	ImageInport backgroundInport_;
	ImageOutport outport_;

	CameraProperty camera_;
	CameraTrackball trackball_;

	BoolProperty overrideColorBuffer_;
	FloatVec4Property overrideColor_;

	CompositeProperty meshProperties_;
	OptionPropertyInt cullFace_;
	BoolProperty enableDepthTest_;
	SimpleLightingProperty lightingProperty_;

	CompositeProperty layers_;
	BoolProperty colorLayer_;
	BoolProperty texCoordLayer_;
	BoolProperty normalsLayer_;
	BoolProperty viewNormalsLayer_;

	Shader shader_;
};

}  // namespace inviwo
