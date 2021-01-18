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

#include <inviwo/cyclonevis/processors/texturedmeshrenderer.h>
#include <modules/opengl/geometry/meshgl.h>
#include <inviwo/core/datastructures/buffer/bufferramprecision.h>
#include <inviwo/core/interaction/trackball.h>
#include <inviwo/core/common/inviwoapplication.h>
#include <inviwo/core/rendering/meshdrawerfactory.h>
#include <modules/base/algorithm/mesh/axisalignedboundingbox.h>
#include <modules/base/algorithm/mesh/meshcameraalgorithms.h>
#include <modules/opengl/rendering/meshdrawergl.h>
#include <inviwo/core/processors/processor.h>
#include <modules/opengl/shader/shader.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/openglutils.h>

#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/core/algorithm/boundingbox.h>

#include <limits>
#include "windows.h"

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TexturedMeshRenderer::processorInfo_{
    "org.inviwo.TexturedMeshRenderer",      // Class identifier
    "Textured Mesh Renderer",                // Display name
    "CycloneVis",              // Category
    CodeState::Experimental,  // Code state
    Tags::GL,               // Tags
};
const ProcessorInfo TexturedMeshRenderer::getProcessorInfo() const { return processorInfo_; }

TexturedMeshRenderer::TexturedMeshRenderer()
    : Processor()
	, inport_("geometry")
	, textureOneInport_("textureOneInport", true)
	, textureTwoInport_("textureTwoInport", true)
	, backgroundInport_("backgroundInport", true)
	, outport_("image")
	, method_("method", "Method",
		{
		  {"weightedSumMode", "Weighted Sum", Method::WeightedSumMode},
		  {"multiplyMode", "Multiply", Method::MultiplyMode},
		  {"screenMode", "Screen", Method::ScreenMode},
		  {"overlayMode", "Overlay", Method::OverlayMode},
		  {"softLightMode", "Soft Light", Method::SoftLightMode},
		  {"softLightAdobeVerMode", "Soft Light Adobe Ver", Method::SoftLightAdobeVerMode}
		}, 0)
	, blendCoef_("blendCoef", "Image Blend Coefficient", 0.5f, 0.0f, 1.0f, 0.1f)
	, opacity_("opacity", "Opacity", 1.0f, 0.0f, 1.0f, 0.1f)
	, camera_("camera", "Camera", util::boundingBox(inport_))
	, trackball_(&camera_)
	, overrideColorBuffer_("overrideColorBuffer", "Override Color Buffer", false,
		InvalidationLevel::InvalidResources)
	, overrideColor_("overrideColor", "Override Color", vec4(0.75f, 0.75f, 0.75f, 1.0f), vec4(0.0f),
		vec4(1.0f))
	, meshProperties_("geometry", "Geometry Rendering Properties")
	, cullFace_("cullFace", "Cull Face",
		{ {"culldisable", "Disable", GL_NONE},
		 {"cullfront", "Front", GL_FRONT},
		 {"cullback", "Back", GL_BACK},
		 {"cullfrontback", "Front & Back", GL_FRONT_AND_BACK} },
		0)
	, enableDepthTest_("enableDepthTest_", "Enable Depth Test", true)
	, lightingProperty_("lighting", "Lighting", &camera_)
	, layers_("layers", "Output Layers")
	, colorLayer_("colorLayer", "Color", true, InvalidationLevel::InvalidResources)
	, texCoordLayer_("texCoordLayer", "Texture Coordinates", false,
		InvalidationLevel::InvalidResources)
	, normalsLayer_("normalsLayer", "Normals (World Space)", false,
		InvalidationLevel::InvalidResources)
	, viewNormalsLayer_("viewNormalsLayer", "Normals (View space)", false,
		InvalidationLevel::InvalidResources)
	, shader_("texturedmeshrendering.vert", "texturedmeshrendering.frag", Shader::Build::No) {

	addPort(inport_);
	addPort(textureOneInport_);
	addPort(textureTwoInport_).setOptional(true);
	addPort(backgroundInport_).setOptional(true);
	addPort(outport_);

	addProperties(method_, blendCoef_, opacity_, camera_, meshProperties_, lightingProperty_, trackball_, layers_);

	meshProperties_.addProperties(cullFace_, enableDepthTest_, overrideColorBuffer_,
		overrideColor_);

	overrideColor_.setSemantics(PropertySemantics::Color)
		.visibilityDependsOn(overrideColorBuffer_, [](const BoolProperty p) { return p.get(); });

	layers_.addProperties(colorLayer_, texCoordLayer_, normalsLayer_, viewNormalsLayer_);

	method_.onChange([this]() {
		switch (method_) {
			case Method::WeightedSumMode:
				blendCoef_.setVisible(true);
				break;

			case Method::MultiplyMode:
				blendCoef_.setVisible(false);
				break;

			case Method::ScreenMode:
				blendCoef_.setVisible(false);
				break;

			case Method::OverlayMode:
				blendCoef_.setVisible(false);
				break;

			case Method::SoftLightMode:
				blendCoef_.setVisible(false);
				break;

			case Method::SoftLightAdobeVerMode:
				blendCoef_.setVisible(false);
				break;

			default:
				break;
		}
	});

	shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });
}

//TexturedMeshRenderer::~TexturedMeshRenderer() = default;

void TexturedMeshRenderer::initializeResources() {
	utilgl::addShaderDefines(shader_, lightingProperty_);

	auto vert = shader_.getVertexShaderObject();
	auto frag = shader_.getFragmentShaderObject();

	vert->setShaderDefine("OVERRIDE_COLOR_BUFFER", overrideColorBuffer_);
	frag->setShaderDefine("COLOR_LAYER", colorLayer_);

	// first two layers (color and picking) are reserved
	int layerID = 2;
	frag->setShaderDefine("TEXCOORD_LAYER", texCoordLayer_);
	if (texCoordLayer_) frag->addOutDeclaration("tex_coord_out", layerID++);

	frag->setShaderDefine("NORMALS_LAYER", normalsLayer_);
	if (normalsLayer_) frag->addOutDeclaration("normals_out", layerID++);

	frag->setShaderDefine("VIEW_NORMALS_LAYER", viewNormalsLayer_);
	if (viewNormalsLayer_) frag->addOutDeclaration("view_normals_out", layerID++);

	// get a hold of the current output data
	auto prevData = outport_.getData();
	auto numLayers = static_cast<std::size_t>(layerID - 1);  // Don't count picking
	if (prevData->getNumberOfColorLayers() != numLayers) {
		// create new image with matching number of layers
		auto image = std::make_shared<Image>(prevData->getDimensions(), prevData->getDataFormat());
		// update number of layers
		for (auto i = image->getNumberOfColorLayers(); i < numLayers; ++i) {
			image->addColorLayer(std::shared_ptr<Layer>(image->getColorLayer(0)->clone()));
		}
		outport_.setData(image);
	}

	shader_.build();
}

void TexturedMeshRenderer::process() {

	utilgl::activateTargetAndClearOrCopySource(outport_, backgroundInport_);
	shader_.activate();

	// Textures
	TextureUnit colorOneTexUnit, colorTwoTexUnit;

	if (textureOneInport_.isReady()) {
		utilgl::bindColorTexture(textureOneInport_, colorOneTexUnit.getEnum());
	}

	if (textureTwoInport_.isReady()) {
		utilgl::bindColorTexture(textureTwoInport_, colorTwoTexUnit.getEnum());
	}
	else {
		utilgl::bindColorTexture(textureOneInport_, colorTwoTexUnit.getEnum());
	}

	// Blending mode
	int whichMode = method_.getSelectedIndex();

	shader_.setUniform("inportOneTexture", colorOneTexUnit.getUnitNumber());
	shader_.setUniform("inportTwoTexture", colorTwoTexUnit.getUnitNumber());
	shader_.setUniform("blendCoef", blendCoef_.get());
	shader_.setUniform("blendMode", whichMode);
	shader_.setUniform("opacity", opacity_.get());

	utilgl::GlBoolState depthTest(GL_DEPTH_TEST, enableDepthTest_);
	utilgl::CullFaceState culling(cullFace_);
	utilgl::BlendModeState blendModeStateGL(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	utilgl::setUniforms(shader_, camera_, lightingProperty_, overrideColor_);
	for (auto mesh : inport_) {
		utilgl::setShaderUniforms(shader_, *mesh, "geometry");
		shader_.setUniform("pickingEnabled", meshutil::hasPickIDBuffer(mesh.get()));
		MeshDrawerGL::DrawObject drawer{ mesh->getRepresentation<MeshGL>(),
										mesh->getDefaultMeshInfo() };
		drawer.draw();
	}

	shader_.deactivate();
	utilgl::deactivateCurrentTarget();
	TextureUnit::setZeroUnit();
}

}  // namespace inviwo
