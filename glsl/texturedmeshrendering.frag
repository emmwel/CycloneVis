/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2014-2020 Inviwo Foundation
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

// Owned by the TexturedMeshRenderer Processor

#include "utils/shading.glsl"

#if !defined(TEXCOORD_LAYER) || !defined(NORMALS_LAYER) \
    || !defined(VIEW_NORMALS_LAYER) || !defined(COLOR_LAYER)
#  define COLOR_LAYER
#endif

uniform LightParameters lighting;
uniform CameraParameters camera;

uniform sampler2D inportOneTexture;
uniform sampler2D inportTwoTexture;
uniform float blendCoef;
uniform int blendMode;

in vec4 worldPosition_;
in vec3 normal_;
in vec3 viewNormal_;
in vec3 texCoord_;
in vec4 color_;
flat in vec4 pickColor_;

vec3 overlayBlender(vec3 a, vec3 b) {
	vec3 outBlend = vec3(0.0);

	for (int i = 0; i < 3; i++) {
		float a = a[i];
		float b = b[i];
		
		if (a < 0.5) {
			outBlend[i] = 2 * a * b; 
		}
		else {
			outBlend[i] = 1.0 - (2 * (1 - a) * (1 - b));
		}
	}

	return outBlend;
}

vec3 softLightBlender(vec3 a, vec3 b) {
	vec3 outBlend = vec3(0.0);

	for (int i = 0; i < 3; i++) {
		float a = a[i];
		float b = b[i];

		if (b < 0.5) {
			outBlend[i] = (2.0 * a * b) + (a * a * (1.0 - 2.0 * b)); 
		}
		else {
			outBlend[i] = (2.0 * a * (1.0 - b)) + (sqrt(a) * (2 * b - 1.0));
		}
	}

	return outBlend;
}

void main() {

	// apply color according to blend mode
	// inportTextureOne is background layer, inportTextureTwo is the top layer
	vec4 fragColor = color_;

	if (blendMode == 0) {
		// Weighted sum
		fragColor = blendCoef * texture(inportOneTexture, texCoord_.xy).rgba + (1.0f - blendCoef) * texture(inportTwoTexture, texCoord_.xy).rgba;
	}
	if (blendMode == 1) {
		// Multiply
		vec3 backgroundLayer = texture(inportOneTexture, texCoord_.xy).rgb;
		vec3 topLayer = texture(inportTwoTexture, texCoord_.xy).rgb;

		fragColor = vec4(backgroundLayer * topLayer, 1.0);
	}
	if (blendMode == 2) {
		// Screen
		vec3 backgroundLayer = texture(inportOneTexture, texCoord_.xy).rgb;
		vec3 topLayer = texture(inportTwoTexture, texCoord_.xy).rgb;

		fragColor = vec4(vec3(1.0) - (vec3(1.0) - backgroundLayer) * (vec3(1.0) - topLayer), 1.0);
	}
	if (blendMode == 3) {
		// Overlay
		vec3 backgroundLayer = texture(inportOneTexture, texCoord_.xy).rgb;
		vec3 topLayer = texture(inportTwoTexture, texCoord_.xy).rgb;

		fragColor = vec4(overlayBlender(backgroundLayer, topLayer), 1.0);
	}
	if (blendMode == 4) {
		// Soft Light -- Photoshop version
		vec3 backgroundLayer = texture(inportOneTexture, texCoord_.xy).rgb;
		vec3 topLayer = texture(inportTwoTexture, texCoord_.xy).rgb;

		fragColor = vec4(softLightBlender(backgroundLayer, topLayer), 1.0);
	
	}

    
	
    vec3 toCameraDir_ = camera.position - worldPosition_.xyz;

	fragColor.rgb = APPLY_LIGHTING(lighting, fragColor.rgb, fragColor.rgb, fragColor.rgb, worldPosition_.xyz,
                                   normalize(normal_), normalize(toCameraDir_));

#ifdef COLOR_LAYER
    FragData0 = fragColor;
#endif
#ifdef TEXCOORD_LAYER
    tex_coord_out = vec4(texCoord_,1.0f);
#endif
#ifdef NORMALS_LAYER
    normals_out = vec4((normalize(normal_)+1.0)*0.5,1.0f);
#endif
#ifdef VIEW_NORMALS_LAYER
    view_normals_out = vec4((normalize(viewNormal_)+1.0)*0.5,1.0f);
#endif

    PickingData = pickColor_;
}
