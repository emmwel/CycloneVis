#pragma once
#include <inviwo/core/common/inviwo.h>
#include <inviwo/cyclonevis/cyclonevismodule.h>

namespace coordTransform {
	using vec2 = glm::vec2;
	using vec3 = glm::vec3;
	using vec4 = glm::vec4;

	// Order:
	// Cartesian: (x, y, z)
	// Cylindrical: (r, theta, z)
	// Spherical: (rho, theta, phi)

	vec3 cartesianToCylindrical(vec3 coords) {
		float r = sqrt(coords.x * coords.x + coords.y * coords.y);

		// use atan2 so function takes quadrant into account
		float theta = atan2(coords.y, coords.x);
		float z = coords.z;

		return vec3(r, theta, z);
	}

	vec3 cylindricalToCartesian(vec3 coords) {
		// r * cos(theta)
		float x = coords[0] * cos(coords[1]);

		// r * sin(theta)
		float y = coords[0] * sin(coords[1]);

		float z = coords.z;

		return vec3(x, y, z);
	}

	vec3 cartesianToSpherical(vec3 coords) {

		float rho = sqrt(coords.x * coords.x + coords.y * coords.y + coords.z * coords.z);
		float theta = atan2(coords.y, coords.x);
		float phi = acos(coords.x / rho);

		return vec3(rho, theta, phi);
	}

	vec3 sphericalToCartesian(vec3 coords) {
		// rho * sin(phi) * cos(theta)
		float x = coords[0] * sin(coords[2]) * cos(coords[1]);

		// rho * sin(phi) * sin(theta)
		float y = coords[0] * sin(coords[2]) * sin(coords[1]);

		// rho * cos(phi)
		float z = coords[0] * cos(coords[2]);

		return vec3(x, y, z);
	}
}