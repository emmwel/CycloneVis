#pragma once
#include <inviwo/core/common/inviwo.h>
#include <inviwo/cyclonevis/cyclonevismodule.h>

namespace coordTransform {
	using vec2 = glm::vec2;
	using vec3 = glm::vec3;
	using vec4 = glm::vec4;

	// CONSTANTS
	const float EARTH_RADIUS_M = 6371000.0;
	const float TO_RAD = M_PI / 180;
	const float TO_DEG = 180 / M_PI;

	// Order:
	// Cartesian: (x, y, z)
	// Polar: (r, phi)
	// Cylindrical: (r, theta, z)
	// Spherical: (rho, theta, phi)

	vec2 cartesianToPolar(vec2 coords) {
		double r = sqrt(coords.x * coords.x + coords.y * coords.y);
		double phi = atan2(coords.y, coords.x);

		return vec2(r, phi);
	}

	vec2 polarToCartesian(vec2 coords) {
		// r * cos(phi)
		double x = coords[0] * cos(coords[1]);

		// r * sin(phi)
		double y = coords[0] * sin(coords[1]);

		return vec2(x, y);
	}

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

	vec3 cylindricalToSpherical(vec3 coords) {
		// sqrt(r^2 + z^2)
		double rho = sqrt(coords[0] * coords[0] + coords[2] * coords[2]);

		// arctan(r/z)
		double theta = atan2(coords[0], coords[2]);

		// phi is theta
		double phi = coords[1];

		return vec3(rho, theta, phi);
	}

	vec3 cartesianToSpherical(vec3 coords) {

		float rho = sqrt(coords.x * coords.x + coords.y * coords.y + coords.z * coords.z);
		float theta = atan2(coords.y, coords.x);
		float phi = acos(coords.z / rho);

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

	vec3 sphericalToCylindrical(vec3 coords) {
		// r * sin(theta)
		double r = coords[0] * sin(coords[1]);

		// phi
		double theta = coords[2];

		// r * cos(theta)
		double z = coords[0] * cos(coords[1]);

		return vec3(r, theta, z);
	}

	/*--------------- DISTANCE MEASURES ---------------*/

	// function assumes both points are in spherical coordinatesystem
	float distanceSphericalCoords(vec3 p1, vec3 p2) {
		float distance = sqrt(
			(p1[0] * p1[0]) + 
			(p2[0] * p2[0]) - 
			(2 * p1[0] * p2[0] * (sin(p1[1]) * sin(p2[1]) * cos(p1[2] - p2[2]))) + 
			(cos(p1[1])*cos(p2[1]))
		);
		return distance;
	}
}