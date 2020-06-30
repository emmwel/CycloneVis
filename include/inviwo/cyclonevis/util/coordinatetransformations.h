#pragma once
#include <inviwo/core/common/inviwo.h>
#include <inviwo/cyclonevis/cyclonevismodule.h>

namespace coordTransform {
	using vec2 = glm::vec2;
	using vec3 = glm::vec3;
	using vec4 = glm::vec4;

	// CONSTANTS
	const float EARTH_RADIUS_M = 6371000.0;
	const float EARTH_RADIUS_KM = 6731.0;
	const float TO_RAD = M_PI / 180.0;
	const float TO_DEG = 180.0 / M_PI;

	// Order:
	// Cartesian: (x, y, z)
	// Polar: (r, phi)
	// Cylindrical: (r, theta, z)
	// Spherical: (rho, theta, phi)

	inline float rangeMapper(float inputValue, vec2 inRange, vec2 outRange) {
		float slope = (outRange[1] - outRange[0]) / (inRange[1] - inRange[0]);
		return (outRange[0] + slope * (inputValue - inRange[0]));
	}

	/*--------------- TRANSFORMATIONS ---------------*/

	inline vec2 cartesianToPolar(vec2 coords) {
		double r = sqrt(coords.x * coords.x + coords.y * coords.y);
		double phi = atan2(coords.y, coords.x);

		return vec2(r, phi);
	}

	inline vec2 polarToCartesian(vec2 coords) {
		// r * cos(phi)
		double x = coords[0] * cos(coords[1]);

		// r * sin(phi)
		double y = coords[0] * sin(coords[1]);

		return vec2(x, y);
	}

	inline vec3 cartesianToCylindrical(vec3 coords) {
		float r = sqrt(coords.x * coords.x + coords.y * coords.y);

		// use atan2 so function takes quadrant into account
		float theta = atan2(coords.y, coords.x);
		float z = coords.z;

		return vec3(r, theta, z);
	}

	inline vec3 cylindricalToCartesian(vec3 coords) {
		// r * cos(theta)
		float x = coords[0] * cos(coords[1]);

		// r * sin(theta)
		float y = coords[0] * sin(coords[1]);

		float z = coords.z;

		return vec3(x, y, z);
	}

	inline vec3 cylindricalToSpherical(vec3 coords) {
		// sqrt(r^2 + z^2)
		double rho = sqrt(coords[0] * coords[0] + coords[2] * coords[2]);

		// arctan(r/z)
		double theta = atan2(coords[0], coords[2]);

		// phi is theta
		double phi = coords[1];

		return vec3(rho, theta, phi);
	}

	inline vec3 cartesianToSpherical(vec3 coords) {

		float rho = sqrt(coords.x * coords.x + coords.y * coords.y + coords.z * coords.z);
		float theta = atan2(coords.y, coords.x);
		float phi = acos(coords.z / rho);

		return vec3(rho, theta, phi);
	}

	inline vec3 sphericalToCartesian(vec3 coords) {
		// rho * sin(phi) * cos(theta)
		float x = coords[0] * sin(coords[2]) * cos(coords[1]);

		// rho * sin(phi) * sin(theta)
		float y = coords[0] * sin(coords[2]) * sin(coords[1]);

		// rho * cos(phi)
		float z = coords[0] * cos(coords[2]);

		return vec3(x, y, z);
	}

	inline vec3 sphericalToCylindrical(vec3 coords) {
		// r * sin(theta)
		double r = coords[0] * sin(coords[1]);

		// phi
		double theta = coords[2];

		// r * cos(theta)
		double z = coords[0] * cos(coords[1]);

		return vec3(r, theta, z);
	}

	/*--------------- LAT-LONG ---------------*/

	inline vec2 cartesianToLatLong(vec2 coords, vec2 dimX, vec2 dimY) {
		// Here it is assumed that the cartesian system is in the middle point of the world
		// and that coords is in the world coordinate system

		float latitude = rangeMapper(coords.y, dimY, vec2(-90, 90));
		float longitude = rangeMapper(coords.x, dimX, vec2(-180, 180));

		return vec2(latitude, longitude);
	}

	inline vec3 mapToLatLongAlt(vec3 coords, vec2 dimX, vec2 dimY, vec2 dimZ, float sphereRad) {
		// Here it is assumed that the cartesian system is in the middle point of the world
		// and that coords is in the world coordinate system

		float latitude = rangeMapper(coords.y, dimY, vec2(-90, 90));
		float longitude = rangeMapper(coords.x, dimX, vec2(-180, 180));
		float altitude = sphereRad + rangeMapper(coords.z, dimZ, vec2(0, dimZ[1]-dimZ[0]));

		return vec3(latitude, longitude, altitude);
	}

	inline vec3 latLongAltToSpherical(vec3 coords) { // takes as [lat, long, alt]
		float rho = coords[2];

		// go from [-180, 180] --> [0, 360]
		float theta = TO_RAD * (180 + coords[1]);

		// go from [-90, 90] --> [180, 0] (lat long uses elevation angle with the x-axis)
		// but spherical coordinates uses inclination with y
		float phi = TO_RAD * (90 - coords[0]);

		return vec3(rho, theta, phi);
	}

	/*--------------- DISTANCE MEASURES ---------------*/

	// function assumes both points are in spherical coordinatesystem
	inline float distanceSphericalCoords(vec3 p1, vec3 p2) {
		float distance = sqrt(
			(p1[0] * p1[0]) + 
			(p2[0] * p2[0]) - 
			(2 * p1[0] * p2[0] * (
					(sin(p1[1]) * sin(p2[1]) * cos(p1[2] - p2[2])) +
					(cos(p1[1]) * cos(p2[1]))
				)
			)
		);
		return distance;
	}

	// great circle distance using haversine formula
	inline float distanceHaversine(vec2 p1, vec2 p2) {
		// Turn given lat-long from degrees to radians
		vec2 p1Rad = TO_RAD * p1;
		vec2 p2Rad = TO_RAD * p2;

		float difLat = p2Rad[0] - p1Rad[0];
		float difLong = p2Rad[1] - p1Rad[1];

		float haversineCentralAngle = sin(difLat / 2) * sin(difLat / 2) +
			(cos(p1Rad[0])* cos(p2Rad[0]) * sin(difLong/2) * sin(difLong/2));

		// Using atan2 instead of asin gives the quadrant of the angle 
		float centralAngle = 2 * atan2(sqrt(haversineCentralAngle), sqrt(1 - haversineCentralAngle));

		// distance = radius * centralangle
		return EARTH_RADIUS_M * centralAngle;

	}
}