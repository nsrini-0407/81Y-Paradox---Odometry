#pragma once
#include <cmath>
#include <algorithm>

constexpr double PI = 3.14159265358979;

//Convert degrees to radians and vice versa:
inline double toRad(double deg) { return deg * PI / 180.0; }
inline double toDeg(double rad) { return rad * 180.0 / PI; }

//Clamp a value beteen min and max
inline double clamp(double val, double minVal, double maxVal) {
	return std::max(minVal, std::min(maxVal, val)); 
}

//Normalizing angles to the range [-180, 180]
inline double normalizeAngle(double angle) {
	while (angle > 180.0) angle -= 360.0;
	while (angle <= -180.0) angle += 360.0;
	return angle;
}


//Normalizing angles to the range [0, 360]
inline double normalizeAngle360(double angle) {
	while (angle>= 360.0) angle -= 360.0;
	while (angle < 0.0) angle += 360.0;
	return angle;
}

//Sign of a value: returns -1, 0, or 1
inline double sign(double val) {
	if (val > 0) return 1.0;
	else if (val < 0) return -1.0;
	else return 0.0;
}

//slew rate limiter: limits the rate of change of a value to maxChange per second
inline double slew(double target, double current, double maxChange, double dt) {
	double delta = target - current;
	double maxDelta = maxChange * dt;
	return current + clamp(delta, -maxDelta, maxDelta);
}