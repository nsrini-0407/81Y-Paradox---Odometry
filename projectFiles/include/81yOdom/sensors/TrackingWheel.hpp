#pragma once
#include "pros/rotation.hpp"
#include "81yOdom/utils/Math.hpp"

class TrackingWheel {
public :
    //wheel diameter in inches, gearRatio is output/input (usually 1.0)
    TrackingWheel(pros::Rotation* sensor, double wheelDiameter, double gearRatio = 1.0)
        : sensor(sensor),
        circumference(wheelDiameter * PI),
        gearRatio(gearRatio) {
        sensor->reset_position();
    }

    //Return total distance traveled in inches
    double getDistanceInches() {
        double degrees = sensor->get_position() / 100.0; 
        double rotations = degrees / 360.0;
        return rotations * circumference * gearRatio;
    }

    //called at the start of each odom cycle - stores a current position as a baseline for delta calculations
    void recordPosition() {
        lastPosition = getDistanceInches();
    }

    //Call after recordPosition() - returns delta since last cycle
    double getDelta() {
        return getDistanceInches() - lastPosition;
    }

    void reset() {
        sensor->reset_position();
        lastPosition = 0;
    }

    private :
    pros::Rotation* sensor;
    double circumference;
    double gearRatio;
    double lastPosition = 0.0;
};