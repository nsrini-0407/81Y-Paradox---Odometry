#pragma once
#include "pros/imu.hpp"
#include "81yOdom/utils/Math.hpp"

class IMU {
    public: 
        explicit IMU(int port): imu(port) {}

        void calibrate() {
            imu.reset(true); //calibrates the inertial sensor, true makes it block the program from running while it calibrates
        }

        //Returns heading in degrees, normalized to -180 to 180
        //IMU returns 0-360, CW positive by default, so we fix this with some math
        double getHeading() {
            double raw = imu.get_heading(); //0-360, CW
            double ccw = normalizeAngle(-(raw)); //flip to CCW
            return normalizeAngle(ccw + offset); //add offset and normalize again to ensure it stays within bounds
        }

        void setHeading(double degrees) {
            offset = degrees - getHeading() + offset;
        }

    private: 
        pros::IMU imu; 
        double offset = 0.0;
};