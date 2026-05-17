#pragma once
#include <cmath>
#include "81yOdom/utils/Math.hpp"

struct Pose {
    double x;       //inches, field x
    double y;       //inches, field y
    double theta;      //degrees, heading (0 = forward, CCW positive)

    Pose(double x = 0.0, double y = 0.0, double theta = 0.0) 
        : x(x), y(y), theta(theta) {}

        //Straight-line distance to another pose (or point)
        double distanceTo(double tx, double ty) const {
            double dx = tx - x;
            double dy = ty - y;
            return std::sqrt(dx*dx + dy*dy);
        }

        //Angle from this pose toward a target point (degrees)
        double angleTo(double tx, double ty) const {
            double dx = tx - x;
            double dy = ty - y;
            return toDeg(std::atan2(dx, dy)); //note :atan2(x,y) not (y,x) - field coords
        }

        //Heading error from current theta to a target angle (normalized)
        double headingError(double targetAngle) const {
            return normalizeAngle(targetAngle - theta);
        }
};