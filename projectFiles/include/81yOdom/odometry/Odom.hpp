#pragma once
#include "81yOdom/sensors/TrackingWheel.hpp"
#include "81yOdom/sensors/IMU.hpp"
#include "81yOdom/odometry/Pose.hpp"

class Odom {
    public: 
        Odom(TrackingWheel* left, TrackingWheel* right, 
        TrackingWheel* back, IMU* imu, 
        double trackWidth, double backOffset)
        : left(left), right(right), back(back), imu(imu),
        trackWidth(trackWidth), backOffset(backOffset) {}

        void setPose(double x, double y, double theta) {
            pose.x = x;
            pose.y = y; 
            pose.theta = theta;
            prevTheta = theta;
        }

        Pose getPose() const {return pose;}

        //Called every 10ms from a PROS round-robin task, updates the robot's pose based on changes in the tracking wheels and IMU since the last cycle
        void update(); 



    private: 
        TrackingWheel* left;
        TrackingWheel* right;
        TrackingWheel* back;
        IMU* imu;

        double trackWidth; 
        double backOffset; 

        Pose pose; 
        double prevTheta = 0;
};