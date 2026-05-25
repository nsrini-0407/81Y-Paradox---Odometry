#include "81yOdom/odometry/Odom.hpp"
#include "81yOdom/utils/Math.hpp"
#include "pros/rtos.h"
#include <cmath>

void Odom::update() {
    //Record encoder positions at the start of cycle
    left->recordPosition();
    right->recordPosition();
    back->recordPosition();

    //wait for movement, called after 10ms delay in task

    //Get deltas since last cycle
    double dL = left->getDelta();
    double dR = right->getDelta();
    double dB = back->getDelta();

    //Get heading from IMU (safer then encoder-based heading, which can be inaccurate due to slippage)
    double currTheta = imu->getHeading();
    double dTheta = toRad(normalizeAngle(currTheta - prevTheta));
    prevTheta = currTheta;
    pose.theta = currTheta;


    //Calculate local displacement
    double localX, localY;

    if (std::fabs(dTheta) < 1e-6) {
        //Robot went (nearly) straight - linear approximation
        localX  = dB - (backOffset * dTheta);
        localY = (dL + dR) / 2.0;
    } else {
        //robot turned, use arc chord formula 
        double radiusL = dL / dTheta;
        double radiusR = dR / dTheta; 
        double radius = (radiusL + radiusR) / 2.0; //average radius of the turn
        localY = radius * std::sin(dTheta);

        double radiusB = dB / dTheta - backOffset; //adjust back encoder reading by its offset
        localX = radiusB * (1 - std::cos(dTheta)); //chord length formula for back encoder
    }

    double avgTheta = toRad(pose.theta) - (dTheta / 2.0); //average heading during the movement
    //Convert local displacement to global coordinates
    pose.x += localY * std::sin(avgTheta) + localX * std::cos(avgTheta);
    pose.y += localY * std::cos(avgTheta) - localX * std::sin(avgTheta);
};