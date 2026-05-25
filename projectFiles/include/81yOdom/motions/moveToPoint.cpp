#include "main.h"
#include "pros/motor_group.hpp"

void moveToPoint(double tx, double ty, double timeoutMs, double maxSpeed, double minSpeed, 
    bool reversed, PID& drivePID, PID& turnPID, Odom& odom, pros::MotorGroup left, pros::MotorGroup right){
        drivePID.reset();
        turnPID.reset();
        SettleCondition settle(0.5, 200); //consider settled when within 0.5 inches for 200ms
        Timer timer(timeoutMs);

        bool close = false;
        double prevDrive = 0;
        double prevTurn = 0;
        const double slewRate = 300.0; //max change in voltage per second

        while (!timer.isDone()) {
            Pose pose = odom.getPose();

            double distance = pose.distanceTo(tx, ty);
            double angleToTarget = pose.angleTo(tx, ty);

            //If reversed, approach with the back of the robot
            double adjustedTheta  = reversed ? normalizeAngle(angleToTarget + 180) : pose.theta;

            double headingError = normalizeAngle(angleToTarget - adjustedTheta);

            //project distance ont foward axis 
            double lateralError = distance * std::cos(toRad(headingError));

            //Enter settling mode when close to target
            if (!close && distance < 7.5) {
                close = true; 
                maxSpeed = std::max(std::fabs(prevDrive), 20.0); //reduce max speed when close, but don't go below 20
            }

            if (settle.update(lateralError) && close) {
                break; //exit if settled and close to target
            }

            double dt = 0.0; 
            double driveOut = drivePID.Update(lateralError, dt);
            double turnOut = close ? 0.0: turnPID.Update(headingError, dt); //stop turning when close to target

            //Clamp 
            driveOut = clamp(driveOut, -maxSpeed, maxSpeed);
            turnOut = clamp(turnOut, -maxSpeed, maxSpeed);

            //Slew
            driveOut = slew(driveOut, prevDrive, slewRate, dt);
            turnOut = slew(turnOut, prevTurn, slewRate, dt);

            //Min drivespeed
            if (!close && std::fabs(driveOut) < minSpeed)
                driveOut = minSpeed * sign(driveOut);

            prevDrive = driveOut;
            prevTurn = turnOut;

            //Motor mixing + desaturation
            double leftOut = driveOut + turnOut;
            double rightOut = driveOut - turnOut;

            double maxOut = std::max(std::fabs(leftOut) , std::fabs(rightOut));
            if (maxOut > maxSpeed) {
                leftOut *= maxSpeed / maxOut;
                rightOut *= maxSpeed / maxOut;
            }

            left.move_voltage(leftOut * 120);
            right.move_voltage(rightOut * 120);
            pros::delay(10);

        }

        left.brake();
        right.brake();
}