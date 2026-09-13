#include "main.h"

void driveStraight(double distanceIn, double targetHeading, double timeoutMs, double maxSpeed, PID& drivePID, PID& headingPID, Odom& odom, pros::MotorGroup& left, pros::MotorGroup& right) {

    drivePID.reset(); //reset PID state before starting
    headingPID.reset();
    SettleCondition settle(0.5, 200); //consider settled when within 0.5 inches for 150ms
    Timer timer(timeoutMs);

    Pose startPose = odom.getPose();

    while (!timer.isDone())  {
        Pose current = odom.getPose();

        //Distance traveled from start
        double traveled = current.distanceTo(startPose.x, startPose.y);
        double distError = distanceIn - traveled;

        //Heading correction
        double headingError = current.headingError(targetHeading);

        if (settle.update(distError)) {
            break; //exit if settled
        }

        double dt = 0.01; //assuming this loop runs every 10ms, adjust if different
        double driveOut = drivePID.Update(distError, dt);
        double headingOut = headingPID.Update(headingError, dt);

        driveOut = clamp(driveOut, -maxSpeed, maxSpeed);
        
        double leftOut = driveOut + headingOut;
        double rightOut = driveOut - headingOut;

        //Desaturate: if either output is above max, scale both down proportionally
        double maxOut = std::max(std::fabs(leftOut), std::fabs(rightOut));
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