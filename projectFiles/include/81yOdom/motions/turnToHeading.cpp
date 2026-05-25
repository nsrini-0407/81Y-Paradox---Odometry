#include "main.h"
#include "pros/rtos.hpp"

void turnToHeading(double targetDeg, double timeoutMs, double maxSpeed, double minSpeed, PID& pid, Odom& odom, pros::MotorGroup& left, pros::MotorGroup& right) {

    pid.reset(); //reset PID state before starting
    SettleCondition settle(1.0, 150); //consider settled when within 1 degree for 150ms
    Timer timer(timeoutMs);
    double prevOut = 0;

    while (!timer.isDone())  {
        double error = odom.getPose().headingError(targetDeg);

        if (settle.update(error)) {
            break; //exit if settled
        }

        double dt = 0.01; //assuming this loop runs every 10ms, adjust if different
        double out = pid.Update(error, dt);

        //Clamp output to max/min speed
        out = std::clamp(out, -maxSpeed, maxSpeed);
        if (std::fabs(out) < minSpeed) {
            out = (out > 0) ? minSpeed : -minSpeed; //enforce minimum speed to prevent stalling
        }

        left.move_voltage(out*120);
        right.move_voltage(-out*120); //invert for opposite side
        
        pros::delay(10); //wait for 10ms before next update
    }

    left.brake();
    right.brake();

}
