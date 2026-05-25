#pragma once
#include <algorithm>
#include <cmath>


struct PIDConfig {
    double kP;
    double kI; 
    double kD; 
    double integralWindupLimit = 50.0; //clamp to prevent excessive integral buildup
    double smallErrorThreshold = 0.0; //within this stop accumulating I term, to prevent windup when close to target
};

class PID {

    explicit PID(PIDConfig config) : config(config) {}

    public:
    
        double Update (double error, double dt) {
            //only accumulate integral if error is above threshold to prevent windup when close to target
            if (std::fabs(error) > config.smallErrorThreshold) {
                integral += error * dt; 
            }
            //Anti-windup clamp 
            integral = std::clamp(integral, -config.integralWindupLimit, config.integralWindupLimit);

            double derivative = (dt > 0) ? (error - prevError) / dt: 0.0; 
            prevError = error;

            return (config.kP * error) + (config.kI * error) + (config.kD * error);
        }

        void reset() {
            integral = 0.0;
            prevError = 0.0;
        }

        void setConfig(PIDConfig newConfig) {config = newConfig;}

    private: 
    PIDConfig config;
    double integral = 0.0; 
    double prevError = 0.0;
};