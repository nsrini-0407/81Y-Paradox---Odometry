#pragma once
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>

//Exits the movement when error stays below threshold for a required duration 
class SettleCondition {
    private: 
        double threshold; //error threshold to consider "settled"
        double settleTimeMs; //time in ms that error must stay below threshold to exit
        uint32_t settleStart = 0; //time when error first went below threshold

    public: 
        SettleCondition(double threshold, double settleTimeMs) 
            : threshold(threshold), settleTimeMs(settleTimeMs) {}

    bool update (double error) {
        if (std::fabs(error) < threshold) {
            if (settleStart == 0) settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTimeMs) {
                return true; //error has been below threshold for required time, exit
            } else {
                settleStart = 0; //reset timer if error goes above threshold
            }

        }
        return false; //not settled yet
    }

    void reset() {settleStart = 0;} //reset the settle timer, call when starting a new movement
};