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
        bool settling = false;

    public: 
        SettleCondition(double threshold, double settleTimeMs) 
            : threshold(threshold), settleTimeMs(settleTimeMs) {}

    bool update (double error) {
        if (std::fabs(error) < threshold) {
            if (!settling) {
                settleStart = pros::millis();
                settling = true;
            }
            return pros::millis() - settleStart >= settleTimeMs;

        }
        settling = false;
        return false; //not settled yet
    }

    void reset() {
        settleStart = 0;
        settling = false;
    } //reset the settle timer, call when starting a new movement
};