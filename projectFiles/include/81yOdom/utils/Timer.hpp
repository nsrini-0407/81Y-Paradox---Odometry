#pragma once
#include "pros/rtos.hpp"

class Timer{
    public:
        explicit Timer(double timeoutMs) 
        : startTime(pros::millis()), timeout(timeoutMs) {}

        bool isDone() const {
            return pros::millis() - startTime >= timeout;
        }

        double getElapsed() const {
            return pros::millis() - startTime;
        }

        void reset() {
            startTime = pros::millis();
        }

        private:
            uint32_t startTime;
            double timeout;
    };