#pragma once
#include "pros/motors.hpp"
#include <string>
#include <vector>
#include <functional>

// ============================================================================
//  81Y Paradox - Custom Brain GUI
//  Drop this header + gui.cpp + logo.c into your PROS project.
// ============================================================================

namespace gui {

// ---- Autonomous selector -------------------------------------------------
// Add one entry per autonomous routine you have. `name` shows on the button,
// `run` is the function that gets called if this routine is selected.
struct AutonRoutine {
    std::string name;
    std::function<void()> run;
};

// Register your routines here (edit the .cpp file's g_routines vector).
// Call this from autonomous() in main.cpp to execute whatever was selected.
void runSelectedAuton();

// Returns the index of the currently selected auton (-1 if none picked yet).
int getSelectedAutonIndex();
std::string getSelectedAutonName();

// ---- Motor watch list -------------------------------------------------
// A motor to display on the "Motors" tab, with a friendly label.
struct WatchedMotor {
    std::string label;      // e.g. "L Front"
    pros::Motor* motor;     // pointer to your existing motor object
};

// Call once, before gui::init(), to tell the GUI which motors to monitor.
void setWatchedMotors(const std::vector<WatchedMotor>& motors);

// ---- Logo --------------------------------------------------------------
// Optional. Pass a pointer to your converted logo's `lv_image_dsc_t`
// (declared via LV_IMAGE_DECLARE in the file the LVGL image converter gives
// you). Call before gui::init(). If you never call this, the Home tab shows
// a plain placeholder box instead. Takes `const void*` so this header
// doesn't need to include LVGL.
void setLogoImage(const void* img_dsc);

// ---- Lifecycle -------------------------------------------------------
// Call once from initialize() in main.cpp, after motors are constructed.
void init();

}  // namespace gui