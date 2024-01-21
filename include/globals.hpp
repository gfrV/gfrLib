#pragma once

#include "gfrLib/chassis.hpp"

#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"

extern gfrLib::Chassis chassis;
extern pros::Rotation leftrot;
extern pros::Rotation rightrot;
extern pros::Rotation middlerot;


extern pros::Controller master;