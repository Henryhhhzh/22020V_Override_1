#pragma once

// Every autonomous routine. One routine per file in src/autons/, so opening a
// routine shows you its whole sequence -- segments, mechanism moves and DSR
// resets -- with nothing in between to scroll past.
//
// Pick which one runs in autonomous() at the bottom of main.cpp.
//
// To add a routine: create src/autons/<name>.cpp, ASSET() the segments it uses
// at the top of that file, and add one line here. The PROS build compiles
// everything under src/ automatically, so there is no Makefile change.

// Restored encoder drive sequence with the rebuilt lift and main claw.
void simple_auton();

// Bot 1's first routine, driven from the five planner exports in static/.
void bot1_auton1();

// Four numbered planner paths in static/auton_26_september/.
void auton_26_september();

// Two-part circle test from static/circle_test/.
void ramsete_auton_example();
