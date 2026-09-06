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

// Encoder-only opening routine. Uses no odometry, so it runs even when
// tracking is untrustworthy.
void simple_auton();

// Bot 1's first routine, driven from the five planner exports in static/.
void bot1_auton1();

// A hand-written 24-inch forward path. Sanity check for the follower itself
// rather than a competition routine.
void ramsete_auton_example();

// Empty placeholders.
void example_auton();
void example_auton2();
