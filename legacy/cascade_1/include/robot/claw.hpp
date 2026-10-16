#pragma once

// Pneumatic claws and the intake. The definitions currently live in main.cpp
// alongside the other mechanism code; this header exists so autons in
// src/autons/ can reach them.

// Toggle the main claw. Logical retracted means physically closed for this
// inverted solenoid. Each time it closes, the cascade is raised from wherever
// it currently sits by clawCloseCascadeLiftDegrees.
void toggleOpenClaw();

void toggleVariableClaw();

// Run the intake outward only while the outtake button is held.
void handleOuttakeControl();
