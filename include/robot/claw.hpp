#pragma once

// Main pneumatic claw, independent of the lift.
// The main claw's logical retracted state is physically closed.
void toggleOpenClaw();

// Y toggles once per click. Button release does nothing.
// Closing enables the raised lift floor; opening near zero requests its return.
void handleClawControl();
