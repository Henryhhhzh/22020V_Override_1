#pragma once

// Call lift_reset(), lift_up(), lift_manual_down(), then lift_down() each driver loop.
// All stops use the configured PROS HOLD brake. Driver controls are nonblocking.
// D-pad Right returns to encoder zero regardless of claw state, cancelling
// pending sequences. Encoder zero itself never changes. The reset continues
// after release; another lift input can interrupt once Right is released.
// Repeated reset presses only reset the lift; there is no tilt action.
void lift_reset();

// L1 raises to the upper limit and cancels any pending lowering sequence.
void lift_up();

// L2 lowers only while held, respecting the current claw-dependent floor.
// Release holds with PROS HOLD. Cancels scoring sequences without changing the claw.
// Reset and up take priority; manual down takes priority over scoring down.
void lift_manual_down();

// D-pad Down lowers slowly while held. Release opens the claw, holds for the configured
// minimum wait and driver pull-away distance, then resets to encoder zero
// at the up speed. Another D-pad Down press takes manual control. Claw toggles change the
// manual minimum without restarting/cancelling the wait or moving a high HOLD position.
// A manual claw click in the same iteration as D-pad Down release overrides auto-opening.
void lift_down();

// Called once per manual claw toggle. Opening near zero requests a return;
// closing cancels that pickup return. The driver loop always enforces a fixed
// raised manual minimum while closed, including for the preload on driver enable.
// A completed reset holds zero until another lift command leaves reset mode.
// High claw toggles preserve HOLD. Scoring wait timers are independent.
void requestLiftPickupAssist(bool clawClosed);

// Blocking autonomous move, independent of controller buttons. Clamp the target
// to the live claw-dependent minimum and upper limit, except targets at/below
// zero reset to zero. Return false on timeout,
// disabled state, or bad encoder readings; always finish with PROS HOLD braking.
// Encoder zero is preserved. Do not run concurrently with driver lift controls.
bool lift_move_to(double targetDegrees, int velocityRpm, unsigned int timeoutMs);
