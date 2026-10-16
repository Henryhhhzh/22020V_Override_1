# Retired cascade and arm robot

This archive holds the mechanisms from the robot before the rebuild. It is
outside `src/`, so PROS does not compile it or claim any of its device ports.

## Contents

- `src/robot/mechanisms.cpp`: the controller formerly embedded in `src/main.cpp`,
  including atomic targets, holding and movement logic, presets, manual controls,
  controller readouts, claw/intake functions, and the old startup and driver hooks.
- `src/robot/hardware.cpp` and `include/robot/hardware.hpp`: old mechanism devices
  and their port assignments.
- `include/robot/config.hpp`: old mechanism types, targets, limits, tolerances,
  speeds, holding flags, and button assignments.
- `include/robot/{arm,cascade,claw,mechanisms}.hpp`: the original mechanism APIs.
- `src/robot/{arm,cascade}.cpp`: the unfinished modular copies. These were already
  excluded from the old build because they duplicate definitions in the controller
  extracted from `main.cpp`. They are retained for reference, not as additional
  modules to compile with `mechanisms.cpp`.
- `src/autons/simple_auton.cpp`: the complete old routine that uses the arm,
  cascade, claws, and encoder drive helpers, plus its empty example routines.
- `include/autons/autons.hpp`: declarations for those archived routines.

## Former mechanism ports

| Device | Assignment |
| --- | --- |
| Arm motor | Smart port 4 |
| Cascade motor | Smart port 20, reversed |
| Intake motor | Smart port 11 |
| Arm rotation sensor | Smart port 5, reversed |
| Variable claw | ADI H, initially retracted |
| Main open claw | ADI F, initially retracted, inverted |

## Active program

Drivetrain motors, driver controller, IMU, horizontal odometry, chassis geometry,
motion controller settings, Ramsete, DSR, encoder drive helpers, and all path-only
autonomous routines remain in the active project. Their settings and behavior
were preserved during this move. The historical `old_program_22020X_PB.txt` and
standalone DSR examples were also left as they were.

The old mechanism task, initialization, displays, and driver bindings are no longer
called. The original `simple_auton()` remains here for reference. A replacement
`src/autons/simple_auton.cpp` now uses the rebuilt lift and main claw and is selected
in `autonomous()`; it preserves the old drive sequence with trial lift heights.
Driver control still uses the same arcade inputs and 50 ms loop delay.

Only the main pneumatic claw is restored to active `robot/claw.hpp` and
`src/robot/claw.cpp`, using ADI F and R2. Closing it no longer raises the retired
cascade. The variable claw (ADI H and Y), intake, arm, cascade, and their controls
remain archived. The new lift has its own active file pair.

This is a source archive, not an independently buildable replacement project.
Headers retain their original `robot/...` include names for reference; restoring
the code requires deliberately integrating the archived headers and source with
the active shared controller and drive helpers. Do not add the entire archive to
the build: the unfinished modular copies duplicate the old controller definitions.
