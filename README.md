# Team 16970 Lynx Lynx: FTC Robot Code (Road Runner + Assisted Driver Controls)

This repository contains the Team 16970 Lynx Lynx FTC robot codebase. It is built around Road Runner motion planning and a shared “robot base” class that is reused across TeleOp and Autonomous opmodes.

The core value this code provides to the robot is **assisted control**: during TeleOp, drivers can **select target coordinates** and have the robot drive there using Road Runner trajectories while the code **manages slide and clamp/outtake state** to reduce driver workload, increase consistency, and improve cycle speed.

## Repository Layout

- `FtcRobotController/`: FTC SDK controller app (mostly upstream SDK code + samples)
- `TeamCode/`: team-specific robot code (TeleOp/Auto, drive base, mechanisms)
- `MeepMeepTesting/`: desktop MeepMeep module for trajectory/logic visualization

## Architecture Overview

### Shared Robot Base

Most match code inherits from `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Statics.java`, which acts as a shared base layer for:

- **Hardware mapping** (motors, servos, sensors)
- **Road Runner setup** via `SampleMecanumDrive` + `TrajectorySequence`
- **TeleOp “UI” state** (selected target coordinate, scoring mode flags, etc.)
- **Mechanism control** (slide control, clamp/outtake sequences)
- **High-level motion primitive**: `Drive(x, y, w, savepos)` which plans and follows a trajectory to a selected target

### Road Runner Drive + Localization

- **Drive**: `TeamCode/.../drive/SampleMecanumDrive.java`
- **Localization**: `TeamCode/.../drive/StandardTrackingWheelLocalizer.java` (3 tracking wheels / odometry pods)

The drive base uses `TrajectorySequence` planning (splines, turns, waits) and updates continuously in both TeleOp and Auto using `drive.update()`.

### Pose Persistence (Auto → TeleOp)

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PoseStorage.java` stores a pose and some selection state so TeleOp can start with a reasonable pose estimate after Autonomous.

## TeleOp Functionality (Driver Assistance)

The current TeleOp in this repo is `DriveV10` (`@TeleOp(name = "DriveV10")`). It combines manual driving with a coordinate-driven assisted mode.

### Manual Field-Centric Drive With Heading Assist

In `Statics.manual()`:

- The left stick commands translation (field-centric transform using the current pose heading).
- The right stick can set a **desired heading**; the code computes heading error and applies a smooth corrective turn.
- Triggers can override turn control for fine manual adjustments.

### Coordinate Targeting (“Drive To Selection”)

Drivers can select a target and send the robot there without hand-driving the full approach:

- **Selection state** is stored as `(xcordset, ycordset, wcordset)` inside `Statics`.
- The `Statics.UI()` method updates these values from gamepad inputs.
- Pressing the “go” control triggers `Statics.Drive(x, y, w, savepos)` which:
  - Computes an approach pose and heading using `math2(...)` (a grid/field abstraction).
  - Builds a Road Runner `TrajectorySequence` (including an approach/back-up distance and spline segments).
  - Starts the trajectory asynchronously and keeps updating it while also servicing the slide control loop.

This is the primary feature that turns TeleOp into a repeatable, semi-automated scoring workflow.

### Slide Control (Single Motor) + Homing

The slide is controlled by a single motor (`M0_2`) with two operating modes:

- **Normal mode**: closed-loop position control toward `target` using a smooth power curve (sigmoid-style).
- **Homing/calibration mode**: uses a digital limit switch (`D0`) to find a repeatable zero and reset the encoder.

Both TeleOp and Auto call `Slide()` frequently so the slide keeps moving while driving/trajectory-following continues.

### Clamp / Outtake State Management

The code coordinates the clamp and outtake with dedicated sequences:

- **Clamp sequence**: `Statics.ServoClamp()`
  - Closes the clamp servo (`S0`) then uses sensor state (digital input `D5`) and slide position to complete the intake/secure routine.
  - Opens to a scoring configuration depending on the selected wall/side (`wcordset`).
- **Drop sequence**: `Statics.drop()`
  - Adjusts the clamp position and then performs a controlled slide move to release a game piece consistently.
  - Sets outtake servos (`S1`, `S2`) to defined “carry/score” positions.

`DriveV10.ServoTrigger()` also supports **sensor-driven clamp triggering** while “at the wall” (front distance sensor + digital sensor gating), plus manual override buttons for clamp/drop.

## Autonomous Functionality

There are multiple Auto opmodes in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. The current Autonomous in this repo is `Auto14` (`@Autonomous(name="Auto14")`). It combines vision-based selection with repeatable cycling logic, and demonstrates:

- **Vision-based selection** using TFOD + Vuforia (webcam input) to determine a zone/label.
- **Wall-relative calibration** using side distance sensors to reduce start-placement error.
- **Cycle-based scoring loop** (`Cycle()`): repeat clamp → drive to target → drop, then park.
- **Park selection** driven by the detected zone.

Auto opmodes reuse the same base mechanisms and trajectory-following loop from `Statics` to keep behavior consistent between periods.

## Hardware Configuration (FTC Configuration Names)

These names must match the Robot Controller configuration on the phone.

### Drivetrain Motors (Mecanum)

- `M0`: drive motor (mapped as left front in `SampleMecanumDrive`)
- `M1`: drive motor (right front)
- `M2`: drive motor (right rear)
- `M3`: drive motor (left rear)

### Linear Slide (Single Motor)

- `M0_2`: linear slide motor (encoder-based, brake enabled)

### Odometry / Tracking Wheel Encoders

Used by `StandardTrackingWheelLocalizer`:

- `M1_2`: left tracking encoder
- `M2_2`: front (lateral) tracking encoder
- `M3_2`: right tracking encoder

### Servos

- `S0`: clamp servo (“cam” positions: closed / top-open / open)
- `S1`: outtake linkage servo (“umbrella”)
- `S2`: outtake linkage servo (“umbrella”)

### Digital Inputs

- `D0`: slide limit switch (homing / encoder reset)
- `D5`: clamp/sequence gating input (used in clamp automation and TeleOp trigger logic)

### Distance Sensors

- `D1`: front distance sensor (used for TeleOp intake/clamp triggering logic)
- `D2`: right distance sensor (used by some Auto init/calibration logic)
- `D3`: back distance sensor
- `D4`: left distance sensor (used by some Auto init/calibration logic)

### Camera

- `Webcam 1`: webcam used for Vuforia/TFOD in Auto opmodes

## Development Notes

- Many constants are `@Config` fields (FTC Dashboard compatible), allowing live tuning without redeploying.
- `MeepMeepTesting/` can be used to visualize trajectories and validate field-coordinate math without running on hardware.
