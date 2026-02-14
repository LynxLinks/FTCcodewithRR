# Team 16970 Lynx Lynx: Robot Controller Implementation

This repository contains the control software for the Team 16970 Lynx Lynx robot, utilizing the Road Runner library for advanced motion profiling and a custom architectural framework designed for efficiency and reliability.

## System Architecture

The project follows a modular, inheritance-based architecture to ensure code reuse and consistency between Autonomous and TeleOp periods.

### Control Hierarchy
- **Statics.java**: The core base class extending `LinearOpMode`. It manages hardware initialization, global constants, and shared utility methods. This ensures that both TeleOp and Autonomous modes use identical hardware mapping and movement logic.
- **Road Runner Integration**: Built on top of `SampleMecanumDrive`, the system uses spline-based pathing and custom `TrajectorySequence` structures for fluid, high-speed movement.
- **State Persistence**: A `PoseStorage` class is used to transfer the robot's final heading and position from the Autonomous period to TeleOp, enabling persistent field-centric control.

### Navigation and Kinematics
- **Grid-Based Coordinate System**: The robot uses a custom abstraction layer (`math2()`) that translates field grid coordinates into raw X/Y/Heading poses. This allows for high-level command calls like `Drive(x, y, zone)` rather than manual coordinate entry.
- **Field-Centric Localization**: Employs three-wheel odometry (via `StandardTrackingWheelLocalizer`) for precise real-time positioning, decoupled from wheel slippage.

## Core Functionality

### 1. Autonomous Capabilities
The Autonomous implementation (`Auto14.java`) provides high-scoring reliability through several automated sub-systems:

- **Vision Processing**: Uses TensorFlow Object Detection (TFOD) via Vuforia to identify randomized game elements and determine optimal parking zones.
- **Wall-Relative Calibration**: Utilizes side-mounted distance sensors (`D2`, `D4`) during initialization to calibrate the robot's starting position relative to the field walls, mitigating human placement error.
- **Trajectory Cycling**: A state-driven `Cycle()` method manages the transition between intake and scoring positions, allowing the robot to execute multiple scoring loops within the 30-second window.

### 2. TeleOp Features
The TeleOp mode (`DriveV10.java`) focuses on driver assistance and sub-system automation:

- **Field-Centric Mecanum Drive**: The drive base orientation is decoupled from the driver's perspective, allowing for intuitive navigation regardless of the robot's heading.
- **Automated Linear Slides**:
  - **Auto-Calibration**: Uses a digital touch sensor (`D0`) for automatic zero-point calibration on startup.
  - **Motion Profiling**: Uses a custom sigmoid-style power curve for smooth slide acceleration and deceleration.
  - **Preset Heights**: Quick-access height presets are mapped to gamepad buttons for varying scoring levels.
- **Sensor-Driven Intake**: A combination of distance sensors (`D1`) and magnetic sensors (`D5`) triggers an automated clamping sequence once a game element is detected in the intake, reducing driver workload.

### 3. Hardware Configuration
- **Drive Base**: 4-motor Mecanum configuration.
- **Linear Motion**: Single-motor linear slide system with encoder feedback and limit switch protection.
- **Sensors**: 
  - 4x Distance sensors for field localization and object detection.
  - 2x Digital channels for mechanical limits.
  - 1x Webcam for computer vision.

## Implementation Details

### Slide Control Logic
The slide movement uses a non-blocking `UntilSlide()` method, allowing the robot to continue pathing or executing other tasks while the slides are in motion. This parallel processing is critical for maximizing efficiency in both game periods.

### Adaptive Pathing
The `math2()` logic includes "stagger" offsets for red and blue alliances, automatically adjusting trajectories based on the detected starting position and chosen alliance.
