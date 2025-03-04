# Team 9143 - 2025 Robot Code

This repository contains the code for Team 9143's 2025 FRC robot, designed for the 2025 FIRST Robotics Competition game.

---

## Robot Overview

Our 2025 robot features:

- **Swerve Drivetrain**: Utilizes the CTRE Phoenix 6 library for precise and agile movement.
- **Elevator Subsystem**: Handles vertical movement for game piece manipulation.
- **CorAl (Coral and Algae) Subsystem**: Manages game piece scoring with a pivoting mechanism and intake system.
- **AlLow Subsystem**: Provides additional game piece handling capabilities.
- **Vision System**: Uses multiple Limelight cameras for AprilTag tracking and vision-assisted driving.
- **Telemetry System**: Provides real-time data logging and visualization for debugging and performance monitoring.
- **Autonomous Routines**: Built using PathPlanner for efficient and reliable autonomous operation.

---

## Subsystems

### Drivetrain
The **Swerve Drivetrain** is the core of the robot's movement system, providing precise control and agility.

#### Swerve Class
The `Swerve` class extends the CTRE Phoenix 6 `SwerveDrivetrain` class and implements the `Subsystem` interface. It provides the following functionality:
- **Swerve Drive Control**: Manages the swerve drive modules, including field-centric and robot-centric driving.
- **Vision Integration**: Integrates with the vision subsystem for AprilTag tracking and vision-assisted driving.
- **SysId Routines**: Supports system identification (SysId) routines for characterizing translation, steer, and rotation.
- **Autonomous Path Following**: Configures the `AutoBuilder` for PathPlanner autonomous routines.
- **Shuffleboard Integration**: Displays robot pose, speeds, and module states on Shuffleboard.
- **Simulation Support**: Includes a simulation thread for testing in a simulated environment.

---

### Elevator
The **Elevator Subsystem** is responsible for vertical movement, enabling the robot to manipulate game pieces at different heights.

#### Elevator Class
The `Elevator` class extends `SubsystemBase` and provides the following functionality:
- **Dual-Motor Control**: Manages two SparkMax motors for the elevator, ensuring synchronized movement.
- **Closed-Loop Control**: Uses PID control with MAXMotion for smooth and precise position control.
- **Manual Control**: Allows manual override of the elevator position for fine-tuning during operation.
- **Safety Checks**: Monitors encoder synchronization and ensures the elevator stays within safe bounds.
- **Shuffleboard Integration**: Displays elevator status, position, and motor information on Shuffleboard.
- **Encoder Management**: Resets and tracks encoder positions to ensure accurate height control.

---

### CorAl (Coral and Algae)
The **CorAl Subsystem** handles the manipulation and scoring of game elements, including coral and algae.

#### CorAl Class
The `CorAl` class extends `SubsystemBase` and provides the following functionality:
- **Pivot Mechanism Control**: Manages the pivot mechanism using a TalonFX motor with position control for precise angle adjustments.
- **Intake Mechanism Control**: Controls the intake rollers using a TalonFX motor for game piece manipulation.
- **Game Piece Detection**: Uses a CANrange sensor to detect game pieces and confirm their presence with temporal filtering.
- **Dual Encoder Feedback**: Utilizes a motor encoder and through bore encoder for accurate position tracking and redundancy.
- **Safety Checks**: Ensures the pivot mechanism stays within safe angular limits and monitors encoder discrepancies.
- **Shuffleboard Integration**: Displays pivot angle, intake status, and game piece detection information on Shuffleboard.
- **Manual Override**: Allows manual control of the pivot and intake mechanisms for fine-tuning during operation.

---

### AlLow
The **AlLow Subsystem** provides additional game piece handling capabilities, complementing the CorAl subsystem.

#### AlLow Class
The `AlLow` class extends `SubsystemBase` and provides the following functionality:
- **Pivot Mechanism Control**: Manages the pivot mechanism using a SparkMax motor with position control for precise angle adjustments.
- **Roller Mechanism Control**: Controls the roller mechanism using a SparkMax motor for game piece handling.
- **Closed-Loop Control**: Uses PID control with feedforward for smooth and precise position control of the pivot mechanism.
- **Safety Checks**: Ensures the pivot mechanism stays within safe angular limits and monitors motor currents.
- **Shuffleboard Integration**: Displays pivot angle, roller status, and motor information on Shuffleboard.
- **Manual Override**: Allows manual control of the pivot and roller mechanisms for fine-tuning during operation.

---

### Vision
The **Vision System** uses multiple Limelight cameras to detect AprilTags and assist with robot localization and targeting.

#### Vision Class
The `Vision` class extends `SubsystemBase` and provides the following functionality:
- **AprilTag Tracking**: Detects and tracks AprilTags using multiple Limelight cameras for precise localization.
- **Robot Pose Estimation**: Provides real-time robot pose estimates based on AprilTag detections for vision-assisted localization.
- **Target Detection**: Identifies the best AprilTag target from all available Limelight cameras for accurate targeting.
- **Shuffleboard Integration**: Displays target information, tracking status, and pose estimation data on Shuffleboard.
- **LED Control**: Manages Limelight LED states to enable or disable tracking as needed.
- **Position Tracking**: Updates the robot's position using vision data when enabled, improving localization accuracy.

---

### Telemetry
The **Telemetry System** provides real-time data logging and visualization for monitoring and debugging the robot's performance.

#### Telemetry Class
The `Telemetry` class provides the following functionality:
- **Real-Time Data Logging**: Logs robot state, including pose, speeds, and module states, to NetworkTables for live telemetry.
- **Mechanism2d Visualization**: Visualizes swerve module states using Mechanism2d for easy debugging and monitoring.
- **SignalLogger Integration**: Records telemetry data to a log file for post-match analysis.
- **Field2d Integration**: Displays the robot's pose on a field visualization for real-time positioning feedback.
- **Module State Visualization**: Provides detailed visual feedback on each swerve module's state, including speed and direction.

---

## Configuration

The robot's configuration constants are contained in the `Constants.java` file, organized into inner classes for each subsystem:

- `ElevatorConstants`: Settings for the Elevator subsystem.
- `CorAlConstants`: Settings for the CorAl subsystem.
- `AlLowConstants`: Settings for the AlLow subsystem.
- `VisionConstants`: Settings for the Vision subsystem.

### TunerConstants Class
The `TunerConstants` class is generated by the CTRE Tuner X Swerve Project Generator and provides the following functionality:
- **Swerve Drive Configuration**: Contains all the necessary constants and configurations for the swerve drivetrain, including motor IDs, gear ratios, encoder offsets, and PID gains.
- **Module Constants**: Defines the configuration for each swerve module, including drive and steer motor IDs, encoder offsets, and module positions.
- **Drivetrain Constants**: Configures the overall swerve drivetrain, including CAN bus settings, Pigeon 2 IMU configuration, and simulation parameters.
- **Swerve Module Factory**: Provides a factory method for creating swerve module constants with the specified configurations.
- **Swerve Drivetrain Creation**: Includes a method to create a `Swerve` drivetrain instance using the configured constants.

---

## Autonomous

Autonomous routines are created using **PathPlanner** and are selectable via a `SendableChooser` on the Shuffleboard "Auto" tab. These routines enable the robot to perform complex tasks autonomously, such as navigating the field, scoring game pieces, and returning to a starting position.

---

## Getting Started

To set up and deploy the robot code, follow these steps:

1. **Clone this repository** to your local machine.
2. **Open the project** in WPILib VSCode.
3. **Build and deploy** the code to the robot.

---

## Dependencies

The robot code relies on the following libraries and tools:

- **WPILib**: The core library for FRC robot programming.
- **Phoenix 6**: Provides support for CTRE motor controllers and sensors.
- **REVLib**: Supports REV Robotics motor controllers and sensors.
- **PathPlanner**: Used for generating and following autonomous paths.
- **LimelightHelpers**: A utility class for interacting with Limelight cameras.

### LimelightHelpers Class
The `LimelightHelpers` class simplifies interaction with Limelight cameras and provides the following functionality:
- **Target Detection**: Retrieves information about detected targets, including their position, size, and ID.
- **Robot Pose Estimation**: Provides methods to get the robot's estimated pose on the field based on AprilTag detections.
- **Camera Control**: Allows control over Limelight camera settings, such as LED mode, pipeline selection, and streaming mode.
- **Data Retrieval**: Provides easy access to various Limelight data points, such as target coordinates, latency, and camera status.
- **Field2d Integration**: Supports integration with WPILib's `Field2d` class for visualizing the robot's position on the field.
