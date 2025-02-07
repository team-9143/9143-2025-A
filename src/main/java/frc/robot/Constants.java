package frc.robot;

/*
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	
	// Constants defining IDs and ports for motors, sensors, and other devices.
	public static class DeviceConstants {}

	public final class ElevatorConstants {
		// Motor IDs
		public static final int ELEVATOR_LEFT_ID = 58; // CAN ID for the left elevator motor
		public static final int ELEVATOR_RIGHT_ID = 59; // CAN ID for the right elevator motor

		// Motor Inversion
		public static final boolean ELEVATOR_LEFT_INVERTED = false; // Set to true if the left motor is inverted
		public static final boolean ELEVATOR_RIGHT_INVERTED = false; // Set to true if the right motor is inverted

		// Current Limits
		public static final int ELEVATOR_CURRENT_LIMIT = 60; // Current limit for the elevator motors (in amps)

		// Encoder Conversion Factors
		public static final double ELEVATOR_POSITION_CONVERSION = 1.0; // Convert encoder ticks to position units
		public static final double ELEVATOR_VELOCITY_CONVERSION = 1.0; // Convert encoder ticks to velocity units

		// PID Constants
		public static final double ELEVATOR_kP = 0.1; // Proportional gain for the elevator's PID controller
		public static final double ELEVATOR_kI = 0.0; // Integral gain for the elevator's PID controller
		public static final double ELEVATOR_kD = 0.0; // Derivative gain for the elevator's PID controller

		// Position Limits
		public static final double ELEVATOR_MIN_POSITION = 0.0; // Minimum allowed position for the elevator (in inches)
		public static final double ELEVATOR_MAX_POSITION = 100.0; // Maximum allowed position for the elevator (in inches)

		// Allowed Error
		public static final double ELEVATOR_ALLOWED_ERROR = 1.0; // Allowed error threshold for the elevator to be "at target"

		// Manual Control Parameters
		public static final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual elevator control
		public static final double ELEVATOR_MANUAL_SPEED_LIMIT = 0.5; // Speed limit for manual elevator control

		// Preset Heights
		public static final double ELEVATOR_BASE_HEIGHT = 0.0;
		public static final double ELEVATOR_CORAL_L1_HEIGHT = 20.0;
		public static final double ELEVATOR_CORAL_L2_HEIGHT = 40.0;
		public static final double ELEVATOR_CORAL_L3_HEIGHT = 60.0;
		public static final double ELEVATOR_CORAL_L4_HEIGHT = 80.0;
	}
	
	public final class CorAlConstants {
		// Motor IDs
		public static final int CORAL_PIVOT_MOTOR_ID = 60; // CAN ID for the pivot motor
		public static final int CORAL_INTAKE_MOTOR_ID = 61; // CAN ID for the intake motor

		// Motor Inversion
		public static final boolean CORAL_PIVOT_MOTOR_INVERTED = false; // Set to true if the pivot motor is inverted
		public static final boolean CORAL_INTAKE_MOTOR_INVERTED = false; // Set to true if the intake motor is inverted

		// Current Limits
		public static final int CORAL_PIVOT_CURRENT_LIMIT = 30; // Current limit for the pivot motor (in amps)
		public static final int CORAL_INTAKE_CURRENT_LIMIT = 20; // Current limit for the intake motor (in amps)

		// Encoder Conversion Factors
		public static final double CORAL_PIVOT_POSITION_CONVERSION = 360.0; // Convert encoder rotations to degrees
		public static final double CORAL_PIVOT_VELOCITY_CONVERSION = 360.0; // Convert encoder rotations to degrees per second

		// PID Constants
		public static final double CORAL_PIVOT_kP = 0.02; // Proportional gain for the pivot motor's PID controller
		public static final double CORAL_PIVOT_kI = 0.0; // Integral gain for the pivot motor's PID controller
		public static final double CORAL_PIVOT_kD = 0.0; // Derivative gain for the pivot motor's PID controller

		// Pivot Angle Limits
		public static final double CORAL_PIVOT_MIN_ANGLE = 0.0; // Minimum allowed angle for the pivot (in degrees)
		public static final double CORAL_PIVOT_MAX_ANGLE = 90.0; // Maximum allowed angle for the pivot (in degrees)

		// Allowed Error
		public static final double CORAL_PIVOT_ALLOWED_ERROR = 1.0; // Allowed error threshold for the pivot to be "at target"

		// Manual Control Parameters
		public static final double CORAL_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual pivot control
		public static final double CORAL_MANUAL_SPEED_LIMIT = 0.5; // Speed limit for manual pivot control

		// Pivot Preset Angles
		public static final double CORAL_BASE_ANGLE = 0.0;
		public static final double CORAL_LOW_ANGLE = 5.0;
		public static final double CORAL_MID_ANGLE = 10.0;
		public static final double CORAL_HIGH_ANGLE = 15.0;
		public static final double ALGAE_INTAKE_ANGLE = 20.0;
		public static final double ALGAE_SCORE_ANGLE = 25.0;

		// Roller Speeds
		public static final double CORAL_INTAKE_SPEED = 0.5;
		public static final double ALGAE_INTAKE_SPEED = -0.5;
	}
		
	public final class AlLowConstants {
		// Motor IDs
		public static final int ALLOW_PIVOT_MOTOR_ID = 62; // CAN ID for the pivot motor
		public static final int ALLOW_ROLLER_MOTOR_ID = 63; // CAN ID for the roller motor

		// Motor Inversion
		public static final boolean ALLOW_PIVOT_MOTOR_INVERTED = false; // Set to true if the pivot motor is inverted
		public static final boolean ALLOW_ROLLER_MOTOR_INVERTED = false; // Set to true if the roller motor is inverted

		// Current Limits
		public static final int ALLOW_PIVOT_CURRENT_LIMIT = 30; // Current limit for the pivot motor (in amps)
		public static final int ALLOW_ROLLER_CURRENT_LIMIT = 20; // Current limit for the roller motor (in amps)

		// Encoder Conversion Factors
		public static final double ALLOW_PIVOT_POSITION_CONVERSION = 1.0; // Convert encoder ticks to degrees
		public static final double ALLOW_PIVOT_VELOCITY_CONVERSION = 1.0; // Convert encoder ticks to degrees per second

		// PID Constants for Pivot Motor
		public static final double ALLOW_PIVOT_kP = 0.1; // Proportional gain for the pivot motor's PID controller
		public static final double ALLOW_PIVOT_kI = 0.0; // Integral gain for the pivot motor's PID controller
		public static final double ALLOW_PIVOT_kD = 0.0; // Derivative gain for the pivot motor's PID controller

		// Pivot Angle Limits
		public static final double ALLOW_PIVOT_MIN_ANGLE = 0.0; // Minimum allowed angle for the pivot (in degrees)
		public static final double ALLOW_PIVOT_MAX_ANGLE = 90.0; // Maximum allowed angle for the pivot (in degrees)

		// Allowed Error
		public static final double ALLOW_PIVOT_ALLOWED_ERROR = 1.0; // Allowed error threshold for the pivot to be "at target"

		// Manual Control Parameters
		public static final double ALLOW_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual pivot control
		public static final double ALLOW_MANUAL_SPEED_LIMIT = 0.5; // Speed limit for manual pivot control

		// Pivot Preset Angles
		public static final double ALLOW_BASE_ANGLE = 0.0;
		public static final double ALLOW_INTAKE_ANGLE = 45.0;
	}
}
