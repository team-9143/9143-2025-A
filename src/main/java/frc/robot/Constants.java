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
		public static final double ELEVATOR_POSITION_CONVERSION = 0.0146; // Convert encoder ticks to position units
		public static final double ELEVATOR_VELOCITY_CONVERSION = 0.0146 / 60.0; // Convert RPM to inches per second

		// PID Constants
		public static final double ELEVATOR_kP = 1.0; // Proportional gain for the elevator's PID controller
		public static final double ELEVATOR_kI = 0.0002; // Integral gain for the elevator's PID controller
		public static final double ELEVATOR_kD = 0.08; // Derivative gain for the elevator's PID controller
		public static final double ELEVATOR_kF = 0.15; // Feed forward for gravity compensation

		// Position Limits
		public static final double ELEVATOR_MIN_POSITION = 0.0; // Minimum allowed position for the elevator (in inches)
		public static final double ELEVATOR_MAX_POSITION = 20.5; // Maximum allowed position for the elevator (in inches)

		// Allowed Error
		public static final double ELEVATOR_ALLOWED_ERROR = 0.5; // Allowed error threshold for the elevator to be "at target"

		// Manual Control Parameters
		public static final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual elevator control
		public static final double ELEVATOR_MANUAL_SPEED_LIMIT = 0.5; // Speed limit for manual elevator control

		// Preset Heights
		public static final double ELEVATOR_BASE_HEIGHT = 0.0;
		public static final double ELEVATOR_CORAL_L2_HEIGHT = 2.0;
		public static final double ELEVATOR_CORAL_L3_HEIGHT = 8.0;
		public static final double ELEVATOR_CORAL_L4_HEIGHT = 20.5;
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
		public static final double CORAL_PIVOT_POSITION_CONVERSION = 360.0 / 2241.6; // Convert encoder ticks to degrees
		public static final double CORAL_PIVOT_VELOCITY_CONVERSION = 360.0 / 2241.6; // Convert encoder RPS to degrees per second

		// PID Constants
		public static final double CORAL_PIVOT_kP = 12.0; // Proportional gain for the pivot motor's PID controller
		public static final double CORAL_PIVOT_kI = 0.0; // Integral gain for the pivot motor's PID controller
		public static final double CORAL_PIVOT_kD = 0.5; // Derivative gain for the pivot motor's PID controller
		public static final double CORAL_PIVOT_kG = 0.15; // Gravity compensation feed forward

		// Pivot Angle Limits
		public static final double CORAL_PIVOT_MIN_ANGLE = 0.0; // Minimum allowed angle for the pivot (in degrees)
		public static final double CORAL_PIVOT_MAX_ANGLE = 160.0; // Maximum allowed angle for the pivot (in degrees)

		// Allowed Error
		public static final double CORAL_PIVOT_ALLOWED_ERROR = 0.5; // Allowed error threshold for the pivot to be "at target"

		// Manual Control Parameters
		public static final double CORAL_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual pivot control
		public static final double CORAL_MANUAL_SPEED_LIMIT = 0.5; // Speed limit for manual pivot control

		// Pivot Preset Angles
		public static final double CORAL_BASE_ANGLE = 0.0;
		public static final double CORAL_RAISE_ANGLE = 90.0;
		public static final double ALGAE_INTAKE_ANGLE = 156.0;
		public static final double ALGAE_SCORE_ANGLE = 135.0;

		// Roller Speeds
		public static final double CORAL_INTAKE_SPEED = 0.2;
		public static final double ALGAE_INTAKE_SPEED = -0.2;

		// Through Bore ID
		public static final int THROUGH_BORE_DIO_PORT = 0;

		// Through Bore Angles
		public static final double THROUGH_BORE_MIN_ANGLE = 0.0;
        public static final double THROUGH_BORE_MAX_ANGLE = 160.0;
        public static final double THROUGH_BORE_ALLOWED_DISCREPANCY = 2.0; 
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

	public static final class VisionConstants {
        // Limelight names
        public static final String[] LIMELIGHT_NAMES = {"funnel", "barge", "reef"};

        // AprilTag pipeline index
        public static final int APRILTAG_PIPELINE = 0;

        // Distance offsets for AprilTags (in meters)
        public static final double[][] APRILTAG_OFFSETS = {
            // Format: {horizontalOffset, verticalOffset}
            {0.0, 0.0}, // Tag 1
            {0.0, 0.0}, // Tag 2
			{0.0, 0.0}, // Tag 3
			{0.0, 0.0}, // Tag 4
			{0.0, 0.0}, // Tag 5
			{0.0, 0.0}, // Tag 6
			{0.0, 0.0}, // Tag 7
			{0.0, 0.0}, // Tag 8
			{0.0, 0.0}, // Tag 9
			{0.0, 0.0}, // Tag 10
			{0.0, 0.0}, // Tag 11
			{0.0, 0.0}, // Tag 12
			{0.0, 0.0}, // Tag 13
			{0.0, 0.0}, // Tag 14
			{0.0, 0.0}, // Tag 15
			{0.0, 0.0}, // Tag 16
			{0.0, 0.0}, // Tag 17
			{0.0, 0.0}, // Tag 18
			{0.0, 0.0}, // Tag 19
			{0.0, 0.0}, // Tag 20
			{0.0, 0.0}, // Tag 21
			{0.0, 0.0}, // Tag 22
        };

        // Tracking gains
        public static final class TrackingGains {
            public static final double DISTANCE_kP = 0.1;
            public static final double ANGLE_kP = 0.05;
            public static final double ROTATION_kP = 0.03;
            public static final double VELOCITY_DEADBAND = 0.05;
            public static final double ROTATION_DEADBAND = 0.02;
            public static final double MAX_LINEAR_VELOCITY = 2.0; // m/s
            public static final double MAX_ANGULAR_VELOCITY = 1.0; // rad/s
        }
    }
}
