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
		public static final double ELEVATOR_POSITION_CONVERSION = 0.0291; // Inches per count (based on encoder resolution and gear ratio)
		public static final double ELEVATOR_VELOCITY_CONVERSION = 0.000485; // Inches per second per count (based on encoder resolution and gear ratio)
		
		// Motion Profile
		public static final double ELEVATOR_MAX_VELOCITY = 1000; // Maximum velocity for the elevator (in encoder units per second)
		public static final double ELEVATOR_MAX_ACCELERATION = 500; // Maximum acceleration for the elevator (in encoder units per second squared)

		// PID Constants
		public static final double ELEVATOR_kP = 10.0; // Proportional gain for the elevator's PID controller
		public static final double ELEVATOR_kI = 0.0; // Integral gain for the elevator's PID controller
		public static final double ELEVATOR_kD = 0.0; // Derivative gain for the elevator's PID controller
		public static final double ELEVATOR_kF = 0.15; // Feedforward gain for gravity compensation

		// Position Limits
		public static final double ELEVATOR_MIN_POSITION = -0.25; // Minimum allowed position for the elevator (in inches)
		public static final double ELEVATOR_MAX_POSITION = 52.0; // Maximum allowed position for the elevator (in inches)

		// Allowed Error
		public static final double ELEVATOR_ALLOWED_ERROR = 0.5; // Allowed error threshold for the elevator to be "at target" (in inches)

		// Manual Control Parameters
		public static final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual elevator control
		public static final double ELEVATOR_MANUAL_SPEED_LIMIT = 0.25; // Speed limit for manual elevator control

		// Preset Heights
		public enum PresetHeights {
			BASE(0.0),
			CORAL_L2(12.0),
			CORAL_L3(28.0),
			CORAL_L4(200),
			ALGAE_L2(20.5),
			ALGAE_L3(37.5),
			ALGAE_NET(52.0);

			private final double height;

			PresetHeights(double height) {
				this.height = height;
			}

			public double getHeight() {
				return height;
			}
		}
	}
	
	public final class CorAlConstants {
		// Motor IDs
		public static final int CORAL_PIVOT_MOTOR_ID = 60; // CAN ID for the pivot motor
		public static final int CORAL_INTAKE_MOTOR_ID = 61; // CAN ID for the intake motor
	
		// Motor Inversion
		public static final boolean CORAL_PIVOT_MOTOR_INVERTED = false; // Set to true if the pivot motor is inverted
		public static final boolean CORAL_INTAKE_MOTOR_INVERTED = true; // Set to true if the intake motor is inverted
	
		// Current Limits
		public static final int CORAL_PIVOT_CURRENT_LIMIT = 30; // Current limit for the pivot motor (in amps)
		public static final int CORAL_INTAKE_CURRENT_LIMIT = 20; // Current limit for the intake motor (in amps)
	
		// Encoder Conversion Factors
		public static final double CORAL_PIVOT_POSITION_CONVERSION = 360.0 / 2241.6; // Convert encoder ticks to degrees (based on encoder resolution)
		public static final double CORAL_PIVOT_VELOCITY_CONVERSION = 360.0 / 2241.6; // Convert encoder RPS to degrees per second (based on encoder resolution)
	
		// PID Constants
		public static final double CORAL_PIVOT_kP = 0.05; // Proportional gain for the pivot motor's PID controller
		public static final double CORAL_PIVOT_kI = 0.005; // Integral gain for the pivot motor's PID controller
		public static final double CORAL_PIVOT_kD = 0.02; // Derivative gain for the pivot motor's PID controller
		public static final double CORAL_PIVOT_kF = 0.35; // Feedforward gain for gravity compensation
		
		// Time Constants
		public static final double GAME_PIECE_DETECTION_DELAY = 0.5; // Delay in seconds after game piece detection before stopping intake
	
		// Pivot Angle Limits
		public static final double CORAL_PIVOT_MIN_ANGLE = -0.5; // Minimum allowed angle for the pivot (in degrees)
		public static final double CORAL_PIVOT_MAX_ANGLE = 160.0; // Maximum allowed angle for the pivot (in degrees)
	
		// Allowed Error
		public static final double CORAL_PIVOT_ALLOWED_ERROR = 0.5; // Allowed error threshold for the pivot to be "at target" (in degrees)
	
		// Manual Control Parameters
		public static final double CORAL_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual pivot control
		public static final double CORAL_MANUAL_SPEED_LIMIT = 0.1; // Speed limit for manual pivot control
	
		// Pivot Preset Angles
		public enum PivotPresetAngles {
			BASE(0.0),
			RAISE(90.0),
			ALGAE_INTAKE(148.5),
			ALGAE_SCORE(105.0);

			private final double angle;

			PivotPresetAngles(double angle) {
				this.angle = angle;
			}

			public double getAngle() {
				return angle;
			}
		}

		// Roller Speeds
		public static final double CORAL_INTAKE_SPEED = 0.3; // Speed for intaking coral (normalized output)
		public static final double CORAL_SCORE_SPEED = 0.3; // Speed for scoring coral (normalized output)
		public static final double ALGAE_INTAKE_SPEED = -0.1; // Speed for intaking algae (normalized output)
		public static final double ALGAE_SCORE_SPEED = 0.3; // Speed for scoring algae (normalized output)

		public static final double ROLLER_RUN_TIME = 1.0; // Time to run rollers during scoring (in seconds)
	
		// Through Bore Encoder
		public static final int THROUGH_BORE_DIO_PORT = 0; // DIO port for the Through Bore Encoder
		public static final double THROUGH_BORE_MIN_ANGLE = -0.5; // Minimum angle for the Through Bore Encoder (in degrees)
		public static final double THROUGH_BORE_MAX_ANGLE = 160.0; // Maximum angle for the Through Bore Encoder (in degrees)
		public static final double THROUGH_BORE_ALLOWED_DISCREPANCY = 2.0; // Allowed discrepancy between encoders (in degrees)
	
		// CANRange Sensor
		public static final int CANRANGE_SENSOR_ID = 64; // CAN ID for the CANRange sensor
		public static final double CANRANGE_DETECTION_THRESHOLD = 10.0; // Distance threshold for detecting a game piece (in mm)
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
		public static final double ALLOW_PIVOT_POSITION_CONVERSION = 1.0; // Convert encoder ticks to degrees (placeholder value)
		public static final double ALLOW_PIVOT_VELOCITY_CONVERSION = 1.0; // Convert encoder ticks to degrees per second (placeholder value)

		// PID Constants for Pivot Motor
		public static final double ALLOW_PIVOT_kP = 0.1; // Proportional gain for the pivot motor's PID controller
		public static final double ALLOW_PIVOT_kI = 0.0; // Integral gain for the pivot motor's PID controller
		public static final double ALLOW_PIVOT_kD = 0.0; // Derivative gain for the pivot motor's PID controller
		public static final double ALLOW_PIVOT_kF = 0.15; // Feedforward gain for gravity compensation

		// Pivot Angle Limits
		public static final double ALLOW_PIVOT_MIN_ANGLE = 0.0; // Minimum allowed angle for the pivot (in degrees)
		public static final double ALLOW_PIVOT_MAX_ANGLE = 90.0; // Maximum allowed angle for the pivot (in degrees)

		// Allowed Error
		public static final double ALLOW_PIVOT_ALLOWED_ERROR = 1.0; // Allowed error threshold for the pivot to be "at target" (in degrees)

		// Manual Control Parameters
		public static final double ALLOW_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual pivot control
		public static final double ALLOW_MANUAL_SPEED_LIMIT = 0.5; // Speed limit for manual pivot control

		// Pivot Preset Angles
		public enum PivotPresetAngles {
			BASE(0.0),
			INTAKE(45.0);

			private final double angle;

			PivotPresetAngles(double angle) {
				this.angle = angle;
			}

			public double getAngle() {
				return angle;
			}
		}
	}

	public static final class VisionConstants {
        // Limelight names
        public static final String[] LIMELIGHT_NAMES = {"funnel", "barge", "reef"};

        // AprilTag pipeline index
        public static final int APRILTAG_PIPELINE = 0;

        // Distance offsets for AprilTags (in meters)
        public static final double[][] APRILTAG_OFFSETS = {
            // Format: {horizontalOffset, verticalOffset}
            {0.5, 0.3}, // Tag 1: Horizontal offset = 0.5m, Vertical offset = 0.3m
            {-0.5, 0.3}, // Tag 2: Horizontal offset = -0.5m, Vertical offset = 0.3m
			{0.0, 0.0}, // Tag 3: Placeholder values
			{0.0, 0.0}, // Tag 4: Placeholder values
			{0.0, 0.0}, // Tag 5: Placeholder values
			{0.0, 0.0}, // Tag 6: Placeholder values
			{0.0, 0.0}, // Tag 7: Placeholder values
			{0.0, 0.0}, // Tag 8: Placeholder values
			{0.0, 0.0}, // Tag 9: Placeholder values
			{0.0, 0.0}, // Tag 10: Placeholder values
			{0.0, 0.0}, // Tag 11: Placeholder values
			{0.0, 0.0}, // Tag 12: Placeholder values
			{0.0, 0.0}, // Tag 13: Placeholder values
			{0.0, 0.0}, // Tag 14: Placeholder values
			{0.0, 0.0}, // Tag 15: Placeholder values
			{0.0, 0.0}, // Tag 16: Placeholder values
			{0.0, 0.0}, // Tag 17: Placeholder values
			{0.0, 0.0}, // Tag 18: Placeholder values
			{0.0, 0.0}, // Tag 19: Placeholder values
			{0.0, 0.0}, // Tag 20: Placeholder values
			{0.0, 0.0}, // Tag 21: Placeholder values
			{0.0, 0.0}, // Tag 22: Placeholder values
        };

        // Tracking gains
        public static final class TrackingGains {
			public static final double DISTANCE_kP = 0.1; // Proportional gain for distance control
			public static final double ROTATION_kP = 0.03; // Proportional gain for rotation control
			public static final double VELOCITY_DEADBAND = 0.05; // Deadband for velocity control (in m/s)
			public static final double ROTATION_DEADBAND = 0.02; // Deadband for rotation control (in radians)
			public static final double MAX_LINEAR_VELOCITY = 2.0; // Maximum linear velocity for vision tracking (in m/s)
			public static final double MAX_ANGULAR_VELOCITY = 1.0; // Maximum angular velocity for vision tracking (in rad/s)
		}
    }
}