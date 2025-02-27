package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	// Blue alliance sees forward as 0 degrees (toward red alliance wall)
	private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
	// Red alliance sees forward as 180 degrees (toward blue alliance wall)
	private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
	// Keep track if we've ever applied the operator perspective before or not
	private boolean m_hasAppliedOperatorPerspective = false;

	// Swerve request to apply during robot-centric path following
	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

	// Swerve requests to apply during SysId characterization
	private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
	private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
	private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

	// Vision subsystem for AprilTag tracking
	private Vision vision;
	
	// Track vision tracking state internally
	private boolean isVisionTrackingEnabled = false;
	
	public Command aprilTagTrackingCommand;

	// Shuffleboard tab and layouts
	private final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

	// SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
	private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,   		// Use default ramp rate (1 V/s)
			Volts.of(4),	// Reduce dynamic step voltage to 4 V to prevent brownout
			null,   		// Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			output -> setControl(m_translationCharacterization.withVolts(output)),
			null,
			this
		)
	);

	// SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
	private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,			// Use default ramp rate (1 V/s)
			Volts.of(7),	// Use dynamic voltage of 7 V
			null,			// Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			volts -> setControl(m_steerCharacterization.withVolts(volts)),
			null,
			this
		)
	);

	/*
		* SysId routine for characterizing rotation.
		* This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
		* See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
		*/
	private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
		new SysIdRoutine.Config(
			// This is in radians per secondÂ², but SysId only supports "volts per second"
			Volts.of(Math.PI / 6).per(Second),
			// This is in radians per second, but SysId only supports "volts"
			Volts.of(Math.PI),
			null,	// Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			output -> {
				/* output is actually radians per second, but SysId only supports "volts" */
				setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
				/* also log the requested output for SysId */
				SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
			},
			null,
			this
		)
	);

	// The SysId routine to test
	private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 *
	 * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
	 * @param modules               Constants for each specific module
	 */
	public Swerve(
		SwerveDrivetrainConstants drivetrainConstants,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, modules);

		this.vision = new Vision(this);

		configureShuffleboardData();
		
		if (Utils.isSimulation()) {
			startSimThread();
		}

		aprilTagTrackingCommand = createAprilTagTrackingCommand();

		configureAutoBuilder();
	}

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 *
	 * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
	 * @param odometryUpdateFrequency The frequency to run the odometry loop
	 * @param modules                 Constants for each specific module
	 */
	public Swerve(
		SwerveDrivetrainConstants drivetrainConstants,
		double odometryUpdateFrequency,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, odometryUpdateFrequency, modules);

		this.vision = new Vision(this);
			
		configureShuffleboardData();

		if (Utils.isSimulation()) {
			startSimThread();
		}

		aprilTagTrackingCommand = createAprilTagTrackingCommand();

		configureAutoBuilder();
	}

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 *
	 * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
	 * @param odometryUpdateFrequency   The frequency to run the odometry loop
	 * @param odometryStandardDeviation The standard deviation for odometry calculation
	 * @param visionStandardDeviation   The standard deviation for vision calculation
	 * @param modules                   Constants for each specific module
	 */
	public Swerve(
		SwerveDrivetrainConstants drivetrainConstants,
		double odometryUpdateFrequency,
		Matrix<N3, N1> odometryStandardDeviation,
		Matrix<N3, N1> visionStandardDeviation,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

		this.vision = new Vision(this);
			
		configureShuffleboardData();

		if (Utils.isSimulation()) {
			startSimThread();
		}

		aprilTagTrackingCommand = createAprilTagTrackingCommand();

		configureAutoBuilder();
	}

	private void configureAutoBuilder() {
		try {
			var config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
				() -> getState().Pose,      // Supplier of current robot pose
				this::resetPose,            // Consumer for seeding pose against auto
				() -> getState().Speeds,    // Supplier of current robot speeds
				// Consumer of ChassisSpeeds and feedforwards to drive the robot
				(speeds, feedforwards) -> setControl(
					m_pathApplyRobotSpeeds.withSpeeds(speeds)
						.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
						.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
				),
				new PPHolonomicDriveController(
					// PID constants for translation
					new PIDConstants(0.8, 0.005, 0.1),
					// PID constants for rotation
					new PIDConstants(0.6, 0.005, 0.0)
				),
				config,
				// Assume the path needs to be flipped for Red vs Blue, this is normally the case
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
				this
			);
		} catch (Exception ex) {
			DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
		}
	}

	private void configureShuffleboardData() {
		// Robot Pose Layout
		swerveTab.addNumber("X Position", () -> getState().Pose.getX())
			.withWidget(BuiltInWidgets.kTextView)
			.withSize(3, 2)
			.withPosition(0, 0);
		
		swerveTab.addNumber("Y Position", () -> getState().Pose.getY())
			.withWidget(BuiltInWidgets.kTextView)
			.withSize(3, 2)
			.withPosition(0, 2);
		
		swerveTab.addNumber("Heading (deg)", () -> getState().Pose.getRotation().getDegrees())
			.withWidget(BuiltInWidgets.kTextView)
			.withSize(3, 2)
			.withPosition(0, 4);
		
		// Robot Speeds Layout
		swerveTab.addNumber("Speed (m/s)", () -> 
			Math.hypot(getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond))
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(3, 2)
			.withPosition(3, 0);
			
		swerveTab.addNumber("Vx (m/s)", () -> getState().Speeds.vxMetersPerSecond)
			.withWidget(BuiltInWidgets.kTextView)
			.withSize(3, 2)
			.withPosition(3, 2);
			
		swerveTab.addNumber("Vy (m/s)", () -> getState().Speeds.vyMetersPerSecond)
			.withWidget(BuiltInWidgets.kTextView)
			.withSize(3, 2)
			.withPosition(3, 4);
			
		swerveTab.addNumber("Omega (rad/s)", () -> getState().Speeds.omegaRadiansPerSecond)
			.withWidget(BuiltInWidgets.kTextView)
			.withSize(3, 2)
			.withPosition(3, 6);
		
		// Module States Layout - Add entries for each module
		for (int i = 0; i < 4; i++) {
			final int moduleIndex = i;
			String moduleName = "Module " + (i + 1);
			
			swerveTab.addNumber(moduleName + " Angle", () -> getState().ModuleStates[moduleIndex].angle.getDegrees())
				.withSize(3, 2)
				.withPosition(6, i * 2);
				
			swerveTab.addNumber(moduleName + " Speed", () -> getState().ModuleStates[moduleIndex].speedMetersPerSecond)
				.withSize(3, 2)
				.withPosition(9, i * 2);
		}
	}

	/**
	 * Returns a command that applies the specified control request to this swerve drivetrain.
	 *
	 * @param request Function returning the request to apply
	 * @return Command to run
	 */
	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	/**
	 * Runs the SysId Quasistatic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Quasistatic test
	 * @return Command to run
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutineToApply.quasistatic(direction);
	}

	/**
	 * Runs the SysId Dynamic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Dynamic test
	 * @return Command to run
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutineToApply.dynamic(direction);
	}

	/**
	 * Set the currently active SysId routine to use.
	 * 
	 * @param routineType The type of SysId routine to use
	 */
	public void setSysIdRoutine(SysIdRoutineType routineType) {
		switch (routineType) {
			case TRANSLATION:
				m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
				break;
			case STEER:
				m_sysIdRoutineToApply = m_sysIdRoutineSteer;
				break;
			case ROTATION:
				m_sysIdRoutineToApply = m_sysIdRoutineRotation;
				break;
		}
	}
	
	/**
	 * Enum for selecting which SysId routine to use
	 */
	public enum SysIdRoutineType {
		TRANSLATION,
		STEER,
		ROTATION
	}

	/**
	 * Enable or disable vision tracking
	 * 
	 * @param enabled Whether vision tracking should be enabled
	 */
	public void setVisionTrackingEnabled(boolean enabled) {
		isVisionTrackingEnabled = enabled;
		vision.toggleTracking(enabled);
	}
	
	/**
	 * Get whether vision tracking is currently enabled
	 * 
	 * @return Whether vision tracking is enabled
	 */
	public boolean isVisionTrackingEnabled() {
		return isVisionTrackingEnabled;
	}

	// Creates a command to track the best AprilTag using vision.
	public Command createAprilTagTrackingCommand() {
		return run(() -> {
			if (isVisionTrackingEnabled) {
				Optional<Vision.AprilTagTarget> target = vision.getBestTarget();
				if (target.isPresent()) {
					// Validate tag ID is within bounds
					int tagId = target.get().id;
					if (tagId < 0 || tagId >= VisionConstants.APRILTAG_OFFSETS.length) {
						System.out.println("Warning: AprilTag ID " + tagId + " out of bounds");
						return;
					}
					
					// Retrieve target offsets from Constants
					double targetHorizontalOffset = vision.getTargetHorizontalOffset(tagId);
					double targetVerticalOffset = vision.getTargetVerticalOffset(tagId);

					// Calculate distance directly from pose components
					double currentDistance = Math.hypot(target.get().poseX, target.get().poseY);
					
					// Calculate errors
					double horizontalError = target.get().poseY - targetHorizontalOffset;
					double distanceError = currentDistance - targetVerticalOffset;
					double angleError = target.get().tx;

					// Apply deadbands
					if (Math.abs(horizontalError) < VisionConstants.TrackingGains.VELOCITY_DEADBAND) horizontalError = 0;
					if (Math.abs(distanceError) < VisionConstants.TrackingGains.VELOCITY_DEADBAND) distanceError = 0;
					if (Math.abs(angleError) < VisionConstants.TrackingGains.ROTATION_DEADBAND) angleError = 0;

					// Convert errors to robot-relative velocities
					double vx = -distanceError * VisionConstants.TrackingGains.DISTANCE_kP; // Forward/backward movement
					double vy = -horizontalError * VisionConstants.TrackingGains.DISTANCE_kP; // Left/right movement
					double omega = -angleError * VisionConstants.TrackingGains.ROTATION_kP; // Rotational movement

					// Clamp velocities to maximum values
					vx = Math.min(Math.max(vx, -VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY), VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY);
					vy = Math.min(Math.max(vy, -VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY), VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY);
					omega = Math.min(Math.max(omega, -VisionConstants.TrackingGains.MAX_ANGULAR_VELOCITY), VisionConstants.TrackingGains.MAX_ANGULAR_VELOCITY);

					// Apply velocities using robot-centric control
					setControl(new SwerveRequest.RobotCentric()
						.withVelocityX(vx) // Forward/backward movement
						.withVelocityY(vy) // Left/right movement
						.withRotationalRate(omega)); // Rotation to align with the tag
				}
			}
		});
	}

	public void addVisionMeasurement(
		Pose2d visionPose, 
		double timestampSeconds, 
		edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> stdDevs) {
		// Check if the robot is rotating too quickly
		boolean rotatingTooFast = Math.abs(getState().Speeds.omegaRadiansPerSecond) > 2.0;
		
		// Only add vision measurement if we're not rotating too quickly
		if (!rotatingTooFast) {
			// Call the parent method to add vision measurement with standard deviation
			super.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
		}
	}

	public void setVision(Vision vision) {
		this.vision = vision;
	}

	@Override
	public void periodic() {
		// Apply operator perspective if not already applied
		if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {
				setOperatorPerspectiveForward(
					allianceColor == Alliance.Red
						? kRedAlliancePerspectiveRotation
						: kBlueAlliancePerspectiveRotation
				);
				m_hasAppliedOperatorPerspective = true;
			});
		}
	}

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		// Run simulation at a faster rate so PID gains behave more reasonably
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			// use the measured time delta, get battery voltage from WPILib
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}
}