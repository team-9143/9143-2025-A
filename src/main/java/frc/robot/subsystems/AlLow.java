/*
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import frc.robot.Constants.AlLowConstants;

public class AlLow extends SubsystemBase {
	private final SparkMax pivotMotor;
	private final SparkMax rollerMotor;
	private final RelativeEncoder pivotEncoder;
	private final SparkClosedLoopController pivotController;
	
	// Shuffleboard elements
	private final ShuffleboardTab allowTab;
	
	private double targetAngle = 0;
	// Track manual mode state internally
	private boolean manualModeEnabled = false;

	public AlLow() {
		// Initialize motors
		pivotMotor = new SparkMax(AlLowConstants.ALLOW_PIVOT_MOTOR_ID, MotorType.kBrushless);
		rollerMotor = new SparkMax(AlLowConstants.ALLOW_ROLLER_MOTOR_ID, MotorType.kBrushless);

		// Configure motors
		SparkMaxConfig pivotConfig = new SparkMaxConfig();
		SparkMaxConfig rollerConfig = new SparkMaxConfig();
		
		configureMotor(pivotMotor, pivotConfig, AlLowConstants.ALLOW_PIVOT_MOTOR_INVERTED, true);
		configureMotor(rollerMotor, rollerConfig, AlLowConstants.ALLOW_ROLLER_MOTOR_INVERTED, false);

		// Get encoder and controller
		pivotEncoder = pivotMotor.getEncoder();
		pivotController = pivotMotor.getClosedLoopController();

		// Initialize Shuffleboard tab and layouts
		allowTab = Shuffleboard.getTab("AlLow");

		// Configure Shuffleboard
		configureShuffleboard();
	}

	private void configureMotor(SparkMax motor, SparkMaxConfig config, boolean isInverted, boolean isPivot) {
		config.inverted(isInverted)
			  .idleMode(IdleMode.kBrake)
			  .smartCurrentLimit(isPivot ? AlLowConstants.ALLOW_PIVOT_CURRENT_LIMIT : AlLowConstants.ALLOW_ROLLER_CURRENT_LIMIT);

		if (isPivot) {
			// Configure encoder conversion factors
			config.encoder.positionConversionFactor(AlLowConstants.ALLOW_PIVOT_POSITION_CONVERSION)
				.velocityConversionFactor(AlLowConstants.ALLOW_PIVOT_VELOCITY_CONVERSION);

			// Configure closed-loop control with feedforward
			config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(AlLowConstants.ALLOW_PIVOT_kP)
				.i(AlLowConstants.ALLOW_PIVOT_kI)
				.d(AlLowConstants.ALLOW_PIVOT_kD)
				.velocityFF(AlLowConstants.ALLOW_PIVOT_kF) // Add feedforward gain
				.outputRange(-1, 1);
		}

		// Apply configuration
		motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void configureShuffleboard() {
		// Status Layout - Boolean indicators
		allowTab.addBoolean("At Target Angle", this::isAtTargetAngle)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withSize(3, 3)
			.withPosition(0, 0);
		
		allowTab.addBoolean("Within Bounds", () -> !isPivotOutOfBounds())
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withSize(3, 3)
			.withPosition(0, 3);
		
		allowTab.addBoolean("Manual Mode", this::isInManualMode)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withSize(3, 2)
			.withPosition(0, 6);
		
		// Position Layout - Angle information
		allowTab.addNumber("Current Angle", this::getPivotAngle)
			.withWidget(BuiltInWidgets.kDial)
			.withProperties(Map.of(
				"Min", AlLowConstants.ALLOW_PIVOT_MIN_ANGLE,
				"Max", AlLowConstants.ALLOW_PIVOT_MAX_ANGLE))
			.withSize(3, 3)
			.withPosition(3, 0);
		
		allowTab.addNumber("Target Angle", () -> targetAngle)
			.withSize(3, 1)
			.withPosition(3, 3);
		
		allowTab.addNumber("Current Angle", () -> getPivotAngle())
			.withSize(3, 1)
			.withPosition(3, 4);
		
		allowTab.addNumber("Angle Error", () -> targetAngle - getPivotAngle())
			.withSize(3, 1)
			.withPosition(3, 5);
		
		allowTab.addNumber("Pivot Encoder", pivotEncoder::getPosition)
			.withSize(3, 1)
			.withPosition(3, 6);
		
		// Pivot Motor Layout - Current and output information
		allowTab.addNumber("Pivot Current", pivotMotor::getOutputCurrent)
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(5, 4)
			.withPosition(6, 0);
		
		allowTab.addNumber("Pivot Output", pivotMotor::getAppliedOutput)
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(5, 4)
			.withPosition(6, 4);
		
		// Roller Motor Layout - Current and output information
		allowTab.addNumber("Roller Current", rollerMotor::getOutputCurrent)
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(5, 4)
			.withPosition(11, 0);
		
		allowTab.addNumber("Roller Output", rollerMotor::getAppliedOutput)
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(5, 4)
			.withPosition(11, 4);
		
		// PID Layout - Control parameters
		allowTab.addNumber("P Gain", () -> AlLowConstants.ALLOW_PIVOT_kP)
			.withSize(2, 2)
			.withPosition(17, 0);
			
		allowTab.addNumber("I Gain", () -> AlLowConstants.ALLOW_PIVOT_kI)
			.withSize(2, 2)
			.withPosition(17, 2);
		
		allowTab.addNumber("D Gain", () -> AlLowConstants.ALLOW_PIVOT_kD)
			.withSize(2, 2)
			.withPosition(17, 4);
		
		// Update to show feedforward gain
		allowTab.addNumber("F Gain", () -> AlLowConstants.ALLOW_PIVOT_kF)
			.withSize(2, 2)
			.withPosition(17, 6);
		
	}

	@Override
	public void periodic() {
		// Only used for telemetry updates, not control
		// All control is handled through explicit command calls
		
		// Add safety checks to match other subsystems
		performSafetyChecks();
	}

	public void setPivotAngle(double angle) {
		// Clamp target angle
		targetAngle = Math.min(Math.max(angle, AlLowConstants.ALLOW_PIVOT_MIN_ANGLE),
			AlLowConstants.ALLOW_PIVOT_MAX_ANGLE);
		
		// Disable manual mode when using position control
		setManualMode(false);
		
		// Calculate dynamic feed forward based on gravity compensation
		double ff = calculateFeedForward(targetAngle);
		
		// Create a ClosedLoopSlot object for slot 0
		ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;
		
		// Use the feed forward with position control
		pivotController.setReference(targetAngle, ControlType.kPosition, slot, ff);
		
		System.out.println("Setting pivot angle to: " + targetAngle + " with FF: " + ff); // Debug print
	}

	private double calculateFeedForward(double targetAngle) {
		// Calculate feedforward based on the target angle (gravity compensation)
		// This uses a sine function to apply more force when the arm is horizontal
		// and less when vertical, similar to the elevator implementation
		return AlLowConstants.ALLOW_PIVOT_kF * Math.sin(Math.toRadians(targetAngle));
	}

	public void resetPivotEncoder() {
		pivotEncoder.setPosition(0);
	}

	public void setRollerSpeed(double speed) {
		rollerMotor.set(speed);
	}

	public void stopRoller() {
		setRollerSpeed(0);
	}

	public void manualPivotControl(double speed) {
		// Enable manual mode tracking
		setManualMode(true);
		
		// Apply deadband and limits
		if (Math.abs(speed) < AlLowConstants.ALLOW_MANUAL_CONTROL_DEADBAND) {
			speed = 0;
		}
		speed = Math.min(Math.max(speed * AlLowConstants.ALLOW_MANUAL_SPEED_LIMIT, -1), 1);

		// Only allow movement if within bounds or moving toward valid range
		if (!isPivotOutOfBounds() || 
			(getPivotAngle() <= AlLowConstants.ALLOW_PIVOT_MIN_ANGLE && speed > 0) ||
			(getPivotAngle() >= AlLowConstants.ALLOW_PIVOT_MAX_ANGLE && speed < 0)) {
			pivotMotor.set(speed);
		} else {
			stopPivot();
		}
	}

	public void stopPivot() {
		pivotMotor.set(0);
	}

	private boolean isPivotOutOfBounds() {
		double currentAngle = getPivotAngle();
		return currentAngle < AlLowConstants.ALLOW_PIVOT_MIN_ANGLE || 
			currentAngle > AlLowConstants.ALLOW_PIVOT_MAX_ANGLE;
	}

	public double getPivotAngle() {
		return pivotEncoder.getPosition();
	}

	public boolean isAtTargetAngle() {
		double currentAngle = getPivotAngle();
		return Math.abs(targetAngle - currentAngle) <= AlLowConstants.ALLOW_PIVOT_ALLOWED_ERROR;
	}
	
	// Manual mode tracking methods
	public void setManualMode(boolean enabled) {
		manualModeEnabled = enabled;
	}
	
	public boolean isInManualMode() {
		return manualModeEnabled;
	}
	
	// Safety check method 
	private void performSafetyChecks() {
		// Check for out-of-bounds conditions
		if (isPivotOutOfBounds() && !manualModeEnabled) {
			System.err.println("WARNING: Pivot out of bounds! Angle: " + getPivotAngle());
			stopPivot();
		}
		
		// Check for excessive current or other safety conditions
		if (pivotMotor.getOutputCurrent() > AlLowConstants.ALLOW_PIVOT_CURRENT_LIMIT * 1.1) {
			System.err.println("WARNING: Pivot motor current exceeded limit! Current: " + 
								pivotMotor.getOutputCurrent());
		}
		
		if (rollerMotor.getOutputCurrent() > AlLowConstants.ALLOW_ROLLER_CURRENT_LIMIT * 1.1) {
			System.err.println("WARNING: Roller motor current exceeded limit! Current: " + 
								rollerMotor.getOutputCurrent());
		}
	}
}
*/