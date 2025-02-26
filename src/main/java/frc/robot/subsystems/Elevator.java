package frc.robot.subsystems;

import java.util.Map;

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

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

	private final SparkMax leftMotor;
	private final SparkMax rightMotor;
	private final RelativeEncoder leftEncoder;
	private final RelativeEncoder rightEncoder;
	private final SparkClosedLoopController leftController;
	private final SparkClosedLoopController rightController;
	private final SparkMaxConfig leftConfig;
	private final SparkMaxConfig rightConfig;
	private final ShuffleboardTab elevatorTab;

	// Track target position internally instead of reading from Shuffleboard
	private double currentTargetPosition = 0.0;
	// Track manual mode state internally
	private boolean manualModeEnabled = false;

	public Elevator() {
		// Initialize motors
		leftMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
		rightMotor = new SparkMax(ElevatorConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);

		// Configure motors
		leftConfig = new SparkMaxConfig();
		rightConfig = new SparkMaxConfig();

		configureMotor(leftMotor, leftConfig, ElevatorConstants.ELEVATOR_LEFT_INVERTED);
		configureMotor(rightMotor, rightConfig, ElevatorConstants.ELEVATOR_RIGHT_INVERTED);

		// Get encoders and controllers
		leftEncoder = leftMotor.getEncoder();
		rightEncoder = rightMotor.getEncoder();
		leftController = leftMotor.getClosedLoopController();
		rightController = rightMotor.getClosedLoopController();

		// Initialize Shuffleboard tab and information displays
		elevatorTab = Shuffleboard.getTab("Elevator");
		
		configureShuffleboard();
	}

	private void configureMotor(SparkMax motor, SparkMaxConfig config, boolean isInverted) {
		config.inverted(isInverted)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);

		// Configure encoder conversion factors
		config.encoder.positionConversionFactor(ElevatorConstants.ELEVATOR_POSITION_CONVERSION)
			.velocityConversionFactor(ElevatorConstants.ELEVATOR_VELOCITY_CONVERSION);

		// Configure closed-loop control
		config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.p(ElevatorConstants.ELEVATOR_kP)
			.i(ElevatorConstants.ELEVATOR_kI)
			.d(ElevatorConstants.ELEVATOR_kD)
			.velocityFF(ElevatorConstants.ELEVATOR_kF)
			.outputRange(-1, 1);

		// Apply configuration
		motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void configureShuffleboard() {
		// Status Layout - Boolean indicators
		elevatorTab.addBoolean("At Target", this::isAtTargetPosition)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withSize(3, 2)
			.withPosition(0,0);
		
		elevatorTab.addBoolean("Within Bounds", () -> !isElevatorOutOfBounds())
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withSize(3, 2)
			.withPosition(0,2);
		
		elevatorTab.addBoolean("Manual Mode", () -> manualModeEnabled)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withSize(3, 2)
			.withPosition(0,4);
		
		// Position Layout - Height information
		elevatorTab.addNumber("Current Height", this::getCurrentPosition)
			.withWidget(BuiltInWidgets.kDial)
			.withProperties(Map.of(
				"Min", ElevatorConstants.ELEVATOR_MIN_POSITION,
				"Max", ElevatorConstants.ELEVATOR_MAX_POSITION))
			.withSize(3, 2)
			.withPosition(3,0);
		
		elevatorTab.addNumber("Target Height", () -> currentTargetPosition)
			.withSize(3, 2)
			.withPosition(3,2);
		
		elevatorTab.addNumber("Position Error", () -> currentTargetPosition - getCurrentPosition())
			.withSize(3, 2)
			.withPosition(3, 4);
		
		elevatorTab.addNumber("Left Encoder", () -> leftEncoder.getPosition())
			.withSize(3, 2)
			.withPosition(0, 6);
		
		elevatorTab.addNumber("Right Encoder", () -> rightEncoder.getPosition())
			.withSize(3, 2)
			.withPosition(3, 6);
			
		// Left Motor Layout - Current and output information
		elevatorTab.addNumber("Left Motor Current", leftMotor::getOutputCurrent)
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(5, 4)
			.withPosition(6, 0);
		
		elevatorTab.addNumber("Left Motor Output", leftMotor::getAppliedOutput)
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(5, 4)
			.withPosition(6, 4);

		// Right Motor Layout - Current and output information
		elevatorTab.addNumber("Right Motor Current", rightMotor::getOutputCurrent)
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(5, 4)
			.withPosition(11, 0);
		
		elevatorTab.addNumber("Right Motor Output", rightMotor::getAppliedOutput)
			.withWidget(BuiltInWidgets.kGraph)
			.withSize(5, 4)
			.withPosition(11, 4);
			
		// PID Layout - Control parameters
		elevatorTab.addNumber("P Gain", () -> ElevatorConstants.ELEVATOR_kP)
			.withSize(3, 2)
			.withPosition(16, 0);
			
		elevatorTab.addNumber("I Gain", () -> ElevatorConstants.ELEVATOR_kI)
			.withSize(3, 2)	
			.withPosition(16, 2);
		
		elevatorTab.addNumber("D Gain", () -> ElevatorConstants.ELEVATOR_kD)
			.withSize(3, 2)
			.withPosition(16, 4);
		
		elevatorTab.addNumber("F Gain", () -> ElevatorConstants.ELEVATOR_kF)
			.withSize(3, 2)
			.withPosition(16, 6);
	}

	public void setPosition(double targetPosition) {
		// Clamp target position within safe limits
		targetPosition = Math.min(Math.max(targetPosition, ElevatorConstants.ELEVATOR_MIN_POSITION),
			ElevatorConstants.ELEVATOR_MAX_POSITION);
		
		// Update internal target position tracking
		currentTargetPosition = targetPosition;

		System.out.println("Setting elevator position to: " + targetPosition); // Debug print

		// Calculate dynamic feed forward based on gravity compensation
		double ff = calculateFeedForward(targetPosition);

		// Create a ClosedLoopSlot object for slot 0
    	ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;

		leftController.setReference(targetPosition, ControlType.kPosition, slot, ff);
		rightController.setReference(targetPosition, ControlType.kPosition, slot, ff);
	}

	public void resetEncoders() {
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);
	}

	public void manualControl(double speed) {
		// Apply deadband and limits
		if (Math.abs(speed) < ElevatorConstants.ELEVATOR_MANUAL_CONTROL_DEADBAND) {
			speed = 0;
		}
		speed = Math.min(Math.max(speed * ElevatorConstants.ELEVATOR_MANUAL_SPEED_LIMIT, -1), 1);

		// Safety checks
		if (!isElevatorOutOfBounds() || 
			(getCurrentPosition() <= ElevatorConstants.ELEVATOR_MIN_POSITION && speed > 0) ||
			(getCurrentPosition() >= ElevatorConstants.ELEVATOR_MAX_POSITION && speed < 0)) {
			leftMotor.set(speed);
			rightMotor.set(speed);
		} else {
			stopElevator();
		}
	}

	public void stopElevator() {
		leftMotor.set(0);
		rightMotor.set(0);
	}

	private boolean isElevatorOutOfBounds() {
		double currentPosition = getCurrentPosition();
		return currentPosition < ElevatorConstants.ELEVATOR_MIN_POSITION || 
			currentPosition > ElevatorConstants.ELEVATOR_MAX_POSITION;
	}

	public double getCurrentPosition() {
		// Average both encoders to ensure synchronization
		return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
	}

	public boolean isAtTargetPosition() {
		double currentPosition = getCurrentPosition();
		return Math.abs(currentTargetPosition - currentPosition) <= ElevatorConstants.ELEVATOR_ALLOWED_ERROR;
	}

	private double calculateFeedForward(double targetPosition) {
		// Calculate feedforward based on the target position (e.g., gravity compensation)
		return ElevatorConstants.ELEVATOR_kF * Math.sin(Math.toRadians(targetPosition));
	}

	public void setManualMode(boolean enabled) {
		manualModeEnabled = enabled;
	}
	
	public boolean isInManualMode() {
		return manualModeEnabled;
	}

	@Override
	public void periodic() {
		// Safety checks
		performSafetyChecks();
	}
	
	// Performs all safety checks and takes appropriate actions
    private void performSafetyChecks() {
        // Check for out-of-bounds conditions
        if (isElevatorOutOfBounds() && !manualModeEnabled) {
            System.err.println("WARNING: Elevator out of bounds! Position: " + getCurrentPosition());
            stopElevator();
        }
        
        // Check for encoder synchronization
        if (Math.abs(leftEncoder.getPosition() - rightEncoder.getPosition()) > 
            ElevatorConstants.ELEVATOR_ALLOWED_ERROR) {
            System.err.println("WARNING: Elevator encoders out of sync!");
            System.err.println("Left encoder: " + leftEncoder.getPosition());
            System.err.println("Right encoder: " + rightEncoder.getPosition());
        }
    }
}