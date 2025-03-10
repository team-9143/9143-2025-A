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

    // Track target position internally
    private double currentTargetPosition = 0.0;
    // Track manual mode state internally
    private boolean manualModeEnabled = false;
    // Track position control mode internally
    private boolean positionControlEnabled = false;

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

        // Reset encoders on initialization
        resetEncoders();

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

        // Configure encoder conversion factors with corrected conversion factor between height and ticks
        // config.encoder.positionConversionFactor(1.0 / ElevatorConstants.ELEVATOR_POSITION_CONVERSION)
            // .velocityConversionFactor(1.0 / ElevatorConstants.ELEVATOR_VELOCITY_CONVERSION);

        // Configure closed-loop control with PID values
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ElevatorConstants.ELEVATOR_kP)
            .i(ElevatorConstants.ELEVATOR_kI)
            .d(ElevatorConstants.ELEVATOR_kD)
            .velocityFF(ElevatorConstants.ELEVATOR_kF)
            .outputRange(-1, 1);

        // Configure MAXMotion parameters for trajectory control
        config.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.ELEVATOR_MAX_VELOCITY)
            .maxAcceleration(ElevatorConstants.ELEVATOR_MAX_ACCELERATION)
            .allowedClosedLoopError(ElevatorConstants.ELEVATOR_ALLOWED_ERROR);

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
        
        elevatorTab.addBoolean("Position Control", () -> positionControlEnabled)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0,6);
        
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
        
        elevatorTab.addNumber("Encoders", () -> leftEncoder.getPosition())
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
        // Enable position control mode
        positionControlEnabled = true;

        System.out.println("Setting elevator position to: " + targetPosition); // Debug print

        // Initial send of position command
        updatePositionControl();
    }

    private void updatePositionControl() {
        if (positionControlEnabled) {
            // Use static feedforward value
            double ff = ElevatorConstants.ELEVATOR_kF;

            // Use MAXMotion for smoother position control
            ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;

            leftController.setReference(currentTargetPosition, ControlType.kMAXMotionPositionControl, slot, ff);
            rightController.setReference(currentTargetPosition, ControlType.kMAXMotionPositionControl, slot, ff);
        }
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        System.out.println("Elevator encoders reset to 0");
    }

    public void manualControl(double speed) {
        // Disable position control when in manual mode
        positionControlEnabled = false;
        
        // Apply deadband and limits
        if (Math.abs(speed) < ElevatorConstants.ELEVATOR_MANUAL_CONTROL_DEADBAND) {
            speed = 0;
        }
        
        speed = Math.min(Math.max(speed * ElevatorConstants.ELEVATOR_MANUAL_SPEED_LIMIT, -1), 1);

        // Use static feedforward value
        double ff = ElevatorConstants.ELEVATOR_kF;

        // Safety checks
        if (!isElevatorOutOfBounds() || 
            (getCurrentPosition() <= ElevatorConstants.ELEVATOR_MIN_POSITION && speed > 0) ||
            (getCurrentPosition() >= ElevatorConstants.ELEVATOR_MAX_POSITION && speed < 0)) {

            if (speed == 0) {
                leftMotor.set(ff);
                rightMotor.set(ff);
            } else {
                leftMotor.set(speed);
                rightMotor.set(speed);
            }
        } else {
            System.err.println("WARNING: Elevator out of bounds! Position: " + getCurrentPosition());
            stopElevator();
        }
    }

    public void stopElevator() {
        if (!positionControlEnabled) {
            leftMotor.set(0);
            rightMotor.set(0);
        }
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

    public void setManualMode(boolean enabled) {
        manualModeEnabled = enabled;
        // Disable position control when entering manual mode
        if (enabled) {
            positionControlEnabled = false;
        }
    }
    
    public boolean isInManualMode() {
        return manualModeEnabled;
    }
    
    // Performs all safety checks and takes appropriate actions
    private void performSafetyChecks() {
        // Check for out-of-bounds conditions
        if (isElevatorOutOfBounds() && !manualModeEnabled) {
            System.err.println("WARNING: Elevator out of bounds! Position: " + getCurrentPosition());
            stopElevator();
            positionControlEnabled = false;
        }
        
        // Check for encoder synchronization
        if (Math.abs(leftEncoder.getPosition() - rightEncoder.getPosition()) > 
            ElevatorConstants.ELEVATOR_ALLOWED_ERROR) {
            System.err.println("WARNING: Elevator encoders out of sync!");
            System.err.println("Left encoder: " + leftEncoder.getPosition());
            System.err.println("Right encoder: " + rightEncoder.getPosition());
        }
    }

    @Override
    public void periodic() {
        // Continue executing position control if enabled
        if (positionControlEnabled) {
            updatePositionControl();
        }
        
        // Safety checks
        performSafetyChecks();
    }
}