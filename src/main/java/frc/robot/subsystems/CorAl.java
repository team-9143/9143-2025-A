package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CorAlConstants;

/**
 * The CorAl (Coral and Algae) subsystem class controls the robot's game piece manipulation mechanism.
 * It includes a pivot mechanism and an intake system for handling coral and algae game pieces.
 * 
 * Features:
 * - Dual motor control for pivot and intake mechanisms
 * - Position control using both motor encoder and through bore encoder feedback
 * - Game piece detection using a CANRange sensor
 * - Comprehensive safety checks and soft limits
 * - Advanced telemetry via Shuffleboard
 */
public class CorAl extends SubsystemBase {
    // Hardware Components
    private final TalonFX pivotMotor;             // Motor controlling the pivot mechanism
    private final TalonFX intakeMotor;            // Motor controlling the intake rollers
    private final DigitalInput throughBoreInput;  // Input for the through bore encoder
    private final DutyCycle throughBore;          // Duty cycle reader for the through bore encoder
    private final CANrange canRangeSensor;        // Distance sensor for game piece detection
    
    // Shuffleboard Display
    private final ShuffleboardTab coralTab;       // Tab for CorAl subsystem telemetry

    // Motor Control Objects
    private final PositionVoltage positionRequest; // Position control request for the pivot motor
    private final DutyCycleOut percentRequest;     // Percent output control request for motors

    // Game Piece Detection Variables
    private boolean possibleGamePieceDetected = false;  // Indicates a possible game piece detection
    private final Timer gameDetectionTimer = new Timer(); // Timer for confirming game piece detection
    private boolean gameDetectionTimerRunning = false;   // Flag indicating if timer is running
    private boolean gamePieceDetected = false;           // Confirmed game piece detection status
    
    // Position Tracking
    private double currentTargetAngle = 0;        // Current target angle for position control
    
    // Encoder Calibration Variables
    private double throughBoreOffset = 0;         // Offset for the through bore encoder reading
    private double previousRawThroughBoreAngle = 0; // Previous raw angle for unwrapping
    private double throughBoreUnwrapOffset = 0;   // Unwrap offset for handling rotation beyond 360Â°

    // Sensor Calibration
    private double canRangeBaseline = 0;          // Baseline reading for the CANRange sensor

    // Control Mode
    private boolean positionControlActive = false; // Flag indicating if position control is active

    /**
     * Constructor for the CorAl subsystem.
     * Initializes all hardware components, calibrates sensors, and sets up Shuffleboard displays.
     */
    public CorAl() {
        // Initialize motors with specified CAN IDs
        pivotMotor = new TalonFX(CorAlConstants.CORAL_PIVOT_MOTOR_ID);
        intakeMotor = new TalonFX(CorAlConstants.CORAL_INTAKE_MOTOR_ID);

        // Initialize REV Through Bore Encoder on specified DIO port
        throughBoreInput = new DigitalInput(CorAlConstants.THROUGH_BORE_DIO_PORT);
        throughBore = new DutyCycle(throughBoreInput);

        // Initialize CANRange sensor for game piece detection
        canRangeSensor = new CANrange(CorAlConstants.CANRANGE_SENSOR_ID);

        // Calibrate the CANRange baseline (reading when nothing is detected)
        canRangeBaseline = canRangeSensor.getDistance().getValueAsDouble();
        System.out.println("CANRange baseline calibrated to: " + canRangeBaseline);

        // Initialize control request objects for motors
        positionRequest = new PositionVoltage(0).withSlot(0);
        percentRequest = new DutyCycleOut(0);

        // Configure motors with appropriate settings
        configurePivotMotor(pivotMotor);
        configureIntakeMotor(intakeMotor);

        // Initialize Shuffleboard tab for telemetry
        coralTab = Shuffleboard.getTab("CorAl");

        // Configure Shuffleboard displays with telemetry data
        configureShuffleboard();

        // Initialize game piece detection timer
        gameDetectionTimer.reset();
        
        // Zero encoders to calibrate the system
        zeroEncoders();
    }

    /**
     * Zeros both the motor encoder and through bore encoder to establish a reference point.
     * This calibrates the system to use the current position as the zero position.
     */
    public void zeroEncoders() {
        // Calculate the through bore offset to make current position zero
        throughBoreOffset = getRawThroughBoreAngle();
        
        // Reset the motor encoder to match the through bore reading
        pivotMotor.setPosition(0);
        
        System.out.println("Encoders zeroed: Through bore offset set to " + throughBoreOffset + ".");
    }

    /**
     * Configures the pivot motor with appropriate settings.
     * 
     * @param motor The TalonFX motor to configure
     */
    private void configurePivotMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure motor direction and brake mode
        config.MotorOutput.Inverted = CorAlConstants.CORAL_PIVOT_MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure current limits for motor protection
        config.CurrentLimits.SupplyCurrentLimit = CorAlConstants.CORAL_PIVOT_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configure encoder and position conversion
        config.Feedback.SensorToMechanismRatio = CorAlConstants.CORAL_PIVOT_POSITION_CONVERSION;
        
        // Configure PID constants for position control
        config.Slot0.kP = CorAlConstants.CORAL_PIVOT_kP;
        config.Slot0.kI = CorAlConstants.CORAL_PIVOT_kI;
        config.Slot0.kD = CorAlConstants.CORAL_PIVOT_kD;
        
        // Use built-in gravity compensation for consistent performance
        config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = CorAlConstants.CORAL_PIVOT_kF;

        // Apply configuration to the motor
        motor.getConfigurator().apply(config);
    }

    /**
     * Configures the intake motor with appropriate settings.
     * 
     * @param motor The TalonFX motor to configure
     */
    private void configureIntakeMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure motor direction and brake mode
        config.MotorOutput.Inverted = CorAlConstants.CORAL_INTAKE_MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure current limits for motor protection
        config.CurrentLimits.SupplyCurrentLimit = CorAlConstants.CORAL_INTAKE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply configuration to the motor
        motor.getConfigurator().apply(config);
    }

    /**
     * Configures the Shuffleboard tab with telemetry data and controls.
     * Organizes information into logical groups for monitoring subsystem status.
     */
    private void configureShuffleboard() {
        // Status Layout - Boolean indicators for system state
        coralTab.addBoolean("At Target", this::isAtTargetAngle)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0,0);
            
        coralTab.addBoolean("Game Piece Detected", this::isGamePieceDetected)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0,2);
            
        coralTab.addBoolean("Feedback Valid", this::isMotorFeedbackValid)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0,4);
           
        coralTab.addBoolean("Through Bore Connected", this::isThroughBoreConnected)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0,6);
            
        // Position & Sensor Layout - Angle and CANrange information
        coralTab.addNumber("Through Bore Angle", this::getThroughBoreAngle)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", CorAlConstants.CORAL_PIVOT_MIN_ANGLE, 
                "max", CorAlConstants.CORAL_PIVOT_MAX_ANGLE))
            .withSize(3, 2)
            .withPosition(3,0);

        coralTab.addNumber("Target Angle", () -> currentTargetAngle)
            .withSize(3, 2)
            .withPosition(3,2);
        
        coralTab.addNumber("Angle Error", () -> currentTargetAngle - getPivotAngle())
            .withSize(3, 2)
            .withPosition(3,4);

        coralTab.addNumber("CANRange Distance", this::getCANRangeDistance)
            .withSize(3, 2)
            .withPosition(3,6);
            
        // Motor Layout - Current and output information for diagnostics
        coralTab.addNumber("Pivot Current", () -> pivotMotor.getSupplyCurrent().getValueAsDouble())
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 4)
            .withPosition(6,0);

        coralTab.addNumber("Pivot Output", () -> pivotMotor.getDutyCycle().getValueAsDouble())
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 4)
            .withPosition(6,4);

        coralTab.addNumber("Intake Current", () -> intakeMotor.getSupplyCurrent().getValueAsDouble())
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 4)
            .withPosition(11,0);
            
        coralTab.addNumber("Intake Output", () -> intakeMotor.getDutyCycle().getValueAsDouble())
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 4)
            .withPosition(11,4);
            
        // PID Layout - Control parameters for tuning
        coralTab.addNumber("P Gain", () -> CorAlConstants.CORAL_PIVOT_kP)
            .withSize(3, 2)
            .withPosition(16, 0);
            
        coralTab.addNumber("I Gain", () -> CorAlConstants.CORAL_PIVOT_kI)
            .withSize(3, 2)
            .withPosition(16, 2);
        
        coralTab.addNumber("D Gain", () -> CorAlConstants.CORAL_PIVOT_kD)
            .withSize(3, 2)
            .withPosition(16, 4);
        
        coralTab.addNumber("F Gain", () -> CorAlConstants.CORAL_PIVOT_kF)
            .withSize(3, 2)
            .withPosition(16, 6);
    }

    /**
     * Sets the pivot mechanism to a specified angle using position control.
     * Uses either through bore encoder or motor encoder for feedback.
     * 
     * @param targetAngle The target angle in degrees
     */
    public void setPivotAngle(double targetAngle) {
        // Enable position control mode
        positionControlActive = true;

        // Constrain the target angle to valid range to prevent mechanical damage
        targetAngle = Math.min(Math.max(targetAngle, CorAlConstants.CORAL_PIVOT_MIN_ANGLE), 
                             CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
        currentTargetAngle = targetAngle;

        // Check if the through bore encoder is connected before setting position
        if (isThroughBoreConnected()) {
            // Set position based on the through bore reading for better accuracy
            // Need to convert the target angle to motor position units
            double currentThroughBoreAngle = getThroughBoreAngle();
            double currentMotorPosition = pivotMotor.getPosition().getValueAsDouble();
            double positionError = targetAngle - currentThroughBoreAngle;
            
            // Update motor position setpoint based on the through bore reading
            pivotMotor.setControl(positionRequest.withPosition(currentMotorPosition + positionError));
        } else {
            // If through bore is unavailable, use motor encoder as fallback
            pivotMotor.setControl(positionRequest.withPosition(targetAngle));
            System.out.println("WARNING: Using motor encoder as fallback; through bore disconnected.");
        }
    }

    /**
     * Resets the pivot encoder to calibrate the system.
     * Calls zeroEncoders() and logs the action.
     */
    public void resetPivotEncoder() {
        zeroEncoders();
        System.out.println("Pivot encoder reset.");
    }

    /**
     * Sets the intake roller speed.
     * 
     * @param speed The speed value (-1.0 to 1.0)
     */
    public void setIntakeSpeed(double speed) {
        intakeMotor.setControl(percentRequest.withOutput(speed));
    }

    /**
     * Stops the intake roller motor.
     */
    public void stopIntake() {
        setIntakeSpeed(0);
    }

    /**
     * Provides manual control of the pivot motor.
     * Does not interrupt active position control.
     * 
     * @param speed The manual control speed (-1.0 to 1.0)
     */
    public void manualPivotControl(double speed) {
        // Do not interrupt position control with manual input
        if (positionControlActive) {
            return;
        }
        pivotMotor.setControl(percentRequest.withOutput(speed));
    }

    /**
     * Disables position control mode, allowing manual control.
     */
    public void disablePositionControl() {
        positionControlActive = false;
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivot() {
        pivotMotor.setControl(percentRequest.withOutput(0));
    }

    /**
     * Gets the raw angle reading from the through bore encoder without offset.
     * 
     * @return The raw angle in degrees
     */
    public double getRawThroughBoreAngle() {
        double dutyCycle = throughBore.getOutput();
        return dutyCycle * (CorAlConstants.THROUGH_BORE_MAX_ANGLE - CorAlConstants.THROUGH_BORE_MIN_ANGLE) 
               + CorAlConstants.THROUGH_BORE_MIN_ANGLE;
    }
    
    /**
     * Gets the calibrated angle from the through bore encoder with offset applied.
     * Handles continuous rotation by unwrapping angles beyond 360 degrees.
     * 
     * @return The calibrated angle in degrees
     */
    public double getThroughBoreAngle() {
        if (!isThroughBoreConnected()) {
            // Fall back to motor position if through bore is disconnected
            return pivotMotor.getPosition().getValueAsDouble();
        }
        
        // Get raw angle
        double rawAngle = getRawThroughBoreAngle();

        // Special handling for the negative value jump that occurs after ~19 degrees
        // When we detect a negative angle, add 160 to maintain continuous sequence
        if(rawAngle < 0) {
            double correctedAngle = rawAngle + 160;
            System.out.println("Negative angle detected: " + rawAngle + ", correcting to " + correctedAngle);
            rawAngle = correctedAngle;
        }

        // Check if the jump is large (assuming jumps > 180 degrees are due to wrapping)
        if (Math.abs(rawAngle - previousRawThroughBoreAngle) > 180) {
            // Decide which way to adjust the unwrap offset
            if (rawAngle > previousRawThroughBoreAngle) {
                throughBoreUnwrapOffset -= 360;
            } else {
                throughBoreUnwrapOffset += 360;
            }
        }
        previousRawThroughBoreAngle = rawAngle;
        
        double unwrappedAngle = rawAngle + throughBoreUnwrapOffset;
        return unwrappedAngle - throughBoreOffset;
    }

    /**
     * Gets the pivot angle - returns the through bore angle as the primary source.
     * 
     * @return The current pivot angle in degrees
     */
    public double getPivotAngle() {
        // Use through bore as primary angle source for greater accuracy
        return getThroughBoreAngle();
    }

    /**
     * Checks if the pivot is at the target angle within allowed error.
     * 
     * @return true if at target angle, false otherwise
     */
    public boolean isAtTargetAngle() {
        return Math.abs(getPivotAngle() - currentTargetAngle) <= CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
    }

    /**
     * Checks if the motor feedback is valid by comparing encoders.
     * 
     * @return true if feedback is valid, false otherwise
     */
    public boolean isMotorFeedbackValid() {
        if (!isThroughBoreConnected()) {
            return true; // Assume valid if the Through Bore Encoder is disconnected
        }

        double motorAngle = pivotMotor.getPosition().getValueAsDouble();
        double throughBoreAngle = getThroughBoreAngle();
        double discrepancy = Math.abs(motorAngle - throughBoreAngle);

        return discrepancy <= CorAlConstants.THROUGH_BORE_ALLOWED_DISCREPANCY;
    }

    /**
     * Checks if the through bore encoder is connected.
     * 
     * @return true if connected, false otherwise
     */
    public boolean isThroughBoreConnected() {
        return throughBore.getFrequency() > 0;
    }

    /**
     * Checks if the pivot is outside its allowable angular range.
     * 
     * @return true if out of bounds, false otherwise
     */
    private boolean isPivotOutOfBounds() {
        double currentAngle = getPivotAngle();
        return currentAngle < CorAlConstants.CORAL_PIVOT_MIN_ANGLE || 
               currentAngle > CorAlConstants.CORAL_PIVOT_MAX_ANGLE;
    }

    /**
     * Gets the distance reading from the CANRange sensor with conversion.
     * 
     * @return The distance in appropriate units
     */
    public double getCANRangeDistance() {
        double rawDistance = canRangeSensor.getDistance().getValueAsDouble();
        double conversionFactor = 65.535 / 0.4; // Approximately 163.8375
        return rawDistance * conversionFactor;
    }

    /**
     * Checks if a game piece might be present based on CANRange reading.
     * 
     * @return true if a possible game piece is detected, false otherwise
     */
    private boolean isPossibleGamePiece() {
        return getCANRangeDistance() > 0;
    }

    /**
     * Implementation of the game piece detection logic with temporal filtering.
     * Uses a timer to confirm detections and prevent false positives.
     * 
     * @return true if a game piece is confirmed, false otherwise
     */
    public boolean isGamePieceDetected() {
        if (isPossibleGamePiece()) {
            // Game piece possibly detected (reading > 0)
            if (!possibleGamePieceDetected) {
                // First detection - start the timer
                gameDetectionTimer.reset();
                gameDetectionTimer.start();
                gameDetectionTimerRunning = true;
                possibleGamePieceDetected = true;
                // System.out.println("Possible game piece detected - starting confirmation timer");
            }
            
            // Check if timer has completed (confirmation successful)
            if (gameDetectionTimerRunning && 
                gameDetectionTimer.hasElapsed(CorAlConstants.GAME_PIECE_DETECTION_CONFIRMATION_TIME)) {
                gameDetectionTimer.stop();
                gameDetectionTimerRunning = false;
                // System.out.println("Game piece confirmed after " + 
                    // CorAlConstants.GAME_PIECE_DETECTION_CONFIRMATION_TIME + " seconds");
                // Immediately stop the intake when confirmed
                stopIntake();
                return true;
            }
        } else {
            // No game piece detected (reading <= 0)
            if (possibleGamePieceDetected) {
                // Was detecting, but reading dropped to 0 - reset detection
                possibleGamePieceDetected = false;
                if (gameDetectionTimerRunning) {
                    gameDetectionTimer.stop();
                    gameDetectionTimerRunning = false;
                    // System.out.println("Detection cancelled - CANRange reading dropped to 0");
                }
            }
        }
        
        // Return true only if game piece has been confirmed
        return possibleGamePieceDetected && !gameDetectionTimerRunning;
    }

    /**
     * Performs all safety checks and takes appropriate actions.
     * Ensures system operates within safe parameters.
     */
    private void performSafetyChecks() {
        // Check for motor feedback validity
        if (!isMotorFeedbackValid() && isThroughBoreConnected()) {
            System.err.println("WARNING: Motor feedback discrepancy detected!");
            System.err.println("Motor Encoder Angle: " + pivotMotor.getPosition().getValueAsDouble());
            System.err.println("Through Bore Encoder Angle: " + getThroughBoreAngle());
            
            // Sync motor encoder to through bore reading to prevent further drift
            pivotMotor.setPosition(getThroughBoreAngle());
            System.out.println("Motor encoder synced to through bore reading");
        }

        // Check for out-of-bounds conditions
        if (isPivotOutOfBounds()) {
            System.err.println("WARNING: Pivot out of bounds! Angle: " + getPivotAngle());
            stopPivot();
        }
    }

    /**
     * Periodic method called by the CommandScheduler.
     * Updates subsystem state and performs safety checks.
     */
    @Override
    public void periodic() {
        // Update game piece detection state
        gamePieceDetected = isGamePieceDetected();
        
        // Perform safety checks
        performSafetyChecks();

        // If position control is active and the target angle is reached, disable position control
        if (positionControlActive && isAtTargetAngle()) {
            disablePositionControl();
            System.out.println("Position control completed. Manual control enabled.");
        }
    }
}