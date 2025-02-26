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

public class CorAl extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final TalonFX intakeMotor;
    private final DigitalInput throughBoreInput;
    private final DutyCycle throughBore;
    private final CANrange canRangeSensor;
    
    // Shuffleboard elements
    private final ShuffleboardTab coralTab;

    // Control requests for the motors
    private final PositionVoltage positionRequest;
    private final DutyCycleOut percentRequest;

    // Timer for delaying roller stop after game piece detection
    private final Timer detectionTimer = new Timer();
    private boolean gamePieceDetected = false;

    private final Timer rollerTimer = new Timer();
    private boolean rollerTimerRunning = false;
    private double rollerRunTime = 1.0;
    
    // Target angle tracking
    private double currentTargetAngle = 0;

    public CorAl() {
        // Initialize motors
        pivotMotor = new TalonFX(CorAlConstants.CORAL_PIVOT_MOTOR_ID);
        intakeMotor = new TalonFX(CorAlConstants.CORAL_INTAKE_MOTOR_ID);

        // Initialize REV Through Bore Encoder
        throughBoreInput = new DigitalInput(CorAlConstants.THROUGH_BORE_DIO_PORT);
        throughBore = new DutyCycle(throughBoreInput);

        // Initialize CANRange sensor
        canRangeSensor = new CANrange(CorAlConstants.CANRANGE_SENSOR_ID);

        // Initialize control requests
        positionRequest = new PositionVoltage(0).withSlot(0);
        percentRequest = new DutyCycleOut(0);

        // Configure motors
        configurePivotMotor(pivotMotor);
        configureIntakeMotor(intakeMotor);

        // Initialize Shuffleboard tab and layouts
        coralTab = Shuffleboard.getTab("CorAl");

        // Configure Shuffleboard
        configureShuffleboard();

        // Initialize detection timer
        detectionTimer.reset();
    }

    private void configurePivotMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure inversion and brake mode
        config.MotorOutput.Inverted = CorAlConstants.CORAL_PIVOT_MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = CorAlConstants.CORAL_PIVOT_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configure the encoder and PID
        config.Feedback.SensorToMechanismRatio = CorAlConstants.CORAL_PIVOT_POSITION_CONVERSION;
        config.Slot0.kP = CorAlConstants.CORAL_PIVOT_kP;
        config.Slot0.kI = CorAlConstants.CORAL_PIVOT_kI;
        config.Slot0.kD = CorAlConstants.CORAL_PIVOT_kD;
        
        // Use built-in gravity compensation - don't apply manual FF
        config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = CorAlConstants.CORAL_PIVOT_kF;

        // Apply configuration
        motor.getConfigurator().apply(config);
    }

    private void configureIntakeMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure inversion and brake mode
        config.MotorOutput.Inverted = CorAlConstants.CORAL_INTAKE_MOTOR_INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = CorAlConstants.CORAL_INTAKE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply configuration
        motor.getConfigurator().apply(config);
    }

    private void configureShuffleboard() {
        // Status Layout - Boolean indicators
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
        coralTab.addNumber("Current Angle", this::getPivotAngle)
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
            
        // Motor Layout - Current and output information
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
            
        // PID Layout - Control parameters
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

    public void setPivotAngle(double targetAngle) {
        // Constrain the target angle to valid range
        targetAngle = Math.min(Math.max(targetAngle, CorAlConstants.CORAL_PIVOT_MIN_ANGLE), 
                             CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
        currentTargetAngle = targetAngle;

        // Check if the feedback is valid before setting the position
        if (isMotorFeedbackValid()) {
            // Let Phoenix 6 handle gravity compensation through the configured slot
            pivotMotor.setControl(positionRequest.withPosition(targetAngle));
        } else {
            // If motor feedback is invalid, stop the pivot for safety
            stopPivot();
            System.err.println("WARNING: Cannot set position - motor feedback invalid!");
        }
    }

    public void resetPivotEncoder() {
        pivotMotor.setPosition(getThroughBoreAngle());
        System.out.println("Pivot encoder reset to Through Bore value: " + getThroughBoreAngle());
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.setControl(percentRequest.withOutput(speed));
    }

    public void stopIntake() {
        setIntakeSpeed(0);
    }

    public void startRollersWithTimer(double speed, double runTimeSeconds) {
        setIntakeSpeed(speed);
        rollerRunTime = runTimeSeconds;
        rollerTimer.reset();
        rollerTimer.start();
        rollerTimerRunning = true;
    }

    public void manualPivotControl(double speed) {
        pivotMotor.setControl(percentRequest.withOutput(speed));
    }

    public void stopPivot() {
        pivotMotor.setControl(percentRequest.withOutput(0));
    }

    public double getPivotAngle() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtTargetAngle() {
        return Math.abs(getPivotAngle() - currentTargetAngle) <= CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
    }

    public double getThroughBoreAngle() {
        double dutyCycle = throughBore.getOutput();
        return dutyCycle * (CorAlConstants.THROUGH_BORE_MAX_ANGLE - CorAlConstants.THROUGH_BORE_MIN_ANGLE) 
               + CorAlConstants.THROUGH_BORE_MIN_ANGLE;
    }

    public boolean isMotorFeedbackValid() {
        if (!isThroughBoreConnected()) {
            return true; // Assume valid if the Through Bore Encoder is disconnected
        }

        double krakenAngle = getPivotAngle();
        double throughBoreAngle = getThroughBoreAngle();
        double discrepancy = Math.abs(krakenAngle - throughBoreAngle);

        return discrepancy <= CorAlConstants.THROUGH_BORE_ALLOWED_DISCREPANCY;
    }

    public boolean isThroughBoreConnected() {
        return throughBore.getFrequency() > 0;
    }

    private boolean isPivotOutOfBounds() {
        double currentAngle = getPivotAngle();
        return currentAngle < CorAlConstants.CORAL_PIVOT_MIN_ANGLE || 
               currentAngle > CorAlConstants.CORAL_PIVOT_MAX_ANGLE;
    }

    public double getCANRangeDistance() {
        return canRangeSensor.getDistance().getValueAsDouble(); // Get distance in millimeters
    }

    public boolean isGamePieceDetected() {
        return getCANRangeDistance() < CorAlConstants.CANRANGE_DETECTION_THRESHOLD;
    }

    // Handles game piece detection and manages the timer for auto-stopping intake
    private void handleGamePieceDetection() {
        if (isGamePieceDetected()) {
            if (!gamePieceDetected) {
                // Start the timer when the game piece is first detected
                detectionTimer.reset();
                detectionTimer.start();
                gamePieceDetected = true;
            }
            
            // Stop the intake if the delay has elapsed
            if (detectionTimer.hasElapsed(0.5)) { // 0.5-second delay
                stopIntake();
            }
        } else {
            // Reset the timer and flag if no game piece is detected
            detectionTimer.stop();
            detectionTimer.reset();
            gamePieceDetected = false;
        }
    }
    
    // Performs all safety checks and takes appropriate actions
    private void performSafetyChecks() {
        // Check for motor feedback validity
        if (!isMotorFeedbackValid()) {
            System.err.println("WARNING: Motor feedback discrepancy detected!");
            System.err.println("Kraken X60 Angle: " + getPivotAngle());
            System.err.println("Through Bore Encoder Angle: " + getThroughBoreAngle());
            stopPivot(); // Stop the mechanism to prevent damage
        }

        // Check for out-of-bounds conditions
        if (isPivotOutOfBounds()) {
            System.err.println("WARNING: Pivot out of bounds! Angle: " + getPivotAngle());
            stopPivot();
        }
    }

    @Override
    public void periodic() {
        // Game piece detection logic (consolidated in one place)
        handleGamePieceDetection();
        
        // Safety checks
        performSafetyChecks();

        // Check if rollers should be stopped based on timer.
        if (rollerTimerRunning && rollerTimer.get() >= rollerRunTime) {
            stopIntake();
            rollerTimer.stop();
            rollerTimerRunning = false;
        }
    }
}