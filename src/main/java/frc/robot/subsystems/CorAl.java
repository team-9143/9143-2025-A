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
    private final ShuffleboardTab tab;

    // Control requests for the motors
    private final PositionVoltage positionRequest;
    private final DutyCycleOut percentRequest;

	// Timer for delaying roller stop after game piece detection
    private final Timer detectionTimer = new Timer();
    private boolean gamePieceDetected = false;

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
        configureMotor(pivotMotor, CorAlConstants.CORAL_PIVOT_MOTOR_INVERTED, true);  // Pivot motor
        configureMotor(intakeMotor, CorAlConstants.CORAL_INTAKE_MOTOR_INVERTED, false); // Intake motor

        // Initialize Shuffleboard
        tab = Shuffleboard.getTab("CorAl");
        configureShuffleboard();

		// Initialize detection timer
        detectionTimer.reset();
    }

    private void configureMotor(TalonFX motor, boolean isInverted, boolean usePID) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure inversion and brake mode
        config.MotorOutput.Inverted = isInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = usePID ? 
            CorAlConstants.CORAL_PIVOT_CURRENT_LIMIT : 
            CorAlConstants.CORAL_INTAKE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configure the encoder and PID (only for pivot motor)
        if (usePID) {
            config.Feedback.SensorToMechanismRatio = CorAlConstants.CORAL_PIVOT_POSITION_CONVERSION;
            config.Slot0.kP = CorAlConstants.CORAL_PIVOT_kP;
            config.Slot0.kI = CorAlConstants.CORAL_PIVOT_kI;
            config.Slot0.kD = CorAlConstants.CORAL_PIVOT_kD;
            config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
        }

        // Apply configuration
        motor.getConfigurator().apply(config);
    }

    private void configureShuffleboard() {
        // Control Panel
        tab.add("Target Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", CorAlConstants.CORAL_PIVOT_MIN_ANGLE, "Max", CorAlConstants.CORAL_PIVOT_MAX_ANGLE))
            .withPosition(0, 0)
            .withSize(2, 1);

        tab.addBoolean("At Target", this::isAtTargetAngle)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 0)
            .withSize(1, 1);

        // Status Panel
        tab.addNumber("Current Angle", this::getPivotAngle)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", CorAlConstants.CORAL_PIVOT_MIN_ANGLE, "Max", CorAlConstants.CORAL_PIVOT_MAX_ANGLE))
            .withPosition(0, 1)
            .withSize(2, 2);

        tab.addNumber("Through Bore Angle", this::getThroughBoreAngle)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(2, 1);

        // Motor Status
        tab.addNumber("Pivot Current", () -> pivotMotor.getSupplyCurrent().getValueAsDouble())
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(0, 3)
            .withSize(2, 2);

        tab.addNumber("Intake Current", () -> intakeMotor.getSupplyCurrent().getValueAsDouble())
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(2, 3)
            .withSize(2, 2);

		// CANRange Sensor Status
        tab.addNumber("CANRange Distance", this::getCANRangeDistance)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(2, 1);
    }

    public void setPivotAngle(double targetAngle) {
        targetAngle = Math.min(Math.max(targetAngle, CorAlConstants.CORAL_PIVOT_MIN_ANGLE), CorAlConstants.CORAL_PIVOT_MAX_ANGLE);

        if (!isMotorFeedbackValid()) {
            targetAngle = getThroughBoreAngle();
            System.err.println("Using Through Bore Encoder as backup");
        }

        double gravityFF = CorAlConstants.CORAL_PIVOT_kG * Math.cos(Math.toRadians(targetAngle));
        pivotMotor.setControl(positionRequest.withPosition(targetAngle).withFeedForward(gravityFF));
    }

    public void resetPivotEncoder() {
        pivotMotor.setPosition(0);
    }

    public void setIntakeSpeed(double speed) {
        // Stop the intake if a game piece is detected and the delay has elapsed
        if (isGamePieceDetected() && detectionTimer.hasElapsed(0.5)) { // 0.5-second delay
            stopIntake();
        } else {
            intakeMotor.setControl(percentRequest.withOutput(speed));
        }
    }

    public void stopIntake() {
        setIntakeSpeed(0);
    }

    public void manualPivotControl(double speed) {
        if (Math.abs(speed) < CorAlConstants.CORAL_MANUAL_CONTROL_DEADBAND) {
            speed = 0;
        }
        speed = Math.min(Math.max(speed * CorAlConstants.CORAL_MANUAL_SPEED_LIMIT, -1), 1);

        if (!isPivotOutOfBounds() || 
            (getPivotAngle() <= CorAlConstants.CORAL_PIVOT_MIN_ANGLE && speed > 0) ||
            (getPivotAngle() >= CorAlConstants.CORAL_PIVOT_MAX_ANGLE && speed < 0)) {
            pivotMotor.setControl(percentRequest.withOutput(speed));
        } else {
            stopPivot();
        }
    }

    public void stopPivot() {
        pivotMotor.setControl(percentRequest.withOutput(0));
    }

    public double getPivotAngle() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtTargetAngle() {
        return Math.abs(getPivotAngle() - pivotMotor.getClosedLoopReference().getValueAsDouble()) <= CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
    }

    public double getThroughBoreAngle() {
        double dutyCycle = throughBore.getOutput();
        return dutyCycle * (CorAlConstants.THROUGH_BORE_MAX_ANGLE - CorAlConstants.THROUGH_BORE_MIN_ANGLE) + CorAlConstants.THROUGH_BORE_MIN_ANGLE;
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
        return currentAngle < CorAlConstants.CORAL_PIVOT_MIN_ANGLE || currentAngle > CorAlConstants.CORAL_PIVOT_MAX_ANGLE;
    }

	public double getCANRangeDistance() {
        return canRangeSensor.getDistance().getValueAsDouble(); // Get distance in millimeters
    }

    public boolean isGamePieceDetected() {
        return getCANRangeDistance() < CorAlConstants.CANRANGE_DETECTION_THRESHOLD;
    }

    @Override
    public void periodic() {
		// Check if a game piece is detected
        if (isGamePieceDetected()) {
            if (!gamePieceDetected) {
                // Start the timer when the game piece is first detected
                detectionTimer.reset();
                detectionTimer.start();
                gamePieceDetected = true;
            }
        } else {
            // Reset the timer and flag if no game piece is detected
            detectionTimer.stop();
            detectionTimer.reset();
            gamePieceDetected = false;
        }

        // Stop the intake if a game piece is detected and the delay has elapsed
        if (isGamePieceDetected() && detectionTimer.hasElapsed(0.5)) { // 0.5-second delay
            stopIntake();
        }

        if (!isMotorFeedbackValid()) {
            System.err.println("WARNING: Motor feedback discrepancy detected!");
            System.err.println("Kraken X60 Angle: " + getPivotAngle());
            System.err.println("Through Bore Encoder Angle: " + getThroughBoreAngle());
            stopPivot(); // Stop the mechanism to prevent damage
        }

        if (isPivotOutOfBounds()) {
            stopPivot();
        }
    }
}