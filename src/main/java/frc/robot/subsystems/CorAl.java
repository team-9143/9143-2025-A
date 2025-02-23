package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CorAlConstants;

public class CorAl extends SubsystemBase {
	private final TalonFX pivotMotor;
	private final TalonFX intakeMotor;
	private final DigitalInput throughBoreInput;
	private final DutyCycle throughBore;
	private final ShuffleboardTab tab;
	private final SimpleWidget pivotTargetAngleWidget;
	private final SimpleWidget resetPivotEncoderWidget;

	// Control requests for the motors
	private final PositionVoltage positionRequest;
	private final DutyCycleOut percentRequest;

	public CorAl() {
		// Initialize motors
		pivotMotor = new TalonFX(CorAlConstants.CORAL_PIVOT_MOTOR_ID);
		intakeMotor = new TalonFX(CorAlConstants.CORAL_INTAKE_MOTOR_ID);

		// Initialize REV Through Bore Encoder (Absolute Mode)
		throughBoreInput = new DigitalInput(CorAlConstants.THROUGH_BORE_DIO_PORT);
        throughBore = new DutyCycle(throughBoreInput);

		// Initialize control requests
		positionRequest = new PositionVoltage(0).withSlot(0);
		percentRequest = new DutyCycleOut(0);

		// Configure motors
		configureMotor(pivotMotor, CorAlConstants.CORAL_PIVOT_MOTOR_INVERTED, true);  // Pivot motor
		configureMotor(intakeMotor, CorAlConstants.CORAL_INTAKE_MOTOR_INVERTED, false); // Intake motor

		// Initialize Shuffleboard tab and widgets
		tab = Shuffleboard.getTab("CorAl");
		pivotTargetAngleWidget = tab.add("Pivot Target Angle", 0)
			.withPosition(0, 0)
			.withSize(2, 1);
		resetPivotEncoderWidget = tab.add("Reset Pivot Encoder", false)
			.withPosition(2, 0)
			.withSize(2, 1);

		// Configure Shuffleboard
		configureShuffleboard();
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
			// Position and velocity scaling
			config.Feedback.SensorToMechanismRatio = CorAlConstants.CORAL_PIVOT_POSITION_CONVERSION;

			// PID configuration
			config.Slot0.kP = CorAlConstants.CORAL_PIVOT_kP;
			config.Slot0.kI = CorAlConstants.CORAL_PIVOT_kI;
			config.Slot0.kD = CorAlConstants.CORAL_PIVOT_kD;
    		config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
			
			// Motion Magic configuration if needed
			config.MotionMagic.MotionMagicAcceleration = 100;
			config.MotionMagic.MotionMagicCruiseVelocity = 50;
		}

		// Apply configuration
		motor.getConfigurator().apply(config);
	}

	private void configureShuffleboard() {
		// Control Panel (Row 0)
		tab.add("Target Angle", 0)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of(
				"Min", CorAlConstants.CORAL_PIVOT_MIN_ANGLE,
				"Max", CorAlConstants.CORAL_PIVOT_MAX_ANGLE))
			.withPosition(0, 0)
			.withSize(2, 1);

		tab.add("Manual Mode", false)
			.withWidget(BuiltInWidgets.kToggleSwitch)
			.withPosition(2, 0)
			.withSize(1, 1);

		tab.add("Manual Speed", 0.0)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("Min", -1.0, "Max", 1.0))
			.withPosition(3, 0)
			.withSize(2, 1);

		// Status Panel (Row 1)
		tab.addNumber("Current Angle", this::getPivotAngle)
			.withWidget(BuiltInWidgets.kDial)
			.withProperties(Map.of(
				"Min", CorAlConstants.CORAL_PIVOT_MIN_ANGLE,
				"Max", CorAlConstants.CORAL_PIVOT_MAX_ANGLE))
			.withPosition(0, 1)
			.withSize(2, 2);

		tab.addBoolean("At Target", this::isAtTargetAngle)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withPosition(2, 1)
			.withSize(1, 1);

		// Motor Status (Row 2)
		tab.addNumber("Pivot Current", () -> pivotMotor.getSupplyCurrent().getValueAsDouble())
			.withWidget(BuiltInWidgets.kGraph)
			.withPosition(0, 3)
			.withSize(2, 2);

		tab.addNumber("Intake Current", () -> intakeMotor.getSupplyCurrent().getValueAsDouble())
			.withWidget(BuiltInWidgets.kGraph)
			.withPosition(2, 3)
			.withSize(2, 2);

		// PID Tuning (Row 4)
		tab.addNumber("P Gain", () -> CorAlConstants.CORAL_PIVOT_kP)
			.withPosition(0, 5)
			.withSize(1, 1);
		tab.addNumber("I Gain", () -> CorAlConstants.CORAL_PIVOT_kI)
			.withPosition(1, 5)
			.withSize(1, 1);
		tab.addNumber("D Gain", () -> CorAlConstants.CORAL_PIVOT_kD)
			.withPosition(2, 5)
			.withSize(1, 1);
		tab.addNumber("G Gain", () -> CorAlConstants.CORAL_PIVOT_kG)
			.withPosition(3, 5)
			.withSize(1, 1);
	}

	public void setPivotAngle(double targetAngle) {
		// Clamp target angle
		targetAngle = Math.min(Math.max(targetAngle, 
			CorAlConstants.CORAL_PIVOT_MIN_ANGLE),
			CorAlConstants.CORAL_PIVOT_MAX_ANGLE);

			// Use Through Bore Encoder as backup if motor feedback is invalid
			if (!isMotorFeedbackValid()) {
				targetAngle = getThroughBoreAngle();
				System.err.println("Using Through Bore Encoder as backup");
			}

		double gravityFF = CorAlConstants.CORAL_PIVOT_kG * Math.cos(Math.toRadians(targetAngle));
		
		pivotMotor.setControl(positionRequest.withPosition(targetAngle).withFeedForward(gravityFF));
		pivotTargetAngleWidget.getEntry().setDouble(targetAngle);
	}

	public void resetPivotEncoder() {
		pivotMotor.setPosition(0);
	}

	public void setIntakeSpeed(double speed) {
		intakeMotor.setControl(percentRequest.withOutput(speed));
	}

	public void stopIntake() {
		setIntakeSpeed(0);
	}

	public void manualPivotControl(double speed) {
		// Apply deadband and limits
		if (Math.abs(speed) < CorAlConstants.CORAL_MANUAL_CONTROL_DEADBAND) {
			speed = 0;
		}
		speed = Math.min(Math.max(speed * CorAlConstants.CORAL_MANUAL_SPEED_LIMIT, -1), 1);

		// Safety checks
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

	public void stopRoller() {
		intakeMotor.setControl(percentRequest.withOutput(0));
	}

	private boolean isPivotOutOfBounds() {
		double currentAngle = getPivotAngle();
		return currentAngle < CorAlConstants.CORAL_PIVOT_MIN_ANGLE || 
			currentAngle > CorAlConstants.CORAL_PIVOT_MAX_ANGLE;
	}

	public double getPivotAngle() {
		return pivotMotor.getPosition().getValueAsDouble();
	}

	public boolean isAtTargetAngle() {
		return Math.abs(pivotTargetAngleWidget.getEntry().getDouble(0) - getPivotAngle()) <= 
			CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
	}

	public double getThroughBoreAngle() {
        // Measure the duty cycle (ratio of high time to total period)
        double dutyCycle = throughBore.getOutput();

        // Convert the duty cycle to an angle (example conversion formula)
        double angle = dutyCycle * CorAlConstants.THROUGH_BORE_MAX_ANGLE;

        return angle;
    }

	public boolean isMotorFeedbackValid() {
        double krakenAngle = getPivotAngle();
        double throughBoreAngle = getThroughBoreAngle();
        double discrepancy = Math.abs(krakenAngle - throughBoreAngle);

        return discrepancy <= CorAlConstants.THROUGH_BORE_ALLOWED_DISCREPANCY;
    }

	private void validateMotorFeedback() {
        if (!isMotorFeedbackValid()) {
            System.err.println("WARNING: Motor feedback discrepancy detected!");
            System.err.println("Kraken X60 Angle: " + getPivotAngle());
            System.err.println("Through Bore Encoder Angle: " + getThroughBoreAngle());
        }
    }

	@Override
	public void periodic() {
		// Log Kraken X60 and Through Bore Encoder angles for testing
		double krakenAngle = getPivotAngle();
		double throughBoreAngle = getThroughBoreAngle();
		System.out.println("Kraken X60 Angle: " + krakenAngle + " | Through Bore Encoder Angle: " + throughBoreAngle);
		
		// Check for encoder reset
		if (resetPivotEncoderWidget.getEntry().getBoolean(false)) {
			resetPivotEncoder();
			resetPivotEncoderWidget.getEntry().setBoolean(false);
		}

		// Validate motor feedback
        validateMotorFeedback();

		// Safety check
		if (isPivotOutOfBounds()) {
			stopPivot();
		}
	}
}