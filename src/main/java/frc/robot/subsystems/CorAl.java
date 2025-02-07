package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CorAlConstants;

public class CorAl extends SubsystemBase {
	private final TalonFX pivotMotor;
	private final TalonFX intakeMotor;
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
			
			// Motion Magic configuration if needed
			config.MotionMagic.MotionMagicAcceleration = 100;
			config.MotionMagic.MotionMagicCruiseVelocity = 50;
		}

		// Apply configuration
		motor.getConfigurator().apply(config);
	}

	private void configureShuffleboard() {
		// Status indicators
		tab.addNumber("Pivot Angle", this::getPivotAngle)
		   .withPosition(0, 1)
		   .withSize(2, 1);
		
		tab.addNumber("Pivot Current", () -> pivotMotor.getSupplyCurrent().getValueAsDouble())
		   .withPosition(2, 1)
		   .withSize(2, 1);
		   
		tab.addNumber("Intake Current", () -> intakeMotor.getSupplyCurrent().getValueAsDouble())
		   .withPosition(4, 1)
		   .withSize(2, 1);

		// Status booleans
		tab.addBoolean("Pivot Out of Bounds", this::isPivotOutOfBounds)
		   .withPosition(0, 2)
		   .withSize(2, 1);
		   
		tab.addBoolean("At Target Angle", this::isAtTargetAngle)
		   .withPosition(2, 2)
		   .withSize(2, 1);
	}

	@Override
	public void periodic() {
		// Check for encoder reset
		if (resetPivotEncoderWidget.getEntry().getBoolean(false)) {
			resetPivotEncoder();
			resetPivotEncoderWidget.getEntry().setBoolean(false);
		}

		// Safety check
		if (isPivotOutOfBounds()) {
			stopPivot();
		}
	}

	public void setPivotAngle(double targetAngle) {
		// Clamp target angle
		targetAngle = Math.min(Math.max(targetAngle, 
			CorAlConstants.CORAL_PIVOT_MIN_ANGLE),
			CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
		
		pivotMotor.setControl(positionRequest.withPosition(targetAngle));
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
}