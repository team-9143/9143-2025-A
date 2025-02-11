/*
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlLowConstants;

public class AlLow extends SubsystemBase {
	private final SparkMax pivotMotor;
	private final SparkMax rollerMotor;
	private final RelativeEncoder pivotEncoder;
	private final SparkClosedLoopController pivotController;
	private final ShuffleboardTab tab;
	private final SimpleWidget pivotTargetAngleWidget;
	private final SimpleWidget manualControlWidget;
	private final SimpleWidget manualSpeedWidget;
	private final SimpleWidget resetPivotEncoderWidget;
	private final SparkMaxConfig pivotConfig;
	private final SparkMaxConfig rollerConfig;

	public AlLow() {
		// Initialize motors
		pivotMotor = new SparkMax(AlLowConstants.ALLOW_PIVOT_MOTOR_ID, MotorType.kBrushless);
		rollerMotor = new SparkMax(AlLowConstants.ALLOW_ROLLER_MOTOR_ID, MotorType.kBrushless);

		// Configure motors
		pivotConfig = new SparkMaxConfig();
		rollerConfig = new SparkMaxConfig();
		
		configureMotor(pivotMotor, pivotConfig, AlLowConstants.ALLOW_PIVOT_MOTOR_INVERTED, true);
		configureMotor(rollerMotor, rollerConfig, AlLowConstants.ALLOW_ROLLER_MOTOR_INVERTED, false);

		// Get encoder and controller
		pivotEncoder = pivotMotor.getEncoder();
		pivotController = pivotMotor.getClosedLoopController();

		// Initialize Shuffleboard tab and widgets
		tab = Shuffleboard.getTab("AlLow");
		pivotTargetAngleWidget = tab.add("Pivot Target Angle", 0)
			.withPosition(0, 0)
			.withSize(2, 1);
		manualControlWidget = tab.add("Manual Control", false)
			.withPosition(2, 0)
			.withSize(2, 1);
		manualSpeedWidget = tab.add("Manual Speed", 0)
			.withPosition(4, 0)
			.withSize(2, 1);
		resetPivotEncoderWidget = tab.add("Reset Pivot Encoder", false)
			.withPosition(6, 0)
			.withSize(2, 1);

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

			// Configure closed-loop control
			config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(AlLowConstants.ALLOW_PIVOT_kP)
				.i(AlLowConstants.ALLOW_PIVOT_kI)
				.d(AlLowConstants.ALLOW_PIVOT_kD)
				.outputRange(-1, 1);
		}

		// Apply configuration
		motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void configureShuffleboard() {
		// Status indicators
		tab.addNumber("Pivot Angle", this::getPivotAngle)
		   .withPosition(0, 1)
		   .withSize(2, 1);
		
		tab.addNumber("Pivot Current", pivotMotor::getOutputCurrent)
		   .withPosition(2, 1)
		   .withSize(2, 1);
		   
		tab.addNumber("Roller Current", rollerMotor::getOutputCurrent)
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

		// Control logic
		if (manualControlWidget.getEntry().getBoolean(false)) {
			// Manual control
			double manualSpeed = manualSpeedWidget.getEntry().getDouble(0);
			manualPivotControl(manualSpeed);
		} else {
			// Position control
			double targetAngle = pivotTargetAngleWidget.getEntry().getDouble(0);
			setPivotAngle(targetAngle);
		}
	}

	public void setPivotAngle(double targetAngle) {
		// Clamp target angle
		targetAngle = Math.min(Math.max(targetAngle, AlLowConstants.ALLOW_PIVOT_MIN_ANGLE),
			AlLowConstants.ALLOW_PIVOT_MAX_ANGLE);
		
		pivotController.setReference(targetAngle, ControlType.kPosition);
		pivotTargetAngleWidget.getEntry().setDouble(targetAngle);
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
		// Apply deadband and limits
		if (Math.abs(speed) < AlLowConstants.ALLOW_MANUAL_CONTROL_DEADBAND) {
			speed = 0;
		}
		speed = Math.min(Math.max(speed * AlLowConstants.ALLOW_MANUAL_SPEED_LIMIT, -1), 1);

		// Safety checks
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
		double targetAngle = pivotTargetAngleWidget.getEntry().getDouble(0);
		double currentAngle = getPivotAngle();
		return Math.abs(targetAngle - currentAngle) <= AlLowConstants.ALLOW_PIVOT_ALLOWED_ERROR;
	}
}
*/