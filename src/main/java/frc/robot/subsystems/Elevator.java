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
	private final ShuffleboardTab tab;
	private final SimpleWidget targetPositionWidget;
	private final SimpleWidget manualControlWidget;
	private final SimpleWidget manualSpeedWidget;
	private final SimpleWidget resetEncoderWidget;

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

		// Initialize Shuffleboard tab and widgets
		tab = Shuffleboard.getTab("Elevator");
		targetPositionWidget = tab.add("Target Position", 0)
			.withPosition(0, 0)
			.withSize(2, 1);
		manualControlWidget = tab.add("Manual Control", false)
			.withPosition(2, 0)
			.withSize(2, 1);
		manualSpeedWidget = tab.add("Manual Speed", 0)
			.withPosition(4, 0)
			.withSize(2, 1);
		resetEncoderWidget = tab.add("Reset Encoder", false)
			.withPosition(6, 0)
			.withSize(2, 1);

		// Configure Shuffleboard
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
			.outputRange(-1, 1);

		// Apply configuration
		motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void configureShuffleboard() {
		// Status indicators
		tab.addNumber("Left Encoder Position", leftEncoder::getPosition)
		   .withPosition(0, 1)
		   .withSize(2, 1);
		tab.addNumber("Right Encoder Position", rightEncoder::getPosition)
		   .withPosition(2, 1)
		   .withSize(2, 1);

		// Status booleans
		tab.addBoolean("At Target Position", this::isAtTargetPosition)
		   .withPosition(4, 1)
		   .withSize(2, 1);
		tab.addBoolean("Manual Control Active", () -> manualControlWidget.getEntry().getBoolean(false))
		   .withPosition(6, 1)
		   .withSize(2, 1);
	}

	@Override
	public void periodic() {
		// Check for encoder reset
		if (resetEncoderWidget.getEntry().getBoolean(false)) {
			resetEncoders();
			resetEncoderWidget.getEntry().setBoolean(false);
		}

		// Control logic
		if (manualControlWidget.getEntry().getBoolean(false)) {
			// Manual control
			double manualSpeed = manualSpeedWidget.getEntry().getDouble(0);
			manualControl(manualSpeed);
		} else {
			// Position control
			double targetPosition = targetPositionWidget.getEntry().getDouble(0);
			setPosition(targetPosition);
		}
	}

	public void setPosition(double targetPosition) {
		// Clamp target position within safe limits
		targetPosition = Math.min(Math.max(targetPosition, ElevatorConstants.ELEVATOR_MIN_POSITION),
			ElevatorConstants.ELEVATOR_MAX_POSITION);

		leftController.setReference(targetPosition, ControlType.kPosition);
		rightController.setReference(targetPosition, ControlType.kPosition);
		targetPositionWidget.getEntry().setDouble(targetPosition);
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
		return leftEncoder.getPosition(); // Assuming both encoders are synchronized
	}

	public boolean isAtTargetPosition() {
		double targetPosition = targetPositionWidget.getEntry().getDouble(0);
		double currentPosition = getCurrentPosition();
		return Math.abs(targetPosition - currentPosition) <= ElevatorConstants.ELEVATOR_ALLOWED_ERROR;
	}
}