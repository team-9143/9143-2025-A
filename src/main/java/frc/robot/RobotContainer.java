package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.AlLowConstants;
import frc.robot.Constants.CorAlConstants;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CorAl;
//import frc.robot.subsystems.AlLow;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 20% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver_controller = new CommandXboxController(0);
    private final CommandXboxController operator_controller = new CommandXboxController(1);

    public final Swerve swerve = TunerConstants.createDrivetrain();

    private final Elevator elevator = new Elevator();
    private final CorAl coral = new CorAl();
    //private final AlLow allow = new AlLow();
    private final Vision vision = new Vision(swerve);

    private final SendableChooser<Command> autoChooser;
    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    public RobotContainer() {
        swerve.setVision(vision);

        // Create auto chooser and put it on the Auto tab in Shuffleboard
        autoChooser = AutoBuilder.buildAutoChooser();
        autoTab.add("Auto Mode", autoChooser)
            .withSize(5, 3)
            .withPosition(0, 0);

        configureBindings();
    }

    private void configureBindings() {
        configureSwerveBindings();
        configureElevatorBindings();
        configureCorAlBindings();
        //configureAlLowBindings();
    }

    private void configureSwerveBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerve.setDefaultCommand(
            // Drivetrain will execute this command periodically
            swerve.applyRequest(() ->
                drive.withVelocityX(-driver_controller.getLeftY() * MaxSpeed * 0.25) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_controller.getLeftX() * MaxSpeed * 0.25) // Drive left with negative X (left)
                    .withRotationalRate(-driver_controller.getRightX() * MaxAngularRate)// Drive clockwise with X (right)
            )
        );

        driver_controller.a().whileTrue(swerve.applyRequest(() -> brake));
        driver_controller.b().whileTrue(swerve.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_controller.getLeftY(), -driver_controller.getLeftX()))
        ));

        driver_controller.pov(0).whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver_controller.pov(180).whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver_controller.back().and(driver_controller.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        driver_controller.back().and(driver_controller.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        driver_controller.start().and(driver_controller.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        driver_controller.start().and(driver_controller.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driver_controller.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        driver_controller.y().onTrue(Commands.runOnce(() -> {
            boolean newTrackingState = !swerve.isVisionTrackingEnabled();
            swerve.setVisionTrackingEnabled(newTrackingState);
            
            if (newTrackingState) {
                swerve.aprilTagTrackingCommand.schedule();
            } else {
                if (swerve.getCurrentCommand() == swerve.aprilTagTrackingCommand) {
                    swerve.aprilTagTrackingCommand.cancel();
                }
                // Stop the robot when tracking is disabled
                swerve.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
            }
        }));

        swerve.registerTelemetry(logger::telemeterize);
    }

    private void configureElevatorBindings() {
        // Elevator preset heights
        operator_controller.povDown().onTrue(Commands.runOnce(() -> {
            elevator.setManualMode(false);
            elevator.setPosition(ElevatorConstants.PresetHeights.BASE.getHeight());
        }, elevator));
    
        operator_controller.povLeft().onTrue(Commands.runOnce(() -> {
            elevator.setManualMode(false);
            elevator.setPosition(ElevatorConstants.PresetHeights.CORAL_L2.getHeight());
        }, elevator));
    
        operator_controller.povRight().onTrue(Commands.runOnce(() -> {
            elevator.setManualMode(false);
            elevator.setPosition(ElevatorConstants.PresetHeights.CORAL_L3.getHeight());
        }, elevator));
    
        operator_controller.povUp().onTrue(Commands.runOnce(() -> {
            elevator.setManualMode(false);
            elevator.setPosition(ElevatorConstants.PresetHeights.CORAL_L4.getHeight());
        }, elevator));
    
        // Elevator encoder reset
        operator_controller.leftBumper().onTrue(Commands.runOnce(() -> elevator.resetEncoders(), elevator).ignoringDisable(true));
    
        // Elevator manual control - modified to respect position control
        elevator.setDefaultCommand(Commands.run(() -> {
            double speed = -operator_controller.getLeftY();
            if (Math.abs(speed) > ElevatorConstants.ELEVATOR_MANUAL_CONTROL_DEADBAND) {
                // Only set manual mode if stick is actually being moved
                elevator.setManualMode(true);
                elevator.manualControl(speed);
            } else if (elevator.isInManualMode()) {
                // Only stop if we're in manual mode
                elevator.stopElevator();
            }
            // If not in manual mode, position control will be handled by periodic()
        }, elevator));
    }

    private void configureCorAlBindings() {
        // CorAl lifting
        operator_controller.rightTrigger().onTrue(
            Commands.sequence(
                Commands.runOnce(coral::stopIntake, coral),
                Commands.runOnce(() -> coral.setPivotAngle(CorAlConstants.PivotPresetAngles.RAISE.getAngle()), coral),
                Commands.waitUntil(coral::isAtTargetAngle)                
            )
        );

        // CorAl retraction
        operator_controller.a().onTrue(
            Commands.sequence(
                Commands.runOnce(coral::stopIntake, coral),
                Commands.runOnce(() -> coral.setPivotAngle(CorAlConstants.PivotPresetAngles.BASE.getAngle()), coral),
                Commands.waitUntil(coral::isAtTargetAngle)
            )
        );

        // Coral scoring
        operator_controller.x().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> coral.setPivotAngle(CorAlConstants.PivotPresetAngles.BASE.getAngle()), coral),
                Commands.waitUntil(coral::isAtTargetAngle),
                Commands.runOnce(() -> coral.startRollersWithTimer(CorAlConstants.CORAL_INTAKE_SPEED, CorAlConstants.ROLLER_RUN_TIME), coral)
            )
        );

        // Algae intaking
        operator_controller.b().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> coral.setPivotAngle(CorAlConstants.PivotPresetAngles.ALGAE_INTAKE.getAngle()), coral),
                Commands.waitUntil(coral::isAtTargetAngle),
                Commands.runOnce(() -> coral.startRollersWithTimer(CorAlConstants.ALGAE_INTAKE_SPEED, CorAlConstants.ROLLER_RUN_TIME), coral)
            )
        );

        // Algae scoring
        operator_controller.y().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> coral.setPivotAngle(CorAlConstants.PivotPresetAngles.ALGAE_SCORE.getAngle()), coral),
                Commands.waitUntil(coral::isAtTargetAngle),
                Commands.runOnce(() -> coral.startRollersWithTimer(CorAlConstants.CORAL_INTAKE_SPEED, CorAlConstants.ROLLER_RUN_TIME), coral)
            )
        );

        // Pivot encoder reset
        operator_controller.rightBumper().onTrue(Commands.runOnce(() -> coral.resetPivotEncoder(), coral).ignoringDisable(true));

        // Pivot manual control
        coral.setDefaultCommand(Commands.run(() -> {
            double stickInput = operator_controller.getRightX();
            // Only apply manual control if outside deadband
            if (Math.abs(stickInput) >= CorAlConstants.CORAL_MANUAL_CONTROL_DEADBAND) {
                double speed = stickInput * CorAlConstants.CORAL_MANUAL_SPEED_LIMIT;
                coral.manualPivotControl(speed);
            } else {
                coral.stopPivot();
            }
        }, coral));
    }

	/*
    private void configureAlLowBindings() {
        // AlLow extension (with rollers activated)
        operator_controller.rightTrigger().whileTrue(Commands.run(() -> {
            allow.setPivotAngle(AlLowConstants.PivotPresetAngles.INTAKE.getAngle()); // Extend to intaking angle
            allow.setRollerSpeed(-1.0); // Spin rollers inward
        }, allow));

        // AlLow retraction (with rollers stopped)
        operator_controller.leftTrigger().onTrue(Commands.runOnce(() -> {
            allow.setPivotAngle(AlLowConstants.PivotPresetAngles.BASE.getAngle()); // Retract to base angle
            allow.stopRoller(); // Stop rollers
        }, allow));

        // AlLow ejection
        operator_controller.back().whileTrue(Commands.run(() -> allow.setRollerSpeed(1.0), allow));

        // AlLow pivot encoder reset
        operator_controller.start().onTrue(Commands.runOnce(() -> allow.resetPivotEncoder(), allow).ignoringDisable(true));

        // AlLow pivot manual control
        allow.setDefaultCommand(Commands.run(() -> {
            double speed = -operator_controller.getRightX(); // Use right stick X for pivot control
            allow.manualPivotControl(speed);
        }, allow));
    }
	*/

    public Command getAutonomousCommand() {
        // Run the path selected from the auto chooser
        return autoChooser.getSelected();
    }

    public void disabledInit() {
        // Stop all subsystems when disabled
        elevator.stopElevator();
        coral.stopPivot();
        coral.stopIntake();
        //allow.stopPivot();
        //allow.stopRoller();
    }
}