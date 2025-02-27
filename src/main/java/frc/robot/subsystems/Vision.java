package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private Swerve swerve;

    private boolean trackingEnabled = false;
    private boolean positionTrackingEnabled = true;

    /*
    // Shuffleboard
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    private final GenericEntry trackingEnabledWidget;
    private final GenericEntry bestTagIdWidget;
    private final GenericEntry bestTagDistanceWidget;
    private final GenericEntry bestTagHorizontalOffsetWidget;
    private final GenericEntry targetDetectedWidget;
    private final GenericEntry targetYawWidget;
    private final GenericEntry targetPitchWidget;
    private final GenericEntry targetPoseXWidget;
    private final GenericEntry targetPoseYWidget;
    private final GenericEntry targetErrorXWidget;
    private final GenericEntry targetErrorYWidget;
    private final GenericEntry targetErrorAngleWidget;
    private final GenericEntry limelightNameWidget;
    */

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        /*

        // Status layout - general vision status information
        trackingEnabledWidget = visionTab.add("Tracking Enabled", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0, 0)
            .getEntry();

        targetDetectedWidget = visionTab.add("Target Detected", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 2)
            .withPosition(0, 2)
            .getEntry();

        // Target information layout - main data about the detected target
        bestTagIdWidget = visionTab.add("Best Tag ID", -1)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(3, 0)
            .getEntry();

        bestTagDistanceWidget = visionTab.add("Best Tag Distance", 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withSize(3, 2)
            .withPosition(3, 2)
            .getEntry();

        // Target position and orientation layout - detailed positioning data
        targetYawWidget = visionTab.add("Target Yaw (tx)", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(6, 0)
            .getEntry();

        targetPitchWidget = visionTab.add("Target Pitch (ty)", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(6, 2)
            .getEntry();

        targetPoseXWidget = visionTab.add("Target Pose X", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(9, 0)
            .getEntry();

        targetPoseYWidget = visionTab.add("Target Pose Y", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(9, 2)
            .getEntry();

        // Offsets and errors layout - tracking performance metrics
        bestTagHorizontalOffsetWidget = visionTab.add("Target Horizontal Offset", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(12, 0)
            .getEntry();

        // Error tracking for PID feedback visualization
        targetErrorXWidget = visionTab.add("Distance Error", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 2)
            .withPosition(0, 4)
            .getEntry();

        targetErrorYWidget = visionTab.add("Horizontal Error", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 2)
            .withPosition(5, 4)
            .getEntry();

        targetErrorAngleWidget = visionTab.add("Angle Error", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(5, 2)
            .withPosition(10, 4)
            .getEntry();

        // Source information
        limelightNameWidget = visionTab.add("Limelight Source", "none")
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 2)
            .withPosition(12, 2)
            .getEntry();
        */

        // Set all Limelights to AprilTag pipeline (Pipeline 0)
        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            LimelightHelpers.setPipelineIndex("limelight-" + name, 0);
        }
    }

    // Toggles AprilTag tracking and controls Limelight LEDs.
    public void toggleTracking(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;
            for (String name : VisionConstants.LIMELIGHT_NAMES) {
                if (trackingEnabled) {
                    LimelightHelpers.setLEDMode_ForceOn("limelight-" + name);
                } else {
                    LimelightHelpers.setLEDMode_ForceOff("limelight-" + name);
                }
            }
        }
    }

    // Returns whether AprilTag tracking is enabled.
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    // Retrieves the horizontal offset for a specific AprilTag.
    public double getTargetHorizontalOffset(int tagId) {
        if (tagId >= 0 && tagId < VisionConstants.APRILTAG_OFFSETS.length) {
            return VisionConstants.APRILTAG_OFFSETS[tagId][0];
        }
        // Return default value and log warning
        System.out.println("Warning: Tag ID " + tagId + " out of bounds");
        return 0.0;
    }

    // Retrieves the vertical offset for a specific AprilTag.
    public double getTargetVerticalOffset(int tagId) {
        if (tagId >= 0 && tagId < VisionConstants.APRILTAG_OFFSETS.length) {
            return VisionConstants.APRILTAG_OFFSETS[tagId][1];
        }
        // Return default value and log warning
        System.out.println("Warning: Tag ID " + tagId + " out of bounds");
        return 0.0;
    }

    // Retrieves the best AprilTag target from all Limelights.
    public Optional<AprilTagTarget> getBestTarget() {
        AprilTagTarget bestTarget = null;
        double bestArea = 0;

        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            String limelightName = "limelight-" + name;
            if (LimelightHelpers.getTV(limelightName)) {
                double currentArea = LimelightHelpers.getTA(limelightName);
                if (currentArea > bestArea) {
                    bestTarget = new AprilTagTarget(
                        (int) LimelightHelpers.getFiducialID(limelightName),
                        LimelightHelpers.getTX(limelightName),
                        LimelightHelpers.getTY(limelightName),
                        LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getX(),
                        LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getY(),
                        name
                    );
                    bestArea = currentArea;
                }
            }
        }

        return Optional.ofNullable(bestTarget);
    }

    // Retrieves the latest pose estimate for vision-assisted localization.
    public Optional<Pose2d> getLatestPoseEstimate() {
        Optional<AprilTagTarget> target = getBestTarget();
        if (target.isPresent()) {
            return Optional.of(LimelightHelpers.getBotPose2d_wpiBlue("limelight-" + target.get().limelightName));
        }
        return Optional.empty();
    }

    // Stops vision-based movement cleanly.
    public void stopVisionMovement() {
        trackingEnabled = false;
        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            LimelightHelpers.setLEDMode_ForceOff("limelight-" + name);
        }
    }

    public void enablePositionTracking(boolean enabled) {
        positionTrackingEnabled = enabled;
    }

    public boolean isPositionTrackingEnabled() {
        return positionTrackingEnabled;
    }

    // Represents an AprilTag target detected by a Limelight.
    public static class AprilTagTarget {
        public final int id;
        public final double tx;
        public final double ty;
        public final double poseX;
        public final double poseY;
        public final String limelightName;

        public AprilTagTarget(int id, double tx, double ty, double poseX, double poseY, String limelightName) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.poseX = poseX;
            this.poseY = poseY;
            this.limelightName = limelightName;
        }
    }

	private void updateRobotPosition() {
        // Find the Limelight with the most visible tags for best position estimate
        String bestLimelight = null;
        int maxTagCount = 0;

        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            String limelightName = "limelight-" + name;
            if (LimelightHelpers.getTV(limelightName)) {
                int tagCount = LimelightHelpers.getTargetCount(limelightName);
                if (tagCount > maxTagCount) {
                    maxTagCount = tagCount;
                    bestLimelight = limelightName;
                }
            }
        }

        // Only update position if we have at least one tag visible
        if (bestLimelight != null && maxTagCount > 0) {
            // Get robot pose estimate from Limelight on the field
            Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue(bestLimelight);

            // Get timestamp of when this measurement was taken
            double timestamp = LimelightHelpers.getLatency_Pipeline(bestLimelight) / 1000.0 +
                              LimelightHelpers.getLatency_Capture(bestLimelight) / 1000.0;
            double timestampSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - timestamp;

            // Calculate standard deviation based on distance and number of tags
            // This is a simple model - you may want to tune this for your robot
            if (botPose != null) {
                // Calculate standard deviation - more tags and closer distance means more confidence
                double stdDevPoseFactor = 1.0 / Math.sqrt(maxTagCount);
                // Closer tags (larger area) mean better confidence
                double tagArea = LimelightHelpers.getTA(bestLimelight);

                // Basic standard deviation model - this can be tuned
                double xyStdDev = 0.1 + stdDevPoseFactor * (1.0 / Math.max(0.1, tagArea));
                double rotStdDev = 0.05 + stdDevPoseFactor * (1.0 / Math.max(0.1, tagArea));

                // Create matrix for standard deviation
                edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> visionStdDevs =
                    edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);

                // Add the vision measurement to the Swerve drivetrain
                swerve.addVisionMeasurement(botPose, timestampSeconds, visionStdDevs);
            }
        }
    }

    @Override
    public void periodic() {
        // Update Shuffleboard with latest vision data
        Optional<AprilTagTarget> target = getBestTarget();
        boolean hasTarget = target.isPresent();

        // Update status widgets
        // trackingEnabledWidget.setBoolean(trackingEnabled);
        // targetDetectedWidget.setBoolean(hasTarget);

        if (hasTarget) {
            // Get the current target
            AprilTagTarget currentTarget = target.get();

            // Calculate distance and errors
            double distance = Math.hypot(currentTarget.poseX, currentTarget.poseY);
            double targetHorizontalOffset = getTargetHorizontalOffset(currentTarget.id);
            double targetVerticalOffset = getTargetVerticalOffset(currentTarget.id);
            double horizontalError = currentTarget.poseY - targetHorizontalOffset;
            double distanceError = distance - targetVerticalOffset;
            double angleError = currentTarget.tx;

            /*
            // Update target information widgets
            bestTagIdWidget.setDouble(currentTarget.id);
            bestTagDistanceWidget.setDouble(distance);
            bestTagHorizontalOffsetWidget.setDouble(targetHorizontalOffset);

            // Update position and orientation widgets
            targetYawWidget.setDouble(currentTarget.tx);
            targetPitchWidget.setDouble(currentTarget.ty);
            targetPoseXWidget.setDouble(currentTarget.poseX);
            targetPoseYWidget.setDouble(currentTarget.poseY);

            // Update error metrics
            targetErrorXWidget.setDouble(distanceError);
            targetErrorYWidget.setDouble(horizontalError);
            targetErrorAngleWidget.setDouble(angleError);

            // Update source information
            limelightNameWidget.setString(currentTarget.limelightName);
            */
        } else {
            /*
            // Reset values when no target is detected
            bestTagIdWidget.setDouble(-1);
            bestTagDistanceWidget.setDouble(0.0);
            bestTagHorizontalOffsetWidget.setDouble(0.0);
            targetYawWidget.setDouble(0.0);
            targetPitchWidget.setDouble(0.0);
            targetPoseXWidget.setDouble(0.0);
            targetPoseYWidget.setDouble(0.0);
            targetErrorXWidget.setDouble(0.0);
            targetErrorYWidget.setDouble(0.0);
            targetErrorAngleWidget.setDouble(0.0);
            limelightNameWidget.setString("none");
            */
        }

        if (positionTrackingEnabled) {
            updateRobotPosition();
        }
    }
}